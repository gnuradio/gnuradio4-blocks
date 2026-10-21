#!/usr/bin/env python3
"""Run the restylers that .restyled.yaml lists, without Restyled's container images.

Restyled runs each formatter from an image on public.ecr.aws or Docker Hub, and the
pull of those images fails when the registries throttle the shared runners. This
script runs the same formatters at the versions Restyled's stable channel named on
2026-10-21 (PIP_PINS and PRETTIER below), with Restyled's command lines, file
selection and order, then reports any file they change.

    restyle.py install [--tools DIR]
    restyle.py run [--tools DIR] [--event FILE] [--apply]
                   (--base REF | --paths-from FILE | PATH...)

`install` puts the pinned tools under DIR (a Python virtual environment and an npm
prefix). `run`, from the repository's top directory, formats the selected files in a
temporary copy of the working tree, prints the differences and exits 1 when a file
changed, 0 when none did and 2 on an error. The working tree is left untouched;
`--apply` writes the formatted files back into it. Locally, after `install`:

    python3 .github/ci-tools/restyle.py run --base upstream/main
    python3 .github/ci-tools/restyle.py run --base upstream/main --apply

The file set is the pull request's changed files, as Restyled takes it; `--base`
gives the same set from git (`git diff --name-only REF...HEAD`).

Supported in .restyled.yaml: `enabled`, `exclude`, `also_exclude`, the `ignore`
settings for authors, branches and labels, `restylers_version: stable`, and
`restylers` entries that name a restyler below, optionally with an image tag that
matches the pinned version. Every other setting that could change the result is
refused, since this script would not apply it; the keys that drive Restyled's pull
request interactions (comments, statuses, labels and so on) are ignored.

The script differs from Restyled in three cases, each where Restyled fails or
passes for a reason unrelated to formatting:

- Restyled runs each formatter in a container capped at 128 MB. This script sets no
  cap: a file too large for that cap is formatted here and fails Restyled's run.
- Restyled misses a difference in a file whose name holds a non-ASCII character,
  since git quotes such a name in the list Restyled compares. This script reports it.
- Restyled fails a pull request that changes no file. This script passes it.
"""

import argparse
import difflib
import json
import os
import re
import shutil
import subprocess
import sys
import tempfile
import unicodedata

# The images of Restyled's stable manifest
# (https://docs.restyled.io/data-files/restylers/manifests/stable/restylers.yaml)
# on 2026-10-21; each pin is the version the image reports, its dependencies included.
PIP_PINS = [
    "clang-format==18.1.8",  # restyled/restyler-clang-format:v18
    "cmakelang==0.6.13",  # restyler-cmake-format:0.6.13-1
    "six==1.16.0",
    # cmake-format reads .cmake-format.yaml through PyYAML; the image's 6.0.1 has no
    # build for Python 3.14, and the version does not change cmake-format's output
    "pyyaml==6.0.3",
    "shellcheck-py==0.11.0.1",  # restyler-shellcheck:v0.11.0
    "black==26.5.1",  # restyler-black:v26.5.1, on Python 3.14
    "click==8.4.0",
    "mypy_extensions==1.1.0",
    "packaging==26.2",
    "pathspec==1.1.1",
    "platformdirs==4.9.6",
    "pytokens==0.4.1",
]
PRETTIER = "prettier@3.9.9"  # restyler-prettier:v3.9.9-5

VERSION_CHECKS = {
    "clang-format": (["clang-format", "--version"], "version 18.1.8"),
    "cmake-format": (["cmake-format", "--version"], "0.6.13"),
    "shellcheck": (["shellcheck", "--version"], "version: 0.11.0"),
    "black": (["black", "--version"], " 26.5.1"),
    "prettier": (["prettier", "--version"], "3.9.9"),
    "patch": (["patch", "--version"], "GNU patch"),
}

# The restylers' definitions in the manifest: command, include globs, interpreters
# and whether the command takes `--` before the paths.
SHELLCHECK_FIX = 'shellcheck --format=diff "$@" | patch -p 1'
RESTYLERS = {
    "clang-format": {
        "command": ["clang-format", "-i"],
        "separator": False,
        "include": [
            "**/*.c",
            "**/*.cc",
            "**/*.cpp",
            "**/*.cxx",
            "**/*.c++",
            "**/*.C",
            "**/*.cs",
            "**/*.h",
            "**/*.hh",
            "**/*.hpp",
            "**/*.hxx",
            "**/*.h++",
            "**/*.H",
            "**/*.java",
            "**/*.js",
            "**/*.m",
        ],
        "version": "18.1.8",
        "tool": "clang-format",
    },
    "cmake-format": {
        "command": ["cmake-format", "--in-place"],
        "separator": True,
        "include": ["**/CMakeLists.txt", "**/*.cmake-format-test"],
        "version": "0.6.13",
        "tool": "cmake-format",
    },
    "prettier-markdown": {
        "command": ["prettier", "--write"],
        "separator": True,
        "include": ["**/*.md", "**/*.markdown"],
        "version": "3.9.9",
        "tool": "prettier",
        "outside": True,
    },
    "prettier-yaml": {
        "command": ["prettier", "--write"],
        "separator": True,
        "include": ["**/*.yml", "**/*.yaml"],
        "version": "3.9.9",
        "tool": "prettier",
        "outside": True,
    },
    "shellcheck": {
        "command": ["sh", "-c", SHELLCHECK_FIX, "shellcheck-fix"],
        "separator": True,
        "include": ["**/*.sh", "**/*.bash"],
        "interpreters": ["sh", "bash"],
        "version": "0.11.0",
        "tool": "shellcheck",
    },
    "whitespace": {
        "command": None,
        "include": [
            "**/*",
            "!**/*.gif",
            "!**/*.ico",
            "!**/*.jpeg",
            "!**/*.jpg",
            "!**/*.pdf",
            "!**/*.png",
            "!**/fonts/**/*",
        ],
        "version": "0.2.0.5",
        "tool": None,
    },
    "black": {
        "command": ["black"],
        "separator": True,
        "include": ["**/*.py"],
        "interpreters": ["python"],
        "version": "26.5.1",
        "tool": "black",
    },
}

DEFAULT_EXCLUDES = [
    "**/*.patch",
    "**/.git/**/*",
    "**/Gemfile.lock",
    "**/node_modules/**/*",
    "**/package-lock.json",
    "**/pnpm-lock.yaml",
    "**/stack*.yaml.lock",
    "**/vendor/**/*",
    "**/yarn.lock",
]

IGNORED_KEYS = {
    "auto",
    "comments",
    "commit_template",
    "labels",
    "logging",
    "pull_requests",
    "request_review",
    "statuses",
}
# The pull requests Restyled skips unless the configuration says otherwise.
DEFAULT_IGNORE = {
    "authors": ["*[bot]"],
    "branches": ["renovate/*"],
    "labels": ["restyled-ignore"],
}


class Failure(Exception):
    pass


def is_space(c):
    """Haskell's Data.Char.isSpace, which Restyled and its whitespace tool use."""
    o = ord(c)
    if o <= 0x377:
        return c == " " or 0x09 <= o <= 0x0D or o == 0xA0
    return unicodedata.category(c) == "Zs"


def strip_end(s, pred=is_space):
    end = len(s)
    while end and pred(s[end - 1]):
        end -= 1
    return s[:end]


def glob_regex(pattern, text=False):
    """System.FilePath.Glob as Restyled compiles it, dot files matched implicitly.

    Over authors, branches and labels (`text`) brackets are literal characters.
    """
    out, i = [], 0
    while i < len(pattern):
        if pattern.startswith("**/", i):
            out.append("(?:[^/]*/)*")
            i += 3
        elif pattern[i] == "*":
            out.append("[^/]*")
            i += 1
        elif pattern[i] == "?":
            out.append("[^/]")
            i += 1
        elif pattern[i] in "[<" and not text:
            raise Failure(
                f"glob {pattern!r}: character classes and ranges are not supported"
            )
        else:
            out.append(re.escape(pattern[i]))
            i += 1
    return re.compile("".join(out) + r"\Z")


def glob_match(pattern, path, text=False):
    return glob_regex(pattern, text).match(path) is not None


def included(includes, path):
    result = False
    for pattern in includes:
        if pattern.startswith("!"):
            result = result and not glob_match(pattern[1:], path)
        else:
            result = result or glob_match(pattern, path)
    return result


INTERPRETERS = {
    "sh": "sh",
    "bash": "bash",
    "python": "python",
    "python2": "python",
    "python2.7": "python",
    "python3": "python",
    "python3.6": "python",
}


def interpreter(path):
    """The shebang interpreter of an executable file, as Restyled reads it.

    The whole first line must parse as `#!/usr/bin/env NAME` or `#!PATH` with no
    further arguments. `#!/bin/bash -e` names no interpreter.
    """
    if not os.access(path, os.X_OK):
        return None
    try:
        with open(path, "rb") as f:
            text = f.read().decode("utf-8", errors="replace")
    except OSError:
        return None
    if not text:
        return None
    line = text.split("\n", 1)[0]
    line = strip_end(line)
    start = 0
    while start < len(line) and is_space(line[start]):
        start += 1
    line = line[start:]
    if not line.startswith("#!"):
        return None
    rest = line[2:]

    def word(s):
        return s if s and not any(is_space(c) for c in s) else None

    name = None
    if rest.startswith("/usr/bin/env "):
        name = word(rest[len("/usr/bin/env ") :])
    if name is None and word(rest) is not None:
        name = rest.rsplit("/", 1)[-1]
    if name is None:
        return None
    return INTERPRETERS.get(name, name)


def hs_lines(text):
    if not text:
        return []
    parts = text.split("\n")
    if text.endswith("\n"):
        parts.pop()
    return parts


def whitespace_format(text):
    """restyler-whitespace 0.2.0.5: trailing spaces removed, one final newline."""
    if not text:
        return text
    text = "".join(strip_end(line) + "\n" for line in hs_lines(text))
    return text.rstrip("\n") + "\n"


def run_whitespace(paths, root):
    for p in paths:
        path = os.path.join(root, p)
        with open(path, "rb") as f:
            data = f.read()
        try:
            text = data.decode("utf-8")
        except UnicodeDecodeError:
            print(f"whitespace: {p} is not UTF-8, left as is", file=sys.stderr)
            continue
        if "\r\n" in text:
            print(f"whitespace: {p} has CRLF line ends, left as is", file=sys.stderr)
            continue
        formatted = whitespace_format(text)
        if formatted != text:
            with open(path, "wb") as f:
                f.write(formatted.encode("utf-8"))


def run_restyler(name, spec, paths, root):
    if spec["command"] is None:
        run_whitespace(paths, root)
        return
    args = list(spec["command"]) + (["--"] if spec["separator"] else [])
    if spec.get("outside"):
        # Restyled's prettier runs from its own directory with absolute paths. The
        # repository's .gitignore and .prettierignore then do not apply.
        with tempfile.TemporaryDirectory() as cwd:
            argv = args + [os.path.join(root, p) for p in paths]
            rc = subprocess.run(argv, cwd=cwd).returncode
    else:
        rc = subprocess.run(args + ["./" + p for p in paths], cwd=root).returncode
    if rc != 0:
        raise Failure(f"{name} exited with {rc}")


def load_config(path):
    import yaml

    if not os.path.exists(path):
        raise Failure(f"{path} not found")
    try:
        with open(path) as f:
            config = yaml.safe_load(f) or {}
    except yaml.YAMLError as e:
        raise Failure(f"{path}: {e}")
    known = {
        "enabled",
        "exclude",
        "also_exclude",
        "ignore",
        "ignore_authors",
        "ignore_branches",
        "ignore_labels",
        "restylers_version",
        "restylers",
    }
    unknown = set(config) - known - IGNORED_KEYS
    if unknown:
        raise Failure(f"{path}: unsupported keys {sorted(unknown)}")
    if config.get("restylers_version", "stable") != "stable":
        raise Failure(f"{path}: only restylers_version: stable is supported")

    restylers = []
    for entry in config.get("restylers", ["*"]):
        if isinstance(entry, str) and entry.startswith("!"):
            name, options = entry[1:], {"enabled": False}
        elif isinstance(entry, str):
            name, options = entry, {}
        elif isinstance(entry, dict) and "name" in entry:
            name, options = entry["name"], {
                k: v for k, v in entry.items() if k != "name"
            }
        elif isinstance(entry, dict) and len(entry) == 1:
            name, options = next(iter(entry.items()))
            options = options or {}
        else:
            raise Failure(f"{path}: restyler entry {entry!r} is not supported")
        if name not in RESTYLERS:
            raise Failure(f"{path}: restyler {name!r} is not supported")
        for key, value in options.items():
            if key == "enabled":
                continue
            if key != "image" or not isinstance(value, dict) or set(value) != {"tag"}:
                raise Failure(f"{path}: option {key!r} of {name} is not supported")
            pinned = RESTYLERS[name]["version"]
            if not pinned.startswith(str(value["tag"]).lstrip("v")):
                raise Failure(
                    f"{path}: {name} tag {value['tag']} does not match {pinned}"
                )
        if options.get("enabled", True):
            restylers.append(name)

    ignore = dict(DEFAULT_IGNORE)
    nested = config.get("ignore") or {}
    for kind in ignore:
        if kind in nested:
            ignore[kind] = nested[kind]
        if f"ignore_{kind}" in config:
            ignore[kind] = config[f"ignore_{kind}"]
    excludes = config.get("exclude", DEFAULT_EXCLUDES) + config.get("also_exclude", [])
    return {
        "enabled": config.get("enabled", True),
        "excludes": excludes,
        "ignore": ignore,
        "restylers": restylers,
    }


def ignored(config, event_path):
    """The reason Restyled would skip the pull request in the event file, or None.

    Restyled matches `ignore.branches` against the branch the pull request targets.
    """
    try:
        with open(event_path) as f:
            pr = json.load(f)["pull_request"]
    except (ValueError, KeyError) as e:
        raise Failure(f"{event_path}: no pull request in the event ({e})")
    found = {
        "authors": [pr["user"]["login"]],
        "branches": [pr["base"]["ref"]],
        "labels": [label["name"] for label in pr.get("labels", [])],
    }
    for kind, globs in config["ignore"].items():
        for value in found[kind]:
            if any(glob_match(g, value, text=True) for g in globs):
                return f"{value!r} matches ignore.{kind}"
    return None


def tool_paths(tools):
    return [
        os.path.join(tools, "venv", "bin"),
        os.path.join(tools, "node", "node_modules", ".bin"),
    ]


def check_versions(names):
    for name in names:
        argv, expected = VERSION_CHECKS[name]
        if shutil.which(argv[0]) is None:
            raise Failure(f"{argv[0]} not found; run `restyle.py install` first")
        out = subprocess.run(argv, capture_output=True, text=True).stdout
        if expected not in out:
            raise Failure(
                f"{argv[0]} is not the pinned version ({expected!r} expected): {out.strip()}"
            )


def copy_tree(paths, root):
    """Copy the working tree's tracked and unignored files, and `paths`, under root.

    The formatters find their configuration (.clang-format, .cmake-format.yaml and
    the like) in the copy. An empty .git directory marks the copy's top for black.
    """
    listed = subprocess.run(
        ["git", "ls-files", "-z", "--cached", "--others", "--exclude-standard"],
        capture_output=True,
        check=True,
    ).stdout.decode("utf-8", errors="surrogateescape")
    for p in set(filter(None, listed.split("\0"))) | set(paths):
        if not os.path.lexists(p) or os.path.isdir(p):
            continue
        dst = os.path.join(root, p)
        os.makedirs(os.path.dirname(dst), exist_ok=True)
        shutil.copy2(p, dst, follow_symlinks=False)
    os.makedirs(os.path.join(root, ".git"), exist_ok=True)


def changed_paths(args):
    if args.base:
        out = subprocess.run(
            ["git", "diff", "--name-only", f"{args.base}...HEAD"],
            capture_output=True,
            text=True,
            check=True,
        ).stdout
        return out.splitlines()
    if args.paths_from:
        with open(args.paths_from) as f:
            return [line for line in f.read().splitlines() if line]
    return args.paths


def command_run(args):
    venv_python = os.path.join(args.tools, "venv", "bin", "python")
    if os.path.realpath(sys.prefix) != os.path.realpath(
        os.path.join(args.tools, "venv")
    ):
        if not os.path.exists(venv_python):
            raise Failure(
                f"no tools under {args.tools}; run `restyle.py install` first"
            )
        os.execv(venv_python, [venv_python] + sys.argv)
    os.environ["PATH"] = os.pathsep.join(tool_paths(args.tools) + [os.environ["PATH"]])

    top = subprocess.run(
        ["git", "rev-parse", "--show-toplevel"], capture_output=True, text=True
    ).stdout.strip()
    if not top or not os.path.samefile(top, "."):
        raise Failure("run restyle.py from the repository's top directory")

    config = load_config(args.config)
    if not config["enabled"]:
        print("restyling is disabled in the configuration")
        return 0
    reason = ignored(config, args.event) if args.event else None
    if reason:
        print(f"skipped: {reason}")
        return 0

    # Restyled drops deleted paths and every symbolic link.
    paths = [
        p for p in changed_paths(args) if os.path.isfile(p) and not os.path.islink(p)
    ]
    paths = [p for p in paths if not any(glob_match(g, p) for g in config["excludes"])]
    shebangs = {p: interpreter(p) for p in paths}

    selected = {}
    for name in config["restylers"]:
        spec = RESTYLERS[name]
        chosen = []
        for p in paths:
            if shebangs[p] in spec.get("interpreters", []) or included(
                spec["include"], p
            ):
                chosen.append(p)
        if chosen:
            selected[name] = chosen

    tools = {RESTYLERS[n]["tool"] for n in selected} - {None}
    if "shellcheck" in tools:
        tools.add("patch")
    check_versions(sorted(tools))

    with tempfile.TemporaryDirectory() as root:
        copy_tree(paths, root)
        for name, chosen in selected.items():
            print(f"{name}: {len(chosen)} file(s)")
            run_restyler(name, RESTYLERS[name], chosen, root)
        after = {}
        for p in paths:
            with open(os.path.join(root, p), "rb") as f:
                after[p] = f.read()

    changed = []
    for p in paths:
        with open(p, "rb") as f:
            before = f.read()
        if after[p] != before:
            changed.append(p)
            diff = difflib.unified_diff(
                before.decode("utf-8", errors="replace").splitlines(True),
                after[p].decode("utf-8", errors="replace").splitlines(True),
                f"a/{p}",
                f"b/{p}",
            )
            for line in diff:
                sys.stdout.write(line if line.endswith("\n") else line + "\n")

    if changed:
        if args.apply:
            for p in changed:
                with open(p, "wb") as f:
                    f.write(after[p])
        print(
            f"\n{len(changed)} file(s) differ from the restylers' output"
            + (", now written back:" if args.apply else " (--apply writes them):"),
            file=sys.stderr,
        )
        for p in changed:
            print(f"  {p}", file=sys.stderr)
        return 1
    print(f"no differences in {len(paths)} file(s)")
    return 0


def command_install(args):
    venv = os.path.join(args.tools, "venv")
    subprocess.run([sys.executable, "-m", "venv", venv], check=True)
    subprocess.run(
        [os.path.join(venv, "bin", "pip"), "install", "--quiet", *PIP_PINS], check=True
    )
    node = os.path.join(args.tools, "node")
    os.makedirs(node, exist_ok=True)
    subprocess.run(
        ["npm", "install", "--silent", "--no-save", "--prefix", node, PRETTIER],
        check=True,
    )
    return 0


def main():
    tools = argparse.ArgumentParser(add_help=False)
    tools.add_argument(
        "--tools",
        default=os.path.join(
            os.environ.get("XDG_CACHE_HOME", os.path.expanduser("~/.cache")),
            "restyle-tools",
        ),
        help="directory holding the pinned tools",
    )
    parser = argparse.ArgumentParser(description=__doc__.split("\n", 1)[0])
    sub = parser.add_subparsers(dest="command", required=True)
    sub.add_parser("install", parents=[tools], help="install the pinned tools")
    run = sub.add_parser("run", parents=[tools], help="restyle the changed files")
    run.add_argument("--config", default=".restyled.yaml")
    run.add_argument(
        "--event",
        help="the pull_request event file; its author, branch and labels may skip the run",
    )
    run.add_argument("--base", help="restyle the files changed since REF")
    run.add_argument("--paths-from", help="restyle the files listed in FILE")
    run.add_argument(
        "--apply", action="store_true", help="write the formatted files back"
    )
    run.add_argument("paths", nargs="*")
    args = parser.parse_args()
    if (
        args.command == "run"
        and sum(map(bool, [args.base, args.paths_from, args.paths])) != 1
    ):
        parser.error("run takes exactly one of --base, --paths-from or paths")
    try:
        return command_install(args) if args.command == "install" else command_run(args)
    except (Failure, subprocess.CalledProcessError, OSError) as e:
        print(f"restyle: {e}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    sys.exit(main())
