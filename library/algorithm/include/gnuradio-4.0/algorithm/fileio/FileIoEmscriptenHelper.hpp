#ifndef FILEIOEMSCRIPTENHELPER_HPP
#define FILEIOEMSCRIPTENHELPER_HPP

#if defined(__EMSCRIPTEN__)
#include <emscripten.h>
#include <emscripten/emscripten.h>
#include <emscripten/fetch.h>
#include <emscripten/html5.h>
#include <emscripten/threading.h>
#endif

#if defined(__EMSCRIPTEN__)
#define FILEIO_EXPORT EMSCRIPTEN_KEEPALIVE
#else
#define FILEIO_EXPORT
#endif

#ifdef __cplusplus
#define FILEIO_EXTERN_C extern "C"
#else
#define FILEIO_EXTERN_C
#endif

namespace gr::algorithm::fileio {
constexpr bool isWebAssembly() noexcept {
#if defined(__EMSCRIPTEN__)
    return true;
#else
    return false;
#endif
}

inline bool isMainThread() {
#if defined(__EMSCRIPTEN__)
    return emscripten_is_main_runtime_thread();
#else
    return true; // Native: assume single-threaded or main thread
#endif
}

inline bool isTabVisible() {
#if defined(__EMSCRIPTEN__)
    EmscriptenVisibilityChangeEvent status;
    if (emscripten_get_visibility_status(&status) == EMSCRIPTEN_RESULT_SUCCESS) {
        return !status.hidden;
    }
#endif
    return true;
}

// clang-format off
inline void listPersistentFiles([[maybe_unused]] bool recursive = true) {
#if defined(__EMSCRIPTEN__)
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wdollar-in-identifier-extension"
    EM_ASM_({
        function listDir(path, recursive, indent = "") {
            try {
                const entries = FS.readdir(path);
                for (let entry of entries) {
                    if (entry === '.' || entry === '..') continue;
                    const fullPath = path + (path.endsWith('/') ? "" : "/") + entry;
                    const stat = FS.stat(fullPath);
                    if (FS.isDir(stat.mode)) {
                        console.log(indent + '[Dir] ' + fullPath);
                        if (recursive) {
                            listDir(fullPath, recursive, indent + '  ');
                        }
                    } else {
                        console.log(indent + '[File] ' + fullPath);
                    }
                }
            } catch (e) {
                console.error('Error listing directory:', path, e);
            }
        }
        listDir('/', $0 !== 0);
    }, recursive ? 1 : 0);
#pragma GCC diagnostic pop
#else

#endif
}
// clang-format on

} // namespace gr::algorithm::fileio

#endif // FILEIOEMSCRIPTENHELPER_HPP
