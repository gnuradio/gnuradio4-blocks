#ifndef FILEIOHELPERS_HPP
#define FILEIOHELPERS_HPP

#include <expected>
#include <gnuradio-4.0/Message.hpp>

namespace gr::algorithm::fileio {

struct Capabilities {
    bool httpEnabled = false; // GR_HTTP_ENABLED or fetch()
};

inline Capabilities capabilities() noexcept {
    Capabilities c{};

#if __EMSCRIPTEN__
    c.httpEnabled = true; // We always compile with emscripten fetch here
#else

#if GR_HTTP_ENABLED
    c.httpEnabled = true;
#else
    c.httpEnabled = false;
#endif
#endif
    return c;
}

namespace detail {
[[nodiscard]] inline bool ciEquals(std::string_view a, std::string_view b) {
    if (a.size() != b.size()) {
        return false;
    }
    return std::ranges::equal(a, b, [](char ac, char bc) { return std::tolower(static_cast<unsigned char>(ac)) == std::tolower(static_cast<unsigned char>(bc)); });
}

[[nodiscard]] inline bool startsWithScheme(std::string_view uri) {
    const auto p = uri.find(":/");
    if (p == std::string_view::npos || p == 0UZ) {
        return false;
    }
    for (char c : uri.substr(0, p)) {
        if (!std::isalpha(static_cast<unsigned char>(c))) {
            return false;
        }
    }
    return true;
}

[[nodiscard]] inline std::string_view schemeOf(std::string_view uri) {
    if (!startsWithScheme(uri)) {
        return {};
    }
    return uri.substr(0, uri.find(":/"));
}

[[nodiscard]] inline bool isHttpUri(std::string_view uri) {
    const auto sc = schemeOf(uri);
    return ciEquals(sc, "http") || ciEquals(sc, "https");
}

[[nodiscard]] inline bool isFileUri(std::string_view uri) { return ciEquals(schemeOf(uri), "file"); }

enum class UriKind {
    LocalPath,
    FileUri,
    HttpUri,
    UnsupportedUri,
};

[[nodiscard]] inline UriKind classifyUri(std::string_view uri) {
    if (startsWithScheme(uri)) {
        if (isFileUri(uri)) {
            return UriKind::FileUri;
        }
        if (isHttpUri(uri)) {
            return UriKind::HttpUri;
        }
        return UriKind::UnsupportedUri;
    }
    if (uri.contains(":/")) {
        return UriKind::UnsupportedUri;
    }
    return UriKind::LocalPath;
}

[[nodiscard]] inline std::expected<std::string, gr::Error> toLocalPath(std::string_view uri) {
    const auto uriKind = classifyUri(uri);
    if (uriKind == UriKind::LocalPath) {
        return std::string(uri);
    }
    if (uriKind == UriKind::FileUri) {
        const auto       colon = uri.find(':');
        std::string_view rest  = (colon == std::string_view::npos) ? std::string_view{} : uri.substr(colon + 1);
        std::string_view path;
        if (rest.starts_with("//")) {
            rest.remove_prefix(2);
            const auto             slash     = rest.find('/');
            const std::string_view authority = (slash == std::string_view::npos) ? rest : rest.substr(0, slash);
            path                             = (slash == std::string_view::npos) ? std::string_view{} : rest.substr(slash);
            if (!authority.empty() && authority != "localhost") {
                return std::unexpected(gr::Error{std::format("URI (`{}`) with non-local host (`{}`) is not supported.", uri, authority)});
            }
        } else {
            path = rest;
        }

        const auto first = path.find_first_not_of('/');
        path             = (first == std::string_view::npos) ? std::string_view{} : path.substr(first);
        return std::format("/{}", path);
    }
    return std::unexpected(gr::Error{std::format("not a local file input: {}", uri)});
}

} // namespace detail

} // namespace gr::algorithm::fileio
#endif // FILEIOHELPERS_HPP
