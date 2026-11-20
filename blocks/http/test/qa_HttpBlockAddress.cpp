#include <boost/ut.hpp>

#include <cstddef>
#include <cstdint>
#include <expected>
#include <filesystem>
#include <format>
#include <fstream>
#include <iterator>
#include <optional>
#include <print>
#include <random>
#include <string>
#include <string_view>
#include <system_error>
#include <thread>
#include <utility>
#include <vector>

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/Graph.hpp>
#include <gnuradio-4.0/Scheduler.hpp>
#include <gnuradio-4.0/http/HttpBlock.hpp>
#include <gnuradio-4.0/meta/formatter.hpp>
#include <gnuradio-4.0/testing/TagMonitors.hpp>

// The HTTP blocks refuse every url but an http: or https: address. These cases need no server: a refused block
// reaches no host.

struct MapCounter : gr::Block<MapCounter> {
    gr::PortIn<gr::pmt::Value::Map> in;
    std::size_t                     count = 0UZ;

    GR_MAKE_REFLECTABLE(MapCounter, in);

    void processOne(const gr::pmt::Value::Map& /*value*/) { ++count; }
};

namespace {

// Makes a new directory under the temporary directory the working directory, and on exit restores the previous one
// and removes the new one.
struct ScopedWorkingDirectory {
    std::filesystem::path previous = std::filesystem::current_path();
    std::filesystem::path directory;

    ScopedWorkingDirectory() {
        std::random_device random;
        for (int attempt = 0; attempt < 128 && directory.empty(); ++attempt) {
            const auto candidate = std::filesystem::temp_directory_path() / std::format("gr4-http-cwd-{:08x}{:08x}", random(), random());
            if (std::filesystem::create_directory(candidate)) {
                directory = candidate;
            }
        }
        std::filesystem::current_path(directory);
    }
    ScopedWorkingDirectory(const ScopedWorkingDirectory&)            = delete;
    ScopedWorkingDirectory& operator=(const ScopedWorkingDirectory&) = delete;
    ~ScopedWorkingDirectory() {
        std::error_code ec;
        std::filesystem::current_path(previous, ec);
        std::filesystem::remove_all(directory, ec);
        if (ec) {
            std::println(stderr, "ScopedWorkingDirectory: cannot remove '{}': {}", directory.string(), ec.message());
        }
    }
};

std::expected<void, gr::Error> runAndWait(gr::scheduler::Simple<>& sched) {
#ifdef __EMSCRIPTEN__
    // the file reader and writer refuse to block the main WASM thread
    std::optional<std::expected<void, gr::Error>> result;
    std::thread([&sched, &result] { result = sched.runAndWait(); }).join();
    return std::move(*result);
#else
    return sched.runAndWait();
#endif
}

std::string errorMessage(const std::expected<void, gr::Error>& result) { return result.has_value() ? std::string{} : result.error().message; }

// the urls each case gives: a relative and an absolute path, both file: forms and an ftp: address
std::vector<std::string> refusedUrls(const std::filesystem::path& directory, std::string_view name) {
    const std::string absolute = (directory / name).string();
    return {std::string(name), absolute, std::format("file:{}", absolute), std::format("file://{}", absolute), std::format("ftp://127.0.0.1/{}", name)};
}

} // namespace

const boost::ut::suite<"HTTP block addresses"> httpBlockAddressTests = [] {
    using namespace boost::ut;

    "HttpSink refuses at start a url other than an http: or https: address and writes no file"_test = [] {
        const ScopedWorkingDirectory    workingDirectory;
        const std::vector<std::uint8_t> payload{'p', 'a', 'r', 'a', 'm'};

        for (const std::string& url : refusedUrls(workingDirectory.directory, "out.bin")) {
            gr::Graph graph;
            auto&     source = graph.emplaceBlock<gr::blocks::testing::TagSource<std::uint8_t, gr::blocks::testing::ProcessFunction::USE_PROCESS_BULK>>({{"values", gr::Tensor<std::uint8_t>(payload.begin(), payload.end())}, {"n_samples_max", gr::Size_t(payload.size())}});
            auto&     sink   = graph.emplaceBlock<gr::blocks::http::HttpSink>({{"url", url}});
            expect(graph.connect<"out", "in">(source, sink).has_value()) << url;

            gr::scheduler::Simple<> sched;
            expect(sched.exchange(std::move(graph)).has_value()) << url;
            const auto result = runAndWait(sched);
            expect(!result.has_value()) << url << ": the run must fail";
            expect(sink.state() == gr::lifecycle::State::ERROR) << url << ": the sink's start() must fail, state " << gr::meta::enumName(sink.state()).value_or("");
            expect(errorMessage(result).contains(std::format("url '{}' must be an http: or https: address", url))) << url << ": " << errorMessage(result);
            expect(std::filesystem::is_empty(workingDirectory.directory)) << url << ": no file is made";

            // start() alone, with no write to come, refuses the url
            std::string refusal;
            try {
                gr::blocks::http::HttpSink alone(gr::property_map{{"url", url}});
                alone.settings().init();
                std::ignore = alone.settings().applyStagedParameters();
                alone.start();
            } catch (const gr::exception& refused) {
                refusal = refused.message;
            }
            expect(refusal.contains(std::format("url '{}' must be an http: or https: address", url))) << url << ": start() alone: " << refusal;
        }
    };

    "HttpSource refuses at start a url other than an http: or https: address and reads no file"_test = [] {
        const ScopedWorkingDirectory workingDirectory;
        std::ofstream(workingDirectory.directory / "in.txt", std::ios::binary) << "Hello world!";

        for (const std::string& url : refusedUrls(workingDirectory.directory, "in.txt")) {
            gr::Graph graph;
            auto&     source  = graph.emplaceBlock<gr::blocks::http::HttpSource>({{"url", url}});
            auto&     counter = graph.emplaceBlock<MapCounter>();
            expect(graph.connect<"out", "in">(source, counter).has_value()) << url;

            gr::scheduler::Simple<> sched;
            expect(sched.exchange(std::move(graph)).has_value()) << url;
            const auto result = runAndWait(sched);
            expect(!result.has_value()) << url << ": the run must fail";
            expect(source.state() == gr::lifecycle::State::ERROR) << url << ": the source's start() must fail, state " << gr::meta::enumName(source.state()).value_or("");
            expect(errorMessage(result).contains(std::format("url '{}' must be an http: or https: address", url))) << url << ": " << errorMessage(result);
            expect(eq(counter.count, 0UZ)) << url << ": the source publishes nothing";
        }
        std::error_code ec;
        expect(eq(std::ranges::distance(std::filesystem::directory_iterator(workingDirectory.directory, ec)), std::ptrdiff_t{1})) << "no file is made beside the one the source would read";
    };
};

int main() { /* tests are statically executed */ return boost::ut::cfg<boost::ut::override>.run(); }
