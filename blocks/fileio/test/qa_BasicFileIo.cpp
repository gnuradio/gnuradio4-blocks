#include <boost/ut.hpp>

#include <gnuradio-4.0/fileio/BasicFileIo.hpp>

#include <gnuradio-4.0/Scheduler.hpp>
#include <gnuradio-4.0/testing/NullSources.hpp>

#include <format>
#include <optional>

namespace {
using namespace std::chrono_literals;
template<typename Scheduler>
auto createWatchdog(Scheduler& sched, std::chrono::seconds timeOut = 2s, std::chrono::milliseconds pollingPeriod = 40ms) {
    auto externalInterventionNeeded = std::make_shared<std::atomic_bool>(false);

    std::thread watchdogThread([&sched, externalInterventionNeeded, timeOut, pollingPeriod]() {
        auto timeout = std::chrono::steady_clock::now() + timeOut;
        while (std::chrono::steady_clock::now() < timeout) {
            // a run ends in STOPPED, or in ERROR when the scheduler's start could not complete
            if (const auto state = sched.state(); state == gr::lifecycle::State::STOPPED || state == gr::lifecycle::State::ERROR) {
                return;
            }
            std::this_thread::sleep_for(pollingPeriod);
        }
        std::println("watchdog kicked in");
        externalInterventionNeeded->store(true, std::memory_order_relaxed);
        sched.requestStop();
        std::println("requested scheduler to stop");
    });

    return std::make_pair(std::move(watchdogThread), externalInterventionNeeded);
}

template<typename Scheduler>
std::expected<void, gr::Error> runSchedulerAndWait(Scheduler& sched) {
#ifdef __EMSCRIPTEN__
    std::optional<std::expected<void, gr::Error>> result;
    std::thread                                   worker([&sched, &result]() { result = sched.runAndWait(); });
    worker.join();
    return std::move(*result);
#else
    return sched.runAndWait();
#endif
}

template<typename DataType>
void runTest(const gr::blocks::fileio::Mode mode) {
    using namespace boost::ut;
    using namespace gr::blocks::fileio;
    using namespace gr::blocks::testing;
    using scheduler = gr::scheduler::Simple<>;

    constexpr gr::Size_t nSamples    = 1024U;
    const gr::Size_t     maxFileSize = mode == gr::blocks::fileio::Mode::multi ? 256U : 0U;
    std::string          modeName{magic_enum::enum_name(mode)};
    std::string          fileName = std::format("/tmp/gr4_file_sink_test/TestFileName_{}.bin", modeName);
    gr::blocks::fileio::detail::deleteFilesContaining(fileName);

    "BasicFileSink"_test = [&] { // NOSONAR capture all
        std::string testCaseName = std::format("BasicFileSink: failed for type '{}' and '{}", gr::meta::type_name<DataType>(), modeName);
        gr::Graph   flow;

        auto& source   = flow.emplaceBlock<ConstantSource<DataType>>({{"n_samples_max", nSamples}});
        auto& fileSink = flow.emplaceBlock<BasicFileSink<DataType>>({{"file_name", fileName}, {"mode", modeName}, {"max_bytes_per_file", maxFileSize}});
        expect(flow.connect<"out", "in">(source, fileSink).has_value());

        scheduler sched;
        if (auto ret = sched.exchange(std::move(flow)); !ret) {
            throw std::runtime_error(std::format("failed to initialize scheduler: {}", ret.error()));
        }
        auto [watchdogThread, externalInterventionNeeded] = createWatchdog(sched, 2s);
        expect(runSchedulerAndWait(sched).has_value()) << testCaseName;

        if (watchdogThread.joinable()) {
            watchdogThread.join();
        }
        expect(!externalInterventionNeeded->load(std::memory_order_relaxed)) << testCaseName;
        expect(eq(source.count, nSamples)) << testCaseName;
        expect(eq(fileSink._totalBytesWritten / sizeof(DataType), nSamples)) << testCaseName;

        std::vector<std::filesystem::path> files = gr::blocks::fileio::detail::getSortedFilesContaining(fileName);
        if (mode == gr::blocks::fileio::Mode::multi) {
            // greater-equal 'ge' because files can be legitimally zero-sized
            expect(ge(files.size(), (nSamples * sizeof(DataType)) / maxFileSize)) << testCaseName;
        } else {
            expect(eq(files.size(), 1U)) << testCaseName;
        }
        for (const auto& file : files) {
            auto fileSize = gr::blocks::fileio::detail::getFileSize(file);
            if (mode == gr::blocks::fileio::Mode::multi) {
                // less-equal 'le' because files can be legitimally zero-sized
                expect(le(fileSize, maxFileSize)) << testCaseName;
            } else {
                expect(eq(fileSize, nSamples * sizeof(DataType))) << testCaseName;
            }
        }
    };

    // N.B. test directory contains the output files from the previous sink test
    "BasicFileSource"_test = [&] { // NOSONAR capture all
        std::string testCaseName = std::format("BasicFileSource: failed for type '{}' and '{}", gr::meta::type_name<DataType>(), modeName);
        gr::Graph   flow;
        auto&       fileSource = flow.emplaceBlock<BasicFileSource<DataType>>({{"file_name", fileName}, {"mode", modeName}});
        auto&       sink       = flow.emplaceBlock<CountingSink<DataType>>();

        expect(flow.connect<"out", "in">(fileSource, sink).has_value());

        scheduler schedRead;
        if (auto ret = schedRead.exchange(std::move(flow)); !ret) {
            throw std::runtime_error(std::format("failed to initialize scheduler: {}", ret.error()));
        }
        auto [watchdogThreadRead, externalInterventionNeededRead] = createWatchdog(schedRead, 2s);
        expect(runSchedulerAndWait(schedRead).has_value()) << testCaseName;

        if (watchdogThreadRead.joinable()) {
            watchdogThreadRead.join();
        }
        expect(!externalInterventionNeededRead->load(std::memory_order_relaxed)) << testCaseName;
        expect(eq(sink.count, nSamples)) << testCaseName;
        expect(eq(fileSource._totalBytesRead, nSamples * sizeof(DataType))) << testCaseName;
    };

    // Test for `offset` and `length` parameters
    "BasicFileSource with offset and length"_test = [&] { // NOSONAR capture all
        constexpr gr::Size_t offsetSamples = 8U;
        constexpr gr::Size_t lengthSamples = 8U;
        std::string          testCaseName  = std::format("BasicFileSource with offset and length: failed for type '{}' and '{}", gr::meta::type_name<DataType>(), modeName);
        gr::Graph            flow;
        auto&                fileSource = flow.emplaceBlock<BasicFileSource<DataType>>({{"file_name", fileName}, {"mode", modeName}, {"offset", offsetSamples}, {"length", lengthSamples}});
        auto&                sink       = flow.emplaceBlock<CountingSink<DataType>>();

        expect(flow.connect<"out", "in">(fileSource, sink).has_value());

        scheduler schedRead;
        if (auto ret = schedRead.exchange(std::move(flow)); !ret) {
            throw std::runtime_error(std::format("failed to initialize scheduler: {}", ret.error()));
        }
        auto [watchdogThreadRead, externalInterventionNeededRead] = createWatchdog(schedRead, 2s);
        expect(runSchedulerAndWait(schedRead).has_value()) << testCaseName;

        if (watchdogThreadRead.joinable()) {
            watchdogThreadRead.join();
        }
        expect(!externalInterventionNeededRead->load(std::memory_order_relaxed)) << testCaseName;

        auto nonEmptyFileCount = static_cast<gr::Size_t>(std::ranges::count_if(gr::blocks::fileio::detail::getSortedFilesContaining(fileName), [](const auto& file) { return std::filesystem::file_size(file) > 0; }));
        expect(eq(sink.count, nonEmptyFileCount * lengthSamples)) << testCaseName;
        expect(eq(fileSource._totalBytesRead, nonEmptyFileCount * lengthSamples * sizeof(DataType))) << testCaseName;
    };

    expect(!gr::blocks::fileio::detail::deleteFilesContaining(fileName).empty());
}

// sets the working directory to a new temporary directory, and restores it and removes the directory on exit
struct ScopedWorkingDirectory {
    std::filesystem::path previous  = std::filesystem::current_path();
    std::filesystem::path directory = std::filesystem::temp_directory_path() / std::format("gr4_file_io_cwd_{}", std::chrono::steady_clock::now().time_since_epoch().count());

    ScopedWorkingDirectory() {
        std::filesystem::create_directories(directory);
        std::filesystem::current_path(directory);
    }
    ScopedWorkingDirectory(const ScopedWorkingDirectory&)            = delete;
    ScopedWorkingDirectory& operator=(const ScopedWorkingDirectory&) = delete;
    ~ScopedWorkingDirectory() {
        std::error_code ec;
        std::filesystem::current_path(previous, ec);
        std::filesystem::remove_all(directory, ec);
    }
};

template<typename TValue>
std::string errorMessage(const std::expected<TValue, gr::Error>& result) {
    return result.has_value() ? std::string{} : result.error().message;
}

std::expected<void, gr::Error> writeFloatFile(const std::string& fileName, const std::string& modeName, gr::Size_t nSamples, gr::Size_t maxBytesPerFile = 0U) {
    using namespace boost::ut;
    using namespace gr::blocks::fileio;
    using namespace gr::blocks::testing;

    gr::Graph flow;
    auto&     source   = flow.emplaceBlock<ConstantSource<float>>({{"n_samples_max", nSamples}});
    auto&     fileSink = flow.emplaceBlock<BasicFileSink<float>>({{"file_name", fileName}, {"mode", modeName}, {"max_bytes_per_file", maxBytesPerFile}});
    expect(flow.connect<"out", "in">(source, fileSink).has_value());

    gr::scheduler::Simple<> sched;
    if (auto ret = sched.exchange(std::move(flow)); !ret) {
        throw std::runtime_error(std::format("failed to initialize scheduler: {}", ret.error()));
    }
    auto [watchdogThread, externalInterventionNeeded] = createWatchdog(sched, 2s);
    auto result                                       = runSchedulerAndWait(sched);
    if (watchdogThread.joinable()) {
        watchdogThread.join();
    }
    expect(!externalInterventionNeeded->load(std::memory_order_relaxed)) << fileName;
    return result;
}

// returns the number of samples the source delivered
std::expected<gr::Size_t, gr::Error> readFloatFile(const std::string& fileName, const std::string& modeName) {
    using namespace boost::ut;
    using namespace gr::blocks::fileio;
    using namespace gr::blocks::testing;

    gr::Graph flow;
    auto&     fileSource = flow.emplaceBlock<BasicFileSource<float>>({{"file_name", fileName}, {"mode", modeName}});
    auto&     sink       = flow.emplaceBlock<CountingSink<float>>();
    expect(flow.connect<"out", "in">(fileSource, sink).has_value());

    gr::scheduler::Simple<> sched;
    if (auto ret = sched.exchange(std::move(flow)); !ret) {
        throw std::runtime_error(std::format("failed to initialize scheduler: {}", ret.error()));
    }
    auto [watchdogThread, externalInterventionNeeded] = createWatchdog(sched, 2s);
    auto result                                       = runSchedulerAndWait(sched);
    if (watchdogThread.joinable()) {
        watchdogThread.join();
    }
    expect(!externalInterventionNeeded->load(std::memory_order_relaxed)) << fileName;
    if (!result.has_value()) {
        return std::unexpected(result.error());
    }
    return sink.count.value;
}

} // anonymous namespace

const boost::ut::suite<"basic file IO tests"> basicFileIOTests = [] {
    using namespace std::chrono_literals;
    using namespace boost::ut;
    using namespace gr;

    constexpr auto kArithmeticTypes = std::tuple<uint8_t, int16_t, int32_t, float, std::complex<float>>();

    using enum gr::blocks::fileio::Mode;
    "overwrite mode"_test = []<typename T>(const T&) { runTest<T>(overwrite); } | kArithmeticTypes;

    "append mode"_test = []<typename T>(const T&) { runTest<T>(append); } | kArithmeticTypes;

    "create new mode"_test = []<typename T>(const T&) { runTest<T>(multi); } | kArithmeticTypes;

    "double header type smoke test"_test = [] { runTest<double>(overwrite); };

    "BasicFileSink rejects a rotation cap outside multi mode"_test = [] {
        using namespace gr::blocks::fileio;
        const auto applySettings = [](const char* modeName, gr::Size_t cap) {
            BasicFileSink<float> sink({{"file_name", "/tmp/gr4_file_sink_test/cap.bin"}, {"mode", modeName}, {"max_bytes_per_file", cap}});
            sink.settings().init();
            std::ignore = sink.settings().applyStagedParameters();
        };
        expect(throws([&] { applySettings("overwrite", 256U); }));
        expect(throws([&] { applySettings("append", 256U); }));
        expect(nothrow([&] { applySettings("multi", 256U); }));
        expect(nothrow([&] { applySettings("overwrite", 0U); }));
    };

    "round-trip of a recording larger than one output span"_test = [] {
        using namespace gr::blocks::fileio;
        using namespace gr::blocks::testing;
        using scheduler = gr::scheduler::Simple<>;

        constexpr gr::Size_t nSamples = 100'000U; // 24x the 4096-sample output ring: one span cannot hold it
        const std::string    fileName = "/tmp/gr4_file_sink_test/TestLargeRoundTrip.bin";
        gr::blocks::fileio::detail::deleteFilesContaining(fileName);

        gr::Graph writeFlow;
        auto&     source   = writeFlow.emplaceBlock<ConstantSource<float>>({{"n_samples_max", nSamples}});
        auto&     fileSink = writeFlow.emplaceBlock<BasicFileSink<float>>({{"file_name", fileName}, {"mode", "overwrite"}});
        expect(writeFlow.connect<"out", "in">(source, fileSink).has_value());

        scheduler schedWrite;
        if (auto ret = schedWrite.exchange(std::move(writeFlow)); !ret) {
            throw std::runtime_error(std::format("failed to initialize scheduler: {}", ret.error()));
        }
        expect(runSchedulerAndWait(schedWrite).has_value());
        expect(eq(fileSink._totalBytesWritten, nSamples * sizeof(float)));

        gr::Graph readFlow;
        auto&     fileSource = readFlow.emplaceBlock<BasicFileSource<float>>({{"file_name", fileName}, {"mode", "overwrite"}});
        auto&     sink       = readFlow.emplaceBlock<CountingSink<float>>();
        expect(readFlow.connect<"out", "in">(fileSource, sink).has_value());

        scheduler schedRead;
        if (auto ret = schedRead.exchange(std::move(readFlow)); !ret) {
            throw std::runtime_error(std::format("failed to initialize scheduler: {}", ret.error()));
        }
        auto [watchdogThread, externalInterventionNeeded] = createWatchdog(schedRead, 5s);
        expect(runSchedulerAndWait(schedRead).has_value());
        if (watchdogThread.joinable()) {
            watchdogThread.join();
        }
        expect(!externalInterventionNeeded->load(std::memory_order_relaxed)) << "reading a multi-span recording must not stall";
        expect(eq(sink.count, nSamples));
        expect(eq(fileSource._totalBytesRead, nSamples * sizeof(float)));

        expect(!gr::blocks::fileio::detail::deleteFilesContaining(fileName).empty());
    };

    "BasicFileSink writes a bare file name into the working directory"_test = [] {
        constexpr gr::Size_t nSamples        = 1024U;
        constexpr gr::Size_t maxBytesPerFile = 256U;

        for (const std::string modeName : {"overwrite", "multi"}) {
            ScopedWorkingDirectory workingDirectory;
            const auto             written = writeFloatFile("tone.f32", modeName, nSamples, modeName == "multi" ? maxBytesPerFile : 0U);
            expect(written.has_value()) << modeName << ": " << errorMessage(written);

            // the multi mode names each file '<time>_<index>_tone.f32'
            std::size_t    nFiles = 0UZ;
            std::uintmax_t nBytes = 0U;
            for (const auto& entry : std::filesystem::directory_iterator(workingDirectory.directory)) {
                expect(entry.is_regular_file() && entry.path().filename().string().ends_with("tone.f32")) << modeName << ": " << entry.path().string();
                nFiles++;
                nBytes += entry.file_size();
            }
            if (modeName == "multi") {
                expect(ge(nFiles, nSamples * sizeof(float) / maxBytesPerFile)) << modeName;
                expect(eq(gr::blocks::fileio::detail::deleteFilesContaining("tone.f32").size(), nFiles)) << modeName;
            } else {
                expect(eq(nFiles, 1UZ)) << modeName;
            }
            expect(eq(nBytes, nSamples * sizeof(float))) << modeName;
        }
    };

    "BasicFileSource reads a bare file name from the working directory"_test = [] {
        constexpr gr::Size_t nSamples        = 1024U;
        constexpr gr::Size_t maxBytesPerFile = 256U;

        for (const std::string modeName : {"overwrite", "multi"}) {
            ScopedWorkingDirectory workingDirectory;
            const auto             written = writeFloatFile((workingDirectory.directory / "tone.f32").string(), modeName, nSamples, modeName == "multi" ? maxBytesPerFile : 0U);
            expect(written.has_value()) << modeName << ": " << errorMessage(written);

            const auto nRead = readFloatFile("tone.f32", modeName);
            expect(nRead.has_value()) << modeName << ": " << errorMessage(nRead);
            expect(eq(nRead.value_or(0U), nSamples)) << modeName;
        }
    };

    "BasicFileSink creates the missing directories of a file name"_test = [] {
        constexpr gr::Size_t   nSamples = 1024U;
        ScopedWorkingDirectory workingDirectory;

        const auto written = writeFloatFile("missing/sub/tone.f32", "overwrite", nSamples);
        expect(written.has_value()) << errorMessage(written);
        std::error_code ec;
        expect(eq(std::filesystem::file_size(workingDirectory.directory / "missing/sub/tone.f32", ec), nSamples * sizeof(float))) << ec.message();
    };

    "BasicFileSource refuses a file name whose directory does not exist"_test = [] {
        ScopedWorkingDirectory workingDirectory;

        const auto nRead = readFloatFile("missing/tone.f32", "overwrite");
        expect(!nRead.has_value());
        expect(errorMessage(nRead).contains("path/file 'missing/tone.f32' does not exist.")) << errorMessage(nRead);
        expect(!std::filesystem::exists(workingDirectory.directory / "missing"));
    };
};

int main() { /* not needed for UT */ }
