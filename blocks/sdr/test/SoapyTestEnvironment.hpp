#ifndef GNURADIO_SDR_TEST_SOAPY_TEST_ENVIRONMENT_HPP
#define GNURADIO_SDR_TEST_SOAPY_TEST_ENVIRONMENT_HPP

#include <cstdlib>
#include <format>
#include <stdexcept>
#include <string>

// The loopback module's directory and the empty SoapySDR root reach the test through its environment: ctest sets
// GR_SDR_TEST_SOAPY_MODULE_DIR and GR_SDR_TEST_SOAPY_EMPTY_ROOT (see CMakeLists.txt), and a packaged test's runner sets
// them to where it installed the two directories. The test binary carries no path of the tree it was built in.

// A directory the run names through the test's environment. Without it the test cannot find the loopback module or
// confine SoapySDR to it, so it stops and names the variable. blocks/sdr/Readme.md shows a direct run that sets them.
[[nodiscard]] inline std::string soapyTestDirectory(const char* variable) {
    const char* value = std::getenv(variable);
    if (value == nullptr || *value == '\0') {
        throw std::runtime_error(std::format("the environment variable {} is not set or empty; run the test through ctest, or set it as blocks/sdr/Readme.md describes", variable));
    }
    return value;
}

// the directory holding the loopback module, gr-sdr-loopback
[[nodiscard]] inline std::string soapyModuleDirectory() { return soapyTestDirectory("GR_SDR_TEST_SOAPY_MODULE_DIR"); }

// an empty directory that stands in for SoapySDR's installation root, so that none of the host's modules load
[[nodiscard]] inline std::string soapyEmptyRoot() { return soapyTestDirectory("GR_SDR_TEST_SOAPY_EMPTY_ROOT"); }

#endif // GNURADIO_SDR_TEST_SOAPY_TEST_ENVIRONMENT_HPP
