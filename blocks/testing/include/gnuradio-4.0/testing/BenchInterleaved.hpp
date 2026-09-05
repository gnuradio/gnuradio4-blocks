#ifndef GNURADIO_TESTING_BENCH_INTERLEAVED_HPP
#define GNURADIO_TESTING_BENCH_INTERLEAVED_HPP

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <functional>
#include <print>
#include <span>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace gr::blocks::testing::bench {

/// @brief One thing being measured. `body` processes `samplesPerCall` samples and returns a value derived from its
/// output, which is summed and printed so the optimizer cannot delete the work.
///
/// `bitsPerCall` is the bits the call moves, which for a bit-field block is the figure that compares two item widths:
/// the cost of these blocks is proportional to bits and not to items, so one item of eight bits and eight items of one
/// bit are the same work reported two ways. Left zero, the report carries throughput instead.
///
/// `samples` overrides the run-wide sample count for this arm, which a filter comparison needs: a pass is sized by its
/// own tap count, and 11001 taps at decimation one against 143 at sixty-four are four orders apart in cost per sample,
/// so one buffer cannot serve both. `tapsPerSample` is `N/M`, the multiply-accumulates an input sample costs, and is
/// what turns the headline figure into a per-tap number comparable across shapes.
struct Arm {
    std::string             label;
    std::function<double()> body;
    std::size_t             bitsPerCall   = 0UZ;
    std::size_t             samples       = 0UZ;
    double                  tapsPerSample = 0.0;

    Arm(std::string armLabel, std::function<double()> armBody, std::size_t bits = 0UZ, std::size_t armSamples = 0UZ, double taps = 0.0) : label(std::move(armLabel)), body(std::move(armBody)), bitsPerCall(bits), samples(armSamples), tapsPerSample(taps) {}
    Arm(std::string armLabel, std::size_t armSamples, std::function<double()> armBody) : label(std::move(armLabel)), body(std::move(armBody)), samples(armSamples) {}
    Arm(std::string armLabel, std::size_t armSamples, double taps, std::function<double()> armBody) : label(std::move(armLabel)), body(std::move(armBody)), samples(armSamples), tapsPerSample(taps) {}
};

/**
 * @brief Run every arm once per repeat, in the same order, and report the best with its spread.
 *
 * Interleaving is what makes the arms comparable: a clock ramp, a thermal excursion or a neighbor's build lands on all
 * of them alike rather than on whichever ran last. The first pass is the warm-up and is discarded. The figure is the
 * best of the rest, because the best is the one the machine is capable of and every larger number is something else
 * that happened; the spread is printed beside it so a run that was disturbed says so.
 *
 * Pin the run to one core: on a hybrid CPU the same binary reports figures a factor of three apart according to
 * which core type it lands on.
 */
inline void report(std::span<Arm> arms, std::size_t samplesPerCall, std::size_t repeats) {
    std::vector<double> best(arms.size(), 1e300);
    std::vector<double> worst(arms.size(), 0.0);
    double              checksum = 0.0;

    for (std::size_t repeat = 0UZ; repeat <= repeats; ++repeat) {
        for (std::size_t a = 0UZ; a < arms.size(); ++a) {
            const std::size_t divisor = arms[a].samples == 0UZ ? samplesPerCall : arms[a].samples;
            const auto        start   = std::chrono::steady_clock::now();
            checksum += arms[a].body();
            const double ns = std::chrono::duration<double, std::nano>(std::chrono::steady_clock::now() - start).count() / static_cast<double>(divisor);
            if (repeat > 0UZ) {
                best[a]  = std::min(best[a], ns);
                worst[a] = std::max(worst[a], ns);
            }
        }
    }

    if (samplesPerCall == 0UZ) {
        std::println("best of {} interleaved runs after one discarded warm-up, each arm over its own sample count", repeats);
    } else {
        std::println("{} samples per call, best of {} interleaved runs after one discarded warm-up", samplesPerCall, repeats);
    }
    for (std::size_t a = 0UZ; a < arms.size(); ++a) {
        if (arms[a].bitsPerCall != 0UZ) {
            const double perBit = best[a] * static_cast<double>(arms[a].samples == 0UZ ? samplesPerCall : arms[a].samples) / static_cast<double>(arms[a].bitsPerCall);
            std::println("{:<44} {:7.3f} ns/item    (spread {:6.3f})  {:7.3f} ns/bit", arms[a].label, best[a], worst[a] - best[a], perBit);
        } else if (arms[a].tapsPerSample > 0.0) {
            std::println("{:<44} {:10.3f} ns/input  (spread {:8.3f})  {:.4f} ns per tap and output", arms[a].label, best[a], worst[a] - best[a], best[a] / arms[a].tapsPerSample);
        } else {
            std::println("{:<44} {:7.3f} ns/sample  (spread {:6.3f})  {:9.1f} Msample/s", arms[a].label, best[a], worst[a] - best[a], 1e3 / best[a]);
        }
    }
    std::println("[checksum {:g}]", checksum);
}

/// @brief `report` for a set whose arms each carry their own sample count.
inline void report(std::span<Arm> arms, std::size_t repeats) { report(arms, 0UZ, repeats); }

/// @brief One arm measured on its own: `body` processes `samplesPerCall` samples and returns a value derived from its
/// output; the returned values are summed and printed, which is what keeps the optimizer from deleting the work.
template<typename FBody>
void report(std::string_view label, std::size_t samplesPerCall, std::size_t iterations, FBody&& body) {
    double checksum = body();

    const auto start = std::chrono::steady_clock::now();
    for (std::size_t i = 0UZ; i < iterations; ++i) {
        checksum += body();
    }
    const double elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
    const double samples = static_cast<double>(iterations * samplesPerCall);

    std::println("{:<34} {:7.2f} ns/sample {:9.1f} Msample/s   [checksum {:g}]", label, 1e9 * elapsed / samples, samples / elapsed / 1e6, checksum);
}

} // namespace gr::blocks::testing::bench

#endif // GNURADIO_TESTING_BENCH_INTERLEAVED_HPP
