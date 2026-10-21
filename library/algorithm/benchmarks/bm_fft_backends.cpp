#include <algorithm>
#include <bit>
#include <chrono>
#include <cmath>
#include <complex>
#include <cstddef>
#include <numbers>
#include <print>
#include <string_view>
#include <thread>
#include <tuple>
#include <vector>

#include <gnuradio-4.0/algorithm/fourier/fft.hpp>

// The three backends of gr::algorithm::FFT on two shapes. The first is a long single-precision complex transform of a
// power-of-two length, repeated at a fixed size. The second is the short real-input audio transform.
//
// Each row has two columns beyond the elapsed time. The first is the cost of one butterfly unit, the time divided by
// N log2(N). It compares across lengths and across engines. The second is the error, the relative L2 norm of the whole
// spectrum against a double-precision reference of the same input. It shows how much accuracy a backend gives up for
// its speed.

using namespace std::chrono;

template<typename T>
using Aligned = std::vector<T, gr::allocator::Aligned<T>>;

template<typename T>
Aligned<T> broadbandSignal(std::size_t N) {
    Aligned<T> signal(N);
    for (std::size_t i = 0; i < N; ++i) {
        const double x = std::sin(0.7 * static_cast<double>(i) + 0.3) + 0.4 * std::cos(2.9 * static_cast<double>(i));
        const double y = std::cos(1.3 * static_cast<double>(i) - 0.9) - 0.2 * std::sin(0.11 * static_cast<double>(i));
        if constexpr (gr::meta::complex_like<T>) {
            signal[i] = T(static_cast<typename T::value_type>(x), static_cast<typename T::value_type>(y));
        } else {
            signal[i] = static_cast<T>(x);
        }
    }
    return signal;
}

/// the same transform in double precision through PocketFFT
template<typename TIn>
[[nodiscard]] std::vector<std::complex<double>> doubleReference(const Aligned<TIn>& signal) {
    using WideIn = std::conditional_t<gr::meta::complex_like<TIn>, std::complex<double>, double>;

    Aligned<WideIn> wide(signal.size());
    for (std::size_t i = 0; i < signal.size(); ++i) {
        wide[i] = static_cast<WideIn>(signal[i]);
    }

    gr::algorithm::FFT<WideIn, std::complex<double>> fft{};
    fft.backend = gr::algorithm::FftBackend::PocketFFT;
    Aligned<std::complex<double>> out(signal.size());
    fft.compute(wide, out);
    return {out.begin(), out.end()};
}

[[nodiscard]] double relativeL2Error(const Aligned<std::complex<float>>& actual, const std::vector<std::complex<double>>& reference) {
    double numerator   = 0.;
    double denominator = 0.;
    for (std::size_t k = 0; k < reference.size(); ++k) {
        const std::complex<double> difference(static_cast<double>(actual[k].real()) - reference[k].real(), static_cast<double>(actual[k].imag()) - reference[k].imag());
        numerator += std::norm(difference);
        denominator += std::norm(reference[k]);
    }
    return denominator > 0. ? std::sqrt(numerator / denominator) : std::sqrt(numerator);
}

// Each measurement gets about the same amount of work. A short length is then not timed against the clock's own
// resolution, and a long one does not dominate the run.
[[nodiscard]] std::size_t repetitionsFor(std::size_t N) {
    constexpr double  kUnitBudget = 3.e8; // butterfly units per measurement
    const double      units       = static_cast<double>(N) * std::log2(static_cast<double>(N));
    const std::size_t requested   = static_cast<std::size_t>(kUnitBudget / units);
    return std::clamp(requested, 5UZ, 2000UZ);
}

template<typename TIn>
void benchBackend(std::string_view label, const Aligned<TIn>& signal, const std::vector<std::complex<double>>& reference, gr::algorithm::FftBackend backend, std::size_t threads = 1UZ) {
    const std::size_t                            N = signal.size();
    Aligned<std::complex<float>>                 out(N);
    gr::algorithm::FFT<TIn, std::complex<float>> fft{};
    fft.backend = backend;
    fft.threads = threads;
    if (fft.resolveBackend(N) != backend) {
        std::println("  {:>10} N = {:8}           -- the backend does not take this length", label, N);
        return;
    }
    fft.compute(signal, out); // the plan and the tables are built here, not in the timed loop

    const std::size_t repetitions = repetitionsFor(N);
    const auto        start       = steady_clock::now();
    for (std::size_t i = 0; i < repetitions; ++i) {
        fft.compute(signal, out);
    }
    const auto elapsed = duration_cast<nanoseconds>(steady_clock::now() - start);

    const double perTransform = static_cast<double>(elapsed.count()) / static_cast<double>(repetitions);
    const double units        = static_cast<double>(N) * std::log2(static_cast<double>(N));
    std::println("  {:>10} N = {:8} {:2} thread(s) {:12.1f} us {:9.3f} ns/unit  error {:9.2e}  ({} repetitions)", //
        label, N, threads, perTransform * 1.e-3, perTransform / units, relativeL2Error(out, reference), repetitions);
}

// The parts of the four-step, each timed alone. The backend runs the two batches through PocketFFT's multi-dimensional
// driver. It folds the transpose into the output strides of the second batch. The design rests on two numbers. They
// are the saving from that fold and the cost of the twiddle pass between the batches. This function measures both. The
// passes are rebuilt against PocketFFT directly, and each one is timed apart from the others. The batches and the
// twiddle pass run on `threads` threads. The transpose runs on the calling thread.
void benchFourStepParts(std::size_t N, std::size_t threads) {
    using Cplx                 = std::complex<float>;
    const std::size_t exponent = static_cast<std::size_t>(std::countr_zero(N));
    const std::size_t n1       = 1UZ << (exponent / 2UZ);
    const std::size_t n2       = N / n1;

    const auto        input = broadbandSignal<Cplx>(N);
    Aligned<Cplx>     work(N);
    Aligned<Cplx>     out(N);
    std::vector<Cplx> twiddles(N);
    for (std::size_t i1 = 0; i1 < n1; ++i1) {
        for (std::size_t i2 = 0; i2 < n2; ++i2) {
            const double angle     = -2. * std::numbers::pi * static_cast<double>(i1) * static_cast<double>(i2) / static_cast<double>(N);
            twiddles[i1 * n2 + i2] = Cplx(static_cast<float>(std::cos(angle)), static_cast<float>(std::sin(angle)));
        }
    }

    const pocketfft::shape_t  shape{n1, n2};
    const pocketfft::stride_t rowMajor{static_cast<std::ptrdiff_t>(n2 * sizeof(Cplx)), static_cast<std::ptrdiff_t>(sizeof(Cplx))};
    const pocketfft::stride_t transposed{static_cast<std::ptrdiff_t>(sizeof(Cplx)), static_cast<std::ptrdiff_t>(n1 * sizeof(Cplx))};

    const std::size_t repetitions = repetitionsFor(N);
    const auto        timed       = [repetitions](auto&& body) {
        body();
        const auto start = steady_clock::now();
        for (std::size_t i = 0; i < repetitions; ++i) {
            body();
        }
        return static_cast<double>(duration_cast<nanoseconds>(steady_clock::now() - start).count()) / static_cast<double>(repetitions) * 1.e-6;
    };

    // The backend spreads the twiddle pass over the threads in equal contiguous chunks. These passes do the same, the
    // table variant by rows.
    const auto spread = [threads](std::size_t total, auto&& body) {
        pocketfft::detail::threading::thread_map(threads, [&] {
            const std::size_t index = pocketfft::detail::threading::thread_id();
            const std::size_t count = pocketfft::detail::threading::num_threads();
            const std::size_t chunk = (total + count - 1UZ) / count;
            const std::size_t begin = std::min(total, index * chunk);
            body(begin, std::min(total, begin + chunk));
        });
    };

    const double firstBatch            = timed([&] { pocketfft::c2c(shape, rowMajor, rowMajor, pocketfft::shape_t{0UZ}, true, input.data(), work.data(), 1.f, threads); });
    const double twiddlePass           = timed([&] {
        spread(N, [&](std::size_t begin, std::size_t end) {
            for (std::size_t i = begin; i < end; ++i) {
                work[i] *= twiddles[i];
            }
        });
    });
    const double secondBatchInPlace    = timed([&] { pocketfft::c2c(shape, rowMajor, rowMajor, pocketfft::shape_t{1UZ}, true, work.data(), work.data(), 1.f, threads); });
    const double secondBatchTransposed = timed([&] { pocketfft::c2c(shape, rowMajor, transposed, pocketfft::shape_t{1UZ}, true, work.data(), out.data(), 1.f, threads); });

    // The pass the fold replaces, a blocked out-of-place transpose of the same record. It reads and writes the record
    // again.
    constexpr std::size_t kBlock    = 32UZ;
    const double          transpose = timed([&] {
        for (std::size_t r = 0; r < n1; r += kBlock) {
            for (std::size_t c = 0; c < n2; c += kBlock) {
                for (std::size_t i = r; i < std::min(r + kBlock, n1); ++i) {
                    for (std::size_t j = c; j < std::min(c + kBlock, n2); ++j) {
                        out[j * n1 + i] = work[i * n2 + j];
                    }
                }
            }
        }
    });

    // The same twiddles read from two small tables instead of one N-entry table. i1*i2 splits as a*N2 + b, and every
    // entry is W_N1^a * W_N^b. The pass then moves two records instead of three.
    std::vector<Cplx> coarse(n1);
    std::vector<Cplx> fine(n2);
    for (std::size_t a = 0; a < n1; ++a) {
        coarse[a] = Cplx(std::polar(1., -2. * std::numbers::pi * static_cast<double>(a) / static_cast<double>(n1)));
    }
    for (std::size_t b = 0; b < n2; ++b) {
        fine[b] = Cplx(std::polar(1., -2. * std::numbers::pi * static_cast<double>(b) / static_cast<double>(N)));
    }
    const std::size_t shift             = static_cast<std::size_t>(std::countr_zero(n2));
    const std::size_t mask              = n2 - 1UZ;
    const double      twiddleFromTables = timed([&] {
        spread(n1, [&](std::size_t beginRow, std::size_t endRow) {
            for (std::size_t i1 = beginRow; i1 < endRow; ++i1) {
                for (std::size_t i2 = 0; i2 < n2; ++i2) {
                    const std::size_t product = i1 * i2;
                    work[i1 * n2 + i2] *= coarse[product >> shift] * fine[product & mask];
                }
            }
        });
    });

    // SimdFFT has no batched entry point. The same batch through it is a loop over one-dimensional transforms of
    // contiguous, 64-byte aligned rows. That loop is the best case for a split built on SimdFFT instead of PocketFFT.
    // Compare it with "second batch in place", the same work through PocketFFT's driver.
    std::vector<gr::algorithm::SimdFFT<float, gr::algorithm::Transform::Complex>> engines(threads);
    for (auto& engine : engines) {
        engine.resize(n2);
    }
    const double simdBatch = timed([&] {
        pocketfft::detail::threading::thread_map(threads, [&] {
            const std::size_t index = pocketfft::detail::threading::thread_id();
            const std::size_t count = pocketfft::detail::threading::num_threads();
            for (std::size_t r = index; r < n1; r += count) {
                engines[index].transform<gr::algorithm::Direction::Forward, gr::algorithm::Order::Ordered>( //
                    std::span<const float>(reinterpret_cast<const float*>(work.data() + r * n2), 2UZ * n2), std::span<float>(reinterpret_cast<float*>(out.data() + r * n2), 2UZ * n2));
            }
        });
    });

    std::println("  N = {:8} = {} x {}, {} thread(s): first batch {:8.2f} ms, twiddle pass {:8.2f} ms, second batch in place {:8.2f} ms, second batch transposing {:8.2f} ms, a separate blocked transpose {:8.2f} ms", //
        N, n1, n2, threads, firstBatch, twiddlePass, secondBatchInPlace, secondBatchTransposed, transpose);
    std::println("  {:>28} {} thread(s): the twiddle pass from two small tables {:8.2f} ms, the second batch through SimdFFT instead of PocketFFT {:8.2f} ms", //
        "", threads, twiddleFromTables, simdBatch);
}

// The same four-step built on SimdFFT instead of PocketFFT. SimdFFT transforms one contiguous, 64-byte aligned span at
// a time. It has no batched entry point and no strided access. The columns cannot be transformed where they lie in the
// record, and the result cannot be written transposed. The split on SimdFFT therefore takes five passes over the record
// against PocketFFT's three. The five passes are a transposing copy, the column batch, a transposing pass that applies
// the twiddles, the row batch and a transpose into natural bin order. The two shapes share no transpose. Every pass
// runs on the same threads. Each thread holds its own engines, because a SimdFFT instance transforms through scratch it
// owns.
void benchSimdFourStep(const Aligned<std::complex<float>>& signal, const std::vector<std::complex<double>>& reference, std::size_t threads) {
    using Cplx                 = std::complex<float>;
    const std::size_t N        = signal.size();
    const std::size_t exponent = static_cast<std::size_t>(std::countr_zero(N));
    const std::size_t n1       = 1UZ << (exponent / 2UZ);
    const std::size_t n2       = N / n1;

    Aligned<Cplx>     a(N);
    Aligned<Cplx>     b(N);
    Aligned<Cplx>     out(N);
    std::vector<Cplx> twiddles(N); // W_N^(i2*k1), laid out as the {n2, n1} array the first batch writes
    for (std::size_t i2 = 0; i2 < n2; ++i2) {
        for (std::size_t k1 = 0; k1 < n1; ++k1) {
            twiddles[i2 * n1 + k1] = Cplx(std::polar(1., -2. * std::numbers::pi * static_cast<double>(i2) * static_cast<double>(k1) / static_cast<double>(N)));
        }
    }

    std::vector<gr::algorithm::SimdFFT<float, gr::algorithm::Transform::Complex>> columns(threads);
    std::vector<gr::algorithm::SimdFFT<float, gr::algorithm::Transform::Complex>> rows(threads);
    for (std::size_t t = 0; t < threads; ++t) {
        columns[t].resize(n1);
        rows[t].resize(n2);
    }

    constexpr std::size_t kBlock   = 32UZ;
    const auto            parallel = [threads](auto&& body) { pocketfft::detail::threading::thread_map(threads, [&] { body(pocketfft::detail::threading::thread_id(), pocketfft::detail::threading::num_threads()); }); };
    const auto            reading  = [](const Cplx* p, std::size_t n) { return std::span<const float>(reinterpret_cast<const float*>(p), 2UZ * n); };
    const auto            writing  = [](Cplx* p, std::size_t n) { return std::span<float>(reinterpret_cast<float*>(p), 2UZ * n); };

    const auto oneTransform = [&] {
        parallel([&](std::size_t id, std::size_t count) { // the record read as {n1, n2}, transposed so the columns lie contiguously
            for (std::size_t r = id * kBlock; r < n1; r += count * kBlock) {
                for (std::size_t c = 0; c < n2; c += kBlock) {
                    for (std::size_t i = r; i < std::min(r + kBlock, n1); ++i) {
                        for (std::size_t j = c; j < std::min(c + kBlock, n2); ++j) {
                            a[j * n1 + i] = signal[i * n2 + j];
                        }
                    }
                }
            }
        });
        parallel([&](std::size_t id, std::size_t count) { // the n2 columns, each of length n1
            for (std::size_t r = id; r < n2; r += count) {
                columns[id].transform<gr::algorithm::Direction::Forward, gr::algorithm::Order::Ordered>(reading(a.data() + r * n1, n1), writing(b.data() + r * n1, n1));
            }
        });
        parallel([&](std::size_t id, std::size_t count) { // the twiddles, applied in the transpose back to {n1, n2}
            for (std::size_t r = id * kBlock; r < n2; r += count * kBlock) {
                for (std::size_t c = 0; c < n1; c += kBlock) {
                    for (std::size_t i = r; i < std::min(r + kBlock, n2); ++i) {
                        for (std::size_t j = c; j < std::min(c + kBlock, n1); ++j) {
                            a[j * n2 + i] = b[i * n1 + j] * twiddles[i * n1 + j];
                        }
                    }
                }
            }
        });
        parallel([&](std::size_t id, std::size_t count) { // the n1 rows, each of length n2
            for (std::size_t r = id; r < n1; r += count) {
                rows[id].transform<gr::algorithm::Direction::Forward, gr::algorithm::Order::Ordered>(reading(a.data() + r * n2, n2), writing(b.data() + r * n2, n2));
            }
        });
        parallel([&](std::size_t id, std::size_t count) { // bin k2*n1 + k1 is at [k1][k2], so transpose the record once more
            for (std::size_t r = id * kBlock; r < n1; r += count * kBlock) {
                for (std::size_t c = 0; c < n2; c += kBlock) {
                    for (std::size_t i = r; i < std::min(r + kBlock, n1); ++i) {
                        for (std::size_t j = c; j < std::min(c + kBlock, n2); ++j) {
                            out[j * n1 + i] = b[i * n2 + j];
                        }
                    }
                }
            }
        });
    };

    oneTransform();
    const std::size_t repetitions = repetitionsFor(N);
    const auto        start       = steady_clock::now();
    for (std::size_t i = 0; i < repetitions; ++i) {
        oneTransform();
    }
    const auto elapsed = duration_cast<nanoseconds>(steady_clock::now() - start);

    const double perTransform = static_cast<double>(elapsed.count()) / static_cast<double>(repetitions);
    const double units        = static_cast<double>(N) * std::log2(static_cast<double>(N));
    std::println("  {:>10} N = {:8} {:2} thread(s) {:12.1f} us {:9.3f} ns/unit  error {:9.2e}  ({} repetitions)", //
        "simd-4step", N, threads, perTransform * 1.e-3, perTransform / units, relativeL2Error(out, reference), repetitions);
}

// The native path against PocketFFT at lengths whose largest prime factor exceeds the root. Both run Bluestein there.
// The native path's cost follows the ratio of its padded length to N, and PocketFFT's follows the factors of its own
// padded length. The two arms alternate in one process, and the best of nine rounds is kept. That is the figure Auto's
// rule rests on. Run it pinned to one core. Auto's rule rests on -O3 builds. This target compiles at -O2, where
// PocketFFT's Bluestein path is up to 2.5 times slower than at -O3.
void benchLargePrimeFactor(const std::vector<std::size_t>& lengths) {
    using Cplx = std::complex<float>;
    using Fft  = gr::algorithm::FFT<Cplx, Cplx>;
    std::println("{:>8} {:>6} {:>7} {:>10} {:>10} {:>8}", "N", "pad/N", "Auto", "native", "pocketfft", "pk/nat");
    for (const std::size_t N : lengths) {
        const auto    signal = broadbandSignal<Cplx>(N);
        Aligned<Cplx> out(N);
        Fft           native{};
        Fft           pocket{};
        native.backend = gr::algorithm::FftBackend::Native;
        pocket.backend = gr::algorithm::FftBackend::PocketFFT;
        native.compute(signal, out);
        pocket.compute(signal, out);

        const double      units       = static_cast<double>(N) * std::log2(static_cast<double>(N));
        const std::size_t repetitions = std::max(3UZ, static_cast<std::size_t>(2.e7 / units));
        const auto        timed       = [&](Fft& fft) {
            const auto start = steady_clock::now();
            for (std::size_t i = 0; i < repetitions; ++i) {
                fft.compute(signal, out);
            }
            return static_cast<double>(duration_cast<nanoseconds>(steady_clock::now() - start).count()) / static_cast<double>(repetitions) / units;
        };
        double bestNative = timed(native);
        double bestPocket = timed(pocket);
        for (std::size_t round = 1UZ; round < 9UZ; ++round) {
            bestNative = std::min(bestNative, timed(native));
            bestPocket = std::min(bestPocket, timed(pocket));
        }
        std::println("{:8} {:6.2f} {:>7} {:10.3f} {:10.3f} {:8.2f}", N, static_cast<double>(std::bit_ceil(2UZ * N + 1UZ)) / static_cast<double>(N), //
            Fft::autoBackend(N) == gr::algorithm::FftBackend::Native ? "native" : "pocket", bestNative, bestPocket, bestPocket / bestNative);
    }
}

template<typename TIn>
void sweep(std::size_t N) {
    using gr::algorithm::FftBackend;
    const auto signal    = broadbandSignal<TIn>(N);
    const auto reference = doubleReference(signal);
    benchBackend("native", signal, reference, FftBackend::Native);
    benchBackend("simd", signal, reference, FftBackend::Simd);
    benchBackend("pocketfft", signal, reference, FftBackend::PocketFFT);
}

int main() {
    std::println("complex<float> forward, one instance per backend and length");
    for (std::size_t exponent = 9UZ; exponent <= 22UZ; ++exponent) {
        sweep<std::complex<float>>(1UZ << exponent);
    }

    std::println("");
    std::println("composite lengths, complex<float> forward");
    for (const std::size_t N : {1250UZ, 4000UZ, 105UZ, 1009UZ}) {
        sweep<std::complex<float>>(N);
    }

    std::println("");
    std::println("native against PocketFFT where the largest prime factor exceeds the root, ns per unit, engines alternating");
    benchLargePrimeFactor({131UZ, 251UZ, 523UZ, 1009UZ, 1049UZ, 2018UZ, 3027UZ, 4099UZ, 7789UZ, 10007UZ, 20495UZ, 49157UZ, 65537UZ, 98317UZ});

    std::println("");
    std::println("float input, forward only (the audio transform)");
    for (const std::size_t N : {2048UZ, 8192UZ}) {
        sweep<float>(N);
    }

    // The four-step split against SimdFFT, the engine Auto runs on one thread at each of these lengths. Read the split
    // against the SimdFFT row. SimdFFT is single-threaded by construction, and its row is printed once per length. Run
    // this section on all the performance cores, not on one of them. The section measures the effect of the thread
    // count, and pinning it to one core would measure the pin.
    std::println("");
    std::println("the four-step split against SimdFFT, complex<float> forward ({} hardware threads)", std::thread::hardware_concurrency());
    for (std::size_t exponent = 14UZ; exponent <= 22UZ; ++exponent) {
        const std::size_t N         = 1UZ << exponent;
        const auto        signal    = broadbandSignal<std::complex<float>>(N);
        const auto        reference = doubleReference(signal);
        benchBackend("simd", signal, reference, gr::algorithm::FftBackend::Simd);
        for (const std::size_t threads : {1UZ, 2UZ, 4UZ, 6UZ}) {
            benchBackend("four-step", signal, reference, gr::algorithm::FftBackend::FourStep, threads);
        }
    }

    // The same decomposition through PocketFFT's batched driver and through loops over SimdFFT, at 2^20 and 2^22.
    std::println("");
    std::println("the four-step on PocketFFT's batched driver against the same split built on SimdFFT loops");
    for (const std::size_t N : {1UZ << 20UZ, 1UZ << 22UZ}) {
        const auto signal    = broadbandSignal<std::complex<float>>(N);
        const auto reference = doubleReference(signal);
        benchBackend("simd flat", signal, reference, gr::algorithm::FftBackend::Simd);
        for (const std::size_t threads : {1UZ, 2UZ, 4UZ, 6UZ}) {
            benchBackend("four-step", signal, reference, gr::algorithm::FftBackend::FourStep, threads);
            benchSimdFourStep(signal, reference, threads);
        }
    }

    std::println("");
    std::println("the split's own passes, so the twiddle pass and the folded transpose are costed rather than argued");
    for (const std::size_t N : {1UZ << 20UZ, 1UZ << 22UZ}) {
        for (const std::size_t threads : {1UZ, 4UZ}) {
            benchFourStepParts(N, threads);
        }
    }
    return 0;
}
