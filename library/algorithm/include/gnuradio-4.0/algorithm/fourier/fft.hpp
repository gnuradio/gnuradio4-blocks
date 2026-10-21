#ifndef GNURADIO_ALGORITHM_FFT_HPP
#define GNURADIO_ALGORITHM_FFT_HPP

#include <algorithm>
#include <bit>
#include <complex>
#include <execution>
#include <numbers>
#include <ranges>
#include <stdexcept>
#include <thread>
#include <type_traits>
#include <utility>
#include <vector>

#include <gnuradio-4.0/algorithm/fourier/SimdFFT.hpp>
#include <gnuradio-4.0/meta/utils.hpp>

// Vendored, BSD-3-Clause. The include path is the header's installed path. A consumer with the include root on its
// path resolves it without a second include directory from the CMake target.
// The header's thread pool is a process-wide static. This macro makes the header register fork handlers. They stop the
// workers before a fork and start them again in the parent and in the child. A forked child can then run a threaded
// transform.
#if (defined(__unix__) || defined(__APPLE__)) && !defined(__EMSCRIPTEN__) && !defined(POCKETFFT_PTHREADS)
#define POCKETFFT_PTHREADS
#endif
// A consumer that finds the header through an include path that is not a system path compiles it as its own code.
// The header then raises -Wshadow and -Wdouble-promotion under the project's warning set.
// GCC cannot prove the plan pointers non-null once the mixed-radix codelets are inlined. -Wnull-dereference then
// fires inside the header. An optimizer pass raises the diagnostic after inlining, and a system include does not
// suppress it.
#if defined(__GNUC__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wdouble-promotion"
#if !defined(__clang__)
#pragma GCC diagnostic ignored "-Wnull-dereference"
#endif
#endif
#include <gnuradio-4.0/third_party/pocketfft/pocketfft_hdronly.h>
#if defined(__GNUC__)
#pragma GCC diagnostic pop
#endif

namespace gr::algorithm {

/// @brief transform engine behind FFT<>::compute()
enum class FftBackend {
    Native,    ///< this file's radix-2 kernels, and Bluestein for the other lengths
    Simd,      ///< SimdFFT.hpp for the sizes it accepts, Native for the rest
    PocketFFT, ///< the vendored PocketFFT, a per-instance FFTPACK/Bluestein plan for any length
    FourStep,  ///< one long power-of-two complex transform in two batches through PocketFFT's multi-dimensional driver, on `threads` threads
    Auto       ///< chosen per length from measurements, see autoBackend()
};

namespace detail {

template<gr::meta::complex_like T>
constexpr T complex_mult(T a, T b) {
    return a * b; // SIMD optimisation candidate
}

// the fixed-size kernels carry their own twiddles and therefore need the direction; the generic one reads the (already direction-correct) table
template<gr::meta::complex_like C, std::size_t N = std::dynamic_extent, bool inverse = false>
constexpr void fft_stage_kernel(C* data, const C* twiddles, std::size_t halfsize = 0UZ) {
    using ValueType = typename C::value_type;

    if constexpr (N == 2UZ) {
        const C a = data[0];
        const C b = data[1];

        data[0] = a + b;
        data[1] = a - b;
    } else if constexpr (N == 4UZ) {
        constexpr ValueType        wImag     = inverse ? ValueType(1) : ValueType(-1);
        constexpr std::array<C, 2> twiddles4 = {
            C{ValueType(1), ValueType(0)}, // W_4^0
            C{ValueType(0), wImag},        // W_4^1 == -j (forward)
        };
        const C a = data[0];
        const C b = data[1];
        const C c = complex_mult(data[2], twiddles4[0]); // W_4^0 == 1
        const C d = complex_mult(data[3], twiddles4[1]); // W_4^1 == -j

        data[0] = a + c;
        data[1] = b + d;
        data[2] = a - c;
        data[3] = b - d;
    } else if constexpr (N == 8UZ) {
        constexpr auto inv_sqrt2 = static_cast<ValueType>(1 / std::numbers::sqrt2_v<ValueType>); // std::sqrt(0.5f)

        constexpr ValueType        wImag     = inverse ? inv_sqrt2 : -inv_sqrt2;
        constexpr ValueType        wImagUnit = inverse ? ValueType(1) : ValueType(-1);
        constexpr std::array<C, 4> twiddles8 = {//
            C{ValueType(1), ValueType(0)},      //
            C{inv_sqrt2, wImag},                //
            C{ValueType(0), wImagUnit},         //
            C{-inv_sqrt2, wImag}};

        const C a0 = data[0];
        const C a1 = data[1];
        const C a2 = data[2];
        const C a3 = data[3];

        const C b0 = complex_mult(data[4], twiddles8[0]);
        const C b1 = complex_mult(data[5], twiddles8[1]);
        const C b2 = complex_mult(data[6], twiddles8[2]);
        const C b3 = complex_mult(data[7], twiddles8[3]);

        data[0] = a0 + b0;
        data[1] = a1 + b1;
        data[2] = a2 + b2;
        data[3] = a3 + b3;

        data[4] = a0 - b0;
        data[5] = a1 - b1;
        data[6] = a2 - b2;
        data[7] = a3 - b3;
    } else if constexpr (N == std::dynamic_extent) {
        for (std::size_t j = 0; j < halfsize; ++j) {
            const auto temp    = complex_mult(data[j + halfsize], twiddles[j]);
            data[j + halfsize] = data[j] - temp;
            data[j] += temp;
        }
    } else {
        static_assert(gr::meta::always_false<C>, "unimplemented power N of 2, 4, or 8");
    }
}

} // namespace detail

/**
 * @brief radix-2 / Bluestein DFT with optional SimdFFT and PocketFFT backends, for any transform length.
 *
 * Transforms are unnormalized, matching SimdFFT's convention: feeding the output of a
 * Direction::Forward instance into a Direction::Backward one of the same length returns N times the
 * input. Real-valued input (R2C) is forward-only. Every backend keeps that convention and produces
 * the same full N-bin spectrum. `backend` changes the cost of compute(). It changes the result by each
 * engine's rounding. The Native path's twiddle recurrence adds an error that grows with the length.
 *
 * compute() mutates per-instance scratch (SimdFFT state, aligned buffers, Bluestein caches, the
 * PocketFFT plan, the four-step buffers, the per-length plan cache): hold one instance per thread.
 * `threads` above one lets one compute() split a long transform over that many threads. It does not
 * make an instance shareable. The first such call starts PocketFFT's process-wide pool of one worker per
 * logical CPU. The pool lasts until the process exits. A fork stops the pool and starts it again in the parent
 * and in the child. Do not fork while another thread is inside a threaded compute().
 */
template<typename TInput, gr::meta::complex_like TOutput = std::conditional_t<gr::meta::complex_like<TInput>, TInput, std::complex<TInput>>, Direction TDirection = Direction::Forward>
requires((gr::meta::complex_like<TInput> || std::floating_point<TInput>))
struct FFT {
    using ValueType = typename TOutput::value_type;

    std::vector<std::vector<TOutput>>                     stageTwiddles{};
    std::vector<TOutput, gr::allocator::Aligned<TOutput>> bluesteinExpTable{};
    std::vector<TOutput, gr::allocator::Aligned<TOutput>> bluesteinChirpFFT{};
    std::vector<std::size_t>                              bitReverseTable{};
    std::size_t                                           fftSize{0};

    constexpr static Transform kTransform = gr::meta::complex_like<TInput> ? Transform::Complex : Transform::Real;
    constexpr static Direction kDirection = TDirection;
    constexpr static bool      kInverse   = kDirection == Direction::Backward;
    static_assert(kTransform == Transform::Complex || kDirection == Direction::Forward, "real-valued input (R2C) supports the forward transform only");

    SimdFFT<ValueType, kTransform>                            simdFFT{};
    std::vector<ValueType, gr::allocator::Aligned<ValueType>> alignedInputBuffer{};
    std::vector<ValueType, gr::allocator::Aligned<ValueType>> alignedOutputBuffer{};

    /// The one-dimensional PocketFFT plan for this instance's current length. It is built through
    /// pocketfft::detail directly and not through the header's c2c()/r2c() drivers. The plan belongs to this
    /// instance, and the header's global plan cache and its mutex are not used.
    using PocketPlan = std::conditional_t<kTransform == Transform::Complex, pocketfft::detail::pocketfft_c<ValueType>, pocketfft::detail::pocketfft_r<ValueType>>;
    std::unique_ptr<PocketPlan> pocketPlan{};
    std::size_t                 pocketPlanBuilds{0UZ}; ///< plans built since construction, one per distinct length and not one per compute()

    /// The four-step split's per-length state. The work buffer holds the first batch's result. The table holds the
    /// inter-pass twiddles. Each has N entries. Both are saved and restored with the rest of the per-length state.
    std::vector<TOutput, gr::allocator::Aligned<TOutput>> fourStepWork{};
    std::vector<TOutput, gr::allocator::Aligned<TOutput>> fourStepTwiddles{};

    FftBackend backend{FftBackend::Auto};
    /// Deprecated, set backend instead. It is read only when backend is Auto. There, false selects the Native
    /// path at every length and thread count.
    bool useSimdFFT{true};

    /// threads one compute() may use. At one, the whole transform runs on the calling thread. Above one, at a
    /// length the four-step accepts, Auto splits the transform over that many threads. The count is clamped to
    /// the number of logical CPUs std::thread::hardware_concurrency() reports. The instance stays
    /// single-consumer in both cases. The extra threads are workers of PocketFFT's process-wide pool. They touch
    /// only this instance's buffers during the call.
    std::size_t threads{1UZ};

    void initAll() {
        precomputeTwiddleFactors();
        precomputeBitReversal();
    }

    // returns 'out' itself: a reference to an lvalue argument, the moved container by value for an rvalue argument
    decltype(auto) compute(const std::ranges::input_range auto& in, std::ranges::output_range<TOutput> auto&& out) {
        computeInto(in, out);
        if constexpr (std::is_lvalue_reference_v<decltype(out)>) {
            return (out);
        } else {
            return std::remove_cvref_t<decltype(out)>(std::move(out));
        }
    }

    auto compute(const std::ranges::input_range auto& in) {
        using input_container_t = std::remove_cvref_t<decltype(in)>;
        using output_alloc_t    = gr::allocator::detail::deduce_output_allocator_t<input_container_t, TOutput>;
        return compute(in, std::vector<TOutput, output_alloc_t>(in.size()));
    }

    /// the backend compute() runs for this length, a backend that accepts the length and not Auto
    [[nodiscard]] FftBackend resolveBackend(std::size_t n) const noexcept {
        FftBackend selected = backend;
        if (selected == FftBackend::Auto) {
            const bool splitPays = useSimdFFT && effectiveThreads() > 1UZ && n >= kFourStepMinSize && canFourStep(n);
            selected             = splitPays ? FftBackend::FourStep : (useSimdFFT ? autoBackend(n) : FftBackend::Native);
        }
        if (selected == FftBackend::FourStep && !canFourStep(n)) {
            selected = useSimdFFT ? autoBackend(n) : FftBackend::Native;
        }
        if (selected == FftBackend::Simd && !SimdFFT<ValueType, kTransform>::canProcessSize(n, Order::Ordered)) {
            selected = FftBackend::Native;
        }
        return selected;
    }

    /// threads one compute() uses, the request clamped between one and the number of logical CPUs.
    /// The count is read once and kept. hardware_concurrency() costs more than a short transform.
    [[nodiscard]] std::size_t effectiveThreads() const noexcept {
        static const std::size_t available = [] {
            const unsigned int reported = std::thread::hardware_concurrency();
            return reported == 0U ? 1UZ : static_cast<std::size_t>(reported);
        }();
        return std::clamp(threads, 1UZ, available);
    }

    /// The shortest length at which Auto takes the split. It is a policy bound and not a capability bound. The split
    /// reads and writes extra records, and the two batches must recover that cost. bm_fft_backends measures the
    /// crossover per thread count. A backend selected by hand runs at any length canFourStep() accepts. That is how
    /// the measurement reaches below the bound.
    ///
    /// The bound is one number for every thread count and both precisions. It was measured for complex<float> on six
    /// performance cores of an i7-12700H, one core per thread. The measured split against SimdFFT on the calling
    /// thread:
    ///   * four threads are faster from 2^14 upward;
    ///   * two threads are faster at 2^16 by 1.06 to 1.29 times in two runs;
    ///   * two threads at 2^17 and 2^18 range from 12 percent slower to 1.45 times faster, and the sign changed
    ///     between runs.
    static constexpr std::size_t kFourStepMinSize = 1UZ << 16UZ;

    /// Lengths the four-step can run at. The transform is complex and the length a power of two of at least 4.
    /// Both factors are then powers of two, and neither batch needs a mixed-radix fallback.
    [[nodiscard]] static constexpr bool canFourStep(std::size_t n) noexcept { return kTransform == Transform::Complex && n >= 4UZ && std::has_single_bit(n); }

    /// N = N1 * N2 with N1 <= N2 and both powers of two, the two batches as square as the length allows
    [[nodiscard]] static constexpr std::pair<std::size_t, std::size_t> fourStepFactors(std::size_t n) noexcept {
        const std::size_t n1 = 1UZ << (static_cast<std::size_t>(std::countr_zero(n)) / 2UZ);
        return {n1, n / n1};
    }

    /// True when the square of n's largest prime factor exceeds n. A mixed-radix engine then falls back on a direct
    /// O(p^2) codelet for that factor and cannot split it further.
    [[nodiscard]] static constexpr bool largestPrimeFactorExceedsRoot(std::size_t n) noexcept {
        std::size_t remaining = n;
        while (remaining > 1UZ && (remaining & 1UZ) == 0UZ) {
            remaining >>= 1UZ;
        }
        for (std::size_t divisor = 3UZ; divisor * divisor <= remaining; divisor += 2UZ) {
            while (remaining % divisor == 0UZ) {
                remaining /= divisor;
            }
        }
        return remaining * remaining > n;
    }

    /// The backend Auto picks. The figures are complex<float> forward on one performance core of an i7-12700H, in ns
    /// per butterfly unit (the time over N log2 N). The first two rules rest on bm_fft_backends, built at -O2 with
    /// -march=native. The third rests on a sweep built at -O3 without -march=native.
    ///
    ///   * SimdFFT at every length it accepts. bm_fft_backends measures it from N = 512 to 2^22 at 0.31 to 0.54,
    ///     against 1.02 to 1.41 for PocketFFT. Its error is at the precision's limit.
    ///   * PocketFFT at the composite lengths SimdFFT rejects. It takes 1.09 at N = 1250 and 1.02 at N = 105. The
    ///     radix-2/Bluestein path takes 3.33 and 3.30. PocketFFT's error is at float epsilon. The error of the
    ///     radix-2/Bluestein path grows with the transform length, because its twiddles come from a recurrence.
    ///   * Where the largest prime factor exceeds sqrt(n), both engines run Bluestein. The native path pads to
    ///     bit_ceil(2n + 1) and transforms with SimdFFT. Its cost follows the ratio of the padded length to n:
    ///     about 3 near a ratio of 2, 4.4 near 3.2 and 5.5 near 3.9. PocketFFT pads to a smooth length near 2n. Its
    ///     cost lies between 2.6 and 6.3 and follows the factors of that length. The native path is taken when the
    ///     ratio is below three. Over 52 lengths from 131 to 124529, with the engines alternating in one process and
    ///     the best of nine rounds kept, the native path was faster at 20 of 26 lengths below the ratio, by up to
    ///     1.69 times. PocketFFT was faster at 18 of 26 lengths at or above it, by up to 2.1 times. With
    ///     -march=native the counts are 18 and 16. At -O2 PocketFFT's Bluestein path takes up to 2.5 times as long,
    ///     and the native path was faster at 24 of the 26 lengths at or above the ratio.
    [[nodiscard]] static FftBackend autoBackend(std::size_t n) noexcept {
        if (SimdFFT<ValueType, kTransform>::canProcessSize(n, Order::Ordered)) {
            return FftBackend::Simd;
        }
        return largestPrimeFactorExceedsRoot(n) && std::bit_ceil(2UZ * n + 1UZ) < 3UZ * n ? FftBackend::Native : FftBackend::PocketFFT;
    }

    template<typename InRange, typename OutRange>
    bool trySimdFFT(const InRange& in, OutRange&& out) {
        if (simdFFT.size() != in.size()) {
            simdFFT.resize(in.size());
        }
        if constexpr (kTransform == Transform::Complex) {
            return trySimdFFT_C2C(in, out, in.size());
        } else {
            return trySimdFFT_R2C(in, out, in.size());
        }
    }

    /// PocketFFT works in place on a contiguous buffer, and the output range is that buffer. The function returns
    /// false for a range that cannot serve as one. compute() then takes the Native path.
    template<typename InRange, typename OutRange>
    bool tryPocketFFT([[maybe_unused]] const InRange& in, [[maybe_unused]] OutRange&& out) {
        if constexpr (!requires {
                          { out.data() } -> std::convertible_to<TOutput*>;
                      }) {
            return false;
        } else {
            const std::size_t N = in.size();
            ensurePocketPlan(N);
            if constexpr (kTransform == Transform::Complex) {
                pocketFFT_C2C(in, out);
            } else {
                pocketFFT_R2C(in, out, N);
            }
            return true;
        }
    }

    /// The four-step split of one long transform. Like the PocketFFT backend, it writes through the caller's
    /// output range. The function returns false for a range that cannot serve, and compute() falls back.
    template<typename InRange, typename OutRange>
    bool tryFourStep([[maybe_unused]] const InRange& in, [[maybe_unused]] OutRange&& out) {
        if constexpr (kTransform != Transform::Complex || !requires {
                          { out.data() } -> std::convertible_to<TOutput*>;
                      }) {
            return false;
        } else {
            fourStep_C2C(in, out);
            return true;
        }
    }

private:
    void computeInto(const std::ranges::input_range auto& in, std::ranges::output_range<TOutput> auto& out) {
        if constexpr (requires { out.resize(in.size()); }) {
            if (out.size() != in.size()) {
                out.resize(in.size());
            }
        }

        const auto size = in.size();
        if (size == 0) {
            return;
        }

        selectPlan(size);

        switch (resolveBackend(size)) {
        case FftBackend::Simd:
            if (trySimdFFT(in, out)) { // the size was checked by resolveBackend()
                return;
            }
            break;
        case FftBackend::PocketFFT:
            if (tryPocketFFT(in, out)) { // false only for an output range this backend cannot address
                return;
            }
            break;
        case FftBackend::FourStep:
            if (tryFourStep(in, out)) { // false only for an output range this backend cannot address
                return;
            }
            break;
        default: break;
        }

        // fallback to original implementation
        std::ranges::transform(in, out.begin(), [](auto v) {
            if constexpr (std::floating_point<TInput>) {
                return TOutput(ValueType(v), 0);
            } else {
                return static_cast<TOutput>(v);
            }
        });

        if (std::has_single_bit(size)) {
            ensureRadix2Tables();
            transformRadix2(out);
        } else {
            ensureBluesteinTable(size);
            transformBluestein(out);
        }
    }

    /// Unnormalized, with a plan scale factor of one. Forward and backward differ only in the flag.
    static constexpr bool kPocketForward = !kInverse;

    void ensurePocketPlan(std::size_t N) {
        if (!pocketPlan || pocketPlan->length() != N) {
            pocketPlan = std::make_unique<PocketPlan>(N);
            ++pocketPlanBuilds;
        }
    }

    void pocketFFT_C2C(const auto& in, auto&& out) {
        using PocketComplex = pocketfft::detail::cmplx<ValueType>;
        static_assert(sizeof(PocketComplex) == sizeof(TOutput) && alignof(PocketComplex) == alignof(TOutput), "PocketFFT's cmplx<T> must have std::complex<T>'s layout to transform the output range in place");

        std::ranges::transform(in, out.begin(), [](const auto& v) { return static_cast<TOutput>(v); });
        pocketPlan->exec(reinterpret_cast<PocketComplex*>(out.data()), ValueType(1), kPocketForward);
    }

    /// R2C is forward-only and produces the same full N-bin spectrum as the other two backends. PocketFFT's
    /// half-complex result [DC, Re(1), Im(1), …, Nyquist] is unpacked and mirrored by Hermitian symmetry.
    void pocketFFT_R2C(const auto& in, auto&& out, std::size_t N) {
        if (alignedOutputBuffer.size() != N) { // the SimdFFT path's scratch, N reals, written before it is read
            alignedOutputBuffer.resize(N);
        }
        std::ranges::transform(in, alignedOutputBuffer.begin(), [](const auto& v) { return static_cast<ValueType>(v); });
        pocketPlan->exec(alignedOutputBuffer.data(), ValueType(1), true);

        out[0]            = TOutput(alignedOutputBuffer[0], ValueType(0)); // DC has no twin
        std::size_t index = 1UZ;
        std::size_t bin   = 1UZ;
        for (; index + 1UZ < N; index += 2UZ, ++bin) {
            out[bin]     = TOutput(alignedOutputBuffer[index], alignedOutputBuffer[index + 1UZ]);
            out[N - bin] = std::conj(out[bin]);
        }
        if (index < N) { // even N, where the Nyquist bin is real and not mirrored, like DC
            out[bin] = TOutput(alignedOutputBuffer[index], ValueType(0));
        }
    }

    // ------------------------------------------------------------- the four-step split of one long transform
    //
    // N = N1 * N2. The record is read as an N1 x N2 row-major array x[i1][i2] = in[i1*N2 + i2]. With n = i1*N2 + i2
    // and k = k2*N1 + k1 the definition factors exactly, because W_N^(i1*N2*k2*N1) is one.
    //
    //   X[k2*N1 + k1] = sum over i2 of W_N^(i2*k1) W_N2^(i2*k2) ( sum over i1 of W_N1^(i1*k1) x[i1][i2] )
    //
    // The transform is then three steps. The N2 columns are one batch along axis 0. A pointwise multiply by
    // W_N^(i1*i2) follows. The N1 rows are a second batch along axis 1. Both batches go through PocketFFT's
    // multi-dimensional driver. That driver gathers VLEN neighboring transforms into one vector register and
    // threads the work. One long transform gets neither benefit on its own.
    //
    // The second batch leaves Z[k1][k2]. The natural bin order k2*N1 + k1 needs it transposed. The transpose is
    // folded into that batch's output strides and costs no pass of its own. The batch dimension is k1, with an
    // output stride of one element. The driver's vector lanes therefore still write neighboring addresses, and
    // the record is written exactly once. A separate transpose pass would read and write it again.
    //
    // Memory traffic per transform is counted in records, 32 MiB each at N = 2^22 for complex<float>. The first
    // batch reads the input and writes the work buffer. The twiddle pass reads the work buffer and the table and
    // writes the work buffer. The second batch reads the work buffer and writes the output. That is six records,
    // against two for one flat transform. This traffic is the cost of using the batched driver. The split is
    // therefore taken only from kFourStepMinSize upward.
    //
    // Each call also allocates. pocketfft::c2c builds two plans per call, because the header's plan cache is compiled
    // out, and its vectorized passes allocate scratch. A count of operator new, malloc and aligned_alloc calls,
    // averaged over ten steady-state calls of complex<float> on one thread, gives 190 allocations totaling 1.08 MB
    // per call at N = 2^16 and 574 totaling 16.9 MB at N = 2^20. Four threads give 232 and 616 per call. SimdFFT
    // allocates nothing.

    void ensureFourStepTables(std::size_t N) {
        if (fourStepTwiddles.size() == N) {
            return;
        }
        const auto [n1, n2] = fourStepFactors(N);
        fourStepWork.resize(N);
        fourStepTwiddles.resize(N);

        // W_N^(i1*i2), built in double and rounded once. The recurrence in precomputeTwiddleFactors() is not used.
        // Its error grows with the table's length, and this table is as long as the record. The product i1*i2
        // stays below N. Splitting it as a*N2 + b makes every entry the product of two exactly rounded reads. The
        // table then costs N1 + N2 trigonometric calls instead of N.
        const double                      turns = kInverse ? 2. : -2.;
        std::vector<std::complex<double>> coarse(n1); // W_N^(a*N2), equal to W_N1^a
        std::vector<std::complex<double>> fine(n2);   // W_N^b
        for (std::size_t a = 0UZ; a < n1; ++a) {
            coarse[a] = std::polar(1., turns * std::numbers::pi * static_cast<double>(a) / static_cast<double>(n1));
        }
        for (std::size_t b = 0UZ; b < n2; ++b) {
            fine[b] = std::polar(1., turns * std::numbers::pi * static_cast<double>(b) / static_cast<double>(N));
        }

        const std::size_t shift = static_cast<std::size_t>(std::countr_zero(n2));
        const std::size_t mask  = n2 - 1UZ;
        for (std::size_t i1 = 0UZ; i1 < n1; ++i1) {
            for (std::size_t i2 = 0UZ; i2 < n2; ++i2) {
                const std::size_t          product = i1 * i2;
                const std::complex<double> w       = coarse[product >> shift] * fine[product & mask];
                fourStepTwiddles[i1 * n2 + i2]     = TOutput(static_cast<ValueType>(w.real()), static_cast<ValueType>(w.imag()));
            }
        }
    }

    /// The inter-pass multiply, spread over the same threads the two batches use. PocketFFT's threading::thread_map
    /// is a plain parallel map over its own pool. The split starts no threads beyond those the batches start.
    void multiplyFourStepTwiddles(std::size_t nThreads) {
        TOutput* const       data  = fourStepWork.data();
        const TOutput* const table = fourStepTwiddles.data();
        const std::size_t    total = fourStepWork.size();
        pocketfft::detail::threading::thread_map(nThreads, [data, table, total] {
            const std::size_t index = pocketfft::detail::threading::thread_id();
            const std::size_t count = pocketfft::detail::threading::num_threads();
            const std::size_t chunk = (total + count - 1UZ) / count;
            const std::size_t begin = std::min(total, index * chunk);
            const std::size_t end   = std::min(total, begin + chunk);
            for (std::size_t i = begin; i < end; ++i) {
                data[i] = detail::complex_mult(data[i], table[i]);
            }
        });
    }

    void fourStep_C2C(const auto& in, auto&& out) {
        using PocketComplex = std::complex<ValueType>;
        static_assert(sizeof(PocketComplex) == sizeof(TOutput) && alignof(PocketComplex) == alignof(TOutput), "PocketFFT's driver takes std::complex<T>, which must have TOutput's layout to transform the caller's ranges through");

        const std::size_t N = in.size();
        ensureFourStepTables(N);
        const auto [n1, n2]        = fourStepFactors(N);
        const std::size_t nThreads = effectiveThreads();

        const pocketfft::shape_t  shape{n1, n2};
        const pocketfft::stride_t rowMajor{static_cast<std::ptrdiff_t>(n2 * sizeof(TOutput)), static_cast<std::ptrdiff_t>(sizeof(TOutput))};
        const pocketfft::stride_t transposed{static_cast<std::ptrdiff_t>(sizeof(TOutput)), static_cast<std::ptrdiff_t>(n1 * sizeof(TOutput))};

        // An input that is already a contiguous range of TOutput is the first batch's source as it is. Any other
        // input is first converted into the output range, and the second batch overwrites it.
        const PocketComplex* source = nullptr;
        if constexpr (requires {
                          { in.data() } -> std::convertible_to<const TOutput*>;
                      }) {
            if constexpr (std::is_same_v<std::remove_cvref_t<std::ranges::range_value_t<std::remove_cvref_t<decltype(in)>>>, TOutput>) {
                source = reinterpret_cast<const PocketComplex*>(in.data());
            }
        }
        if (source == nullptr) {
            std::ranges::transform(in, out.begin(), [](const auto& v) { return static_cast<TOutput>(v); });
            source = reinterpret_cast<const PocketComplex*>(out.data());
        }

        pocketfft::c2c(shape, rowMajor, rowMajor, pocketfft::shape_t{0UZ}, kPocketForward, source, reinterpret_cast<PocketComplex*>(fourStepWork.data()), ValueType(1), nThreads);
        multiplyFourStepTwiddles(nThreads);
        pocketfft::c2c(shape, rowMajor, transposed, pocketfft::shape_t{1UZ}, kPocketForward, reinterpret_cast<const PocketComplex*>(fourStepWork.data()), reinterpret_cast<PocketComplex*>(out.data()), ValueType(1), nThreads);
    }

    bool trySimdFFT_C2C(const auto& in, auto&& out, std::size_t N) {
        using InputValueType        = typename std::remove_cvref_t<decltype(in)>::value_type::value_type;
        const std::size_t nElements = 2UZ * N;
        static_assert(sizeof(std::complex<ValueType>) == 2 * sizeof(ValueType), "SimdFFT backend expects interleaved scalars; complex<T> must be 2*T bytes.");

        if constexpr (std::is_same_v<InputValueType, ValueType>) {
            if (gr::allocator::isAligned(in.data(), 64UZ) && gr::allocator::isAligned(out.data(), 64UZ)) { // zero-copy path (same precision and aligned)
                std::span<const ValueType> inputSpan(reinterpret_cast<const ValueType*>(in.data()), nElements);
                std::span<ValueType>       outputSpan(reinterpret_cast<ValueType*>(out.data()), nElements);
                simdFFT.template transform<kDirection, Order::Ordered>(inputSpan, outputSpan);
                return true;
            } // future else: copy and avoid reinterpret_cast (slightly UB)
        }

        // buffered path for non-aligned or different precision
        if (alignedInputBuffer.size() != nElements) {
            alignedInputBuffer.resize(nElements);
        }
        if (alignedOutputBuffer.size() != nElements) {
            alignedOutputBuffer.resize(nElements);
        }

        // copy input with type conversion if needed
        const auto* inputPtr = reinterpret_cast<const InputValueType*>(in.data());
        for (std::size_t i = 0; i < nElements; ++i) {
            alignedInputBuffer[i] = static_cast<ValueType>(inputPtr[i]);
        }

        simdFFT.template transform<kDirection, Order::Ordered>(alignedInputBuffer, alignedOutputBuffer);

        // copy output with type conversion if needed
        auto* outputPtr = reinterpret_cast<ValueType*>(out.data());
        std::memcpy(outputPtr, alignedOutputBuffer.data(), nElements * sizeof(ValueType));

        return true;
    }

    bool trySimdFFT_R2C(const auto& in, auto&& out, std::size_t N) {
        using InputValueType = typename std::remove_cvref_t<decltype(in)>::value_type;
        static_assert(std::is_trivially_copyable_v<InputValueType>);
        static_assert(std::is_trivially_copyable_v<ValueType>);
        assert(std::size(in) >= N);
        assert(std::size(out) >= N);

        if (alignedOutputBuffer.size() != N) {
            alignedOutputBuffer.resize(N); // packed real: [DC, Nyq, re1, im1, ...]
        }

        if constexpr (std::is_same_v<InputValueType, ValueType>) {
            if (gr::allocator::isAligned(in.data(), 64UZ)) { // input is cacheline-aligned
                simdFFT.template transform<kDirection, Order::Ordered>(std::span<const ValueType>{in.data(), N}, alignedOutputBuffer);
            } else { // not cacheline-aligned -> copy to aligned scratch
                if (alignedInputBuffer.size() != N) {
                    alignedInputBuffer.resize(N);
                }
                std::memcpy(alignedInputBuffer.data(), in.data(), N * sizeof(ValueType));
                simdFFT.template transform<kDirection, Order::Ordered>(std::span<const ValueType>{alignedInputBuffer.data(), N}, alignedOutputBuffer);
            }
        } else { // type conversion needed
            if (alignedInputBuffer.size() != N) {
                alignedInputBuffer.resize(N);
            }
            for (std::size_t i = 0; i < N; ++i) { // element-wise conversion (memcpy is invalid across types)
                alignedInputBuffer[i] = static_cast<ValueType>(in[i]);
            }
            simdFFT.template transform<kDirection, Order::Ordered>(std::span<const ValueType>{alignedInputBuffer.data(), N}, alignedOutputBuffer);
        }

        // unpack to full spectrum: [DC, Nyquist, re1, im1, re2, im2, ...] → N complex values
        out[0] = TOutput(static_cast<ValueType>(alignedOutputBuffer[0]), 0); // DC component at bin 0
        for (std::size_t k = 1; k < N / 2; ++k) {                            // positive frequencies (bins 1 to N/2-1)
            out[k] = TOutput(static_cast<ValueType>(alignedOutputBuffer[2 * k]), static_cast<ValueType>(alignedOutputBuffer[2 * k + 1]));
        }
        out[N / 2] = TOutput(static_cast<ValueType>(alignedOutputBuffer[1]), 0); // nyquist component at bin N/2

        for (std::size_t k = N / 2 + 1; k < N; ++k) { // negative frequencies (bins N/2+1 to N-1) - Hermitian symmetry -> complex conjugates
            const std::size_t mirrorIdx = N - k;
            out[k]                      = TOutput(static_cast<ValueType>(alignedOutputBuffer[2 * mirrorIdx]), -static_cast<ValueType>(alignedOutputBuffer[2 * mirrorIdx + 1])); // Conjugate
        }

        return true;
    }

    void transformRadix2(std::ranges::input_range auto& inPlace) {
        const std::size_t N = inPlace.size();
        if (!std::has_single_bit(N)) {
            throw std::invalid_argument(std::format("Input data must be power-of-two, input size: {}", inPlace.size()));
        }

        for (std::size_t i = 0UZ; i < N; ++i) {
            const std::size_t j = bitReverseTable[i];
            if (j > i) {
                std::swap(inPlace[i], inPlace[j]);
            }
        }

        const std::size_t nStages = static_cast<std::size_t>(std::countr_zero(N));
        for (std::size_t stage = 0, size = 2; stage < nStages; size *= 2, ++stage) {
            const auto&       twiddles = stageTwiddles[stage];
            const std::size_t halfsize = size / 2;

            for (std::size_t i = 0; i < N; i += size) {
                TOutput* block = &inPlace[i];

                switch (size) { // optimised non-branching fft sub kernels
                case 2: detail::fft_stage_kernel<TOutput, 2, kInverse>(block, twiddles.data()); break;
                case 4: detail::fft_stage_kernel<TOutput, 4, kInverse>(block, twiddles.data()); break;
                case 8: detail::fft_stage_kernel<TOutput, 8, kInverse>(block, twiddles.data()); break;
                default: // generic case
                    detail::fft_stage_kernel<TOutput>(block, twiddles.data(), halfsize);
                    break;
                }
            }
        }
    }

    std::unique_ptr<FFT<TOutput, TOutput>>                fftCache;
    std::vector<TOutput, gr::allocator::Aligned<TOutput>> aCache{};
    std::vector<TOutput, gr::allocator::Aligned<TOutput>> bCache{};

    // chirp-z: X[k] = w[k] * (a (*) b)[k]; the cyclic convolution's inverse FFT is expressed as conj(FFT(conj(.)))/m
    void transformBluestein(std::ranges::input_range auto& inPlace) {
        const std::size_t n = inPlace.size();
        const std::size_t m = std::bit_ceil(2 * n + 1);

        using input_container_t = std::remove_cvref_t<decltype(inPlace)>;
        using output_alloc_t    = gr::allocator::detail::deduce_output_allocator_t<input_container_t, TOutput>;
        std::vector<TOutput, output_alloc_t> a(m);
        for (std::size_t i = 0; i < n; ++i) {
            a[i] = detail::complex_mult(inPlace[i], bluesteinExpTable[i]);
        }

        // convolve input with chirp function
        if (a.size() != bluesteinChirpFFT.size()) {
            throw std::domain_error("mismatched lengths for convolution");
        }
        if (!fftCache) {
            fftCache = std::make_unique<FFT<TOutput, TOutput>>();
        }

        fftCache->compute(a, aCache);

        std::transform(aCache.begin(), aCache.end(), bluesteinChirpFFT.begin(), aCache.begin(), [](const TOutput& x, const TOutput& y) { return std::conj(detail::complex_mult(x, y)); });
        fftCache->compute(aCache, bCache);

        const ValueType scale = ValueType(1) / ValueType(m);
        std::transform(bCache.begin(), std::next(bCache.begin(), static_cast<std::ptrdiff_t>(n)), bluesteinExpTable.begin(), inPlace.begin(), //
            [scale](const TOutput& v, const TOutput& w) { return detail::complex_mult(std::conj(v) * scale, w); });
    }

    // the radix-2 tables are read by transformRadix2() alone and the Bluestein table by transformBluestein()
    // alone, and both scale with fftSize (8 MiB of bit-reversal indices at N = 2^20), so build each only on
    // the path that reads it. initAll() builds the radix-2 pair together, so bitReverseTable.size() keys both.
    void ensureRadix2Tables() {
        if (bitReverseTable.size() != fftSize) {
            initAll();
        }
    }

    void ensureBluesteinTable(std::size_t n) {
        if (bluesteinExpTable.size() != n) {
            precomputeBluesteinTable(n);
        }
    }

    struct Plan {
        std::size_t                                               size{0UZ};
        SimdFFT<ValueType, kTransform>                            simdFFT{};
        std::vector<ValueType, gr::allocator::Aligned<ValueType>> alignedInputBuffer{};
        std::vector<ValueType, gr::allocator::Aligned<ValueType>> alignedOutputBuffer{};
        std::vector<std::vector<TOutput>>                         stageTwiddles{};
        std::vector<std::size_t>                                  bitReverseTable{};
        std::vector<TOutput, gr::allocator::Aligned<TOutput>>     bluesteinExpTable{};
        std::vector<TOutput, gr::allocator::Aligned<TOutput>>     bluesteinChirpFFT{};
        std::unique_ptr<FFT<TOutput, TOutput>>                    fftCache{};
        std::vector<TOutput, gr::allocator::Aligned<TOutput>>     aCache{};
        std::vector<TOutput, gr::allocator::Aligned<TOutput>>     bCache{};
        std::unique_ptr<PocketPlan>                               pocketPlan{};
        std::vector<TOutput, gr::allocator::Aligned<TOutput>>     fourStepWork{};
        std::vector<TOutput, gr::allocator::Aligned<TOutput>>     fourStepTwiddles{};
    };

    // a parked plan holds the whole per-length state, ~16 MB at N = 2^20 for complex<float>, so the count is
    // capped instead of grown: three parked plus the live one covers the interleavings that occur in practice,
    // and a further length evicts the least recently used plan rather than enlarging the footprint
    static constexpr std::size_t kMaxParkedPlans = 3UZ;
    std::vector<Plan>            parkedPlans{};

    void swapPlan(Plan& plan) {
        std::swap(simdFFT, plan.simdFFT);
        alignedInputBuffer.swap(plan.alignedInputBuffer);
        alignedOutputBuffer.swap(plan.alignedOutputBuffer);
        stageTwiddles.swap(plan.stageTwiddles);
        bitReverseTable.swap(plan.bitReverseTable);
        bluesteinExpTable.swap(plan.bluesteinExpTable);
        bluesteinChirpFFT.swap(plan.bluesteinChirpFFT);
        fftCache.swap(plan.fftCache);
        aCache.swap(plan.aCache);
        bCache.swap(plan.bCache);
        pocketPlan.swap(plan.pocketPlan);
        fourStepWork.swap(plan.fourStepWork);
        fourStepTwiddles.swap(plan.fourStepTwiddles);
    }

    // the slot that yields the incoming plan takes the outgoing one, so a cached length change is a plain swap
    void selectPlan(std::size_t size) {
        if (size == fftSize) {
            return;
        }
        if (fftSize == 0UZ) {
            fftSize = size;
            return;
        }

        const std::size_t nParked = parkedPlans.size();
        std::size_t       slot    = nParked;
        for (std::size_t i = 0UZ; i < nParked; ++i) {
            if (parkedPlans[i].size == size) {
                slot = i;
                break;
            }
        }
        if (slot == nParked) {
            if (nParked < kMaxParkedPlans) {
                parkedPlans.emplace_back();
            } else {
                slot = 0UZ;
            }
        }

        swapPlan(parkedPlans[slot]);
        parkedPlans[slot].size = fftSize;

        const auto parked = std::next(parkedPlans.begin(), static_cast<std::ptrdiff_t>(slot));
        std::rotate(parked, std::next(parked), parkedPlans.end());
        fftSize = size;
    }

    void precomputeTwiddleFactors() {
        stageTwiddles.clear();
        const auto minus2Pi = ValueType(kInverse ? 2 : -2) * std::numbers::pi_v<ValueType>;
        for (std::size_t size = 2UZ; size <= fftSize; size *= 2UZ) {
            const std::size_t    m{size / 2};
            const TOutput        w{std::exp(TOutput(0., minus2Pi / static_cast<ValueType>(size)))};
            std::vector<TOutput> twiddles;
            if (size == 2) {
                twiddles.push_back(TOutput{1.0, 0.0});
            } else if (size == 8) {
                twiddles.emplace_back(1.0, 0.0);                         // W_8^0
                twiddles.emplace_back(std::sqrt(0.5), -std::sqrt(0.5));  // W_8^1
                twiddles.emplace_back(0.0, -1.0);                        // W_8^2
                twiddles.emplace_back(-std::sqrt(0.5), -std::sqrt(0.5)); // W_8^3
                if constexpr (kInverse) {
                    std::ranges::transform(twiddles, twiddles.begin(), [](const TOutput& t) { return std::conj(t); });
                }
            } else {
                TOutput wk{1., 0.};
                for (std::size_t j = 0UZ; j < m; ++j) {
                    twiddles.push_back(wk);
                    wk *= w;
                }
            }
            stageTwiddles.push_back(std::move(twiddles));
        }
    }

    void precomputeBluesteinTable(std::size_t n) {
        bluesteinExpTable.resize(n);
        for (std::size_t i = 0; i < n; ++i) {
            const std::uintmax_t tmp   = static_cast<std::uintmax_t>(i) * i % (2 * n);
            const ValueType      angle = ValueType(kInverse ? 1 : -1) * std::numbers::pi_v<ValueType> * static_cast<ValueType>(tmp) / static_cast<ValueType>(n);
            bluesteinExpTable[i]       = std::polar<ValueType>(1.0, angle);
        }

        const std::size_t                                     m = std::bit_ceil(2 * n + 1);
        std::vector<TOutput, gr::allocator::Aligned<TOutput>> b(m);
        b[0] = bluesteinExpTable[0];
        for (std::size_t i = 1; i < n; ++i) {
            b[i] = b[m - i] = std::conj(bluesteinExpTable[i]);
        }

        FFT<TOutput, TOutput> fft{}; // always power-of-two
        bluesteinChirpFFT = fft.compute(b);
    }

    void precomputeBitReversal() {
        const std::size_t width = static_cast<std::size_t>(std::countr_zero(fftSize));
        bitReverseTable.resize(fftSize);
        for (std::size_t i = 0; i < fftSize; ++i) {
            std::size_t val = i, result = 0;
            for (std::size_t j = 0; j < width; ++j, val >>= 1) {
                result = (result << 1) | (val & 1U);
            }
            bitReverseTable[i] = result;
        }
    }
};

} // namespace gr::algorithm

#endif // GNURADIO_ALGORITHM_FFT_HPP
