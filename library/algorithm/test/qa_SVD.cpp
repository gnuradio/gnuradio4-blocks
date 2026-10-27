#include <boost/ut.hpp>

#include <gnuradio-4.0/Tensor.hpp>
#include <gnuradio-4.0/algorithm/math/SVD.hpp>
#include <gnuradio-4.0/algorithm/math/TensorMath.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <complex>
#include <functional>
#include <iomanip>
#include <limits>
#include <random>
#include <string>
#include <tuple>
#include <vector>

using namespace gr::math;

template<typename T, svd::Algorithm Algo>
struct SvdTestConfig {
    using value_type                          = T;
    static constexpr svd::Algorithm algorithm = Algo;

    static std::string name() {
        std::string typeName = gr::meta::complex_like<T> ? "complex" : (std::is_same_v<T, float> ? "float" : "double");
        std::string algoName = (Algo == svd::Algorithm::Auto) ? "Auto" : (Algo == svd::Algorithm::Jacobi ? "Jacobi" : "GolubReinsch");
        return typeName + "/" + algoName;
    }
};

// Real-valued type/algorithm combinations
using RealSvdConfigs = std::tuple<                       //
    SvdTestConfig<double, svd::Algorithm::Auto>,         //
    SvdTestConfig<double, svd::Algorithm::Jacobi>,       //
    SvdTestConfig<double, svd::Algorithm::GolubReinsch>, //
    SvdTestConfig<float, svd::Algorithm::Auto>,          //
    SvdTestConfig<float, svd::Algorithm::Jacobi>,        //
    SvdTestConfig<float, svd::Algorithm::GolubReinsch>>; //

// Complex-valued type/algorithm combinations (GolubReinsch falls back to Jacobi)
using ComplexSvdConfigs = std::tuple<                                  //
    SvdTestConfig<std::complex<double>, svd::Algorithm::Auto>,         //
    SvdTestConfig<std::complex<double>, svd::Algorithm::Jacobi>,       //
    SvdTestConfig<std::complex<double>, svd::Algorithm::GolubReinsch>, //
    SvdTestConfig<std::complex<float>, svd::Algorithm::Auto>,          //
    SvdTestConfig<std::complex<float>, svd::Algorithm::Jacobi>,        //
    SvdTestConfig<std::complex<float>, svd::Algorithm::GolubReinsch>>; //

// All configurations
using AllSvdConfigs = std::tuple<                                      //
    SvdTestConfig<double, svd::Algorithm::Auto>,                       //
    SvdTestConfig<double, svd::Algorithm::Jacobi>,                     //
    SvdTestConfig<double, svd::Algorithm::GolubReinsch>,               //
    SvdTestConfig<float, svd::Algorithm::Auto>,                        //
    SvdTestConfig<float, svd::Algorithm::Jacobi>,                      //
    SvdTestConfig<float, svd::Algorithm::GolubReinsch>,                //
    SvdTestConfig<std::complex<double>, svd::Algorithm::Auto>,         //
    SvdTestConfig<std::complex<double>, svd::Algorithm::Jacobi>,       //
    SvdTestConfig<std::complex<double>, svd::Algorithm::GolubReinsch>, //
    SvdTestConfig<std::complex<float>, svd::Algorithm::Auto>,          //
    SvdTestConfig<std::complex<float>, svd::Algorithm::Jacobi>,        //
    SvdTestConfig<std::complex<float>, svd::Algorithm::GolubReinsch>>; //

// ============================================================================
// Test Helpers
// ============================================================================

template<gr::TensorLike Tensor>
void randomize(Tensor& tensor, typename Tensor::value_type min = typename Tensor::value_type{-1}, typename Tensor::value_type max = typename Tensor::value_type{1}, unsigned seed = 42) {
    using T = typename Tensor::value_type;
    std::mt19937 gen(seed);

    if constexpr (std::is_floating_point_v<T>) {
        std::uniform_real_distribution<T> dis(min, max);
        for (auto& x : tensor) {
            x = dis(gen);
        }
    } else if constexpr (gr::meta::complex_like<T>) {
        using RealT = typename T::value_type;
        std::uniform_real_distribution<RealT> dis(std::real(min), std::real(max));
        for (auto& x : tensor) {
            x = T{dis(gen), dis(gen)};
        }
    } else {
        std::uniform_int_distribution<T> dis(min, max);
        for (auto& x : tensor) {
            x = dis(gen);
        }
    }
}

template<typename T>
[[nodiscard]] bool approx_equal(T a, T b, gr::meta::fundamental_base_value_type_t<T> epsilon = static_cast<gr::meta::fundamental_base_value_type_t<T>>(1e-6)) {
    if constexpr (gr::meta::complex_like<T>) {
        return std::abs(a - b) <= epsilon;
    } else {
        return std::abs(a - b) <= epsilon * std::max(T{1}, std::max(std::abs(a), std::abs(b)));
    }
}

template<gr::TensorLike TensorA, typename T = typename TensorA::value_type>
[[nodiscard]] bool tensors_approximately_equal(const TensorA& a, gr::TensorOf<T> auto const& b, gr::meta::fundamental_base_value_type_t<T> epsilon = static_cast<gr::meta::fundamental_base_value_type_t<T>>(1e-6)) {
    if (a.size() != b.size()) {
        return false;
    }
    for (std::size_t i = 0; i < a.size(); ++i) {
        if (!approx_equal(a.data()[i], b.data()[i], epsilon)) {
            return false;
        }
    }
    return true;
}

template<typename T>
[[nodiscard]] constexpr gr::meta::fundamental_base_value_type_t<T> defaultTolerance() {
    using RealT = gr::meta::fundamental_base_value_type_t<T>;
    return std::is_same_v<RealT, float> ? static_cast<RealT>(1e-4f) : static_cast<RealT>(1e-9f);
}

// Reconstruct A from U, S, V and check against original
template<typename T, typename RealT = gr::meta::fundamental_base_value_type_t<T>>
[[nodiscard]] bool verifyReconstruction(const gr::Tensor<T>& A, const gr::Tensor<T>& U, const gr::Tensor<RealT>& S, const gr::Tensor<T>& V, RealT tol) {
    const std::size_t m = A.extent(0);
    const std::size_t n = A.extent(1);
    const std::size_t k = S.size();

    if (k == 0) {
        return true; // empty SVD is trivially correct
    }

    // Build diagonal matrix from S
    gr::Tensor<T> S_diag({k, k});
    S_diag.fill(T{0});
    for (std::size_t i = 0; i < k; ++i) {
        S_diag[i, i] = static_cast<T>(S[i]);
    }

    // US = U * S_diag
    gr::Tensor<T> US({m, k});
    US.fill(T{0});
    gemm(US, U, S_diag);

    // A_recon = US * V^H (use transpose for real, conjTranspose for complex)
    gr::Tensor<T> Vh;
    if constexpr (gr::meta::complex_like<T>) {
        Vh = conjTranspose(V);
    } else {
        Vh = transpose(V);
    }
    gr::Tensor<T> A_recon({m, n});
    A_recon.fill(T{0});
    gemm(A_recon, US, Vh);

    // Compute relative Frobenius error for debugging
    RealT normDiff{0}, normA{0};
    for (std::size_t i = 0; i < A.size(); ++i) {
        auto diff = A.data()[i] - A_recon.data()[i];
        if constexpr (gr::meta::complex_like<T>) {
            normDiff += std::norm(diff);
            normA += std::norm(A.data()[i]);
        } else {
            normDiff += diff * diff;
            normA += A.data()[i] * A.data()[i];
        }
    }
    RealT relError = std::sqrt(normDiff) / std::sqrt(normA);
    bool  pass     = tensors_approximately_equal(A, A_recon, tol);
    if (!pass) {
        std::cerr << "  verifyReconstruction FAILED: relError=" << relError << ", tol=" << tol << "\n";
    }
    return pass;
}

// Verify matrix is orthogonal/unitary: Q^H * Q = I
template<typename T, typename RealT = gr::meta::fundamental_base_value_type_t<T>>
[[nodiscard]] bool verifyOrthogonal(const gr::Tensor<T>& Q, RealT tol) {
    const std::size_t cols = Q.extent(1);

    gr::Tensor<T> Qh = conjTranspose(Q);
    gr::Tensor<T> QhQ({cols, cols});
    QhQ.fill(T{0});
    gemm(QhQ, Qh, Q);

    for (std::size_t i = 0; i < cols; ++i) {
        for (std::size_t j = 0; j < cols; ++j) {
            T expected = (i == j) ? T{1} : T{0};
            if (!approx_equal(QhQ[i, j], expected, tol)) {
                return false;
            }
        }
    }
    return true;
}

// The measured errors of one decomposition, each divided by max(m, n) * eps of the element type.
struct SvdRatios {
    svd::Status status = svd::Status::InvalidInput;
    std::size_t uRows  = 0;
    std::size_t uCols  = 0;
    std::size_t vRows  = 0;
    std::size_t vCols  = 0;
    std::size_t sSize  = 0;
    long double reconstruction{0}; // ||A - U diag(S) V^H||_F / ||A||_F over the first S.size() columns
    long double orthogonalityU{0}; // max |U^H U - I| over every column of U
    long double orthogonalityV{0}; // max |V^H V - I| over every column of V
    long double singularValues{0}; // max |S_l - expected_l| / expected_0
};

// The bound of each ratio in SvdRatios.
constexpr long double kRatioBound = 50.0L;

template<typename T>
[[nodiscard]] std::complex<long double> toLongDouble(T value) {
    if constexpr (gr::meta::complex_like<T>) {
        return {static_cast<long double>(value.real()), static_cast<long double>(value.imag())};
    } else {
        return {static_cast<long double>(value), 0.0L};
    }
}

template<typename T>
[[nodiscard]] long double orthogonalityError(const gr::Tensor<T>& Q) {
    long double worst{0};
    for (std::size_t a = 0; a < Q.extent(1); ++a) {
        for (std::size_t b = 0; b < Q.extent(1); ++b) {
            std::complex<long double> sum{0};
            for (std::size_t r = 0; r < Q.extent(0); ++r) {
                sum += std::conj(toLongDouble(Q[r, a])) * toLongDouble(Q[r, b]);
            }
            if (a == b) {
                sum -= 1.0L;
            }
            worst = std::max(worst, std::abs(sum));
        }
    }
    return worst;
}

// The m x n matrix `scale * entries`, the entries row by row.
template<typename T>
[[nodiscard]] gr::Tensor<T> scaledMatrix(std::size_t m, std::size_t n, gr::meta::fundamental_base_value_type_t<T> scale, std::initializer_list<long double> entries) {
    using RealT = gr::meta::fundamental_base_value_type_t<T>;
    gr::Tensor<T> A({m, n});
    std::size_t   index = 0;
    for (long double entry : entries) {
        A.data()[index++] = T{scale * static_cast<RealT>(entry)};
    }
    return A;
}

template<typename T, typename RealT = gr::meta::fundamental_base_value_type_t<T>>
[[nodiscard]] SvdRatios ratiosOf(const gr::Tensor<T>& A, svd::Status status, const gr::Tensor<T>& U, const gr::Tensor<RealT>& S, const gr::Tensor<T>& V, const std::vector<long double>& expected) {
    const std::size_t m    = A.extent(0);
    const std::size_t n    = A.extent(1);
    const long double unit = static_cast<long double>(std::max(m, n)) * static_cast<long double>(std::numeric_limits<RealT>::epsilon());

    SvdRatios ratios;
    ratios.status = status;
    ratios.uRows  = U.extent(0);
    ratios.uCols  = U.extent(1);
    ratios.vRows  = V.extent(0);
    ratios.vCols  = V.extent(1);
    ratios.sSize  = S.size();
    if (U.extent(0) != m || V.extent(0) != n || U.extent(1) < S.size() || V.extent(1) < S.size()) {
        ratios.reconstruction = std::numeric_limits<long double>::infinity();
        return ratios;
    }

    // A and S enter the sums times 2^-shift, which brings the largest part of A into [1, 2). The squares then stay
    // in range where long double has the range of double.
    long double largestPart{0};
    for (const T& value : A) {
        const std::complex<long double> entry = toLongDouble(value);
        largestPart                           = std::max({largestPart, std::abs(entry.real()), std::abs(entry.imag())});
    }
    const int  shift   = largestPart > 0.0L && std::isfinite(largestPart) ? std::ilogb(largestPart) : 0;
    const auto shifted = [shift](std::complex<long double> z) { return std::complex<long double>{std::ldexp(z.real(), -shift), std::ldexp(z.imag(), -shift)}; };

    long double normA{0};
    long double normDiff{0};
    for (std::size_t i = 0; i < m; ++i) {
        for (std::size_t j = 0; j < n; ++j) {
            std::complex<long double> sum{0};
            for (std::size_t l = 0; l < S.size(); ++l) {
                sum += toLongDouble(U[i, l]) * std::ldexp(static_cast<long double>(S[l]), -shift) * std::conj(toLongDouble(V[j, l]));
            }
            const std::complex<long double> entry = shifted(toLongDouble(A[i, j]));
            normA += std::norm(entry);
            normDiff += std::norm(entry - sum);
        }
    }
    ratios.reconstruction = (normA > 0.0L ? std::sqrt(normDiff / normA) : std::sqrt(normDiff)) / unit;
    ratios.orthogonalityU = orthogonalityError(U) / unit;
    ratios.orthogonalityV = orthogonalityError(V) / unit;
    if (expected.size() == S.size() && !expected.empty()) {
        std::vector<long double> sorted(S.size());
        for (std::size_t l = 0; l < S.size(); ++l) {
            sorted[l] = static_cast<long double>(S[l]);
        }
        std::ranges::sort(sorted, std::greater<>{});
        long double worst{0};
        for (std::size_t l = 0; l < S.size(); ++l) {
            worst = std::max(worst, std::abs(sorted[l] - expected[l]));
        }
        ratios.singularValues = worst / (expected[0] * unit);
    } else if (!expected.empty()) {
        ratios.singularValues = std::numeric_limits<long double>::infinity();
    }
    return ratios;
}

// The ratios of gesvd(U, S, V, A) with the given algorithm.
template<typename T>
[[nodiscard]] SvdRatios svdRatios(const gr::Tensor<T>& A, svd::Algorithm algorithm, const std::vector<long double>& expected, bool fullMatrices = false) {
    using RealT = gr::meta::fundamental_base_value_type_t<T>;
    svd::Config<RealT> config;
    config.algorithm    = algorithm;
    config.fullMatrices = fullMatrices;
    gr::Tensor<T>     U, V;
    gr::Tensor<RealT> S;
    const svd::Status status = gesvd(U, S, V, A, config);
    return ratiosOf(A, status, U, S, V, expected);
}

// The largest singular value error of gesvd(S, A), divided by max(m, n) eps expected[0].
template<typename T>
[[nodiscard]] long double valuesOnlyRatio(const gr::Tensor<T>& A, svd::Algorithm algorithm, const std::vector<long double>& expected) {
    using RealT = gr::meta::fundamental_base_value_type_t<T>;
    svd::Config<RealT> config;
    config.algorithm = algorithm;
    gr::Tensor<RealT> S;
    if (gesvd(S, A, config) != svd::Status::Success || S.size() != expected.size() || expected.empty()) {
        return std::numeric_limits<long double>::infinity();
    }
    const long double unit = static_cast<long double>(std::max(A.extent(0), A.extent(1))) * static_cast<long double>(std::numeric_limits<RealT>::epsilon());
    long double       worst{0};
    for (std::size_t l = 0; l < S.size(); ++l) {
        worst = std::max(worst, std::abs(static_cast<long double>(S[l]) - expected[l]));
    }
    return worst / (expected[0] * unit);
}

// The ratios of detail::svdJacobi or detail::svdGolubReinsch on a tall A, with gesvd's default tolerance and
// iteration limit and without gesvd's scaling of A.
template<typename T>
[[nodiscard]] SvdRatios unscaledRatios(const gr::Tensor<T>& A, svd::Algorithm algorithm, const std::vector<long double>& expected) {
    using RealT               = gr::meta::fundamental_base_value_type_t<T>;
    const std::size_t largest = std::max(A.extent(0), A.extent(1));
    const RealT       tol     = std::numeric_limits<RealT>::epsilon() * static_cast<RealT>(largest);
    const std::size_t maxIter = 30UZ * largest;
    gr::Tensor<T>     U(A);
    gr::Tensor<T>     V;
    gr::Tensor<RealT> S;
    const svd::Status status = algorithm == svd::Algorithm::Jacobi ? gr::math::detail::svdJacobi(U, &V, S, maxIter, tol, true) : gr::math::detail::svdGolubReinsch(U, &V, S, maxIter, tol, true);
    return ratiosOf(A, status, U, S, V, expected);
}

template<typename T>
[[nodiscard]] std::string typeLabel() {
    if constexpr (gr::meta::complex_like<T>) {
        return std::is_same_v<T, std::complex<float>> ? "complex<float>" : "complex<double>";
    } else {
        return std::is_same_v<T, float> ? "float" : "double";
    }
}

// Expects Success and every ratio at most kRatioBound, through gesvd or, with `unscaled`, through unscaledRatios.
template<typename T>
void expectSvdHolds(const gr::Tensor<T>& A, svd::Algorithm algorithm, const std::vector<long double>& expected, const std::string& label, bool unscaled = false) {
    using boost::ut::expect;
    using boost::ut::le;
    const SvdRatios ratios = unscaled ? unscaledRatios(A, algorithm, expected) : svdRatios(A, algorithm, expected);
    const auto      name   = typeLabel<T>() + " " + label;
    expect(ratios.status == svd::Status::Success) << name << "status" << static_cast<int>(ratios.status);
    expect(le(ratios.reconstruction, kRatioBound)) << name << "reconstruction";
    expect(le(ratios.orthogonalityU, kRatioBound)) << name << "orthogonality of U";
    expect(le(ratios.orthogonalityV, kRatioBound)) << name << "orthogonality of V";
    expect(le(ratios.singularValues, kRatioBound)) << name << "singular values";
}

const boost::ut::suite<"SVD Basic Properties"> _svd_basic = [] {
    using namespace boost::ut;
    using namespace boost::ut::literals;
    using gr::Tensor;

    "singular values are non-negative"_test = []<typename Config> {
        using T     = Config::value_type;
        using RealT = gr::meta::fundamental_base_value_type_t<T>;

        Tensor<T> A({4, 4});
        randomize(A, T{-10}, T{10});

        svd::Config<RealT> config;
        config.algorithm = Config::algorithm;

        Tensor<T>     U, V;
        Tensor<RealT> S;
        auto          status = gesvd(U, S, V, A, config);

        expect(status == svd::Status::Success) << Config::name();
        for (const auto& s : S) {
            expect(ge(s, RealT{0})) << Config::name();
        }
    } | AllSvdConfigs{};

    "singular values are sorted descending"_test = []<typename Config> {
        using T     = Config::value_type;
        using RealT = gr::meta::fundamental_base_value_type_t<T>;

        Tensor<T> A({5, 5});
        randomize(A, T{-10}, T{10});

        svd::Config<RealT> config;
        config.algorithm = Config::algorithm;

        Tensor<T>     U, V;
        Tensor<RealT> S;
        gesvd(U, S, V, A, config);

        for (std::size_t i = 0; i + 1 < S.size(); ++i) {
            expect(ge(S[i], S[i + 1])) << Config::name();
        }
    } | AllSvdConfigs{};

    "correct output dimensions for square matrix"_test = []<typename Config> {
        using T     = Config::value_type;
        using RealT = gr::meta::fundamental_base_value_type_t<T>;

        Tensor<T> A({4, 4});
        randomize(A, T{-5}, T{5});

        svd::Config<RealT> config;
        config.algorithm = Config::algorithm;

        Tensor<T>     U, V;
        Tensor<RealT> S;
        gesvd(U, S, V, A, config);

        expect(eq(U.extent(0), 4UZ)) << Config::name();
        expect(eq(U.extent(1), 4UZ)) << Config::name();
        expect(eq(S.size(), 4UZ)) << Config::name();
        expect(eq(V.extent(0), 4UZ)) << Config::name();
        expect(eq(V.extent(1), 4UZ)) << Config::name();
    } | AllSvdConfigs{};

    "correct output dimensions for tall matrix"_test = []<typename Config> {
        using T     = Config::value_type;
        using RealT = gr::meta::fundamental_base_value_type_t<T>;

        Tensor<T> A({6, 3});
        randomize(A, T{-5}, T{5});

        svd::Config<RealT> config;
        config.algorithm = Config::algorithm;

        Tensor<T>     U, V;
        Tensor<RealT> S;
        gesvd(U, S, V, A, config);

        expect(eq(U.extent(0), 6UZ)) << Config::name();
        expect(eq(U.extent(1), 3UZ)) << Config::name();
        expect(eq(S.size(), 3UZ)) << Config::name();
        expect(eq(V.extent(0), 3UZ)) << Config::name();
        expect(eq(V.extent(1), 3UZ)) << Config::name();
    } | AllSvdConfigs{};

    "correct output dimensions for wide matrix"_test = []<typename Config> {
        using T     = Config::value_type;
        using RealT = gr::meta::fundamental_base_value_type_t<T>;

        Tensor<T> A({3, 6});
        randomize(A, T{-5}, T{5});

        svd::Config<RealT> config;
        config.algorithm = Config::algorithm;

        Tensor<T>     U, V;
        Tensor<RealT> S;
        gesvd(U, S, V, A, config);

        expect(eq(U.extent(0), 3UZ)) << Config::name();
        expect(eq(U.extent(1), 3UZ)) << Config::name();
        expect(eq(S.size(), 3UZ)) << Config::name();
        expect(eq(V.extent(0), 6UZ)) << Config::name();
        expect(eq(V.extent(1), 3UZ)) << Config::name();
    } | AllSvdConfigs{};
};

const boost::ut::suite<"SVD Reconstruction"> _svd_reconstruction = [] {
    using namespace boost::ut;
    using namespace boost::ut::literals;
    using gr::Tensor;

    "reconstruction A = U*S*V^H (square)"_test = []<typename Config> {
        using T     = Config::value_type;
        using RealT = gr::meta::fundamental_base_value_type_t<T>;

        // Use a simple known matrix instead of random
        Tensor<T> A({3, 3});
        A = {T{4}, T{1}, T{2}, T{1}, T{5}, T{3}, T{2}, T{3}, T{6}};

        svd::Config<RealT> config;
        config.algorithm = Config::algorithm;

        Tensor<T>     U, V;
        Tensor<RealT> S;
        gesvd(U, S, V, A, config);

        expect(verifyReconstruction(A, U, S, V, defaultTolerance<T>())) << Config::name();
    } | AllSvdConfigs{};

    "reconstruction A = U*S*V^H (tall)"_test = []<typename Config> {
        using T     = Config::value_type;
        using RealT = gr::meta::fundamental_base_value_type_t<T>;

        Tensor<T> A({6, 3});
        randomize(A, T{-10}, T{10});

        svd::Config<RealT> config;
        config.algorithm = Config::algorithm;

        Tensor<T>     U, V;
        Tensor<RealT> S;
        gesvd(U, S, V, A, config);

        expect(verifyReconstruction(A, U, S, V, defaultTolerance<T>())) << Config::name();
    } | AllSvdConfigs{};

    "reconstruction A = U*S*V^H (wide)"_test = []<typename Config> {
        using T     = Config::value_type;
        using RealT = gr::meta::fundamental_base_value_type_t<T>;

        Tensor<T> A({3, 7});
        randomize(A, T{-10}, T{10});

        svd::Config<RealT> config;
        config.algorithm = Config::algorithm;

        Tensor<T>     U, V;
        Tensor<RealT> S;
        gesvd(U, S, V, A, config);

        expect(verifyReconstruction(A, U, S, V, defaultTolerance<T>())) << Config::name();
    } | AllSvdConfigs{};
};

const boost::ut::suite<"SVD Orthogonality"> _svd_orthogonality = [] {
    using namespace boost::ut;
    using namespace boost::ut::literals;
    using gr::Tensor;

    "U is orthogonal/unitary"_test = []<typename Config> {
        using T     = Config::value_type;
        using RealT = gr::meta::fundamental_base_value_type_t<T>;

        Tensor<T> A({5, 4});
        randomize(A, T{-10}, T{10});

        svd::Config<RealT> config;
        config.algorithm = Config::algorithm;

        Tensor<T>     U, V;
        Tensor<RealT> S;
        gesvd(U, S, V, A, config);

        expect(verifyOrthogonal(U, defaultTolerance<T>())) << Config::name();
    } | AllSvdConfigs{};

    "V is orthogonal/unitary"_test = []<typename Config> {
        using T     = Config::value_type;
        using RealT = gr::meta::fundamental_base_value_type_t<T>;

        Tensor<T> A({4, 5});
        randomize(A, T{-10}, T{10});

        svd::Config<RealT> config;
        config.algorithm = Config::algorithm;

        Tensor<T>     U, V;
        Tensor<RealT> S;
        gesvd(U, S, V, A, config);

        expect(verifyOrthogonal(V, defaultTolerance<T>())) << Config::name();
    } | AllSvdConfigs{};
};

const boost::ut::suite<"SVD Special Matrices"> _svd_special = [] {
    using namespace boost::ut;
    using namespace boost::ut::literals;
    using gr::Tensor;

    "identity matrix has all singular values = 1"_test = []<typename Config> {
        using T     = Config::value_type;
        using RealT = gr::meta::fundamental_base_value_type_t<T>;

        Tensor<T> I({4, 4});
        I.fill(T{0});
        for (std::size_t i = 0; i < 4; ++i) {
            I[i, i] = T{1};
        }

        svd::Config<RealT> config;
        config.algorithm = Config::algorithm;

        Tensor<T>     U, V;
        Tensor<RealT> S;
        gesvd(U, S, V, I, config);

        for (const auto& s : S) {
            expect(approx_equal(s, RealT{1}, defaultTolerance<T>())) << Config::name();
        }
    } | AllSvdConfigs{};

    "diagonal matrix singular values equal diagonal entries (sorted)"_test = []<typename Config> {
        using T     = Config::value_type;
        using RealT = gr::meta::fundamental_base_value_type_t<T>;

        Tensor<T> D({4, 4});
        D.fill(T{0});
        D[0, 0] = T{3};
        D[1, 1] = T{7};
        D[2, 2] = T{1};
        D[3, 3] = T{5};

        svd::Config<RealT> config;
        config.algorithm = Config::algorithm;

        Tensor<T>     U, V;
        Tensor<RealT> S;
        gesvd(U, S, V, D, config);

        Tensor<RealT> expected({4});
        expected = {7, 5, 3, 1};
        expect(tensors_approximately_equal(S, expected, defaultTolerance<T>())) << Config::name();
    } | RealSvdConfigs{};

    "zero matrix has all singular values = 0"_test = []<typename Config> {
        using T     = Config::value_type;
        using RealT = gr::meta::fundamental_base_value_type_t<T>;

        Tensor<T> Z({3, 4});
        Z.fill(T{0});

        svd::Config<RealT> config;
        config.algorithm = Config::algorithm;

        Tensor<T>     U, V;
        Tensor<RealT> S;
        gesvd(U, S, V, Z, config);

        for (const auto& s : S) {
            expect(approx_equal(s, RealT{0}, defaultTolerance<T>())) << Config::name();
        }
    } | AllSvdConfigs{};

    "single element matrix"_test = []<typename Config> {
        using T     = Config::value_type;
        using RealT = gr::meta::fundamental_base_value_type_t<T>;

        Tensor<T> A({1, 1});
        A[0] = T{5};

        svd::Config<RealT> config;
        config.algorithm = Config::algorithm;

        Tensor<T>     U, V;
        Tensor<RealT> S;
        gesvd(U, S, V, A, config);

        expect(eq(S.size(), 1UZ)) << Config::name();
        expect(approx_equal(S[0], RealT{5}, defaultTolerance<T>())) << Config::name();
    } | RealSvdConfigs{};
};

const boost::ut::suite<"SVD Algorithm Comparison"> _svd_algo_compare = [] {
    using namespace boost::ut;
    using namespace boost::ut::literals;
    using gr::Tensor;

    "Jacobi and GolubReinsch produce same singular values"_test = [] {
        "small square matrix"_test = [] {
            Tensor<double> A({4, 4});
            A = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16};

            svd::Config<double> jacobiConfig;
            jacobiConfig.algorithm = svd::Algorithm::Jacobi;

            svd::Config<double> grConfig;
            grConfig.algorithm = svd::Algorithm::GolubReinsch;

            Tensor<double> U_j, V_j, S_j;
            Tensor<double> U_gr, V_gr, S_gr;

            gesvd(U_j, S_j, V_j, A, jacobiConfig);
            gesvd(U_gr, S_gr, V_gr, A, grConfig);

            // verify singular values match
            expect(tensors_approximately_equal(S_j, S_gr, 1e-9));

            // verify reconstruction works for both algorithms
            expect(verifyReconstruction(A, U_j, S_j, V_j, 1e-9));
            expect(verifyReconstruction(A, U_gr, S_gr, V_gr, 1e-9));
        };

        "random 5x5"_test = [] {
            Tensor<double> A({5, 5});
            randomize(A, -10.0, 10.0);

            svd::Config<double> jacobiConfig;
            jacobiConfig.algorithm = svd::Algorithm::Jacobi;

            svd::Config<double> grConfig;
            grConfig.algorithm = svd::Algorithm::GolubReinsch;

            Tensor<double> U_j, V_j, S_j;
            Tensor<double> U_gr, V_gr, S_gr;

            gesvd(U_j, S_j, V_j, A, jacobiConfig);
            gesvd(U_gr, S_gr, V_gr, A, grConfig);

            expect(tensors_approximately_equal(S_j, S_gr, 1e-9));
        };

        "tall matrix 8x3"_test = [] {
            Tensor<double> A({8, 3});
            randomize(A, -5.0, 5.0);

            svd::Config<double> jacobiConfig;
            jacobiConfig.algorithm = svd::Algorithm::Jacobi;

            svd::Config<double> grConfig;
            grConfig.algorithm = svd::Algorithm::GolubReinsch;

            Tensor<double> U_j, V_j, S_j;
            Tensor<double> U_gr, V_gr, S_gr;

            gesvd(U_j, S_j, V_j, A, jacobiConfig);
            gesvd(U_gr, S_gr, V_gr, A, grConfig);

            expect(tensors_approximately_equal(S_j, S_gr, 1e-9));
        };

        "wide matrix 3x8"_test = [] {
            Tensor<double> A({3, 8});
            randomize(A, -5.0, 5.0);

            svd::Config<double> jacobiConfig;
            jacobiConfig.algorithm = svd::Algorithm::Jacobi;

            svd::Config<double> grConfig;
            grConfig.algorithm = svd::Algorithm::GolubReinsch;

            Tensor<double> U_j, V_j, S_j;
            Tensor<double> U_gr, V_gr, S_gr;

            gesvd(U_j, S_j, V_j, A, jacobiConfig);
            gesvd(U_gr, S_gr, V_gr, A, grConfig);

            expect(tensors_approximately_equal(S_j, S_gr, 1e-9));
        };

        "rank-deficient matrix"_test = [] {
            Tensor<double> A({4, 4});
            // rank-1 matrix: A = u * v^T
            A = {1, 2, 3, 4, 2, 4, 6, 8, 3, 6, 9, 12, 4, 8, 12, 16};

            svd::Config<double> jacobiConfig;
            jacobiConfig.algorithm = svd::Algorithm::Jacobi;

            svd::Config<double> grConfig;
            grConfig.algorithm = svd::Algorithm::GolubReinsch;

            Tensor<double> U_j, V_j, S_j;
            Tensor<double> U_gr, V_gr, S_gr;

            gesvd(U_j, S_j, V_j, A, jacobiConfig);
            gesvd(U_gr, S_gr, V_gr, A, grConfig);

            expect(tensors_approximately_equal(S_j, S_gr, 1e-6));
        };

        "Hilbert 4x4 (ill-conditioned)"_test = [] {
            Tensor<double> H({4, 4});
            for (std::size_t i = 0; i < 4; ++i) {
                for (std::size_t j = 0; j < 4; ++j) {
                    H[i, j] = 1.0 / static_cast<double>(i + j + 1);
                }
            }

            svd::Config<double> jacobiConfig;
            jacobiConfig.algorithm = svd::Algorithm::Jacobi;

            svd::Config<double> grConfig;
            grConfig.algorithm = svd::Algorithm::GolubReinsch;

            Tensor<double> U_j, V_j, S_j;
            Tensor<double> U_gr, V_gr, S_gr;

            gesvd(U_j, S_j, V_j, H, jacobiConfig);
            gesvd(U_gr, S_gr, V_gr, H, grConfig);

            expect(tensors_approximately_equal(S_j, S_gr, 1e-9));
        };
    };
};

const boost::ut::suite<"SVD Edge Cases"> _svd_edge_cases = [] {
    using namespace boost::ut;
    using namespace boost::ut::literals;
    using gr::Tensor;

    "empty matrix"_test = []<typename Config> {
        using T     = Config::value_type;
        using RealT = gr::meta::fundamental_base_value_type_t<T>;

        Tensor<T> A({0, 0});

        svd::Config<RealT> config;
        config.algorithm = Config::algorithm;

        Tensor<T>     U, V;
        Tensor<RealT> S;
        auto          status = gesvd(U, S, V, A, config);

        expect(status == svd::Status::Success) << Config::name();
        expect(eq(U.size(), 0UZ)) << Config::name();
        expect(eq(S.size(), 0UZ)) << Config::name();
        expect(eq(V.size(), 0UZ)) << Config::name();
    } | RealSvdConfigs{};

    "single row vector"_test = []<typename Config> {
        using T     = Config::value_type;
        using RealT = gr::meta::fundamental_base_value_type_t<T>;

        Tensor<T> A({1, 5});
        A = {T{1}, T{2}, T{3}, T{4}, T{5}};

        svd::Config<RealT> config;
        config.algorithm = Config::algorithm;

        Tensor<T>     U, V;
        Tensor<RealT> S;
        gesvd(U, S, V, A, config);

        expect(eq(U.extent(0), 1UZ)) << Config::name();
        expect(eq(S.size(), 1UZ)) << Config::name();
        expect(eq(V.extent(0), 5UZ)) << Config::name();

        RealT expected_s = std::sqrt(RealT{1 + 4 + 9 + 16 + 25});
        expect(approx_equal(S[0], expected_s, defaultTolerance<T>())) << Config::name();
    } | RealSvdConfigs{};

    "single column vector"_test = []<typename Config> {
        using T     = Config::value_type;
        using RealT = gr::meta::fundamental_base_value_type_t<T>;

        Tensor<T> A({5, 1});
        A = {T{1}, T{2}, T{3}, T{4}, T{5}};

        svd::Config<RealT> config;
        config.algorithm = Config::algorithm;

        Tensor<T>     U, V;
        Tensor<RealT> S;
        gesvd(U, S, V, A, config);

        expect(eq(U.extent(0), 5UZ)) << Config::name();
        expect(eq(S.size(), 1UZ)) << Config::name();
        expect(eq(V.extent(0), 1UZ)) << Config::name();

        RealT expected_s = std::sqrt(RealT{1 + 4 + 9 + 16 + 25});
        expect(approx_equal(S[0], expected_s, defaultTolerance<T>())) << Config::name();
    } | RealSvdConfigs{};

    "very tall matrix 100x2"_test = []<typename Config> {
        using T     = Config::value_type;
        using RealT = gr::meta::fundamental_base_value_type_t<T>;

        Tensor<T> A({100, 2});
        randomize(A, T{-10}, T{10});

        svd::Config<RealT> config;
        config.algorithm = Config::algorithm;

        Tensor<T>     U, V;
        Tensor<RealT> S;
        auto          status = gesvd(U, S, V, A, config);

        expect(status == svd::Status::Success) << Config::name();
        expect(eq(U.extent(0), 100UZ)) << Config::name();
        expect(eq(U.extent(1), 2UZ)) << Config::name();
        expect(eq(S.size(), 2UZ)) << Config::name();
        expect(eq(V.extent(0), 2UZ)) << Config::name();
        expect(eq(V.extent(1), 2UZ)) << Config::name();
        expect(verifyReconstruction(A, U, S, V, defaultTolerance<T>())) << Config::name();
    } | RealSvdConfigs{};

    "very wide matrix 2x100"_test = []<typename Config> {
        using T     = Config::value_type;
        using RealT = gr::meta::fundamental_base_value_type_t<T>;

        Tensor<T> A({2, 100});
        randomize(A, T{-10}, T{10});

        svd::Config<RealT> config;
        config.algorithm = Config::algorithm;

        Tensor<T>     U, V;
        Tensor<RealT> S;
        auto          status = gesvd(U, S, V, A, config);

        expect(status == svd::Status::Success) << Config::name();
        expect(eq(U.extent(0), 2UZ)) << Config::name();
        expect(eq(U.extent(1), 2UZ)) << Config::name();
        expect(eq(S.size(), 2UZ)) << Config::name();
        expect(eq(V.extent(0), 100UZ)) << Config::name();
        expect(eq(V.extent(1), 2UZ)) << Config::name();
        expect(verifyReconstruction(A, U, S, V, defaultTolerance<T>())) << Config::name();
    } | RealSvdConfigs{};

    "rank-1 outer product"_test = []<typename Config> {
        using T     = Config::value_type;
        using RealT = gr::meta::fundamental_base_value_type_t<T>;

        // A = u * v^T where u = [1,2,3,4]^T, v = [1,1,1]^T
        Tensor<T> A({4, 3});
        A = {T{1}, T{1}, T{1}, T{2}, T{2}, T{2}, T{3}, T{3}, T{3}, T{4}, T{4}, T{4}};

        svd::Config<RealT> config;
        config.algorithm = Config::algorithm;

        Tensor<T>     U, V;
        Tensor<RealT> S;
        gesvd(U, S, V, A, config);

        // Only one non-zero singular value
        expect(gt(S[0], RealT{1})) << Config::name();
        expect(approx_equal(S[1], RealT{0}, static_cast<RealT>(1e-6f))) << Config::name();
        expect(approx_equal(S[2], RealT{0}, static_cast<RealT>(1e-6f))) << Config::name();
    } | RealSvdConfigs{};

    "clustered singular values"_test = []<typename Config> {
        using T     = Config::value_type;
        using RealT = gr::meta::fundamental_base_value_type_t<T>;

        // Diagonal with very close values
        Tensor<T> D({4, 4});
        D.fill(T{0});
        D[0, 0] = T{1.0};
        D[1, 1] = static_cast<T>(1.0f + 1e-10f);
        D[2, 2] = static_cast<T>(1.0f + 2e-10f);
        D[3, 3] = T{0.5};

        svd::Config<RealT> config;
        config.algorithm = Config::algorithm;

        Tensor<T>     U, V;
        Tensor<RealT> S;
        auto          status = gesvd(U, S, V, D, config);

        expect(status == svd::Status::Success) << Config::name();
        // Check reconstruction even with clustered values
        expect(verifyReconstruction(D, U, S, V, defaultTolerance<T>())) << Config::name();
    } | RealSvdConfigs{};

    "matrix with embedded zero row"_test = []<typename Config> {
        using T     = Config::value_type;
        using RealT = gr::meta::fundamental_base_value_type_t<T>;

        Tensor<T> A({4, 3});
        A = {T{1}, T{2}, T{3}, T{0}, T{0}, T{0}, T{4}, T{5}, T{6}, T{7}, T{8}, T{9}};

        svd::Config<RealT> config;
        config.algorithm = Config::algorithm;

        Tensor<T>     U, V;
        Tensor<RealT> S;
        auto          status = gesvd(U, S, V, A, config);

        expect(status == svd::Status::Success) << Config::name();
        expect(verifyReconstruction(A, U, S, V, defaultTolerance<T>())) << Config::name();
    } | RealSvdConfigs{};

    "matrix with embedded zero column"_test = []<typename Config> {
        using T     = Config::value_type;
        using RealT = gr::meta::fundamental_base_value_type_t<T>;

        Tensor<T> A({3, 4});
        A = {T{1}, T{0}, T{2}, T{3}, T{4}, T{0}, T{5}, T{6}, T{7}, T{0}, T{8}, T{9}};

        svd::Config<RealT> config;
        config.algorithm = Config::algorithm;

        Tensor<T>     U, V;
        Tensor<RealT> S;
        auto          status = gesvd(U, S, V, A, config);

        expect(status == svd::Status::Success) << Config::name();
        expect(verifyReconstruction(A, U, S, V, defaultTolerance<T>())) << Config::name();
    } | RealSvdConfigs{};

    "Vandermonde-like matrix"_test = []<typename Config> {
        using T     = Config::value_type;
        using RealT = gr::meta::fundamental_base_value_type_t<T>;

        // V[i,j] = x[i]^j for x = [1, 2, 3, 4]
        Tensor<T> V_mat({4, 4});
        for (std::size_t i = 0; i < 4; ++i) {
            T x = T(i + 1);
            for (std::size_t j = 0; j < 4; ++j) {
                V_mat[i, j] = std::pow(x, T(j));
            }
        }

        svd::Config<RealT> config;
        config.algorithm = Config::algorithm;

        Tensor<T>     U, V;
        Tensor<RealT> S;
        auto          status = gesvd(U, S, V, V_mat, config);

        expect(status == svd::Status::Success) << Config::name();
        // Vandermonde matrices are ill-conditioned, need looser tolerance
        auto tol = std::is_same_v<RealT, float> ? static_cast<RealT>(1e-3f) : static_cast<RealT>(1e-8);
        expect(verifyReconstruction(V_mat, U, S, V, tol)) << Config::name();
    } | RealSvdConfigs{};
};

const boost::ut::suite<"SVD Numerical Stability"> _svd_stability = [] {
    using namespace boost::ut;
    using namespace boost::ut::literals;
    using gr::Tensor;

    "Hilbert matrix (ill-conditioned)"_test = []<typename Config> {
        using T     = Config::value_type;
        using RealT = gr::meta::fundamental_base_value_type_t<T>;

        if constexpr (!gr::meta::complex_like<T>) {
            Tensor<T> H({5, 5});
            for (std::size_t i = 0; i < 5; ++i) {
                for (std::size_t j = 0; j < 5; ++j) {
                    H[i, j] = T{1} / T(i + j + 1);
                }
            }

            svd::Config<RealT> config;
            config.algorithm = Config::algorithm;

            Tensor<T>     U, V;
            Tensor<RealT> S;
            auto          status = gesvd(U, S, V, H, config);

            expect(status == svd::Status::Success) << Config::name();
            // Hilbert matrices are highly ill-conditioned (cond ~10^5 for 5x5), need looser tolerance
            auto tol = std::is_same_v<RealT, float> ? static_cast<RealT>(1e-2f) : static_cast<RealT>(1e-8);
            expect(verifyReconstruction(H, U, S, V, tol)) << Config::name();
        }
    } | RealSvdConfigs{};

    "matrix with widely varying singular values"_test = []<typename Config> {
        using T     = Config::value_type;
        using RealT = gr::meta::fundamental_base_value_type_t<T>;

        if constexpr (!gr::meta::complex_like<T>) {
            Tensor<T> A({3, 3});
            A.fill(T{0});
            A[0, 0] = T{1e6};
            A[1, 1] = T{1};
            A[2, 2] = static_cast<T>(1e-6f);

            svd::Config<RealT> config;
            config.algorithm = Config::algorithm;

            Tensor<T>     U, V;
            Tensor<RealT> S;
            gesvd(U, S, V, A, config);

            expect(approx_equal(S[0], RealT{1e6}, RealT{1})) << Config::name();
            expect(approx_equal(S[1], RealT{1}, static_cast<RealT>(1e-6f))) << Config::name();
            expect(approx_equal(S[2], static_cast<RealT>(1e-6f), static_cast<RealT>(1e-12f))) << Config::name();
        }
    } | RealSvdConfigs{};

    "nearly singular matrix"_test = []<typename Config> {
        using T     = Config::value_type;
        using RealT = gr::meta::fundamental_base_value_type_t<T>;

        // Matrix with condition number ~ 10^6
        Tensor<T> A({3, 3});
        A = {T{1}, T{1}, T{1}, T{1}, T{1}, static_cast<T>(1.000001f), T{1}, static_cast<T>(1.000001f), T{1}};

        svd::Config<RealT> config;
        config.algorithm = Config::algorithm;

        Tensor<T>     U, V;
        Tensor<RealT> S;
        auto          status = gesvd(U, S, V, A, config);

        expect(status == svd::Status::Success) << Config::name();
        // Smallest singular value should be very small
        expect(lt(S[2], static_cast<RealT>(1e-4f))) << Config::name();
    } | RealSvdConfigs{};

    "matrix with negative elements"_test = []<typename Config> {
        using T     = Config::value_type;
        using RealT = gr::meta::fundamental_base_value_type_t<T>;

        Tensor<T> A({3, 3});
        A = {T{-5}, T{2}, T{-1}, T{3}, T{-4}, T{2}, T{-1}, T{1}, T{-3}};

        svd::Config<RealT> config;
        config.algorithm = Config::algorithm;

        Tensor<T>     U, V;
        Tensor<RealT> S;
        auto          status = gesvd(U, S, V, A, config);

        expect(status == svd::Status::Success) << Config::name();
        // All singular values must be non-negative
        for (const auto& s : S) {
            expect(ge(s, RealT{0})) << Config::name();
        }
        expect(verifyReconstruction(A, U, S, V, defaultTolerance<T>())) << Config::name();
    } | AllSvdConfigs{};
};

const boost::ut::suite<"SVD Large Matrices"> _svd_large = [] {
    using namespace boost::ut;
    using namespace boost::ut::literals;
    using gr::Tensor;

    "50x50 random matrix"_test = []<typename Config> {
        using T     = Config::value_type;
        using RealT = gr::meta::fundamental_base_value_type_t<T>;

        if constexpr (!gr::meta::complex_like<T>) {
            Tensor<T> A({50, 50});
            randomize(A, T{-10}, T{10});

            svd::Config<RealT> config;
            config.algorithm = Config::algorithm;

            Tensor<T>     U, V;
            Tensor<RealT> S;
            auto          status = gesvd(U, S, V, A, config);

            expect(status == svd::Status::Success) << Config::name();
            expect(eq(S.size(), 50UZ)) << Config::name();

            for (std::size_t i = 0; i + 1 < S.size(); ++i) {
                expect(ge(S[i], S[i + 1])) << Config::name();
            }
        }
    } | RealSvdConfigs{};

    "100x100 random double"_test = [] {
        using Config = SvdTestConfig<double, svd::Algorithm::Auto>;
        using T      = double;
        using RealT  = double;

        Tensor<T> A({100, 100});
        randomize(A, T{-10}, T{10});

        svd::Config<RealT> config;
        config.algorithm = Config::algorithm;

        Tensor<T>     U, V;
        Tensor<RealT> S;
        auto          status = gesvd(U, S, V, A, config);

        expect(status == svd::Status::Success);
        expect(eq(S.size(), 100UZ));
    };

    "30x100 rectangular"_test = []<typename Config> {
        using T     = Config::value_type;
        using RealT = gr::meta::fundamental_base_value_type_t<T>;

        if constexpr (!gr::meta::complex_like<T>) {
            Tensor<T> A({30, 100});
            randomize(A, T{-5}, T{5});

            svd::Config<RealT> config;
            config.algorithm = Config::algorithm;

            Tensor<T>     U, V;
            Tensor<RealT> S;
            auto          status = gesvd(U, S, V, A, config);

            expect(status == svd::Status::Success) << Config::name();
            expect(eq(U.extent(0), 30UZ)) << Config::name();
            expect(eq(U.extent(1), 30UZ)) << Config::name();
            expect(eq(S.size(), 30UZ)) << Config::name();
            expect(eq(V.extent(0), 100UZ)) << Config::name();
            expect(eq(V.extent(1), 30UZ)) << Config::name();
        }
    } | RealSvdConfigs{};
};

const boost::ut::suite<"SVD Configuration"> _svd_config = [] {
    using namespace boost::ut;
    using namespace boost::ut::literals;
    using gr::Tensor;

    "compute only k singular values"_test = [] {
        Tensor<double> A({6, 6});
        randomize(A, -10.0, 10.0);

        svd::Config<double> config;
        config.k = 3;

        Tensor<double> U, V, S;
        auto           status = gesvd(U, S, V, A, config);

        expect(status == svd::Status::EarlyReturn);
        expect(eq(S.size(), 3UZ));
        expect(eq(U.extent(1), 3UZ));
        expect(eq(V.extent(1), 3UZ));

        for (std::size_t i = 0; i + 1 < S.size(); ++i) {
            expect(ge(S[i], S[i + 1]));
        }
    };

    "custom tolerance"_test = [] {
        Tensor<double> A({4, 4});
        randomize(A, -10.0, 10.0);

        svd::Config<double> config;
        config.tolerance = 1e-14;

        Tensor<double> U, V, S;
        auto           status = gesvd(U, S, V, A, config);

        expect(status == svd::Status::Success);
    };

    "custom max iterations"_test = [] {
        Tensor<double> A({4, 4});
        randomize(A, -10.0, 10.0);

        svd::Config<double> config;
        config.maxIterations = 1000;

        Tensor<double> U, V, S;
        auto           status = gesvd(U, S, V, A, config);

        expect(status == svd::Status::Success);
    };

    "singular values only (no U, V)"_test = [] {
        Tensor<double> A({5, 5});
        randomize(A, -10.0, 10.0);

        Tensor<double> S;
        auto           status = gesvd(S, A);

        expect(status == svd::Status::Success);
        expect(eq(S.size(), 5UZ));

        // Compare with full SVD
        Tensor<double> U, V, S_full;
        gesvd(U, S_full, V, A);

        expect(tensors_approximately_equal(S, S_full, 1e-9));
    };
};

const boost::ut::suite<"SVD Complex Support"> _svd_complex = [] {
    using namespace boost::ut;
    using namespace boost::ut::literals;
    using gr::Tensor;
    using Complex  = std::complex<double>;
    using ComplexF = std::complex<float>;

    "complex reconstruction with conjugate transpose"_test = [] {
        Tensor<Complex> A({4, 4});
        randomize(A, Complex{-5, -5}, Complex{5, 5});

        Tensor<Complex> U, V;
        Tensor<double>  S;
        gesvd(U, S, V, A);

        expect(verifyReconstruction(A, U, S, V, 1e-9));
    };

    "complex U^H * U = I"_test = [] {
        Tensor<Complex> A({5, 3});
        randomize(A, Complex{-10, -10}, Complex{10, 10});

        Tensor<Complex> U, V;
        Tensor<double>  S;
        gesvd(U, S, V, A);

        expect(verifyOrthogonal(U, 1e-9));
    };

    "complex V^H * V = I"_test = [] {
        Tensor<Complex> A({3, 5});
        randomize(A, Complex{-10, -10}, Complex{10, 10});

        Tensor<Complex> U, V;
        Tensor<double>  S;
        gesvd(U, S, V, A);

        expect(verifyOrthogonal(V, 1e-9));
    };

    "complex float precision"_test = [] {
        Tensor<ComplexF> A({3, 3});
        A = {ComplexF{1, 2}, ComplexF{3, 4}, ComplexF{5, 6}, ComplexF{7, 8}, ComplexF{9, 10}, ComplexF{11, 12}, ComplexF{13, 14}, ComplexF{15, 16}, ComplexF{17, 18}};

        Tensor<ComplexF> U, V;
        Tensor<float>    S;
        auto             status = gesvd(U, S, V, A);

        expect(status == svd::Status::Success);
        expect(eq(S.size(), 3UZ));

        for (const auto& s : S) {
            expect(ge(s, 0.0f));
        }
    };

    "Hermitian matrix"_test = [] {
        // A = A^H
        Tensor<Complex> A({3, 3});
        A[0, 0] = Complex{3, 0};
        A[0, 1] = Complex{1, 2};
        A[0, 2] = Complex{0, -1};
        A[1, 0] = Complex{1, -2};
        A[1, 1] = Complex{4, 0};
        A[1, 2] = Complex{2, 1};
        A[2, 0] = Complex{0, 1};
        A[2, 1] = Complex{2, -1};
        A[2, 2] = Complex{5, 0};

        Tensor<Complex> U, V;
        Tensor<double>  S;
        auto            status = gesvd(U, S, V, A);

        expect(status == svd::Status::Success);
        expect(verifyReconstruction(A, U, S, V, 1e-9));
    };
};

// A = s [[2, 1], [0, 1]] has the singular values s (sqrt(5) + 1) / sqrt(2) and s (sqrt(5) - 1) / sqrt(2).
std::vector<long double> upperTriangularSingularValues(long double s) {
    const long double root5 = std::sqrt(5.0L);
    return {s * (root5 + 1.0L) / std::sqrt(2.0L), s * (root5 - 1.0L) / std::sqrt(2.0L)};
}

const boost::ut::suite<"SVD Jacobi on hard inputs"> _svd_jacobi_hard = [] {
    using namespace boost::ut;
    using namespace boost::ut::literals;
    using AllTypes = std::tuple<float, double, std::complex<float>, std::complex<double>>;

    // The first three cases run detail::svdJacobi on the unscaled input; gesvd scales every input near 1 first.
    "Jacobi orthogonalizes columns whose squared norms lie near eps squared"_test = []<typename T> {
        using RealT       = gr::meta::fundamental_base_value_type_t<T>;
        const RealT scale = RealT{4} * std::numeric_limits<RealT>::epsilon();
        // s = 4 eps puts both squared column norms near eps^2
        expectSvdHolds(scaledMatrix<T>(2UZ, 2UZ, scale, {2, 1, 0, 1}), svd::Algorithm::Jacobi, upperTriangularSingularValues(static_cast<long double>(scale)), "s = 4 eps", true);
    } | AllTypes{};

    "Jacobi orthogonalizes columns whose squared norms overflow in their product"_test = []<typename T> {
        using RealT       = gr::meta::fundamental_base_value_type_t<T>;
        const RealT scale = std::sqrt(std::sqrt(std::numeric_limits<RealT>::max()));
        // the squared column norms 4 s^2 and 2 s^2 have a product beyond the largest finite value
        expectSvdHolds(scaledMatrix<T>(2UZ, 2UZ, scale, {2, 1, 0, 1}), svd::Algorithm::Jacobi, upperTriangularSingularValues(static_cast<long double>(scale)), "s = max^(1/4)", true);
    } | AllTypes{};

    "Jacobi keeps a singular value below eps"_test = []<typename T> {
        using RealT      = gr::meta::fundamental_base_value_type_t<T>;
        const RealT tiny = std::numeric_limits<RealT>::epsilon() / RealT{16};
        expectSvdHolds(scaledMatrix<T>(1UZ, 1UZ, tiny, {1}), svd::Algorithm::Jacobi, {static_cast<long double>(tiny)}, "[[eps / 16]]", true);
    } | AllTypes{};

    "gesvd holds where the squares of the entries overflow or underflow"_test = []<typename T> {
        using RealT = gr::meta::fundamental_base_value_type_t<T>;
        // the squares of the entries pass the largest finite value at the first scale and round to zero at the second
        const RealT hugeScale = RealT{4} * std::sqrt(std::numeric_limits<RealT>::max());
        const RealT tinyScale = std::sqrt(std::numeric_limits<RealT>::denorm_min()) / RealT{4};
        expectSvdHolds(scaledMatrix<T>(2UZ, 2UZ, hugeScale, {2, 1, 0, 1}), svd::Algorithm::Jacobi, upperTriangularSingularValues(static_cast<long double>(hugeScale)), "s = 4 sqrt(max)");
        expectSvdHolds(scaledMatrix<T>(2UZ, 2UZ, tinyScale, {2, 1, 0, 1}), svd::Algorithm::Jacobi, upperTriangularSingularValues(static_cast<long double>(tinyScale)), "s = sqrt(denorm_min) / 4");
        expect(le(valuesOnlyRatio(scaledMatrix<T>(2UZ, 2UZ, hugeScale, {2, 1, 0, 1}), svd::Algorithm::Jacobi, upperTriangularSingularValues(static_cast<long double>(hugeScale))), kRatioBound)) << "values only, s = 4 sqrt(max)";
        expect(le(valuesOnlyRatio(scaledMatrix<T>(2UZ, 2UZ, tinyScale, {2, 1, 0, 1}), svd::Algorithm::Jacobi, upperTriangularSingularValues(static_cast<long double>(tinyScale))), kRatioBound)) << "values only, s = sqrt(denorm_min) / 4";
    } | AllTypes{};

    "gesvd scales complex input whose magnitudes overflow while their parts are finite"_test = []<typename T> {
        using RealT       = gr::meta::fundamental_base_value_type_t<T>;
        const RealT scale = RealT{3} * (std::numeric_limits<RealT>::max() / RealT{8});
        // A = (1 + i) s [[2, 1], [0, 1]]: every part is finite and |(2 + 2i) s| exceeds the largest finite value
        gr::Tensor<T> A({2UZ, 2UZ});
        A[0UZ, 0UZ] = T{RealT{2} * scale, RealT{2} * scale};
        A[0UZ, 1UZ] = T{scale, scale};
        A[1UZ, 0UZ] = T{};
        A[1UZ, 1UZ] = T{scale, scale};

        std::vector<long double> expected = upperTriangularSingularValues(static_cast<long double>(scale));
        for (long double& value : expected) {
            value *= std::sqrt(2.0L);
        }
        // sigma_1 exceeds the largest finite value and rounds to infinity; sigma_2 is finite
        const long double largestFinite = static_cast<long double>(std::numeric_limits<RealT>::max());
        const long double unit          = 2.0L * static_cast<long double>(std::numeric_limits<RealT>::epsilon());
        const auto        expectValues  = [&](const gr::Tensor<RealT>& S, const std::string& label) {
            expect(eq(S.size(), expected.size())) << label;
            for (std::size_t l = 0UZ; l < std::min(S.size(), expected.size()); ++l) {
                const long double value = static_cast<long double>(S[l]);
                if (expected[l] > largestFinite) {
                    expect(std::isinf(value)) << label << "singular value" << l;
                } else {
                    expect(le(std::abs(value - expected[l]) / (expected[l] * unit), kRatioBound)) << label << "singular value" << l;
                }
            }
        };

        svd::Config<RealT> config;
        config.algorithm = svd::Algorithm::Jacobi;
        gr::Tensor<T>     U, V;
        gr::Tensor<RealT> S;
        const svd::Status status = gesvd(U, S, V, A, config);
        const SvdRatios   ratios = ratiosOf(A, status, U, S, V, {});
        expect(status == svd::Status::Success);
        expect(le(ratios.orthogonalityU, kRatioBound)) << "orthogonality of U";
        expect(le(ratios.orthogonalityV, kRatioBound)) << "orthogonality of V";
        expectValues(S, "U, S and V");

        gr::Tensor<RealT> valuesOnly;
        expect(gesvd(valuesOnly, A, config) == svd::Status::Success);
        expectValues(valuesOnly, "values only");
    } | std::tuple<std::complex<float>, std::complex<double>>{};

    "Jacobi returns orthonormal U and V for rank-deficient input"_test = []<typename T> {
        // a zero singular value, a rounding-level one, and a zero one that reaches V through the transposed path
        expectSvdHolds(scaledMatrix<T>(2UZ, 2UZ, 1, {1, 0, 0, 0}), svd::Algorithm::Jacobi, {1.0L, 0.0L}, "[[1, 0], [0, 0]]");
        expectSvdHolds(scaledMatrix<T>(2UZ, 2UZ, 1, {1, 1, 0, 0}), svd::Algorithm::Jacobi, {std::sqrt(2.0L), 0.0L}, "[[1, 1], [0, 0]]");
        expectSvdHolds(scaledMatrix<T>(2UZ, 3UZ, 1, {1, 0, 0, 0, 0, 0}), svd::Algorithm::Jacobi, {1.0L, 0.0L}, "[[1, 0, 0], [0, 0, 0]]");
    } | AllTypes{};
};

const boost::ut::suite<"GolubReinsch Internals -- Regression Tests"> _gr_internals = [] {
    using namespace boost::ut;
    using namespace boost::ut::literals;
    using gr::Tensor;
    using gr::math::detail::householderVector;
    using gr::math::detail::applyHouseholderLeft;
    using gr::math::detail::bidiagonalise;
    using gr::math::detail::accumulateU;
    using gr::math::detail::accumulateV;

    "Householder vector computation"_test = [] {
        "basic 3-element vector"_test = [] {
            std::vector<double> x = {3.0, 4.0, 0.0};
            double              tau;

            double beta = householderVector(x.data(), 3, tau);

            expect(approx_equal(beta, -5.0, 1e-10)) << "beta = " << beta;

            // Verify H*[3,4,0] = [beta, 0, 0]
            std::vector<double> orig = {3.0, 4.0, 0.0};
            double              v0 = 1.0, v1 = x[1], v2 = x[2];
            double              dot   = v0 * orig[0] + v1 * orig[1] + v2 * orig[2];
            double              scale = tau * dot;

            expect(approx_equal(orig[0] - scale * v0, beta, 1e-10));
            expect(approx_equal(orig[1] - scale * v1, 0.0, 1e-10));
            expect(approx_equal(orig[2] - scale * v2, 0.0, 1e-10));
        };

        "single element (no reflection needed)"_test = [] {
            std::vector<double> x = {5.0};
            double              tau;
            double              beta = householderVector(x.data(), 1, tau);

            expect(approx_equal(beta, 5.0, 1e-10));
            expect(approx_equal(tau, 0.0, 1e-10));
        };

        "negative first element"_test = [] {
            std::vector<double> x = {-3.0, 4.0};
            double              tau;
            double              beta = householderVector(x.data(), 2, tau);

            expect(approx_equal(beta, 5.0, 1e-10)) << "beta = " << beta;
        };
    };

    "the reflector maps x onto its first axis at any input scale"_test = []<typename T> {
        using RealT               = gr::meta::fundamental_base_value_type_t<T>;
        constexpr std::size_t n   = 5UZ;
        constexpr RealT       eps = std::numeric_limits<RealT>::epsilon();
        // At eps^2 the first entry of the reflector lies below eps. Below sqrt(denorm_min) the square of every entry underflows to zero.
        const std::array<RealT, 3>  scales{RealT{1}, eps * eps, std::sqrt(std::numeric_limits<RealT>::denorm_min()) / RealT{16}};
        const std::array<double, n> re{3.0, -1.0, 2.0, 0.5, -4.0};
        const std::array<double, n> im{1.0, 0.0, -2.0, 1.5, 0.5};

        for (const RealT scale : scales) {
            Tensor<T> x({n, 1UZ});
            RealT     unitNormSquared{0};
            for (std::size_t i = 0UZ; i < n; ++i) {
                if constexpr (gr::meta::complex_like<T>) {
                    x[i, 0UZ] = T{static_cast<RealT>(re[i]), static_cast<RealT>(im[i])};
                } else {
                    x[i, 0UZ] = static_cast<RealT>(re[i]);
                }
                unitNormSquared += static_cast<RealT>(std::norm(x[i, 0UZ]));
                x[i, 0UZ] *= scale;
            }
            const RealT norm = scale * std::sqrt(unitNormSquared);

            std::vector<T> v(n);
            for (std::size_t i = 0UZ; i < n; ++i) {
                v[i] = x[i, 0UZ];
            }
            RealT tau{0};
            std::ignore = householderVector(v.data(), n, tau);
            applyHouseholderLeft(x, v.data() + 1, tau, 0UZ, 0UZ, n - 1UZ); // H x with H = I - tau [1, v_tail] [1, v_tail]^H

            const RealT tolerance = RealT{16} * static_cast<RealT>(n) * eps;
            expect(le(std::abs(static_cast<RealT>(std::abs(x[0UZ, 0UZ])) - norm) / norm, tolerance)) << scale << "|(H x)_0| = ||x||";
            for (std::size_t i = 1UZ; i < n; ++i) {
                expect(le(static_cast<RealT>(std::abs(x[i, 0UZ])) / norm, tolerance)) << scale << "(H x)_" << i << "= 0";
            }
        }
    } | std::tuple<float, double, std::complex<float>, std::complex<double>>{};

    "Bidiagonalisation"_test = [] {
        "produces correct structure"_test = [] {
            Tensor<double> A({4, 3});
            A = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12};

            Tensor<double> A_orig(A);

            std::vector<double> tauU, tauV, diag, superdiag;
            bidiagonalise(A, tauU, tauV, diag, superdiag);

            expect(eq(diag.size(), 3UZ));
            expect(eq(superdiag.size(), 2UZ));
            expect(eq(tauU.size(), 3UZ));
            expect(eq(tauV.size(), 2UZ));

            // Accumulate U and V
            Tensor<double> U, V;
            accumulateU(A, tauU, U);
            accumulateV(A, tauV, V);

            // Build bidiagonal B
            Tensor<double> B({3, 3});
            B.fill(0.0);
            for (std::size_t i = 0; i < 3; ++i) {
                B[i, i] = diag[i];
            }
            for (std::size_t i = 0; i < 2; ++i) {
                B[i, i + 1] = superdiag[i];
            }

            // Verify U^T * A_orig * V ≈ B
            Tensor<double> UT = transpose(U);
            Tensor<double> UtA({3, 3});
            UtA.fill(0.0);
            gemm(UtA, UT, A_orig);

            Tensor<double> UtAV({3, 3});
            UtAV.fill(0.0);
            gemm(UtAV, UtA, V);

            expect(tensors_approximately_equal(UtAV, B, 1e-9));
        };

        "U and V are orthogonal"_test = [] {
            Tensor<double> A({4, 3});
            randomize(A, -10.0, 10.0);

            std::vector<double> tauU, tauV, diag, superdiag;
            bidiagonalise(A, tauU, tauV, diag, superdiag);

            Tensor<double> U, V;
            accumulateU(A, tauU, U);
            accumulateV(A, tauV, V);

            expect(verifyOrthogonal(U, 1e-9));
            expect(verifyOrthogonal(V, 1e-9));
        };

        "square 4x4 matrix produces correct structure"_test = [] {
            Tensor<double> A({4, 4});
            A = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16};

            Tensor<double> A_orig(A);

            std::vector<double> tauU, tauV, diag, superdiag;
            bidiagonalise(A, tauU, tauV, diag, superdiag);

            expect(eq(diag.size(), 4UZ));
            expect(eq(superdiag.size(), 3UZ));

            // Accumulate U and V
            Tensor<double> U, V;
            accumulateU(A, tauU, U);
            accumulateV(A, tauV, V);

            // Build bidiagonal B
            Tensor<double> B({4, 4});
            B.fill(0.0);
            for (std::size_t i = 0; i < 4; ++i) {
                B[i, i] = diag[i];
            }
            for (std::size_t i = 0; i < 3; ++i) {
                B[i, i + 1] = superdiag[i];
            }

            // Verify U^T * A_orig * V ≈ B
            Tensor<double> UT = transpose(U);
            Tensor<double> UtA({4, 4});
            UtA.fill(0.0);
            gemm(UtA, UT, A_orig);

            Tensor<double> UtAV({4, 4});
            UtAV.fill(0.0);
            gemm(UtAV, UtA, V);

            // Verify U^T * A * V = B (bidiagonal)
            expect(tensors_approximately_equal(UtAV, B, 1e-9));

            // Verify A = U * B * V^T (reconstruction)
            Tensor<double> UB({4, 4});
            UB.fill(0.0);
            gemm(UB, U, B);
            Tensor<double> UBVt({4, 4});
            UBVt.fill(0.0);
            Tensor<double> Vt = transpose(V);
            gemm(UBVt, UB, Vt);

            expect(tensors_approximately_equal(A_orig, UBVt, 1e-9));
        };
    };
};

int main() { /* not needed for boost-ut */ return 0; }
