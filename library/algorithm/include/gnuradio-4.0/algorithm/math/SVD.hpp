#ifndef GNURADIO_SVD_HPP
#define GNURADIO_SVD_HPP

#include <gnuradio-4.0/Tensor.hpp>
#include <gnuradio-4.0/algorithm/math/TensorMath.hpp>

#include <algorithm>
#include <cmath>
#include <concepts>
#include <limits>
#include <numeric>
#include <ranges>
#include <vector>

namespace gr::math {

namespace svd {
enum class Status : std::uint8_t {
    Success,
    EarlyReturn,   /// truncated to k singular values
    MaxIterations, /// did not converge within iteration limit
    InvalidInput,
    NumericalError
};

enum class Algorithm : std::uint8_t {
    Jacobi,       /// one-sided Jacobi iteration (robust, parallelisable, best for tall matrices)
    GolubReinsch, /// Householder bidiagonalisation + implicit QR (faster for large matrices)
    Auto          /// automatically select based on matrix characteristics
};

template<typename T>
struct Config {
    std::size_t maxIterations = 0;               /// 0 = auto (30 * max(m,n))
    T           tolerance     = T{-1};           /// <0 = auto (eps * max(m,n))
    std::size_t k             = 0;               /// 0 = all singular values, >0 = first k only
    Algorithm   algorithm     = Algorithm::Auto; /// defaults to numerically safe and then efficient choice
    bool        fullMatrices  = false;           /// false = thin SVD, true = U m×m and V n×n (k then limits S alone)
};
} // namespace svd

namespace detail {

template<typename T>
constexpr auto sign(T val) {
    if constexpr (gr::meta::complex_like<T>) {
        using BaseValueType = gr::meta::fundamental_base_value_type_t<T>;
        auto mag            = std::abs(val);
        return (mag > BaseValueType{0}) ? (val / mag) : T{1};
    } else {
        return (val > T{0}) ? T{1} : ((val < T{0}) ? T{-1} : T{0});
    }
}

template<typename T>
constexpr T innerProduct(const T& a, const T& b) {
    if constexpr (gr::meta::complex_like<T>) {
        return std::conj(a) * b;
    } else {
        return a * b;
    }
}

template<typename T>
constexpr T conj(const T& val) {
    if constexpr (gr::meta::complex_like<T>) {
        return std::conj(val);
    } else {
        return val;
    }
}

/// A real or complex floating-point element type.
template<typename T>
concept FloatingElement = std::floating_point<T> || (gr::meta::complex_like<T> && std::floating_point<gr::meta::fundamental_base_value_type_t<T>>);

/// A plane rotation J = [[c, s], [-conj(s), c]] with a real cosine c.
template<typename T, typename BaseValueType = gr::meta::fundamental_base_value_type_t<T>>
struct JacobiRotation {
    BaseValueType c;
    T             s;
    BaseValueType shift; /// J^H G J has alpha - shift and beta + shift on its diagonal
};

/**
 * @brief The Jacobi rotation that diagonalizes the Hermitian 2 x 2 matrix G = [[alpha, gamma], [conj(gamma), beta]].
 *
 * J^H G J has a zero off-diagonal entry. Of the two such rotations this returns the one with the
 * smaller angle, |t| <= 1 for t = |s| / c. For gamma = 0 it returns the identity. Every other pair
 * takes its angle from zeta = (beta - alpha) / (2 |gamma|), and alpha = beta gives 45 degrees.
 *
 * The angle depends only on the ratios of alpha, beta and |gamma|. When beta - alpha or 2 |gamma|
 * overflows, the function divides the three by a power of two near the largest of their magnitudes,
 * within the normal exponent range, and forms the angle from the scaled values. For |zeta| at or
 * above 2^(max_exponent / 2 - 1) the function takes t = 1 / (2 |zeta|), the limit of the full
 * formula, and computes it as (2 |gamma| / |beta - alpha|) / 2. That form needs neither zeta^2 nor
 * zeta, so t rounds to zero only below the smallest subnormal value. The shift t |gamma| has
 * |t| <= 1 and stays finite.
 */
template<typename T, typename BaseValueType = gr::meta::fundamental_base_value_type_t<T>>
[[nodiscard]] JacobiRotation<T, BaseValueType> jacobiRotation(BaseValueType alpha, BaseValueType beta, T gamma) noexcept {
    using limits               = std::numeric_limits<BaseValueType>;
    const BaseValueType absOff = std::abs(gamma);
    if (absOff == BaseValueType{0}) {
        return {BaseValueType{1}, T{0}, BaseValueType{0}};
    }
    BaseValueType difference = beta - alpha;
    BaseValueType twiceOff   = BaseValueType{2} * absOff;
    if (!std::isfinite(difference) || !std::isfinite(twiceOff)) {
        const int exponent = std::clamp(std::ilogb(std::max({std::abs(alpha), std::abs(beta), absOff})), limits::min_exponent - 1, limits::max_exponent - 1);
        difference         = std::ldexp(beta, -exponent) - std::ldexp(alpha, -exponent);
        twiceOff           = BaseValueType{2} * std::ldexp(absOff, -exponent);
    }

    const T             phase     = gamma / absOff; // +1 or -1 for a real element type
    const BaseValueType zeta      = difference / twiceOff;
    const BaseValueType magnitude = std::abs(zeta);
    const BaseValueType large     = std::ldexp(BaseValueType{1}, limits::max_exponent / 2 - 1);
    BaseValueType       t         = magnitude < large ? BaseValueType{1} / (magnitude + std::sqrt(BaseValueType{1} + zeta * zeta)) : (twiceOff / std::abs(difference)) / BaseValueType{2};
    if (zeta < BaseValueType{0}) {
        t = -t;
    }
    const BaseValueType c = BaseValueType{1} / std::sqrt(BaseValueType{1} + t * t);
    return {c, phase * (c * t), t * absOff};
}

/// Multiplies columns i and j of the first `rows` rows of M by the rotation [[c, s], [-conj(s), c]] from the right.
template<typename T, typename BaseValueType = gr::meta::fundamental_base_value_type_t<T>>
void rotateColumns(Tensor<T>& M, std::size_t i, std::size_t j, BaseValueType c, T s, std::size_t rows) noexcept {
    for (std::size_t k = 0; k < rows; ++k) {
        const T mi = M[k, i];
        const T mj = M[k, j];
        M[k, i]    = c * mi - conj(s) * mj;
        M[k, j]    = s * mi + c * mj;
    }
}

/**
 * @brief Replaces the columns of Q not marked in `keep` by an orthonormal completion of the marked columns.
 *
 * The marked columns must be orthonormal. Every other column becomes a unit vector orthogonal to the marked columns
 * and to the replaced columns before it. A replaced column starts from its own direction when that direction keeps at
 * least half its length after the projection. Otherwise it starts from the unit vector e_r whose row r has the least
 * weight in the columns accepted so far. Each start vector is orthogonalized in two Gram-Schmidt passes. Q needs at
 * least as many rows as columns.
 */
template<typename T, typename BaseValueType = gr::meta::fundamental_base_value_type_t<T>>
void completeOrthonormal(Tensor<T>& Q, const std::vector<bool>& keep) {
    const std::size_t rows = Q.extent(0);
    const std::size_t cols = Q.extent(1);

    std::vector<std::size_t>   accepted;
    std::vector<BaseValueType> rowWeight(rows, BaseValueType{0});
    auto                       accept = [&](std::size_t j) {
        accepted.push_back(j);
        for (std::size_t r = 0; r < rows; ++r) {
            rowWeight[r] += squaredMagnitude(Q[r, j]);
        }
    };
    for (std::size_t j = 0; j < cols; ++j) {
        if (keep[j]) {
            accept(j);
        }
    }

    std::vector<T> candidate(rows);
    auto           orthogonalize = [&] {
        for (int pass = 0; pass < 2; ++pass) {
            for (std::size_t b : accepted) {
                T dot{0};
                for (std::size_t r = 0; r < rows; ++r) {
                    dot += innerProduct(Q[r, b], candidate[r]);
                }
                for (std::size_t r = 0; r < rows; ++r) {
                    candidate[r] -= dot * Q[r, b];
                }
            }
        }
        BaseValueType normSq{0};
        for (const T& value : candidate) {
            normSq += squaredMagnitude(value);
        }
        return std::sqrt(normSq);
    };

    for (std::size_t j = 0; j < cols; ++j) {
        if (keep[j]) {
            continue;
        }
        BaseValueType ownNormSq{0};
        for (std::size_t r = 0; r < rows; ++r) {
            ownNormSq += squaredMagnitude(Q[r, j]);
        }
        const BaseValueType ownNorm = std::sqrt(ownNormSq);
        BaseValueType       norm{0};
        if (ownNorm > BaseValueType{0}) {
            for (std::size_t r = 0; r < rows; ++r) {
                candidate[r] = Q[r, j] / ownNorm;
            }
            norm = orthogonalize();
        }
        if (!(norm >= BaseValueType{1} / BaseValueType{2})) {
            const auto lightest = static_cast<std::size_t>(std::ranges::min_element(rowWeight) - rowWeight.begin());
            std::ranges::fill(candidate, T{0});
            candidate[lightest] = T{1};
            norm                = orthogonalize();
        }
        for (std::size_t r = 0; r < rows; ++r) {
            Q[r, j] = norm > BaseValueType{0} ? candidate[r] / norm : T{0};
        }
        accept(j);
    }
}

// one-sided Jacobi SVD: works best when m >= n (tall matrices)
// A is modified in-place to become U, V and singularValues are outputs
template<typename T, typename BaseValueType = gr::meta::fundamental_base_value_type_t<T>>
svd::Status svdJacobi(Tensor<T>& A, Tensor<T>* V, Tensor<BaseValueType>& singularValues, std::size_t maxIter, BaseValueType tol, bool computeV) {
    const std::size_t m = A.extent(0);
    const std::size_t n = A.extent(1);

    if (n == 0 || m == 0) {
        if (computeV && V) {
            V->resize({n, 0});
        }
        singularValues.resize({0});
        return svd::Status::Success;
    }

    // initialise V as identity if needed
    if (computeV && V) {
        V->resize({n, n});
        V->fill(T{0});
        for (std::size_t i = 0; i < n; ++i) {
            (*V)[i, i] = T{1};
        }
    }

    // A column whose squared norm is at most (eps ||A||_F)^2 is negligible and takes part in no rotation.
    constexpr BaseValueType eps = std::numeric_limits<BaseValueType>::epsilon();
    BaseValueType           frobeniusSq{0};
    for (const T& value : A) {
        frobeniusSq += squaredMagnitude(value);
    }
    const BaseValueType negligible = eps * eps * frobeniusSq;

    std::size_t iter      = 0;
    bool        converged = false;

    // main Jacobi iteration
    while (iter < maxIter && !converged) {
        converged = true;

        for (std::size_t i = 0; i < n - 1; ++i) {
            for (std::size_t j = i + 1; j < n; ++j) {
                // compute a_i^H * a_j and ||a_i||^2, ||a_j||^2
                T             dot_ij{0};
                BaseValueType dot_ii{0};
                BaseValueType dot_jj{0};

                for (std::size_t k = 0; k < m; ++k) {
                    dot_ij += innerProduct(A[k, i], A[k, j]);
                    dot_ii += squaredMagnitude(A[k, i]);
                    dot_jj += squaredMagnitude(A[k, j]);
                }

                if (dot_ii <= negligible || dot_jj <= negligible) {
                    continue;
                }
                // the pair is orthogonal enough when |a_i^H a_j| <= tol ||a_i|| ||a_j||
                const BaseValueType threshold = tol * std::sqrt(dot_ii) * std::sqrt(dot_jj);

                if (std::abs(dot_ij) > threshold) {
                    converged = false;

                    // orthogonalize columns i and j: the rotation that diagonalizes their 2 x 2 Gram matrix
                    const auto rotation = jacobiRotation(dot_ii, dot_jj, dot_ij);
                    rotateColumns(A, i, j, rotation.c, rotation.s, m);
                    if (computeV && V) {
                        rotateColumns(*V, i, j, rotation.c, rotation.s, n);
                    }
                }
            }
        }

        ++iter;
    }

    // the column norms are the singular values; each column of nonzero norm is scaled to unit length
    singularValues.resize({n});
    BaseValueType largest{0};
    for (std::size_t j = 0; j < n; ++j) {
        BaseValueType norm{0};
        for (std::size_t i = 0; i < m; ++i) {
            norm += squaredMagnitude(A[i, j]);
        }
        singularValues[j] = std::sqrt(norm);
        largest           = std::max(largest, singularValues[j]);
        if (singularValues[j] > BaseValueType{0}) {
            for (std::size_t i = 0; i < m; ++i) {
                A[i, j] /= singularValues[j];
            }
        }
    }

    // the columns whose singular value is at most max(m, n) eps sigma_max are made orthonormal to the others
    if (computeV) {
        const BaseValueType noiseLevel = static_cast<BaseValueType>(std::max(m, n)) * eps * largest;
        std::vector<bool>   keep(n);
        for (std::size_t j = 0; j < n; ++j) {
            keep[j] = singularValues[j] > noiseLevel;
        }
        completeOrthonormal(A, keep);
    }

    return converged ? svd::Status::Success : svd::Status::MaxIterations;
}

// ============================================================================
// Golub-Reinsch SVD: Householder bidiagonalisation + implicit QR iteration
// ============================================================================

template<typename T, typename BaseValueType = gr::meta::fundamental_base_value_type_t<T>>
BaseValueType householderVector(T* x, std::size_t n, BaseValueType& tau) {
    // compute Householder vector v and scalar tau such that:
    // H = I - tau * v * v^H applies H * x = [beta; 0; 0; ...]
    // x[0] is overwritten with beta and x[1:n] with the tail of v (v[0] = 1 is implicit)
    // returns beta for real x and |beta| for complex x

    if (n == 0) {
        tau = BaseValueType{0};
        return BaseValueType{0};
    }
    if (n == 1) {
        tau = BaseValueType{0};
        if constexpr (gr::meta::complex_like<T>) {
            return std::abs(x[0]);
        } else {
            return static_cast<BaseValueType>(x[0]);
        }
    }

    // The reflector is formed from x * down, where down is the power of two that brings the largest magnitude into
    // [1, 2), or to eps or above for a subnormal one. The norms, beta and v0 stay finite at any input scale, the
    // square of the largest magnitude does not underflow, and the scaling itself is exact. Only beta returns to the
    // input scale.
    BaseValueType maxMagnitude{0};
    for (std::size_t i = 0; i < n; ++i) {
        maxMagnitude = std::max(maxMagnitude, static_cast<BaseValueType>(std::abs(x[i])));
    }
    const int           exponent = std::clamp(std::ilogb(maxMagnitude), std::numeric_limits<BaseValueType>::min_exponent - 1, std::numeric_limits<BaseValueType>::max_exponent - 1);
    const BaseValueType down     = std::ldexp(BaseValueType{1}, -exponent);
    const BaseValueType up       = std::ldexp(BaseValueType{1}, exponent);

    // compute ||x[1:n] * down||^2
    BaseValueType xnorm_sq{0};
    for (std::size_t i = 1; i < n; ++i) {
        xnorm_sq += squaredMagnitude(x[i] * down);
    }

    if (xnorm_sq == BaseValueType{0}) {
        // x is already [alpha, 0, 0, ...], no reflection needed
        tau = BaseValueType{0};
        if constexpr (gr::meta::complex_like<T>) {
            return std::abs(x[0]);
        } else {
            return static_cast<BaseValueType>(x[0]);
        }
    }

    // compute the scaled norm of the entire vector: ||x|| * down = sqrt(|x[0] * down|^2 + xnorm_sq)
    const T       x0        = x[0] * down;
    BaseValueType x0_mag_sq = squaredMagnitude(x0);
    BaseValueType xnorm     = std::sqrt(x0_mag_sq + xnorm_sq);

    // choose sign to avoid cancellation: beta = -sign(x[0]) * ||x||, here scaled by down
    T beta;
    if constexpr (gr::meta::complex_like<T>) {
        // for complex: use phase of x[0]
        if (std::abs(x0) > BaseValueType{0}) {
            T phase = x0 / std::abs(x0);
            beta    = -phase * xnorm;
        } else {
            beta = T{-xnorm, 0};
        }
    } else {
        beta = (x0 >= BaseValueType{0}) ? -xnorm : xnorm;
    }

    // v[0] = x[0] - beta, v[1:n] = x[1:n], all scaled by down
    T             v0        = x0 - beta;
    BaseValueType v0_mag_sq = squaredMagnitude(v0);
    BaseValueType vnorm_sq  = v0_mag_sq + xnorm_sq;

    // for normalized vector v' = [1, x[1]/v0, ...]:
    // H = I - tau * v * v^H = I - tau' * v' * v'^H where tau' = tau * |v0|^2
    // tau = 2 / ||v||^2, so tau' = 2 * |v0|^2 / ||v||^2
    tau = BaseValueType{2} * v0_mag_sq / vnorm_sq;

    // store normalized tail: x[1:n] /= v0, the same ratio in scaled coordinates
    // |v0| >= ||x * down|| >= eps, so its reciprocal is finite
    T scale = T{1} / v0;
    for (std::size_t i = 1; i < n; ++i) {
        x[i] = (x[i] * down) * scale;
    }

    // x[0] receives beta at the input scale, the first entry of H * x
    beta = beta * up;
    x[0] = beta;

    // for complex: return |beta| (the bidiagonal will be real)
    // for real: return beta with sign (handle signs during SVD)
    if constexpr (gr::meta::complex_like<T>) {
        return std::abs(beta);
    } else {
        return beta;
    }
}

template<typename T, typename BaseValueType = gr::meta::fundamental_base_value_type_t<T>>
void applyHouseholderLeft(Tensor<T>& A, const T* vTail, BaseValueType tau, std::size_t rowStart, std::size_t colStart, std::size_t tailLen) {
    // performs: A[rowStart:, colStart:] = H * A[rowStart:, colStart:]
    // where H = I - tau * v * v^H
    // v has implicit first element v[0] = 1, followed by tailLen elements in vTail

    if (tau == BaseValueType{0}) {
        return;
    }

    const std::size_t m = A.extent(0);
    const std::size_t n = A.extent(1);

    // for each column j >= colStart:
    // A[:, j] -= tau * v * (v^H * A[:, j])
    for (std::size_t j = colStart; j < n; ++j) {
        // compute dot = v^H * A[rowStart:, j]
        T dot = A[rowStart, j]; // v[0] = 1 (implicit)
        for (std::size_t i = 0; i < tailLen && (rowStart + 1 + i) < m; ++i) {
            dot += conj(vTail[i]) * A[rowStart + 1 + i, j];
        }

        // A[rowStart:, j] -= tau * dot * v
        T scale = tau * dot;
        A[rowStart, j] -= scale; // v[0] = 1
        for (std::size_t i = 0; i < tailLen && (rowStart + 1 + i) < m; ++i) {
            A[rowStart + 1 + i, j] -= scale * vTail[i];
        }
    }
}

template<typename T, typename BaseValueType = gr::meta::fundamental_base_value_type_t<T>>
void applyHouseholderRight(Tensor<T>& A, const T* vTail, BaseValueType tau, std::size_t rowStart, std::size_t rowEnd, std::size_t colStart, std::size_t tailLen) {
    // performs: A[rowStart:rowEnd, colStart:] = A[rowStart:rowEnd, colStart:] * H
    // where H = I - tau * v * v^H
    // v has implicit first element v[0] = 1, followed by tailLen elements in vTail

    if (tau == BaseValueType{0}) {
        return;
    }

    const std::size_t n = A.extent(1);

    // for each row i in [rowStart, rowEnd):
    // A[i, :] -= tau * (A[i, :] * v) * v^H
    for (std::size_t i = rowStart; i < rowEnd; ++i) {
        // compute dot = A[i, colStart:] * v
        T dot = A[i, colStart]; // v[0] = 1 (implicit)
        for (std::size_t j = 0; j < tailLen && (colStart + 1 + j) < n; ++j) {
            dot += A[i, colStart + 1 + j] * vTail[j];
        }

        // A[i, colStart:] -= tau * dot * v^H
        T scale = tau * dot;
        A[i, colStart] -= scale; // v[0] = 1
        for (std::size_t j = 0; j < tailLen && (colStart + 1 + j) < n; ++j) {
            A[i, colStart + 1 + j] -= scale * conj(vTail[j]);
        }
    }
}

template<typename T, typename BaseValueType = gr::meta::fundamental_base_value_type_t<T>>
void bidiagonalise(Tensor<T>& A, std::vector<BaseValueType>& tauU, std::vector<BaseValueType>& tauV, std::vector<BaseValueType>& diag, std::vector<BaseValueType>& superdiag) {
    // bidiagonalise A in-place: A -> U^H * A * V = B (upper bidiagonal)
    // stores Householder vectors in-place, tauU and tauV store the tau values
    // diag and superdiag store the real bidiagonal elements

    const std::size_t m      = A.extent(0);
    const std::size_t n      = A.extent(1);
    const std::size_t minDim = std::min(m, n);

    tauU.resize(minDim);
    tauV.resize(minDim > 1 ? minDim - 1 : 0);
    diag.resize(minDim);
    superdiag.resize(minDim > 1 ? minDim - 1 : 0);

    std::vector<T> work(std::max(m, n));

    for (std::size_t k = 0; k < minDim; ++k) {
        // left Householder: eliminate A[k+1:m, k]
        std::size_t colLen = m - k;
        for (std::size_t i = 0; i < colLen; ++i) {
            work[i] = A[k + i, k];
        }

        BaseValueType tauL;
        BaseValueType beta = householderVector(work.data(), colLen, tauL);
        tauU[k]            = tauL;
        diag[k]            = beta;

        // store Householder vector back (skip first element, it's implicit 1)
        A[k, k] = work[0]; // beta of the left reflector
        for (std::size_t i = 1; i < colLen; ++i) {
            A[k + i, k] = work[i];
        }

        // apply H_L to remaining columns A[k:m, k+1:n]
        if (k + 1 < n && colLen > 1) {
            applyHouseholderLeft(A, work.data() + 1, tauL, k, k + 1, colLen - 1);
        }

        // right Householder: eliminate A[k, k+2:n]
        if (k + 1 < n) {
            std::size_t rowLen = n - k - 1;
            for (std::size_t j = 0; j < rowLen; ++j) {
                work[j] = A[k, k + 1 + j];
            }

            BaseValueType tauR;
            BaseValueType gamma = householderVector(work.data(), rowLen, tauR);
            if (k < tauV.size()) {
                tauV[k]      = tauR;
                superdiag[k] = gamma;
            }

            // store Householder vector back
            A[k, k + 1] = work[0]; // beta of the right reflector
            for (std::size_t j = 1; j < rowLen; ++j) {
                A[k, k + 1 + j] = work[j];
            }

            // apply H_R to remaining rows A[k+1:m, k+1:n]
            if (k + 1 < m && rowLen > 1) {
                applyHouseholderRight(A, work.data() + 1, tauR, k + 1, m, k + 1, rowLen - 1);
            }
        }
    }
}

template<typename T, typename BaseValueType = gr::meta::fundamental_base_value_type_t<T>>
bool bidiagonalQRStep(std::vector<BaseValueType>& diag, std::vector<BaseValueType>& superdiag, Tensor<T>* U, Tensor<T>* V, std::size_t start, std::size_t end, BaseValueType tol) {
    // one implicit QR step on bidiagonal matrix B with Wilkinson shift
    // B is stored as diag (length n) and superdiag (length n-1)
    // returns true if converged (superdiag[end-1] became negligible)

    if (end <= start + 1) {
        return true;
    }

    // The shift and the first bulge are formed from the active block's entries times `down`, the power of two that
    // brings the block's largest magnitude into [1, 2), or as near as the exponent range allows when that magnitude is
    // subnormal. Their squares stay finite at any input scale. The first rotation depends only on the ratio of x and z.
    BaseValueType largest{0};
    for (std::size_t i = start; i < end; ++i) {
        largest = std::max(largest, std::abs(diag[i]));
        if (i + 1 < end) {
            largest = std::max(largest, std::abs(superdiag[i]));
        }
    }
    if (largest == BaseValueType{0}) {
        return true;
    }
    const int           exponent = std::clamp(std::ilogb(largest), std::numeric_limits<BaseValueType>::min_exponent - 1, std::numeric_limits<BaseValueType>::max_exponent - 1);
    const BaseValueType down     = std::ldexp(BaseValueType{1}, -exponent);

    // Wilkinson shift: eigenvalue of trailing 2x2 of B^T * B closest to last diagonal
    BaseValueType d_nm1 = diag[end - 1] * down;
    BaseValueType d_nm2 = ((end >= 2) ? diag[end - 2] : BaseValueType{0}) * down;
    BaseValueType e_nm2 = ((end >= 2 && start < end - 1) ? superdiag[end - 2] : BaseValueType{0}) * down;
    BaseValueType e_nm3 = ((end >= 3 && start < end - 2) ? superdiag[end - 3] : BaseValueType{0}) * down;

    // elements of B^T * B trailing 2x2
    BaseValueType a11 = d_nm2 * d_nm2 + e_nm3 * e_nm3;
    BaseValueType a12 = d_nm2 * e_nm2;
    BaseValueType a22 = d_nm1 * d_nm1 + e_nm2 * e_nm2;

    // Wilkinson shift: eigenvalue of [[a11, a12], [a12, a22]] closest to a22
    BaseValueType delta = (a11 - a22) / BaseValueType{2};
    BaseValueType shift;
    if (delta == BaseValueType{0}) {
        shift = a22 - std::abs(a12);
    } else {
        BaseValueType sign_delta = (delta > BaseValueType{0}) ? BaseValueType{1} : BaseValueType{-1};
        shift                    = a22 - a12 * a12 / (delta + sign_delta * std::sqrt(delta * delta + a12 * a12));
    }

    // initial Givens to introduce bulge
    BaseValueType x = (diag[start] * down) * (diag[start] * down) - shift;
    BaseValueType z = (diag[start] * down) * (superdiag[start] * down);

    // chase bulge down the bidiagonal
    for (std::size_t k = start; k < end - 1; ++k) {
        // Right Givens: zero out z using rotation on columns k and k+1
        BaseValueType c, s, r;
        gr::math::givens(x, z, c, s, r);

        // For k > start, the bulge from previous iteration is at (k-1, k)
        // The right Givens zeros it: superdiag[k-1] = r
        if (k > start) {
            superdiag[k - 1] = r;
        }

        // Apply right Givens to bidiagonal: B := B * G^T
        // G^T = [[c, -s], [s, c]] applied to columns k, k+1
        BaseValueType d_k  = diag[k];
        BaseValueType e_k  = superdiag[k];
        BaseValueType d_k1 = diag[k + 1];

        // New values after B * G^T:
        // B[k,k] = c * d_k + s * e_k
        // B[k,k+1] = -s * d_k + c * e_k
        // B[k+1,k] = s * d_k1  (bulge created!)
        // B[k+1,k+1] = c * d_k1
        diag[k]             = c * d_k + s * e_k;
        superdiag[k]        = -s * d_k + c * e_k;
        BaseValueType bulge = s * d_k1; // the bulge at position (k+1, k)
        diag[k + 1]         = c * d_k1;

        // accumulate V: V := V * G^T
        if (V) {
            gr::math::applyGivensRight<BaseValueType>(*V, k, k + 1, c, s, 0, V->extent(0));
        }

        // Left Givens: zero out bulge at (k+1, k)
        // Rotate rows k and k+1 to zero bulge
        gr::math::givens(diag[k], bulge, c, s, r);

        // Apply left Givens to bidiagonal: B := G * B
        // G = [[c, s], [-s, c]] applied to rows k, k+1
        diag[k] = r;

        // Row k: [r, superdiag[k], 0, ...]
        // Row k+1: [0, diag[k+1], superdiag[k+1], ...]
        // After G * B:
        // New B[k, k+1] = c * superdiag[k] + s * diag[k+1]
        // New B[k+1, k+1] = -s * superdiag[k] + c * diag[k+1]
        // New B[k, k+2] = s * superdiag[k+1]  (new bulge if k+2 < end!)
        // New B[k+1, k+2] = c * superdiag[k+1]
        BaseValueType old_e  = superdiag[k];
        BaseValueType old_d1 = diag[k + 1];

        superdiag[k] = c * old_e + s * old_d1;
        diag[k + 1]  = -s * old_e + c * old_d1;

        // Set up for next iteration
        if (k + 2 < end) {
            // New bulge appears at (k, k+2) from left Givens
            z                = s * superdiag[k + 1];
            superdiag[k + 1] = c * superdiag[k + 1];
            x                = superdiag[k]; // for next right Givens
        }

        // accumulate U: U := U * G
        if (U) {
            gr::math::applyGivensRight<BaseValueType>(*U, k, k + 1, c, s, 0, U->extent(0));
        }
    }

    // check convergence: is superdiag[end-2] negligible?
    BaseValueType off    = std::abs(superdiag[end - 2]);
    BaseValueType thresh = tol * (std::abs(diag[end - 2]) + std::abs(diag[end - 1]));
    return off <= thresh;
}

template<typename T, typename BaseValueType = gr::meta::fundamental_base_value_type_t<T>>
void accumulateU(const Tensor<T>& A, const std::vector<BaseValueType>& tauU, Tensor<T>& U) {
    // accumulate U from stored Householder vectors (left transformations)

    const std::size_t m      = A.extent(0);
    const std::size_t n      = A.extent(1);
    const std::size_t minDim = std::min(m, n);

    // initialize U as identity
    U.resize({m, minDim});
    U.fill(T{0});
    for (std::size_t i = 0; i < minDim; ++i) {
        U[i, i] = T{1};
    }

    // apply Householder transformations in reverse order
    std::vector<T> v(m);
    for (std::size_t k = minDim; k-- > 0;) {
        std::size_t vLen = m - k;

        // reconstruct Householder vector
        v[0] = T{1}; // implicit
        for (std::size_t i = 1; i < vLen; ++i) {
            v[i] = A[k + i, k];
        }

        // U = H_k * U = (I - tau * v * v^H) * U
        // for each column j of U
        for (std::size_t j = k; j < minDim; ++j) {
            T dot{0};
            for (std::size_t i = 0; i < vLen; ++i) {
                dot += conj(v[i]) * U[k + i, j];
            }
            T scale = tauU[k] * dot;
            for (std::size_t i = 0; i < vLen; ++i) {
                U[k + i, j] -= scale * v[i];
            }
        }
    }
}

template<typename T, typename BaseValueType = gr::meta::fundamental_base_value_type_t<T>>
void accumulateV(const Tensor<T>& A, const std::vector<BaseValueType>& tauV, Tensor<T>& V) {
    // accumulate V from stored Householder vectors (right transformations)

    const std::size_t m      = A.extent(0);
    const std::size_t n      = A.extent(1);
    const std::size_t minDim = std::min(m, n);

    // initialize V as identity
    V.resize({n, minDim});
    V.fill(T{0});
    for (std::size_t i = 0; i < minDim; ++i) {
        V[i, i] = T{1};
    }

    if (minDim <= 1) {
        return;
    }

    // apply Householder transformations in forward order (H_0, H_1, ..., H_{n-2})
    // bidiagonalise applies: A := A * H_0 * H_1 * ... * H_{n-2}, so V = H_0 * H_1 * ... * H_{n-2}
    std::vector<T> v(n);
    for (std::size_t k = 0; k < tauV.size(); ++k) {
        std::size_t vLen = n - k - 1;
        if (vLen == 0) {
            continue;
        }

        // reconstruct Householder vector
        v[0] = T{1}; // implicit
        for (std::size_t j = 1; j < vLen; ++j) {
            v[j] = A[k, k + 1 + j];
        }

        // V = V * H_k = V * (I - tau * v * v^H)
        // for each row i of V
        for (std::size_t i = 0; i < n; ++i) {
            T dot{0};
            for (std::size_t j = 0; j < vLen; ++j) {
                dot += V[i, k + 1 + j] * v[j];
            }
            T scale = tauV[k] * dot;
            for (std::size_t j = 0; j < vLen; ++j) {
                V[i, k + 1 + j] -= scale * conj(v[j]);
            }
        }
    }
}

template<typename T, typename BaseValueType = gr::meta::fundamental_base_value_type_t<T>>
svd::Status svdGolubReinsch(Tensor<T>& A, Tensor<T>* V, Tensor<BaseValueType>& singularValues, std::size_t maxIter, BaseValueType tol, bool computeV) {
    const std::size_t m      = A.extent(0);
    const std::size_t n      = A.extent(1);
    const std::size_t minDim = std::min(m, n);

    if (minDim == 0) {
        singularValues.resize({0});
        if (computeV && V) {
            V->resize({n, 0});
        }
        return svd::Status::Success;
    }

    // step 1: bidiagonalise A
    std::vector<BaseValueType> tauU, tauV;
    std::vector<BaseValueType> diag, superdiag;
    bidiagonalise(A, tauU, tauV, diag, superdiag);

    // step 2: accumulate U and V if needed
    Tensor<T> Umat, Vmat;
    accumulateU(A, tauU, Umat);
    if (computeV) {
        accumulateV(A, tauV, Vmat);
    }

    // Both zero tests below are floored at 6 minDim^2 times the smallest normal value. A block of rounding residue
    // then deflates even where its own entries are subnormal. A block of normal entries far below the largest one
    // keeps its relative tests.
    const auto          floorSize = static_cast<BaseValueType>(minDim);
    const BaseValueType zeroFloor = BaseValueType{6} * floorSize * floorSize * std::numeric_limits<BaseValueType>::min();
    // The relative zero test never exceeds max(m, n) eps. A larger `tol` loosens only the deflation of the
    // superdiagonal.
    const BaseValueType zeroTol = std::min(tol, std::numeric_limits<BaseValueType>::epsilon() * static_cast<BaseValueType>(std::max(m, n)));

    // step 3: implicit QR iteration on bidiagonal
    std::size_t iter = 0;
    std::size_t end  = minDim;

    while (end > 1 && iter < maxIter) {
        // find largest unreduced submatrix
        std::size_t start = 0;

        // check for negligible superdiagonal elements (deflation)
        for (std::size_t i = end - 1; i > 0; --i) {
            BaseValueType off    = std::abs(superdiag[i - 1]);
            BaseValueType thresh = std::max(tol * (std::abs(diag[i - 1]) + std::abs(diag[i])), zeroFloor);
            if (off <= thresh) {
                superdiag[i - 1] = BaseValueType{0};
                if (i == end - 1) {
                    --end; // deflate from bottom
                } else {
                    start = i; // found unreduced block
                    break;
                }
            }
        }

        if (end <= 1) {
            break;
        }

        // A diagonal entry of the active block [start, end) counts as zero when it is at most zeroTol times the
        // block's largest entry. It is set to zero and its row or column is cleared by Givens rotations.
        BaseValueType blockLargest{0};
        for (std::size_t i = start; i < end; ++i) {
            blockLargest = std::max(blockLargest, std::abs(diag[i]));
            if (i + 1 < end) {
                blockLargest = std::max(blockLargest, std::abs(superdiag[i]));
            }
        }
        std::size_t zeroAt = end;
        for (std::size_t i = start; i < end; ++i) {
            if (std::abs(diag[i]) <= std::max(zeroTol * blockLargest, zeroFloor)) {
                zeroAt = i;
                break;
            }
        }

        if (zeroAt < end) {
            diag[zeroAt] = BaseValueType{0};
            if (zeroAt + 1 < end) {
                // chase superdiag[zeroAt] right along row zeroAt: rotations of rows (j, zeroAt), applied to U
                for (std::size_t j = zeroAt + 1; j < end; ++j) {
                    BaseValueType c, s, r;
                    gr::math::givens(diag[j], superdiag[zeroAt], c, s, r);
                    diag[j] = r;
                    if (j + 1 < end) {
                        const BaseValueType next = superdiag[j];
                        superdiag[j]             = c * next;
                        superdiag[zeroAt]        = -s * next;
                    } else {
                        superdiag[zeroAt] = BaseValueType{0};
                    }
                    gr::math::applyGivensRight<BaseValueType>(Umat, j, zeroAt, c, s, 0, m);
                }
            } else {
                // chase superdiag[end - 2] up column end - 1: rotations of columns (k, end - 1), applied to V
                BaseValueType bulge = superdiag[end - 2];
                superdiag[end - 2]  = BaseValueType{0};
                for (std::size_t k = end - 1; k-- > start;) {
                    BaseValueType c, s, r;
                    gr::math::givens(diag[k], bulge, c, s, r);
                    diag[k] = r;
                    if (k > start) {
                        bulge            = -s * superdiag[k - 1];
                        superdiag[k - 1] = c * superdiag[k - 1];
                    }
                    if (computeV) {
                        gr::math::applyGivensRight<BaseValueType>(Vmat, k, end - 1, c, s, 0, Vmat.extent(0));
                    }
                }
            }
            ++iter;
            continue;
        }

        // perform one QR step
        bidiagonalQRStep<T>(diag, superdiag, &Umat, computeV ? &Vmat : nullptr, start, end, tol);
        ++iter;
    }

    // step 4: ensure singular values are positive and sort
    for (std::size_t i = 0; i < minDim; ++i) {
        if (diag[i] < BaseValueType{0}) {
            diag[i] = -diag[i];
            // flip sign of corresponding column in Umat
            for (std::size_t j = 0; j < m; ++j) {
                Umat[j, i] = -Umat[j, i];
            }
        }
    }

    // copy results
    singularValues.resize({minDim});
    for (std::size_t i = 0; i < minDim; ++i) {
        singularValues[i] = diag[i];
    }

    A = std::move(Umat);
    if (computeV && V) {
        *V = std::move(Vmat);
    }

    return (iter < maxIter) ? svd::Status::Success : svd::Status::MaxIterations;
}

template<typename BaseValueType, typename T>
void sortSingularValues(Tensor<BaseValueType>& S, Tensor<T>* U, Tensor<T>* V) {
    const std::size_t n = S.size();
    if (n == 0) {
        return;
    }

    std::vector<std::size_t> indices(n);
    std::iota(indices.begin(), indices.end(), 0);

    std::ranges::sort(indices, [&S](std::size_t i, std::size_t j) { return S[i] > S[j]; });

    Tensor<BaseValueType> sortedS({n});
    for (std::size_t i = 0; i < n; ++i) {
        sortedS[i] = S[indices[i]];
    }
    S = std::move(sortedS);

    if (U && U->size() > 0 && U->extent(1) >= n) {
        Tensor<T> sortedU({U->extent(0), U->extent(1)});
        for (std::size_t row = 0; row < U->extent(0); ++row) {
            for (std::size_t col = 0; col < n; ++col) {
                sortedU[row, col] = (*U)[row, indices[col]];
            }
            for (std::size_t col = n; col < U->extent(1); ++col) {
                sortedU[row, col] = (*U)[row, col];
            }
        }
        *U = std::move(sortedU);
    }

    if (V && V->size() > 0 && V->extent(1) >= n) {
        Tensor<T> sortedV({V->extent(0), V->extent(1)});
        for (std::size_t row = 0; row < V->extent(0); ++row) {
            for (std::size_t col = 0; col < n; ++col) {
                sortedV[row, col] = (*V)[row, indices[col]];
            }
            for (std::size_t col = n; col < V->extent(1); ++col) {
                sortedV[row, col] = (*V)[row, col];
            }
        }
        *V = std::move(sortedV);
    }
}

/**
 * @brief Multiplies A by 2^-e and returns e.
 *
 * 2^-e brings the largest real or imaginary part of A into [1, 2), or as near as the exponent range allows. The scaling
 * is exact for every entry that stays in the normal range. A zero or non-finite largest part leaves A unchanged and
 * returns 0.
 */
template<typename T, typename BaseValueType = gr::meta::fundamental_base_value_type_t<T>>
[[nodiscard]] int scaleToUnitMagnitude(Tensor<T>& A) {
    BaseValueType largest{0};
    for (const T& value : A) {
        if constexpr (gr::meta::complex_like<T>) {
            largest = std::max({largest, static_cast<BaseValueType>(std::abs(value.real())), static_cast<BaseValueType>(std::abs(value.imag()))});
        } else {
            largest = std::max(largest, static_cast<BaseValueType>(std::abs(value)));
        }
    }
    if (!(largest > BaseValueType{0}) || !std::isfinite(largest)) {
        return 0;
    }
    const int           exponent = std::clamp(std::ilogb(largest), std::numeric_limits<BaseValueType>::min_exponent - 1, std::numeric_limits<BaseValueType>::max_exponent - 1);
    const BaseValueType down     = std::ldexp(BaseValueType{1}, -exponent);
    for (T& value : A) {
        value *= down;
    }
    return exponent;
}

/// Extends the orthonormal columns of Q by an orthonormal completion to `columns` columns.
template<typename T>
void extendOrthonormal(Tensor<T>& Q, std::size_t columns) {
    const std::size_t rows = Q.extent(0);
    const std::size_t have = Q.extent(1);
    if (have >= columns) {
        return;
    }
    Tensor<T> full({rows, columns});
    full.fill(T{0});
    std::vector<bool> keep(columns, false);
    for (std::size_t j = 0; j < have; ++j) {
        keep[j] = true;
        for (std::size_t i = 0; i < rows; ++i) {
            full[i, j] = Q[i, j];
        }
    }
    completeOrthonormal(full, keep);
    Q = std::move(full);
}

} // namespace detail

/**
 * @brief singular value decomposition: A = U * diag(S) * V^H
 *
 * computes the thin SVD by default (U is m×min(m,n), V is n×min(m,n))
 * set config.fullMatrices = true for full SVD
 *
 * @param policy execution policy (cpu_policy, cpu_parallel_policy)
 * @param U output left singular vectors [m × min(m,n)], [m × k] when config.k truncates, [m × m] if fullMatrices whatever k
 * @param S output singular values [min(m,n)], or [k] when config.k truncates, always real and non-negative, descending order
 * @param V output right singular vectors [n × min(m,n)], [n × k] when config.k truncates, [n × n] if fullMatrices whatever k
 * @param A input matrix [m × n]
 * @param config SVD configuration
 * @return status indicating success or type of failure
 */
template<ExecutionPolicy Policy, TensorLike TensorU, TensorLike TensorS, TensorLike TensorV, TensorLike TensorA>
requires std::same_as<typename TensorU::value_type, typename TensorA::value_type> && std::same_as<typename TensorV::value_type, typename TensorA::value_type> && std::same_as<typename TensorS::value_type, gr::meta::fundamental_base_value_type_t<typename TensorA::value_type>>
svd::Status gesvd(Policy&& /*policy*/, TensorU& U, TensorS& S, TensorV& V, const TensorA& A, const svd::Config<gr::meta::fundamental_base_value_type_t<typename TensorA::value_type>>& config = {}) {
    using T             = typename TensorA::value_type;
    using BaseValueType = gr::meta::fundamental_base_value_type_t<T>;

    if (A.rank() != 2) {
        return svd::Status::InvalidInput;
    }

    const std::size_t m      = A.extent(0);
    const std::size_t n      = A.extent(1);
    const std::size_t minDim = std::min(m, n);

    if (m == 0 || n == 0) {
        U.resize({m, 0});
        S.resize({0});
        V.resize({n, 0});
        if (config.fullMatrices) {
            detail::extendOrthonormal(U, m);
            detail::extendOrthonormal(V, n);
        }
        return svd::Status::Success;
    }

    std::size_t actualK = (config.k > 0 && config.k <= minDim) ? config.k : minDim;

    BaseValueType tol = config.tolerance;
    if (tol < BaseValueType{0}) {
        tol = std::numeric_limits<BaseValueType>::epsilon() * static_cast<BaseValueType>(std::max(m, n));
    }

    std::size_t maxIter = config.maxIterations;
    if (maxIter == 0) {
        maxIter = 30 * std::max(m, n);
    }

    // select algorithm
    svd::Algorithm algo = config.algorithm;
    if (algo == svd::Algorithm::Auto) {
        // GolubReinsch for real types, Jacobi for complex (GR complex support is WIP)
        if constexpr (gr::meta::complex_like<T>) {
            algo = svd::Algorithm::Jacobi;
        } else {
            algo = svd::Algorithm::GolubReinsch;
        }
    } else if (algo == svd::Algorithm::GolubReinsch && gr::meta::complex_like<T>) {
        // GolubReinsch doesn't fully support complex yet, fall back to Jacobi
        algo = svd::Algorithm::Jacobi;
    }

    // both algorithms work best on tall matrices (m >= n)
    // for wide matrices (m < n), transpose and swap U/V
    bool      transposed = false;
    Tensor<T> Awork;

    if (m < n) {
        transposed = true;
        Awork      = conjTranspose(A);
    } else {
        Awork = Tensor<T>(A);
    }

    // both algorithms run on A times a power of two near 1 / max |a_ij|; S returns to the input scale
    const int exponent = detail::scaleToUnitMagnitude(Awork);

    // perform SVD
    svd::Status           status;
    Tensor<T>             Vtmp;
    Tensor<BaseValueType> Stmp;

    if (transposed) {
        if (algo == svd::Algorithm::Jacobi) {
            status = detail::svdJacobi(Awork, &Vtmp, Stmp, maxIter, tol, true);
        } else {
            status = detail::svdGolubReinsch(Awork, &Vtmp, Stmp, maxIter, tol, true);
        }
        V = std::move(Awork);
        U = std::move(Vtmp);
        S = std::move(Stmp);
    } else {
        if (algo == svd::Algorithm::Jacobi) {
            status = detail::svdJacobi(Awork, &V, S, maxIter, tol, true);
        } else {
            status = detail::svdGolubReinsch(Awork, &V, S, maxIter, tol, true);
        }
        U = std::move(Awork);
    }
    for (auto& value : S) {
        value = std::ldexp(value, exponent);
    }

    // ensure correct dimensions for thin SVD
    if (U.extent(0) != m || U.extent(1) != minDim) {
        Tensor<T> U_correct({m, minDim});
        U_correct.fill(T{0});
        for (std::size_t i = 0; i < std::min(m, U.extent(0)); ++i) {
            for (std::size_t j = 0; j < std::min(minDim, U.extent(1)); ++j) {
                U_correct[i, j] = U[i, j];
            }
        }
        U = std::move(U_correct);
    }

    if (V.extent(0) != n || V.extent(1) != minDim) {
        Tensor<T> V_correct({n, minDim});
        V_correct.fill(T{0});
        for (std::size_t i = 0; i < std::min(n, V.extent(0)); ++i) {
            for (std::size_t j = 0; j < std::min(minDim, V.extent(1)); ++j) {
                V_correct[i, j] = V[i, j];
            }
        }
        V = std::move(V_correct);
    }

    if (S.size() != minDim) {
        Tensor<BaseValueType> S_correct({minDim});
        S_correct.fill(BaseValueType{0});
        for (std::size_t i = 0; i < std::min(minDim, S.size()); ++i) {
            S_correct[i] = S[i];
        }
        S = std::move(S_correct);
    }

    // sort singular values in descending order
    detail::sortSingularValues(S, &U, &V);

    // full U and V: the columns past min(m, n) complete the singular vectors to orthonormal bases
    if (config.fullMatrices) {
        detail::extendOrthonormal(U, m);
        detail::extendOrthonormal(V, n);
    }

    // truncate if user requested fewer singular values
    if (actualK < minDim) {
        const std::size_t     uCols = config.fullMatrices ? m : actualK;
        const std::size_t     vCols = config.fullMatrices ? n : actualK;
        Tensor<T>             U_final({m, uCols});
        Tensor<BaseValueType> S_final({actualK});
        Tensor<T>             V_final({n, vCols});

        for (std::size_t i = 0; i < m; ++i) {
            for (std::size_t j = 0; j < uCols; ++j) {
                U_final[i, j] = U[i, j];
            }
        }

        for (std::size_t i = 0; i < actualK; ++i) {
            S_final[i] = S[i];
        }

        for (std::size_t i = 0; i < n; ++i) {
            for (std::size_t j = 0; j < vCols; ++j) {
                V_final[i, j] = V[i, j];
            }
        }

        U = std::move(U_final);
        S = std::move(S_final);
        V = std::move(V_final);

        if (status == svd::Status::Success) {
            status = svd::Status::EarlyReturn;
        }
    }

    return status;
}

/**
 * @brief singular value decomposition: A = U * diag(S) * V^H  (default to cpu_policy)
 *
 * computes the thin SVD by default (U is m×min(m,n), V is n×min(m,n))
 * set config.fullMatrices = true for full SVD
 *
 * @param U output left singular vectors, shaped as for the overload with a policy
 * @param S output singular values, real and non-negative, descending order
 * @param V output right singular vectors, shaped as for the overload with a policy
 * @param A input matrix [m × n]
 * @param config SVD configuration
 * @return status indicating success or type of failure
 */
template<TensorLike TensorU, TensorLike TensorS, TensorLike TensorV, TensorLike TensorA>
requires std::same_as<typename TensorU::value_type, typename TensorA::value_type> && std::same_as<typename TensorV::value_type, typename TensorA::value_type> && std::same_as<typename TensorS::value_type, gr::meta::fundamental_base_value_type_t<typename TensorA::value_type>>
svd::Status gesvd(TensorU& U, TensorS& S, TensorV& V, const TensorA& A, const svd::Config<gr::meta::fundamental_base_value_type_t<typename TensorA::value_type>>& config = {}) {
    return gesvd(cpu_policy{}, U, S, V, A, config);
}

/**
 * @brief singular values only decomposition
 *
 * computes only the singular values S without U and V matrices
 * more efficient when only singular values are needed
 *
 * @param policy execution policy
 * @param S output singular values [min(m,n)], real and non-negative, descending order
 * @param A input matrix [m × n]
 * @param config SVD configuration
 * @return status indicating success or type of failure
 */
template<ExecutionPolicy Policy, TensorLike TensorS, TensorLike TensorA>
requires std::same_as<typename TensorS::value_type, gr::meta::fundamental_base_value_type_t<typename TensorA::value_type>>
svd::Status gesvd(Policy&& /*policy*/, TensorS& S, const TensorA& A, const svd::Config<gr::meta::fundamental_base_value_type_t<typename TensorA::value_type>>& config = {}) {
    using T             = typename TensorA::value_type;
    using BaseValueType = gr::meta::fundamental_base_value_type_t<T>;

    if (A.rank() != 2) {
        return svd::Status::InvalidInput;
    }

    const std::size_t m      = A.extent(0);
    const std::size_t n      = A.extent(1);
    const std::size_t minDim = std::min(m, n);

    if (m == 0 || n == 0) {
        S.resize({0});
        return svd::Status::Success;
    }

    std::size_t actualK = (config.k > 0 && config.k <= minDim) ? config.k : minDim;

    BaseValueType tol = config.tolerance;
    if (tol < BaseValueType{0}) {
        tol = std::numeric_limits<BaseValueType>::epsilon() * static_cast<BaseValueType>(std::max(m, n));
    }

    std::size_t maxIter = config.maxIterations;
    if (maxIter == 0) {
        maxIter = 30 * std::max(m, n);
    }

    svd::Algorithm algo = config.algorithm;
    if (algo == svd::Algorithm::Auto) {
        // Golub-Reinsch for real types, Jacobi for complex (GR complex support is WIP)
        if constexpr (gr::meta::complex_like<T>) {
            algo = svd::Algorithm::Jacobi;
        } else {
            algo = svd::Algorithm::GolubReinsch;
        }
    } else if (algo == svd::Algorithm::GolubReinsch && gr::meta::complex_like<T>) {
        // Golub-Reinsch doesn't fully support complex yet, fall back to Jacobi
        algo = svd::Algorithm::Jacobi;
    }

    bool      transposed = (m < n);
    Tensor<T> Awork      = transposed ? conjTranspose(A) : Tensor<T>(A);
    const int exponent   = detail::scaleToUnitMagnitude(Awork);

    svd::Status status;
    Tensor<T>*  noV = nullptr; // explicit type for template deduction
    if (algo == svd::Algorithm::Jacobi) {
        status = detail::svdJacobi(Awork, noV, S, maxIter, tol, false);
    } else {
        status = detail::svdGolubReinsch(Awork, noV, S, maxIter, tol, false);
    }
    for (auto& value : S) {
        value = std::ldexp(value, exponent);
    }

    // ensure correct size
    if (S.size() != minDim) {
        Tensor<BaseValueType> S_correct({minDim});
        S_correct.fill(BaseValueType{0});
        for (std::size_t i = 0; i < std::min(minDim, S.size()); ++i) {
            S_correct[i] = S[i];
        }
        S = std::move(S_correct);
    }

    // sort descending
    std::ranges::sort(S, std::greater<>{});

    // truncate if requested
    if (actualK < minDim) {
        Tensor<BaseValueType> S_final({actualK});
        for (std::size_t i = 0; i < actualK; ++i) {
            S_final[i] = S[i];
        }
        S = std::move(S_final);

        if (status == svd::Status::Success) {
            status = svd::Status::EarlyReturn;
        }
    }

    return status;
}

/**
 * @brief singular values only decomposition (default to cpu_policy)
 *
 * computes only the singular values S without U and V matrices
 * more efficient when only singular values are needed
 *
 * @param S output singular values [min(m,n)], real and non-negative, descending order
 * @param A input matrix [m × n]
 * @param config SVD configuration
 * @return status indicating success or type of failure
 */
template<TensorLike TensorS, TensorLike TensorA>
requires std::same_as<typename TensorS::value_type, gr::meta::fundamental_base_value_type_t<typename TensorA::value_type>>
svd::Status gesvd(TensorS& S, const TensorA& A, const svd::Config<gr::meta::fundamental_base_value_type_t<typename TensorA::value_type>>& config = {}) {
    return gesvd(cpu_policy{}, S, A, config);
}

} // namespace gr::math

#endif // GNURADIO_SVD_HPP
