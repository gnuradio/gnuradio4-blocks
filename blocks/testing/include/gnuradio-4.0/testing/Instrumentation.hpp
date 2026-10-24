#ifndef GNURADIO_TESTING_INSTRUMENTATION_HPP
#define GNURADIO_TESTING_INSTRUMENTATION_HPP

// GCC states a sanitizer in a macro and clang answers __has_feature. This header asks both.
#if defined(__SANITIZE_ADDRESS__) || defined(__SANITIZE_THREAD__)
#define GR_TESTING_SANITIZER_PRESENT 1
#elif defined(__has_feature)
#if __has_feature(address_sanitizer) || __has_feature(thread_sanitizer) || __has_feature(memory_sanitizer)
#define GR_TESTING_SANITIZER_PRESENT 1
#endif
#endif

namespace gr::blocks::testing {

/// @brief True when a sanitizer rewrites this translation unit's memory accesses.
///
/// The instrumentation costs several times what an access costs. A loop's share of that cost follows its share of
/// memory traffic, and two loops slow by different factors.
inline constexpr bool kInstrumented =
#ifdef GR_TESTING_SANITIZER_PRESENT
    true;
#else
    false;
#endif

/// @brief True when the translation unit is compiled with optimization. Both compilers define `__OPTIMIZE__` from -O1.
inline constexpr bool kOptimized =
#ifdef __OPTIMIZE__
    true;
#else
    false;
#endif

/// @brief True in an optimized build without a sanitizer. A cost measured there describes optimized code.
///
/// A ratio of two measured loops describes the generated code: the work the compiler hoisted, vectorized or elided.
/// Unoptimized code differs from it, and instrumentation adds a per-access charge the two loops pay unequally. A test
/// that asserts such a ratio measures it only where this holds.
inline constexpr bool kCostMeasurable = kOptimized && !kInstrumented;

} // namespace gr::blocks::testing

#undef GR_TESTING_SANITIZER_PRESENT

#endif // GNURADIO_TESTING_INSTRUMENTATION_HPP
