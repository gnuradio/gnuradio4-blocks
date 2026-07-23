#ifndef GR4_TESTING_BUILTIN_TEST_BLOCKS_HPP
#define GR4_TESTING_BUILTIN_TEST_BLOCKS_HPP

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/meta/reflection.hpp>

#include <gnuradio-4.0/testing/NamespaceCompatibility.hpp>

namespace gr::blocks::testing {

template<typename T>
struct builtin_multiply : gr::Block<builtin_multiply<T>> {
    T factor = static_cast<T>(1.0f);

    gr::PortIn<T>  in;
    gr::PortOut<T> out;

    GR_MAKE_REFLECTABLE(builtin_multiply, in, out, factor);

    builtin_multiply() = delete;

    explicit builtin_multiply(gr::property_map properties) {
        auto it = properties.find("factor");
        if (it != properties.cend()) {
            auto ptr = gr::checked_access_ptr{it->second.get_if<T>()};
            if (ptr != nullptr) {
                factor = *ptr;
            }
        }
    }

    [[nodiscard]] constexpr auto processOne(T value) const noexcept { return value * factor; }
};

template<typename T>
struct builtin_counter : gr::Block<builtin_counter<T>> {
    static gr::Size_t s_event_count;

    gr::PortIn<T>  in;
    gr::PortOut<T> out;

    GR_MAKE_REFLECTABLE(builtin_counter, in, out);

    [[nodiscard]] constexpr auto processOne(T value) const noexcept {
        s_event_count++;
        return value;
    }
};

template<typename T>
gr::Size_t builtin_counter<T>::s_event_count = 0;

} // namespace gr::blocks::testing

#endif // GR4_TESTING_BUILTIN_TEST_BLOCKS_HPP
