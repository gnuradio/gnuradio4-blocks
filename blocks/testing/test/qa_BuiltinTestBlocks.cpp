#include <boost/ut.hpp>

#include <gnuradio-4.0/testing/BuiltinTestBlocks.hpp>

const boost::ut::suite BuiltinTestBlockTests = [] {
    using namespace boost::ut;

    "multiply by configured factor"_test = [] {
        gr::blocks::testing::builtin_multiply<float> multiply({{"factor", 2.5f}});
        expect(eq(multiply.processOne(4.0f), 10.0f));
    };

    "count processed samples"_test = [] {
        using Counter          = gr::blocks::testing::builtin_counter<double>;
        Counter::s_event_count = 0;

        Counter counter;
        expect(eq(counter.processOne(3.0), 3.0));
        expect(eq(counter.processOne(7.0), 7.0));
        expect(eq(Counter::s_event_count, gr::Size_t{2}));
    };
};

int main() { return boost::ut::cfg<boost::ut::override>.run(); }
