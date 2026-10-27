#include <boost/ut.hpp>

#include <concepts>
#include <cstdint>
#include <format>
#include <iterator>
#include <string>
#include <string_view>
#include <type_traits>
#include <utility>

#include <gnuradio-4.0/Tag.hpp>

#include <gnuradio-4.0/algorithm/dataset/RecordVocabulary.hpp>

// The vocabulary is asserted against the framework's own declarations, so a framework retyping of a borrowed key
// breaks the build here and not a peer at run time.

const boost::ut::suite<"record vocabulary"> vocabularyTests = [] {
    using namespace boost::ut;
    using namespace gr::dataset;

    "the nine borrowed keys keep the framework's declared types"_test = [] {
        static_assert(std::same_as<std::remove_cvref_t<decltype(gr::tag::SAMPLE_RATE)>::value_type, float>);
        static_assert(std::same_as<std::remove_cvref_t<decltype(gr::tag::FREQUENCY)>::value_type, double>);
        static_assert(std::same_as<std::remove_cvref_t<decltype(gr::tag::SIGNAL_NAME)>::value_type, std::string>);
        static_assert(std::same_as<std::remove_cvref_t<decltype(gr::tag::SIGNAL_QUANTITY)>::value_type, std::string>);
        static_assert(std::same_as<std::remove_cvref_t<decltype(gr::tag::SIGNAL_UNIT)>::value_type, std::string>);
        static_assert(std::same_as<std::remove_cvref_t<decltype(gr::tag::SIGNAL_MIN)>::value_type, float>);
        static_assert(std::same_as<std::remove_cvref_t<decltype(gr::tag::SIGNAL_MAX)>::value_type, float>);
        static_assert(std::same_as<std::remove_cvref_t<decltype(gr::tag::N_DROPPED_SAMPLES)>::value_type, gr::Size_t>);
        static_assert(std::same_as<std::remove_cvref_t<decltype(gr::tag::TRIGGER_NAME)>::value_type, std::string>);
        expect(true) << "the nine borrowed declarations are asserted at compile time";
    };

    "twenty-one keys, each with exactly one declared type"_test = [] {
        using enum VocabularyType;
        const std::pair<std::string_view, VocabularyType> declared[]{                                                                     // borrowed from the framework's reserved stream vocabulary
            {"sample_rate", Float}, {"frequency", Double}, {"signal_name", String}, {"signal_quantity", String}, {"signal_unit", String}, //
            {"signal_min", Float}, {"signal_max", Float}, {"n_dropped_samples", Size}, {"trigger_name", String},
            // defined by the record-metadata vocabulary
            {"protocol", String}, {"schema_version", Size}, {"sample_start", UInt64}, {"sequence", UInt64}, {"discontinuity", String},                                 //
            {"crc_ok", Bool}, {"corrected_errors", Size}, {"uncorrectable_errors", Size}, {"discard_reason", String}, {"source_id", String}, {"dropped_events", Size}, //
            {"header_corrected_errors", UInt64}};

        expect(eq(std::size(declared), 21UZ)) << "twelve defined here and nine borrowed";
        for (const auto& [key, type] : declared) {
            expect(vocabularyType(key) == type) << std::format("{} is declared once", key);
            const std::string prefixed = std::format("gr:{}", key);
            expect(vocabularyType(recordName(prefixed)) == type) << std::format("gr:{} is the same key once the framework's prefix is stripped", key);
        }

        // a producer's own private keys are unbounded and impose nothing
        for (const std::string_view key : {"ctx", "n_pre", "n_post", "n_max", "sync_errors", "header_ok", "crc_value", "crc_width", "packet_number", "header_flags", "header_items", "trigger_time", "trigger_offset", "trigger_meta_info", "num_channels"}) {
            expect(vocabularyType(key) == NotVocabulary) << std::format("{} is producer-private or a stream-only reserved key", key);
        }
    };

    "a value of the wrong type fails its key's declaration and a private key passes anything"_test = [] {
        expect(holdsVocabularyType(VocabularyType::Float, gr::pmt::Value(48000.f)));
        expect(!holdsVocabularyType(VocabularyType::Float, gr::pmt::Value(48000.0))) << "no promotion, no narrowing, no conversion";
        expect(holdsVocabularyType(VocabularyType::Size, gr::pmt::Value(gr::Size_t{1U})));
        expect(!holdsVocabularyType(VocabularyType::Size, gr::pmt::Value(std::uint64_t{1ULL})));
        expect(holdsVocabularyType(VocabularyType::UInt64, gr::pmt::Value(std::uint64_t{1ULL})));
        expect(!holdsVocabularyType(VocabularyType::UInt64, gr::pmt::Value(gr::Size_t{1U})));
        expect(holdsVocabularyType(VocabularyType::Bool, gr::pmt::Value(true)));
        expect(holdsVocabularyType(VocabularyType::String, gr::pmt::Value(std::string("ax25"))));
        expect(holdsVocabularyType(VocabularyType::NotVocabulary, gr::pmt::Value(std::uint64_t{1ULL})));
        expect(holdsVocabularyType(VocabularyType::NotVocabulary, gr::pmt::Value(gr::property_map{})));
        expect(holdsVocabularyType(VocabularyType::Double, gr::pmt::Value(1.0e6)));
        expect(!holdsVocabularyType(VocabularyType::Double, gr::pmt::Value(1.0e6f)));
        expect(!holdsVocabularyType(VocabularyType::Bool, gr::pmt::Value(std::uint64_t{1ULL})));
        expect(!holdsVocabularyType(VocabularyType::String, gr::pmt::Value(true)));
    };

    "the record name strips one leading gr: prefix and nothing else"_test = [] {
        static_assert(recordName("gr:sample_rate") == "sample_rate");
        static_assert(recordName("sample_rate") == "sample_rate");
        expect(eq(recordName("gr:gr:sample_rate"), std::string_view("gr:sample_rate"))) << "one prefix per call";
        expect(eq(recordName("x:gr:sample_rate"), std::string_view("x:gr:sample_rate"))) << "the prefix only at the start";
        expect(eq(recordName("gr:"), std::string_view())) << "the bare prefix leaves an empty key";
        expect(eq(recordName("GR:sample_rate"), std::string_view("GR:sample_rate"))) << "the prefix is case-sensitive";
    };
};

int main() { /* not needed for UT */ }
