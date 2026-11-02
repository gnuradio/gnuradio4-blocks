#if defined(__clang__) || defined(__GNUC__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wdouble-promotion"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#ifndef __clang__
#pragma GCC diagnostic ignored "-Wuseless-cast"
#endif
#endif

#include "exprtk/exprtk.hpp" // include the original ExprTk header

#if defined(__clang__) || defined(__GNUC__)
#pragma GCC diagnostic pop
#endif

// explicit instantiations for supported types
template class exprtk::expression<float>;
template class exprtk::expression<double>;

template class exprtk::parser<float>;
template class exprtk::parser<double>;

// MSVC cannot instantiate symbol_table<T> explicitly: doing so also instantiates
// type_store<vector_holder<T>, ...>::type_ref(), whose `static RawType null_type = ...` requires a
// default constructor that vector_holder has no reason to provide. The member is never called for
// the vector store, so GCC and Clang never instantiate it. Only the explicit instantiation is
// affected -- ExprTk itself compiles and runs fine under cl.exe.
#ifndef _MSC_VER
template class exprtk::symbol_table<float>;
template class exprtk::symbol_table<double>;
#endif

template class exprtk::function_compositor<float>;
template class exprtk::function_compositor<double>;
