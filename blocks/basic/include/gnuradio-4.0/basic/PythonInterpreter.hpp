#ifndef GNURADIO_PYTHONINTERPRETER_HPP
#define GNURADIO_PYTHONINTERPRETER_HPP

// Circumvent a POSIX violation in pyconfig.h by making it the first include
// See for details:
//   1. https://github.com/python/cpython/issues/61322
//   2. http://pubs.opengroup.org/onlinepubs/007904975/functions/xsh_chap02_02.html
#ifdef __clang__
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wmacro-redefined"
#endif
#ifdef __GNUC__
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wold-style-cast"
#ifndef __clang__
#pragma GCC diagnostic ignored "-Wuseless-cast"
#endif
#endif
#include <Python.h>

#include <atomic>
#include <cassert>
#include <cctype>
#include <complex>
#include <cstdint>
#include <exception>
#include <expected>
#include <optional>
#include <regex>
#include <span>
#include <sstream>
#include <string>
#include <string_view>
#include <tuple>
#include <type_traits>
#include <utility>

#include <gnuradio-4.0/Message.hpp>

// One NumPy API table serves every unit of a linked module. The unit that defines 'GR_PYTHON_RUNTIME_OWNER' before this
// include holds the table and the runtime that imports it, and every other unit of the module uses them.
#include <numpy/numpyconfig.h>
#define NPY_NO_DEPRECATED_API  NPY_1_7_API_VERSION
#define PY_ARRAY_UNIQUE_SYMBOL gr_python_PyArray_API
#ifndef GR_PYTHON_RUNTIME_OWNER
#define NO_IMPORT_ARRAY
#endif
#include <numpy/arrayobject.h>

namespace gr::python {

inline PyObject* TrueObj  = Py_True;
inline PyObject* FalseObj = Py_False;
inline PyObject* NoneObj  = Py_None;

constexpr inline bool isPyDict(const PyObject* obj) { return PyDict_Check(obj); }

constexpr inline bool isPyString(const PyObject* obj) { return PyUnicode_Check(obj); }

constexpr inline void PyDecRef(PyObject* obj) { // wrapper to isolate unsafe warning on C-API casts
    Py_XDECREF(obj);
}

constexpr inline void PyIncRef(PyObject* obj) { // wrapper to isolate unsafe warning on C-API casts
    Py_XINCREF(obj);
}

constexpr inline std::string PyBytesAsString(PyObject* op) { return PyBytes_AsString(op); }

class PyObjectGuard {
    PyObject* _ptr;

    void move(PyObjectGuard&& other) noexcept {
        PyDecRef(_ptr);
        std::swap(_ptr, other._ptr);
    }

public:
    explicit PyObjectGuard(PyObject* ptr = nullptr) : _ptr(ptr) {}

    explicit PyObjectGuard(PyObjectGuard&& other) noexcept : _ptr(other._ptr) { move(std::move(other)); }

    ~PyObjectGuard() { PyDecRef(_ptr); }

    PyObjectGuard& operator=(PyObjectGuard&& other) noexcept {
        if (this != &other) {
            move(std::move(other));
        }
        return *this;
    }

    PyObjectGuard(const PyObjectGuard& other) : _ptr(other._ptr) { // copy constructor
        PyIncRef(_ptr);
    }

    PyObjectGuard& operator=(const PyObjectGuard& other) {
        if (this == &other) {
            return *this;
        }
        _ptr = other._ptr;
        PyIncRef(_ptr);
        return *this;
    }

    operator PyObject*() const { return _ptr; }

    PyObject* get() const { return _ptr; }

    /// hands the reference to the caller and leaves the guard empty
    [[nodiscard]] PyObject* release() noexcept { return std::exchange(_ptr, nullptr); }
};

class PyGILGuard {
    PyGILState_STATE _state;

public:
    PyGILGuard() : _state(PyGILState_Ensure()) {}

    ~PyGILGuard() { PyGILState_Release(_state); }

    PyGILGuard(const PyGILGuard&)            = delete;
    PyGILGuard& operator=(const PyGILGuard&) = delete;
};

[[nodiscard]] inline std::string toString(PyObject* object) {
    PyObjectGuard strObj(PyObject_Repr(object));
    PyObjectGuard bytesObj(PyUnicode_AsEncodedString(strObj.get(), "utf-8", "strict"));
    return python::PyBytesAsString(bytesObj.get());
}

[[nodiscard]] inline std::string toLineCountAnnotated(std::string_view code, std::size_t min = 0UZ, std::size_t max = std::numeric_limits<std::size_t>::max(), std::size_t marker = std::numeric_limits<std::size_t>::max() - 1UZ) {
    if (code.empty()) {
        return "";
    }
    auto splitLines = [](std::string_view str) {
        std::istringstream       stream{std::string(str)}; // Convert string_view to string
        std::vector<std::string> lines;
        std::string              line;
        while (std::getline(stream, line)) {
            lines.push_back(line);
        }
        return lines;
    };

    auto        lines = splitLines(code);
    std::string annotatedCode;
    annotatedCode.reserve(code.size() + lines.size() * 4UZ /*sizeof "123:"*/);
    for (std::size_t i = std::max(0UZ, min); i < std::min(lines.size(), max); i++) {
        // N.B. Python starts counting from '1' not '0'
        annotatedCode += std::format("{:3}:{}{}\n", i, lines[i], i == marker - 1UZ ? "   ####### <== here's your problem #######" : "");
    }
    return annotatedCode;
}

[[nodiscard]] inline std::string getDebugPythonObjectAttributes(PyObject* obj) {
    if (obj == nullptr) {
        return "The provided PyObject is null.\n";
    }

    PyObjectGuard dirList(PyObject_Dir(obj));
    if (!dirList) {
        PyErr_Print();
        return "Failed to get attribute list from object.\n";
    }

    // iterate over the list of attribute names
    std::string ret;
    Py_ssize_t  size = PyList_Size(dirList);
    for (Py_ssize_t i = 0; i < size; i++) {
        PyObject*     attrName = PyList_GetItem(dirList, i); // borrowed reference, no need to decref
        PyObjectGuard attrValue(PyObject_GetAttr(obj, attrName));
        ret += std::format("item {:3}: key: {} value: {}\n", i, toString(attrName), attrValue ? toString(attrValue) : "<Unable to retrieve value>");
    }
    return ret;
}

inline void throwCurrentPythonError(std::string_view msg, std::source_location location = std::source_location::current(), std::string_view pythonCode = "") {
    PyObjectGuard exception(PyErr_GetRaisedException());
    if (!exception) {
        throw gr::exception(std::format("{}\nPython error: <unknown exception>\ntrace-back: {}", msg, toLineCountAnnotated(pythonCode)), location);
    }
    // std::println("detailed debug info: {}", getDebugPythonObjectAttributes(exception))

    std::size_t min    = 0UZ;
    std::size_t max    = std::numeric_limits<std::size_t>::max();
    std::size_t marker = std::numeric_limits<std::size_t>::max() - 1UZ;
    if (PyObjectGuard lineStr(PyObject_GetAttrString(exception.get(), "lineno")); lineStr) {
        marker = PyLong_AsSize_t(lineStr);
        min    = marker > 5UZ ? marker - 5UZ : 0;
        max    = marker < (std::numeric_limits<std::size_t>::max() - 5UZ) ? marker + 5UZ : marker < std::numeric_limits<std::size_t>::max();
    }

    throw gr::exception(std::format("{}\nPython error: {}\n{}", msg, toString(exception), toLineCountAnnotated(pythonCode, min, max, marker)), location);
}

/// Clears the raised Python exception and returns its representation.
[[nodiscard]] inline std::string takeCurrentPythonError() {
    PyObjectGuard exception(PyErr_GetRaisedException());
    return exception ? toString(exception) : std::string("<unknown exception>");
}

[[nodiscard]] inline std::string getDictionary(std::string_view moduleName) {
    PyObject* module = PyDict_GetItemString(PyImport_GetModuleDict(), moduleName.data());
    if (module == nullptr) {
        return "";
    }

    if (PyObject* module_dict = PyModule_GetDict(module); module_dict != nullptr) {
        PyObjectGuard dictGuard(PyObject_Repr(module_dict));
        return PyUnicode_AsUTF8(dictGuard);
    }
    return "";
}

template<typename T>
concept NoParamNoReturn = requires(T t) {
    { t() } -> std::same_as<void>;
};

template<typename T>
int numpyType() noexcept {
    // clang-format off
    if constexpr (std::is_same_v<T, bool>)          return NPY_BOOL;
    else if constexpr (std::is_same_v<T, std::int8_t>)   return NPY_BYTE;
    else if constexpr (std::is_same_v<T, std::uint8_t>)  return NPY_UBYTE;
    else if constexpr (std::is_same_v<T, std::int16_t>)  return NPY_SHORT;
    else if constexpr (std::is_same_v<T, std::uint16_t>) return NPY_USHORT;
    else if constexpr (std::is_same_v<T, std::int32_t>)  return NPY_INT;
    else if constexpr (std::is_same_v<T, std::uint32_t>) return NPY_UINT;
    else if constexpr (std::is_same_v<T, std::int64_t>)  return NPY_LONG;
    else if constexpr (std::is_same_v<T, std::uint64_t>) return NPY_ULONG;
    else if constexpr (std::is_same_v<T, float>)    return NPY_FLOAT;
    else if constexpr (std::is_same_v<T, double>)   return NPY_DOUBLE;
    else if constexpr (std::is_same_v<T, std::complex<float>>)  return NPY_CFLOAT;
    else if constexpr (std::is_same_v<T, std::complex<double>>) return NPY_CDOUBLE;
    else if constexpr (std::is_same_v<T, char*> || std::is_same_v<T, const char*>) return NPY_STRING;
    else return NPY_NOTYPE;
    // clang-format on
}

template<typename T>
requires std::is_arithmetic_v<T> || std::is_same_v<T, std::complex<float>> || std::is_same_v<T, std::complex<double>>
constexpr inline PyObject* toPyArray(T* arrayData, std::initializer_list<std::size_t> dimensions) {
    assert(dimensions.size() >= 1 && "nDim needs to be >= 1");

    std::vector<npy_intp> npyDims(dimensions.begin(), dimensions.end());
    // N.B. reinterpret cast is needed to access NumPy's unsafe C-API
    void*     data    = const_cast<void*>(reinterpret_cast<const void*>(arrayData));
    PyObject* npArray = PyArray_SimpleNewFromData(static_cast<int>(dimensions.size()), npyDims.data(), python::numpyType<std::remove_cv_t<T>>(), data);
    if (!npArray) {
        python::throwCurrentPythonError("Unable to create NumPy array");
    }
    PyArray_CLEARFLAGS(reinterpret_cast<PyArrayObject*>(npArray), NPY_ARRAY_OWNDATA);

    if constexpr (!std::is_const_v<T>) {
        PyArray_ENABLEFLAGS(reinterpret_cast<PyArrayObject*>(npArray), NPY_ARRAY_WRITEABLE);
    } else {
        PyArray_CLEARFLAGS(reinterpret_cast<PyArrayObject*>(npArray), NPY_ARRAY_WRITEABLE);
    }
    return npArray;
}

[[nodiscard]] inline std::expected<PyObject*, std::string> toPyObject(const pmt::Value::Map& map, std::string_view key);
[[nodiscard]] inline std::expected<PyObject*, std::string> toPyObject(const pmt::Value& value, std::string_view key);

namespace detail {
/// returns a new reference, or null with a Python error set
template<typename T>
[[nodiscard]] PyObject* toPyScalar(const T& value) {
    if constexpr (std::is_same_v<T, bool>) { // before the integers: a Python bool is an int
        return PyBool_FromLong(value ? 1L : 0L);
    } else if constexpr (std::is_same_v<T, std::pmr::string>) {
        return PyUnicode_FromStringAndSize(value.data(), static_cast<Py_ssize_t>(value.size()));
    } else if constexpr (std::is_same_v<T, std::complex<float>> || std::is_same_v<T, std::complex<double>>) {
        return PyComplex_FromDoubles(static_cast<double>(value.real()), static_cast<double>(value.imag()));
    } else if constexpr (std::is_floating_point_v<T>) {
        return PyFloat_FromDouble(static_cast<double>(value));
    } else if constexpr (std::is_signed_v<T>) {
        return PyLong_FromLongLong(static_cast<long long>(value));
    } else {
        return PyLong_FromUnsignedLongLong(static_cast<unsigned long long>(value));
    }
}

[[nodiscard]] inline std::unexpected<std::string> conversionFailed(std::string_view key) {
    PyErr_Clear();
    return std::unexpected(std::format("'{}' could not be converted to a Python object", key));
}

template<typename T>
[[nodiscard]] std::expected<PyObject*, std::string> toPyList(const Tensor<T>& tensor, std::string_view key) {
    if (tensor.rank() > 1UZ) {
        return std::unexpected(std::format("'{}' is a tensor of rank {}, and a Python list takes a rank of one", key, tensor.rank()));
    }
    PyObjectGuard list(PyList_New(static_cast<Py_ssize_t>(tensor.size())));
    if (!list) {
        return conversionFailed(key);
    }
    Py_ssize_t index = 0;
    for (const auto& element : tensor) {
        PyObject* item = nullptr;
        if constexpr (std::is_same_v<T, pmt::Value>) {
            auto converted = toPyObject(element, std::format("{}[{}]", key, index));
            if (!converted) {
                return std::unexpected(converted.error());
            }
            item = *converted;
        } else {
            item = toPyScalar(static_cast<T>(element));
        }
        if (item == nullptr) {
            return conversionFailed(key);
        }
        PyList_SetItem(list, index++, item); // steals the reference
    }
    return list.release();
}
} // namespace detail

/// Converts a map to a new Python dict whose entries are converted as by the overload for a value.
[[nodiscard]] inline std::expected<PyObject*, std::string> toPyObject(const pmt::Value::Map& map, std::string_view key) {
    PyObjectGuard dict(PyDict_New());
    if (!dict) {
        return detail::conversionFailed(key);
    }
    for (const auto& [entryKey, entry] : map) {
        const std::string name(entryKey);
        auto              converted = toPyObject(entry, std::format("{}.{}", key, name));
        if (!converted) {
            return std::unexpected(converted.error());
        }
        PyObjectGuard item(*converted);
        if (PyDict_SetItemString(dict, name.c_str(), item) != 0) {
            return detail::conversionFailed(key);
        }
    }
    return dict.release();
}

/// Converts a value to a new Python object: None, bool, int, float, complex or str, a dict for a map, and a list for a
/// tensor of rank one. A value without such a form returns an error that names its place: 'key' for the value itself,
/// 'key.name' for a map entry and 'key[i]' for a list element. A tensor of rank two or more has no such form.
[[nodiscard]] inline std::expected<PyObject*, std::string> toPyObject(const pmt::Value& value, std::string_view key) {
    if (value.is_monostate()) {
        PyIncRef(NoneObj);
        return NoneObj;
    }
    if (const auto* map = value.get_if<pmt::Value::Map>(); map != nullptr) {
        return toPyObject(*map, key);
    }
    if (const auto* list = value.get_if<Tensor<pmt::Value>>(); list != nullptr) {
        return detail::toPyList(*list, key);
    }
    std::optional<std::expected<PyObject*, std::string>> converted;
    auto                                                 convertAs = [&value, key, &converted]<typename T>() {
        if (converted.has_value()) {
            return;
        }
        if (const T* scalar = value.get_if<T>(); scalar != nullptr) {
            PyObject* object = detail::toPyScalar(*scalar);
            converted        = object != nullptr ? std::expected<PyObject*, std::string>(object) : detail::conversionFailed(key);
        } else if (const auto* tensor = value.get_if<Tensor<T>>(); tensor != nullptr) {
            converted = detail::toPyList(*tensor, key);
        }
    };
    [&convertAs]<typename... Ts>(std::type_identity<std::tuple<Ts...>>) { (convertAs.template operator()<Ts>(), ...); }(std::type_identity<std::tuple<bool, std::int8_t, std::int16_t, std::int32_t, std::int64_t, std::uint8_t, std::uint16_t, std::uint32_t, std::uint64_t, float, double, std::complex<float>, std::complex<double>, std::pmr::string>>{});
    if (converted.has_value()) {
        return *converted;
    }
    return std::unexpected(std::format("'{}' holds a type without a Python form", key));
}

template<typename T>
std::string sanitizedPythonBlockName() {
    std::string str = gr::meta::type_name<T>();
    std::replace(str.begin(), str.end(), ':', '_');
    std::replace(str.begin(), str.end(), '<', '_');
    std::replace(str.begin(), str.end(), '>', '_');
    str.erase(std::remove_if(str.begin(), str.end(), [](unsigned char c) { return std::isalnum(static_cast<unsigned char>(c)) == 0 && c != '_'; }), str.end());
    return str;
}

} // namespace gr::python
#ifdef __GNUC__
#pragma GCC diagnostic pop
#endif
#ifdef __clang__
#pragma clang diagnostic pop
#endif

#include <format>
#include <stdexcept>
#include <vector>

namespace gr::python {

enum class EnforceFunction { MANDATORY, OPTIONAL };

/// Runs one block's Python code in a module of its own. Each instance creates a private module and holds it for its whole
/// life. The module's dictionary holds the block's capsule and every name the block's script defines, and two instances
/// whose scripts define the same name each keep their own.
class Interpreter {
    /// Imports the linked module's NumPy table and initializes the embedded interpreter if it is not yet initialized. Each
    /// linked module holds one instance, which the module's first 'Interpreter' constructs. The instance that initialized
    /// the interpreter finalizes it at process exit. NumPy cannot be imported again into a process after 'Py_Finalize()'.
    /// A block's destruction therefore releases only the block's own objects. An interpreter that the host process
    /// initialized is left for the host to finalize.
    class Runtime {
        bool               _ownsInterpreter = Py_IsInitialized() == 0;
        std::exception_ptr _numpyError;

    public:
        Runtime(); // defined in the unit that holds the module's table

        ~Runtime() {
            if (_ownsInterpreter && Py_IsInitialized()) {
                std::ignore = PyGILState_Ensure();
                Py_Finalize();
            }
        }

        Runtime(const Runtime&)            = delete;
        Runtime& operator=(const Runtime&) = delete;

        void throwIfUnusable() const {
            if (_numpyError) {
                std::rethrow_exception(_numpyError);
            }
        }
    };

    // the hidden visibility keeps one instance per linked module, beside the module's own table
    [[gnu::visibility("hidden")]] static Runtime& runtime();

    static std::atomic<std::size_t> _nModules;
    PyModuleDef*                    _moduleDefinitions;
    PyObject*                       _pModule = nullptr; // owned reference, released under the interpreter lock
    PyObject*                       _pDict   = nullptr; // borrowed from _pModule

public:
    template<typename T>
    explicit(false) Interpreter(T* classReference, PyModuleDef* moduleDefinitions = nullptr, std::source_location location = std::source_location::current()) : _moduleDefinitions(moduleDefinitions) {
        runtime().throwIfUnusable();
        assert(Py_IsInitialized() && "Python isn't properly initialised");
        // Ensure the Python GIL is initialized for this instance
        python::PyGILGuard localGuard;

        const std::string moduleName = std::format("{}_{}", moduleDefinitions != nullptr ? moduleDefinitions->m_name : "gr_python", _nModules.fetch_add(1UZ, std::memory_order_relaxed));
        PyObjectGuard     module(PyModule_New(moduleName.c_str())); // moves into '_pModule' once the constructor succeeds
        if (!module) {
            python::throwCurrentPythonError(std::format("failed to create the module {}", moduleName), location);
        }
        _pDict = PyModule_GetDict(module);
        if (PyDict_SetItemString(_pDict, "__builtins__", PyEval_GetBuiltins()) != 0) {
            python::throwCurrentPythonError(std::format("failed to add the builtins to the module {}", moduleName), location);
        }
        if (classReference == nullptr || moduleDefinitions == nullptr) {
            _pModule = module.release();
            return;
        }
        PyObjectGuard capsule(PyCapsule_New(static_cast<void*>(classReference), _moduleDefinitions->m_name, nullptr));
        if (!capsule) {
            python::throwCurrentPythonError(std::format("Interpreter(*{}) - failed to create a capsule", gr::meta::type_name<T>()));
        }
        if (PyDict_SetItemString(_pDict, "capsule", capsule) != 0) {
            python::throwCurrentPythonError(std::format("Interpreter(*{}) - failed to store the capsule", gr::meta::type_name<T>()), location);
        }

        // replaces the 'PyImport_AppendInittab("ClassName", &classDefinition)' to allow for other blocks being added
        // after the global Python interpreter is already being initialised
        PyObject* m = PyModule_Create(moduleDefinitions);
        if (m) {
            int ret = PyDict_SetItemString(PyImport_GetModuleDict(), moduleDefinitions->m_name, m);
            python::PyDecRef(m); // The module dict holds a reference now.
            if (ret != 0) {
                python::throwCurrentPythonError(std::format("Error inserting module {}.", _moduleDefinitions->m_name), location);
            }
        } else {
            python::throwCurrentPythonError(std::format("failed to create the module {}.", _moduleDefinitions->m_name), location);
        }
        if (PyDict_GetItemString(PyImport_GetModuleDict(), moduleDefinitions->m_name)) { // module successfully inserted - performing some additional checks
            assert(python::getDictionary(moduleDefinitions->m_name).size() > 0 && "dictionary exist for module");

            if (PyObject* imported_module = PyImport_ImportModule(moduleDefinitions->m_name); imported_module != nullptr) {
                python::PyDecRef(imported_module);
            } else {
                python::throwCurrentPythonError(std::format("Check import of {} failed.", _moduleDefinitions->m_name), location);
            }
        } else {
            python::throwCurrentPythonError(std::format("Manual import of {} failed.", _moduleDefinitions->m_name), location);
        }
        _pModule = module.release();
    }

    ~Interpreter() {
        if (Py_IsInitialized()) {
            PyGILGuard localGuard;
            python::PyDecRef(_pModule);
        }
    }

    // Prevent copying and moving
    Interpreter(const Interpreter&)            = delete;
    Interpreter& operator=(const Interpreter&) = delete;
    Interpreter(Interpreter&&)                 = delete;
    Interpreter& operator=(Interpreter&&)      = delete;

    PyObject* getModule() { return _pModule; }

    PyObject* getDictionary() { return _pDict; }

    template<NoParamNoReturn Func>
    void invoke(Func func, std::string_view pythonCode = "", std::source_location location = std::source_location::current()) {
        assert(Py_IsInitialized());
        PyGILGuard localGuard;
        if (PyInterpreterState_Get() != PyInterpreterState_Main()) {
            python::throwCurrentPythonError("detected sub-interpreter change which is not supported by NumPy", location, pythonCode);
        }
        if (PyErr_Occurred()) {
            python::throwCurrentPythonError("python::Interpreter::invoke() -- uncleared Python error before executing func", location, pythonCode);
        }

        func();

        if (PyErr_Occurred()) {
            python::throwCurrentPythonError("python::Interpreter::invoke() -- uncleared Python error after executing func", location, pythonCode);
        }
    }

    /// Calls the module's function with the positional arguments in the tuple 'functionArguments' and the keyword
    /// arguments in the dict 'keywordArguments'. Either may be null for none.
    template<EnforceFunction forced = EnforceFunction::MANDATORY>
    python::PyObjectGuard invokeFunction(std::string_view functionName, PyObject* functionArguments = nullptr, PyObject* keywordArguments = nullptr, std::source_location location = std::source_location::current()) {
        PyGILGuard localGuard;
        const bool hasFunction = PyObject_HasAttrString(getModule(), functionName.data());
        if constexpr (forced == EnforceFunction::MANDATORY) {
            if (!hasFunction) {
                python::throwCurrentPythonError(std::format("getFunction('{}', '{}') Python function not found or is not callable", functionName, python::toString(functionArguments)), location);
            }
        } else {
            if (!hasFunction) {
                return python::PyObjectGuard(nullptr);
            }
        }
        python::PyObjectGuard pyFunc(PyObject_GetAttrString(getModule(), functionName.data()));
        python::PyObjectGuard noArguments(functionArguments == nullptr ? PyTuple_New(0) : nullptr);
        return python::PyObjectGuard(PyObject_Call(pyFunc, functionArguments != nullptr ? functionArguments : noArguments.get(), keywordArguments));
    }
};

inline std::atomic<std::size_t> Interpreter::_nModules{0UZ};

#ifdef GR_PYTHON_RUNTIME_OWNER
Interpreter::Runtime::Runtime() {
    if (_ownsInterpreter) {
        Py_Initialize();
        if (PyErr_Occurred()) {
            PyErr_Print();
        }
    }

    {
        python::PyGILGuard guard;
        if (_import_array() < 0) {
            // initialize NumPy -- N.B. NumPy does not support sub-interpreters (as of Python 3.12):
            // "sys:1: UserWarning: NumPy was imported from a Python sub-interpreter but NumPy does not properly support sub-interpreters.
            // This will likely work for most users but might cause hard to track down issues or subtle bugs.
            // A common user of the rare sub-interpreter feature is wsgi which also allows single-interpreter mode.
            // Improvements in the case of bugs are welcome, but is not on the NumPy roadmap, and full support may require significant effort to achieve."
            try {
                python::throwCurrentPythonError("failed to initialize NumPy");
            } catch (...) {
                _numpyError = std::current_exception();
            }
        }
    }
    // 'Py_Initialize()' leaves the interpreter lock with this thread. Saving the thread's state releases it, and
    // every later call takes it through 'PyGILGuard' on whichever thread makes the call.
    if (_ownsInterpreter) {
        std::ignore = PyEval_SaveThread();
    }
}

Interpreter::Runtime& Interpreter::runtime() {
    static Runtime instance;
    return instance;
}
#endif

} // namespace gr::python

#endif // GNURADIO_PYTHONINTERPRETER_HPP
