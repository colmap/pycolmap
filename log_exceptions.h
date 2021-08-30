#pragma once
#include <exception>
#include <string>
#include <sstream>

#include <pybind11/pybind11.h>

#define THROW_EXCEPTION(exception, msg) \
throw TemplateException<exception>(__FILE__, __LINE__, msg);

#define THROW_CUSTOM_CHECK_MSG(condition, exception, msg) \
if (!condition)  \
    throw TemplateException<exception>(__FILE__, __LINE__, \
    __MergeTwoConstChar(__GetCheckString(#condition).c_str(), msg).c_str());

#define THROW_CUSTOM_CHECK(condition, exception) \
if (!condition)  \
    throw TemplateException<exception>(__FILE__, __LINE__,__GetCheckString(#condition).c_str());

inline const char* __ColmapGetConstFileBaseName(const char* file) {
  const char* base = strrchr(file, '/');
  if (!base) {
    base = strrchr(file, '\\');
  }
  return base ? (base + 1) : file;
}

template <typename T>
inline T TemplateException(const char* file, const int line, const char* txt) {
    std::stringstream ss;
    ss << "["<<__ColmapGetConstFileBaseName(file)<<":"<<line<<"] "<<txt;
    return T(ss.str());
}

inline std::string __GetConditionString(const char* cond_str) {
    std::stringstream ss;
    ss << "Condition Failed: "<<cond_str;
    return ss.str();
}


inline std::string __GetCheckString(const char* cond_str) {
    std::stringstream ss;
    ss << "Check Failed: "<<cond_str;
    return ss.str();
}


inline std::string __MergeTwoConstChar(const char* expr1, const char* expr2) {
    return (std::string(expr1) + std::string("\n") + expr2);
}


inline void __ThrowCheckImpl(const char* file, const int line, const bool result,
                       const char* expr_str) {
    if (!result) {
        throw TemplateException<py::value_error>(file, line, __GetCheckString(expr_str).c_str());
    }
}

template <typename T1, typename T2>
void __ThrowCheckOpImpl(const char* file, const int line, const bool result,
                         const T1& val1, const T2& val2, const char* val1_str,
                         const char* val2_str, const char* op_str) {
  if (!result) {
      std::stringstream ss;
      ss << val1_str << " " << op_str << " " << val2_str <<" ("<<val1<<" vs. "<<val2<<")";
      std::string msg = ss.str();
      throw TemplateException<py::value_error>(file, line, __GetCheckString(msg.c_str()).c_str());
  }
}

// Option checker macros. In contrast to glog, this function does not abort the
// program, but simply returns false on failure.
#define THROW_CHECK(expr) \
  __ThrowCheckImpl(__FILE__, __LINE__, (expr), #expr);

#define THROW_CHECK_OP(name, op, val1, val2)                                   \
  __ThrowCheckOpImpl(__FILE__, __LINE__, (val1 op val2), val1, val2,           \
                           #val1, #val2, #op);

#define THROW_CHECK_EQ(val1, val2) THROW_CHECK_OP(_EQ, ==, val1, val2)
#define THROW_CHECK_NE(val1, val2) THROW_CHECK_OP(_NE, !=, val1, val2)
#define THROW_CHECK_LE(val1, val2) THROW_CHECK_OP(_LE, <=, val1, val2)
#define THROW_CHECK_LT(val1, val2) THROW_CHECK_OP(_LT, <, val1, val2)
#define THROW_CHECK_GE(val1, val2) THROW_CHECK_OP(_GE, >=, val1, val2)
#define THROW_CHECK_GT(val1, val2) THROW_CHECK_OP(_GT, >, val1, val2)