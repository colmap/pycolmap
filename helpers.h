// Author: Philipp Lindenberger (Phil26AT)

#pragma once

#include "colmap/util/threading.h"

#include <iostream>
#include <regex>
#include <string>

#include <glog/logging.h>
#include <pybind11/embed.h>
#include <pybind11/eval.h>
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>

namespace py = pybind11;
using namespace pybind11::literals;

#include "log_exceptions.h"

template <typename T>
inline T pyStringToEnum(const py::enum_<T>& enm, const std::string& value) {
  auto values = enm.attr("__members__").template cast<py::dict>();
  auto strVal = py::str(value);
  if (values.contains(strVal)) {
    return T(values[strVal].template cast<T>());
  }
  std::string msg =
      "Invalid string value " + value + " for enum " +
      std::string(enm.attr("__name__").template cast<std::string>());
  THROW_EXCEPTION(std::out_of_range, msg.c_str());
  T t;
  return t;
}

template <typename T>
inline void AddStringToEnumConstructor(py::enum_<T>& enm) {
  enm.def(py::init([enm](const std::string& value) {
    return pyStringToEnum(enm, py::str(value));  // str constructor
  }));
  py::implicitly_convertible<std::string, T>();
}

inline void UpdateFromDict(py::object& self, py::dict& dict) {
  for (auto& it : dict) {
    try {
      if (py::hasattr(self.attr(it.first), "mergedict")) {
        self.attr(it.first).attr("mergedict").attr("__call__")(it.second);
      } else {
        self.attr(it.first) = it.second;
      }
    } catch (const py::error_already_set& ex) {
      if (ex.matches(PyExc_TypeError)) {
        // If fail we try bases of the class
        py::list bases = self.attr(it.first)
                             .attr("__class__")
                             .attr("__bases__")
                             .cast<py::list>();
        bool success_on_base = false;
        for (auto& base : bases) {
          try {
            self.attr(it.first) = base(it.second);
            success_on_base = true;
            break;
          } catch (const py::error_already_set& ex) {
            continue;  // We anyway throw afterwards
          }
        }
        if (success_on_base) {
          continue;
        }
        std::stringstream ss;
        ss << self.attr("__class__")
                  .attr("__name__")
                  .template cast<std::string>()
           << "." << py::str(it.first).template cast<std::string>()
           << ": Could not convert "
           << py::type::of(it.second.cast<py::object>())
                  .attr("__name__")
                  .template cast<std::string>()
           << ": " << py::str(it.second).template cast<std::string>() << " to '"
           << py::type::of(self.attr(it.first))
                  .attr("__name__")
                  .template cast<std::string>()
           << "'.";
        // We write the err message to give info even if exceptions
        // is catched outside, e.g. in function overload resolve
        LOG(ERROR) << "Internal TypeError: " << ss.str();
        throw(py::type_error(std::string("Failed to merge dict into class: ") +
                             "Could not assign " +
                             py::str(it.first).template cast<std::string>()));
      } else if (ex.matches(PyExc_AttributeError) &&
                 py::str(ex.value()).cast<std::string>() ==
                     std::string("can't set attribute")) {
        std::stringstream ss;
        ss << self.attr("__class__")
                  .attr("__name__")
                  .template cast<std::string>()
           << "." << py::str(it.first).template cast<std::string>()
           << " defined readonly.";
        throw py::attribute_error(ss.str());
      } else if (ex.matches(PyExc_AttributeError)) {
        LOG(ERROR) << "Internal AttributeError: "
                   << py::str(ex.value()).cast<std::string>();
        throw;
      } else {
        LOG(ERROR) << "Internal Error: "
                   << py::str(ex.value()).cast<std::string>();
        throw;
      }
    }
  }
}

template <typename T, typename... options>
inline py::dict ConvertToDict(const T& self) {
  auto pyself = py::cast(self);
  py::dict dict;
  for (auto& handle : pyself.attr("__dir__")()) {
    std::string attribute = py::str(handle);
    auto member = pyself.attr(attribute.c_str());
    if (attribute.find("__") != 0 &&
        attribute.rfind("__") == std::string::npos &&
        !py::hasattr(member, "__func__")) {
      if (py::hasattr(member, "todict")) {
        dict[attribute.c_str()] =
            member.attr("todict").attr("__call__")().template cast<py::dict>();
      } else {
        dict[attribute.c_str()] = member;
      }
    }
  }
  return dict;
}

template <typename T, typename... options>
inline std::string CreateSummary(const T& self, bool write_type) {
  std::stringstream ss;
  auto pyself = py::cast(self);
  std::string prefix = "    ";
  bool after_subsummary = false;
  ss << pyself.attr("__class__").attr("__name__").template cast<std::string>()
     << ":\n";
  for (auto& handle : pyself.attr("__dir__")()) {
    std::string attribute = py::str(handle);
    auto member = pyself.attr(attribute.c_str());

    if (attribute.find("__") != 0 &&
        attribute.rfind("__") == std::string::npos &&
        !py::hasattr(member, "__func__")) {
      if (py::hasattr(member, "summary")) {
        std::string summ = member.attr("summary")
                               .attr("__call__")(write_type)
                               .template cast<std::string>();
        summ = std::regex_replace(summ, std::regex("\n"), "\n" + prefix);
        if (!after_subsummary) {
          ss << prefix;
        }
        ss << attribute << ": " << summ;
        after_subsummary = true;
      } else {
        if (!after_subsummary) {
          ss << prefix;
        }
        ss << attribute;
        if (write_type) {
          ss << ": "
             << py::type::of(member)
                    .attr("__name__")
                    .template cast<std::string>();
        }
        ss << " = " << py::str(member).template cast<std::string>() << "\n";
        after_subsummary = false;
      }
    }
  }
  return ss.str();
}

template <typename T, typename... options>
inline void make_dataclass(py::class_<T, options...> cls) {
  cls.def("mergedict", &UpdateFromDict);
  cls.def("summary", &CreateSummary<T>, "write_type"_a = false);
  cls.def("todict", &ConvertToDict<T>);
  cls.def(py::init([cls](py::dict dict) {
    auto self = py::object(cls());
    self.attr("mergedict").attr("__call__")(dict);
    return self.cast<T>();
  }));
  cls.def(py::init([cls](py::kwargs kwargs) {
    py::dict dict = kwargs.cast<py::dict>();
    auto self = py::object(cls(dict));
    return self.cast<T>();
  }));
  py::implicitly_convertible<py::dict, T>();
  py::implicitly_convertible<py::kwargs, T>();
}

// Catch python keyboard interrupts

/*
// single
if (PyInterrupt().Raised()) {
    // stop the execution and raise an exception
    throw py::error_already_set();
}

// loop
PyInterrupt py_interrupt = PyInterrupt(2.0)
for (...) {
    if (py_interrupt.Raised()) {
        // stop the execution and raise an exception
        throw py::error_already_set();
    }
    // Do your workload here
}


*/
struct PyInterrupt {
  using clock = std::chrono::steady_clock;
  using sec = std::chrono::duration<double>;
  PyInterrupt(double gap = -1.0);

  inline bool Raised();

 private:
  std::mutex mutex_;
  bool found = false;
  colmap::Timer timer_;
  clock::time_point start;
  double gap_;
};

PyInterrupt::PyInterrupt(double gap) : gap_(gap), start(clock::now()) {}

bool PyInterrupt::Raised() {
  const sec duration = clock::now() - start;
  if (!found && duration.count() > gap_) {
    std::lock_guard<std::mutex> lock(mutex_);
    py::gil_scoped_acquire acq;
    found = (PyErr_CheckSignals() != 0);
    start = clock::now();
  }
  return found;
}

// Instead of thread.Wait() call this to allow interrupts through python
void PyWait(Thread* thread, double gap = 2.0) {
  PyInterrupt py_interrupt(gap);
  while (thread->IsRunning()) {
    if (py_interrupt.Raised()) {
      LOG(ERROR) << "Stopping thread...";
      thread->Stop();
      thread->Wait();
      throw py::error_already_set();
    }
  }
  // after finishing join the thread to avoid abort
  thread->Wait();
}
