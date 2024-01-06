// Author: Philipp Lindenberger (Phil26AT)

#pragma once

#include "colmap/util/string.h"
#include "colmap/util/threading.h"
#include "colmap/util/types.h"

#include "pycolmap/log_exceptions.h"

#include <iostream>
#include <regex>
#include <string>

#include <glog/logging.h>
#include <pybind11/embed.h>
#include <pybind11/eval.h>
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/stl_bind.h>

using namespace colmap;
using namespace pybind11::literals;
namespace py = pybind11;

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
          } catch (const py::error_already_set&) {
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
void AddDefaultsToDocstrings(py::class_<T, options...> cls) {
  auto obj = cls();
  for (auto& handle : obj.attr("__dir__")()) {
    const std::string attribute = py::str(handle);
    const auto member = obj.attr(attribute.c_str());
    if (attribute.find("__") == 0 ||
        attribute.rfind("__") != std::string::npos ||
        py::hasattr(member, "__func__")) {
      continue;
    }
    auto prop = cls.attr(attribute.c_str());
    const auto type_name = py::type::of(member).attr("__name__");
    const std::string doc =
        StringPrintf("%s (%s, default: %s)",
                     py::str(prop.doc()).cast<std::string>().c_str(),
                     type_name.template cast<std::string>().c_str(),
                     py::str(member).cast<std::string>().c_str());
    prop.doc() = py::str(doc);
  }
}

template <typename T, typename... options>
inline void MakeDataclass(py::class_<T, options...> cls) {
  AddDefaultsToDocstrings(cls);
  cls.def("mergedict", &UpdateFromDict);
  if (!py::hasattr(cls, "summary")) {
    cls.def("summary", &CreateSummary<T>, "write_type"_a = false);
  }
  cls.attr("__repr__") = cls.attr("summary");
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
  Timer timer_;
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

namespace PYBIND11_NAMESPACE {

// Bind COLMAP's backport implementation of std::span. This copies the content
// into a list. We could instead create a view with an Eigen::Map but the cast
// should be explicit and cannot be automatic - likely not worth the added
// logic.
namespace detail {
template <typename Type>
struct type_caster<span<Type>> : list_caster<span<Type>, Type> {};
}  // namespace detail

// Fix long-standing bug https://github.com/pybind/pybind11/issues/4529
// TODO(sarlinpe): remove when https://github.com/pybind/pybind11/pull/4972
// appears in the next release of pybind11.
template <typename Map,
          typename holder_type = std::unique_ptr<Map>,
          typename... Args>
class_<Map, holder_type> bind_map_fix(handle scope,
                                      const std::string& name,
                                      Args&&... args) {
  using KeyType = typename Map::key_type;
  using MappedType = typename Map::mapped_type;
  using StrippedKeyType = detail::remove_cvref_t<KeyType>;
  using StrippedMappedType = detail::remove_cvref_t<MappedType>;
  using KeysView = detail::keys_view<StrippedKeyType>;
  using ValuesView = detail::values_view<StrippedMappedType>;
  using ItemsView = detail::items_view<StrippedKeyType, StrippedMappedType>;
  using Class_ = class_<Map, holder_type>;

  // If either type is a non-module-local bound type then make the map binding
  // non-local as well; otherwise (e.g. both types are either module-local or
  // converting) the map will be module-local.
  auto* tinfo = detail::get_type_info(typeid(MappedType));
  bool local = !tinfo || tinfo->module_local;
  if (local) {
    tinfo = detail::get_type_info(typeid(KeyType));
    local = !tinfo || tinfo->module_local;
  }

  Class_ cl(scope,
            name.c_str(),
            pybind11::module_local(local),
            std::forward<Args>(args)...);
  std::string key_type_name(detail::type_info_description(typeid(KeyType)));
  std::string mapped_type_name(
      detail::type_info_description(typeid(MappedType)));

  // Wrap KeysView[KeyType] if it wasn't already wrapped
  if (!detail::get_type_info(typeid(KeysView))) {
    class_<KeysView> keys_view(scope,
                               ("KeysView[" + key_type_name + "]").c_str(),
                               pybind11::module_local(local));
    keys_view.def("__len__", &KeysView::len);
    keys_view.def("__iter__",
                  &KeysView::iter,
                  keep_alive<0, 1>() /* Essential: keep view alive while
                                        iterator exists */
    );
    keys_view.def(
        "__contains__",
        static_cast<bool (KeysView::*)(const KeyType&)>(&KeysView::contains));
    // Fallback for when the object is not of the key type
    keys_view.def(
        "__contains__",
        static_cast<bool (KeysView::*)(const object&)>(&KeysView::contains));
  }
  // Similarly for ValuesView:
  if (!detail::get_type_info(typeid(ValuesView))) {
    class_<ValuesView> values_view(
        scope,
        ("ValuesView[" + mapped_type_name + "]").c_str(),
        pybind11::module_local(local));
    values_view.def("__len__", &ValuesView::len);
    values_view.def("__iter__",
                    &ValuesView::iter,
                    keep_alive<0, 1>() /* Essential: keep view alive while
                                          iterator exists */
    );
  }
  // Similarly for ItemsView:
  if (!detail::get_type_info(typeid(ItemsView))) {
    class_<ItemsView> items_view(scope,
                                 ("ItemsView[" + key_type_name + ", ")
                                     .append(mapped_type_name + "]")
                                     .c_str(),
                                 pybind11::module_local(local));
    items_view.def("__len__", &ItemsView::len);
    items_view.def("__iter__",
                   &ItemsView::iter,
                   keep_alive<0, 1>() /* Essential: keep view alive while
                                         iterator exists */
    );
  }

  cl.def(init<>());

  // Register stream insertion operator (if possible)
  detail::map_if_insertion_operator<Map, Class_>(cl, name);

  cl.def(
      "__bool__",
      [](const Map& m) -> bool { return !m.empty(); },
      "Check whether the map is nonempty");

  cl.def(
      "__iter__",
      [](Map& m) { return make_key_iterator(m.begin(), m.end()); },
      keep_alive<0, 1>() /* Essential: keep map alive while iterator exists */
  );

  cl.def(
      "keys",
      [](Map& m) {
        return std::unique_ptr<KeysView>(
            new detail::KeysViewImpl<Map, KeysView>(m));
      },
      keep_alive<0, 1>() /* Essential: keep map alive while view exists */
  );

  cl.def(
      "values",
      [](Map& m) {
        return std::unique_ptr<ValuesView>(
            new detail::ValuesViewImpl<Map, ValuesView>(m));
      },
      keep_alive<0, 1>() /* Essential: keep map alive while view exists */
  );

  cl.def(
      "items",
      [](Map& m) {
        return std::unique_ptr<ItemsView>(
            new detail::ItemsViewImpl<Map, ItemsView>(m));
      },
      keep_alive<0, 1>() /* Essential: keep map alive while view exists */
  );

  cl.def(
      "__getitem__",
      [](Map& m, const KeyType& k) -> MappedType& {
        auto it = m.find(k);
        if (it == m.end()) {
          throw key_error();
        }
        return it->second;
      },
      return_value_policy::reference_internal  // ref + keepalive
  );

  cl.def("__contains__", [](Map& m, const KeyType& k) -> bool {
    auto it = m.find(k);
    if (it == m.end()) {
      return false;
    }
    return true;
  });
  // Fallback for when the object is not of the key type
  cl.def("__contains__", [](Map&, const object&) -> bool { return false; });

  // Assignment provided only if the type is copyable
  detail::map_assignment<Map, Class_>(cl);

  cl.def("__delitem__", [](Map& m, const KeyType& k) {
    auto it = m.find(k);
    if (it == m.end()) {
      throw key_error();
    }
    m.erase(it);
  });

  // Always use a lambda in case of `using` declaration
  cl.def("__len__", [](const Map& m) { return m.size(); });

  return cl;
}
}  // namespace PYBIND11_NAMESPACE
