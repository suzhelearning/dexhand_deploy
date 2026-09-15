// CPython boundary only; numeric receipt state lives in execution_guard.hpp.
// Calls retain the GIL and never block or call external services.
#define PY_SSIZE_T_CLEAN
#include <Python.h>
#include "execution_guard.hpp"
#include <cmath>
#include <memory>

namespace {
constexpr const char* capsule_name = "tianji.execution_guard.v1";
struct PythonError {};
template<class F> PyObject* protect(F fn) noexcept {
  try { return fn(); }
  catch (const PythonError&) { return nullptr; }
  catch (const std::bad_alloc&) { return PyErr_NoMemory(); }
  catch (const tianji_control::StateError& e) {
    auto* value = PyUnicode_DecodeUTF8(e.detail.data(),e.detail.size(),"surrogatepass");
    if (value) { PyErr_SetObject(PyExc_RuntimeError,value); Py_DECREF(value); }
  }
  catch (const std::invalid_argument& e) { PyErr_SetString(PyExc_ValueError, e.what()); }
  catch (const std::exception& e) { PyErr_SetString(PyExc_RuntimeError, e.what()); }
  catch (...) { PyErr_SetString(PyExc_RuntimeError, "unknown native guard failure"); }
  return nullptr;
}
std::int64_t positive(PyObject* value, const char* name) {
  if (value && PyLong_CheckExact(value)) {
    const auto result = PyLong_AsLongLong(value);
    if (!PyErr_Occurred() && result > 0) return result;
    PyErr_Clear();
  }
  throw std::invalid_argument(std::string(name)+" must be positive int64");
}
bool int64(PyObject* value, std::int64_t& out) {
  if (!value || !PyLong_CheckExact(value)) return false;
  out = PyLong_AsLongLong(value);
  if (PyErr_Occurred()) { PyErr_Clear(); return false; }
  return true;
}
std::string text(PyObject* value) {
  Py_ssize_t size;
  const char* data = PyUnicode_AsUTF8AndSize(value, &size);
  if (!data) {
    if (!PyErr_ExceptionMatches(PyExc_UnicodeEncodeError)) throw PythonError{};
    PyErr_Clear();
    auto* encoded = PyUnicode_AsEncodedString(value,"utf-8","surrogatepass");
    if (!encoded) throw PythonError{};
    std::string result;
    try { result.assign(PyBytes_AS_STRING(encoded),PyBytes_GET_SIZE(encoded)); }
    catch (...) { Py_DECREF(encoded); throw; }
    Py_DECREF(encoded);
    return result;
  }
  return std::string(data, size);
}
bool equal(PyObject* a, PyObject* b) {
  if (!a || !b || Py_TYPE(a) != Py_TYPE(b)) return false;
  const auto result = PyObject_RichCompareBool(a,b,Py_EQ);
  if (result < 0) throw PythonError{};
  return result != 0;
}
bool literal(PyObject* value, const char* expected) {
  if (!value || !PyUnicode_Check(value)) return false;
  return text(value) == expected;
}
tianji_control::Positions positions(PyObject* value) {
  if (!value || !PyDict_Check(value) || PyDict_Size(value) != 2 ||
      !PyDict_GetItemString(value,"left") || !PyDict_GetItemString(value,"right"))
    throw std::invalid_argument("positions must include exactly both arms");
  tianji_control::Positions result;
  int index = 0;
  for (const auto* side : {"left", "right"}) {
    auto* q = PyDict_GetItemString(value, side);
    if ((!PyList_Check(q) && !PyTuple_Check(q)) || PySequence_Size(q) != 7)
      throw std::invalid_argument("positions must be finite seven-joint vectors");
    for (int i=0; i<7; ++i) {
      auto* v = PyList_Check(q) ? PyList_GET_ITEM(q,i) : PyTuple_GET_ITEM(q,i);
      if (!PyFloat_CheckExact(v) && !PyLong_CheckExact(v))
        throw std::invalid_argument("positions must be finite seven-joint vectors");
      const double number = PyFloat_AsDouble(v);
      if (PyErr_Occurred()) throw PythonError{};
      if (!std::isfinite(number)) throw std::invalid_argument("positions must be finite seven-joint vectors");
      result[index++] = number;
    }
  }
  return result;
}
struct Holder {
  tianji_control::ExecutionGuard core;
  PyObject* config;
  Holder(PyObject* c, std::int64_t age, std::int64_t count) : core(age,count), config(Py_NewRef(c)) {}
  ~Holder() { Py_DECREF(config); }
};
Holder& holder(PyObject* capsule) {
  auto* pointer = static_cast<Holder*>(PyCapsule_GetPointer(capsule, capsule_name));
  if (!pointer) throw PythonError{};
  return *pointer;
}
void destroy(PyObject* capsule) { delete static_cast<Holder*>(PyCapsule_GetPointer(capsule, capsule_name)); }
PyObject* create(PyObject*, PyObject* config) {
  return protect([&]() -> PyObject* {
    if (!PyDict_Check(config)) throw std::invalid_argument("guard configuration must be a dict");
    for (auto* key : {"run_id", "coordinator_instance_id", "router_zid"}) {
      auto* v = PyDict_GetItemString(config,key);
      if (!v || !PyUnicode_Check(v) || PyUnicode_GetLength(v) == 0)
        throw std::invalid_argument("execution identities must be explicit nonempty strings");
    }
    (void)positive(PyDict_GetItemString(config,"execution_epoch"),"execution_epoch");
    const auto age = positive(PyDict_GetItemString(config,"maximum_receipt_age_ns"),"maximum_receipt_age_ns");
    const auto count = positive(PyDict_GetItemString(config,"max_in_flight"),"max_in_flight");
    auto state = std::make_unique<Holder>(config, age, count);
    auto* result = PyCapsule_New(state.get(), capsule_name, destroy);
    if (!result) throw PythonError{};
    state.release();
    return result;
  });
}
PyObject* check(PyObject*, PyObject* args) {
  return protect([&]() -> PyObject* {
    PyObject *capsule, *now;
    if (!PyArg_UnpackTuple(args,"check",2,2,&capsule,&now)) throw PythonError{};
    return PyBool_FromLong(holder(capsule).core.check(positive(now,"now_ns")));
  });
}
PyObject* pause(PyObject*, PyObject* args) {
  return protect([&]() -> PyObject* {
    PyObject *capsule, *reason;
    if (!PyArg_UnpackTuple(args,"pause",2,2,&capsule,&reason)) throw PythonError{};
    if (!PyUnicode_Check(reason)) throw std::invalid_argument("pause reason required");
    holder(capsule).core.pause(text(reason));
    Py_RETURN_NONE;
  });
}
PyObject* register_tick(PyObject*, PyObject* args) {
  return protect([&]() -> PyObject* {
    PyObject *capsule, *tick, *now, *q;
    if (!PyArg_UnpackTuple(args,"register",4,4,&capsule,&tick,&now,&q)) throw PythonError{};
    auto& core = holder(capsule).core;
    auto id = positive(tick,"tick_id");
    core.validate_tick(id);
    const auto values = positions(q); // preserve validation order of Python owner
    core.register_tick(id, positive(now,"now_ns"), values);
    Py_RETURN_NONE;
  });
}
PyObject* observe(PyObject*, PyObject* args) {
  return protect([&]() -> PyObject* {
    PyObject *capsule, *receipt, *time;
    if (!PyArg_UnpackTuple(args,"observe",3,3,&capsule,&receipt,&time)) throw PythonError{};
    auto& state = holder(capsule);
    auto& core = state.core;
    const auto now = positive(time,"now_ns");
    if (!core.check(now) || !PyDict_Check(receipt)) Py_RETURN_FALSE;
    for (auto* key : {"run_id", "execution_epoch", "router_zid", "publisher_instance_id"}) {
      const char* config_key = std::string(key)=="publisher_instance_id" ? "coordinator_instance_id" : key;
      if (!equal(PyDict_GetItemString(receipt,key),PyDict_GetItemString(state.config,config_key))) Py_RETURN_FALSE;
    }
    std::int64_t tick;
    if (!int64(PyDict_GetItemString(receipt,"tick_id"),tick)) Py_RETURN_FALSE;
    const auto* pending = core.find(tick);
    if (!pending) Py_RETURN_FALSE;
    constexpr const char* keys[] = {"schema_version", "kind", "run_id", "execution_epoch", "tick_id",
      "timestamp_ns", "publisher_instance_id", "router_zid", "stage", "accepted", "reason", "command_position_rad"};
    bool schema = PyDict_Size(receipt) == 12;
    for (auto* key : keys) schema = schema && PyDict_GetItemString(receipt,key);
    std::int64_t version, timestamp;
    auto* accepted = PyDict_GetItemString(receipt,"accepted");
    auto* reason = PyDict_GetItemString(receipt,"reason");
    if (!schema || !int64(PyDict_GetItemString(receipt,"schema_version"),version) || version!=1 ||
        !literal(PyDict_GetItemString(receipt,"kind"),"arm_bilateral_receipt") ||
        !literal(PyDict_GetItemString(receipt,"stage"),"coordinator_command") ||
        !PyBool_Check(accepted) || !PyUnicode_Check(reason) ||
        !int64(PyDict_GetItemString(receipt,"timestamp_ns"),timestamp) || timestamp < pending->sent || timestamp > now) {
      core.pause("invalid receipt schema/time");
      Py_RETURN_FALSE;
    }
    tianji_control::Positions q;
    try { q = positions(PyDict_GetItemString(receipt,"command_position_rad")); }
    catch (const std::invalid_argument& e) { core.pause(e.what()); Py_RETURN_FALSE; }
    return PyBool_FromLong(core.observe(tick, accepted==Py_True, text(reason), q));
  });
}
PyObject* reason(PyObject*, PyObject* capsule) {
  return protect([&]() -> PyObject* {
    const auto& value = holder(capsule).core.reason();
    if (!value) Py_RETURN_NONE;
    return PyUnicode_DecodeUTF8(value->data(),value->size(),"surrogatepass");
  });
}
PyObject* in_flight(PyObject*, PyObject* capsule) {
  return protect([&]() { return PyLong_FromSize_t(holder(capsule).core.in_flight()); });
}
PyMethodDef methods[] = {
  {"create",create,METH_O,nullptr}, {"check",check,METH_VARARGS,nullptr},
  {"pause",pause,METH_VARARGS,nullptr}, {"register",register_tick,METH_VARARGS,nullptr},
  {"observe",observe,METH_VARARGS,nullptr}, {"reason",reason,METH_O,nullptr},
  {"in_flight",in_flight,METH_O,nullptr}, {nullptr,nullptr,0,nullptr}};
PyModuleDef module = {PyModuleDef_HEAD_INIT,"_tianji_execution",nullptr,-1,methods,nullptr,nullptr,nullptr,nullptr};
} // namespace
PyMODINIT_FUNC PyInit__tianji_execution() { return PyModule_Create(&module); }
