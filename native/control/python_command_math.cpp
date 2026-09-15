#define PY_SSIZE_T_CLEAN
#include <Python.h>
#include "command_math.hpp"
#include <stdexcept>

namespace {
struct PythonError {};
template<class F> PyObject* protect(F fn) noexcept {
  try { return fn(); }
  catch (const PythonError&) { return nullptr; }
  catch (const std::bad_alloc&) { return PyErr_NoMemory(); }
  catch (const std::exception& e) { PyErr_SetString(PyExc_ValueError, e.what()); return nullptr; }
}
double number(PyObject* obj, bool finite=true) {
  if (!obj || (!PyFloat_CheckExact(obj) && !PyLong_CheckExact(obj)))
    throw std::invalid_argument("builtin numeric value required");
  const double result = PyFloat_AsDouble(obj);
  if (PyErr_Occurred()) throw PythonError{};
  if (finite && !std::isfinite(result)) throw std::invalid_argument("finite value required");
  return result;
}
bool boolean(PyObject* obj) {
  if (!PyBool_Check(obj)) throw std::invalid_argument("boolean required");
  return obj == Py_True;
}
std::int64_t timestamp(PyObject* obj) {
  if (!PyLong_CheckExact(obj)) throw std::invalid_argument("nonnegative int64 timestamp required");
  const auto value = PyLong_AsLongLong(obj);
  if (PyErr_Occurred()) throw PythonError{};
  if (value < 0) throw std::invalid_argument("nonnegative int64 timestamp required");
  return value;
}
PyObject* item(PyObject* obj, Py_ssize_t i) {
  return PyList_CheckExact(obj) ? PyList_GET_ITEM(obj,i) : PyTuple_GET_ITEM(obj,i);
}
bool sequence(PyObject* obj, Py_ssize_t size) {
  return (PyList_CheckExact(obj) || PyTuple_CheckExact(obj)) && PySequence_Size(obj) == size;
}
tianji_control::Joints7 joints(PyObject* obj, bool finite=true) {
  if (!sequence(obj,7)) throw std::invalid_argument("seven joint values required");
  tianji_control::Joints7 out;
  for (int i=0;i<7;++i) out[i]=number(item(obj,i),finite);
  return out;
}
PyObject* list(const tianji_control::Joints7& values) {
  auto* result=PyList_New(7);
  if (!result) return nullptr;
  for (int i=0;i<7;++i) {
    auto* value=PyFloat_FromDouble(values[i]);
    if (!value) { Py_DECREF(result); return nullptr; }
    PyList_SET_ITEM(result,i,value);
  }
  return result;
}
double field(PyObject* obj, const char* key, bool positive) {
  if (!PyDict_CheckExact(obj)) throw std::invalid_argument("configuration dict required");
  const double value=number(PyDict_GetItemString(obj,key));
  if (positive ? value<=0 : value<0) throw std::invalid_argument("invalid command configuration");
  return value;
}
PyObject* validate(PyObject*, PyObject* args) {
  return protect([&]() -> PyObject* {
    PyObject *q,*old,*limits,*config,*now,*source,*anchor,*hold,*sim;
    if (!PyArg_ParseTuple(args,"OOOOOOOOO",&q,&old,&limits,&config,&now,&source,&anchor,&hold,&sim)) return nullptr;
    if (!sequence(limits,2)) throw std::invalid_argument("lower and upper limits required");
    const tianji_control::CommandConfig cfg{field(config,"maximum_command_step_rad",true),
      field(config,"command_step_time_window_s",false),field(config,"rate_hz",true),
      field(config,"proposal_timeout_s",true)};
    std::optional<std::int64_t> anch;
    if (anchor!=Py_None) anch=timestamp(anchor);
    auto result=tianji_control::validate_command(joints(q,false),joints(old),
      joints(item(limits,0)),joints(item(limits,1)),cfg,timestamp(now),timestamp(source),anch,
      boolean(hold),boolean(sim));
    return Py_BuildValue("iddd",static_cast<int>(result.error),result.delta,result.allowed,result.elapsed);
  });
}
PyObject* track(PyObject*, PyObject* args) {
  return protect([&]() -> PyObject* {
    PyObject *q,*old,*step,*clip,*hold;
    if (!PyArg_ParseTuple(args,"OOOOO",&q,&old,&step,&clip,&hold)) return nullptr;
    double maximum=number(step);
    if (maximum<=0) throw std::invalid_argument("positive command step required");
    return list(tianji_control::track_command(joints(q),joints(old),maximum,boolean(clip),boolean(hold)));
  });
}
PyObject* home(PyObject*, PyObject* args) {
  return protect([&]() -> PyObject* {
    PyObject *start,*target,*elapsed,*minimum,*speed;
    if (!PyArg_ParseTuple(args,"OOOOO",&start,&target,&elapsed,&minimum,&speed)) return nullptr;
    double duration=number(minimum), maximum=number(speed);
    if (duration<=0 || maximum<=0) throw std::invalid_argument("positive Home settings required");
    return list(tianji_control::home_command(joints(start),joints(target),number(elapsed),duration,maximum));
  });
}
PyMethodDef methods[]={
  {"validate",validate,METH_VARARGS,"Validate one fresh, authorized arm proposal."},
  {"track",track,METH_VARARGS,"Clip or hold an already validated target."},
  {"home",home,METH_VARARGS,"Bounded linear return with exact Home endpoint."},
  {nullptr,nullptr,0,nullptr}};
PyModuleDef module={PyModuleDef_HEAD_INIT,"_tianji_command_math",nullptr,-1,methods,
                    nullptr,nullptr,nullptr,nullptr};
}
PyMODINIT_FUNC PyInit__tianji_command_math() { return PyModule_Create(&module); }
