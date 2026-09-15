// Private CPython adapter. Retains the official model/data owners and the GIL;
// callers must not concurrently access these data from another native thread.
#define PY_SSIZE_T_CLEAN
#include <Python.h>
#include <mujoco/mujoco.h>
#include <cmath>
#include <memory>
#include <vector>

namespace {
constexpr const char* capsule_name = "tianji.mujoco.kernel";
struct Kernel {
  PyObject* model_owner = nullptr;
  PyObject* data_owner = nullptr;
  mjModel* model = nullptr;
  mjData* data = nullptr;
  std::vector<std::vector<int>> groups;
  std::vector<double> values;
  ~Kernel() { Py_XDECREF(data_owner); Py_XDECREF(model_owner); }
};
struct Ref {
  PyObject* p;
  explicit Ref(PyObject* value): p(value) {}
  ~Ref() { Py_XDECREF(p); }
};
bool sequence(PyObject* p) { return PyList_CheckExact(p) || PyTuple_CheckExact(p); }
PyObject* item(PyObject* p, Py_ssize_t i) {
  return PyList_CheckExact(p) ? PyList_GET_ITEM(p, i) : PyTuple_GET_ITEM(p, i);
}
PyObject* invalid(const char* message) { PyErr_SetString(PyExc_ValueError, message); return nullptr; }
void destroy(PyObject* capsule) {
  auto* k = static_cast<Kernel*>(PyCapsule_GetPointer(capsule, capsule_name));
  delete k;
}
bool exact_type(PyObject* obj, PyObject* module, const char* name) {
  Ref type(PyObject_GetAttrString(module, name));
  return type.p && reinterpret_cast<PyObject*>(Py_TYPE(obj)) == type.p;
}
PyObject* create_impl(PyObject* args) {
  PyObject *model, *data, *groups;
  if (!PyArg_ParseTuple(args, "OOO", &model, &data, &groups)) return nullptr;
  Ref module(PyImport_ImportModule("mujoco"));
  if (!module.p) return nullptr;
  if (!exact_type(model, module.p, "MjModel") || !exact_type(data, module.p, "MjData"))
    return invalid("official MuJoCo model/data required");
  Ref owner(PyObject_GetAttrString(data, "model"));
  if (!owner.p) return nullptr;
  if (owner.p != model) return invalid("MuJoCo data belongs to a different model");
  Ref version(PyObject_CallMethod(module.p, "mj_version", nullptr));
  if (!version.p) return nullptr;
  if (PyLong_AsLong(version.p) != mjVERSION_HEADER || mj_version() != mjVERSION_HEADER)
    return invalid("MuJoCo runtime/header version mismatch; rebuild native MuJoCo");
  Ref ma(PyObject_GetAttrString(model, "_address"));
  Ref da(PyObject_GetAttrString(data, "_address"));
  if (!ma.p || !da.p) return nullptr;
  auto k = std::make_unique<Kernel>();
  k->model = static_cast<mjModel*>(PyLong_AsVoidPtr(ma.p));
  k->data = static_cast<mjData*>(PyLong_AsVoidPtr(da.p));
  if (PyErr_Occurred()) return nullptr;
  if (!k->model || !k->data) return invalid("null MuJoCo address");
  if (!sequence(groups) || PySequence_Size(groups) > 4)
    return invalid("at most four joint groups required");
  std::vector<bool> used(k->model->nq, false);
  for (Py_ssize_t g = 0; g < PySequence_Size(groups); ++g) {
    auto* group = item(groups, g);
    if (!sequence(group) || PySequence_Size(group) > k->model->nq)
      return invalid("invalid joint address group");
    std::vector<int> addresses;
    for (Py_ssize_t i = 0; i < PySequence_Size(group); ++i) {
      auto* value = item(group, i);
      if (!PyLong_CheckExact(value)) return invalid("joint address must be an integer");
      long address = PyLong_AsLong(value);
      if (PyErr_Occurred()) return nullptr;
      if (address < 0 || address >= k->model->nq || used[address])
        return invalid("joint address out of range or duplicated");
      used[address] = true;
      addresses.push_back(static_cast<int>(address));
      k->values.push_back(0.);
    }
    k->groups.push_back(std::move(addresses));
  }
  Py_INCREF(model); k->model_owner = model;
  Py_INCREF(data); k->data_owner = data;
  PyObject* capsule = PyCapsule_New(k.get(), capsule_name, destroy);
  if (capsule) k.release();
  return capsule;
}
PyObject* create(PyObject*, PyObject* args) {
  try { return create_impl(args); }
  catch (const std::bad_alloc&) { return PyErr_NoMemory(); }
}
PyObject* apply(PyObject*, PyObject* args) {
  PyObject *capsule, *batch;
  if (!PyArg_ParseTuple(args, "OO", &capsule, &batch)) return nullptr;
  auto* k = static_cast<Kernel*>(PyCapsule_GetPointer(capsule, capsule_name));
  if (!k) return nullptr;
  if (!sequence(batch) || PySequence_Size(batch) != static_cast<Py_ssize_t>(k->groups.size()))
    return invalid("joint batch must match configured groups");
  size_t offset = 0;
  // Validate the ENTIRE batch before any qpos writes. Exact builtins prevent
  // user-defined conversion callbacks/reentrancy during the transaction.
  for (size_t g = 0; g < k->groups.size(); ++g) {
    auto* values = item(batch, g);
    const size_t count = k->groups[g].size();
    if (values != Py_None) {
      if (!sequence(values) || PySequence_Size(values) != static_cast<Py_ssize_t>(count))
        return invalid("joint group length mismatch");
      for (size_t i = 0; i < count; ++i) {
        auto* value = item(values, i);
        if (!PyFloat_CheckExact(value) && !PyLong_CheckExact(value))
          return invalid("joint position must be a finite builtin number");
        const double q = PyFloat_AsDouble(value);
        if (PyErr_Occurred()) return nullptr;
        if (!std::isfinite(q)) return invalid("nonfinite joint position");
        k->values[offset + i] = q;
      }
    }
    offset += count;
  }
  offset = 0;
  for (size_t g = 0; g < k->groups.size(); ++g) {
    if (item(batch, g) != Py_None)
      for (size_t i = 0; i < k->groups[g].size(); ++i)
        k->data->qpos[k->groups[g][i]] = k->values[offset + i];
    offset += k->groups[g].size();
  }
  mj_forward(k->model, k->data);
  Py_RETURN_NONE;
}
PyObject* positions(PyObject*, PyObject* args) {
  PyObject* capsule;
  Py_ssize_t start, count;
  if (!PyArg_ParseTuple(args, "Onn", &capsule, &start, &count)) return nullptr;
  auto* k = static_cast<Kernel*>(PyCapsule_GetPointer(capsule, capsule_name));
  if (!k) return nullptr;
  const auto size = static_cast<Py_ssize_t>(k->groups.size());
  if (start < 0 || start > size || count < 0 || count > size - start)
    return invalid("feedback group range invalid");
  Py_ssize_t length = 0;
  for (Py_ssize_t g = start; g < start + count; ++g) length += k->groups[g].size();
  PyObject* result = PyList_New(length);
  if (!result) return nullptr;
  Py_ssize_t i = 0;
  for (Py_ssize_t g = start; g < start + count; ++g) {
    for (int address : k->groups[g]) {
      PyObject* value = PyFloat_FromDouble(k->data->qpos[address]);
      if (!value) { Py_DECREF(result); return nullptr; }
      PyList_SET_ITEM(result, i++, value);
    }
  }
  return result;
}
PyMethodDef methods[] = {
  {"create", create, METH_VARARGS, "Bind official owned MuJoCo objects and joint groups."},
  {"apply", apply, METH_VARARGS, "Apply validated joint groups and forward kinematics."},
  {"positions", positions, METH_VARARGS, "Copy joint feedback without exposing mutable storage."},
  {nullptr, nullptr, 0, nullptr}
};
PyModuleDef definition = {PyModuleDef_HEAD_INIT, "_tianji_mujoco", nullptr, -1, methods,
                         nullptr, nullptr, nullptr, nullptr};
}
PyMODINIT_FUNC PyInit__tianji_mujoco() { return PyModule_Create(&definition); }
