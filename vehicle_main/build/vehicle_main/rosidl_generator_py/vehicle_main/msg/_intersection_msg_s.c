// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from vehicle_main:msg/IntersectionMsg.idl
// generated code does not contain a copyright notice
#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
#include <Python.h>
#include <stdbool.h>
#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-function"
#endif
#include "numpy/ndarrayobject.h"
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif
#include "rosidl_runtime_c/visibility_control.h"
#include "vehicle_main/msg/detail/intersection_msg__struct.h"
#include "vehicle_main/msg/detail/intersection_msg__functions.h"

#include "rosidl_runtime_c/primitives_sequence.h"
#include "rosidl_runtime_c/primitives_sequence_functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool vehicle_main__msg__intersection_msg__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[51];
    {
      char * class_name = NULL;
      char * module_name = NULL;
      {
        PyObject * class_attr = PyObject_GetAttrString(_pymsg, "__class__");
        if (class_attr) {
          PyObject * name_attr = PyObject_GetAttrString(class_attr, "__name__");
          if (name_attr) {
            class_name = (char *)PyUnicode_1BYTE_DATA(name_attr);
            Py_DECREF(name_attr);
          }
          PyObject * module_attr = PyObject_GetAttrString(class_attr, "__module__");
          if (module_attr) {
            module_name = (char *)PyUnicode_1BYTE_DATA(module_attr);
            Py_DECREF(module_attr);
          }
          Py_DECREF(class_attr);
        }
      }
      if (!class_name || !module_name) {
        return false;
      }
      snprintf(full_classname_dest, sizeof(full_classname_dest), "%s.%s", module_name, class_name);
    }
    assert(strncmp("vehicle_main.msg._intersection_msg.IntersectionMsg", full_classname_dest, 50) == 0);
  }
  vehicle_main__msg__IntersectionMsg * ros_message = _ros_message;
  {  // position_x
    PyObject * field = PyObject_GetAttrString(_pymsg, "position_x");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->position_x = PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // position_y
    PyObject * field = PyObject_GetAttrString(_pymsg, "position_y");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->position_y = PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // num_directions
    PyObject * field = PyObject_GetAttrString(_pymsg, "num_directions");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->num_directions = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // directions
    PyObject * field = PyObject_GetAttrString(_pymsg, "directions");
    if (!field) {
      return false;
    }
    // TODO(dirk-thomas) use a better way to check the type before casting
    assert(field->ob_type != NULL);
    assert(field->ob_type->tp_name != NULL);
    assert(strcmp(field->ob_type->tp_name, "numpy.ndarray") == 0);
    PyArrayObject * seq_field = (PyArrayObject *)field;
    Py_INCREF(seq_field);
    assert(PyArray_NDIM(seq_field) == 1);
    assert(PyArray_TYPE(seq_field) == NPY_UINT32);
    Py_ssize_t size = 32;
    uint32_t * dest = ros_message->directions;
    for (Py_ssize_t i = 0; i < size; ++i) {
      uint32_t tmp = *(npy_uint32 *)PyArray_GETPTR1(seq_field, i);
      memcpy(&dest[i], &tmp, sizeof(uint32_t));
    }
    Py_DECREF(seq_field);
    Py_DECREF(field);
  }
  {  // intersection_state
    PyObject * field = PyObject_GetAttrString(_pymsg, "intersection_state");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->intersection_state = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // intersection_next_state
    PyObject * field = PyObject_GetAttrString(_pymsg, "intersection_next_state");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->intersection_next_state = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // intersection_switch_time
    PyObject * field = PyObject_GetAttrString(_pymsg, "intersection_switch_time");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->intersection_switch_time = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * vehicle_main__msg__intersection_msg__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of IntersectionMsg */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("vehicle_main.msg._intersection_msg");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "IntersectionMsg");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  vehicle_main__msg__IntersectionMsg * ros_message = (vehicle_main__msg__IntersectionMsg *)raw_ros_message;
  {  // position_x
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->position_x);
    {
      int rc = PyObject_SetAttrString(_pymessage, "position_x", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // position_y
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->position_y);
    {
      int rc = PyObject_SetAttrString(_pymessage, "position_y", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // num_directions
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->num_directions);
    {
      int rc = PyObject_SetAttrString(_pymessage, "num_directions", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // directions
    PyObject * field = NULL;
    field = PyObject_GetAttrString(_pymessage, "directions");
    if (!field) {
      return NULL;
    }
    assert(field->ob_type != NULL);
    assert(field->ob_type->tp_name != NULL);
    assert(strcmp(field->ob_type->tp_name, "numpy.ndarray") == 0);
    PyArrayObject * seq_field = (PyArrayObject *)field;
    assert(PyArray_NDIM(seq_field) == 1);
    assert(PyArray_TYPE(seq_field) == NPY_UINT32);
    assert(sizeof(npy_uint32) == sizeof(uint32_t));
    npy_uint32 * dst = (npy_uint32 *)PyArray_GETPTR1(seq_field, 0);
    uint32_t * src = &(ros_message->directions[0]);
    memcpy(dst, src, 32 * sizeof(uint32_t));
    Py_DECREF(field);
  }
  {  // intersection_state
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->intersection_state);
    {
      int rc = PyObject_SetAttrString(_pymessage, "intersection_state", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // intersection_next_state
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->intersection_next_state);
    {
      int rc = PyObject_SetAttrString(_pymessage, "intersection_next_state", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // intersection_switch_time
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->intersection_switch_time);
    {
      int rc = PyObject_SetAttrString(_pymessage, "intersection_switch_time", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
