#ifndef __ARRAY_H
#define __ARRAY_H

#include <Python.h>

#define Array_Check(op) PyObject_TypeCheck(op, &svms_ArrayType)

enum array_type {
  AT_DOUBLE, AT_INT, AT_DOC,
};

typedef struct {
  PyObject_HEAD
  void *array; /* The underlying array, with "length"+"offset" entries. */
  int length; /* Length of the underlying array. */
  int offset; /* Where item 0 is in the array. */
  enum array_type type;
  char ifree;
} svms_ArrayObject;

extern PyTypeObject svms_ArrayType;
extern PyTypeObject svms_ArrayIterType;

svms_ArrayObject* Array_FromArray(void*array,int length,int offset,enum array_type type);
int Array_InitType(PyObject *module);

#endif // __ARRAY_H
