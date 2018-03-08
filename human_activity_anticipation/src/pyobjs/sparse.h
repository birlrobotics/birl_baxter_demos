#ifndef __SPARSE_H
#define __SPARSE_H

#include <Python.h>
#include "../svm_light/svm_common.h"

#define Sparse_Check(op) PyObject_TypeCheck(op, &svms_SparseType)

typedef struct {
  PyObject_HEAD
  int length;
  char ifree;
  SVECTOR *sparse;
  PyObject *owner;
} svms_SparseObject;

extern PyTypeObject svms_SparseType;
extern PyTypeObject svms_SparseIterType;

svms_SparseObject* Sparse_FromSparse(SVECTOR *sv, PyObject*owner);
// Returns a **copy** of the enclosed vector.
SVECTOR *Sparse_AsSparse(PyObject *sparse);

int Sparse_InitType(PyObject *module);

#endif // __SPARSE_H
