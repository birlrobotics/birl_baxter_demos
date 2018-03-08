#ifndef __CONSTRAINTS_H
#define __CONSTRAINTS_H

#include <Python.h>
#include "../svm_struct/svm_struct_common.h"

#define Constraints_Check(op) PyObject_TypeCheck(op, &svms_ConstraintsType)

typedef struct {
  PyObject_HEAD
  CONSTSET cset; /* Underlying very important C structure. */
} svms_ConstraintsObject;

extern PyTypeObject svms_ConstraintsType;
extern PyTypeObject svms_ConstraintsIterType;

svms_ConstraintsObject* Constraints_FromConstraints(CONSTSET sample);
int Constraints_InitType(PyObject *module);

#endif // __CONSTRAINTS_H
