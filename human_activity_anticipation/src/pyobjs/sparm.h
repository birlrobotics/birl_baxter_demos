#ifndef __SPARM_H
#define __SPARM_H

#include <Python.h>
#include "../svm_struct_api_types.h"

#define Sparm_Check(op) PyObject_TypeCheck(op, &svms_SparmType)

typedef struct {
  PyObject_HEAD
  STRUCT_LEARN_PARM *sparm; /* Underlying very important C structure. */
  PyObject *dict;
} svms_SparmObject;

extern PyTypeObject svms_SparmType;

svms_SparmObject* Sparm_FromSparm(STRUCT_LEARN_PARM *sparm);
int Sparm_InitType(PyObject *module);

#endif // __SPARM_H
