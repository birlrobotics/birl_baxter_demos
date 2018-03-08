#ifndef __STRUCTMODEL_H
#define __STRUCTMODEL_H

#include <Python.h>
#include "../svm_struct_api_types.h"

#define StructModel_Check(op) PyObject_TypeCheck(op, &svms_StructModelType)

typedef struct {
  PyObject_HEAD
  STRUCTMODEL *sm; /* Underlying very important C structure. */
  char ifree;
  PyObject *dict;
} svms_StructModelObject;

extern PyTypeObject svms_StructModelType;

svms_StructModelObject* StructModel_FromStructModel(STRUCTMODEL *sm);
int StructModel_InitType(PyObject *module);

#endif // __SPARM_H
