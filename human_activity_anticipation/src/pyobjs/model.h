#ifndef __MODEL_H
#define __MODEL_H

#include <Python.h>
#include "../svm_light/svm_common.h"

#define Model_Check(op) PyObject_TypeCheck(op, &svms_ModelType)

typedef struct {
  PyObject_HEAD
  unsigned char ifree;
  MODEL *model; /* Underlying very important C structure. */
} svms_ModelObject;

extern PyTypeObject svms_ModelType;

svms_ModelObject* Model_FromModel(MODEL *model);
int Model_InitType(PyObject *module);

#endif // __SPARM_H
