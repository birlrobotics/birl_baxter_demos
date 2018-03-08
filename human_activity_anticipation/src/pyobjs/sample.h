#ifndef __SAMPLE_H
#define __SAMPLE_H

#include <Python.h>
#include "../svm_struct/svm_struct_common.h"

#define Sample_Check(op) PyObject_TypeCheck(op, &svms_SampleType)

typedef struct {
  PyObject_HEAD
  SAMPLE sample; /* Underlying very important C structure. */
} svms_SampleObject;

extern PyTypeObject svms_SampleType;
extern PyTypeObject svms_SampleIterType;

svms_SampleObject* Sample_FromSample(SAMPLE sample);
int Sample_InitType(PyObject *module);

#endif // __SAMPLE_H
