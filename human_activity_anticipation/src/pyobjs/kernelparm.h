#ifndef __KERNELPARM_H
#define __KERNELPARM_H

#include <Python.h>
#include "../svm_struct_api_types.h"

#define KernelParm_Check(op) PyObject_TypeCheck(op, &svms_KernelParmType)

typedef struct {
  PyObject_HEAD
  KERNEL_PARM *kparm; /* Underlying very important C structure. */
  KERNEL_PARM int_kparm;
} svms_KernelParmObject;

extern PyTypeObject svms_KernelParmType;

svms_KernelParmObject* KernelParm_FromKernelParm(KERNEL_PARM *sm);
KERNEL_PARM KernelParm_AsKernelParm(PyObject *kparm);
int KernelParm_InitType(PyObject *module);

#endif // __SPARM_H
