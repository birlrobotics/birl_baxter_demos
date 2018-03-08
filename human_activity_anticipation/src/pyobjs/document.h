#ifndef __DOCUMENT_H
#define __DOCUMENT_H

#include <Python.h>
#include "../svm_light/svm_common.h"

#define Document_Check(op) PyObject_TypeCheck(op, &svms_DocumentType)

typedef struct {
  PyObject_HEAD
  int length;
  char ifree;
  DOC *doc;
} svms_DocumentObject;

extern PyTypeObject svms_DocumentType;
extern PyTypeObject svms_DocumentIterType;

svms_DocumentObject* Document_FromDocument(DOC *doc);
// Returns a **copy** of the enclosed document.
DOC *Document_AsDocument(PyObject *sparse);

int Document_InitType(PyObject *module);

#endif // __DOCUMENT_H
