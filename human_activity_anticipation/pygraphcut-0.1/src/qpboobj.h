/* Copyright 2007 Thomas Finley, tfinley@gmail.com */

#ifndef __QPBOOBJ_H
#define __QPBOOBJ_H

#include <Python.h>
//#include "graphobj.h"
#include "QPBO.h"

//using namespace std;

typedef struct {
  PyObject_HEAD 
  QPBO<double> *qpbo;
  
} QPBOObject;

extern PyTypeObject QPBOType;

#endif // __QPBOOBJ_H
