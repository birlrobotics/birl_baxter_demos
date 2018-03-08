/* Copyright 2007 Thomas Finley, tfinley@gmail.com */

#ifndef __GRAPHOBJ_H
#define __GRAPHOBJ_H

#include <Python.h>
#include "graph.h"
//#include "instances.inc"

//using namespace std;

typedef struct {
  PyObject_HEAD 
  Graph *graph;
  
} GraphObject;

extern PyTypeObject GraphType;

#endif // __GRAPHOBJ_H
