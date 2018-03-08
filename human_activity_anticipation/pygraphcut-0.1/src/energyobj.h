/* Copyright 2007 Thomas Finley, tfinley@gmail.com */

#ifndef __ENERGYOBJ_H
#define __ENERGYOBJ_H

#include <Python.h>
#include "graphobj.h"
#include "energy.h"

//using namespace std;

typedef struct {
  PyObject_HEAD 
  Energy *energy;
  
} EnergyObject;

extern PyTypeObject EnergyType;

#endif // __ENERGYOBJ_H
