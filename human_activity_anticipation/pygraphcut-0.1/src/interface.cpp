/* Copyright 2007 Thomas Finley, tfinley@gmail.com */

#include <Python.h>
#include "graphobj.h"
#include "energyobj.h"
#include "qpboobj.h"
static PyMethodDef GCOptMethods[] = {
  {NULL, NULL, 0, NULL}
};

void initer(char *modname) {
  PyObject *m;
  // Initialize the model.
  m = Py_InitModule3
    (modname, GCOptMethods,
     "Module for performing graph cuts and energy minimization.");
  if (m==NULL) return;
  // Set up the Graph and Energy classes.
//  if (PyType_Ready(&GraphType) < 0) return;  
//  Py_INCREF(&GraphType);
 // PyModule_AddObject(m, "Graph", (PyObject*)&GraphType);

 // if (PyType_Ready(&EnergyType) < 0) return;
 // Py_INCREF(&EnergyType);
//  PyModule_AddObject(m, "Energy", (PyObject*)&EnergyType);

  if (PyType_Ready(&QPBOType) < 0) return;
  Py_INCREF(&QPBOType);
  PyModule_AddObject(m, "QPBO", (PyObject*)&QPBOType);
}

extern "C" PyMODINIT_FUNC initgraphcut(void) {
  initer("graphcut");
}
