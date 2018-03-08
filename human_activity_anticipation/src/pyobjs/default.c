#import "default.h"
#import "svmapi_globals.h"
#import "svmapi.h"

/* This will search for the default implementation of the indicated
   function name, with a hard failure, that is, if it does not find
   the function in the indicated module, the process will print an
   error message and exit.  */
static PyObject* getFunction(const char *funcname) {
  PyObject *pDict, *pFunc;
  pDict = PyModule_GetDict(svmapi_usermodule);
  pFunc = PyDict_GetItemString(pDict, funcname);
  if (pFunc) return pFunc;
  pFunc = PyObject_GetAttrString(svmapi_thismodule, funcname);
  return pFunc;

  pDict = PyModule_GetDict(svmapi_thismodule);
  pFunc = PyDict_GetItemString(pDict, funcname);
  if (pFunc) return pFunc;
  fprintf(stderr, "Could not find function %s!\n", funcname);
  Py_Exit(1);
  return NULL;
}

PyObject* defaultPy_noop(PyObject*self, PyObject *args) {
  Py_RETURN_NONE;
}

PyObject* defaultPy_fmvc_sm(PyObject*self, PyObject *args) {
  /* Whether slack or margin, the default behavior is to attempt a
     call on the general find_most_violated_constraint function. */
  PyObject *pFunc = getFunction(PYTHON_FMVC);
  if (!pFunc) return NULL;
  return PyObject_CallObject(pFunc, args);
}

PyObject* defaultPy_fmvc(PyObject*self, PyObject *args) {
  PyObject *x, *y, *sm, *sparm;
  if (!PyArg_ParseTuple(args, "OOOO", &x, &y, &sm, &sparm)) {
    return NULL;
  }
  PyObject *pFunc = getFunction(PYTHON_CLASSIFY_EXAMPLE);
  if (!pFunc) return NULL;
  return PyObject_CallFunctionObjArgs(pFunc, x, sm, sparm, NULL);
}

PyObject* defaultPy_loss(PyObject*self, PyObject *args) {
  PyObject *y, *ybar, *sparm;
  if (!PyArg_ParseTuple(args, "OOO", &y, &ybar, &sparm)) {
    return NULL;
  }
  int result = PyObject_RichCompareBool(y, ybar, Py_EQ);
  return PyFloat_FromDouble((result==0) ? 1.0 : 0.0);
}

PyObject* defaultPy_write_model(PyObject*self, PyObject *args) {
  char*file;
  PyObject*sm, *sparm, *pValue;
  if (!PyArg_ParseTuple(args, "sOO", &file, &sm, &sparm)) return NULL;
  
  PyObject *bz2Module, *pickleModule, *outfile;
  // Attempt to load the cPickle or pickle module.
  pickleModule = PyImport_ImportModule("cPickle");
  if (pickleModule==NULL) {
    PyErr_Clear();
    pickleModule = PyImport_ImportModule("pickle");
  }
  if (pickleModule==NULL) return NULL;

  // Attempt to load the BZ2 module.
  bz2Module = PyImport_ImportModule("bz2");
  if (bz2Module==NULL) { 
    PyErr_Clear();
    PyErr_Warn(PyExc_RuntimeWarning, "could not load bz2 module");
  }
  // Construct the file.
  if (bz2Module) {
    outfile = PyObject_CallMethod(bz2Module, "BZ2File", "ss", file, "w");
    if (outfile==NULL) goto error;
  } else {
    outfile = PyFile_FromString(file, "w");
    if (outfile==NULL) goto error;
  }
  // Dump the struct model.
  pValue = PyObject_CallMethod
    (pickleModule,"dump","NOi",sm,outfile,-1);
  PyObject_CallMethod(outfile, "close", NULL);
  Py_DECREF(outfile);
  // Check for errors.
  if (pValue == NULL) {
    goto error;
  }
  // Clean up.
  Py_DECREF(pValue);
  Py_XDECREF(bz2Module);
  Py_DECREF(pickleModule);
  Py_RETURN_NONE;
 error:
  Py_XDECREF(bz2Module);
  Py_DECREF(pickleModule);
  return NULL;
}

PyObject* defaultPy_read_model(PyObject*self, PyObject *args) {
  char*file;
  PyObject*sm, *sparm;
  sm = NULL;
  if (!PyArg_ParseTuple(args, "sO", &file, &sparm)) return NULL;
  // Do the default actions for reading a model file.
  PyObject *bz2Module, *pickleModule, *infile;
  // Attempt to load the cPickle or pickle module.
  pickleModule = PyImport_ImportModule("cPickle");
  if (pickleModule==NULL) {
    PyErr_Clear();
    pickleModule = PyImport_ImportModule("pickle");
  }
  if (pickleModule==NULL) return NULL;
  // Attempt to load the BZ2 module.
  bz2Module = PyImport_ImportModule("bz2");
  if (bz2Module==NULL) { 
    PyErr_Clear();
    PyErr_Warn(PyExc_RuntimeWarning, "could not load bz2 module");
  }
  // Construct the file.
  if (bz2Module) {
    infile = PyObject_CallMethod(bz2Module, "BZ2File", "s", file);
    if (infile==NULL) goto error;
  } else {
    infile = PyFile_FromString(file, "r");
    if (infile==NULL) goto error;
  }
  // Read the structure model.
  sm = PyObject_CallMethod(pickleModule, "load", "O", infile);
  PyObject_CallMethod(infile, "close", NULL);
  Py_DECREF(infile);
 error:
  Py_XDECREF(bz2Module);
  Py_DECREF(pickleModule);
  return sm;
}

PyObject* defaultPy_print_iteration_stats(PyObject*self, PyObject *args) {
  Py_RETURN_NONE;
}

PyObject* defaultPy_print_learning_stats(PyObject*self, PyObject *args) {
  Py_RETURN_NONE;
}

PyObject* defaultPy_print_testing_stats(PyObject*self, PyObject *args) {
  Py_RETURN_NONE;
}

PyObject* defaultPy_eval_prediction(PyObject*self, PyObject *args) {
  Py_RETURN_NONE;
}

PyObject* defaultPy_write_label(PyObject*self, PyObject *args) {
  PyObject *file, *y, *ystr, *ret;
  if (!PyArg_ParseTuple(args, "OO", &file, &y)) return NULL;
  ystr = PyObject_Str(y);
  if (ystr == NULL) return NULL;
  ret = PyObject_CallMethod(file, "write", "O", ystr);
  Py_DECREF(ystr);
  if (ret == NULL) return NULL;
  if (PyObject_CallMethod(file, "write", "c", '\n') == NULL) return NULL;
  Py_RETURN_NONE;
}

PyObject* defaultPy_print_help(PyObject*self, PyObject *args) {
  PyObject *pDict, *pValue;
  pDict = PyModule_GetDict(svmapi_thismodule);
  pValue = PyDict_GetItemString(pDict, "default_help");
  if (pValue == NULL) return NULL;
  if (PyObject_Print(pValue, stdout, Py_PRINT_RAW) == -1) return NULL;
  Py_RETURN_NONE;
}
