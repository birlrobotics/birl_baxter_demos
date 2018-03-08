#ifndef __DEFAULT_H
#define __DEFAULT_H

#include <Python.h>

/**
 * This file defines default functionality for SVM^python API
 * functions.  The intention is that the svm_struct_api.c code will
 * call these automatically if a module is used that does not provide
 * its own implementation of these.
 */

/** This is a "no-operation" function that just returns "None".  Many
    of the default behaviors are defined as doing nothing. */
PyObject* defaultPy_noop(PyObject*self, PyObject *args);
PyObject* defaultPy_fmvc_sm(PyObject*self, PyObject *args);
PyObject* defaultPy_fmvc(PyObject*self, PyObject *args);
PyObject* defaultPy_loss(PyObject*self, PyObject *args);
PyObject* defaultPy_write_model(PyObject*self, PyObject *args);
PyObject* defaultPy_read_model(PyObject*self, PyObject *args);
PyObject* defaultPy_print_iteration_stats(PyObject*self, PyObject *args);
PyObject* defaultPy_print_learning_stats(PyObject*self, PyObject *args);
PyObject* defaultPy_print_testing_stats(PyObject*self, PyObject *args);
PyObject* defaultPy_print_help(PyObject*self, PyObject *args);
PyObject* defaultPy_write_label(PyObject*self, PyObject *args);
PyObject* defaultPy_eval_prediction(PyObject*self, PyObject *args);

#endif __DEFAULT_H
