#ifndef _UTIL_H
#define _UTIL_H

#include <Python.h>

#if PY_MAJOR_VERSION == 2
#if PY_MINOR_VERSION < 4
#define Py_RETURN_NONE return Py_INCREF(Py_None), Py_None
#endif // PY_MINOR_VERSION < 4
#endif // PY_MAJOR_VERSION == 2

#endif // _UTIL_H
