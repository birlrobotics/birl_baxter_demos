#include "sparm.h"
#include "structmember.h"
#include "svmapi_globals.h"

/* STRUCT LEARNING PARAM OBJECT. */

// Methods for the support vector list object.

static PyObject *Sparm_new(PyTypeObject *type, PyObject *args,
			   PyObject *kwds) {
  svms_SparmObject *self = NULL;
  self = (svms_SparmObject*) type->tp_alloc(type, 0);
  if (self != NULL) {
    self->sparm = NULL;
    self->dict = NULL;
  }
  return (PyObject *)self;
}

svms_SparmObject *Sparm_FromSparm(STRUCT_LEARN_PARM *sparm) {
  svms_SparmObject*so = (svms_SparmObject*)
    PyObject_GC_New(svms_SparmObject, &svms_SparmType);
  so->sparm = sparm;
  so->dict = (PyObject*)sparm->pydict;
  Py_INCREF(so->dict);
  return so;
}

static int Sparm_init(svms_SparmObject *self, PyObject *args,
		      PyObject *kwds) {
  PyObject *argv=NULL;
  // The constructor is intended only for use by de-picklers only.
  self->sparm = (STRUCT_LEARN_PARM*)my_malloc(sizeof(STRUCT_LEARN_PARM));
  self->sparm->pydict = NULL;
  self->dict = NULL;
  STRUCT_LEARN_PARM *sp = self->sparm;
  if (!PyArg_ParseTuple(args, "ddidO!iii", &sp->epsilon, &sp->newconstretrain,
			&sp->ccache_size, &sp->C, &PyList_Type, &argv,
			&sp->slack_norm, &sp->loss_type, &sp->loss_function)) {
    // Could not restore!
    free(sp);
    return -1;
  }

  // Try to restore the argument strings.
  sp->custom_argc = PyList_Size(argv);
  if (sp->custom_argc > sizeof(sp->custom_argv)/sizeof(sp->custom_argv[0])) {
    PyErr_Format(PyExc_ValueError, "argv max length %d but %d passed in",
		 (int)(sizeof(sp->custom_argv)/sizeof(sp->custom_argv[0])),
		 (int)sp->custom_argc);
    free(sp);
    return -1;
  }
  int i;
  for (i=0; i<sp->custom_argc; ++i) {
    char*c = PyString_AsString(PyList_GetItem(argv,i));
    if (c==NULL) {
      free(sp);
      return -1;
    }
    strncpy(sp->custom_argv[i], c, sizeof(sp->custom_argv[0]));
  }

  return 0;
}

static PyObject *Sparm_Str(svms_SparmObject *self) {
  STRUCT_LEARN_PARM *sp = self->sparm;
  PyObject *args = Py_BuildValue
    ("sddci", self->ob_type->tp_name, sp->C, sp->epsilon,
     sp->loss_type==1?'s':'m', sp->slack_norm);
  if (args==NULL) return NULL;
  PyObject *format = PyString_FromString
    ("<%s c=%g eps=%g scale=%s%d>");
  if (format==NULL) { Py_DECREF(args); return NULL; }
  PyObject *result = PyString_Format(format, args);
  Py_DECREF(args);
  Py_DECREF(format);
  return result;
}

/* GC support functions. */
static int Sparm_traverse(svms_SparmObject*self,visitproc visit,void*arg) {
  Py_VISIT((PyObject*)self->dict);
  return 0;
}
static int Sparm_clear(svms_SparmObject*self) {
  // Make sure we do not deallocate the next one.
  Py_CLEAR(self->dict);
  return 0;
}
static void Sparm_dealloc(svms_SparmObject *self) {
  self->ob_type->tp_free((PyObject *)self);
}

// Pickling support methods.

static PyObject *Sparm_setstate(svms_SparmObject *self, PyObject *state) {
  if (state == Py_None) Py_RETURN_NONE;
  if (!PyDict_Check(state)) {
    PyErr_SetString(PyExc_TypeError, "state is not a dict");
    return NULL;
  }
  self->sparm->pydict = self->dict = PyDict_Copy(state);
  Py_RETURN_NONE;
}

static PyObject *Sparm_reduce(svms_SparmObject *self) {
  STRUCT_LEARN_PARM* sp = self->sparm;
  return Py_BuildValue
    ("O(ddidNiii)O", self->ob_type, sp->epsilon, sp->newconstretrain,
     sp->ccache_size, sp->C, PyObject_GetAttrString((PyObject*)self, "argv"),
     sp->slack_norm, sp->loss_type, sp->loss_function, self->dict);
}

// Getter-setter methods.

static PyObject *Sparm_getepsilon(svms_SparmObject *self, void *closure) {
  return PyFloat_FromDouble(self->sparm->epsilon);
}
static int Sparm_setepsilon(svms_SparmObject *self, PyObject *value, void*c) {
  value = PyNumber_Float(value);
  if (value == NULL) return -1;
  double eps = PyFloat_AsDouble(value);
  Py_DECREF(value);
  if (eps < 0) {
    PyErr_SetString(PyExc_ValueError, "cannot set epsilon < 0!");
    return -1;
  }
  self->sparm->epsilon = eps;
  return 0;
}

static PyObject *Sparm_getncrt(svms_SparmObject *self, void *closure) {
  return PyInt_FromLong(self->sparm->newconstretrain);
}
static int Sparm_setncrt(svms_SparmObject *self, PyObject *value, void*c) {
  value = PyNumber_Int(value);
  if (value == NULL) return -1;
  int ncrt = PyInt_AsLong(value);
  Py_DECREF(value);
  if (ncrt < 1) {
    PyErr_SetString(PyExc_ValueError,
		    "constraint accumulation must be positive");
    return -1;
  }
  self->sparm->newconstretrain = ncrt;
  return 0;
}

static PyObject *Sparm_getccachesize(svms_SparmObject *self, void *closure) {
  return PyInt_FromLong(self->sparm->ccache_size);
}
static int Sparm_setccachesize(svms_SparmObject *self,PyObject *value,void*c) {
  value = PyNumber_Int(value);
  if (value == NULL) return -1;
  int cc = PyInt_AsLong(value);
  Py_DECREF(value);
  if (cc < 5) {
    PyErr_SetString(PyExc_ValueError,
		    "constraint cache size must be >= 5");
    return -1;
  }
  self->sparm->ccache_size = cc;
  return 0;
}

static PyObject *Sparm_getC(svms_SparmObject *self, void *closure) {
  return PyFloat_FromDouble(self->sparm->C);
}
static int Sparm_setC(svms_SparmObject *self, PyObject *value, void*c) {
  value = PyNumber_Float(value);
  if (value == NULL) return -1;
  double C = PyFloat_AsDouble(value);
  Py_DECREF(value);
  if (C < 0) {
    PyErr_SetString(PyExc_ValueError, "cannot set C < 0!");
    return -1;
  }
  self->sparm->C = C;
  return 0;
}

static PyObject *Sparm_getslacknorm(svms_SparmObject *self, void *closure) {
  return PyInt_FromLong(self->sparm->slack_norm);
}
static int Sparm_setslacknorm(svms_SparmObject *self,PyObject *value,void*c) {
  value = PyNumber_Int(value);
  if (value == NULL) return -1;
  int i = PyInt_AsLong(value);
  Py_DECREF(value);
  if (i != 1 && i != 2) {
    PyErr_SetString(PyExc_ValueError,
		    "slack norm must be either 1 or 2");
    return -1;
  }
  self->sparm->slack_norm = i;
  return 0;
}

static PyObject *Sparm_getlosstype(svms_SparmObject *self, void *closure) {
  return PyInt_FromLong(self->sparm->loss_type);
}
static int Sparm_setlosstype(svms_SparmObject *self,PyObject *value,void*c) {
  value = PyNumber_Int(value);
  if (value == NULL) return -1;
  int i = PyInt_AsLong(value);
  Py_DECREF(value);
  if (i != 1 && i != 2) {
    PyErr_SetString(PyExc_ValueError, "loss type must be 1 (slack rescaling) "
		    "or 2 (margin rescaling)");
    return -1;
  }
  self->sparm->loss_type = i;
  return 0;
}

static PyObject *Sparm_getlossfunction(svms_SparmObject *self, void *closure) {
  return PyInt_FromLong(self->sparm->loss_function);
}
static int Sparm_setlossfunction(svms_SparmObject*self,PyObject*value,void*c) {
  value = PyNumber_Int(value);
  if (value == NULL) return -1;
  int i = PyInt_AsLong(value);
  Py_DECREF(value);
  if (i < 0) {
    PyErr_SetString(PyExc_ValueError, "loss function id must be non-negative");
    return -1;
  }
  self->sparm->loss_function = i;
  return 0;
}

static PyObject *Sparm_getargv(svms_SparmObject *self, void *closure) {
  PyObject *args = PyList_New(self->sparm->custom_argc);
  int i;
  if (args == NULL) return NULL;
  for (i=0; i<self->sparm->custom_argc; ++i) {
    PyObject *s = PyString_FromString(self->sparm->custom_argv[i]);
    if (s==NULL) {
      Py_DECREF(args);
      return NULL;
    }
    PyList_SET_ITEM(args, i, s);
  }
  return args;
}

// Type initialization method.

int Sparm_InitType(PyObject *module) {
  if (PyType_Ready(&svms_SparmType) < 0) return -1;
  Py_INCREF(&svms_SparmType);
  PyModule_AddObject(module, "Sparm", (PyObject*)&svms_SparmType);

  return 0;
}

// Type definition for the sparse object, including methods.

static PyMethodDef svms_SparmMethods[] = {
  // Data setting and retrieval functions.
  {"__reduce__", (PyCFunction)Sparm_reduce, METH_NOARGS,
    "Pickling support."},
  {"__setstate__", (PyCFunction)Sparm_setstate, METH_O,
   "Unpickling support."},
  {NULL}
};

static PyMemberDef svms_SparmMembers[] = {
  //{"twonorm_sq", T_DOUBLE, offsetof(svms_SparmObject, (*svec).twonorm_sq)+offsetof(SVECTOR,twonorm_sq), READONLY},
  {"__dict__", T_OBJECT, offsetof(svms_SparmObject, dict), READONLY},
  {NULL}
};

static PyGetSetDef svms_SparmGetSetters[] = {
  {"epsilon",(getter)Sparm_getepsilon,(setter)Sparm_setepsilon,
   "Precision to which to solve the quadratic program.",NULL},
  {"newconstretrain",(getter)Sparm_getncrt,(setter)Sparm_setncrt,
   "Number of new constraints to accumulate before recomputing the QP.\n"
   "Used in w=1 algorithm.",NULL},
  {"ccache_size",(getter)Sparm_getccachesize,(setter)Sparm_setccachesize,
   "Maximum number of constraints to cache for each example.\n"
   "Used in w=4 algorithm.  Must be >= 5.",NULL},
  {"newconstretrain",(getter)Sparm_getncrt,(setter)Sparm_setncrt,
   "Number of new constraints to accumulate before recomputing the QP.\n"
   "Used in w=1 algorithm.",NULL},
  {"c",(getter)Sparm_getC,(setter)Sparm_setC,
   "Trade-off between margin and loss.",NULL},
  {"slack_norm",(getter)Sparm_getslacknorm,(setter)Sparm_setslacknorm,
   "Norm of the objective function slack term, either 1 or 2.",NULL},
  {"loss_type",(getter)Sparm_getlosstype,(setter)Sparm_setlosstype,
   "Constraint loss type, either slack (1) or margin (2) rescaling.",NULL},
  {"loss_function",(getter)Sparm_getlossfunction,(setter)Sparm_setlossfunction,
   "Identifier for different loss functions.",NULL},

  {"argv",(getter)Sparm_getargv,NULL,//(setter)Sparm_setargv,
   "List of custom command line arguments.",NULL},
  {NULL}
};

PyTypeObject svms_SparmType = {
  PyObject_HEAD_INIT(NULL)
  0,					/* ob_size */
  SVMAPINAME".Sparm",			/* tp_name */
  sizeof(svms_SparmObject),		/* tp_basicsize */
  0,					/* tp_itemsize */
  (destructor)Sparm_dealloc,		/* tp_dealloc */
  0,					/* tp_print */
  0,					/* tp_getattr */
  0,					/* tp_setattr */
  0,					/* tp_compare */
  (reprfunc)Sparm_Str,			/* tp_repr */
  0,					/* tp_as_number */
  0,					/* tp_as_sequence */
  0,					/* tp_as_mapping */
  0,					/* tp_hash */
  0,					/* tp_call */
  (reprfunc)Sparm_Str,			/* tp_str */
  0,					/* tp_getattro */
  0,					/* tp_setattro */
  0,					/* tp_as_buffer */
  Py_TPFLAGS_DEFAULT|Py_TPFLAGS_BASETYPE|Py_TPFLAGS_HAVE_GC,	/* tp_flags */
  "Struct learning parameter objects contrain attributes which control\n"
  "how the structural SVM will proceed in its cutting plane\n"
  "optimization.",
					/* tp_doc */
  (traverseproc)Sparm_traverse,		/* tp_traverse */
  (inquiry)Sparm_clear,			/* tp_clear */
  0,					/* tp_richcompare */
  0,					/* tp_weaklistoffset */
  0,					/* tp_iter */
  0,					/* tp_iternext */
  svms_SparmMethods,			/* tp_methods */
  svms_SparmMembers,			/* tp_members */
  svms_SparmGetSetters,			/* tp_getset */
  0,					/* tp_base */
  0,					/* tp_dict */
  0,					/* tp_descr_get */
  0,					/* tp_descr_set */
  offsetof(svms_SparmObject, dict),	/* tp_dictoffset */
  (initproc)Sparm_init,			/* tp_init */
  0,					/* tp_alloc */
  Sparm_new,				/* tp_new */
};
