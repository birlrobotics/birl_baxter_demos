#include "kernelparm.h"
#include "../svm_light/svm_common.h"
#include "structmember.h"
#include "svmapi_globals.h"

/* KERNEL PARM OBJECT. */

static PyObject *KernelParm_new(PyTypeObject *type, PyObject *args,
			   PyObject *kwds) {
  svms_KernelParmObject *self = NULL;
  self = (svms_KernelParmObject*) type->tp_alloc(type, 0);
  if (self != NULL) {
    self->kparm = &self->int_kparm;
  }
  return (PyObject *)self;
}

svms_KernelParmObject *KernelParm_FromKernelParm(KERNEL_PARM *kp) {
  svms_KernelParmObject*so = (svms_KernelParmObject*)
    PyObject_GC_New(svms_KernelParmObject, &svms_KernelParmType);
  so->kparm = kp;
  return so;
}

KERNEL_PARM KernelParm_AsKernelParm(PyObject *kparm) {
  return *((svms_KernelParmObject*)kparm)->kparm;
}

static int KernelParm_init(svms_KernelParmObject *self, PyObject *args,
			   PyObject *kwds) {
  const char *buff;
  int bufflen;
  static char *kwlist[] = {"kernel_type","poly_degree","rbf_gamma",
			   "coef_lin","coef_const","custom",NULL};
  LEARN_PARM lp;
  KERNEL_PARM*k=self->kparm;
  set_learning_defaults(&lp, k);
  if (!PyArg_ParseTupleAndKeywords
      (args, kwds, "|iiddds#", kwlist, &k->kernel_type, &k->poly_degree,
       &k->rbf_gamma, &k->coef_lin, &k->coef_const, &buff, &bufflen))
    return -1;
  strncpy(k->custom, buff, sizeof(k->custom)-1);
  k->custom[sizeof(k->custom)-1]='\0';
  return 0;
}

static PyObject *KernelParm_Str(svms_KernelParmObject *self) {
  // Make the arguments for the formatting operation.
  KERNEL_PARM *k = self->kparm;
  PyObject *args = Py_BuildValue
    ("siiddds", self->ob_type->tp_name, k->kernel_type, k->poly_degree,
     k->rbf_gamma, k->coef_lin, k->coef_const, k->custom);
  if (args == NULL) return NULL;
  // Make the format for the formatting operation.
  PyObject *format = PyString_FromString
    ("<%s kernel_type=%d poly_degree=%d rbf_gamma=%g coef_lin=%g coef_const=%g custom=%s>");
  if (format==NULL) {
    Py_DECREF(args);
    return NULL;
  }
  // Make the formatted string.
  PyObject *result = PyString_Format(format, args);
  Py_DECREF(args);
  Py_DECREF(format);
  
  return result;
}

static PyObject *KernelParm_reduce(svms_KernelParmObject *self) {
  KERNEL_PARM*k=self->kparm;
  return Py_BuildValue
    ("(O(iifffs))", self->ob_type, k->kernel_type, k->poly_degree,
     k->rbf_gamma, k->coef_lin, k->coef_const, k->custom);
}

/* GC support functions. */
static int KernelParm_traverse(svms_KernelParmObject*self,
				visitproc visit,void*arg) {
  return 0;
}
static int KernelParm_clear(svms_KernelParmObject*self) {
  // Make sure we do not deallocate the next one.
  return 0;
}
static void KernelParm_dealloc(svms_KernelParmObject *self) {
  self->ob_type->tp_free((PyObject *)self);
}

// Getter-setter methods.

static PyObject *KernelParm_getkerneltype(svms_KernelParmObject *self,
					  void *closure) {
  return PyInt_FromLong(self->kparm->kernel_type);
}

static int KernelParm_setkerneltype(svms_KernelParmObject *self,
				    PyObject *value, void*closure) {
  int v = PyInt_AsLong(value);
  if (PyErr_Occurred()) return -1;
  if (v < 0 || v > 5) {
    PyErr_SetString(PyExc_ValueError, "kernel_type must int from 0 to 5");
    return -1;
  }
  self->kparm->kernel_type = v;
  return 0;
}

static PyObject *KernelParm_getpolydegree(svms_KernelParmObject *self,
					  void *closure) {
  return PyInt_FromLong(self->kparm->poly_degree);
}

static int KernelParm_setpolydegree(svms_KernelParmObject *self,
				    PyObject *value, void*closure) {
  int v = PyInt_AsLong(value);
  if (PyErr_Occurred()) return -1;
  if (v < 1) {
    PyErr_SetString(PyExc_ValueError, "poly_degree must be positive");
    return -1;
  }
  self->kparm->poly_degree = v;
  return 0;
}

static PyObject *KernelParm_getrbfgamma(svms_KernelParmObject *self,
					void *closure) {
  return PyFloat_FromDouble(self->kparm->rbf_gamma);
}

static int KernelParm_setrbfgamma(svms_KernelParmObject *self,
				    PyObject *value, void*closure) {
  double v = PyFloat_AsDouble(value);
  if (PyErr_Occurred()) return -1;
  if (v <= 0.0) {
    PyErr_SetString(PyExc_ValueError, "rbf_gamma must be positive");
    return -1;
  }
  self->kparm->rbf_gamma = v;
  return 0;
}

static PyObject *KernelParm_getcoeflin(svms_KernelParmObject *self,
					void *closure) {
  return PyFloat_FromDouble(self->kparm->coef_lin);
}

static int KernelParm_setcoeflin(svms_KernelParmObject *self,
				    PyObject *value, void*closure) {
  double v = PyFloat_AsDouble(value);
  if (PyErr_Occurred()) return -1;
  if (v <= 0.0) {
    PyErr_SetString(PyExc_ValueError,
		    "sigmoid linear coefficient must be positive");
    return -1;
  }
  self->kparm->coef_lin = v;
  return 0;
}

static PyObject *KernelParm_getcoefconst(svms_KernelParmObject *self,
					void *closure) {
  return PyFloat_FromDouble(self->kparm->coef_const);
}

static int KernelParm_setcoefconst(svms_KernelParmObject *self,
				    PyObject *value, void*closure) {
  double v = PyFloat_AsDouble(value);
  if (PyErr_Occurred()) return -1;
  if (v <= 0.0) {
    PyErr_SetString(PyExc_ValueError,
		    "sigmoid constant coefficient must be positive");
    return -1;
  }
  self->kparm->coef_const = v;
  return 0;
}

static PyObject *KernelParm_getcustom(svms_KernelParmObject *self,
				      void *closure) {
  return PyString_FromString(self->kparm->custom);
}

static int KernelParm_setcustom(svms_KernelParmObject *self,
				PyObject *value, void*closure) {
  if (value == NULL) {
    self->kparm->custom[0] = 0;
    return 0;
  }

  char *cust = PyString_AsString(value);
  if (cust==NULL) return -1;
  strncpy(self->kparm->custom, cust, sizeof(self->kparm->custom)-1);
  self->kparm->custom[sizeof(self->kparm->custom)-1] = 0;
  return 0;
}

// Type initialization method.

int KernelParm_InitType(PyObject *module) {
  if (PyType_Ready(&svms_KernelParmType) < 0) return -1;
  Py_INCREF(&svms_KernelParmType);
  PyModule_AddObject(module, "KernelParm", (PyObject*)&svms_KernelParmType);

  return 0;
}

// Type definition for the kernel parameter object, including methods.

static PyMethodDef svms_KernelParmMethods[] = {
  // Data setting and retrieval functions.
  {"__reduce__", (PyCFunction)KernelParm_reduce, METH_NOARGS,
   "Pickling support."},
  {NULL}
};

static PyMemberDef svms_KernelParmMembers[] = {
  //{"twonorm_sq", T_DOUBLE, offsetof(svms_KernelParmObject, (*svec).twonorm_sq)+offsetof(SVECTOR,twonorm_sq), READONLY},
  {NULL}
};

static PyGetSetDef svms_KernelParmGetSetters[] = {
  {"kernel_type",(getter)KernelParm_getkerneltype,
   (setter)KernelParm_setkerneltype,
   "The type of the kernel.  Value values are:\n"
   "  0 - linear\n  1 - polynomial\n  2 - rbf\n  3 - sigmoid\n"
   "  4 - custom\n  5 - matrix", NULL},
  {"poly_degree",(getter)KernelParm_getpolydegree,
   (setter)KernelParm_setpolydegree,
   "The degree of the polynomial kernel.",NULL},
  {"rbf_gamma",(getter)KernelParm_getrbfgamma,
   (setter)KernelParm_setrbfgamma,
   "Gaussian width parameter for RBF kernel.",NULL},
  {"coef_lin",(getter)KernelParm_getcoeflin,
   (setter)KernelParm_setcoeflin,
   "Sigmoid linear coefficient 's' in tanh(s a*b + c).",NULL},
  {"coef_const",(getter)KernelParm_getcoefconst,
   (setter)KernelParm_setcoefconst,
   "Sigmoid constant coefficient 'c' in tanh(s a*b + c).",NULL},
  {"custom",(getter)KernelParm_getcustom,
   (setter)KernelParm_setcustom,
   "String for benefit of the custom kernel.",NULL},
  {NULL}
};

PyTypeObject svms_KernelParmType = {
  PyObject_HEAD_INIT(NULL)
  0,					/* ob_size */
  SVMAPINAME".KernelParm",			/* tp_name */
  sizeof(svms_KernelParmObject),	/* tp_basicsize */
  0,					/* tp_itemsize */
  (destructor)KernelParm_dealloc,	/* tp_dealloc */
  0,					/* tp_print */
  0,					/* tp_getattr */
  0,					/* tp_setattr */
  0,					/* tp_compare */
  (reprfunc)KernelParm_Str,		/* tp_repr */
  0,					/* tp_as_number */
  0,					/* tp_as_sequence */
  0,					/* tp_as_mapping */
  0,					/* tp_hash */
  0,					/* tp_call */
  (reprfunc)KernelParm_Str,		/* tp_str */
  0,					/* tp_getattro */
  0,					/* tp_setattro */
  0,					/* tp_as_buffer */
  Py_TPFLAGS_DEFAULT|Py_TPFLAGS_BASETYPE|Py_TPFLAGS_HAVE_GC,	/* tp_flags */
  "Kernel parameter objects.\n\n"
  "These encapulate a parameterization of the kernel used in learning.",
					/* tp_doc */
  (traverseproc)KernelParm_traverse,	/* tp_traverse */
  (inquiry)KernelParm_clear,		/* tp_clear */
  0,					/* tp_richcompare */
  0,					/* tp_weaklistoffset */
  0,					/* tp_iter */
  0,					/* tp_iternext */
  svms_KernelParmMethods,		/* tp_methods */
  svms_KernelParmMembers,		/* tp_members */
  svms_KernelParmGetSetters,		/* tp_getset */
  0,					/* tp_base */
  0,					/* tp_dict */
  0,					/* tp_descr_get */
  0,					/* tp_descr_set */
  0,					/* tp_dictoffset */
  (initproc)KernelParm_init,		/* tp_init */
  0,					/* tp_alloc */
  KernelParm_new,			/* tp_new */
};
