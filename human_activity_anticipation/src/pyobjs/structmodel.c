#include "structmodel.h"
#include "array.h"
#include "model.h"
#include "structmember.h"
#include "svmapi_globals.h"
#include "../svm_struct_api.h"

/* SVM MODEL OBJECT. */

static PyObject *StructModel_new(PyTypeObject *type, PyObject *args,
			   PyObject *kwds) {
  svms_StructModelObject *self = NULL;
  self = (svms_StructModelObject*) type->tp_alloc(type, 0);
  if (self != NULL) {
    self->sm = NULL;
    self->dict = NULL;
    self->ifree = 0;
  }
  return (PyObject *)self;
}

svms_StructModelObject *StructModel_FromStructModel(STRUCTMODEL *sm) {
  svms_StructModelObject*so = (svms_StructModelObject*)
    PyObject_GC_New(svms_StructModelObject, &svms_StructModelType);
  so->sm = sm;
  so->ifree = 0;
  so->dict = (PyObject*)sm->pydict;
  Py_INCREF(so->dict);
  return so;
}

static int StructModel_init(svms_StructModelObject *self, PyObject *args,
			    PyObject *kwds) {
  svms_ModelObject *model;
  STRUCTMODEL *sm = (STRUCTMODEL*)my_malloc(sizeof(STRUCTMODEL));
  self->sm = sm;
  bzero(sm, sizeof(STRUCTMODEL));
  self->sm->lin_reduce = 1;

  if (!PyArg_ParseTuple(args, "O!i|b", &svms_ModelType, &model,
			&self->sm->sizePsi, &self->sm->lin_reduce))
    goto error;
  // Take possession of the model.
  model->ifree = 0;
  sm->svm_model = model->model;
  
  self->ifree = 1;
  return 0;

 error:
  free(sm);
  self->sm = NULL;
  return -1;
}

/*static PyObject *StructModel_Str(svms_StructModelObject *self) {
  PyObject *result = NULL;
  result = PyString_FromString("StructModely");
  return result;
  }*/

/* GC support functions. */
static int StructModel_traverse(svms_StructModelObject*self,
				visitproc visit,void*arg) {
  Py_VISIT((PyObject*)self->dict);
  return 0;
}
static int StructModel_clear(svms_StructModelObject*self) {
  // Make sure we do not deallocate the next one.
  Py_CLEAR(self->dict);
  return 0;
}
static void StructModel_dealloc(svms_StructModelObject *self) {
  if (self->ifree && self->sm) {
    free_struct_model(*self->sm);
    self->sm = NULL;
  }
  self->ob_type->tp_free((PyObject *)self);
}

// Getter-setter methods.

static PyObject *StructModel_getw(svms_StructModelObject *self,void *closure) {
  if (self->sm->w == NULL) {
    Py_RETURN_NONE;
  }
  return (PyObject*)Array_FromArray(self->sm->w, self->sm->sizePsi, 1,
				    AT_DOUBLE);
}

static PyObject *StructModel_getlinreduce(svms_StructModelObject*self,void *c){
  return PyBool_FromLong((long)self->sm->lin_reduce);
}
static int StructModel_setlinreduce(svms_StructModelObject*self,
				    PyObject*value,void*c){
  char lr = PyObject_IsTrue(value);
  if (lr==-1) return -1;
  self->sm->lin_reduce = lr;
  return 0;
}

static PyObject *StructModel_getsizepsi(svms_StructModelObject*self,void *c) {
  return PyInt_FromLong(self->sm->sizePsi);
}
static int StructModel_setsizepsi(svms_StructModelObject*self,
				  PyObject*value,void*c){
  value = PyNumber_Int(value);
  if (value == NULL) return -1;
  int i = PyInt_AsLong(value);
  Py_DECREF(value);
  if (i < 0) {
    PyErr_SetString(PyExc_ValueError, "size_psi cannot be negative");
    return -1;
  }
  self->sm->sizePsi = i;
  return 0;
}

static PyObject *StructModel_getmodel(svms_StructModelObject *self,
				      void *closure) {
  if (self->sm->svm_model == NULL) {
    Py_RETURN_NONE;
  }
  return (PyObject*)Model_FromModel(self->sm->svm_model);
}

// Type initialization method.

int StructModel_InitType(PyObject *module) {
  if (PyType_Ready(&svms_StructModelType) < 0) return -1;
  Py_INCREF(&svms_StructModelType);
  PyModule_AddObject(module, "StructModel", (PyObject*)&svms_StructModelType);
  return 0;
}

// Pickling support methods.

static PyObject *StructModel_setstate(svms_StructModelObject *self,
				      PyObject *state) {
  if (state == Py_None) Py_RETURN_NONE;
  if (!PyDict_Check(state)) {
    PyErr_SetString(PyExc_TypeError, "state is not a dict");
    return NULL;
  }
  Py_XDECREF(self->dict);
  self->sm->pydict = self->dict = PyDict_Copy(state);
  Py_RETURN_NONE;
}

static PyObject *StructModel_reduce(svms_StructModelObject *self) {
  STRUCTMODEL* sm = self->sm;
  PyObject*retval = Py_BuildValue
    ("O(Nib)O", self->ob_type, StructModel_getmodel(self,NULL),
     sm->sizePsi, sm->lin_reduce, self->dict);
  return retval;
}

// Type definition for the sparse object, including methods.

static PyMethodDef svms_StructModelMethods[] = {
  {"__reduce__", (PyCFunction)StructModel_reduce, METH_NOARGS,
   "Pickling support."},
  {"__setstate__", (PyCFunction)StructModel_setstate, METH_O,
   "Unpickling support."},
  {NULL}
};


static PyMemberDef svms_StructModelMembers[] = {
  {"__dict__", T_OBJECT, offsetof(svms_StructModelObject, dict), READONLY,
   "Dictionary of instance variables."},
  {NULL}
};

static PyGetSetDef svms_StructModelGetSetters[] = {
  {"size_psi",(getter)StructModel_getsizepsi,(setter)StructModel_setsizepsi,
    "Length of the linear feature vector.",NULL},
  {"w",(getter)StructModel_getw,(setter)NULL,
    "The linear learned weights.",NULL},
  {"lin_reduce",(getter)StructModel_getlinreduce,
   (setter)StructModel_setlinreduce,
   "Whether or not to reduce this model to a single support vector.\n"
   "If true, and there is an existing 'w' weight vector (which happens\n"
   "for linear kernels only), the support vectors in the SVM model\n"
   "will be replaced with a single support vector.  If false, no\n"
   "pre-serialization processing will occur.",
   NULL},
  {"svm_model",(getter)StructModel_getmodel,(setter)NULL,
    "The SVM model.",NULL},
  {NULL}
};

PyTypeObject svms_StructModelType = {
  PyObject_HEAD_INIT(NULL)
  0,					/* ob_size */
  SVMAPINAME".StructModel",			/* tp_name */
  sizeof(svms_StructModelObject),	/* tp_basicsize */
  0,					/* tp_itemsize */
  (destructor)StructModel_dealloc,	/* tp_dealloc */
  0,					/* tp_print */
  0,					/* tp_getattr */
  0,					/* tp_setattr */
  0,					/* tp_compare */
  0,//(reprfunc)StructModel_Str,	/* tp_repr */
  0,					/* tp_as_number */
  0,					/* tp_as_sequence */
  0,					/* tp_as_mapping */
  0,					/* tp_hash */
  0,					/* tp_call */
  0,//(reprfunc)StructModel_Str,	/* tp_str */
  0,					/* tp_getattro */
  0,					/* tp_setattro */
  0,					/* tp_as_buffer */
  Py_TPFLAGS_DEFAULT|Py_TPFLAGS_BASETYPE|Py_TPFLAGS_HAVE_GC,	/* tp_flags */
  "Parameterization for the learned structural model.\n\n"
  "Structure model objects which contain parameterizations for\n"
  "learned models.  Aside from basic members including a pointer\n"
  "to the underlying SVM QP model, a user may define their own\n"
  "values by assigning values to this object.  These assignments\n"
  "are retained throughout training, written with the model (if\n"
  "they are Picklable), and reloaded during classification using\n"
  "this model.",
					/* tp_doc */
  (traverseproc)StructModel_traverse,	/* tp_traverse */
  (inquiry)StructModel_clear,		/* tp_clear */
  0,					/* tp_richcompare */
  0,					/* tp_weaklistoffset */
  0,					/* tp_iter */
  0,					/* tp_iternext */
  svms_StructModelMethods,		/* tp_methods */
  svms_StructModelMembers,		/* tp_members */
  svms_StructModelGetSetters,		/* tp_getset */
  0,					/* tp_base */
  0,					/* tp_dict */
  0,					/* tp_descr_get */
  0,					/* tp_descr_set */
  offsetof(svms_StructModelObject,dict),/* tp_dictoffset */
  (initproc)StructModel_init,		/* tp_init */
  0,					/* tp_alloc */
  StructModel_new,			/* tp_new */
};
