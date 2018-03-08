#include "model.h"
#include "array.h"
#include "kernelparm.h"
#include "document.h"
#include "sparse.h"
#include "structmember.h"
#include "svmapi_globals.h"

/* STRUCT MODEL OBJECT. */

// Methods for the support vector list object.

static PyObject *Model_new(PyTypeObject *type, PyObject *args,
			   PyObject *kwds) {
  svms_ModelObject *self = NULL;
  self = (svms_ModelObject*) type->tp_alloc(type, 0);
  if (self != NULL) {
    self->ifree = 0;
    self->model = NULL;
  }
  return (PyObject *)self;
}

svms_ModelObject *Model_FromModel(MODEL *model) {
  svms_ModelObject*so = (svms_ModelObject*)
    PyObject_GC_New(svms_ModelObject, &svms_ModelType);
  so->ifree = 0;
   so->model = model;
  return so;
}

static int Model_init(svms_ModelObject *self, PyObject *args,
		      PyObject *kwds) {
  svms_ArrayObject *alphas, *supvecs;
  svms_KernelParmObject *kp;
  MODEL*m = self->model = (MODEL*)my_malloc(sizeof(MODEL));
  bzero(m, sizeof(MODEL));

  if (!PyArg_ParseTuple
      (args, "O!iiidO!O!", &svms_KernelParmType, &kp, &m->totwords, &m->totdoc,
       &m->at_upper_bound, &m->b, &svms_ArrayType, &alphas, &svms_ArrayType,
       &supvecs)) {
    goto error;
  }

  if (alphas->type != AT_DOUBLE) {
    PyErr_SetString(PyExc_TypeError, "array must be of type double");
    goto error;
  }
  if (supvecs->type != AT_DOC) {
    PyErr_SetString(PyExc_TypeError, "array must be of type document");
    goto error;
  }
  if (alphas->length != supvecs->length) {
    PyErr_SetString(PyExc_ValueError,
		    "alpha and supvec arrays must have same length");
    goto error;
  }
  // Get the alpha and support vector array structures.
  m->sv_num = alphas->length + 1;
  alphas->ifree = 0; // Take custody of the alpha array.
  m->alpha = (double*)alphas->array;
  supvecs->ifree = 0; // Take custody of the document supvec array.
  m->supvec = (DOC**)supvecs->array;
  self->ifree = 1;
  // Get the kernel parameters.
  m->kernel_parm = KernelParm_AsKernelParm((PyObject*)kp);
  return 0;

 error:
  free(self->model);
  self->model = NULL;
  return -1;
}

/*static PyObject *Model_Str(svms_ModelObject *self) {
  PyObject *result = NULL;
  result = PyString_FromString("Modely");
  return result;
  }*/

static PyObject *Model_classify(svms_ModelObject *self, PyObject *arg) {
  if (Document_Check(arg)) {
    DOC *d = ((svms_DocumentObject*)arg)->doc;
    return PyFloat_FromDouble(classify_example(self->model, d));
  } else if (Sparse_Check(arg)) {
    DOC d;
    d.docnum = d.queryid = d.slackid = d.kernelid = 0;
    d.costfactor = 1.0;
    d.fvec = ((svms_SparseObject*)arg)->sparse;
    return PyFloat_FromDouble(classify_example(self->model, &d));
  } else {
    PyErr_SetString(PyExc_TypeError, "type unrecognized");
    return NULL;
  }
}

/* GC support functions. */
static int Model_traverse(svms_ModelObject*self,
				visitproc visit,void*arg) {
  //Py_VISIT((PyObject*)self->dict);
  return 0;
}
static int Model_clear(svms_ModelObject*self) {
  // Make sure we do not deallocate the next one.
  //Py_CLEAR(self->dict);
  return 0;
}
static void Model_dealloc(svms_ModelObject *self) {
  if (self->ifree && self->model) {
    free_model(self->model,1);
    self->model = NULL;
  }
  self->ob_type->tp_free((PyObject *)self);
}

// Getter-setter methods.

static PyObject *Model_getatupperbound(svms_ModelObject *self,void *closure) {
  return PyInt_FromLong(self->model->at_upper_bound);
}
static PyObject *Model_getb(svms_ModelObject *self,void *closure) {
  return PyFloat_FromDouble(self->model->b);
}
static PyObject *Model_gettotwords(svms_ModelObject *self,void *closure) {
  return PyInt_FromLong(self->model->totwords);
}
static PyObject *Model_gettotdoc(svms_ModelObject *self,void *closure) {
  return PyInt_FromLong(self->model->totdoc);
}
static PyObject *Model_getsvnum(svms_ModelObject *self,void *closure) {
  return PyInt_FromLong(self->model->sv_num-1);
}
static PyObject *Model_getsupvec(svms_ModelObject *self,void *closure) {
  return (PyObject*)Array_FromArray
    (self->model->supvec, self->model->sv_num-1, 1, AT_DOC);
}
static PyObject *Model_getalpha(svms_ModelObject *self,void *closure) {
  return (PyObject*)Array_FromArray
    (self->model->alpha, self->model->sv_num-1, 1, AT_DOUBLE);
}
static PyObject *Model_getkernelparm(svms_ModelObject *self,void *closure) {
  return (PyObject*)KernelParm_FromKernelParm(&self->model->kernel_parm);
}

// Type initialization method.

int Model_InitType(PyObject *module) {
  if (PyType_Ready(&svms_ModelType) < 0) return -1;
  Py_INCREF(&svms_ModelType);
  PyModule_AddObject(module, "Model", (PyObject*)&svms_ModelType);

  return 0;
}

static PyObject *Model_reduce(svms_ModelObject *self) {
  //return Py_BuildValue("O(iN)", self->ob_type, self->type,
  //PySequence_Tuple((PyObject*)self));
  MODEL *m = self->model;
  return Py_BuildValue("O(NiiifNN)", self->ob_type,
		       Model_getkernelparm(self,NULL), m->totwords, m->totdoc,
		       m->at_upper_bound, m->b, Model_getalpha(self,NULL),
		       Model_getsupvec(self,NULL));
}

// Type definition for the sparse object, including methods.

static PyMethodDef svms_ModelMethods[] = {
  // Data setting and retrieval functions.
  {"classify",(PyCFunction)Model_classify,METH_O,
   "classify(example)\n\n"
   "Given an example, which can be either a svmapi.Document or a\n"
   SVMAPINAME".Sparse object, return the scalar classification of that\n"
   "document according to this model."},
  {"__reduce__", (PyCFunction)Model_reduce, METH_NOARGS,
   "Pickling support."},
  {NULL,NULL,0,NULL}
};

static PyMemberDef svms_ModelMembers[] = {
  //{"twonorm_sq", T_DOUBLE, offsetof(svms_ModelObject, (*svec).twonorm_sq)+offsetof(SVECTOR,twonorm_sq), READONLY},
  {NULL}
};

static PyGetSetDef svms_ModelGetSetters[] = {
  {"at_upper_bound",(getter)Model_getatupperbound,(setter)NULL,
   "The number of support vectors with maxed alpha.",NULL},
  {"b",(getter)Model_getb,(setter)NULL,
   "The bias term.",NULL},
  {"totwords",(getter)Model_gettotwords,(setter)NULL,
   "The number of features.",NULL},
  {"totdoc",(getter)Model_gettotdoc,(setter)NULL,
   "The number of training documents.",NULL},
  {"sv_num",(getter)Model_getsvnum,(setter)NULL,
   "The number of support vectors.",NULL},
  {"supvec",(getter)Model_getsupvec,(setter)NULL,
   "The support vectors.",NULL},
  {"alpha",(getter)Model_getalpha,(setter)NULL,
   "The alpha multipliers for each support vector.",NULL},
  {"kernel_parm",(getter)Model_getkernelparm,(setter)NULL,
   "Kernel parameters.",NULL},
  /*{"index",(getter)Model_getindex,(setter)NULL,
    "The index from document number to position in the model.",NULL},*/

  {NULL}
};

PyTypeObject svms_ModelType = {
  PyObject_HEAD_INIT(NULL)
  0,					/* ob_size */
  SVMAPINAME".Model",			/* tp_name */
  sizeof(svms_ModelObject),		/* tp_basicsize */
  0,					/* tp_itemsize */
  (destructor)Model_dealloc,		/* tp_dealloc */
  0,					/* tp_print */
  0,					/* tp_getattr */
  0,					/* tp_setattr */
  0,					/* tp_compare */
  0,//(reprfunc)Model_Str,		/* tp_repr */
  0,					/* tp_as_number */
  0,					/* tp_as_sequence */
  0,					/* tp_as_mapping */
  0,					/* tp_hash */
  0,					/* tp_call */
  0,//(reprfunc)Model_Str,		/* tp_str */
  0,					/* tp_getattro */
  0,					/* tp_setattro */
  0,					/* tp_as_buffer */
  Py_TPFLAGS_DEFAULT|Py_TPFLAGS_BASETYPE|Py_TPFLAGS_HAVE_GC,	/* tp_flags */
  "Contains general SVM model parameters.\n\n"
  "These objects contain general parameters related to the SVM QP,\n"
  "but unrelated to any structural-learning specific data.",
					/* tp_doc */
  (traverseproc)Model_traverse,		/* tp_traverse */
  (inquiry)Model_clear,			/* tp_clear */
  0,					/* tp_richcompare */
  0,					/* tp_weaklistoffset */
  0,					/* tp_iter */
  0,					/* tp_iternext */
  svms_ModelMethods,			/* tp_methods */
  svms_ModelMembers,			/* tp_members */
  svms_ModelGetSetters,			/* tp_getset */
  0,					/* tp_base */
  0,					/* tp_dict */
  0,					/* tp_descr_get */
  0,					/* tp_descr_set */
  0,					/* tp_dictoffset */
  (initproc)Model_init,			/* tp_init */
  0,					/* tp_alloc */
  Model_new,				/* tp_new */
};
