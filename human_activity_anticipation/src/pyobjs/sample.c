#include "sample.h"
#include "structmember.h"
#include "svmapi_globals.h"

/* SAMPLE ITERATOR OBJECT. */

static PyObject *svms_GetExample(SAMPLE sample, int i) {
  if (i<0) i+=sample.n;
  if (i>=sample.n) {
    PyErr_SetString(PyExc_IndexError, "sample index out of range");
    return NULL;
  }
  EXAMPLE e = sample.examples[i];
  return Py_BuildValue("OO", e.x.py_x, e.y.py_y);
}

typedef struct {
  PyObject_HEAD
  long it_index;
  SAMPLE sample;
} svms_SampleIterObject;

static PyObject *Sample_iter(PyObject *sample) {
  svms_SampleIterObject *it;
  if (!Sample_Check(sample)) {
    PyErr_BadInternalCall();
    return NULL;
  }
  it = PyObject_New(svms_SampleIterObject, &svms_SampleIterType);
  if (it == NULL) return NULL;
  it->it_index = 0;
  it->sample = ((svms_SampleObject *)sample)->sample;
  return (PyObject *)it;
}

static void SampleIter_dealloc(svms_SampleIterObject *it) {
  //Py_XDECREF(it->svec);
  PyObject_Del(it);
}

static int SampleIter_len(svms_SampleIterObject *it) {
  int len=0;
  len = it->sample.n - it->it_index;
  return len >= 0 ? len : 0;
}

static PyObject *SampleIter_next(svms_SampleIterObject *it) {
  if (it->it_index >= it->sample.n)
    return NULL;
  return svms_GetExample(it->sample, it->it_index++);
}

static PySequenceMethods sampleiter_as_sequence = {
  (lenfunc)SampleIter_len, /* sq_length */
  0, /* sq_concat */
};

PyTypeObject svms_SampleIterType = {
  PyObject_HEAD_INIT(NULL)
  0,					/* ob_size */
  SVMAPINAME".SampleIter",			/* tp_name */
  sizeof(svms_SampleIterObject),	/* tp_basicsize */
  0,					/* tp_itemsize */
  (destructor)SampleIter_dealloc,	/* tp_dealloc */
  0,					/* tp_print */
  0,					/* tp_getattr */
  0,					/* tp_setattr */
  0,					/* tp_compare */
  0,					/* tp_repr */
  0,					/* tp_as_number */
  &sampleiter_as_sequence,		/* tp_as_sequence */
  0,					/* tp_as_mapping */
  0,					/* tp_hash */
  0,					/* tp_call */
  0,					/* tp_str */
  PyObject_GenericGetAttr,		/* tp_getattro */
  0,					/* tp_setattro */
  0,					/* tp_as_buffer */
  Py_TPFLAGS_DEFAULT,			/* tp_flags */
  "Sample iterator objects",		/* tp_doc */
  0,					/* tp_traverse */
  0,					/* tp_clear */
  0,					/* tp_richcompare */
  0,					/* tp_weaklistoffset */
  PyObject_SelfIter,			/* tp_iter */
  (iternextfunc)SampleIter_next,	/* tp_iternext */
  0,					/* tp_methods */
  0,					/* tp_members */
  0,					/* tp_getset */
  0,					/* tp_base */
  0,					/* tp_dict */
  0,					/* tp_descr_get */
  0,					/* tp_descr_set */
  0,					/* tp_dictoffset */
};

/* SAMPLE OBJECT. */

// Methods for the support vector list object.

static PyObject *Sample_new(PyTypeObject *type, PyObject *args,
			   PyObject *kwds) {
  svms_SampleObject *self = NULL;
  self = (svms_SampleObject*) type->tp_alloc(type, 0);
  if (self != NULL) {
    self->sample.n = 0;
    self->sample.examples = NULL;
  }
  return (PyObject *)self;
}

svms_SampleObject *Sample_FromSample(SAMPLE sample) {
  svms_SampleObject*so = (svms_SampleObject*)
    PyObject_GC_New(svms_SampleObject, &svms_SampleType);
  so->sample = sample;
  return so;
}

/*static int Sample_init(svms_SampleObject *self, PyObject *args,
		      PyObject *kwds) {
  return 0;
  }*/

static PyObject *Sample_Str(svms_SampleObject *self) {
  PyObject *result = NULL;
  result = PyString_FromFormat
    ("<%s of length %d>",self->ob_type->tp_name,self->sample.n);
  return result;
}

/* GC support functions. */
static int Sample_traverse(svms_SampleObject*self,visitproc visit,void*arg) {
  //Py_VISIT((PyObject*)self->dict);
  return 0;
}
static int Sample_clear(svms_SampleObject*self) {
  // Make sure we do not deallocate the next one.
  //Py_CLEAR(self->dict);
  return 0;
}
static void Sample_dealloc(svms_SampleObject *self) {
  self->ob_type->tp_free((PyObject *)self);
}

// Getter-setter methods.

// Type initialization method.

int Sample_InitType(PyObject *module) {
  if (PyType_Ready(&svms_SampleType) < 0) return -1;
  Py_INCREF(&svms_SampleType);
  PyModule_AddObject(module, "Sample", (PyObject*)&svms_SampleType);

  if (PyType_Ready(&svms_SampleIterType) < 0) return -1;
  Py_INCREF(&svms_SampleIterType);
  PyModule_AddObject(module, "SampleIter", (PyObject*)&svms_SampleIterType);

  return 0;
}

// Sequence and mapping methods.

static int Sample_Size(svms_SampleObject *self) {
  return self->sample.n;
}

static PyObject* Sample_subscript(svms_SampleObject *self, PyObject *item) {
  if (PyInt_Check(item)) {
    return svms_GetExample(self->sample, PyInt_AsLong(item));
  } else if (PySlice_Check(item)) {
    Py_ssize_t start, stop, step, size, subsize, i;
    PyObject *sublist = NULL;
    size = self->sample.n;
    if (PySlice_GetIndicesEx((PySliceObject*)item,size,&start,&stop,&step,
			     &subsize) < 0) return NULL;
    sublist = PyList_New(subsize);
    if (sublist == NULL) return NULL;
    for (i=0; i<subsize; ++i) {
      PyObject *ex = svms_GetExample(self->sample, i*step + start);
      if (ex == NULL) { // Error getting the example...
	Py_DECREF(sublist);
	return NULL;
      }
      PyList_SET_ITEM(sublist, i, ex);
    }
    return sublist;
  } else {
    PyErr_SetString(PyExc_TypeError, "bad index type for sample");
    return NULL;
  }
}

// Type definition for the sparse object, including methods.

static PyMethodDef svms_SampleMethods[] = {
  // Data setting and retrieval functions.
  {NULL}
};

static PyMemberDef svms_SampleMembers[] = {
  //{"twonorm_sq", T_DOUBLE, offsetof(svms_SampleObject, (*svec).twonorm_sq)+offsetof(SVECTOR,twonorm_sq), READONLY},
  {"n", T_INT, offsetof(svms_SampleObject, sample)+offsetof(SAMPLE,n),
   READONLY, "The number of example in the sample."},
  {NULL}
};

static PyGetSetDef svms_SampleGetSetters[] = {
  /*{"epsilon",(getter)Sample_getepsilon,(setter)Sample_setepsilon,
    "Precision to which to solve the quadratic program.",NULL},*/
  {NULL}
};

static PySequenceMethods Sample_as_sequence = {
  (lenfunc)Sample_Size,			/* sq_length */
  0,					/* sq_concat */
  0,					/* sq_repeat */
  0,					/* sq_item */
  0,					/* sq_slice */
  0,					/* sq_ass_item */
  0,					/* sq_ass_slice */
  0,					/* sq_contains */
};

static PyMappingMethods Sample_as_mapping = {
  (lenfunc)Sample_Size,			/* mp_length */
  (binaryfunc)Sample_subscript,		/* mp_subscript */
  (objobjargproc)0,			/* mp_ass_subscript */
};

PyTypeObject svms_SampleType = {
  PyObject_HEAD_INIT(NULL)
  0,					/* ob_size */
  SVMAPINAME".Sample",			/* tp_name */
  sizeof(svms_SampleObject),		/* tp_basicsize */
  0,					/* tp_itemsize */
  (destructor)Sample_dealloc,		/* tp_dealloc */
  0,					/* tp_print */
  0,					/* tp_getattr */
  0,					/* tp_setattr */
  0,					/* tp_compare */
  (reprfunc)Sample_Str,			/* tp_repr */
  0,					/* tp_as_number */
  &Sample_as_sequence,			/* tp_as_sequence */
  &Sample_as_mapping,			/* tp_as_mapping */
  0,					/* tp_hash */
  0,					/* tp_call */
  (reprfunc)Sample_Str,			/* tp_str */
  0,					/* tp_getattro */
  0,					/* tp_setattro */
  0,					/* tp_as_buffer */
  Py_TPFLAGS_DEFAULT|Py_TPFLAGS_BASETYPE|Py_TPFLAGS_HAVE_GC,	/* tp_flags */
  "The sample of input-output pairs we either learn or classify.\n\n"
  "This object is used by SVM^python to provide user code with the\n"
  "entire training or classification set.",
					/* tp_doc */
  (traverseproc)Sample_traverse,	/* tp_traverse */
  (inquiry)Sample_clear,		/* tp_clear */
  0,					/* tp_richcompare */
  0,					/* tp_weaklistoffset */
  Sample_iter,				/* tp_iter */
  0,					/* tp_iternext */
  svms_SampleMethods,			/* tp_methods */
  svms_SampleMembers,			/* tp_members */
  svms_SampleGetSetters,		/* tp_getset */
  0,					/* tp_base */
  0,					/* tp_dict */
  0,					/* tp_descr_get */
  0,					/* tp_descr_set */
  0,					/* tp_dictoffset */
  0,//(initproc)Sample_init,		/* tp_init */
  0,					/* tp_alloc */
  Sample_new,				/* tp_new */
};
