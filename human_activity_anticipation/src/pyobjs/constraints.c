#include "constraints.h"
#include "document.h"
#include "structmember.h"
#include "svmapi_globals.h"

/* CONSTRAINTS ITERATOR OBJECT. */

static PyObject *svms_GetConstraint(CONSTSET cset, int i) {
  if (i<0) i+=cset.m;
  if (i>=cset.m) {
    PyErr_SetString(PyExc_IndexError, "cset index out of range");
    return NULL;
  }
  return Py_BuildValue("Nd", Document_FromDocument(cset.lhs[i]), cset.rhs[i]);
}

typedef struct {
  PyObject_HEAD
  long it_index;
  CONSTSET cset;
} svms_ConstraintsIterObject;

static PyObject *Constraints_iter(PyObject *cset) {
  svms_ConstraintsIterObject *it;
  if (!Constraints_Check(cset)) {
    PyErr_BadInternalCall();
    return NULL;
  }
  it = PyObject_New(svms_ConstraintsIterObject, &svms_ConstraintsIterType);
  if (it == NULL) return NULL;
  it->it_index = 0;
  it->cset = ((svms_ConstraintsObject *)cset)->cset;
  return (PyObject *)it;
}

static void ConstraintsIter_dealloc(svms_ConstraintsIterObject *it) {
  //Py_XDECREF(it->svec);
  PyObject_Del(it);
}

static int ConstraintsIter_len(svms_ConstraintsIterObject *it) {
  int len=0;
  len = it->cset.m - it->it_index;
  return len >= 0 ? len : 0;
}

static PyObject *ConstraintsIter_next(svms_ConstraintsIterObject *it) {
  if (it->it_index >= it->cset.m)
    return NULL;
  return svms_GetConstraint(it->cset, it->it_index++);
}

static PySequenceMethods csetiter_as_sequence = {
  (lenfunc)ConstraintsIter_len, /* sq_length */
  0, /* sq_concat */
};

PyTypeObject svms_ConstraintsIterType = {
  PyObject_HEAD_INIT(NULL)
  0,					/* ob_size */
  SVMAPINAME".ConstraintsIter",		/* tp_name */
  sizeof(svms_ConstraintsIterObject),	/* tp_basicsize */
  0,					/* tp_itemsize */
  (destructor)ConstraintsIter_dealloc,	/* tp_dealloc */
  0,					/* tp_print */
  0,					/* tp_getattr */
  0,					/* tp_setattr */
  0,					/* tp_compare */
  0,					/* tp_repr */
  0,					/* tp_as_number */
  &csetiter_as_sequence,		/* tp_as_sequence */
  0,					/* tp_as_mapping */
  0,					/* tp_hash */
  0,					/* tp_call */
  0,					/* tp_str */
  PyObject_GenericGetAttr,		/* tp_getattro */
  0,					/* tp_setattro */
  0,					/* tp_as_buffer */
  Py_TPFLAGS_DEFAULT,			/* tp_flags */
  "Constraints iterator objects, to access constraints.",	/* tp_doc */
  0,					/* tp_traverse */
  0,					/* tp_clear */
  0,					/* tp_richcompare */
  0,					/* tp_weaklistoffset */
  PyObject_SelfIter,			/* tp_iter */
  (iternextfunc)ConstraintsIter_next,	/* tp_iternext */
  0,					/* tp_methods */
  0,					/* tp_members */
  0,					/* tp_getset */
  0,					/* tp_base */
  0,					/* tp_dict */
  0,					/* tp_descr_get */
  0,					/* tp_descr_set */
  0,					/* tp_dictoffset */
};

/* CONSTRAINTS OBJECT. */

// Methods for the support vector list object.

static PyObject *Constraints_new(PyTypeObject *type, PyObject *args,
			   PyObject *kwds) {
  svms_ConstraintsObject *self = NULL;
  self = (svms_ConstraintsObject*) type->tp_alloc(type, 0);
  if (self != NULL) {
    self->cset.m = 0;
    self->cset.lhs = NULL;
    self->cset.rhs = NULL;
  }
  return (PyObject *)self;
}

svms_ConstraintsObject *Constraints_FromConstraints(CONSTSET cset) {
  svms_ConstraintsObject*so = (svms_ConstraintsObject*)
    PyObject_GC_New(svms_ConstraintsObject, &svms_ConstraintsType);
  so->cset = cset;
  return so;
}

/*static int Constraints_init(svms_ConstraintsObject *self, PyObject *args,
		      PyObject *kwds) {
  return 0;
  }*/

static PyObject *Constraints_Str(svms_ConstraintsObject *self) {
  PyObject *result = NULL;
  result = PyString_FromFormat
    ("<%s of length %d>",self->ob_type->tp_name, self->cset.m);
  return result;
}

/* GC support functions. */
static int Constraints_traverse(svms_ConstraintsObject*self,visitproc visit,void*arg) {
  return 0;
}
static int Constraints_clear(svms_ConstraintsObject*self) {
  // Make sure we do not deallocate the next one.
  return 0;
}
static void Constraints_dealloc(svms_ConstraintsObject *self) {
  self->ob_type->tp_free((PyObject *)self);
}

// Getter-setter methods.

// Type initialization method.

int Constraints_InitType(PyObject *module) {
  if (PyType_Ready(&svms_ConstraintsType) < 0) return -1;
  Py_INCREF(&svms_ConstraintsType);
  PyModule_AddObject(module, "Constraints", (PyObject*)&svms_ConstraintsType);

  if (PyType_Ready(&svms_ConstraintsIterType) < 0) return -1;
  Py_INCREF(&svms_ConstraintsIterType);
  PyModule_AddObject(module, "ConstraintsIter", (PyObject*)&svms_ConstraintsIterType);

  return 0;
}

// Sequence and mapping methods.

static int Constraints_Size(svms_ConstraintsObject *self) {
  return self->cset.m;
}

static PyObject* Constraints_subscript(svms_ConstraintsObject *self, PyObject *item) {
  if (PyInt_Check(item)) {
    return svms_GetConstraint(self->cset, PyInt_AsLong(item));
  } else if (PySlice_Check(item)) {
    Py_ssize_t start, stop, step, size, subsize, i;
    PyObject *sublist = NULL;
    size = self->cset.m;
    if (PySlice_GetIndicesEx((PySliceObject*)item,size,&start,&stop,&step,
			     &subsize) < 0) return NULL;
    sublist = PyList_New(subsize);
    if (sublist == NULL) return NULL;
    for (i=0; i<subsize; ++i) {
      PyObject *con = svms_GetConstraint(self->cset, i*step + start);
      if (con == NULL) { // Error getting the constraint.
	Py_DECREF(sublist);
	return NULL;
      }
      PyList_SET_ITEM(sublist, i, con);
    }
    return sublist;
  } else {
    PyErr_SetString(PyExc_TypeError, "bad index type for constraints");
    return NULL;
  }
}

// Type definition for the sparse object, including methods.

static PyMethodDef svms_ConstraintsMethods[] = {
  // Data setting and retrieval functions.
  {NULL}
};

static PyMemberDef svms_ConstraintsMembers[] = {
  {"m", T_INT, offsetof(svms_ConstraintsObject, cset)+offsetof(CONSTSET,m),
   READONLY, "The number of constraints in this example."},
  {NULL}
};

static PyGetSetDef svms_ConstraintsGetSetters[] = {
  {NULL}
};

static PySequenceMethods Constraints_as_sequence = {
  (lenfunc)Constraints_Size,		/* sq_length */
  0,					/* sq_concat */
  0,					/* sq_repeat */
  0,					/* sq_item */
  0,					/* sq_slice */
  0,					/* sq_ass_item */
  0,					/* sq_ass_slice */
  0,					/* sq_contains */
};

static PyMappingMethods Constraints_as_mapping = {
  (lenfunc)Constraints_Size,			/* mp_length */
  (binaryfunc)Constraints_subscript,		/* mp_subscript */
};

PyTypeObject svms_ConstraintsType = {
  PyObject_HEAD_INIT(NULL)
  0,					/* ob_size */
  SVMAPINAME".Constraints",			/* tp_name */
  sizeof(svms_ConstraintsObject),	/* tp_basicsize */
  0,					/* tp_itemsize */
  (destructor)Constraints_dealloc,	/* tp_dealloc */
  0,					/* tp_print */
  0,					/* tp_getattr */
  0,					/* tp_setattr */
  0,					/* tp_compare */
  (reprfunc)Constraints_Str,		/* tp_repr */
  0,					/* tp_as_number */
  &Constraints_as_sequence,		/* tp_as_sequence */
  &Constraints_as_mapping,		/* tp_as_mapping */
  0,					/* tp_hash */
  0,					/* tp_call */
  (reprfunc)Constraints_Str,		/* tp_str */
  0,					/* tp_getattro */
  0,					/* tp_setattro */
  0,					/* tp_as_buffer */
  Py_TPFLAGS_DEFAULT|Py_TPFLAGS_BASETYPE|Py_TPFLAGS_HAVE_GC,	/* tp_flags */
  "The constraint set of left-right side pairs.\n\n"
  "The idea of the constraint set is that the left-hand-side is a\n"
  "document and the right-hand-side is a float, where the inequality\n"
  "'lhs*w + slack >= rhs' is maintained.  This object acts roughly\n"
  "like a sequence with regard to constraint access.  The user does\n"
  "not create these objects.",
					/* tp_doc */
  (traverseproc)Constraints_traverse,	/* tp_traverse */
  (inquiry)Constraints_clear,		/* tp_clear */
  0,					/* tp_richcompare */
  0,					/* tp_weaklistoffset */
  Constraints_iter,			/* tp_iter */
  0,					/* tp_iternext */
  svms_ConstraintsMethods,		/* tp_methods */
  svms_ConstraintsMembers,		/* tp_members */
  svms_ConstraintsGetSetters,		/* tp_getset */
  0,					/* tp_base */
  0,					/* tp_dict */
  0,					/* tp_descr_get */
  0,					/* tp_descr_set */
  0,					/* tp_dictoffset */
  0,//(initproc)Constraints_init,	/* tp_init */
  0,					/* tp_alloc */
  Constraints_new,			/* tp_new */
};
