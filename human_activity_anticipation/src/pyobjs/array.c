#include "array.h"
#include "document.h"
#include "structmember.h"
#include "svmapi_globals.h"

/* ARRAY ITERATOR OBJECT. */

static PyObject *svms_GetItem(svms_ArrayObject *array, int i) {
  if (i<0) i+=array->length;
  if (i>=array->length) {
    PyErr_SetString(PyExc_IndexError, "array index out of range");
    return NULL;
  }
  i += array->offset;
  switch (array->type) {
  case AT_DOUBLE:
    return PyFloat_FromDouble(((double*)array->array)[i]);
  case AT_INT:
    return PyInt_FromLong(((int*)array->array)[i]);
  case AT_DOC:
    return (PyObject*)Document_FromDocument(((DOC**)array->array)[i]);
  default:
    PyErr_BadInternalCall();
    return NULL;
  }
}

// Note that this function steals a reference.
static int svms_SetItem(svms_ArrayObject *array, PyObject*item,int i) {
  if (i<0) i+=array->length;
  if (i>=array->length) {
    PyErr_SetString(PyExc_IndexError, "array index out of range");
    return -1;
  }
  i += array->offset;
  switch (array->type) {
  case AT_DOUBLE: {
    double v = PyFloat_AsDouble(item);
    Py_DECREF(item);
    if (PyErr_Occurred()) return -1;
    ((double*)array->array)[i] = v;
    return 0;
  }
  case AT_INT: {
    int v = PyInt_AsLong(item);
    Py_DECREF(item);
    if (PyErr_Occurred()) return -1;
    ((int*)array->array)[i] = v;
    return 0;
  }
  case AT_DOC: {
    if (!Document_Check(item)) {
      PyErr_Format(PyExc_TypeError, "'%s' type expected",
		   svms_DocumentType.tp_name);
      return -1;
    }
    svms_DocumentObject *doc_ob = (svms_DocumentObject*)item;
    if (!array->ifree) {
      // Can't currently assign pointer structures if we don't own the memory.
      PyErr_BadInternalCall();
      return -1;
    }
    if (array->ifree && doc_ob->ob_refcnt==1 && doc_ob->ifree) {
      // It is responsible for itself, on its last refcnt, and we are
      // responsible for our own memory.
      doc_ob->ifree = 0;
      ((DOC**)array->array)[i] = doc_ob->doc;
    } else {
      ((DOC**)array->array)[i] = Document_AsDocument(item);
    }
    Py_DECREF(item);
    return 0;
  }
  default:
    PyErr_BadInternalCall();
    return -1;
  }
}

static void svms_DeallocItem(svms_ArrayObject *array, int i) {
  i+=array->offset;
  switch (array->type) {
  case AT_DOC: {
    DOC *d = ((DOC**)array->array)[i];
    if (d==NULL) break;
    free_example(d, 1);
    ((DOC**)array->array)[i] = NULL;
    break;
  }
  default:
    break; // Do nothing.
  }
}

typedef struct {
  PyObject_HEAD
  long it_index;
  svms_ArrayObject *array;
} svms_ArrayIterObject;

static PyObject *Array_iter(PyObject *array) {
  svms_ArrayIterObject *it;
  if (!Array_Check(array)) {
    PyErr_BadInternalCall();
    return NULL;
  }
  it = PyObject_New(svms_ArrayIterObject, &svms_ArrayIterType);
  if (it == NULL) return NULL;
  it->it_index = 0;
  Py_INCREF(array);
  it->array = (svms_ArrayObject *)array;
  return (PyObject *)it;
}

static void ArrayIter_dealloc(svms_ArrayIterObject *it) {
  Py_XDECREF(it->array);
  PyObject_Del(it);
}

static int ArrayIter_len(svms_ArrayIterObject *it) {
  int len=0;
  len = it->array->length - it->it_index;
  return len >= 0 ? len : 0;
}

static PyObject *ArrayIter_next(svms_ArrayIterObject *it) {
  if (it->it_index >= it->array->length)
    return NULL;
  return svms_GetItem(it->array, it->it_index++);
}

static PySequenceMethods arrayiter_as_sequence = {
  (lenfunc)ArrayIter_len, /* sq_length */
  0, /* sq_concat */
};

PyTypeObject svms_ArrayIterType = {
  PyObject_HEAD_INIT(NULL)
  0,					/* ob_size */
  SVMAPINAME".ArrayIter",			/* tp_name */
  sizeof(svms_ArrayIterObject),	/* tp_basicsize */
  0,					/* tp_itemsize */
  (destructor)ArrayIter_dealloc,	/* tp_dealloc */
  0,					/* tp_print */
  0,					/* tp_getattr */
  0,					/* tp_setattr */
  0,					/* tp_compare */
  0,					/* tp_repr */
  0,					/* tp_as_number */
  &arrayiter_as_sequence,		/* tp_as_sequence */
  0,					/* tp_as_mapping */
  0,					/* tp_hash */
  0,					/* tp_call */
  0,					/* tp_str */
  PyObject_GenericGetAttr,		/* tp_getattro */
  0,					/* tp_setattro */
  0,					/* tp_as_buffer */
  Py_TPFLAGS_DEFAULT,			/* tp_flags */
  "Array iterator objects to access array contents.",	/* tp_doc */
  0,					/* tp_traverse */
  0,					/* tp_clear */
  0,					/* tp_richcompare */
  0,					/* tp_weaklistoffset */
  PyObject_SelfIter,			/* tp_iter */
  (iternextfunc)ArrayIter_next,	/* tp_iternext */
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

static PyObject *Array_new(PyTypeObject *type, PyObject *args,
			   PyObject *kwds) {
  svms_ArrayObject *self = NULL;
  self = (svms_ArrayObject*) type->tp_alloc(type, 0);
  if (self != NULL) {
    self->array = NULL;
    self->ifree = self->offset = self->length = 0;
    self->type = AT_DOUBLE;
  }
  return (PyObject *)self;
}

svms_ArrayObject *Array_FromArray(void*array,int length,int offset,
				  enum array_type type) {
  svms_ArrayObject*so = (svms_ArrayObject*)
    PyObject_GC_New(svms_ArrayObject, &svms_ArrayType);
  so->array = array;
  so->length = length;
  so->offset = offset;
  so->type = type;
  so->ifree = 0;
  return so;
}

static int Array_init(svms_ArrayObject *self, PyObject *args,
	 	      PyObject *kwds) {
  PyObject *tuple;
  int i;
  size_t elem_size = 1;
  if (!PyArg_ParseTuple(args, "iiO!", &self->type, &self->offset,
			&PyTuple_Type, &tuple)) {
    return -1;
  }
  // Determine the size of individual elements.
  switch (self->type) {
  case AT_DOUBLE: elem_size=sizeof(double); break;
  case AT_INT: elem_size=sizeof(int); break;
  case AT_DOC: elem_size=sizeof(DOC*); break;
  default:
    PyErr_SetString(PyExc_ValueError, "bad type for array");
    return -1;
  }
  // Determine the number of elements to be in the array.
  self->length = PyTuple_GET_SIZE(tuple);
  if (self->length == 0) return 0;
  // Allocate the internal memory array.
  self->array = calloc(self->length + self->offset, elem_size);
  self->ifree = 1;
  for (i=0; i<self->length; ++i) {
    if (svms_SetItem(self, PySequence_GetItem(tuple, i), i))
      return -1;
  }
  return 0;
}

static PyObject *Array_Str(svms_ArrayObject *self) {
  PyObject *result = NULL;
  result = PyString_FromFormat
    ("<%s of type %d, length %d>",self->ob_type->tp_name,
     self->type,self->length);
  return result;
}

/* GC support functions. */
static int Array_traverse(svms_ArrayObject*self,visitproc visit,void*arg) {
  //Py_VISIT((PyObject*)self->dict);
  return 0;
}
static int Array_clear(svms_ArrayObject*self) {
  // Make sure we do not deallocate the next one.
  //Py_CLEAR(self->dict);
  return 0;
}
static void Array_dealloc(svms_ArrayObject *self) {
  if (self->ifree && self->array) {
    int i;
    for (i=0; i<self->length; ++i) {
      svms_DeallocItem(self, i);
    }
    free(self->array);
    self->array = NULL;
  }
  self->ob_type->tp_free((PyObject *)self);
}

static PyObject *Array_reduce(svms_ArrayObject *self) {
  return Py_BuildValue("O(iiN)", self->ob_type, self->type, self->offset,
		       PySequence_Tuple((PyObject*)self));
}

// Getter-setter methods.

// Type initialization method.

int Array_InitType(PyObject *module) {
  if (PyType_Ready(&svms_ArrayType) < 0) return -1;
  Py_INCREF(&svms_ArrayType);
  PyModule_AddObject(module, "Array", (PyObject*)&svms_ArrayType);

  if (PyType_Ready(&svms_ArrayIterType) < 0) return -1;
  Py_INCREF(&svms_ArrayIterType);
  PyModule_AddObject(module, "ArrayIter", (PyObject*)&svms_ArrayIterType);

  return 0;
}

// Sequence and mapping methods.

static int Array_Size(svms_ArrayObject *self) {
  return self->length;
}

static PyObject* Array_subscript(svms_ArrayObject *self, PyObject *item) {
  if (PyInt_Check(item)) {
    return svms_GetItem(self, PyInt_AsLong(item));
  } else if (PySlice_Check(item)) {
    Py_ssize_t start, stop, step, size, subsize, i;
    PyObject *sublist = NULL;
    size = self->length;
    if (PySlice_GetIndicesEx((PySliceObject*)item,size,&start,&stop,&step,
			     &subsize) < 0) return NULL;
    sublist = PyList_New(subsize);
    if (sublist == NULL) return NULL;
    for (i=0; i<subsize; ++i) {
      PyObject *item = svms_GetItem(self, i*step + start);
      if (item == NULL) { // Error getting the item...
	Py_DECREF(sublist);
	return NULL;
      }
      PyList_SET_ITEM(sublist, i, item);
    }
    return sublist;
  } else {
    PyErr_SetString(PyExc_TypeError, "bad index type for array");
    return NULL;
  }
}

// Type definition for the sparse object, including methods.

static PyMethodDef svms_ArrayMethods[] = {
  // Data setting and retrieval functions.
  {"__reduce__", (PyCFunction)Array_reduce, METH_NOARGS,
   "Pickling support."},
  {NULL}
};

static PyMemberDef svms_ArrayMembers[] = {
  //{"twonorm_sq", T_DOUBLE, offsetof(svms_ArrayObject, (*svec).twonorm_sq)+offsetof(SVECTOR,twonorm_sq), READONLY},
  {NULL}
};

static PyGetSetDef svms_ArrayGetSetters[] = {
  /*{"epsilon",(getter)Array_getepsilon,(setter)Array_setepsilon,
    "Precision to which to solve the quadratic program.",NULL},*/
  {NULL}
};

static PySequenceMethods Array_as_sequence = {
  (lenfunc)Array_Size,			/* sq_length */
  0,					/* sq_concat */
  0,					/* sq_repeat */
  0,					/* sq_item */
  0,					/* sq_slice */
  0,					/* sq_ass_item */
  0,					/* sq_ass_slice */
  0,					/* sq_contains */
};

static PyMappingMethods Array_as_mapping = {
  (lenfunc)Array_Size,			/* mp_length */
  (binaryfunc)Array_subscript,		/* mp_subscript */
  0,					/* mp_ass_subscript */
};

PyTypeObject svms_ArrayType = {
  PyObject_HEAD_INIT(NULL)
  0,					/* ob_size */
  SVMAPINAME".Array",			/* tp_name */
  sizeof(svms_ArrayObject),		/* tp_basicsize */
  0,					/* tp_itemsize */
  (destructor)Array_dealloc,		/* tp_dealloc */
  0,					/* tp_print */
  0,					/* tp_getattr */
  0,					/* tp_setattr */
  0,					/* tp_compare */
  (reprfunc)Array_Str,			/* tp_repr */
  0,					/* tp_as_number */
  &Array_as_sequence,			/* tp_as_sequence */
  &Array_as_mapping,			/* tp_as_mapping */
  0,					/* tp_hash */
  0,					/* tp_call */
  (reprfunc)Array_Str,			/* tp_str */
  0,					/* tp_getattro */
  0,					/* tp_setattro */
  0,					/* tp_as_buffer */
  Py_TPFLAGS_DEFAULT|Py_TPFLAGS_BASETYPE|Py_TPFLAGS_HAVE_GC,	/* tp_flags */
  "The array wrapper.\n\n"
  "This is used by SVM^python for convenient speedy encapsulation of\n"
  "arrays of various types, e.g., doubles, integers and others, that\n"
  "occur within various structures.",
					/* tp_doc */
  (traverseproc)Array_traverse,		/* tp_traverse */
  (inquiry)Array_clear,			/* tp_clear */
  0,					/* tp_richcompare */
  0,					/* tp_weaklistoffset */
  Array_iter,				/* tp_iter */
  0,					/* tp_iternext */
  svms_ArrayMethods,			/* tp_methods */
  svms_ArrayMembers,			/* tp_members */
  svms_ArrayGetSetters,			/* tp_getset */
  0,					/* tp_base */
  0,					/* tp_dict */
  0,					/* tp_descr_get */
  0,					/* tp_descr_set */
  0,					/* tp_dictoffset */
  (initproc)Array_init,			/* tp_init */
  0,					/* tp_alloc */
  Array_new,				/* tp_new */
};
