#include "sparse.h"
#include "structmember.h"
#include "svmapi_globals.h"

/* SPARSE ITERATOR OBJECT. */

typedef struct {
  PyObject_HEAD
  WORD *ptr;
  svms_SparseObject *sparse;
} svms_SparseIterObject;

static PyObject *Sparse_iter(PyObject *sparse) {
  svms_SparseIterObject *it;
  if (!Sparse_Check(sparse)) {
    PyErr_BadInternalCall();
    return NULL;
  }
  it = PyObject_New(svms_SparseIterObject, &svms_SparseIterType);
  if (it == NULL) return NULL;
  Py_INCREF(sparse);
  it->sparse = (svms_SparseObject *)sparse;
  it->ptr = it->sparse->sparse->words;
  return (PyObject *)it;
}

static void SparseIter_dealloc(svms_SparseIterObject *it) {
  Py_XDECREF(it->sparse);
  PyObject_Del(it);
}

static PyObject *SparseIter_next(svms_SparseIterObject *it) {
  if (it->ptr == NULL) return NULL;
  if (it->ptr->wnum == 0) return NULL;
  PyObject *retval = NULL;
  retval = Py_BuildValue("if", it->ptr->wnum-1, it->ptr->weight);
  it->ptr++;
  return retval;
}

PyTypeObject svms_SparseIterType = {
  PyObject_HEAD_INIT(NULL)
  0,					/* ob_size */
  SVMAPINAME".SparseIter",			/* tp_name */
  sizeof(svms_SparseIterObject),	/* tp_basicsize */
  0,					/* tp_itemsize */
  (destructor)SparseIter_dealloc,	/* tp_dealloc */
  0,					/* tp_print */
  0,					/* tp_getattr */
  0,					/* tp_setattr */
  0,					/* tp_compare */
  0,					/* tp_repr */
  0,					/* tp_as_number */
  0,					/* tp_as_sequence */
  0,					/* tp_as_mapping */
  0,					/* tp_hash */
  0,					/* tp_call */
  0,					/* tp_str */
  PyObject_GenericGetAttr,		/* tp_getattro */
  0,					/* tp_setattro */
  0,					/* tp_as_buffer */
  Py_TPFLAGS_DEFAULT,			/* tp_flags */
  "Sparse iterator objects, for accessing index-value pairs.",	/* tp_doc */
  0,					/* tp_traverse */
  0,					/* tp_clear */
  0,					/* tp_richcompare */
  0,					/* tp_weaklistoffset */
  PyObject_SelfIter,			/* tp_iter */
  (iternextfunc)SparseIter_next,	/* tp_iternext */
  0,					/* tp_methods */
  0,					/* tp_members */
  0,					/* tp_getset */
  0,					/* tp_base */
  0,					/* tp_dict */
  0,					/* tp_descr_get */
  0,					/* tp_descr_set */
  0,					/* tp_dictoffset */
};

/* SPARSE OBJECT. */

static int Sparse_Size(svms_SparseObject *self) {
  if (self->length < 0) {
    // The length is invalid.  Do a new count.
    WORD*w = self->sparse->words;
    self->length = 0;
    if (w) while ((w++)->wnum) ++self->length;
  }
  return self->length;
}

// Methods for the support vector list object.

static char* Sparse_EmptyString(void) { // Convenience function... bleh.
  char*cs = (char*)my_malloc(sizeof(char));
  cs[0] = 0;
  return cs;
}

static PyObject *Sparse_new(PyTypeObject *type, PyObject *args,
			   PyObject *kwds) {
  svms_SparseObject *self = NULL;
  self = (svms_SparseObject*) type->tp_alloc(type, 0);
  if (self != NULL) {
    self->sparse = NULL;
    self->length = -1;
    self->ifree = 1;
    self->owner = NULL;
  }
  return (PyObject *)self;
}

svms_SparseObject *Sparse_FromSparse(SVECTOR *sv, PyObject *owner) {
  svms_SparseObject*so = (svms_SparseObject*)
    PyObject_New(svms_SparseObject, &svms_SparseType);
  so->sparse = sv;
  so->length = -1;
  so->ifree = 0; // Assumes that this is just a placeholder...
  so->owner = owner;
  Py_XINCREF(owner);
  return so;
}

static int Sparse_init(svms_SparseObject *self, PyObject *args,
		       PyObject *kwds) {
  SVECTOR *sv = NULL;
  PyObject *argument=NULL;
  static char *kwlist[] = {"words","userdefined","factor","kernel_id",NULL};
  double factor=1.0;
  long int kernel_id=0;
  char *userdefined = NULL;
  int ud_strlen=0;

  char has_userdefined=PyTuple_Size(args)>=2 ||
    (kwds && PyMapping_HasKeyString(kwds, "userdefined"));
  char has_factor=PyTuple_Size(args)>=3 ||
    (kwds && PyMapping_HasKeyString(kwds, "factor"));
  char has_kernel_id=PyTuple_Size(args)>=4 ||
    (kwds && PyMapping_HasKeyString(kwds, "kernel_id"));

  // Read the arguments
  if (!PyArg_ParseTupleAndKeywords
      (args, kwds, "O|s#dl", kwlist, &argument, &userdefined, &ud_strlen,
       &factor, &kernel_id)) {
    return -1;
  }

  // Set up initial SV object.
  sv = self->sparse = (SVECTOR*)my_malloc(sizeof(SVECTOR));
  sv->words = NULL;
  sv->userdefined = NULL;
  sv->twonorm_sq = 0.0;
  sv->next = NULL;
  sv->factor = 1.0;
  sv->kernel_id = 0;

  if (Sparse_Check(argument)) {
    // This is a sparse object we should copy.
    svms_SparseObject *other = (svms_SparseObject*)argument;
    sv->factor = other->sparse->factor;
    sv->kernel_id = other->sparse->kernel_id;
    if (other->sparse->userdefined && !has_userdefined) {
      int len = strlen(other->sparse->userdefined)+1;
      char *place = (char*)my_malloc(sizeof(char*)*(len));
      strncpy(place, other->sparse->userdefined, len);
      sv->userdefined = place;
    } else {
      sv->userdefined = NULL;
    }
    sv->twonorm_sq = other->sparse->twonorm_sq;
    // Copy over the word array.
    int numwords = Sparse_Size(other);
    sv->words = (WORD*)my_malloc((numwords+1)*sizeof(WORD));
    memcpy(sv->words,other->sparse->words,(numwords+1)*sizeof(WORD));
    self->length = other->length;
  } else if (PySequence_Check(argument)) {
    // This is some sort of sequence.
    
    Py_ssize_t n = PySequence_Size(argument);
    sv->words = (WORD*)my_malloc((n+1)*sizeof(WORD));
    sv->words[n].weight = sv->words[n].wnum = 0;

    Py_ssize_t i,currkey,ii;
    for (i=0, currkey=0, ii=0; i<n; ++i) {
      PyObject *subobj = PySequence_GetItem(argument, i);
      if (subobj == NULL) return -1;
      ++currkey;

      if (PyNumber_Check(subobj)) {
	// It's a single number.  Extract it as the value;
	double value = PyFloat_AsDouble(subobj);
	Py_DECREF(subobj);
	if (PyErr_Occurred()) { return -1; }
	sv->words[ii].wnum = currkey;
	sv->words[ii].weight = value;
	sv->twonorm_sq += value * value;

      } else if (PyTuple_Check(subobj)) {
	// Check the size of the input tuple.
	if (PyTuple_Size(subobj) != 2) {
	  PyErr_SetString(PyExc_TypeError, "input tuples must be length 2");
	  Py_DECREF(subobj);
	  return -1;
	}

	// Extract the key and the value.
	int key = PyInt_AsLong(PyTuple_GetItem(subobj, 0))+1;
	if (PyErr_Occurred()) { Py_DECREF(subobj); return -1; }
	double value = PyFloat_AsDouble(PyTuple_GetItem(subobj, 1));
	if (PyErr_Occurred()) { Py_DECREF(subobj); return -1; }
	Py_DECREF(subobj);

	// Ensure that key is monotonically increasing.
	if (key < currkey) {
	  PyErr_SetString(PyExc_ValueError, "key values out of order");
	  return -1;
	} else {
	  sv->words[ii].wnum = currkey = key;
	  sv->words[ii].weight = value;
	  sv->twonorm_sq += value * value;
	}
      } else {
	// Bad input value type.
	PyErr_SetString(PyExc_TypeError, "input word must be number or tuple");
	Py_DECREF(subobj);
	return -1;
      }
      
      if (sv->words[ii].weight != 0.0) ++ii;
    }
    // Done with the iteration.
    sv->words[ii].wnum = 0;
    sv->words[ii].weight = 0.0;
    self->length = ii;
  }

  // Set the other parameters if they were specified.
  if (has_factor) {
    self->sparse->factor = factor;
  }
  if (has_kernel_id) {
    self->sparse->kernel_id = kernel_id;
  }

  if (has_userdefined) {
    char *place = (char*)my_malloc(sizeof(char)*(ud_strlen+1));
    strncpy(place, userdefined, ud_strlen+1);
    self->sparse->userdefined = place;
  }

  if (self->sparse->userdefined == NULL)
    self->sparse->userdefined = Sparse_EmptyString();

  return 0;
}

static PyObject *Sparse_Str(svms_SparseObject *self) {
  PyObject *pieces = NULL, *result = NULL, *s=NULL;
  char temp_string[256];
  int i, len=Sparse_Size(self);
  WORD*words=self->sparse->words;
  pieces = PyList_New(len+3);
  
  snprintf(temp_string, sizeof(temp_string),"factor=%g",self->sparse->factor);
  PyList_SET_ITEM(pieces, 0, PyString_FromString(temp_string));
  snprintf(temp_string, sizeof(temp_string),
	   "kernel_id=%ld", self->sparse->kernel_id);
  PyList_SET_ITEM(pieces, 1, PyString_FromString(temp_string));
  snprintf(temp_string, sizeof(temp_string),
	   "userdefined=%s", self->sparse->userdefined);
  PyList_SET_ITEM(pieces, 2, PyString_FromString(temp_string));
  
  for (i=0; i<len; ++i) {
    snprintf(temp_string, sizeof(temp_string), "%d:%g",
	     words[i].wnum-1, words[i].weight);
    PyList_SET_ITEM(pieces, i+3, PyString_FromString(temp_string));
  }
  s = PyString_FromString(" ");
  if (s != NULL) {
    result = _PyString_Join(s, pieces);
    Py_DECREF(s);
  }

  Py_XDECREF(pieces);
  return result;
}

SVECTOR *Sparse_AsSparse(PyObject *sparse) {
  if (!Sparse_Check(sparse)) {
    PyErr_BadInternalCall();
    return NULL;
  }
  SVECTOR *old = ((svms_SparseObject*)sparse)->sparse;
  SVECTOR *sv = create_svector(old->words, old->userdefined, old->factor);
  sv->kernel_id = old->kernel_id;
  return sv;
}

static void Sparse_dealloc(svms_SparseObject *self) {
  if (self->sparse && self->ifree) {
    SVECTOR*sv = self->sparse;
    if (sv->words) free(sv->words);
    if (sv->userdefined) free(sv->userdefined);
    free(sv);
    self->sparse = NULL;

    PyObject *owner = self->owner;
    self->owner = NULL;
    Py_XDECREF(owner);
  }
  self->ob_type->tp_free((PyObject *)self);
}

// Pickling support methods.

static PyObject *Sparse_reduce(svms_SparseObject *self) {
  return Py_BuildValue
    ("(O(Nsdi))", self->ob_type, PySequence_Tuple((PyObject*)self),
     self->sparse->userdefined, self->sparse->factor, self->sparse->kernel_id);
}

// Getter-setter methods.

static PyObject *Sparse_gettwonormsq(svms_SparseObject *self, void *closure) {
  return PyFloat_FromDouble(self->sparse->twonorm_sq);
}

static PyObject *Sparse_getkernelid(svms_SparseObject *self, void *closure) {
  return PyInt_FromLong(self->sparse->kernel_id);
}
static int Sparse_setkernelid(svms_SparseObject *self,PyObject *value,void*c) {
  value = PyNumber_Int(value);
  if (value == NULL) return -1;
  int i = PyInt_AsLong(value);
  Py_DECREF(value);
  self->sparse->kernel_id = i;
  return 0;
}

static PyObject *Sparse_getfactor(svms_SparseObject *self, void *closure) {
  return PyFloat_FromDouble(self->sparse->factor);
}
static int Sparse_setfactor(svms_SparseObject *self,PyObject *value,void*c) {
  value = PyNumber_Float(value);
  if (value == NULL) return -1;
  double d = PyFloat_AsDouble(value);
  Py_DECREF(value);
  self->sparse->factor = d;
  return 0;
}

static PyObject *Sparse_getuserdefined(svms_SparseObject *self,void *closure) {
  return Py_BuildValue("s", self->sparse->userdefined);
}
static int Sparse_setuserdefined(svms_SparseObject *self,PyObject *value,void*c) {
  if (value == Py_None || value == NULL) {
    // This is a deletion.
    if (self->sparse->userdefined) 
      free(self->sparse->userdefined);
    self->sparse->userdefined = Sparse_EmptyString();
    return 0;
  }
  
  char *ud = PyString_AsString(value);
  if (ud==NULL) return -1;
  int len = strlen(ud);
  if (self->sparse->userdefined) free(self->sparse->userdefined);
  self->sparse->userdefined = (char*)my_malloc(sizeof(char)*(len+1));
  strncpy(self->sparse->userdefined, ud, len+1);
  return 0;
}

// Type initialization method.

int Sparse_InitType(PyObject *module) {
  if (PyType_Ready(&svms_SparseType) < 0) return -1;
  Py_INCREF(&svms_SparseType);
  PyModule_AddObject(module, "Sparse", (PyObject*)&svms_SparseType);

  if (PyType_Ready(&svms_SparseIterType) < 0) return -1;
  Py_INCREF(&svms_SparseIterType);
  PyModule_AddObject(module, "SparseIter", (PyObject*)&svms_SparseIterType);

  return 0;
}

// Sequence and mapping methods.

static PyObject *svms_GetWord(svms_SparseObject *self, int i) {
  int size = Sparse_Size(self);
  if (i<0) i+=size;
  if (i>=size) {
    PyErr_SetString(PyExc_IndexError, "sparse index out of range");
    return NULL;
  }
  WORD *w = self->sparse->words + i;
  return Py_BuildValue("if", w->wnum-1, w->weight);
}

static PyObject* Sparse_subscript(svms_SparseObject *self, PyObject *item) {
  
  if (PyInt_Check(item)) {
    return svms_GetWord(self, PyInt_AsLong(item));
  } else if (PySlice_Check(item)) {
    Py_ssize_t start, stop, step, size, subsize, i;
    PyObject *sublist = NULL;
    size = Sparse_Size(self);
    if (PySlice_GetIndicesEx((PySliceObject*)item,size,&start,&stop,&step,
			     &subsize) < 0) return NULL;
    sublist = PyList_New(subsize);
    if (sublist == NULL) return NULL;
    for (i=0; i<subsize; ++i) {
      PyObject *item = svms_GetWord(self, i*step + start);
      if (item == NULL) { // Error getting the item...
	Py_DECREF(sublist);
	return NULL;
      }
      PyList_SET_ITEM(sublist, i, item);
    }
    return sublist;
  } else {
    PyErr_SetString(PyExc_TypeError, "bad index type for sparse");
    return NULL;
  }
}

// Type definition for the sparse object, including methods.

static PyMethodDef svms_SparseMethods[] = {
  // Data setting and retrieval functions.
  {"__reduce__", (PyCFunction)Sparse_reduce, METH_NOARGS,
   "Pickling support."},
  {NULL}
};

static PyMemberDef svms_SparseMembers[] = {
  //{"twonorm_sq", T_DOUBLE, offsetof(svms_SparseObject, (*svec).twonorm_sq)+offsetof(SVECTOR,twonorm_sq), READONLY},
  {NULL}
};

static PyGetSetDef svms_SparseGetSetters[] = {
  {"twonorm_sq",(getter)Sparse_gettwonormsq,(setter)NULL,
   "The squared euclidian length of the vector.",NULL},
  {"kernel_id",(getter)Sparse_getkernelid,(setter)Sparse_setkernelid,
   "The kernel ID.  Feature vectors with different K-IDs are\n"
   "considered orthogonal for the purpose of kernel evaluation.",NULL},
  {"userdefined",(getter)Sparse_getuserdefined,(setter)Sparse_setuserdefined,
   "A string holding additional information, or None if unset.",NULL},
  {"factor",(getter)Sparse_getfactor,(setter)Sparse_setfactor,
   "Factor by which this feature vector is multiplied when summed.",NULL},
  {NULL}
};

static PySequenceMethods Sparse_as_sequence = {
  (lenfunc)Sparse_Size,			/* sq_length */
  0,					/* sq_concat */
  0,					/* sq_repeat */
  0,					/* sq_item */
  0,					/* sq_slice */
  0,					/* sq_ass_item */
  0,					/* sq_ass_slice */
  0,					/* sq_contains */
};

static PyMappingMethods Sparse_as_mapping = {
  (lenfunc)Sparse_Size,			/* mp_length */
  (binaryfunc)Sparse_subscript,		/* mp_subscript */
  (objobjargproc)0,			/* mp_ass_subscript */
};

PyTypeObject svms_SparseType = {
  PyObject_HEAD_INIT(NULL)
  0,					/* ob_size */
  SVMAPINAME".Sparse",			/* tp_name */
  sizeof(svms_SparseObject),		/* tp_basicsize */
  0,					/* tp_itemsize */
  (destructor)Sparse_dealloc,		/* tp_dealloc */
  0,					/* tp_print */
  0,					/* tp_getattr */
  0,					/* tp_setattr */
  0,					/* tp_compare */
  (reprfunc)Sparse_Str,			/* tp_repr */
  0,					/* tp_as_number */
  &Sparse_as_sequence,			/* tp_as_sequence */
  &Sparse_as_mapping,			/* tp_as_mapping */
  0,					/* tp_hash */
  0,					/* tp_call */
  (reprfunc)Sparse_Str,			/* tp_str */
  0,					/* tp_getattro */
  0,					/* tp_setattro */
  0,					/* tp_as_buffer */
  Py_TPFLAGS_DEFAULT|Py_TPFLAGS_BASETYPE,	/* tp_flags */
  "Sparse(words [, userdefined, factor, kernel_id])\n"
  "    A single sparse vector with the indicated contents.\n\n"
  "This class is used by both the user to define new sparse vectors,\n"
  "and by SVM^python to pass around sparse vectors with the indicated\n"
  "contents.  The 'words' entry is a vector of either (index,value)\n"
  "pairs indicating that 'index' has value 'value', or just a 'value'\n"
  "number which is implicitly equivalent to '(index+1,value)', where\n"
  "'index' was the index of the last element.",
					/* tp_doc */
  0,					/* tp_traverse */
  0,					/* tp_clear */
  0,					/* tp_richcompare */
  0,					/* tp_weaklistoffset */
  Sparse_iter,				/* tp_iter */
  0,					/* tp_iternext */
  svms_SparseMethods,			/* tp_methods */
  svms_SparseMembers,			/* tp_members */
  svms_SparseGetSetters,		/* tp_getset */
  0,					/* tp_base */
  0,					/* tp_dict */
  0,					/* tp_descr_get */
  0,					/* tp_descr_set */
  0,					/* tp_dictoffset */
  (initproc)Sparse_init,		/* tp_init */
  0,					/* tp_alloc */
  Sparse_new,				/* tp_new */
};
