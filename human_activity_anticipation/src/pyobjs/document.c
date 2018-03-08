#include "document.h"
#include "sparse.h"
#include "structmember.h"
#include "svmapi_globals.h"

/* DOCUMENT ITERATOR OBJECT. */

typedef struct {
  PyObject_HEAD
  SVECTOR *ptr;
  svms_DocumentObject *doc;
} svms_DocumentIterObject;

static PyObject *Document_iter(PyObject *doc) {
  svms_DocumentIterObject *it;
  if (!Document_Check(doc)) {
    PyErr_BadInternalCall();
    return NULL;
  }
  it = PyObject_New(svms_DocumentIterObject, &svms_DocumentIterType);
  if (it == NULL) return NULL;
  Py_INCREF(doc);
  it->doc = (svms_DocumentObject *)doc;
  it->ptr = it->doc->doc->fvec;
  return (PyObject *)it;
}

static void DocumentIter_dealloc(svms_DocumentIterObject *it) {
  Py_XDECREF(it->doc);
  PyObject_Del(it);
}

static PyObject *DocumentIter_next(svms_DocumentIterObject *it) {
  if (it->ptr == NULL) return NULL;
  PyObject *retval = (PyObject*)Sparse_FromSparse(it->ptr, (PyObject*)it->doc);
  it->ptr = it->ptr->next;
  return retval;
}

PyTypeObject svms_DocumentIterType = {
  PyObject_HEAD_INIT(NULL)
  0,					/* ob_size */
  SVMAPINAME".DocumentIter",		/* tp_name */
  sizeof(svms_DocumentIterObject),	/* tp_basicsize */
  0,					/* tp_itemsize */
  (destructor)DocumentIter_dealloc,	/* tp_dealloc */
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
  "Document iterator objects",		/* tp_doc */
  0,					/* tp_traverse */
  0,					/* tp_clear */
  0,					/* tp_richcompare */
  0,					/* tp_weaklistoffset */
  PyObject_SelfIter,			/* tp_iter */
  (iternextfunc)DocumentIter_next,	/* tp_iternext */
  0,					/* tp_methods */
  0,					/* tp_members */
  0,					/* tp_getset */
  0,					/* tp_base */
  0,					/* tp_dict */
  0,					/* tp_descr_get */
  0,					/* tp_descr_set */
  0,					/* tp_dictoffset */
};

/* DOCUMENT OBJECT. */

static int Document_Size(svms_DocumentObject *self) {
  if (self->length < 0) {
    // The length is invalid.  Do a new count.
    SVECTOR*sv = self->doc->fvec;
    self->length = 0;
    while (sv != NULL) {
      sv = sv->next;
      ++self->length;
    }
  }
  return self->length;
}

// Methods for the support vector list object.

static PyObject *Document_new(PyTypeObject *type, PyObject *args,
			   PyObject *kwds) {
  svms_DocumentObject *self = NULL;
  self = (svms_DocumentObject*) type->tp_alloc(type, 0);
  if (self != NULL) {
    self->doc = NULL;
    self->length = -1;
    self->ifree = 1;
  }
  return (PyObject *)self;
}

svms_DocumentObject *Document_FromDocument(DOC *doc) {
  svms_DocumentObject*so = (svms_DocumentObject*)
    PyObject_New(svms_DocumentObject, &svms_DocumentType);
  so->doc = doc;
  so->length = -1;
  so->ifree = 0; // Assumes that this is just a placeholder...
  return so;
}

static int Document_init(svms_DocumentObject *self, PyObject *args,
		       PyObject *kwds) {
  DOC *doc = NULL;
  PyObject *argument=NULL;
  static char *kwlist[] = {"fvec","docnum","queryid","costfactor",
			   "slackid","kernelid",NULL};
  long docnum=0, queryid=0, slackid=0, kernelid=0;
  double costfactor=1.0;
  
  char has_docnum=PyTuple_Size(args)>=2 ||
    (kwds && PyMapping_HasKeyString(kwds, "docnum"));
  char has_queryid=PyTuple_Size(args)>=3 ||
    (kwds && PyMapping_HasKeyString(kwds, "queryid"));
  char has_costfactor=PyTuple_Size(args)>=4 ||
    (kwds && PyMapping_HasKeyString(kwds, "costfactor"));
  char has_slackid=PyTuple_Size(args)>=5 ||
    (kwds && PyMapping_HasKeyString(kwds, "slackid"));
  char has_kernelid=PyTuple_Size(args)>=6 ||
    (kwds && PyMapping_HasKeyString(kwds, "kernelid"));
  // Read the arguments
  if (!PyArg_ParseTupleAndKeywords
      (args, kwds, "O|iidii", kwlist, &argument, &docnum, &queryid,
       &costfactor, &slackid, &kernelid)) {
    return -1;
  }

  // Set up initial document object.
  doc = self->doc = (DOC*)my_malloc(sizeof(DOC));
  doc->fvec = NULL;
  doc->docnum = doc->queryid = doc->slackid = doc->kernelid = 0;
  doc->costfactor = 1.0;
  self->ifree = 1;

  if (Document_Check(argument)) {
    // This is a document object we should copy.
    svms_DocumentObject *other = (svms_DocumentObject*)argument;
    *doc = *(other->doc);
  }

  if (Sparse_Check(argument)) {
    // This is a sparse object.  Turn it into a sparse object
    // sequence, and let the rest of the code handle it.
    argument = Py_BuildValue("(N)", argument);
  }
  
  PyObject *it = PyObject_GetIter(argument), *subobject = NULL;
  if (it==NULL) return -1;
  SVECTOR *sv = NULL;
  while ((subobject = PyIter_Next(it)) != NULL) {
    // Make sure it is a sparse object.
    if (!Sparse_Check(subobject)) {
      PyErr_Format(PyExc_TypeError, "iterable requires '%s' objects",
		   svms_SparseType.tp_name);
      Py_DECREF(subobject);
      break;
    }
    // Copy over the sparse object.
    SVECTOR *newsv = Sparse_AsSparse(subobject);
    if (sv==NULL) {
      self->doc->fvec = newsv;
    } else {
      sv->next = newsv;
    }
    sv = newsv;
    // Prepare for the next iteration.
    Py_DECREF(subobject);
  }
  // Clean up and check for final errors.
  Py_DECREF(it);
  if (PyErr_Occurred()) {
    return -1;
  }

  // Copy over other parameters that were specified in the constructor.
  if (has_docnum) self->doc->docnum = docnum;
  if (has_queryid) self->doc->queryid = queryid;
  if (has_costfactor) self->doc->costfactor = costfactor;
  if (has_slackid) self->doc->slackid = slackid;
  if (has_kernelid) self->doc->kernelid = kernelid;

  return 0;
}

static PyObject *Document_Str(svms_DocumentObject *self) {
  // Make the arguments for the formatting operation.
  PyObject *args = Py_BuildValue
    ("iidii", self->doc->docnum, self->doc->queryid, self->doc->costfactor,
     self->doc->slackid, self->doc->kernelid);
  if (args == NULL) return NULL;
  // Make the format for the formatting operation.
  PyObject *format = PyString_FromString
    ("<document docnum=%d queryid=%d costfactor=%g slackid=%d kernelid=%d>");
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

DOC *Document_AsDocument(PyObject *self) {
  // Check to make sure this is a document object.
  if (!Document_Check(self)) {
    PyErr_BadInternalCall();
    return NULL;
  }
  // Retrieve the old and allocate the new documents.
  DOC *old = ((svms_DocumentObject*)self)->doc;
  DOC *doc = (DOC*)my_malloc(sizeof(DOC));
  // Copy over the scalar fields of the document structure.
  *doc = *old;
  if (doc->fvec == NULL) return doc;
  // Copy over the sparse vector objects.
  doc->fvec = copy_svector(doc->fvec);
  // Unfortunately the copy_svector function does not do *everything*.
  SVECTOR *a, *b;
  for (a=doc->fvec, b=old->fvec; a && b; a=a->next, b=b->next) {
    a->kernel_id = b->kernel_id;
  }
  // Return the newly copied document.
  return doc;
}

static void Document_dealloc(svms_DocumentObject *self) {
  if (self->doc && self->ifree) {
    DOC *d = self->doc;
    self->doc = NULL;
    free_example(d, 1);
  }
  self->ob_type->tp_free((PyObject *)self);
}

// Pickling support methods.

static PyObject *Document_reduce(svms_DocumentObject *self) {
  return Py_BuildValue
    ("(O(Niidii))", self->ob_type, PySequence_Tuple((PyObject*)self),
     self->doc->docnum, self->doc->queryid, self->doc->costfactor,
     self->doc->slackid, self->doc->kernelid);
}

// Getter-setter methods.

static PyObject *Document_getdocnum(svms_DocumentObject *self, void *closure) {
  return PyInt_FromLong(self->doc->docnum);
}

static PyObject *Document_getkernelid(svms_DocumentObject *self,
				      void *closure) {
  return PyInt_FromLong(self->doc->kernelid);
}
static int Document_setkernelid(svms_DocumentObject *self,
				PyObject *value, void*c) {
  value = PyNumber_Int(value);
  if (value == NULL) return -1;
  int i = PyInt_AsLong(value);
  Py_DECREF(value);
  self->doc->kernelid = i;
  return 0;
}

static PyObject *Document_getqueryid(svms_DocumentObject *self,
				      void *closure) {
  return PyInt_FromLong(self->doc->queryid);
}
static int Document_setqueryid(svms_DocumentObject *self,
				PyObject *value, void*c) {
  value = PyNumber_Int(value);
  if (value == NULL) return -1;
  int i = PyInt_AsLong(value);
  Py_DECREF(value);
  self->doc->queryid = i;
  return 0;
}

static PyObject *Document_getslackid(svms_DocumentObject *self,
				      void *closure) {
  return PyInt_FromLong(self->doc->slackid);
}
static int Document_setslackid(svms_DocumentObject *self,
				PyObject *value, void*c) {
  value = PyNumber_Int(value);
  if (value == NULL) return -1;
  int i = PyInt_AsLong(value);
  Py_DECREF(value);
  self->doc->slackid = i;
  return 0;
}

static PyObject *Document_getcostfactor(svms_DocumentObject *self, void *closure) {
  return PyFloat_FromDouble(self->doc->costfactor);
}
static int Document_setcostfactor(svms_DocumentObject *self,PyObject *value,void*c) {
  value = PyNumber_Float(value);
  if (value == NULL) return -1;
  double d = PyFloat_AsDouble(value);
  Py_DECREF(value);
  self->doc->costfactor = d;
  return 0;
}

// Type initialization method.

int Document_InitType(PyObject *module) {
  if (PyType_Ready(&svms_DocumentType) < 0) return -1;
  Py_INCREF(&svms_DocumentType);
  PyModule_AddObject(module, "Document", (PyObject*)&svms_DocumentType);

  if (PyType_Ready(&svms_DocumentIterType) < 0) return -1;
  Py_INCREF(&svms_DocumentIterType);
  PyModule_AddObject(module, "DocumentIter",(PyObject*)&svms_DocumentIterType);

  return 0;
}

// Type definition for the sparse object, including methods.

static PyMethodDef svms_DocumentMethods[] = {
  {"__reduce__", (PyCFunction)Document_reduce, METH_NOARGS,
   "Pickling support."},
  {NULL}
};

static PyMemberDef svms_DocumentMembers[] = {
  //{"twonorm_sq", T_DOUBLE, offsetof(svms_DocumentObject, (*svec).twonorm_sq)+offsetof(SVECTOR,twonorm_sq), READONLY},
  {NULL}
};

static PyGetSetDef svms_DocumentGetSetters[] = {
  {"docnum",(getter)Document_getdocnum,(setter)NULL,
   "The position of the document in the training set array.",NULL},
  {"kernelid",(getter)Document_getkernelid,(setter)Document_setkernelid,
   "Position in the gram matrix where the kernel value can be found\n"
   "when using an explicit gram matrix.",NULL},
  {"slackid",(getter)Document_getslackid,(setter)Document_setslackid,
   "Index of the slack variable corresponding to this constraint.",NULL},
  {"queryid",(getter)Document_getqueryid,(setter)Document_setqueryid,
   "Constraints are generated for documents with the same query ID.\n"
   "Used only when learning rankings.",NULL},
  {"costfactor",(getter)Document_getcostfactor,(setter)Document_setcostfactor,
   "Scales the cost of misclassifying this document by this factor.\n"
   "The upper bound of the alpha of this constraint is scaled by\n"
   "this factor.",NULL},
  {NULL}
};

static PySequenceMethods Document_as_sequence = {
  (lenfunc)Document_Size,			/* sq_length */
  0,					/* sq_concat */
  0,					/* sq_repeat */
  0,					/* sq_item */
  0,					/* sq_slice */
  0,					/* sq_ass_item */
  0,					/* sq_ass_slice */
  0,					/* sq_contains */
};

PyTypeObject svms_DocumentType = {
  PyObject_HEAD_INIT(NULL)
  0,					/* ob_size */
  SVMAPINAME".Document",			/* tp_name */
  sizeof(svms_DocumentObject),		/* tp_basicsize */
  0,					/* tp_itemsize */
  (destructor)Document_dealloc,		/* tp_dealloc */
  0,					/* tp_print */
  0,					/* tp_getattr */
  0,					/* tp_setattr */
  0,					/* tp_compare */
  (reprfunc)Document_Str,		/* tp_repr */
  0,					/* tp_as_number */
  &Document_as_sequence,		/* tp_as_sequence */
  0,//&Document_as_mapping,		/* tp_as_mapping */
  0,					/* tp_hash */
  0,					/* tp_call */
  (reprfunc)Document_Str,		/* tp_str */
  0,					/* tp_getattro */
  0,					/* tp_setattro */
  0,					/* tp_as_buffer */
  Py_TPFLAGS_DEFAULT|Py_TPFLAGS_BASETYPE, /* tp_flags */
  "Document(sparses [,docnum,queryid,costfactor,slackid,kernelid])\n"
  "    Construct a single example document.\n\n"
  "A document vector contains a sequence of "SVMAPINAME".Sparse\n"
  "vectors representing the contents (accessible through iteration)\n"
  "as well as several other data fields.",
					/* tp_doc */
  0,					/* tp_traverse */
  0,					/* tp_clear */
  0,					/* tp_richcompare */
  0,					/* tp_weaklistoffset */
  Document_iter,			/* tp_iter */
  0,					/* tp_iternext */
  svms_DocumentMethods,			/* tp_methods */
  svms_DocumentMembers,			/* tp_members */
  svms_DocumentGetSetters,		/* tp_getset */
  0,					/* tp_base */
  0,					/* tp_dict */
  0,					/* tp_descr_get */
  0,					/* tp_descr_set */
  0,					/* tp_dictoffset */
  (initproc)Document_init,		/* tp_init */
  0,					/* tp_alloc */
  Document_new,				/* tp_new */
};
