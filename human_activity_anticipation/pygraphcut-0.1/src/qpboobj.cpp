/* Copyright 2007 Thomas Finley, tfinley@gmail.com */

#include "qpboobj.h"
#include "structmember.h"
#include "util.h"

#define EXTC 

// Methods for the graph object.

EXTC static PyObject *QPBO_new(PyTypeObject *type, PyObject *args,
				 PyObject *kwds) {
  QPBOObject *self = NULL;
  
  //printf("New function called\n");
  self = (QPBOObject*) type->tp_alloc(type, 0);
  if (self != NULL) {
    self->qpbo = NULL;
  }
  return (PyObject *)self;
}

EXTC static int QPBO_init(QPBOObject *self, PyObject *args,
			   PyObject *kwds) {
  int n,e;
  if (!PyArg_ParseTuple(args, "ii", &n, &e)) return NULL;
  self->qpbo = new QPBO<double>(n,e);
  if (self->qpbo == NULL) {
    PyErr_SetString(PyExc_MemoryError, "could not allocate qpbo structure!");
    return -1;
  }
  return 0;
}

EXTC static void QPBO_dealloc(QPBOObject *self) {
  if (self->qpbo) delete self->qpbo;
  self->ob_type->tp_free((PyObject*)self);
}

// METHODS
/*
static int inrange(QPBOObject *self, int var_index) {
  // Check bounds.
  int max;
  max = ((Graph*)self->qpbo)->get_node_num();
  if (var_index < 0 || var_index >= max) {
    PyErr_Format(PyExc_ValueError,"variable index %d out of bounds",var_index);
    return 0;
  }
  return 1;
}
*/
EXTC static PyObject* QPBO_add_node(QPBOObject *self, PyObject *args) {
  int n;
  if (!PyArg_ParseTuple(args, "i", &n)) return NULL;
  self->qpbo->AddNode(n);
  Py_RETURN_NONE;
}



EXTC static PyObject* QPBO_add_term(QPBOObject *self, PyObject *args) {
  int length=PyTuple_Size(args);

  switch (length) {
  case 1: {
//    double e;
//    if (!PyArg_ParseTuple(args, "d", &e)) return NULL;
 //   self->qpbo->add_constant(e);
    break;
  }
  case 3: {
    int n;
    double c0,c1;
    if (!PyArg_ParseTuple(args, "idd", &n, &c0, &c1)) return NULL;
    self->qpbo->AddUnaryTerm(n,c0,c1);

    break;
  } 
  case 6: {

    int n1,n2;
    double c00,c01,c10,c11;
    if (!PyArg_ParseTuple(args, "iidddd", &n1 , &n2, &c00, &c01, &c10, &c11)) return NULL;
    self->qpbo->AddPairwiseTerm(n1,n2,c00,c01,c10,c11);

    break;
  }
  default:
    PyErr_Format(PyExc_TypeError, "add_term expected 1, 3, or 6"
		 "arguments, got %d", length);
    return NULL;
  }
  Py_RETURN_NONE;
}

EXTC static PyObject* QPBO_solve(QPBOObject *self, PyObject *args) {
  self->qpbo->Solve();
  Py_RETURN_NONE;
}

EXTC static PyObject* QPBO_compute_weak_persistencies(QPBOObject *self, PyObject *args) {
  self->qpbo->ComputeWeakPersistencies();
  Py_RETURN_NONE;
}

EXTC static PyObject* QPBO_get_label(QPBOObject *self, PyObject *args) {
  int a;
  if (!PyArg_ParseTuple(args, "i", &a)) {
    return NULL;
  }
  //if (!inrange(self,a)) return NULL;
  return  PyInt_FromLong(self->qpbo->GetLabel(a));
}

// GETTER SETTER METHODS



// Type definition for the optimization object, including methods.

static PyMethodDef QPBOMethods[] = {
  // Data setting and retrieval functions.

  {"add_node", (PyCFunction)QPBO_add_node, METH_VARARGS,
   "add_node()\n\n"
   "Adds a variable to the qpbo problem, returning the ID."},

  {"add_term", (PyCFunction)QPBO_add_term, METH_VARARGS,
   "add_term(E)\nadd_term(x, E0, E1)\nadd_term(x, y, E00, E01, E10, E11)\n\n"
   "In first form, adds E to the objective.\n"
   "In second form, adds Ei to the objective if x==i.\n"
   "In third form, adds Eij to the objective if x==i and y==j.\n"
   "For the third form, it must be that E00+E11 <= E01+E10 to\n"
   "preserve submodularity."},

  {"solve", (PyCFunction)QPBO_solve, METH_NOARGS,
   "solve()\n\n"
   "Run qpbo minimization and return the minimization value."},

  {"compute_weak_persistencies", (PyCFunction)QPBO_compute_weak_persistencies, METH_NOARGS,
   "compute_weak_persistencies()\n\n"
   "Run qpbo minimization and return the minimization value."},

  {"get_label", (PyCFunction)QPBO_get_label, METH_VARARGS,
   "get_label(i)\n\n"
   "Returns the binary value for variable i."},
  {NULL}
};

static PyMemberDef QPBOMembers[] = {
  {NULL}
};

static PyGetSetDef QPBOGetSet[] = {
  {NULL}
};

PyTypeObject QPBOType = {
  PyObject_HEAD_INIT(NULL)
  0,					/* ob_size */
  "graphcut.QPBO",			/* tp_name */
  sizeof(QPBOObject),			/* tp_basicsize */
  0,					/* tp_itemsize */
  (destructor)QPBO_dealloc,		/* tp_dealloc */
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
  0,					/* tp_getattro */
  0,					/* tp_setattro */
  0,					/* tp_as_buffer */
  Py_TPFLAGS_DEFAULT,			/* tp_flags */
  "QPBO() --> An qpbo minimization object.\n\n"
  "This represents an qpbo minimization object.  One may define binary\n"
  "variables, and set qpbo terms (indicating the qpbo a variable\n"
  "contributes when it is various true/false states), and then run graph\n"
  "cut inference to get the minimization state.\n\n"
  "As a technical matter, this object is a subclass of the Graph object.\n"
  "Calling the superclass methods should not be necessary in most\n"
  "situations.",
  /* tp_doc */
  0,					/* tp_traverse */
  0,					/* tp_clear */
  0,					/* tp_richcompare */
  0,					/* tp_weaklistoffset */
  0,					/* tp_iter */
  0,					/* tp_iternext */
  QPBOMethods,			/* tp_methods */
  QPBOMembers,			/* tp_members */
  QPBOGetSet,				/* tp_getset */
  0, //&QPBOType,				/* tp_base */
  0,					/* tp_dict */
  0,					/* tp_descr_get */
  0,					/* tp_descr_set */
  0,					/* tp_dictoffset */
  (initproc)QPBO_init,		/* tp_init */
  0,					/* tp_alloc */
  QPBO_new,				/* tp_new */
};

