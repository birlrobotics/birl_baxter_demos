/* Copyright 2007 Thomas Finley, tfinley@gmail.com */

#include "energyobj.h"
#include "structmember.h"
#include "util.h"

#define EXTC 

// Methods for the graph object.

EXTC static PyObject *Energy_new(PyTypeObject *type, PyObject *args,
				 PyObject *kwds) {
  EnergyObject *self = NULL;
  
  //printf("New function called\n");
  self = (EnergyObject*) type->tp_alloc(type, 0);
  if (self != NULL) {
    self->energy = NULL;
  }
  return (PyObject *)self;
}

EXTC static int Energy_init(EnergyObject *self, PyObject *args,
			   PyObject *kwds) {
  self->energy = new Energy();
  if (self->energy == NULL) {
    PyErr_SetString(PyExc_MemoryError, "could not allocate energy structure!");
    return -1;
  }
  return 0;
}

EXTC static void Energy_dealloc(EnergyObject *self) {
  if (self->energy) delete self->energy;
  self->ob_type->tp_free((PyObject*)self);
}

// METHODS

static int inrange(EnergyObject *self, int var_index) {
  // Check bounds.
  int max;
  max = ((Graph*)self->energy)->get_node_num();
  if (var_index < 0 || var_index >= max) {
    PyErr_Format(PyExc_ValueError,"variable index %d out of bounds",var_index);
    return 0;
  }
  return 1;
}

EXTC static PyObject* Energy_add_variable(EnergyObject *self, PyObject *args) {
  return PyInt_FromLong(self->energy->add_variable());
}

EXTC static PyObject* Energy_add_term(EnergyObject *self, PyObject *args) {
  int length=PyTuple_Size(args);

  switch (length) {
  case 1: {
    double e;
    if (!PyArg_ParseTuple(args, "d", &e)) return NULL;
    self->energy->add_constant(e);
    break;
  }
  case 3: {
    int x;
    double e0, e1;
    if (!PyArg_ParseTuple(args, "idd", &x, &e0, &e1)) return NULL;
    if (!inrange(self,x)) return NULL;
    self->energy->add_term1(x, e0, e1);
    break;
  } 
  case 6: {
    int x, y;
    double e00, e01, e10, e11;
    if (!PyArg_ParseTuple(args, "iidddd", &x, &y, &e00, &e01, &e10, &e11))
      return NULL;
    if (!inrange(self,x) || !inrange(self,y)) return NULL;
    if (x==y) {
      PyErr_SetString(PyExc_ValueError,
		      "cannot add term between identical variables");
      return NULL;
    }
    if (e01 + e10 < e00 + e11) {
      PyErr_SetString(PyExc_ValueError, "energy function is not submodular");
      return NULL;
    }
    self->energy->add_term2(x, y, e00, e01, e10, e11);
    break;
  }
  default:
    PyErr_Format(PyExc_TypeError, "add_term expected 1, 3, or 6"
		 "arguments, got %d", length);
    return NULL;
  }
  Py_RETURN_NONE;
}

EXTC static PyObject* Energy_minimize(EnergyObject *self, PyObject *args) {
  return PyFloat_FromDouble(self->energy->minimize());
}

EXTC static PyObject* Energy_get_var(EnergyObject *self, PyObject *args) {
  int a;
  if (!PyArg_ParseTuple(args, "i", &a)) {
    return NULL;
  }
  if (!inrange(self,a)) return NULL;
  return PyBool_FromLong(self->energy->get_var(a));
}

// GETTER SETTER METHODS



// Type definition for the optimization object, including methods.

static PyMethodDef EnergyMethods[] = {
  // Data setting and retrieval functions.

  {"add_variable", (PyCFunction)Energy_add_variable, METH_NOARGS,
   "add_variable()\n\n"
   "Adds a variable to the energy problem, returning the ID."},

  {"add_term", (PyCFunction)Energy_add_term, METH_VARARGS,
   "add_term(E)\nadd_term(x, E0, E1)\nadd_term(x, y, E00, E01, E10, E11)\n\n"
   "In first form, adds E to the objective.\n"
   "In second form, adds Ei to the objective if x==i.\n"
   "In third form, adds Eij to the objective if x==i and y==j.\n"
   "For the third form, it must be that E00+E11 <= E01+E10 to\n"
   "preserve submodularity."},

  {"minimize", (PyCFunction)Energy_minimize, METH_NOARGS,
   "minimize()\n\n"
   "Run energy minimization and return the minimization value."},

  {"var", (PyCFunction)Energy_get_var, METH_VARARGS,
   "var(i)\n\n"
   "Returns the binary value for variable i."},
  {NULL}
};

static PyMemberDef EnergyMembers[] = {
  {NULL}
};

static PyGetSetDef EnergyGetSet[] = {
  {NULL}
};

PyTypeObject EnergyType = {
  PyObject_HEAD_INIT(NULL)
  0,					/* ob_size */
  "graphcut.Energy",			/* tp_name */
  sizeof(EnergyObject),			/* tp_basicsize */
  0,					/* tp_itemsize */
  (destructor)Energy_dealloc,		/* tp_dealloc */
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
  "Energy() --> An energy minimization object.\n\n"
  "This represents an energy minimization object.  One may define binary\n"
  "variables, and set energy terms (indicating the energy a variable\n"
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
  EnergyMethods,			/* tp_methods */
  EnergyMembers,			/* tp_members */
  EnergyGetSet,				/* tp_getset */
  &GraphType,				/* tp_base */
  0,					/* tp_dict */
  0,					/* tp_descr_get */
  0,					/* tp_descr_set */
  0,					/* tp_dictoffset */
  (initproc)Energy_init,		/* tp_init */
  0,					/* tp_alloc */
  Energy_new,				/* tp_new */
};

