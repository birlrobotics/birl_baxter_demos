/* Copyright 2007 Thomas Finley, tfinley@gmail.com */

#include "graphobj.h"
#include "structmember.h"
#include "util.h"

#define EXTC 

// Methods for the graph object.

EXTC static PyObject *Graph_new(PyTypeObject *type, PyObject *args,
				PyObject *kwds) {
  GraphObject *self = NULL;
  
  //printf("New function called\n");
  self = (GraphObject*) type->tp_alloc(type, 0);
  if (self != NULL) {
    self->graph = NULL;
  }
  return (PyObject *)self;
}

EXTC static int Graph_init(GraphObject *self, PyObject *args,
			   PyObject *kwds) {
  self->graph = new Graph(0,0);
  if (self->graph == NULL) {
    PyErr_SetString(PyExc_MemoryError, "could not allocate graph structure!");
    return -1;
  }
  return 0;
}

EXTC static void Graph_dealloc(GraphObject *self) {
  if (self->graph) delete self->graph;
  self->ob_type->tp_free((PyObject*)self);
}

// METHODS

EXTC static PyObject* Graph_add_node(GraphObject *self, PyObject *args) {
  int numnodes = 1;
  if (!PyArg_ParseTuple(args, "|i", &numnodes)) return NULL;
  if (numnodes < 1) {
    PyErr_SetString(PyExc_ValueError, "num nodes to create must be positive");
    return NULL;
  }
  return PyInt_FromLong(self->graph->add_node(numnodes));
}

EXTC static PyObject* Graph_add_edge(GraphObject *self, PyObject *args) {
  int a, b, max;
  double a2b_cap, b2a_cap;
  if (!PyArg_ParseTuple(args, "iidd", &a, &b, &a2b_cap, &b2a_cap))
    return NULL;
  // Check bounds.
  if (a2b_cap < 0.0 || b2a_cap < 0.0) {
    PyErr_SetString(PyExc_ValueError, "capacities must not be negative");
    return NULL;
  }
  max = self->graph->get_node_num();
  if (a == b) {
    PyErr_SetString(PyExc_ValueError, "cannot add self-edge capacities");
    return NULL;
  }
  if (a < 0 || a >= max || b < 0 || b >= max) {
    PyErr_SetString(PyExc_ValueError, "node index out of bounds");
    return NULL;
  }
  self->graph->add_edge(a, b, a2b_cap, b2a_cap);
  Py_RETURN_NONE;
}

EXTC static PyObject* Graph_add_tweights(GraphObject *self, PyObject *args) {
  int a, max;
  double source2a_cap, a2sink_cap;
  if (!PyArg_ParseTuple(args, "idd", &a, &source2a_cap, &a2sink_cap))
    return NULL;
  // Check bounds.
  if (source2a_cap < 0.0 || a2sink_cap < 0.0) {
    PyErr_SetString(PyExc_ValueError, "capacities must not be negative");
    return NULL;
  }
  max = self->graph->get_node_num();
  if (a < 0 || a >= max) {
    PyErr_SetString(PyExc_ValueError, "node index out of bounds");
    return NULL;
  }
  self->graph->add_tweights(a, source2a_cap, a2sink_cap);
  Py_RETURN_NONE;
}

EXTC static PyObject* Graph_maxflow(GraphObject *self, PyObject *args) {
  return PyFloat_FromDouble(self->graph->maxflow());
}

EXTC static PyObject* Graph_segment(GraphObject *self, PyObject *args) {
  int a, max;
  if (!PyArg_ParseTuple(args, "i", &a)) {
    return NULL;
  }
  // Check bounds.
  max = self->graph->get_node_num();
  if (a < 0 || a >= max) {
    PyErr_SetString(PyExc_ValueError, "node index out of bounds");
    return NULL;
  }

  return PyBool_FromLong(self->graph->what_segment(a));
}

// GETTER SETTER METHODS

/*static PyObject* Graph_getsource(GraphObject *self, void *closure) {
  return PyInt_FromLong(self->graph->SOURCE); }

static PyObject* Graph_getsink(GraphObject *self, void *closure) {
return PyInt_FromLong(self->graph->SINK); }*/

static PyObject* Graph_getnumnodes(GraphObject *self, void *closure) {
  return PyInt_FromLong(self->graph->get_node_num()); }

static PyObject* Graph_getsegments(GraphObject *self, void *closure) {
  int i, max = self->graph->get_node_num();
  PyObject *retval = PyList_New(max);
  if (retval==NULL) return NULL;
  for (i=0; i<max; ++i) {
    PyList_SET_ITEM(retval, i, PyBool_FromLong(self->graph->what_segment(i)));
  }
  return retval;
}

static PyObject* Graph_getnodes(GraphObject *self, void *closure) {
  PyObject *retval = PyList_New(1);
/*  int i, max = self->graph->get_node_num(), count=0, tomatch=(int)closure;
  
  for (i=0; i<max; ++i) {
    if (self->graph->what_segment(i)==tomatch) ++count;
  }

  PyObject *retval = PyList_New(count);
  if (retval==NULL) return NULL;
  count = 0;

  for (i=0; i<max; ++i) {
    if (self->graph->what_segment(i)!=tomatch) continue;
    PyList_SET_ITEM(retval, count++, PyInt_FromLong(i));
  }
*/
  return retval;
}

// Type definition for the optimization object, including methods.

static PyMethodDef GraphMethods[] = {
  // Data setting and retrieval functions.
  {"add_node", (PyCFunction)Graph_add_node, METH_VARARGS,
   "add_node([numnodes=1])\n\n"
   "Creates numnodes nodes.  Returns ID of first created node.  Note\n"
   "all nodes created have integer indices sequentially in order\n"
   "after the first created node, that is, if i==add_node(j), then\n"
   "nodes with indices i, i+1, i+2, ..., i+j-1 now exist.  The first\n"
   "node ever created should have index 0."},
  {"add_edge", (PyCFunction)Graph_add_edge, METH_VARARGS,
   "add_edge(i, j, i2j_cap, j2i_cap)\n\n"
   "For nodes with index i and j, set capacities between i and j.\n"
   "Note that this is additive, that is, previous capacities between\n"
   "two given nodes are not overridden with this assignment.  Instead\n"
   "the new capacity is the old capacity plus whatever capacity was\n"
   "defined through this function.  Capacities cannot be negative."},
  {"add_tweights", (PyCFunction)Graph_add_tweights, METH_VARARGS,
   "add_tweights(i, source2i_cap, i2sink_cap)\n\n"
   "For node i, set capacity from source to i and to sink from i.\n"
   "Note that this has similar additive behavior as add_edge."},

  {"maxflow", (PyCFunction)Graph_maxflow, METH_NOARGS,
   "maxflow()\n\n"
   "Computes the maxflow/mincut, returning the flow value."},

  {"segment", (PyCFunction)Graph_segment, METH_VARARGS,
   "segment(i)\n\n"
   "Returns True if the given node i is in the sink's partition,\n"
   "or False if it is in the source's partition."},
  {NULL}
};

static PyMemberDef GraphMembers[] = {
  {NULL}
};

static PyGetSetDef GraphGetSet[] = {
  /*{"source", (getter)Graph_getsource, (setter)NULL, 
   "Node ID for the source node.", NULL},
  {"sink", (getter)Graph_getsink, (setter)NULL, 
  "Node ID for the sink node.", NULL},*/
  {"num_nodes", (getter)Graph_getnumnodes, (setter)NULL, 
   "The number of nodes in the graph.", NULL},

  {"segments", (getter)Graph_getsegments, (setter)NULL, 
   "A list, with an element for each node indicating if it is\n"
   "in the sink's partition.", NULL},
  {"source_nodes", (getter)Graph_getnodes, (setter)NULL, 
   "A list of the nodes in the source's parition.", (void*)Graph::SOURCE},
  {"sink_nodes", (getter)Graph_getnodes, (setter)NULL, 
   "A list of the nodes in the sink's partition.", (void*)Graph::SINK},

  {NULL}
};

PyTypeObject GraphType = {
  PyObject_HEAD_INIT(NULL)
  0,					/* ob_size */
  "graphcut.Graph",			/* tp_name */
  sizeof(GraphObject),	/* tp_basicsize */
  0,					/* tp_itemsize */
  (destructor)Graph_dealloc,		/* tp_dealloc */
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
  "Graph() --> A graph empty except for the source and sink.\n\n"
  "This represents a capacity graph one may run linear programming upon.\n"
  "Initially this graph is empty except for the source and sink, but one\n"
  "may add nodes and edges to this with given capacity, and run maxflow\n"
  "to run graph cut to partition nodes into source and sink partitions.\n\n"
  "Note that the source and sink nodes do not exist explicitly, but are\n"
  "assumed to always exist.  Consequences of this include that the number\n"
  "of nodes does not include the source and sink, the first index of a\n"
  "node is 0, and there are different methods for setting capacities\n"
  "between non-source/sink nodes, and a non-source/sink node and either\n"
  "the source or the sink.",
  /* tp_doc */
  0,					/* tp_traverse */
  0,					/* tp_clear */
  0,					/* tp_richcompare */
  0,					/* tp_weaklistoffset */
  0,					/* tp_iter */
  0,					/* tp_iternext */
  GraphMethods,				/* tp_methods */
  GraphMembers,				/* tp_members */
  GraphGetSet,				/* tp_getset */
  0,					/* tp_base */
  0,					/* tp_dict */
  0,					/* tp_descr_get */
  0,					/* tp_descr_set */
  0,					/* tp_dictoffset */
  (initproc)Graph_init,			/* tp_init */
  0,					/* tp_alloc */
  Graph_new,				/* tp_new */
};

