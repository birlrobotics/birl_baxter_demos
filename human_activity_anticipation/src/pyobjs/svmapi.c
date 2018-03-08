#include "svmapi.h"
#include "pyobjs.h"
#include "structmember.h"
#include "default.h"

PyObject *svmapi_usermodule;
PyObject *svmapi_thismodule;

PyObject* py_user_module(PyObject*self, PyObject *args) {
  Py_INCREF(svmapi_usermodule);
  return svmapi_usermodule;
}

static PyMethodDef svmapi_EmbMethods[] = {
  // THE DEFAULT FUNCTIONS
  //{PYTHON_READ_EXAMPLES, defaultPy_read_examples, METH_VARARGS, ""},
  {PYTHON_PARSE_PARAMETERS, defaultPy_noop, METH_VARARGS,
   "parse_parameters(sparm)\n\n"
   "This default implementation does nothing and returns None."},
  {PYTHON_PARSE_PARAMETERS_CLASSIFY, defaultPy_noop, METH_VARARGS,
   "parse_parameters_classify(attribute, value)\n\n"
   "This default implementation does nothing and returns None."},
  //{PYTHON_INIT_MODEL, defaultPy_init_model, METH_VARARGS, ""},
  {PYTHON_INIT_CONSTRAINTS, defaultPy_noop, METH_VARARGS,
   "init_constraints(sample, sm, sparm)\n\n"
   "This default implementation does nothing and returns None,\n"
   "equivalent to no special constraints being declared."},
  //{PYTHON_PSI, defaultPy_psi, METH_VARARGS, ""},
  //{PYTHON_CLASSIFY_EXAMPLE, defaultPy_classify_example, METH_VARARGS, ""},
  {PYTHON_FMVCS, defaultPy_fmvc_sm, METH_VARARGS,
   "find_most_violated_constraint_slack(x, y, sm, sparm)\n\n"
   "This default implementation calls the user's implementation of\n"
   "the more general find_most_violated_constraint(x, y, sm, sparm)."},
  {PYTHON_FMVCM, defaultPy_fmvc_sm, METH_VARARGS,
   "find_most_violated_constraint_margin(x, y, sm, sparm)\n\n"
   "This default implementation calls the user's implementation of\n"
   "the more general find_most_violated_constraint(x, y, sm, sparm)."},
  {PYTHON_FMVC, defaultPy_fmvc, METH_VARARGS, "",
   "find_most_violated_constraint(x, y, sm, sparm)\n\n"
   "This default implementation calls the user's implementation of\n"
   "classify_example(x, sm, sparm)."},
  {PYTHON_LOSS, defaultPy_loss, METH_VARARGS,
   "loss(y, ybar, sparm)\n\n"
   "This default implementation returns zero-one loss based on the\n"
   "comparison y==ybar, equivalent to 1-(y==ybar)."},
  {PYTHON_WRITE_MODEL, defaultPy_write_model, METH_VARARGS,
   "write_model(filename, sm, sparm)\n\n"
   "This default implementation pickles the model and dumps it into\n"
   "a BZ2 archive.  The equivalent code is\n"
   "cPickle.dump(sm, bz2.BZ2File(filename,'w'))"},
  {PYTHON_READ_MODEL, defaultPy_read_model, METH_VARARGS,
   "read_model(filename, sparm)\n\n"
   "This default implementation decompresses and depickles a model\n"
   "in a file, and returns it.  The equivalent code is\n"
   "return cPickle.load(bz2.BZ2File(filename))"},
  {PYTHON_PRINT_ITERATION_STATS, defaultPy_print_iteration_stats, METH_VARARGS,
   "print_iteration_stats(ceps, cached_constraint, sample,\n"
   "                      sm, cset, alpha, sparm)\n\n"
   "This default implementation prints nothing."},
  {PYTHON_PRINT_LEARNING_STATS, defaultPy_print_learning_stats, METH_VARARGS,
   "print_learning_stats(sample, sm, cset, alpha, sparm)\n\n"
   "This default implementation prints nothing."},
  {PYTHON_PRINT_TESTING_STATS, defaultPy_print_testing_stats, METH_VARARGS,
   "print_testing_stats(sample, sm, sparm, teststats)\n\n"
   "This default implementation prints nothing."},
  {PYTHON_EVAL_PREDICTION, defaultPy_eval_prediction, METH_VARARGS,
   "eval_prediction(exnum, (x, y), ypred, sm, sparm, teststats)\n\n"
   "This default implementation does nothing."},
  {PYTHON_WRITE_LABEL, defaultPy_write_label, METH_VARARGS,
   "write_label(fileptr, y)\n\n"
   "This default implementation prints the label to the file.\n"
   "Equivalent code is print>>fileptr, y ."},
  {PYTHON_PRINT_HELP, defaultPy_print_help, METH_VARARGS,
   "print_help()\n\n"
   "This default implementation prints the default help string for\n"
   "SVM^python, contained in svmapi.default_help ."},
  {PYTHON_PRINT_HELP_CLASSIFY, defaultPy_print_help, METH_VARARGS,
   "print_help_classify()\n\n"
   "This default implementation prints the default help string for\n"
   "SVM^python, contained in svmapi.default_help ."},

  //{"user_module", py_user_module, METH_NOARGS,
  //"user_module()\n\n" "Returns the user module."},

  {NULL,NULL,0,NULL}
};

PyMODINIT_FUNC initsvmapi(void) {
  PyObject *pExtModule=NULL;
  svmapi_usermodule=NULL;

  // Attempt to create the extension module.
  pExtModule = Py_InitModule3
    (SVMAPINAME, svmapi_EmbMethods,
     "Structures to support structural SVM learning.");
  // Add various functions and things to the extension module.
  Sparm_InitType(pExtModule);
  StructModel_InitType(pExtModule);
  Model_InitType(pExtModule);
  KernelParm_InitType(pExtModule);
  Sample_InitType(pExtModule);
  Constraints_InitType(pExtModule);

  Array_InitType(pExtModule);
  Sparse_InitType(pExtModule);
  Document_InitType(pExtModule);

  PyModule_AddStringConstant
    (pExtModule, "default_help",
     "         --* string  -> Custom parameters that can be adapted\n"
     "                        for struct learning or classification. The\n"
     "                        * can be replaced by any character and there\n"
     "                        can be multiple options starting with --.\n"
     "         --m modname -> Use the Python module 'modname' for the\n"
     "                        SVM^python user API module.");

  svmapi_usermodule = pExtModule;
  svmapi_thismodule = pExtModule;
}
