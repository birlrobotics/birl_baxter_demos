#ifndef __SVMAPI_GLOBALS_H
#define __SVMAPI_GLOBALS_H

#define SVMAPINAME "svmapi"

#if PY_MAJOR_VERSION == 2
#if PY_MINOR_VERSION < 5
typedef int Py_ssize_t;
typedef inquiry lenfunc;
typedef intargfunc ssizeargfunc;
typedef intobjargproc ssizeobjargproc;
#endif // PY_MINOR_VERSION < 5
#if PY_MINOR_VERSION < 4
#define Py_VISIT(o) do { if(o) { int v=visit((PyObject*)o,arg); \
			   if (v) return v; } } while (0)
#define Py_CLEAR(o) do { if(o) { PyObject *t=(PyObject*)o; (o)=NULL; \
			   Py_DECREF(t); } } while (0)
#define Py_RETURN_NONE return Py_INCREF(Py_None), Py_None
#endif // PY_MINOR_VERSION < 4
#endif // PY_MAJOR_VERSION == 2

/* The Python method names. */
#define PYTHON_READ_EXAMPLES		"read_examples"
#define PYTHON_PARSE_PARAMETERS		"parse_parameters"
#define PYTHON_PARSE_PARAMETERS_CLASSIFY "parse_parameters_classify"
#define PYTHON_INIT_MODEL		"init_model"
#define PYTHON_INIT_CONSTRAINTS		"init_constraints"
#define PYTHON_PSI			"psi"
#define PYTHON_CLASSIFY_EXAMPLE		"classify_example"
#define PYTHON_FMVCS			"find_most_violated_constraint_slack"
#define PYTHON_FMVCM			"find_most_violated_constraint_margin"
#define PYTHON_FMVC			"find_most_violated_constraint"
#define PYTHON_LOSS			"loss"
#define PYTHON_WRITE_MODEL		"write_model"
#define PYTHON_READ_MODEL		"read_model"
#define PYTHON_PRINT_ITERATION_STATS	"print_iteration_stats"
#define PYTHON_PRINT_LEARNING_STATS	"print_learning_stats"
#define PYTHON_PRINT_TESTING_STATS	"print_testing_stats"
#define PYTHON_EVAL_PREDICTION		"eval_prediction"
#define PYTHON_WRITE_LABEL		"write_label"
#define PYTHON_PRINT_HELP		"print_help"
#define PYTHON_PRINT_HELP_CLASSIFY	"print_help_classify"

#endif // __SVMAPI_GLOBALS_H
