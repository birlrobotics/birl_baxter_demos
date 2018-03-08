/***********************************************************************/
/*                                                                     */
/*   svm_struct_api.c                                                  */
/*                                                                     */
/*   Definition of API for attaching implementing SVM learning of      */
/*   structures (e.g. parsing, multi-label classification, HMM)        */ 
/*                                                                     */
/*   Author: Thorsten Joachims                                         */
/*   Date: 03.07.04                                                    */
/*                                                                     */
/*   Copyright (c) 2004  Thorsten Joachims - All rights reserved       */
/*                                                                     */
/*   This software is available for non-commercial use only. It must   */
/*   not be modified and distributed without prior permission of the   */
/*   author. The author is not responsible for implications from the   */
/*   use of this software.                                             */
/*                                                                     */
/***********************************************************************/

/* SVM Python v2.0.4
   Thomas Finley, tfinley@gmail.com */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <Python.h>
#include "svm_struct/svm_struct_common.h"
#include "svm_struct_api.h"

#include "pyobjs/svmapi.h"

//#define DEFAULT_MODULE svmstruct
#define XSTR(A) #A
#define STRINGIFY(A) XSTR(A)

#define PY_RUNCHECK { if(PyErr_Occurred()) { PyErr_Print(); Py_Exit(1); } }

/* The Python modules. */
PyObject *pModule;	// Module that we interface with.
PyObject *pExtModule;	// Extension module we provide.

/* This will search for the default implementation of the indicated
   function name, with a hard failure, that is, if it does not find
   the function in the indicated module, the process will print an
   error message and exit.  */
static PyObject* getFunction(const char *funcname) {
  PyObject *pDict, *pFunc;
  pDict = PyModule_GetDict(pModule);
  pFunc = PyDict_GetItemString(pDict, funcname);
  if (pFunc) return pFunc;
  pDict = PyModule_GetDict(pExtModule);
  pFunc = PyDict_GetItemString(pDict, funcname);
  if (pFunc) return pFunc;
  fprintf(stderr, "Could not find function %s!\n", funcname);
  Py_Exit(1);
  return NULL;
}

static void api_load_module(int argc, char* argv[]) {
  PyObject *pName;
  
  // Attempt to initialize the Python interpreter.
  Py_SetProgramName(argv[0]);
  Py_Initialize();
  PySys_SetArgv(argc, argv);

  // Load the extension module.
  initsvmapi();
  pExtModule = PyImport_AddModule(SVMAPINAME);
  
  // Get the name of the module.
  // First try the --m option...
  char *moduleName = NULL;
  int i;
  for (i=0; i<argc; ++i) if (!strcmp("--m", argv[i])) break;
  if (i<argc-1) moduleName = argv[i+1];
  // Next try the environment variable SVMPYTHON_MODULE.
  if (moduleName == NULL) moduleName = getenv("SVMPYTHON_MODULE");
  // Next just use the default module defined at build time.
  if (moduleName == NULL) moduleName = STRINGIFY(DEFAULT_MODULE);

  // Attempt to load the user module.
  pName = PyString_FromString(moduleName);
  pModule = PyImport_Import(pName);
  Py_DECREF(pName);

  if (pModule == NULL) {
    // If we could not load the module, output some helpful diagnostic output.
    fprintf(stderr, "COULD NOT LOAD MODULE \"%s\"!\n", moduleName);
    fprintf(stderr, "perhaps module is not in module search path?\n");
    fprintf(stderr, "path is: %s\n", Py_GetPath());
    Py_Exit(1);
  }

  svmapi_usermodule = pModule;
}

void        svm_struct_learn_api_init(int argc, char* argv[])
{
  /* Called in learning part before anything else is done to allow any
     initializations that might be necessary. */
  api_load_module(argc, argv);
}

void        svm_struct_learn_api_exit()
{
  /* Called in learning part at the very end to allow any clean-up
     that might be necessary. */
  Py_Finalize();
}

void        svm_struct_classify_api_init(int argc, char* argv[])
{
  /* Called in prediction part before anything else is done to allow
     any initializations that might be necessary. */
  api_load_module(argc, argv);
}

void        svm_struct_classify_api_exit()
{
  /* Called in prediction part at the very end to allow any clean-up
     that might be necessary. */
  Py_Finalize();
}

SAMPLE      read_struct_examples(char *file, STRUCT_LEARN_PARM *sparm)
{
  /* Reads struct examples and returns them in sample. The number of
     examples must be written into sample.n */
  SAMPLE   sample;  /* sample */
  EXAMPLE  *examples;
  PyObject *pFunc,*pValue,*pSeq;
  int i;

  // Call the relevant Python function.
  pFunc = getFunction(PYTHON_READ_EXAMPLES);
  pValue = PyObject_CallFunction(pFunc, "sN", file, Sparm_FromSparm(sparm));

  PY_RUNCHECK;
  
  // Convert this into a sequence.
  pSeq = PySequence_Fast(pValue, "examples not a sequence");
  Py_DECREF(pValue);
  if (pSeq == NULL) {
    PyErr_Print();
    Py_Exit(1);
  }

  // Read the examples from the sequence.
  sample.n = PySequence_Size(pSeq);
  examples=(EXAMPLE *)my_malloc(sizeof(EXAMPLE)*sample.n);
  for (i=0; i<sample.n; ++i) {
    PyObject *pExample = PySequence_Fast_GET_ITEM(pSeq, i);
    if (!pExample || !PySequence_Check(pExample) ||
        PySequence_Size(pExample)<2){
      fprintf(stderr, "%s's item %d is not a sequence element of "
              "at least two items!\n", PYTHON_READ_EXAMPLES, i);
      free(examples);
      Py_DECREF(pSeq);
      Py_Exit(1);
    }
    examples[i].x.py_x = PySequence_GetItem(pExample, 0);
    examples[i].y.py_y = PySequence_GetItem(pExample, 1);
    Py_DECREF(pExample);
  }
  Py_DECREF(pSeq);

  /* fill in your code here */
  sample.examples=examples;
  return(sample);
}

void        init_struct_model(SAMPLE sample, STRUCTMODEL *sm, 
			      STRUCT_LEARN_PARM *sparm, LEARN_PARM *lparm, 
			      KERNEL_PARM *kparm)
{
  /* Initialize structmodel sm. The weight vector w does not need to
     be initialized, but you need to provide the maximum size of the
     feature space in sizePsi. This is the maximum number of different
     weights that can be learned. Later, the weight vector w will
     contain the learned weights for the model. */
  PyObject *pFunc, *pValue;
  // Make sure these are not invalid values.
  sm->sizePsi=-1; /* replace by appropriate number of features */
  sm->lin_reduce = 1;
  sm->pydict = (void*)PyDict_New();
  sm->w = NULL;
  sm->svm_model = NULL;
  // Call the relevant Python function.
  pFunc = getFunction(PYTHON_INIT_MODEL);
  pValue = PyObject_CallFunction(pFunc, "NNN", Sample_FromSample(sample),
				 StructModel_FromStructModel(sm),
				 Sparm_FromSparm(sparm));
  PY_RUNCHECK;
  Py_DECREF(pValue);

  if (sm->sizePsi < 0) {
    fprintf(stderr, "%s did not specify sm.size_psi\n", PYTHON_INIT_MODEL);
    Py_Exit(1);
  }
}

CONSTSET    init_struct_constraints(SAMPLE sample, STRUCTMODEL *sm, 
				    STRUCT_LEARN_PARM *sparm)
{
  /* Initializes the optimization problem. Typically, you do not need
     to change this function, since you want to start with an empty
     set of constraints. However, if for example you have constraints
     that certain weights need to be positive, you might put that in
     here. The constraints are represented as lhs[i]*w >= rhs[i]. lhs
     is an array of feature vectors, rhs is an array of doubles. m is
     the number of constraints. The function returns the initial set
     of constraints. */
  CONSTSET c;
  PyObject *pFunc,*pValue,*pSeq;
  int i;

  // Set defaults of no constraints.
  c.m=0;
  c.lhs=NULL;
  c.rhs=NULL;
  // Call the relevant Python function.
  pFunc = getFunction(PYTHON_INIT_CONSTRAINTS);
  pValue = PyObject_CallFunction(pFunc, "NNN", Sample_FromSample(sample),
				 StructModel_FromStructModel(sm),
				 Sparm_FromSparm(sparm));
  //Py_DECREF(pArgs);
  PY_RUNCHECK;

  // Check for None possibility.
  if (pValue == Py_None) {
    // No constraints if no return value.
    Py_DECREF(pValue);
    return c;
  }
  // Convert this into a sequence.
  pSeq = PySequence_Fast(pValue, "constraints not a sequence");
  Py_DECREF(pValue);
  if (pSeq == NULL) {
    PyErr_Print();
    Py_Exit(1);
  }

  // Read the examples from the sequence.
  c.m = PySequence_Size(pSeq);
  if (c.m==0) {
    // Empty constraint set.
    Py_DECREF(pSeq);
    return c;
  }
  // Initialize the constraint data structures.
  c.lhs = (DOC **)my_malloc(sizeof(DOC*)*c.m);
  c.rhs = (double*)my_malloc(sizeof(double)*c.m);
  bzero(c.lhs, sizeof(DOC*)*c.m);
  bzero(c.rhs, sizeof(double)*c.m);
  // Try to iteratively extract the constraints.
  for (i=0; i<c.m; ++i) {
    PyObject *pConst = PySequence_Fast_GET_ITEM(pSeq, i);
    PyObject *pRHS, *pLHS;
    if (!pConst || !PySequence_Check(pConst) ||
        PySequence_Size(pConst)<2){
      fprintf(stderr, "%s's item %d is not a sequence element of "
              "at least two items!\n", PYTHON_INIT_CONSTRAINTS, i);
      goto error;
    }
    // Extract the right hand side.
    pRHS = PySequence_GetItem(pConst, 1);
    c.rhs[i] = PyFloat_AsDouble(pRHS);
    Py_XDECREF(pRHS);
    if (PyErr_Occurred()) {
      Py_DECREF(pConst);
      goto error;
    }
    // Extract the left hand side.
    pLHS = PySequence_GetItem(pConst, 0);
    c.lhs[i] = Document_AsDocument(pLHS);
    c.lhs[i]->docnum = i;
    Py_XDECREF(pLHS);
    //Py_DECREF(pConst);
    if (PyErr_Occurred()) {
      goto error;
    }
  }
  Py_DECREF(pSeq);
  return(c);

 error:
  // Free what document structures we have copied.
  Py_DECREF(pSeq);
  for (i=0; i<c.m; ++i) {
    if (c.lhs[i]) free_example(c.lhs[i],1);
  }
  free(c.lhs);
  free(c.rhs);
  PY_RUNCHECK;
  fprintf(stderr, PYTHON_INIT_CONSTRAINTS": should not be here\n");
  Py_Exit(2);
  return c;
}

LABEL       classify_struct_example(PATTERN x, STRUCTMODEL *sm, 
				    STRUCT_LEARN_PARM *sparm)
{
  /* Finds the label yhat for pattern x that scores the highest
     according to the linear evaluation function in sm, especially the
     weights sm.w. The returned label is taken as the prediction of sm
     for the pattern x. The weights correspond to the features defined
     by psi() and range from index 1 to index sm->sizePsi. If the
     function cannot find a label, it shall return an empty label as
     recognized by the function empty_label(y). */
  PyObject *pFunc, *pValue;
  // Call the relevant Python function.
  pFunc = getFunction(PYTHON_CLASSIFY_EXAMPLE);
  pValue = PyObject_CallFunction(pFunc, "ONN", (PyObject*)x.py_x,
				 StructModel_FromStructModel(sm),
				 Sparm_FromSparm(sparm));
  PY_RUNCHECK;
  // Store and return the appropriate Y label.
  LABEL y;
  y.py_y = pValue;
  return(y);
}

LABEL fmvc_helper(PATTERN x, LABEL y, STRUCTMODEL *sm,
		  STRUCT_LEARN_PARM *sparm, char *firstfunc) {
  PyObject *pFunc, *pValue;
  // Call the relevant Python function.
  pFunc = getFunction(firstfunc);
  // Call the function.
  pValue = PyObject_CallFunction
    (pFunc, "OONN", (PyObject*)x.py_x, (PyObject*)y.py_y,
     StructModel_FromStructModel(sm), Sparm_FromSparm(sparm));
  PY_RUNCHECK;
  // Store and return the appropriate Y label.
  LABEL ybar;
  ybar.py_y = pValue;
  return(ybar);
}

LABEL find_most_violated_constraint_slackrescaling
(PATTERN x, LABEL y, STRUCTMODEL *sm, STRUCT_LEARN_PARM *sparm) {
  /* Finds the label ybar for pattern x that that is responsible for
     the most violated constraint for the slack rescaling
     formulation. For linear slack variables, this is that label ybar
     that maximizes

            argmax_{ybar} loss(y,ybar)*(1-psi(x,y)+psi(x,ybar)) 

     Note that ybar may be equal to y (i.e. the max is 0), which is
     different from the algorithms described in
     [Tschantaridis/05]. Note that this argmax has to take into
     account the scoring function in sm, especially the weights sm.w,
     as well as the loss function, and whether linear or quadratic
     slacks are used. The weights in sm.w correspond to the features
     defined by psi() and range from index 1 to index
     sm->sizePsi. Most simple is the case of the zero/one loss
     function. For the zero/one loss, this function should return the
     highest scoring label ybar (which may be equal to the correct
     label y), or the second highest scoring label ybar, if
     Psi(x,ybar)>Psi(x,y)-1. If the function cannot find a label, it
     shall return an empty label as recognized by the function
     empty_label(y). */
  return fmvc_helper(x,y,sm,sparm,PYTHON_FMVCS);
}

LABEL       find_most_violated_constraint_marginrescaling
(PATTERN x, LABEL y, STRUCTMODEL *sm, STRUCT_LEARN_PARM *sparm) {
  /* Finds the label ybar for pattern x that that is responsible for
     the most violated constraint for the margin rescaling
     formulation. For linear slack variables, this is that label ybar
     that maximizes

            argmax_{ybar} loss(y,ybar)+psi(x,ybar)

     Note that ybar may be equal to y (i.e. the max is 0), which is
     different from the algorithms described in
     [Tschantaridis/05]. Note that this argmax has to take into
     account the scoring function in sm, especially the weights sm.w,
     as well as the loss function, and whether linear or quadratic
     slacks are used. The weights in sm.w correspond to the features
     defined by psi() and range from index 1 to index
     sm->sizePsi. Most simple is the case of the zero/one loss
     function. For the zero/one loss, this function should return the
     highest scoring label ybar (which may be equal to the correct
     label y), or the second highest scoring label ybar, if
     Psi(x,ybar)>Psi(x,y)-1. If the function cannot find a label, it
     shall return an empty label as recognized by the function
     empty_label(y). */
  return fmvc_helper(x,y,sm,sparm,PYTHON_FMVCM);
}

int         empty_label(LABEL y)
{
  /* Returns true, if y is an empty label. An empty label might be
     returned by find_most_violated_constraint_???(x, y, sm) if there
     is no incorrect label that can be found for x, or if it is unable
     to label x at all */
  return y.py_y == Py_None;
}

SVECTOR *psi_helper(PyObject*sv) {
  // Interpret the return value.
  if (!Sparse_Check(sv)) {
    fprintf(stderr, "%s did not return %s objects\n", PYTHON_PSI,
	    svms_SparseType.tp_name);
    return NULL;
  }
  svms_SparseObject *sparse = (svms_SparseObject*)sv;
  if (sparse->ob_refcnt==1 && sparse->ifree) {
    // If it is responsible for itself, and it is on its last refcnt,
    // then obviously there's no harm in just taking it for
    // ourselves!!
    sparse->ifree = 0;
    //printf("borrowing svec...\n");
    return sparse->sparse;
  } else {
    //printf("copying svec...\n");
    return Sparse_AsSparse(sv);
  }
}

SVECTOR     *psi(PATTERN x, LABEL y, STRUCTMODEL *sm,
		 STRUCT_LEARN_PARM *sparm)
{
  /* Returns a feature vector describing the match between pattern x
     and label y. The feature vector is returned as a list of
     SVECTOR's. Each SVECTOR is in a sparse representation of pairs
     <featurenumber:featurevalue>, where the last pair has
     featurenumber 0 as a terminator. Featurenumbers start with 1 and
     end with sizePsi. Featuresnumbers that are not specified default
     to value 0. As mentioned before, psi() actually returns a list of
     SVECTOR's. Each SVECTOR has a field 'factor' and 'next'. 'next'
     specifies the next element in the list, terminated by a NULL
     pointer. The list can be though of as a linear combination of
     vectors, where each vector is weighted by its 'factor'. This
     linear combination of feature vectors is multiplied with the
     learned (kernelized) weight vector to score label y for pattern
     x. Without kernels, there will be one weight in sm.w for each
     feature. Note that psi has to match
     find_most_violated_constraint_???(x, y, sm) and vice versa. In
     particular, find_most_violated_constraint_???(x, y, sm) finds
     that ybar!=y that maximizes psi(x,ybar,sm)*sm.w (where * is the
     inner vector product) and the appropriate function of the loss +
     margin/slack rescaling method. See that paper for details. */
  SVECTOR *fvec=NULL;

  /* insert code for computing the feature vector for x and y here */
  PyObject *pFunc, *pValue;

  // Call the relevant Python function.
  pFunc = getFunction(PYTHON_PSI);
  pValue = PyObject_CallFunction
  (pFunc, "OONN", (PyObject*)x.py_x, (PyObject*)y.py_y,
   StructModel_FromStructModel(sm), Sparm_FromSparm(sparm));
  PY_RUNCHECK;
  // Interpret the return value.
  if (Sparse_Check(pValue)) {
    fvec = psi_helper(pValue);
    Py_DECREF(pValue);
  } else {
    // Attempt to treat this like some sort of sequence.
    PyObject *pFast = PySequence_Fast(pValue, "return value not sequence");
    if (pFast==NULL) {
      fprintf(stderr, "%s did not return %s object or sequence\n", PYTHON_PSI,
	      svms_SparseType.tp_name);
      Py_Exit(1);
    }
    int i,size = PySequence_Fast_GET_SIZE(pFast);
    for (i=size-1; i>=0; --i) {
      SVECTOR *fvec_new = psi_helper(PySequence_Fast_GET_ITEM(pFast, i));
      if (fvec_new == NULL) {
	// Hmmm, error creating it.  Dispose of what we have created so far.
	if (fvec) {
	  free_svector(fvec);
	  fvec = NULL;
	}
	break;
      } else {
	// Prepend this vector to the list.
	fvec_new->next = fvec;
	fvec = fvec_new;
      }
    }
    Py_DECREF(pFast);
  }

  if (fvec == NULL) {
    Py_Exit(1);
  }

  return(fvec);
}

double      loss(LABEL y, LABEL ybar, STRUCT_LEARN_PARM *sparm) {
  /* loss for correct label y and predicted label ybar. The loss for
     y==ybar has to be zero. sparm->loss_function is set with the -l
     option. */

  PyObject *pFunc, *pValue;
  // Call the relevant Python function.
  pFunc = getFunction(PYTHON_LOSS);
  pValue = PyObject_CallFunction(pFunc, "OON", (PyObject*)y.py_y,
				 (PyObject*)ybar.py_y, Sparm_FromSparm(sparm));
  PY_RUNCHECK;
  double result = PyFloat_AsDouble(pValue);
  Py_DECREF(pValue);
  PY_RUNCHECK;
  return result;
}

void        print_struct_iteration_stats
(double ceps, int cached_constraint, SAMPLE sample, STRUCTMODEL *sm, 
 CONSTSET cset, double *alpha, STRUCT_LEARN_PARM *sparm) {
  /* This function is called just before the end of each cutting plane
     iteration. ceps is the amount by which the most violated
     constraint found in the current iteration was
     violated. cached_constraint is true if the added constraint was
     constructed from the cache. */
  PyObject *pFunc, *pValue;
  // Call the relevant Python function.
  pFunc = getFunction(PYTHON_PRINT_ITERATION_STATS);
  pValue = PyObject_CallFunction
    (pFunc, "dNNNNNN", ceps, PyBool_FromLong(cached_constraint),
     Sample_FromSample(sample), StructModel_FromStructModel(sm),
     Constraints_FromConstraints(cset),
     Array_FromArray(alpha,cset.m,0,AT_DOUBLE), Sparm_FromSparm(sparm));
  PY_RUNCHECK;
  Py_DECREF(pValue);
}


void        print_struct_learning_stats(SAMPLE sample, STRUCTMODEL *sm,
					CONSTSET cset, double *alpha, 
					STRUCT_LEARN_PARM *sparm)
{
  /* This function is called after training and allows final touches
     to the model sm. But primarly it allows computing and printing
     any kind of statistic (e.g. training error) you might want. */
  PyObject *pFunc, *pValue;
  // Call the relevant Python function.
  pFunc = getFunction(PYTHON_PRINT_LEARNING_STATS);
  pValue = PyObject_CallFunction
    (pFunc, "NNNNN", Sample_FromSample(sample),
     StructModel_FromStructModel(sm), Constraints_FromConstraints(cset),
     Array_FromArray(alpha,cset.m,0,AT_DOUBLE), Sparm_FromSparm(sparm));
  PY_RUNCHECK;
  Py_DECREF(pValue);
}

void        print_struct_testing_stats(SAMPLE sample, STRUCTMODEL *sm,
				       STRUCT_LEARN_PARM *sparm, 
				       STRUCT_TEST_STATS *teststats)
{
  /* This function is called after making all test predictions in
     svm_struct_classify and allows computing and printing any kind of
     evaluation (e.g. precision/recall) you might want. You can use
     the function eval_prediction to accumulate the necessary
     statistics for each prediction. */
  PyObject *pFunc, *pValue, *pArgs;
  // Call the relevant Python function.
  pFunc = getFunction(PYTHON_PRINT_TESTING_STATS);
  pArgs = Py_BuildValue
    ("NNNO", Sample_FromSample(sample), StructModel_FromStructModel(sm),
     Sparm_FromSparm(sparm), (PyObject*)teststats->pyobj);
  pValue = PyObject_CallObject(pFunc, pArgs);
  Py_DECREF((PyObject*)teststats->pyobj);
  teststats->pyobj=NULL;
  Py_DECREF(pArgs);
  PY_RUNCHECK;
  Py_DECREF(pValue);
}

void        eval_prediction(long exnum, EXAMPLE ex, LABEL ypred, 
			    STRUCTMODEL *sm, STRUCT_LEARN_PARM *sparm, 
			    STRUCT_TEST_STATS *teststats)
{
  /* This function allows you to accumlate statistic for how well the
     predicition matches the labeled example. It is called from
     svm_struct_classify. See also the function
     print_struct_testing_stats. */
  PyObject *pFunc, *pValue, *pArgs;
  PyObject *newstats = NULL;
  if(exnum == 0) { /* this is the first time the function is
		      called. So initialize the teststats */
    newstats = Py_None;
    Py_INCREF(newstats);
    teststats->pyobj = newstats;
  }
  // Call the relevant Python function.
  pFunc = getFunction(PYTHON_EVAL_PREDICTION);
  pArgs = Py_BuildValue
    ("i(OO)ONNO", exnum, (PyObject*)ex.x.py_x, (PyObject*)ex.y.py_y,
     (PyObject*)ypred.py_y, StructModel_FromStructModel(sm),
     Sparm_FromSparm(sparm), (PyObject*)teststats->pyobj);
  pValue = PyObject_CallObject(pFunc, pArgs);
  Py_DECREF(pArgs);
  PY_RUNCHECK;
  // Replace the test statistics object.
  Py_DECREF((PyObject*)teststats->pyobj);
  teststats->pyobj = pValue;
}

void        write_struct_model(char *file, STRUCTMODEL *sm, 
			       STRUCT_LEARN_PARM *sparm)
{
  /* Writes structural model sm to file file. */

  PyObject *pFunc, *pValue;
  // Reduce the support vectors if appropriate.
  if (sm->w && sm->lin_reduce) {
    MODEL *m = sm->svm_model;
    // Get rid of the old support vectors.
    int i;
    for (i=1; i<m->sv_num; ++i) {
      free_example(m->supvec[i], 1);
    }
    free(m->supvec);
    free(m->alpha);
    // Create the new model sv structurs.
    m->supvec = (DOC**)my_malloc(sizeof(DOC*)*2);
    m->supvec[0] = NULL;
    m->alpha = (double*)my_malloc(sizeof(double)*2);
    m->alpha[0]=0.0; m->alpha[1]=1.0;
    m->sv_num = 2;
    // Create the new support vector.
    SVECTOR *sv = create_svector_n(sm->w, sm->sizePsi,"",1.0);
    m->supvec[1] = create_example(0,0,1,1.0,sv);
  }
  // Call the relevant Python function.
  pFunc = getFunction(PYTHON_WRITE_MODEL);
  pValue = PyObject_CallFunction
    (pFunc,"sNN",file,StructModel_FromStructModel(sm),Sparm_FromSparm(sparm));
  PY_RUNCHECK;
  Py_DECREF(pValue);
}

STRUCTMODEL read_struct_model(char *file, STRUCT_LEARN_PARM *sparm)
{
  /* Reads structural model sm from file file. This function is used
     only in the prediction module, not in the learning module. */

  PyObject *pFunc, *pValue;
  
  // This is our first opportunity to initialize the structured
  // learning parameter object.
  bzero(sparm, sizeof(STRUCT_LEARN_PARM));
  sparm->pydict = (void*)PyDict_New();
  // Try to get the Python reading file.
  pFunc = getFunction(PYTHON_READ_MODEL);
  // We have a function!  Call the deserialization procedure.
  pValue = PyObject_CallFunction(pFunc, "sN", file, Sparm_FromSparm(sparm));
  PY_RUNCHECK;

  // No matter how we got it, we have some sort of object.
  if (!StructModel_Check(pValue)) {
    fprintf(stderr, "%s did not return a %s!\n", PYTHON_READ_MODEL,
	    svms_StructModelType.tp_name);
    Py_DECREF(pValue);
    Py_Exit(1);
  }
  // Now, we know we retrieved some sort of structure model.
  svms_StructModelObject *smo = (svms_StructModelObject*)pValue;
  smo->ifree = 0;
  STRUCTMODEL *sm = smo->sm;
  Py_XDECREF(pValue);
  return *sm; // We are leaking, yes, but relatively little, and only once...
}

int dummy_file_closer(FILE*f) {
  // Do nothing.
  return 0;
}
void        write_label(FILE *fp, LABEL y) {
  /* Writes label y to file handle fp. */
  PyObject *pFunc, *pValue, *pArgs;
  // Call the relevant Python function.
  pFunc = getFunction(PYTHON_WRITE_LABEL);
  pArgs = Py_BuildValue
    ("NO", PyFile_FromFile(fp, "foobar", "w", dummy_file_closer), y.py_y);
  pValue = PyObject_CallObject(pFunc, pArgs);
  Py_DECREF(pArgs);
  PY_RUNCHECK;
  Py_DECREF(pValue);
} 

void        free_pattern(PATTERN x) {
  /* Frees the memory of x. */
  Py_XDECREF((PyObject*)x.py_x);
}

void        free_label(LABEL y) {
  /* Frees the memory of y. */
  Py_XDECREF((PyObject*)y.py_y);
}

void        free_struct_model(STRUCTMODEL sm) 
{
  /* Frees the memory of model. */
  /* if(sm.w) free(sm.w); */ /* this is free'd in free_model */
  if(sm.svm_model) free_model(sm.svm_model,1);
  /* add free calls for user defined data here */
  Py_XDECREF((PyObject*)sm.pydict);
}

void        free_struct_sample(SAMPLE s)
{
  /* Frees the memory of sample s. */
  int i;
  for(i=0;i<s.n;i++) { 
    free_pattern(s.examples[i].x);
    free_label(s.examples[i].y);
  }
  free(s.examples);
}

void        print_struct_help()
{
  /* Prints a help text that is appended to the common help text of
     svm_struct_learn. */

  /* This function is called after making all test predictions in
     svm_struct_classify and allows computing and printing any kind of
     evaluation (e.g. precision/recall) you might want. You can use
     the function eval_prediction to accumulate the necessary
     statistics for each prediction. */
  PyObject *pFunc, *pValue;
  // Call the relevant Python function.
  pFunc = getFunction(PYTHON_PRINT_HELP);
  pValue = PyObject_CallFunctionObjArgs(pFunc, NULL);
  PY_RUNCHECK;
  Py_DECREF(pValue);
}

void         parse_struct_parameters(STRUCT_LEARN_PARM *sparm)
{
  sparm->pydict = (void*)PyDict_New();
  /* Parses the command line parameters that start with -- */
  PyObject *pFunc, *pValue;
  pFunc = getFunction(PYTHON_PARSE_PARAMETERS);
  pValue = PyObject_CallFunction(pFunc, "N", Sparm_FromSparm(sparm));
  PY_RUNCHECK;
  Py_DECREF(pValue);
}

void        print_struct_help_classify()
{
  /* Prints a help text that is appended to the common help text of
     svm_struct_classify. */
  PyObject *pFunc, *pValue;
  // Call the relevant Python function.
  pFunc = getFunction(PYTHON_PRINT_HELP_CLASSIFY);
  pValue = PyObject_CallFunctionObjArgs(pFunc, NULL);
  PY_RUNCHECK;
  Py_DECREF(pValue);
}

void         parse_struct_parameters_classify(char *attribute, char *value)
{
  /* Parses one command line parameters that start with -- . The name
     of the parameter is given in attribute, the value is given in
     value. */
  
  PyObject *pFunc, *pValue;
  // Call the relevant Python function.
  pFunc = getFunction(PYTHON_PARSE_PARAMETERS_CLASSIFY);
  pValue = PyObject_CallFunction(pFunc, "ss", attribute, value);
  PY_RUNCHECK;
  Py_DECREF(pValue);
}
