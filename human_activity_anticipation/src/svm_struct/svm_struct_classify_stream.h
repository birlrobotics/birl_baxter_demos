/***********************************************************************/
/*                                                                     */
/*   svm_struct_classify.c                                             */
/*                                                                     */
/*   Classification module of SVM-struct.                              */
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
/************************************************************************/

#include <stdio.h>
#ifdef __cplusplus
extern "C" {
#endif
#include "../svm_light/svm_common.h"
#include "../svm_struct_api.h"
#ifdef __cplusplus
}
#endif

char testfile[200];
char modelfile[200];
char predictionsfile[200];

void read_input_parameters(int, char **, char *, char *, char *, long *);
void print_help(void);

#ifdef __cplusplus
extern "C" {
#endif
int svm_classifier_init (int argc, char* argv[]);
int svm_classifier_exit (int argc, char* argv[]);
int svm_classifier (int argc, char* argv[]);
#ifdef __cplusplus
}
#endif




