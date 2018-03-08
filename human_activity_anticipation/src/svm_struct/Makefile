# Makefile for SVM-struct, 03.07.04

#CC = gcc
#LD = gcc
#CC = attolcc -mempro -perfpro -block -proc -- gcc
#LD = attolcc -mempro -perfpro -block -proc -- gcc
#CFLAGS = -O3 -fomit-frame-pointer -ffast-math -Wall 
#LDFLAGS = -O3 -lm -Wall
#CFLAGS = -g -Wall
#LDFLAGS = -g -lm
# CFLAGS = -pg -Wall
# LDFLAGS = -pg -Wall

all: svm_struct_noexe

svm_struct_noexe: svm_struct_learn.o svm_struct_classify.o svm_struct_common.o svm_struct_main.o svm_struct_classify_stream.o 

.PHONY: clean
clean:
	rm -f *.o *.tcov *.d core gmon.out *.stackdump


#----------------------#
#----  STRUCT SVM  ----#
#----------------------#

svm_struct_common.o: svm_struct_common.c svm_struct_common.h ../svm_struct_api_types.h
	$(CC) -c $(CFLAGS) svm_struct_common.c -o svm_struct_common.o

svm_struct_learn.o: svm_struct_learn.c ../svm_light/svm_learn.h svm_struct_common.h ../svm_struct_api.h ../svm_struct_api_types.h
	$(CC) -c $(CFLAGS) svm_struct_learn.c -o svm_struct_learn.o

svm_struct_main.o: svm_struct_main.c ../svm_light/svm_common.h ../svm_light/svm_learn.h svm_struct_learn.h svm_struct_common.h ../svm_struct_api.h ../svm_struct_api_types.h
	$(CC) -c $(CFLAGS) svm_struct_main.c -o svm_struct_main.o

svm_struct_classify.o: svm_struct_classify.c svm_struct_common.h ../svm_struct_api_types.h ../svm_struct_api.h ../svm_light/svm_common.h 
	$(CC) -c $(CFLAGS) svm_struct_classify.c -o svm_struct_classify.o

svm_struct_classify_stream.o: svm_struct_classify_stream.c svm_struct_common.h ../svm_struct_api_types.h ../svm_struct_api.h ../svm_light/svm_common.h 
	$(CC) -c $(CFLAGS) svm_struct_classify_stream.c -o svm_struct_classify_stream.o
