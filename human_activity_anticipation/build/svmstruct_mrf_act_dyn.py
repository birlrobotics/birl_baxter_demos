"""A module for SVM^python for multiclass MRF learning."""

"""action_prediction code for streaming """

import random

import os.path
import numpy
import sys
from numpy import random
import time
from operator import concat
import svmapi, array
import commands
from numpy import *
import scipy as Sci
import scipy.linalg
from scipy.sparse import lil_matrix
from scipy.sparse import csr_matrix
from numpy.ma.core import zeros
import glpk
from bitarray import bitarray
from graphcut import *
from scipy.sparse.coo import coo_matrix
import pickle 
#import ipdb

global FOLD
global DIVMBEST
global NUM_CLASSES_OBJ
global NUM_CLASSES_SKEL
global NUM_CLASSES
global ITER
global LP_LIST
global LOSS_WEIGHTS
global CLASSIFY_METHOD
global LEARN_METHOD
global ATTRIBUTE_METHOD
global LOSS_METHOD
global SINGLE_FRAME
global TEMPORAL
global NODEONLY

global Features
global IntegralFeatures
global Labeling
global objMap
global classmap
global affmap
global num_obj_feats
global num_skel_feats
global num_obj_t_feats
global num_skel_t_feats
global num_obj_obj_feats
global num_obj_skel_feats
global numBin
global Binning
global skelFeats_binTh
global skelTFeats_binTh
global objFeats_binTh
global objTFeats_binTh
global skelObjFeats_binTh
global objObjFeats_binTh
global NON_ADDITIVE
global NORMALIZE
global MIDPOINT_TEMPORAL
global NULL_FILTERED
global PATH
global PARTIAL
global ANTICIPATING

ANTICIPATING = True;
PARTIAL = "true";
FOLD = "1"
global HALLUCINATION
global hal_Features
global hal_IntegralFeatures

HALLUCINATION = "false"
#PATH = '/opt/ros/electric/stacks/'
PATH = '/expts/hema/'
MIDPOINT_TEMPORAL = "true"
NORMALIZE = "true"
NON_ADDITIVE="true"
NULL_FILTERED = "false"
classmap = {}
affmap = {}

LP_LIST= []
ITER = 0
NUM_CLASSES = 0
# default options
LOSS_METHOD = "micro"
LEARN_METHOD = "objassoc"
CLASSIFY_METHOD  = "sum1.IP"
ATTRIBUTE_METHOD = "false"
TEMPORAL = "false"
NODEONLY = "false"

numBin = 10
Binning = 1

def readClassmap(filename):
    global classmap,NUM_CLASSES_SKEL
    c = 0
    for l in file(filename):
        c+= 1

        classmap[l.strip()] = c
        print classmap[l.strip()]
    NUM_CLASSES_SKEL = len(classmap)

def readAffmap(filename):
    global affmap,NUM_CLASSES_OBJ
    c = 0
    for l in file(filename):
        c+= 1

        affmap[l.strip()] = c
    NUM_CLASSES_OBJ = len(affmap)

def parse_parameters(sparm):
    temp_arg_list = sparm.argv
    print sparm.argv
    global LEARN_METHOD
    global LOSS_METHOD
    global ATTRIBUTE_METHOD
    global SINGLE_FRAME
    global TEMPORAL
    global NODEONLY
    global FOLD
    global DIVMBEST
    # set default values
    ANTICIPATING = False
    LOSS_METHOD = "micro"
    LEARN_METHOD = "objassoc"
    for i in xrange(0,len(sparm.argv)/2):
        #print i,  len(sparm.argv)/2
        opt = temp_arg_list.pop(0)
        val = temp_arg_list.pop(0)
        if(opt == "--l"):
            LOSS_METHOD = val
        if(opt == "--lm"):
            #print "setting lm to ", val
            LEARN_METHOD = val
        if(opt == "--am"):
            print "setting am to ", val
            ATTRIBUTE_METHOD = val
        if(opt == "--sf"):
            SINGLE_FRAME= val
        if(opt == "--temporal"):
            TEMPORAL= val
        if(opt == "--nodeonly"):
            NODEONLY = val
        if(opt == "--hal"):
            HALLUCINATION = val
        if(opt == "--fold"):
            FOLD = val
        if(opt == "--divmbest"):
            DIVMBEST = val


def parse_parameters_classify(attribute, value):
    global CLASSIFY_METHOD
    global LEARN_METHOD
    global LOSS_METHOD
    global ATTRIBUTE_METHOD
    global SINGLE_FRAME
    global TEMPORAL
    global NODEONLY
    global HALLUCINATION
    global FOLD
    global DIVMBEST
    # set default values
    #print attribute, value
    if(attribute == "--l"):
        LOSS_METHOD = value
    if(attribute == "--lm"):
        LEARN_METHOD = value
        #print "setting lm to ", LEARN_METHOD
    if(attribute == "--cm"):
        CLASSIFY_METHOD= value
    if(attribute == "--am"):
        ATTRIBUTE_METHOD= value
    if(attribute == "--sf"):
        SINGLE_FRAME= value
    if(attribute == "--temporal"):
        TEMPORAL= value
    if(attribute == "--nodeonly"):
        NODEONLY= value
    if(attribute == "--hal"):
        HALLUCINATION = value
    if(attribute == "--fold"):
        FOLD = value
    if(attribute == "--divmbest"):
        DIVMBEST = value

def read_examples(filename,sparm):
    #ipdb.set_trace()
    global SINGLE_FRAME
    examples = []
    if(SINGLE_FRAME == "true"):
        examples = read_examples_single_frame(filename, sparm)
    else:
        #examples = read_examples_multiple_frames(filename, sparm)
        examples = get_examples(filename)
    return examples

def getBinTresholds(feats):
    global numBin
    th = []
    for i in xrange(0,shape(feats)[1]):
        a = sort(feats[:,i])
        #print shape(a)
        bv = []
        binSize=shape(a)[0]/numBin
        index = 0
        for b in xrange(0,numBin):
            index += binSize
            if(index < shape(a)[0]):
                #print index
                bv.append(a[index])
            else:
                bv.append(bv[b-1])
        th.append(bv)
    return th

def getBins(aidList):
    global num_obj_feats,num_skel_feats,num_obj_t_feats,num_skel_t_feats,num_obj_obj_feats,num_obj_skel_feats,skelFeats_binTh,skelTFeats_binTh,objFeats_binTh,objTFeats_binTh,objObjFeats_binTh,skelObjFeats_binTh
    global objMap
    global FOLD,ANTICIPATING 
    if ANTICIPATING: 
      if (os.path.isfile("fold"+FOLD+"/binStumps_skel.txt")):
        skelFeats_binTh = [l.strip().split(',') for l in file("fold"+FOLD+"/binStumps_skel.txt")]
        objFeats_binTh = [l.strip().split(',') for l in file("fold"+FOLD+"/binStumps_obj.txt")]
        skelTFeats_binTh = [l.strip().split(',') for l in file("fold"+FOLD+"/binStumps_skel_temporal.txt")]
        objTFeats_binTh = [l.strip().split(',') for l in file("fold"+FOLD+"/binStumps_obj_temporal.txt")]
        skelObjFeats_binTh = [l.strip().split(',') for l in file("fold"+FOLD+"/binStumps_skel_obj.txt")]
        objObjFeats_binTh = [l.strip().split(',') for l in file("fold"+FOLD+"/binStumps_obj_obj.txt")]
    else:

      if (os.path.isfile("binStumps_skel.txt")):
        skelFeats_binTh = [l.strip().split(',') for l in file("binStumps_skel.txt")]
        objFeats_binTh = [l.strip().split(',') for l in file("binStumps_obj.txt")]
        skelTFeats_binTh = [l.strip().split(',') for l in file("binStumps_skel_temporal.txt")]
        objTFeats_binTh = [l.strip().split(',') for l in file("binStumps_obj_temporal.txt")]
        skelObjFeats_binTh = [l.strip().split(',') for l in file("binStumps_skel_obj.txt")]
        objObjFeats_binTh = [l.strip().split(',') for l in file("binStumps_obj_obj.txt")]
      else:
        all_obj_feats = zeros((1,num_obj_feats))
        all_skel_feats = zeros((1,num_skel_feats))
        all_obj_obj_feats = zeros((1,num_obj_obj_feats))
        all_skel_obj_feats = zeros((1,num_obj_skel_feats))
        all_skel_t_feats = zeros((1,num_skel_t_feats))
        all_obj_t_feats = zeros((1,num_obj_t_feats))
        count = 0
        for aid in aidList:
            for segment in xrange(0,len(Labeling[0][aid])):
                count+=1
                sf = int(Labeling[0][aid][segment][0])-1
                ef = int(Labeling[0][aid][segment][1])-1
                print aid,sf,ef
                (obj_feats,skel_feats,obj_obj_feats,skel_obj_feats) = getSegmentFeatures(aid,sf,ef,0)
                #print shape(all_skel_feats), shape(mat(skel_feats))
                all_skel_feats = concatenate((all_skel_feats,mat(skel_feats)))
                for objId in objMap[aid]:
                    all_obj_feats = concatenate((all_obj_feats,mat(obj_feats[objId])))
                    all_skel_obj_feats = concatenate((all_skel_obj_feats,mat(skel_obj_feats[objId])))
                    for objId2 in objMap[aid]:
                        if(objId == objId2):
                            continue
                        all_obj_obj_feats = concatenate((all_obj_obj_feats,mat(obj_obj_feats[objId][objId2])))

        skelFeats_binTh = getBinTresholds(all_skel_feats[1:,:])
        objFeats_binTh = getBinTresholds(all_obj_feats[1:,:])
        skelObjFeats_binTh = getBinTresholds(all_skel_obj_feats[1:,:])
        objObjFeats_binTh = getBinTresholds(all_obj_obj_feats[1:,:])

        for aid in aidList:
            for link in xrange(1,len(Labeling[0][aid])):

                sf = int(Labeling[0][aid][link-1][0])-1
                mf = int(Labeling[0][aid][link-1][1])-1
                ef =  int(Labeling[0][aid][link][1])-1
                (obj_t_feats, skel_t_feats) = getTemporalFeatures(aid,sf,mf,ef,0)
                all_skel_t_feats = concatenate((all_skel_t_feats,mat(skel_t_feats)))
                for objId in objMap[aid]:
                    all_obj_t_feats = concatenate((all_obj_t_feats,mat(obj_t_feats[objId])))
        skelTFeats_binTh = getBinTresholds(all_skel_t_feats[1:,:])
        objTFeats_binTh = getBinTresholds(all_obj_t_feats[1:,:])
        fptr = open("binStumps_skel.txt","w")
        for l in skelFeats_binTh:
            print>>fptr, ','.join([`e` for e in l])
        fptr.close()
        fptr = open("binStumps_skel_temporal.txt","w")
        for l in skelTFeats_binTh:
            print>>fptr, ','.join([`e` for e in l])
        fptr.close()
        fptr = open("binStumps_obj.txt","w")
        for l in objFeats_binTh:
            print>>fptr, ','.join([`e` for e in l])
        fptr.close()
        fptr = open("binStumps_obj_temporal.txt","w")
        for l in objTFeats_binTh:
            print>>fptr, ','.join([`e` for e in l])
        fptr.close()
        fptr = open("binStumps_skel_obj.txt","w")
        for l in skelObjFeats_binTh:
            print>>fptr, ','.join([`e` for e in l])
        fptr.close()
        fptr = open("binStumps_obj_obj.txt","w")
        for l in objObjFeats_binTh:
            print>>fptr, ','.join([`e` for e in l])
        fptr.close()

def writeSkelFeatures(aid,seg,feats):
    filename = "skel_feats.txt"
    fileptr = open(filename, 'a')
    print >>fileptr,aid,
    print >>fileptr,seg,
    for i in xrange(0,len(feats)):
        if (feats[i]*10%10 == 0 ):
            feats[i] = int(feats[i])
        print>>fileptr,feats[i],
    print>>fileptr,""
    return

        
def writeTSkelFeatures(aid,seg,feats):
    filename = "temporal_skel_feats.txt"
    fileptr = open(filename, 'a')
    print >>fileptr,aid,
    print >>fileptr,seg,
    for i in xrange(0,len(feats)):
        if (feats[i]*10%10 == 0 ):
            feats[i] = int(feats[i])
        print>>fileptr,feats[i],
    print>>fileptr,""
    return

def writeObjFeatures(aid,seg,feats):
    filename = "obj_feats.txt"
    fileptr = open(filename, 'a')
    
    for o1 in objMap[aid]:
        print >>fileptr,aid,
        print >>fileptr,seg,
        print >>fileptr,o1,
        for j in xrange(0,len(feats[o1])):
            if (feats[o1][j]*10%10 == 0 ):
                feats[o1][j] = int(feats[o1][j])
            print>>fileptr,feats[o1][j],
        print>>fileptr,""
    return

def writeTObjFeatures(aid,seg,feats):
    filename = "temporal_obj_feats.txt"
    fileptr = open(filename, 'a')
    
    for o1 in objMap[aid]:
        print >>fileptr,aid,
        print >>fileptr,seg,
        print >>fileptr,o1,
        for j in xrange(0,len(feats[o1])):
            if (feats[o1][j]*10%10 == 0 ):
                feats[o1][j] = int(feats[o1][j])
            print>>fileptr,feats[o1][j],
        print>>fileptr,""
    return

def writeSkelObjFeatures(aid,seg,feats):
    filename = "skel_obj_feats.txt"
    fileptr = open(filename, 'a')
    
    for o1 in objMap[aid]:
        print >>fileptr,aid,
        print >>fileptr,seg,
        print >>fileptr,o1,
        for j in xrange(0,len(feats[o1])):
            print>>fileptr,feats[o1][j],
        print>>fileptr,""
    return

def writeObjObjFeatures(aid,seg,feats):
    filename = "obj_obj_feats.txt"
    fileptr = open(filename, 'a')
    
    for o1 in objMap[aid]:
        for o2 in objMap[aid]:
            if o1 == o2 :
                continue
            print >>fileptr,aid,
            print >>fileptr,seg,
            print >>fileptr,o1,
            print >>fileptr,o2,
            for j in xrange(0,len(feats[o1][o2])):
                print>>fileptr,feats[o1][o2][j],
            print>>fileptr,""
    return

def get_examples(filename):
    global TEMPORAL
    global NUM_CLASSES
    global NUM_CLASSES_OBJ
    global NUM_CLASSES_SKEL
    global NODEONLY
    global NULL_FILTERED
    global Features, IntegralFeatures, Labeling, num_obj_feats,num_skel_feats,num_obj_t_feats,num_skel_t_feats,num_obj_obj_feats,num_obj_skel_feats
    global PATH
    global PARTIAL
    global HALLUCINATION
     # Helper function for reading from files.

    """Parses an input file into an example sequence."""
    # This reads example files of the type read by SVM^multiclass.
    examples = []

    ''' Read the list of aids in the input file '''
    aidList = [line.strip() for line in (file(filename))]
    print aidList

    ''' Read the labeling file '''
    if NULL_FILTERED == "true":
        Labeling = readLabelingFile('../data/labeling_all_filtered.txt')
    else:
        Labeling = readLabelingFile('../data/labeling_ijrr_sampled.txt')

    with open('LabelingBinary.bn', 'wb') as f:
        pickle.dump(Labeling, f)

    ''' Read the frame features  '''
    #Features= readFrameFeatures(PATH + 'object_affordance_detection/activity_detection/frame_features/')
    #IntegralFeatures = getAllIntegralFrames(Features)



    num_obj_feats=0
    num_skel_feats=0
    num_obj_t_feats=0
    num_skel_t_feats=0
    num_obj_obj_feats=0
    num_obj_skel_feats=0
    for aid in aidList:
        if HALLUCINATION == "true":
            Labeling = readLabelingFile( 'hal_labeling_'+aid+'.txt')
        Features= readFrameFeatures( 'orig_'+ aid + "_")
        IntegralFeatures = getAllIntegralFrames(Features)
        #ipdb.set_trace()
        (obj_feats,skel_feats,obj_obj_feats,skel_obj_feats) = getSegmentFeatures(aid,0,2,0)
        if(len(skel_feats)>num_skel_feats):
            num_skel_feats= len(skel_feats)
        N1 = len(obj_feats)
        if(N1>0 and len(obj_feats['1'])> num_obj_feats):
            num_obj_feats = len(obj_feats['1'])
        if(N1>1 and len(obj_obj_feats['1']['2'])> num_obj_obj_feats):
            num_obj_obj_feats = len(obj_obj_feats['1']['2'])
        if(N1>0 and len(skel_obj_feats['1'])>num_obj_skel_feats):
            num_obj_skel_feats = len(skel_obj_feats['1'])
        (obj_t_feats,skel_t_feats) = getTemporalFeatures(aid,0,1,2,0)
        if(len(skel_t_feats)>num_skel_t_feats):
            num_skel_t_feats = len(skel_t_feats)
        if(N1>0 and len(obj_t_feats['1'])> num_obj_t_feats):
            num_obj_t_feats = len(obj_t_feats['1'])

    if(Binning):
        getBins(aidList)
        num_obj_feats=num_obj_feats*numBin
        num_skel_feats=num_skel_feats*numBin
        num_obj_t_feats=num_obj_t_feats*numBin
        num_skel_t_feats=num_skel_t_feats*numBin
        num_obj_obj_feats=num_obj_obj_feats*numBin
        num_obj_skel_feats=num_obj_skel_feats*numBin
        if(N1 == 1): 
            num_obj_obj_feats = 200

    start = time.clock()

    #K1 = 12
    #K2 = 9
    K1 = NUM_CLASSES_OBJ
    K2 = NUM_CLASSES_SKEL
    max_obj_target = K1 # use the max number of classes read from the file
    max_skel_target = K2
    #NUM_CLASSES_OBJ = K1
    #NUM_CLASSES_SKEL = K2
    

    print 'number of obj classes: ', max_obj_target
    print 'number of skel classes: ', max_skel_target

    print 'number of obj features: ', num_obj_feats
    print 'number of skel features: ', num_skel_feats
    print 'number of obj obj edge features: ',num_obj_obj_feats
    print 'number of obj skel edge features: ',num_obj_skel_feats
    print 'number of obj temporal edge features: ',num_obj_t_feats
    print 'number of skel temporal edge features: ',num_skel_t_feats

    print 'time spent in preprocessing data files', start-time.clock()



    frame_count = 0
    temporal_edge_count = 0
    X_multiple = []
    Y_multiple = []
    X_temporal = []
    Y_temporal = []
    edges_obj_obj_list = []
    edges_obj_skel_list = []
    edges_obj_temporal_list = []
    temporal_frames_list = []
    N1_list = []
    E_list = []
    obj_map_list = []
    frame_map = {}
    example_num=-1
    for aid in aidList:
        print "HALLU: ", HALLUCINATION;
        if(HALLUCINATION == "true"):
            print "reading hal features!!"
            Features= readFrameFeatures( 'hal_'+ aid + "_")
            Labeling = readLabelingFile('hal_labeling_'+aid+'.txt')
        else:
            Features= readFrameFeatures( 'orig_'+ aid + "_")
        IntegralFeatures = getAllIntegralFrames(Features)
        X_multiple = []
        Y_multiple = []
        X_temporal = []
        Y_temporal = []
        frame_count = 0
        frame_map = {}
        obj_map_list = []
        edges_obj_obj_list = []
        edges_obj_skel_list = []
        N1_list = []
        E_list = []
        example_num+=1
        numframes = len(Features[1][aid])
        print "numframes = ", numframes
        lastSegment = 0
        for segment in xrange(0,len(Labeling[0][aid])):
            sf = int(Labeling[0][aid][segment][0])-1
            ef = int(Labeling[0][aid][segment][1])-1
            # if hallucination.. check for last segment
            if(HALLUCINATION == "true"):
                '''if (segment != len(Labeling[0][aid])-1):
                    nsf = int(Labeling[0][aid][segment+1][0])-1
                    if(nsf >= numframes -4):
                        lastSegment = 1
                        print "last segment:", segment'''
                if(segment == len(Labeling[0][aid]) - 1):
                    #lastSegment = 1
                    lastSegment = 0 # FULL HALLUCINATION  
            
            if(sf >= numframes -3):
                break;
            if(ef >= numframes):
                ef = numframes-1
            FN = segment+1;
            frame_count +=1
           ## print frame_count
            (X_sparse,N1,E1,E2,obj_map,edges_obj_obj,edges_obj_skel) = getSegmentFeatureXMat(aid,sf,ef,lastSegment)
            Y = getSegmentFeatureYMat(aid,N1,E1,E2,Labeling[1][aid][FN],lastSegment)
            frame_map[FN] = frame_count-1 # 0 indexed
            #print frame_map
            edges_obj_obj_list.append(edges_obj_obj)
            edges_obj_skel_list.append(edges_obj_skel)

            obj_map_list.append(obj_map)

            N1_list.append(N1)
            E_list.append((E1,E2))
            if (frame_count ==1 ):
                X_multiple = X_sparse
                Y_multiple = Y
            else:

                data = scipy.concatenate((X_multiple.data, X_sparse.data))
                row= scipy.concatenate((X_multiple.row, X_sparse.row))
                col = scipy.concatenate((X_multiple.col, X_sparse.col + X_multiple.shape[1]))
                X_multiple = coo_matrix((data,(row,col)), shape= (X_multiple.shape[0], X_multiple.shape[1] + X_sparse.shape[1]) )
                Y_multiple = concatenate((Y_multiple, Y))
        #print obj_map_list
        if(TEMPORAL == "true"):
            temporal_edge_count = 0
            temporal_frames_list = []
            edges_obj_temporal_list = []
            for link in xrange(1,len(Labeling[0][aid])):
                
                sf = int(Labeling[0][aid][link-1][0]) -1
                mf = int(Labeling[0][aid][link-1][1]) -1
                ef =  int(Labeling[0][aid][link][1]) -1
                if(int(Labeling[0][aid][link][1]) -1 >= numframes -4):
                    break;
                if(ef >= numframes):
                    ef = numframes-1
                temporal_edge_count += 1
                FN1 = link
                FN2 = link+1
                E3 = 0
                if aid in objMap:
                    E3 = len(objMap[aid])
                temporal_frames_list.append((frame_map[FN1],frame_map[FN2]))
                edges_obj_temporal = mat(zeros((E3,1)))
                i = -1
                if aid in objMap:
                  for objId in objMap[aid]:
                    i += 1
                    print frame_map[FN2],obj_map_list[frame_map[FN2]]
                    edges_obj_temporal[i,0]= obj_map_list[frame_map[FN2]][objId]


                (X_sparse) = getTemporalFeatureXMat(aid,sf,mf,ef,lastSegment)

                Y = getTemporalFeatureYMat(aid,Labeling[1][aid][link],Labeling[1][aid][link+1],lastSegment)



                if (temporal_edge_count ==1 ):
                    X_temporal = X_sparse
                    Y_temporal = Y
                else:
                    data = scipy.concatenate((X_temporal.data, X_sparse.data))
                    row= scipy.concatenate((X_temporal.row, X_sparse.row))
                    col = scipy.concatenate((X_temporal.col, X_sparse.col + X_temporal.shape[1]))
                    X_temporal = coo_matrix((data,(row,col)), shape= (X_temporal.shape[0], X_temporal.shape[1] + X_sparse.shape[1]) )
                    Y_temporal = concatenate((Y_temporal, Y))

                edges_obj_temporal_list.append(edges_obj_temporal)
            #edges_skel_temporal_list.append(edges_skel_temporal)
            if (temporal_edge_count != 0):
                Y_multiple = concatenate((Y_multiple, Y_temporal))
                data = scipy.concatenate((X_multiple.data, X_temporal.data))
                row= scipy.concatenate((X_multiple.row, X_temporal.row + X_multiple.shape[0]))
                col = scipy.concatenate((X_multiple.col, X_temporal.col + X_multiple.shape[1]))
                X_multiple = coo_matrix((data,(row,col)), shape= ( X_multiple.shape[0] + X_temporal.shape[0], X_multiple.shape[1] + X_temporal.shape[1]) )
            else:
                data = X_multiple.data
                row= X_multiple.row
                col = X_multiple.col
                X_multiple = coo_matrix((data,(row,col)), shape= ( X_multiple.shape[0] + num_obj_t_feats*max_obj_target*max_obj_target+ num_skel_t_feats*max_skel_target*max_skel_target  , X_multiple.shape[1]) )


        Y_m_s = csr_matrix(Y_multiple,dtype='d');
        X_multiple = X_multiple.tocsr()
        #xd =  X_multiple.todense()
        #numpy.savetxt('matrixX_new.txt',xd,delimiter=' ',fmt='%-2.1f')
        #yd =  Y_multiple.todense()
        numpy.savetxt('matrixY_new.txt',Y_multiple,delimiter=' ',fmt='%-2.1f')
        #print Y_m_s.shape
        #print X_multiple.shape

        examples.append(((X_multiple, edges_obj_obj_list, edges_obj_skel_list, N1_list, E_list, frame_count, edges_obj_temporal_list, temporal_frames_list, temporal_edge_count,  example_num,obj_map_list, aid ), (Y_m_s,N1_list,E_list,frame_count,temporal_edge_count,obj_map_list)))
        if HALLUCINATION == "true":
        
            weights = [l.strip() for l in file('fold'+FOLD+'/modelweights_filtered')]
            w_mat = csr_matrix(asmatrix(array(weights),dtype='d'),dtype='d')
            score = (w_mat*X_multiple*Y_m_s)[0,0]
            print "score:", score
            f = open('score_'+aid+'.txt','w')
            f.write(repr(score))
            f.close()
    NUM_CLASSES = max_obj_target + max_skel_target
    
    

    # #print out some very useful statistics.
    print len(examples),'examples read'
    return examples




def getSegmentFeatureXMat(aid,sf,ef,lastSegment):
    global NUM_CLASSES_OBJ
    global NUM_CLASSES_SKEL
    global NODEONLY
    global Features, IntegralFeatures, Labeling, num_obj_feats,num_skel_feats,num_obj_t_feats,num_skel_t_feats,num_obj_obj_feats,num_obj_skel_feats
    global Binning
    max_obj_target = NUM_CLASSES_OBJ
    max_skel_target = NUM_CLASSES_SKEL
    (obj_feats,skel_feats,obj_obj_feats,skel_obj_feats) = getSegmentFeatures(aid,sf,ef,Binning)
    #writeSkelFeatures(aid, sf, skel_feats)
    #writeObjFeatures(aid, sf, obj_feats)
    #writeSkelObjFeatures(aid,sf,skel_obj_feats)
    #writeObjObjFeatures(aid,sf,obj_obj_feats)

    # first line has the number of nodes and number of edges
    N1 = len(obj_feats)
    N2 = 1
    if(NODEONLY == "true"):
        N2 =  0
    E1 = sum(len(v) for v in obj_obj_feats.itervalues());
    E2 = len(skel_obj_feats);

    xrow = zeros(N1*max_obj_target*num_obj_feats + N2*max_skel_target*num_skel_feats  + E1*max_obj_target*max_obj_target*num_obj_obj_feats + E2*max_obj_target*max_skel_target*num_obj_skel_feats )
    xcol = zeros(N1*max_obj_target*num_obj_feats + N2*max_skel_target*num_skel_feats  + E1*max_obj_target*max_obj_target*num_obj_obj_feats + E2*max_obj_target*max_skel_target*num_obj_skel_feats)
    xval = zeros(N1*max_obj_target*num_obj_feats + N2*max_skel_target*num_skel_feats  + E1*max_obj_target*max_obj_target*num_obj_obj_feats + E2*max_obj_target*max_skel_target*num_obj_skel_feats)
    #Xn= mat(zeros((max_target * num_node_feats,max_target*N)));


    obj_map = {}
    i = -1
    if aid in objMap:
      for oid in objMap[aid]:
        i+=1

        # get the segment number
        obj_map[oid]= i
        # Get the features.
        features = mat(obj_feats[oid])
        #print features
        f = features.transpose()
        # fill in the Xn matrix
        for j in xrange(0,max_obj_target):
            #print i , j , num_obj_feats, f.size
            xval[(i*max_obj_target+j)*num_obj_feats:(i*max_obj_target+j+1)*num_obj_feats] = features.A[0];
            for v in xrange(0,num_obj_feats):
                xrow[(i*max_obj_target+j)*num_obj_feats+v] = j*num_obj_feats+v
            xcol[(i*max_obj_target+j)*num_obj_feats:(i*max_obj_target+j+1)*num_obj_feats] = i*max_obj_target+(j)
    #skel nodes
    for i in xrange(0, N2):

        # assumes all features are present
        features = mat(skel_feats) #mat([float(v) for k,v in tokens])
        if (lastSegment ==1):
            features = mat(zeros((len(skel_feats))))
        #print features
        f = features.transpose()

        # fill in the Xn matrix
        for j in xrange(0,max_skel_target):
            #print i , j , num_skel_feats, f.size
            xval[N1*max_obj_target*num_obj_feats+(i*max_skel_target+j)*num_skel_feats:N1*max_obj_target*num_obj_feats+(i*max_skel_target+j+1)*num_skel_feats] = features.A[0];
            for v in xrange(0,num_skel_feats):
                xrow[N1*max_obj_target*num_obj_feats+(i*max_skel_target+j)*num_skel_feats+v] = max_obj_target*num_obj_feats + j*num_skel_feats+v
            xcol[N1*max_obj_target*num_obj_feats + (i*max_skel_target+j)*num_skel_feats : N1*max_obj_target*num_obj_feats + (i*max_skel_target+j+1)*num_skel_feats] = N1*max_obj_target + i*max_skel_target+(j)

    edges_obj_obj = mat(zeros((E1,2)))
    #print E1,max_obj_target*max_obj_target*E1

    i = N1+N2-1
    if aid in objMap:
      for o1 in objMap[aid]:
        for o2 in objMap[aid]:
            if o1 == o2:
                continue
            i+=1

            # get the segment numbers
            edges_obj_obj[i-(N1+N2),0]= obj_map[o1]
            edges_obj_obj[i-(N1+N2),1]= obj_map[o2]

            features = obj_obj_feats[o1][o2]
    #print features
            f = features.transpose()

    # fill in the Xn matrix
            S = N1*max_obj_target*num_obj_feats + N2*max_skel_target*num_skel_feats
            for j in xrange(0,max_obj_target*max_obj_target):
                xval[S + ((i-(N1+N2))*max_obj_target*max_obj_target+j)*num_obj_obj_feats: S + ((i-(N1+N2))*max_obj_target*max_obj_target+(j+1))*num_obj_obj_feats] = features.copy();
                for v in xrange(0,num_obj_obj_feats):
                    xrow[S+((i-(N1+N2))*max_obj_target*max_obj_target+j)*num_obj_obj_feats+v] = max_obj_target*num_obj_feats + max_skel_target*num_skel_feats + j*num_obj_obj_feats+v
                xcol[S + ((i-(N1+N2))*max_obj_target*max_obj_target+j)*num_obj_obj_feats: S + ((i-(N1+N2))*max_obj_target*max_obj_target+(j+1))*num_obj_obj_feats] = N1*max_obj_target + N2*max_skel_target +(i-(N1+N2))*max_obj_target*max_obj_target+j

    edges_obj_skel = mat(zeros((E2,2)))

    for i in xrange(N1+N2+E1,N1+N2+E1+E2):
        oid = objMap[aid][i-(N1+N2+E1)]

        # get the segment numbers
        edges_obj_skel[i-(N1+N2+E1),0]= obj_map[oid]
        edges_obj_skel[i-(N1+N2+E1),1]= 0 #obj_map[int(input[i+1][3])]


        features = skel_obj_feats[oid]
        if (lastSegment ==1):
            features = mat(zeros((len(skel_obj_feats[oid]))))
        
    #print features
        f = features.transpose()

        # fill in the Xn matrix
        S = N1*max_obj_target*num_obj_feats + N2*max_skel_target*num_skel_feats + E1*max_obj_target*max_obj_target*num_obj_obj_feats
        for j in xrange(0,max_obj_target*max_skel_target):
        ##print X[j*9:(j+1)*9,j];
        #Xe[j*num_edge_feats:(j+1)*num_edge_feats,(i-N)*max_target*max_target+j] = f.copy();
            xval[S + ((i-(N1+N2+E1))*max_obj_target*max_skel_target+j)*num_obj_skel_feats: S + ((i-(N1+N2+E1))*max_obj_target*max_skel_target+(j+1))*num_obj_skel_feats] = features.copy();
            for v in xrange(0,num_obj_skel_feats):
                xrow[S+((i-(N1+N2+E1))*max_obj_target*max_skel_target+j)*num_obj_skel_feats+v] = max_obj_target*num_obj_feats + max_skel_target*num_skel_feats + max_obj_target*max_obj_target*num_obj_obj_feats + j*num_obj_skel_feats+v
            xcol[S + ((i-(N1+N2+E1))*max_obj_target*max_skel_target+j)*num_obj_skel_feats: S + ((i-(N1+N2+E1))*max_obj_target*max_skel_target+(j+1))*num_obj_skel_feats] = N1*max_obj_target + N2*max_skel_target + E1*max_obj_target*max_obj_target +(i-(N1+N2+E1))*max_obj_target*max_skel_target+j


    X_sparse = coo_matrix((xval,(xrow,xcol)),shape=(num_obj_feats*max_obj_target+ num_skel_feats*max_skel_target +num_obj_obj_feats*max_obj_target*max_obj_target+ num_obj_skel_feats*max_obj_target*max_skel_target,N1*max_obj_target+N2*max_skel_target+(E1*max_obj_target*max_obj_target)+(E2*max_obj_target*max_skel_target) ),dtype='d');

    return (X_sparse,N1,E1,E2,obj_map,edges_obj_obj,edges_obj_skel)

def getSegmentFeatureYMat(aid,N1,E1,E2,labels,lastSegment):

    global NUM_CLASSES_OBJ
    global NUM_CLASSES_SKEL
    global NODEONLY
    global Features, IntegralFeatures, Labeling, num_obj_feats,num_skel_feats,num_obj_t_feats,num_skel_t_feats,num_obj_obj_feats,num_obj_skel_feats
    global affmap, classmap
    max_obj_target = NUM_CLASSES_OBJ
    max_skel_target = NUM_CLASSES_SKEL
    #print "segment:",segment

    N2 = 1
    if(NODEONLY == "true"):
        N2 =  0

    #FN = segment+1;
    Yn= mat(zeros((max_obj_target*N1+max_skel_target*N2,1)))

#XY_test = mat(zeros((max_target*num_node_feats + max_target*max_target*num_edge_feats,1)))

    i = -1
    #print objMap[aid]
    if aid in objMap:
      for oid in objMap[aid]:
        i+=1
        className =  labels[int(oid)]

        target = className;
        if(target != -1): # if the segment is unlabeled all entries of Yn for the label will be 0
            #print target
            #class_counts[target-1]+=1
            Yn[i*max_obj_target+(target-1),0]=1


    #skel nodes
    for i in xrange(0, N2):
        target = labels[0];
        #print target
        if(target != -1): # if the segment is unlabeled all entries of Yn for the label will be 0
            Yn[N1*max_obj_target + i*max_skel_target+(target-1),0]=1




    Ye1 = mat(zeros((max_obj_target*max_obj_target*E1,1)))
    i = N1+N2-1
    if aid in objMap:
      for o1 in objMap[aid]:
        for o2 in objMap[aid]:
            if o1 == o2:
                continue
            i+=1
            target1 = labels[int(o1)]
            target2 = labels[int(o2)]
            if(target1 != -1 and target2 != -1): # both should be labeled to add to Ye
                #print 'here',i, (i-(N1+N2))*max_obj_target*max_obj_target, (target1-1)*max_obj_target, target2-1
                Ye1[(i-(N1+N2))*max_obj_target*max_obj_target + (target1-1)*max_obj_target+(target2-1)]=1



    Ye2 = mat(zeros((max_obj_target*max_skel_target*E2,1)))
    for i in xrange(N1+N2+E1,N1+N2+E1+E2):
        oid = objMap[aid][i-(N1+N2+E1)]
        target1 = labels[int(oid)];
        target2 = labels[0];
        if(target1 != -1 and target2 != -1): # both should be labeled to add to Ye
        #print target1 , target2
            Ye2[(i-(N1+N2+E1))*max_obj_target*max_skel_target + (target1-1)*max_skel_target+(target2-1)]=1



    Y = concatenate ((Yn,Ye1,Ye2))
    return Y


def getTemporalFeatureXMat(aid,sf,mf,ef,lastSegment):
    global NUM_CLASSES_OBJ
    global NUM_CLASSES_SKEL
    global NODEONLY
    global Features, IntegralFeatures, Labeling, num_obj_feats,num_skel_feats,num_obj_t_feats,num_skel_t_feats,num_obj_obj_feats,num_obj_skel_feats
    global affmap, classmap
    global Binning
    max_obj_target = NUM_CLASSES_OBJ
    max_skel_target = NUM_CLASSES_SKEL

    #input = [line.split() for line in line_reader(file(edge_file.strip()))]
    # first line has the number of nodes and number of edges
    E3 = 0
    if aid in objMap:
        E3 = len(objMap[aid]) #int(input[0][0].strip()); # number of objects

    E4 =  1 #int(input[0][1].strip());
    if(NODEONLY == "true"):
        E4 =  0

    xrow = zeros( E3*max_obj_target*max_obj_target*num_obj_t_feats + E4*max_skel_target*max_skel_target*num_skel_t_feats)
    xcol = zeros( E3*max_obj_target*max_obj_target*num_obj_t_feats + E4*max_skel_target*max_skel_target*num_skel_t_feats)
    xval = zeros( E3*max_obj_target*max_obj_target*num_obj_t_feats + E4*max_skel_target*max_skel_target*num_skel_t_feats)
    #Xn= mat(zeros((max_target * num_node_feats,max_target*N)));

    (obj_feats, skel_feats) = getTemporalFeatures(aid,sf,mf,ef,Binning)
    #writeTObjFeatures(aid, sf, obj_feats)
    #writeTSkelFeatures(aid, sf, skel_feats)
    #edges_skel_temporal = mat(zeros((E4,1)))
    i = -1
    if aid in objMap:
      for objId in objMap[aid]:
        i += 1
        #tokens = [line.split(':') for line in input[i+1][3:]]
        features = mat(obj_feats[objId])
        #print features
        f = features.transpose()

        # fill in the Xn matrix
        for j in xrange(0,max_obj_target*max_obj_target):
            xval[((i*max_obj_target*max_obj_target)+j)*num_obj_t_feats: (i*max_obj_target*max_obj_target+(j+1))*num_obj_t_feats] = features.copy();
            for v in xrange(0,num_obj_t_feats):
                xrow[(i*max_obj_target*max_obj_target+j)*num_obj_t_feats+v] =   j*num_obj_t_feats+v
            xcol[ (i*max_obj_target*max_obj_target+j)*num_obj_t_feats:  (i*max_obj_target*max_obj_target+(j+1))*num_obj_t_feats] = i*max_obj_target*max_obj_target+j
    for i in xrange(E3,E3+E4):

        features = mat(skel_feats)
        if (lastSegment ==1):
            features = mat(zeros((len(skel_feats))))
        #print features
        f = features.transpose()

        # fill in the Xn matrix
        S = E3*max_obj_target*max_obj_target*num_obj_t_feats
        for j in xrange(0,max_skel_target*max_skel_target):
            xval[S + (((i-E3)*max_skel_target*max_skel_target)+j)*num_skel_t_feats: S +((i-E3)*max_skel_target*max_skel_target+(j+1))*num_skel_t_feats] = features.copy();
            for v in xrange(0,num_skel_t_feats):
                xrow[S + ((i-E3)*max_skel_target*max_skel_target+j)*num_skel_t_feats+v] = max_obj_target*max_obj_target*num_obj_t_feats + j*num_skel_t_feats+v
            xcol[ S+ ((i-E3)*max_skel_target*max_skel_target+j)*num_skel_t_feats:  S + ((i-E3)*max_skel_target*max_skel_target+(j+1))*num_skel_t_feats] =  E3*max_obj_target*max_obj_target + (i-E3)*max_skel_target*max_skel_target+j

    X_sparse = coo_matrix((xval,(xrow,xcol)),shape=(num_obj_t_feats*max_obj_target*max_obj_target+ num_skel_t_feats*max_skel_target*max_skel_target , E3*max_obj_target*max_obj_target +E4*max_skel_target*max_skel_target ),dtype='d');
    return X_sparse

def getTemporalFeatureYMat(aid,label1,label2,lastSegment):
    global NUM_CLASSES_OBJ
    global NUM_CLASSES_SKEL
    global NODEONLY
    global Features, IntegralFeatures, Labeling, num_obj_feats,num_skel_feats,num_obj_t_feats,num_skel_t_feats,num_obj_obj_feats,num_obj_skel_feats
    global affmap, classmap
    max_obj_target = NUM_CLASSES_OBJ
    max_skel_target = NUM_CLASSES_SKEL

    #input = [line.split() for line in line_reader(file(edge_file.strip()))]
    # first line has the number of nodes and number of edges
    E3 =  0
    if aid in objMap:
        E3=len(objMap[aid]) #int(input[0][0].strip()); # number of objects
    E4 =  1 #int(input[0][1].strip());
    #FN1 = link #int(input[0][2].strip());
    #FN2 = link+1 #int(input[0][3].strip());
    #Xn= mat(zeros((max_target * num_node_feats,max_target*N)));
    Ye3 = mat(zeros((max_obj_target*max_obj_target*E3,1)))
    Ye4 = mat(zeros((max_skel_target*max_skel_target*E4,1)))

    i = -1
    if aid in objMap:
      for objId in objMap[aid]:
        i += 1
        target1 = label1[int(objId)];
        target2 = label2[int(objId)];
        if(target1 != -1 and target2 != -1): # both should be labeled to add to Ye
            Ye3[i*max_obj_target*max_obj_target + (target1-1)*max_obj_target+(target2-1)]=1

    for i in xrange(E3,E3+E4):
        target1 = label1[0];
        target2 = label2[0];
        #if(target1 == target2):
            #print "same label!!", target1
        if(target1 != -1 and target2 != -1): # both should be labeled to add to Ye
            Ye4[(i-E3)*max_skel_target*max_skel_target + (target1-1)*max_skel_target+(target2-1)]=1

    Y = concatenate ((Ye3,Ye4))
    return Y


def readFile(filename):
    # Helper function for reading from files.
    def line_reader(lines):
        # returns only non-empty lines
        # strips comments (anything after '#')
        for l in lines:
            i = l.find('#')
            if i == -1: yield l.strip()
    data = [line.split(',') for line in line_reader(file(filename))]
    return data

def readLabelingFile(filename):
    global NULL_FILTERED,PATH
    readAffmap( '../affmap.txt')
    if NULL_FILTERED == "true":
        readClassmap( '../classmap_filtered.txt')
    else:
        readClassmap( '../classmap.txt')
    data = readFile(filename)
    labeling = {}
    segmentation = {}
    for i in xrange(0,len(data)):
        #print data[i]
        if data[i][0] not in labeling:
            labeling[data[i][0]] =  {}
            segmentation[data[i][0]] = []
        #segmentation[data[i][0]].append((int(data[i][1])-1,int(data[i][2])-1))
        #if(int(data[i][1]) ==1 ):
         #   data[i][1] = 2
        segmentation[data[i][0]].append((int(data[i][1]),int(data[i][2])))
        labeling[data[i][0]][len(segmentation[data[i][0]])] = [classmap[data[i][3]]]
        for j in xrange(4,len(data[i])):
            #print data[i][0],len(segmentation[data[i][0]]),data[i][j],affmap[data[i][j]]
            labeling[data[i][0]][len(segmentation[data[i][0]])].append(affmap[data[i][j]])
    return (segmentation,labeling)

def getObjMap(objfeats):
    global objMap
    objMap = {}
    for aid in objfeats:
        objMap[aid] = []
        for oid in objfeats[aid]:
            objMap[aid].append(oid)

def readFrameFeatures(datapath):
    global NULL_FILTERED


    #obj_feats_tmp = readFile(datapath+'/data_obj_feats_test_actonly.txt')
    if NULL_FILTERED == "true":
        filename= datapath+'data_obj_feats_filtered.txt'
    else:
        filename= datapath+'data_obj_feats.txt'
    obj_feats_tmp = readFile(filename)
    obj_feats = {}
    obj_temporal_feats = {}
    skel_feats = {}
    skel_temporal_feats = {}
    skel_obj_feats = {}
    obj_obj_feats = {}
    for i in xrange(0,len(obj_feats_tmp)):
        #print obj_feats_tmp[i][0]
        if obj_feats_tmp[i][0] not in obj_feats:
            obj_feats[obj_feats_tmp[i][0]] = {}
        if obj_feats_tmp[i][2] not in obj_feats[obj_feats_tmp[i][0]]:
            obj_feats[obj_feats_tmp[i][0]][obj_feats_tmp[i][2]] = []
        obj_feats[obj_feats_tmp[i][0]][obj_feats_tmp[i][2]].append(array([float(v) for v in obj_feats_tmp[i][5:]]))
    if NULL_FILTERED == "true":
        filename= datapath+'data_skel_feats_filtered.txt'
    else:
        filename= datapath+'data_skel_feats.txt'
    skel_feats_tmp = readFile(filename)
    
    for i in xrange(0,len(skel_feats_tmp)):
        #print skel_feats_tmp[i][0]
        if skel_feats_tmp[i][0] not in skel_feats:
            skel_feats[skel_feats_tmp[i][0]] = []
        skel_feats[skel_feats_tmp[i][0]].append(array([float(v) for v in skel_feats_tmp[i][2:]]))
    #obj_obj_feats_tmp = readFile(datapath+'/data_obj_obj_feats.txt')
    #obj_skel_feats_tmp = readFile(datapath+'/data_skel_obj_feats.txt')
    #obj_temporal_feats_tmp = readFile(datapath+'/data_temporal_obj_feats_test_actonly.txt')
    if NULL_FILTERED == "true":
        filename= datapath+'data_temporal_obj_feats_filtered.txt'
    else:
        filename= datapath+'data_temporal_obj_feats.txt'
    obj_temporal_feats_tmp = readFile(filename)
    for i in xrange(0,len(obj_temporal_feats_tmp)):
        #print obj_temporal_feats_tmp[i][0]
        if obj_temporal_feats_tmp[i][0] not in obj_temporal_feats:
            obj_temporal_feats[obj_temporal_feats_tmp[i][0]] = {}
        if obj_temporal_feats_tmp[i][3] not in obj_temporal_feats[obj_temporal_feats_tmp[i][0]]:
            obj_temporal_feats[obj_temporal_feats_tmp[i][0]][obj_temporal_feats_tmp[i][3]] = []
        obj_temporal_feats[obj_temporal_feats_tmp[i][0]][obj_temporal_feats_tmp[i][3]].append(array([float(v) for v in obj_temporal_feats_tmp[i][4:]]))
    if NULL_FILTERED == "true":
        filename= datapath+'data_temporal_skel_feats_filtered.txt'
    else:
        filename= datapath+'data_temporal_skel_feats.txt'
    skel_temporal_feats_tmp = readFile(filename)
    for i in xrange(0,len(skel_temporal_feats_tmp)):
        #print skel_temporal_feats_tmp[i][0]
        if skel_temporal_feats_tmp[i][0] not in skel_temporal_feats:
            skel_temporal_feats[skel_temporal_feats_tmp[i][0]] = []
        skel_temporal_feats[skel_temporal_feats_tmp[i][0]].append(array([float(v) for v in skel_temporal_feats_tmp[i][3:]]))
    if NULL_FILTERED == "true":
        filename= datapath+'data_skel_obj_feats_filtered.txt'
    else:
        filename= datapath+'data_skel_obj_feats.txt'
    skel_obj_feats_tmp =  readFile(filename)
    for i in xrange(0, len(skel_obj_feats_tmp)):
        if skel_obj_feats_tmp[i][0] not in skel_obj_feats:
            skel_obj_feats[skel_obj_feats_tmp[i][0]] = {}
        if skel_obj_feats_tmp[i][2] not in skel_obj_feats[skel_obj_feats_tmp[i][0]]:
            skel_obj_feats[skel_obj_feats_tmp[i][0]][skel_obj_feats_tmp[i][2]] = []
        skel_obj_feats[skel_obj_feats_tmp[i][0]][skel_obj_feats_tmp[i][2]].append(array([float(v) for v in skel_obj_feats_tmp[i][3:]]))
    if NULL_FILTERED == "true":
        filename= datapath+'data_obj_obj_feats_filtered.txt'
    else:
        filename= datapath+'data_obj_obj_feats.txt'
    obj_obj_feats_tmp = readFile(filename)
    for i in xrange(0, len(obj_obj_feats_tmp)):
        if obj_obj_feats_tmp[i][0] not in obj_obj_feats:
            obj_obj_feats[obj_obj_feats_tmp[i][0]] = {}
        if obj_obj_feats_tmp[i][2] not in obj_obj_feats[obj_obj_feats_tmp[i][0]]:
            obj_obj_feats[obj_obj_feats_tmp[i][0]][obj_obj_feats_tmp[i][2]] = {}
        if obj_obj_feats_tmp[i][3] not in obj_obj_feats[obj_obj_feats_tmp[i][0]][obj_obj_feats_tmp[i][2]]:
            obj_obj_feats[obj_obj_feats_tmp[i][0]][obj_obj_feats_tmp[i][2]][obj_obj_feats_tmp[i][3]] = []
        obj_obj_feats[obj_obj_feats_tmp[i][0]][obj_obj_feats_tmp[i][2]][obj_obj_feats_tmp[i][3]].append(array([float(v) for v in obj_obj_feats_tmp[i][4:]]))
    #for k in obj_feats:
        #print k, obj_feats[k]['1'][0]
    #return (obj_feats,skel_feats,obj_obj_feats, obj_skel_feats, obj_temporal_feats, skel_temporal_feats)
    getObjMap(obj_feats)
    return (obj_feats,skel_feats,obj_temporal_feats,skel_temporal_feats,skel_obj_feats,obj_obj_feats)


'''def getIntegralFrame(feats, feat_list):
    integral_frame = {}
    for findex in feat_list:
        integral_frame[findex] = [float(feats[0][findex])]
        for i in  xrange(1,len(feats)):
            val = integral_frame[findex][i-1]+float(feats[i][findex])
            integral_frame[findex].append(val)
    return integral_frame
'''
def getIntegralFrame(feats, feat_list):
    integral_frame = zeros((len(feats),len(feat_list)))
    #print integral_frame[0,:]
    #print feats[0][ix_(feat_list)]
    integral_frame[0,:] = feats[0][ix_(feat_list)]
    for i in  xrange(1,len(feats)):
        integral_frame[i,:] = integral_frame[i-1,:]+feats[i][ix_(feat_list)]

    return integral_frame

def getAllIntegralFrames(features):
    global PARTIAL
    objIntegralFrames = {}
    for aid in features[0]:
        objIntegralFrames[aid] = {}
        for oid in features[0][aid]:
            objIntegralFrames[aid][oid] = getIntegralFrame(features[2][aid][oid],[0,1,2,3])
    skelIntegralFrames = {}
    skelLocalIntegralFrames = {}
    for aid in features[1]:
        #print len(features[3][aid][0])
        if(PARTIAL == "true"):
          skelIntegralFrames[aid] = getIntegralFrame(features[3][aid],[0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27])
        else:
          skelIntegralFrames[aid] = getIntegralFrame(features[3][aid],[0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31])
    # removing the local features
    '''for aid in features[1]:
        #print len(features[3][aid][0])
        if(PARTIAL == "true"):
          skelLocalIntegralFrames[aid] = getIntegralFrame(features[3][aid],[28,29,30,31,32,33,34,35,36,37,38,39,40,41,42,43,44,45,46,47,48,49,50,51,52,53,54,55])
        else:
          skelLocalIntegralFrames[aid] = getIntegralFrame(features[3][aid],[32,33,34,35,36,37,38,39,40,41,42,43,44,45,46,47,48,49,50,51,52,53,54,55,56,57,58,59,60,61,62,63])
    '''
    return (objIntegralFrames, skelIntegralFrames,skelLocalIntegralFrames)



def printIFrame(IFrame):
    for i in xrange(0,len(IFrame)):
        print i,IFrame[i]

def getObjSegmentFeatures(aid,objId,startFrame,endFrame):
    global NON_ADDITIVE

    obj_sel_feats = Features[0][aid][objId][startFrame+int((endFrame-startFrame+1)/2)]
    
    segLen = endFrame - startFrame +1
    if startFrame == 0:
        obj_add_feats = (IntegralFeatures[0][aid][objId][endFrame-1])*1/segLen
    else:
    #print 'obj feats:',len(Features[0][aid][objId]),len(IntegralFeatures[0][aid][objId])
        obj_add_feats = (IntegralFeatures[0][aid][objId][endFrame-1] - IntegralFeatures[0][aid][objId][startFrame-1])*1/segLen

    #check the dim of the array
    #---------------------------
    dim_num = obj_sel_feats.ndim
    if(dim_num < 2):
        feats =  concatenate((obj_sel_feats,obj_add_feats))
    else:
        feats =  concatenate((obj_sel_feats,obj_add_feats),1)
    #---------------------------
    #feats =  concatenate((obj_sel_feats,obj_add_feats),1)

    if(NON_ADDITIVE == "true"):
        #print shape(Features[0][aid][objId][startFrame+int((endFrame-startFrame)/2)])
        dist =  array([math.pow(obj_add_feats[0],2) + math.pow(obj_add_feats[1],2)+ math.pow(obj_add_feats[2],2)])
        dist = dist*segLen

        #check the dim of the array
        #---------------------------
        dim_num = feats.ndim
        if(dim_num < 2):
            feats = concatenate((feats,dist))
        else:
            feats = concatenate((feats,dist),1)
        #---------------------------
        #feats = concatenate((feats,dist),1)

    return feats


def getSkelSegmentFeatures(aid,startFrame,endFrame):
    global NON_ADDITIVE, PARTIAL
    skel_sel_feats = Features[1][aid][startFrame+int((endFrame-startFrame+1)/2)]
    
    segLen = endFrame - startFrame +1
    if startFrame == 0:
        skel_add_feats = (IntegralFeatures[1][aid][endFrame-1])*1/segLen
    else:
        #print 'skel feats:',len(Features[1][aid]),len(IntegralFeatures[1][aid])
        
        print "seglen", endFrame,startFrame, segLen
        print len(IntegralFeatures[1][aid])
        skel_add_feats = (IntegralFeatures[1][aid][endFrame-1] - IntegralFeatures[1][aid][startFrame-1])*1/segLen
    
    #check the dim of the array
    #---------------------------
    dim_num = skel_sel_feats.ndim
    if(dim_num < 2):
        feats = concatenate((skel_sel_feats,skel_add_feats))
    else:
        feats = concatenate((skel_sel_feats,skel_add_feats),1)
    #---------------------------
    #feats = concatenate((skel_sel_feats,skel_add_feats),1)

    #print shape(skel_add_feats)
    if(NON_ADDITIVE == "true"):
        #print shape(Features[0][aid][objId][startFrame+int((endFrame-startFrame)/2)])
        dist = zeros((7))
        if PARTIAL != "true":
            dist = zeros((8))
        dist[0] = array([math.pow(skel_add_feats[0],2) + math.pow(skel_add_feats[1],2)+ math.pow(skel_add_feats[2],2)])
        dist[1] = array([math.pow(skel_add_feats[4],2) + math.pow(skel_add_feats[5],2)+ math.pow(skel_add_feats[6],2)])
        dist[2] = array([math.pow(skel_add_feats[8],2) + math.pow(skel_add_feats[9],2)+ math.pow(skel_add_feats[10],2)])
        dist[3] = array([math.pow(skel_add_feats[12],2) + math.pow(skel_add_feats[13],2)+ math.pow(skel_add_feats[14],2)])
        dist[4] = array([math.pow(skel_add_feats[16],2) + math.pow(skel_add_feats[17],2)+ math.pow(skel_add_feats[18],2)])
        dist[5] = array([math.pow(skel_add_feats[20],2) + math.pow(skel_add_feats[21],2)+ math.pow(skel_add_feats[22],2)])
        dist[6] = array([math.pow(skel_add_feats[24],2) + math.pow(skel_add_feats[25],2)+ math.pow(skel_add_feats[26],2)])
        if PARTIAL != "true":
            dist[7] = array([math.pow(skel_add_feats[28],2) + math.pow(skel_add_feats[29],2)+ math.pow(skel_add_feats[30],2)])
        #print dist
        dist = dist*segLen

        #check the dim of the array
        #---------------------------
        dim_num = feats.ndim
        if(dim_num < 2):
            feats = concatenate((feats,dist))
        else:
            feats = concatenate((feats,dist),1)
        #---------------------------
        #feats = concatenate((feats,dist),1)

    return feats

def getObjObjSegmentFeatures(aid,objId1,objId2,startFrame,endFrame):
    global NON_ADDITIVE
    segLen = endFrame - startFrame +1.0
    if startFrame == 0:
        midFrame = int(endFrame/2)
        
    else:
        midFrame = startFrame+ int((endFrame-startFrame)/2)
    obj_obj_sel_feats_s = Features[5][aid][objId1][objId2][startFrame]
    obj_obj_sel_feats_e = Features[5][aid][objId1][objId2][endFrame]
    obj_obj_sel_feats_m = Features[5][aid][objId1][objId2][midFrame]

    #check the dim of the array
    #---------------------------
    dim_num = obj_obj_sel_feats_m.ndim
    if(dim_num < 2):
        t =  concatenate((obj_obj_sel_feats_m,obj_obj_sel_feats_e))
    else:
        t =  concatenate((obj_obj_sel_feats_m,obj_obj_sel_feats_e),1)
    #---------------------------
    #t =  concatenate((obj_obj_sel_feats_m,obj_obj_sel_feats_e),1)

    #check the dim of the array
    #---------------------------
    dim_num = obj_obj_sel_feats_s.ndim
    if(dim_num < 2):
        t = concatenate((obj_obj_sel_feats_s,t))
    else:
        t = concatenate((obj_obj_sel_feats_s,t),1)
    #---------------------------
    #t = concatenate((obj_obj_sel_feats_s,t),1)

    if NON_ADDITIVE == "true":
        feats = array(Features[5][aid][objId1][objId2][startFrame:endFrame+1])
        max_feats = feats.max(0)
        max_feats = max_feats*(1/segLen)
        min_feats = feats.min(0)
        min_feats = min_feats*(1/segLen)

        #check the dim of the array
        #---------------------------
        dim_num = t.ndim
        if(dim_num < 2):
            t = concatenate((t,min_feats))
            t = concatenate((t,max_feats))
        else:
            t = concatenate((t,min_feats),1)
            t = concatenate((t,max_feats),1)
        #---------------------------
        #t = concatenate((t,min_feats),1)
        #t = concatenate((t,max_feats),1)
        
    return t
    #return array([1])


def getSkelObjSegmentFeatures(aid,objId,startFrame,endFrame):
    global NON_ADDITIVE
    #print "sf", startFrame, " ef " , endFrame
    segLen = endFrame - startFrame +1.0
    if startFrame == 0:
        midFrame = int(endFrame/2)
        
    else:
        midFrame = startFrame+ int((endFrame-startFrame)/2)
    skel_obj_sel_feats_s = Features[4][aid][objId][startFrame]
    skel_obj_sel_feats_e = Features[4][aid][objId][endFrame]
    skel_obj_sel_feats_m = Features[4][aid][objId][midFrame]

    #check the dim of the array
    #---------------------------
    dim_num = skel_obj_sel_feats_m.ndim
    if(dim_num < 2):
        t =  concatenate((skel_obj_sel_feats_m,skel_obj_sel_feats_e))
    else:
        t =  concatenate((skel_obj_sel_feats_m,skel_obj_sel_feats_e),1)
    #---------------------------
    #t =  concatenate((skel_obj_sel_feats_m,skel_obj_sel_feats_e),1)

    #check the dim of the array
    #---------------------------
    dim_num = skel_obj_sel_feats_s.ndim
    if(dim_num < 2):
        t = concatenate((skel_obj_sel_feats_s,t))
    else:
        t = concatenate((skel_obj_sel_feats_s,t),1)
    #---------------------------
    #t = concatenate((skel_obj_sel_feats_s,t),1)

    if NON_ADDITIVE == "true":
        print startFrame,endFrame
        feats = array(Features[4][aid][objId][startFrame:endFrame+1])
        max_feats = feats.max(0)
        max_feats = max_feats*(1/segLen)
        min_feats = feats.min(0)
        min_feats = min_feats*(1/segLen)

        #check the dim of the array
        #---------------------------
        dim_num = t.ndim
        if(dim_num < 2):
            t = concatenate((t,min_feats))
            t = concatenate((t,max_feats))
        else:
            t = concatenate((t,min_feats),1)
            t = concatenate((t,max_feats),1)
        #---------------------------
        #t = concatenate((t,min_feats),1)
        #t = concatenate((t,max_feats),1)
        
    return t
    #return array([1])

def getTemporalFeatures(aid,sf,mf,ef,binning):
    global Features,skelTFeats_binTh,objTFeats_binTh
    obj_feats = {}

    skel_feats = getSkelTemporalFeatures(aid, sf, mf, ef)
    if(binning):
        skel_feats = getBinnedFeatures(skel_feats,skelTFeats_binTh)
    if aid in objMap:
        for objId in objMap[aid]:
            obj_feats[objId] = getObjTemporalFeatures(aid, objId, sf,mf ,ef)
            if(binning):
                obj_feats[objId] = getBinnedFeatures(obj_feats[objId],objTFeats_binTh)
    return (obj_feats, skel_feats)

def getSkelTemporalFeatures(aid, sf, mf, ef):
    global NON_ADDITIVE 
    global PARTIAL 
    m1 = int(sf + (mf-sf)/2)
    m2 = int(mf +1 + (ef-mf -1)/2)
    if(m2>ef):
      m2=ef
    if NON_ADDITIVE == "true":

      #skel_add_feats = IntegralFeatures[2][aid][m2-1] - IntegralFeatures[2][aid][m1-1] ## CHECK: have changed to transformed features
      skel_add_feats = IntegralFeatures[1][aid][m2-1] - IntegralFeatures[1][aid][m1-1]
      dist = zeros((7))
      if PARTIAL != "true":
          dist = zeros((8))
      dist[0] = array([math.pow(skel_add_feats[0],2) + math.pow(skel_add_feats[1],2)+ math.pow(skel_add_feats[2],2)])
      dist[1] = array([math.pow(skel_add_feats[4],2) + math.pow(skel_add_feats[5],2)+ math.pow(skel_add_feats[6],2)])
      dist[2] = array([math.pow(skel_add_feats[8],2) + math.pow(skel_add_feats[9],2)+ math.pow(skel_add_feats[10],2)])
      dist[3] = array([math.pow(skel_add_feats[12],2) + math.pow(skel_add_feats[13],2)+ math.pow(skel_add_feats[14],2)])
      dist[4] = array([math.pow(skel_add_feats[16],2) + math.pow(skel_add_feats[17],2)+ math.pow(skel_add_feats[18],2)])
      dist[5] = array([math.pow(skel_add_feats[20],2) + math.pow(skel_add_feats[21],2)+ math.pow(skel_add_feats[22],2)])
      dist[6] = array([math.pow(skel_add_feats[24],2) + math.pow(skel_add_feats[25],2)+ math.pow(skel_add_feats[26],2)])
      if PARTIAL != "true":
          dist[7] = array([math.pow(skel_add_feats[28],2) + math.pow(skel_add_feats[29],2)+ math.pow(skel_add_feats[30],2)])
      tmp = dist/(m2-m1)

      #check the dim of the array
      #---------------------------
      dim_num = dist.ndim
      if(dim_num < 2):
          return concatenate((dist,tmp))
      else:
          return concatenate((dist,tmp),1)
      #---------------------------
      #return concatenate((dist,tmp),1)

    else :
      if PARTIAL == "true":
        feat_list=[3,7,11,15,19,23,27]
      else:
        feat_list=[3,7,11,15,19,23,27,31]
      skel_feats = IntegralFeatures[1][aid][m2][ix_(feat_list)] - IntegralFeatures[1][aid][m1][ix_(feat_list)]
      tmp = skel_feats/(m2-m1)

      #check the dim of the array
      #---------------------------
      dim_num = skel_feats.ndim
      if(dim_num < 2):
          return concatenate((skel_feats,tmp))
      else:
          return concatenate((skel_feats,tmp),1)
      #---------------------------
      #return concatenate((skel_feats,tmp),1)
 


def getObjTemporalFeatures(aid, objId, sf,mf ,ef):
    global NON_ADDITIVE
    m1 = int(sf + (mf-sf)/2)
    m2 = int(mf +1 + (ef-mf -1)/2)
    if(m2>ef):
      m2=ef
    if NON_ADDITIVE == "true":
      feat_list=[2]
      obj_feats = IntegralFeatures[0][aid][objId][m2-1][ix_(feat_list)] - IntegralFeatures[0][aid][objId][m1-1][ix_(feat_list)]
      tmp = obj_feats/(m2-m1)

      #check the dim of the array
      #---------------------------
      dim_num = obj_feats.ndim
      if(dim_num < 2):
          obj_feats = concatenate((obj_feats,tmp))
      else:
          obj_feats = concatenate((obj_feats,tmp),1)
      #---------------------------
      #obj_feats = concatenate((obj_feats,tmp),1)

      obj_add_feats = IntegralFeatures[0][aid][objId][m2-1] - IntegralFeatures[0][aid][objId][m1-1]
      #print shape(Features[0][aid][objId][startFrame+int((endFrame-startFrame)/2)])
      dist =  array([math.pow(obj_add_feats[0],2) + math.pow(obj_add_feats[1],2)+ math.pow(obj_add_feats[2],2)])
      dist_norm = dist/(m2-m1)

      #check the dim of the array
      #---------------------------
      dim_num = dist.ndim
      if(dim_num < 2):
          dist = concatenate((dist,dist_norm))
          return concatenate((obj_feats,dist))
      else:
          dist = concatenate((dist,dist_norm),1)
          return concatenate((obj_feats,dist),1)
      #---------------------------
      #dist = concatenate((dist,dist_norm),1)
      #return concatenate((obj_feats,dist),1)

    else:
      feat_list=[2,3]
      obj_feats = IntegralFeatures[0][aid][objId][m2-1][ix_(feat_list)] - IntegralFeatures[0][aid][objId][m1-1][ix_(feat_list)]
      tmp = obj_feats/(m2-m1+1)

      #check the dim of the array
      #---------------------------
      dim_num = obj_feats.ndim
      if(dim_num < 2):
          return concatenate((obj_feats,tmp))
      else:
          return concatenate((obj_feats,tmp),1)
      #---------------------------
      #return concatenate((obj_feats,tmp),1)
    #return array([1])

def getSegmentFeatures(aid,seg):
    global Features, Labeling
    obj_feats = {}
    obj_obj_feats = {}
    skel_obj_feats = {}
    global Labeling
    sf = int(Labeling[0][aid][seg][0])
    ef = int(Labeling[0][aid][seg][1])
    #print sf,ef
    skel_feats = getSkelSegmentFeatures(aid, sf, ef)
    #tmp = [`x+1`+':'+`skel_feats[x]` for x in xrange(0,len(skel_feats))]
    #print "skel feats:", seg, tmp
    if aid in objMap:
        for objId in objMap[aid]:
            obj_feats[objId] = getObjSegmentFeatures(aid, objId, sf, ef)
            tmp = [`x+1`+':'+`obj_feats[objId][x]` for x in xrange(0,len(obj_feats[objId]))]
            #print "obj feats:", seg,objId, tmp
        for objId in objMap[aid]:
            skel_obj_feats[objId] = getSkelObjSegmentFeatures(aid, objId, sf, ef)
        for objId1 in objMap[aid]:
            obj_obj_feats[objId1] = {}
            for objId2 in objMap[aid]:
                if objId1 != objId2:
                    obj_obj_feats[objId1][objId2] = getObjObjSegmentFeatures(aid, objId, sf, ef)        

    return (obj_feats, skel_feats, obj_obj_feats, skel_obj_feats)


def getBinnedFeatures(feats,binTh):
    global numBin
    new_feats = zeros((len(feats)*numBin))
    index = 0
    #print feats
    for i in xrange(0,len(feats)):
        for j in xrange(0,numBin):

            new_feats[index] = (feats[i]<=float(binTh[i][j]))
            index+=1
    #print binTh
    #print new_feats
    return new_feats

def getSegmentFeatures(aid,sf, ef, binning):
    global Features,skelFeats_binTh,skelTFeats_binTh,objFeats_binTh,objTFeats_binTh,skelObjFeats_binTh,objObjFeats_binTh
    obj_feats = {}
    obj_obj_feats = {}
    skel_obj_feats = {}
    

    skel_feats = getSkelSegmentFeatures(aid, sf, ef)
    if(binning):
        skel_feats = getBinnedFeatures(skel_feats,skelFeats_binTh)
    if aid in objMap:
        for objId in objMap[aid]:
            obj_feats[objId] = getObjSegmentFeatures(aid, objId, sf, ef)
            if(binning):
                obj_feats[objId] = getBinnedFeatures(obj_feats[objId],objFeats_binTh)
        for objId in objMap[aid]:
            skel_obj_feats[objId] = getSkelObjSegmentFeatures(aid, objId, sf, ef)
            if(binning):
                skel_obj_feats[objId] = getBinnedFeatures(skel_obj_feats[objId] ,skelObjFeats_binTh)
        for objId1 in objMap[aid]:
            obj_obj_feats[objId1] = {}
            for objId2 in objMap[aid]:
                if objId1 != objId2:
                    obj_obj_feats[objId1][objId2] = getObjObjSegmentFeatures(aid, objId1, objId2, sf, ef)
                    if(binning):
                        obj_obj_feats[objId1][objId2] = getBinnedFeatures(obj_obj_feats[objId1][objId2] ,objObjFeats_binTh)


    return (obj_feats, skel_feats, obj_obj_feats, skel_obj_feats)




'''def getSegmentFeatures(aid,seg):
    global Features, Labeling
    obj_feats = {}
    obj_obj_feats = {}
    skel_obj_feats = {}
    global Labeling
    sf = int(Labeling[0][aid][seg][0])-1
    ef = int(Labeling[0][aid][seg][1])-1
    #print sf,ef
    skel_feats = getSkelSegmentFeatures(aid, sf, ef)
   
    for objId in objMap[aid]:
        obj_feats[objId] = getObjSegmentFeatures(aid, objId, sf, ef)
        
    for objId in objMap[aid]:
        skel_obj_feats[objId] = getSkelObjSegmentFeatures(aid, objId, sf, ef)
    for objId1 in objMap[aid]:
        obj_obj_feats[objId1] = {}
        for objId2 in objMap[aid]:
            if objId1 != objId2:
                obj_obj_feats[objId1][objId2] = getObjObjSegmentFeatures(aid, objId, sf, ef)



    return (obj_feats, skel_feats, obj_obj_feats, skel_obj_feats)'''



def read_examples_multiple_frames(filename,sparm):
    global TEMPORAL
    global NUM_CLASSES
    global NUM_CLASSES_OBJ
    global NUM_CLASSES_SKEL
    global NODEONLY
     # Helper function for reading from files.
    def line_reader(lines):
        # returns only non-empty lines
        # strips comments (anything after '#')
        for l in lines:
            i = l.find('#')
            if i == -1: yield l.strip()


    """Parses an input file into an example sequence."""
    # This reads example files of the type read by SVM^multiclass.
    examples = []
    max_target=0
    num_obj_feats=0
    num_skel_feats=0
    num_obj_t_feats=0
    num_skel_t_feats=0
    num_obj_obj_feats=0
    num_obj_skel_feats=0


    start = time.clock()

    # Open the file and read each example.
    for input_file in file(filename):
        files = [line.split() for line in line_reader(file(input_file.strip()))]
        print files
        print input_file
        for frame_num in xrange(1,int(files[0][0].strip())):
            frame_file = files[frame_num][0].strip()
            print frame_file
            input = [line.split() for line in line_reader(file(frame_file.strip()))]
        # first line has the number of nodes and number of edges
            N1 = int(input[0][0].strip())
            N2 = 1
            if(NODEONLY == "true"):
                N2 =  0
            E1 = int(input[0][1].strip())
            E2 = int(input[0][2].strip())
            K1 = int(input[0][3].strip())
            K2 = int(input[0][4].strip())

            # find the max class and number of node features -- will work for sparse representation
            for i in xrange(0,N1):
                tokens = [line.split(':') for line in input[i+1][2:]]
                for k,v in tokens:
                    if(num_obj_feats<int(k)):
                        num_obj_feats=int(k)
            for i in xrange(N1, N1+N2):
                tokens = [line.split(':') for line in input[i+1][2:]]
                for k,v in tokens:
                    if(num_skel_feats<int(k)):
                        num_skel_feats=int(k)
            for i in xrange(N1+N2, N1+N2+E1):
                tokens = [line.split(':') for line in input[i+1][4:]]
                for k,v in tokens:
                    if(num_obj_obj_feats<int(k)):
                        num_obj_obj_feats=int(k)
            for i in xrange(N1+N2+E1,N1+N2+E1+E2):
                tokens = [line.split(':') for line in input[i+1][3:]]
                for k,v in tokens:
                    if(num_obj_skel_feats<int(k)):
                        num_obj_skel_feats=int(k)
        if(TEMPORAL == "true"):
            for edge_num in xrange(1+int(files[0][0].strip()),1+int(files[0][0].strip())+int(files[0][1].strip())):
                edge_file = files[edge_num][0].strip()
                print edge_file
                input = [line.split() for line in line_reader(file(edge_file.strip()))]
                E3 = int(input[0][0].strip())
                E4 = int(input[0][1].strip())
                for i in xrange(0,E3):
                    tokens = [line.split(':') for line in input[i+1][3:]]
                    for k,v in tokens:
                        if(num_obj_t_feats<int(k)):
                            num_obj_t_feats=int(k)
                for i in xrange(E3,E3+E4):
                    tokens = [line.split(':') for line in input[i+1][3:]]
                    for k,v in tokens:
                        if(num_skel_t_feats<int(k)):
                            num_skel_t_feats=int(k)

    max_obj_target = K1 # use the max number of classes read from the file
    max_skel_target = K2
    NUM_CLASSES_OBJ = K1
    NUM_CLASSES_SKEL = K2

    print 'number of obj classes: ', max_obj_target
    print 'number of skel classes: ', max_skel_target

    print 'number of obj features: ', num_obj_feats
    print 'number of skel features: ', num_skel_feats
    print 'number of obj obj edge features: ',num_obj_obj_feats
    print 'number of obj skel edge features: ',num_obj_skel_feats
    print 'number of obj temporal edge features: ',num_obj_t_feats
    print 'number of obj temporal edge features: ',num_skel_t_feats

    print 'time spent in preprocessing data files', start-time.clock()


    class_counts = zeros(K1+K2)
    frame_count = 0
    temporal_edge_count = 0
    X_multiple = []
    Y_multiple = []
    X_temporal = []
    Y_temporal = []
    edges_obj_obj_list = []
    edges_obj_skel_list = []
    edges_obj_temporal_list = []
    temporal_frames_list = []
    N1_list = []
    E_list = []
    obj_map_list = []
    frame_map = {}
    example_num=-1
    for input_file in file(filename):
        files = [line.split() for line in line_reader(file(input_file.strip()))]
        print input_file
        X_multiple = []
        Y_multiple = []
        X_temporal = []
        Y_temporal = []
        frame_count = 0
        frame_map = {}
        obj_map_list = []
        edges_obj_obj_list = []
        edges_obj_skel_list = []
        N1_list = []
        E_list = []
        example_num+=1
        for frame_num in xrange(1,int(files[0][0].strip())+1):
            frame_file = files[frame_num][0].strip()
            print frame_file
            frame_count +=1
            input = [line.split() for line in line_reader(file(frame_file.strip()))]
            # first line has the number of nodes and number of edges
            N1 = int(input[0][0].strip());
            N2 = 1
            if(NODEONLY == "true"):
                N2 =  0
            E1 = int(input[0][1].strip());
            E2 = int(input[0][2].strip());
            FN = int(input[0][5].strip());
            xrow = zeros(N1*max_obj_target*num_obj_feats + N2*max_skel_target*num_skel_feats  + E1*max_obj_target*max_obj_target*num_obj_obj_feats + E2*max_obj_target*max_skel_target*num_obj_skel_feats )
            xcol = zeros(N1*max_obj_target*num_obj_feats + N2*max_skel_target*num_skel_feats  + E1*max_obj_target*max_obj_target*num_obj_obj_feats + E2*max_obj_target*max_skel_target*num_obj_skel_feats)
            xval = zeros(N1*max_obj_target*num_obj_feats + N2*max_skel_target*num_skel_feats  + E1*max_obj_target*max_obj_target*num_obj_obj_feats + E2*max_obj_target*max_skel_target*num_obj_skel_feats)
            #Xn= mat(zeros((max_target * num_node_feats,max_target*N)));
            Yn= mat(zeros((max_obj_target*N1+max_skel_target*N2,1)))

        #XY_test = mat(zeros((max_target*num_node_feats + max_target*max_target*num_edge_feats,1)))
            obj_map = {}

            for i in xrange(0,N1):
                target = int(input[i+1][0]);
                if(target != -1): # if the segment is unlabeled all entries of Yn for the label will be 0
                    #print target
                    class_counts[target-1]+=1
                    Yn[i*max_obj_target+(target-1),0]=1

                # get the segment number
                obj_map[int(input[i+1][1])] = i

                # Get the features.
                tokens = [line.split(':') for line in input[i+1][2:]]

                # assumes all features are present
                features = mat([float(v) for k,v in tokens])
                #print features
                f = features.transpose()

                # fill in the Xn matrix
                for j in xrange(0,max_obj_target):
                    xval[(i*max_obj_target+j)*num_obj_feats:(i*max_obj_target+j+1)*num_obj_feats] = features.A[0];
                    for v in xrange(0,num_obj_feats):
                        xrow[(i*max_obj_target+j)*num_obj_feats+v] = j*num_obj_feats+v
                    xcol[(i*max_obj_target+j)*num_obj_feats:(i*max_obj_target+j+1)*num_obj_feats] = i*max_obj_target+(j)
            #skel nodes
            for i in xrange(0, N2):
                target = int(input[N1+i+1][0]);
                print target
                if(target != -1): # if the segment is unlabeled all entries of Yn for the label will be 0
                    Yn[N1*max_obj_target + i*max_skel_target+(target-1),0]=1

                   # Get the features.
                tokens = [line.split(':') for line in input[N1+i+1][2:]]

                # assumes all features are present
                features = mat([float(v) for k,v in tokens])
                #print features
                f = features.transpose()

                # fill in the Xn matrix
                for j in xrange(0,max_skel_target):
                    xval[N1*max_obj_target*num_obj_feats+(i*max_skel_target+j)*num_skel_feats:N1*max_obj_target*num_obj_feats+(i*max_skel_target+j+1)*num_skel_feats] = features.A[0];
                    for v in xrange(0,num_skel_feats):
                        xrow[N1*max_obj_target*num_obj_feats+(i*max_skel_target+j)*num_skel_feats+v] = max_obj_target*num_obj_feats + j*num_skel_feats+v
                    xcol[N1*max_obj_target*num_obj_feats + (i*max_skel_target+j)*num_skel_feats : N1*max_obj_target*num_obj_feats + (i*max_skel_target+j+1)*num_skel_feats] = N1*max_obj_target + i*max_skel_target+(j)

            edges_obj_obj = mat(zeros((E1,2)))
            Ye1 = mat(zeros((max_obj_target*max_obj_target*E1,1)))
            for i in xrange(N1+N2,N1+N2+E1):
                target1 = int(input[i+1][0]);
                target2 = int(input[i+1][1]);
                if(target1 != -1 and target2 != -1): # both should be labeled to add to Ye
                    Ye1[(i-(N1+N2))*max_obj_target*max_obj_target + (target1-1)*max_obj_target+(target2-1)]=1
                # get the segment numbers
                edges_obj_obj[i-(N1+N2),0]= obj_map[int(input[i+1][2])]
                edges_obj_obj[i-(N1+N2),1]= obj_map[int(input[i+1][3])]

                tokens = [line.split(':') for line in input[i+1][4:]]
                features = mat([float(v) for k,v in tokens])
            #print features
                f = features.transpose()

            # fill in the Xn matrix
                S = N1*max_obj_target*num_obj_feats + N2*max_skel_target*num_skel_feats
                for j in xrange(0,max_obj_target*max_obj_target):
                    xval[S + ((i-(N1+N2))*max_obj_target*max_obj_target+j)*num_obj_obj_feats: S + ((i-(N1+N2))*max_obj_target*max_obj_target+(j+1))*num_obj_obj_feats] = features.copy();
                    for v in xrange(0,num_obj_obj_feats):
                        xrow[S+((i-(N1+N2))*max_obj_target*max_obj_target+j)*num_obj_obj_feats+v] = max_obj_target*num_obj_feats + max_skel_target*num_skel_feats + j*num_obj_obj_feats+v
                    xcol[S + ((i-(N1+N2))*max_obj_target*max_obj_target+j)*num_obj_obj_feats: S + ((i-(N1+N2))*max_obj_target*max_obj_target+(j+1))*num_obj_obj_feats] = N1*max_obj_target + N2*max_skel_target +(i-(N1+N2))*max_obj_target*max_obj_target+j

            edges_obj_skel = mat(zeros((E2,2)))
            Ye2 = mat(zeros((max_obj_target*max_skel_target*E2,1)))
            for i in xrange(N1+N2+E1,N1+N2+E1+E2):
                target1 = int(input[i+1][0]);
                target2 = int(input[i+1][1]);
                if(target1 != -1 and target2 != -1): # both should be labeled to add to Ye
                #print target1 , target2
                    Ye2[(i-(N1+N2+E1))*max_obj_target*max_skel_target + (target1-1)*max_skel_target+(target2-1)]=1
                # get the segment numbers
                edges_obj_skel[i-(N1+N2+E1),0]= obj_map[int(input[i+1][2])]
                edges_obj_skel[i-(N1+N2+E1),1]= 0 #obj_map[int(input[i+1][3])]

                tokens = [line.split(':') for line in input[i+1][3:]]
                features = mat([float(v) for k,v in tokens])
            #print features
                f = features.transpose()

                # fill in the Xn matrix
                S = N1*max_obj_target*num_obj_feats + N2*max_skel_target*num_skel_feats + E1*max_obj_target*max_obj_target*num_obj_obj_feats
                for j in xrange(0,max_obj_target*max_skel_target):
                ##print X[j*9:(j+1)*9,j];
                #Xe[j*num_edge_feats:(j+1)*num_edge_feats,(i-N)*max_target*max_target+j] = f.copy();
                    xval[S + ((i-(N1+N2+E1))*max_obj_target*max_skel_target+j)*num_obj_skel_feats: S + ((i-(N1+N2+E1))*max_obj_target*max_skel_target+(j+1))*num_obj_skel_feats] = features.copy();
                    for v in xrange(0,num_obj_skel_feats):
                        xrow[S+((i-(N1+N2+E1))*max_obj_target*max_skel_target+j)*num_obj_skel_feats+v] = max_obj_target*num_obj_feats + max_skel_target*num_skel_feats + max_obj_target*max_obj_target*num_obj_obj_feats + j*num_obj_skel_feats+v
                    xcol[S + ((i-(N1+N2+E1))*max_obj_target*max_skel_target+j)*num_obj_skel_feats: S + ((i-(N1+N2+E1))*max_obj_target*max_skel_target+(j+1))*num_obj_skel_feats] = N1*max_obj_target + N2*max_skel_target + E1*max_obj_target*max_obj_target +(i-(N1+N2+E1))*max_obj_target*max_skel_target+j


            X_sparse = coo_matrix((xval,(xrow,xcol)),shape=(num_obj_feats*max_obj_target+ num_skel_feats*max_skel_target +num_obj_obj_feats*max_obj_target*max_obj_target+ num_obj_skel_feats*max_obj_target*max_skel_target,N1*max_obj_target+N2*max_skel_target+(E1*max_obj_target*max_obj_target)+(E2*max_obj_target*max_skel_target) ),dtype='d');

            Y = concatenate ((Yn,Ye1,Ye2))
            #xd =  X_sparse.todense()

            #numpy.savetxt('test.txt',xd,delimiter=' ',fmt='%-2.1f')
            edges_obj_obj_list.append(edges_obj_obj)
            edges_obj_skel_list.append(edges_obj_skel)
            N1_list.append(N1)
            E_list.append((E1,E2))
            obj_map_list.append(obj_map)
            frame_map[FN] = frame_count-1 # 0 indexed
            print frame_map
            #print FN, obj_map

            Y_s = csr_matrix(Y,dtype='d')
            K = max_target
           ## print frame_count

            if (frame_count ==1 ):
                X_multiple = X_sparse
                Y_multiple = Y
            else:

                data = scipy.concatenate((X_multiple.data, X_sparse.data))
                row= scipy.concatenate((X_multiple.row, X_sparse.row))
                col = scipy.concatenate((X_multiple.col, X_sparse.col + X_multiple.shape[1]))
                X_multiple = coo_matrix((data,(row,col)), shape= (X_multiple.shape[0], X_multiple.shape[1] + X_sparse.shape[1]) )
                Y_multiple = concatenate((Y_multiple, Y))
        #print obj_map_list
        if(TEMPORAL == "true"):
            temporal_edge_count = 0
            temporal_frames_list = []
            edges_obj_temporal_list = []
            for edge_num in xrange(1+int(files[0][0].strip()),1+int(files[0][0].strip())+int(files[0][1].strip())):
                edge_file = files[edge_num][0].strip()
                print edge_file
                temporal_edge_count += 1
                input = [line.split() for line in line_reader(file(edge_file.strip()))]
                # first line has the number of nodes and number of edges
                E3 = int(input[0][0].strip());
                E4 = int(input[0][1].strip());
                FN1 = int(input[0][2].strip());
                FN2 = int(input[0][3].strip());
                temporal_frames_list.append((frame_map[FN1],frame_map[FN2]))
                xrow = zeros( E3*max_obj_target*max_obj_target*num_obj_t_feats + E4*max_skel_target*max_skel_target*num_skel_t_feats)
                xcol = zeros( E3*max_obj_target*max_obj_target*num_obj_t_feats + E4*max_skel_target*max_skel_target*num_skel_t_feats)
                xval = zeros( E3*max_obj_target*max_obj_target*num_obj_t_feats + E4*max_skel_target*max_skel_target*num_skel_t_feats)
                #Xn= mat(zeros((max_target * num_node_feats,max_target*N)));
                Ye3 = mat(zeros((max_obj_target*max_obj_target*E3,1)))
                Ye4 = mat(zeros((max_skel_target*max_skel_target*E4,1)))
                edges_obj_temporal = mat(zeros((E3,1)))
                #edges_skel_temporal = mat(zeros((E4,1)))
                for i in xrange(0,E3):
                    target1 = int(input[i+1][0]);
                    target2 = int(input[i+1][1]);
                    if(target1 != -1 and target2 != -1): # both should be labeled to add to Ye
                        Ye3[i*max_obj_target*max_obj_target + (target1-1)*max_obj_target+(target2-1)]=1
                    # get the segment numbers
                    #print frame_map[FN2],obj_map_list[frame_map[FN2]]
                    edges_obj_temporal[i,0]= obj_map_list[frame_map[FN2]][int(input[i+1][2])]
                    

                    tokens = [line.split(':') for line in input[i+1][3:]]
                    features = mat([float(v) for k,v in tokens])
                    #print features
                    f = features.transpose()

                    # fill in the Xn matrix
                    for j in xrange(0,max_obj_target*max_obj_target):
                        xval[((i*max_obj_target*max_obj_target)+j)*num_obj_t_feats: (i*max_obj_target*max_obj_target+(j+1))*num_obj_t_feats] = features.copy();
                        for v in xrange(0,num_obj_t_feats):
                            xrow[(i*max_obj_target*max_obj_target+j)*num_obj_t_feats+v] =   j*num_obj_t_feats+v
                        xcol[ (i*max_obj_target*max_obj_target+j)*num_obj_t_feats:  (i*max_obj_target*max_obj_target+(j+1))*num_obj_t_feats] = i*max_obj_target*max_obj_target+j
                for i in xrange(E3,E3+E4):
                    target1 = int(input[i+1][0]);
                    target2 = int(input[i+1][1]);
                    if(target1 != -1 and target2 != -1): # both should be labeled to add to Ye
                        Ye4[(i-E3)*max_skel_target*max_skel_target + (target1-1)*max_skel_target+(target2-1)]=1
                    
                    #edges_skel_temporal[i-E3,0]= 1 # only one subactivity node

                    tokens = [line.split(':') for line in input[i+1][3:]]
                    features = mat([float(v) for k,v in tokens])
                    #print features
                    f = features.transpose()

                    # fill in the Xn matrix
                    S = E3*max_obj_target*max_obj_target*num_obj_t_feats
                    for j in xrange(0,max_skel_target*max_skel_target):
                        xval[S + (((i-E3)*max_skel_target*max_skel_target)+j)*num_skel_t_feats: S +((i-E3)*max_skel_target*max_skel_target+(j+1))*num_skel_t_feats] = features.copy();
                        for v in xrange(0,num_skel_t_feats):
                            xrow[S + ((i-E3)*max_skel_target*max_skel_target+j)*num_skel_t_feats+v] = max_obj_target*max_obj_target*num_obj_t_feats + j*num_skel_t_feats+v
                        xcol[ S+ ((i-E3)*max_skel_target*max_skel_target+j)*num_skel_t_feats:  S + ((i-E3)*max_skel_target*max_skel_target+(j+1))*num_skel_t_feats] =  E3*max_obj_target*max_obj_target + (i-E3)*max_skel_target*max_skel_target+j

                X_sparse = coo_matrix((xval,(xrow,xcol)),shape=(num_obj_t_feats*max_obj_target*max_obj_target+ num_skel_t_feats*max_skel_target*max_skel_target , E3*max_obj_target*max_obj_target +E4*max_skel_target*max_skel_target ),dtype='d');
                Y = concatenate ((Ye3,Ye4))
                #xd =  X_sparse.todense()

                #numpy.savetxt('test.txt',xd,delimiter=' ',fmt='%-2.1f')
                if (temporal_edge_count ==1 ):
                    X_temporal = X_sparse
                    Y_temporal = Y
                else:
                    data = scipy.concatenate((X_temporal.data, X_sparse.data))
                    row= scipy.concatenate((X_temporal.row, X_sparse.row))
                    col = scipy.concatenate((X_temporal.col, X_sparse.col + X_temporal.shape[1]))
                    X_temporal = coo_matrix((data,(row,col)), shape= (X_temporal.shape[0], X_temporal.shape[1] + X_sparse.shape[1]) )
                    Y_temporal = concatenate((Y_temporal, Y))

                edges_obj_temporal_list.append(edges_obj_temporal)
            #edges_skel_temporal_list.append(edges_skel_temporal)
            if (temporal_edge_count != 0):
                Y_multiple = concatenate((Y_multiple, Y_temporal))
                data = scipy.concatenate((X_multiple.data, X_temporal.data))
                row= scipy.concatenate((X_multiple.row, X_temporal.row + X_multiple.shape[0]))
                col = scipy.concatenate((X_multiple.col, X_temporal.col + X_multiple.shape[1]))
                X_multiple = coo_matrix((data,(row,col)), shape= ( X_multiple.shape[0] + X_temporal.shape[0], X_multiple.shape[1] + X_temporal.shape[1]) )
            else:
                data = X_multiple.data
                row= X_multiple.row
                col = X_multiple.col
                X_multiple = coo_matrix((data,(row,col)), shape= ( X_multiple.shape[0] + num_obj_t_feats*max_obj_target*max_obj_target+ num_skel_t_feats*max_skel_target*max_skel_target  , X_multiple.shape[1]) )


        Y_m_s = csr_matrix(Y_multiple,dtype='d');
        X_multiple = X_multiple.tocsr()
        xd =  X_multiple.todense()
        numpy.savetxt('matrixX.txt',xd,delimiter=' ',fmt='%-2.1f')
        numpy.savetxt('matrixY.txt',Y_multiple,delimiter=' ',fmt='%-2.1f')
        print Y_m_s.shape
        print X_multiple.shape

        examples.append(((X_multiple, edges_obj_obj_list, edges_obj_skel_list, N1_list, E_list, frame_count, edges_obj_temporal_list, temporal_frames_list, temporal_edge_count,  example_num,obj_map_list ), (Y_m_s,N1_list,E_list,frame_count,temporal_edge_count,obj_map_list)))

    NUM_CLASSES = max_obj_target + max_skel_target

    # #print out some very useful statistics.
    print len(examples),'examples read'
    return examples

def read_examples_single_frame(filename,sparm):
    global NODEONLY
    global NUM_CLASSES
    global NUM_CLASSES_OBJ
    global NUM_CLASSES_SKEL

    # Helper function for reading from files.
    def line_reader(lines):
        # returns only non-empty lines
        # strips comments (anything after '#')
        for l in lines:
            i = l.find('#')
            if i == -1: yield l.strip()


    """Parses an input file into an example sequence."""
    # This reads example files of the type read by SVM^multiclass.
    examples = []
    max_target=0
    num_obj_feats=0
    num_skel_feats=0
    num_obj_obj_feats=0
    num_obj_skel_feats=0


    start = time.clock()

    # Open the file and read each example.
    for input_file in file(filename):
        print input_file
        input = [line.split() for line in line_reader(file(input_file.strip()))]
        # first line has the number of nodes and number of edges
        N1 = int(input[0][0].strip())
        N2 = 1
        if(NODEONLY == "true"):
            N2 =  0
        E1 = int(input[0][1].strip())
        E2 = int(input[0][2].strip())
        K1 = int(input[0][3].strip())
        K2 = int(input[0][4].strip())

        # find the max class and number of node features -- will work for sparse representation
        for i in xrange(0,N1):
            tokens = [line.split(':') for line in input[i+1][2:]]
            for k,v in tokens:
                if(num_obj_feats<int(k)):
                    num_obj_feats=int(k)
        for i in xrange(N1, N1+N2):
            tokens = [line.split(':') for line in input[i+1][2:]]
            for k,v in tokens:
                if(num_skel_feats<int(k)):
                    num_skel_feats=int(k)
        for i in xrange(N1+N2, N1+N2+E1):
            tokens = [line.split(':') for line in input[i+1][4:]]
            for k,v in tokens:
                if(num_obj_obj_feats<int(k)):
                    num_obj_obj_feats=int(k)
        for i in xrange(N1+N2+E1,N1+N2+E1+E2):
            tokens = [line.split(':') for line in input[i+1][3:]]
            for k,v in tokens:
                if(num_obj_skel_feats<int(k)):
                    num_obj_skel_feats=int(k)
    max_obj_target = K1 # use the max number of classes read from the file
    max_skel_target = K2
    NUM_CLASSES_OBJ = K1
    NUM_CLASSES_SKEL = K2

    print 'number of obj classes: ', max_obj_target
    print 'number of skel classes: ', max_skel_target

    print 'number of obj features: ', num_obj_feats
    print 'number of skel features: ', num_skel_feats
    print 'number of obj obj edge features: ',num_obj_obj_feats
    print 'number of obj skel edge features: ',num_obj_skel_feats

    print 'time spent in preprocessing data files', start-time.clock()


    class_counts = zeros(K1+K2)

    example_num=-1
    for input_file in file(filename):
        print input_file
        example_num+=1
        input = [line.split() for line in line_reader(file(input_file.strip()))]
        # first line has the number of nodes and number of edges
        N1 = int(input[0][0].strip());
        N2 = 1
        E1 = int(input[0][1].strip());
        E2 = int(input[0][2].strip());
        xrow = zeros(N1*max_obj_target*num_obj_feats + N2*max_skel_target*num_skel_feats  + E1*max_obj_target*max_obj_target*num_obj_obj_feats + E2*max_obj_target*max_skel_target*num_obj_skel_feats )
        xcol = zeros(N1*max_obj_target*num_obj_feats + N2*max_skel_target*num_skel_feats  + E1*max_obj_target*max_obj_target*num_obj_obj_feats + E2*max_obj_target*max_skel_target*num_obj_skel_feats)
        xval = zeros(N1*max_obj_target*num_obj_feats + N2*max_skel_target*num_skel_feats  + E1*max_obj_target*max_obj_target*num_obj_obj_feats + E2*max_obj_target*max_skel_target*num_obj_skel_feats)
        #Xn= mat(zeros((max_target * num_node_feats,max_target*N)));
        Yn= mat(zeros((max_obj_target*N1+max_skel_target*N2,1)))

        #XY_test = mat(zeros((max_target*num_node_feats + max_target*max_target*num_edge_feats,1)))
        obj_map = {}

        for i in xrange(0,N1):
            target = int(input[i+1][0]);
            if(target != -1): # if the segment is unlabeled all entries of Yn for the label will be 0
                #print target
                class_counts[target-1]+=1
                Yn[i*max_obj_target+(target-1),0]=1

            # get the segment number
            obj_map[int(input[i+1][1])] = i

            # Get the features.
            tokens = [line.split(':') for line in input[i+1][2:]]

            # assumes all features are present
            features = mat([float(v) for k,v in tokens])
            #print features
            f = features.transpose()

            # fill in the Xn matrix
            for j in xrange(0,max_obj_target):
                xval[(i*max_obj_target+j)*num_obj_feats:(i*max_obj_target+j+1)*num_obj_feats] = features.A[0];
                for v in xrange(0,num_obj_feats):
                  xrow[(i*max_obj_target+j)*num_obj_feats+v] = j*num_obj_feats+v
                xcol[(i*max_obj_target+j)*num_obj_feats:(i*max_obj_target+j+1)*num_obj_feats] = i*max_obj_target+(j)
        #skel nodes
        for i in xrange(0, N2):
            target = int(input[N1+i+1][0]);
            
            if(target != -1): # if the segment is unlabeled all entries of Yn for the label will be 0
                Yn[N1*max_obj_target + i*max_skel_target+(target-1),0]=1
           
            # Get the features.
            tokens = [line.split(':') for line in input[N1+i+1][2:]]

            # assumes all features are present
            features = mat([float(v) for k,v in tokens])
            #print features
            f = features.transpose()

            # fill in the Xn matrix
            for j in xrange(0,max_skel_target):
                xval[N1*max_obj_target*num_obj_feats+(i*max_skel_target+j)*num_skel_feats:N1*max_obj_target*num_obj_feats+(i*max_skel_target+j+1)*num_skel_feats] = features.A[0];
                for v in xrange(0,num_skel_feats):
                  xrow[N1*max_obj_target*num_obj_feats+(i*max_skel_target+j)*num_skel_feats+v] = max_obj_target*num_obj_feats + j*num_skel_feats+v
                xcol[N1*max_obj_target*num_obj_feats + (i*max_skel_target+j)*num_skel_feats : N1*max_obj_target*num_obj_feats + (i*max_skel_target+j+1)*num_skel_feats] = N1*max_obj_target + i*max_skel_target+(j)
            ##print X
        #Xe = mat(zeros((max_target*max_target*num_edge_feats,max_target*max_target*E)))
        edges_obj_obj = mat(zeros((E1,2)))
        Ye1 = mat(zeros((max_obj_target*max_obj_target*E1,1)))
        for i in xrange(N1+N2,N1+N2+E1):
            target1 = int(input[i+1][0]);
            target2 = int(input[i+1][1]);
            if(target1 != -1 and target2 != -1): # both should be labeled to add to Ye
                Ye1[(i-(N1+N2))*max_obj_target*max_obj_target + (target1-1)*max_obj_target+(target2-1)]=1
            # get the segment numbers
            edges_obj_obj[i-(N1+N2),0]= obj_map[int(input[i+1][2])]
            edges_obj_obj[i-(N1+N2),1]= obj_map[int(input[i+1][3])]

            tokens = [line.split(':') for line in input[i+1][4:]]
            features = mat([float(v) for k,v in tokens])
            #print features
            f = features.transpose()
            #z = max_target*num_obj_feats + (target1-1)*max_target*num_edge_feats+(target2-1)*num_edge_feats
            #XY_test[z : z+num_edge_feats ,0] += f
            # fill in the Xn matrix
            S = N1*max_obj_target*num_obj_feats + N2*max_skel_target*num_skel_feats
            for j in xrange(0,max_obj_target*max_obj_target):
                ##print X[j*9:(j+1)*9,j];
                #Xe[j*num_edge_feats:(j+1)*num_edge_feats,(i-N)*max_target*max_target+j] = f.copy();
                xval[S + ((i-(N1+N2))*max_obj_target*max_obj_target+j)*num_obj_obj_feats: S + ((i-(N1+N2))*max_obj_target*max_obj_target+(j+1))*num_obj_obj_feats] = features.copy();
                for v in xrange(0,num_obj_obj_feats):
                  xrow[S+((i-(N1+N2))*max_obj_target*max_obj_target+j)*num_obj_obj_feats+v] = max_obj_target*num_obj_feats + max_skel_target*num_skel_feats + j*num_obj_obj_feats+v
                xcol[S + ((i-(N1+N2))*max_obj_target*max_obj_target+j)*num_obj_obj_feats: S + ((i-(N1+N2))*max_obj_target*max_obj_target+(j+1))*num_obj_obj_feats] = N1*max_obj_target + N2*max_skel_target +(i-(N1+N2))*max_obj_target*max_obj_target+j
        #print Xn.shape[0], num_node_feats*K
        #print Xn.shape[1] , N*K
        #print Xe.shape[0], num_edge_feats*K*K
        #print Xe.shape[1] , E*K*K
        #print xval
        #print xrow
        #print xcol
        edges_obj_skel = mat(zeros((E2,2)))
        Ye2 = mat(zeros((max_obj_target*max_skel_target*E2,1)))
        for i in xrange(N1+N2+E1,N1+N2+E1+E2):
            target1 = int(input[i+1][0]);
            target2 = int(input[i+1][1]);
            if(target1 != -1 and target2 != -1): # both should be labeled to add to Ye
                #print target1 , target2
                Ye2[(i-(N1+N2+E1))*max_obj_target*max_skel_target + (target1-1)*max_skel_target+(target2-1)]=1
            # get the segment numbers
            edges_obj_skel[i-(N1+N2+E1),0]= obj_map[int(input[i+1][2])]
            edges_obj_skel[i-(N1+N2+E1),1]= 0 #obj_map[int(input[i+1][3])]

            tokens = [line.split(':') for line in input[i+1][3:]]
            features = mat([float(v) for k,v in tokens])
            #print features
            f = features.transpose()
           # z = max_target*num_node_feats + (target1-1)*max_target*num_edge_feats+(target2-1)*num_edge_feats
            #XY_test[z : z+num_edge_feats ,0] += f
            # fill in the Xn matrix
            S = N1*max_obj_target*num_obj_feats + N2*max_skel_target*num_skel_feats + E1*max_obj_target*max_obj_target*num_obj_obj_feats
            for j in xrange(0,max_obj_target*max_skel_target):
                ##print X[j*9:(j+1)*9,j];
                #Xe[j*num_edge_feats:(j+1)*num_edge_feats,(i-N)*max_target*max_target+j] = f.copy();
                xval[S + ((i-(N1+N2+E1))*max_obj_target*max_skel_target+j)*num_obj_skel_feats: S + ((i-(N1+N2+E1))*max_obj_target*max_skel_target+(j+1))*num_obj_skel_feats] = features.copy();
                for v in xrange(0,num_obj_skel_feats):
                  xrow[S+((i-(N1+N2+E1))*max_obj_target*max_skel_target+j)*num_obj_skel_feats+v] = max_obj_target*num_obj_feats + max_skel_target*num_skel_feats + max_obj_target*max_obj_target*num_obj_obj_feats + j*num_obj_skel_feats+v
                xcol[S + ((i-(N1+N2+E1))*max_obj_target*max_skel_target+j)*num_obj_skel_feats: S + ((i-(N1+N2+E1))*max_obj_target*max_skel_target+(j+1))*num_obj_skel_feats] = N1*max_obj_target + N2*max_skel_target + E1*max_obj_target*max_obj_target +(i-(N1+N2+E1))*max_obj_target*max_skel_target+j


        X_sparse = csr_matrix((xval,(xrow,xcol)),shape=(num_obj_feats*max_obj_target+ num_skel_feats*max_skel_target +num_obj_obj_feats*max_obj_target*max_obj_target+ num_obj_skel_feats*max_obj_target*max_skel_target,N1*max_obj_target+N2*max_skel_target+(E1*max_obj_target*max_obj_target)+(E2*max_obj_target*max_skel_target) ),dtype='d');
        #a = concatenate ((Xn, mat(zeros((Xn.shape[0],Xe.shape[1])))),1)
        #b = concatenate ((mat(zeros((Xe.shape[0],Xn.shape[1]))),Xe),1)
        #X = concatenate ((a,b))
        Y = concatenate ((Yn,Ye1,Ye2))
        #print Y
        xd =  X_sparse.todense()

        numpy.savetxt('test.txt',xd,delimiter=' ',fmt='%-2.1f')
        #X_s = csr_matrix(X,dtype='d')
        #print sum(X_s.todense() - X_sparse.todense())
        #assert (sum(X_s.todense() - X_sparse.todense()) == 0)
        #assert ((X_s - X_sparse).sum() == 0)

        Y_s = csr_matrix(Y,dtype='d')
        K = max_target
        examples.append(((X_sparse, edges_obj_obj, edges_obj_skel, N1, E1 ,example_num, obj_map ), (Y_s,N1,E1,obj_map)))

    NUM_CLASSES = max_obj_target + max_skel_target

    # #print out some very useful statistics.
    print len(examples),'examples read'
    return examples
    

def get_index(edges,u,v):
    for i in xrange(0,edges.shape[0]):
        if (edges[i,0] == u and edges[i,1] == v):
            return i
    assert(2 == 1) # should never reach here

def init_model(sample, sm, sparm):

    """Store the number of features and classes in the model."""
    # Note that these features will be stored in the model and written
    # when it comes time to write the model to a file, and restored in
    # the classifier when reading the model from the file.
    ##print sample[0][0].shape[0]
    global NUM_CLASSES
    #sm.num_features = sample[0][0][0].shape[0]
    sm.num_features = sample[0][0][0].get_shape()[0]
    sm.num_classes = NUM_CLASSES
    print 'num of classes: ', sm.num_classes
    sm.size_psi = sm.num_features
    print 'size_psi set to: ',sm.size_psi

thecount = 0







def lp_inference_sum1_IP(X,sm,sparm,LE):
    global NUM_CLASSES_OBJ
    global NUM_CLASSES_SKEL
    global NODEONLY
    start = time.clock()

    K1 = NUM_CLASSES_OBJ
    K2 = NUM_CLASSES_SKEL
    w = sm.w
    edges_obj_obj = X[1]
    edges_obj_skel = X[2]
    E1 = edges_obj_obj.shape[0]
    E2 = edges_obj_skel.shape[0]
    obj_map = X[6]
    N1 = X[3]
    N2 = 1
    if(NODEONLY == "true"):
        N2 =  0
    lp = glpk.LPX()        # Create empty problem instance
    lp.name = 'inference'     # Assign symbolic name to problem
    lp.obj.maximize = True # Set this as a maximization problem
    lp.cols.add(X[0].shape[1])         # Append three columns to this instance
    #lp.cols.add(X[0].get_shape()[1])         # Append three columns to this instance
    #count_t= 0
    for c in lp.cols:      # Iterate over all columns
        if (c.index < N1*K1):
            c.name = 'y_obj_%d_%d' % ( c.index/K1 , (c.index%K1)+1) # Name them y_obj_0_1, etc
            c.kind=int
            
        elif((c.index - N1*K1) < N2*K2) :
            index = c.index  - N1*K1
            c.name = 'y_skel_%d_%d' % ( index/K2 , (index%K2) + 1 ) # name them y_skel_0_1 etc
            c.kind = int
            
        elif((c.index - N1*K1 - N2*K2) < K1*K1*E1):
            index = c.index - N1*K1 - N2*K2
            c.name = 'y_%d-%d_%d-%d' % ( edges_obj_obj[int(index/(K1*K1)),0] ,edges_obj_obj[int(index/(K1*K1)),1] , int((index%(K1*K1))/K1)+1 , int((index%(K1*K1))%K1)+1)
            
        else :
            index = c.index - N1*K1 - N2*K2 - K1*K1*E1
            c.name = 'y_%d-%d_%d-%d' % ( edges_obj_skel[int(index/(K1*K2)),0] ,edges_obj_skel[int(index/(K1*K2)),1] , int((index%(K1*K2))/K2)+1 , int((index%(K1*K2))%K2)+1)
           
        c.bounds = 0.0, 1.0    # Set bound 0 <= xi <= 1
        #count_t +=1
    #print count_t,X[0].shape[1]
    x = X[0]
    #x = (X[0]).todense()
    w_list = [w[i] for i in xrange(0,x.shape[0])]
    w_mat = csr_matrix(asmatrix(array(w_list)),dtype='d')
    ##print w_list
    ##print (asarray(w*x)[0]).tolist()
    
    #print w_mat.transpose().shape[1];

    lp.obj[:] = (asarray((w_mat*x).todense())[0]).tolist() 
    ##print lp.obj[:]

    lp.rows.add(3*E1*K1*K1+3*E2*K1*K2+N1+N2)
    for r in lp.rows:      # Iterate over all rows
        r.name = 'p%d' %  r.index # Name them

    for i in xrange(0,2*E1*K1*K1):
        lp.rows[i].bounds = 0, None
    for i in xrange(2*E1*K1*K1,3*E1*K1*K1):
        lp.rows[i].bounds = None,1
    for i in xrange(3*E1*K1*K1,3*E1*K1*K1 +2*E2*K1*K2):
        lp.rows[i].bounds = 0, None
    for i in xrange(3*E1*K1*K1 + 2*E2*K1*K2, 3*E1*K1*K1 + 3*E2*K1*K2):
        lp.rows[i].bounds = None,1
    for i in xrange(3*E1*K1*K1 + 3*E2*K1*K2 ,3*E1*K1*K1 + 3*E2*K1*K2 + N1):
		if (LE == False) :
			lp.rows[i].bounds = 1,1  ##SUM = 1
		else:
			lp.rows[i].bounds = None,1  ## SUM = 1 is changed to SUM<= 1
    for i in xrange(3*E1*K1*K1 + 3*E2*K1*K2 +N1 ,3*E1*K1*K1 + 3*E2*K1*K2 + N1 + N2):
		if (LE == False) :
			lp.rows[i].bounds = 1,1  ##SUM = 1
		else:
			lp.rows[i].bounds = None,1  ## SUM = 1 is changed to SUM<= 1

    t = []
    for e in xrange(0,E1):
        u = edges_obj_obj[e,0]
        v = edges_obj_obj[e,1]
        n = -1
        for i in xrange(0,K1):
            for j in xrange(0,K1):
                n += 1
                a = int(u*K1 + i)
                b = int(v*K1 + j)
                c = N1*K1 + N2*K2 + e*K1*K1 + i*K1 + j
                ec = e*K1*K1 + n
                t.append((ec,a,1))
                t.append((ec,c,-1))
                ec += E1*K1*K1
                t.append((ec,b,1))
                t.append((ec,c,-1))
                ec += E1*K1*K1
                t.append((ec,a,1))
                t.append((ec,b,1))
                t.append((ec,c,-1))
    for e in xrange(0,E2):
        u = edges_obj_skel[e,0]
        v = edges_obj_skel[e,1]
        n = 3*E1*K1*K1 -1
        for i in xrange(0,K1):
            for j in xrange(0,K2):
                n += 1
                a = int(u*K1 + i)
                b = int(K1*N1+ v*K2 + j)
                c = N1*K1 + N2*K2 + E1*K1*K1 + e*K1*K2 + i*K2 + j
                ec = e*K1*K2 + n
                t.append((ec,a,1))
                t.append((ec,c,-1))
                ec += E2*K1*K2
                t.append((ec,b,1))
                t.append((ec,c,-1))
                ec += E2*K1*K2
                t.append((ec,a,1))
                t.append((ec,b,1))
                t.append((ec,c,-1))
    for e in xrange(0,N1):
        r = 3*E1*K1*K1 + 3*E2*K1*K2 +e
        for i in xrange(0,K1):
            c = e*K1+i
            t.append((r,c,1))
    for e in xrange(0,N2):
        r = 3*E1*K1*K1 + 3*E2*K1*K2 + N1 +e
        for i in xrange(0,K2):
            c = K1*N1+ e*K2+i
            t.append((r,c,1))
    

    ##print len(t)
    lp.matrix = t
    retval=lp.simplex();

    lpFin = time.clock()
    print "Time for LP:", (lpFin-start)

    assert retval == None
    labeling = asmatrix(array([c.primal for c in lp.cols]))
    for c in lp.cols:      # Iterate over all columns
        if (c.index < N1*K1 + N2*K2) :
            c.kind=int


    retval=lp.integer(tm_lim=300000)


    MIPFin = time.clock()
    print "Time for MIP:", (MIPFin-lpFin)

    assert retval == None or retval == "tmlim"
  #  #print 'Z = %g;' % lp.obj.value,  # Retrieve and #print obj func value
   # #print '; '.join('%s = %g' % (c.name, c.primal) for c in lp.cols)
                       # #print struct variable names and primal val
    if(retval == None):
        labeling = asmatrix(array([c.primal for c in lp.cols]))

    #print labeling.T
    #print labeling.shape
    ymax = (csr_matrix(labeling.T,dtype='d'),N1,E1,obj_map)
    #print ymax
    c1 = 0
    c0= 0
    ch =0
    cr = 0
    for c in lp.cols:
        if (c.primal == 1):
            c1 += 1
        elif(c.primal ==0):
            c0 += 1
        elif (c.primal == 0.5):
            ch += 1
        else:
            cr +=1
    #print 'number of 1s: %d' % c1
    #print 'number of 0s: %d' % c0
    #print 'number of 0.5s: %d' % ch
    #print 'number of 0s: %d' % cr
    #score = asarray((w_mat*x*ymax[0]).todense())[0][0];
    score2 = 0#sm.svm_model.classify(psi(x,ymax,sm,sparm))
    #print "objective value = ", round(lp.obj.value,2)
    #print '\n score : ' , round(score,2), ' score2: ',score2;
    #if(lp.obj.value  > 1.1):
    #  assert (round(lp.obj.value,2) ==  round(score,2))
    return ymax


def lp_inference_multiple_frames_sum1_IP(X,sm,sparm,LE):
    global NUM_CLASSES_OBJ
    global NUM_CLASSES_SKEL
    global NODEONLY
    start = time.clock()

    K1 = NUM_CLASSES_OBJ
    K2 = NUM_CLASSES_SKEL
    num_frames = X[5]
    num_temporal_edges = X[8]
    obj_map_list = X[10]
    

  #  print X[3],X[4]

    w = sm.w
    
    x = X[0]
    N1_list = X[3]
    E1_list = X[4]
    #x = (X[0]).todense()

    w_list = [w[i] for i in xrange(0,x.shape[0])]
    ##print w_list

    w_mat = csr_matrix(asmatrix(array(w_list)),dtype='d')
    ##print w_mat.shape , x.shape
    lp = glpk.LPX()        # Create empty problem instance
    lp.name = 'inference'     # Assign symbolic name to problem
    lp.obj.maximize = True # Set this as a maximization problem
    lp.cols.add(X[0].shape[1])         # Append three columns to this instance
    lp.obj[:] = (asarray((w_mat*x).todense())[0]).tolist()
    ##print lp.obj[:]
    num_rows = 0
    for f_index in xrange(0,num_frames):
        E1 = X[1][f_index].shape[0]
        E2 = X[2][f_index].shape[0]
        N1 = N1_list[f_index]
        N2 = 1
        if(NODEONLY == "true"):
            N2 =  0
        num_rows += 3*E1*K1*K1+3*E2*K1*K2+N1+N2

    lp.rows.add(num_rows)
    for r in lp.rows:      # Iterate over all rows
        r.name = 'p%d' %  r.index # Name them

    index_jump =0
    row_index_jump = 0
    t = []
    for f_index in xrange(0,num_frames):

        edges_obj_obj = X[1][f_index]
        edges_obj_skel = X[2][f_index]
        E1 = edges_obj_obj.shape[0]
        E2 = edges_obj_skel.shape[0]
        N1 = N1_list[f_index]
        N2 = 1
        if(NODEONLY == "true"):
            N2 =  0

        #lp.cols.add(X[0].get_shape()[1])         # Append three columns to this instance
        #count_t= 0
        for c in lp.cols:      # Iterate over all columns
            if (c.index < index_jump + N1*K1):
                index = c.index  - index_jump
                c.name = 'y_obj_%d_%d_%d' % ( f_index, index/K1 , (index%K1)+1) # Name them y_obj_0_1, etc
                c.kind=int

            elif((c.index - index_jump - N1*K1) < N2*K2) :
                index = c.index  - index_jump - N1*K1
                c.name = 'y_skel_%d_%d_%d' % ( f_index, index/K2 , (index%K2) + 1 ) # name them y_skel_0_1 etc
                c.kind = int

            elif((c.index - index_jump - N1*K1 - N2*K2) < K1*K1*E1):
                index = c.index - index_jump - N1*K1 - N2*K2
                c.name = 'y_%d_%d-%d_%d-%d' % ( f_index, edges_obj_obj[int(index/(K1*K1)),0] ,edges_obj_obj[int(index/(K1*K1)),1] , int((index%(K1*K1))/K1)+1 , int((index%(K1*K1))%K1)+1)

            elif((c.index - index_jump - N1*K1 - N2*K2 - K1*K1*E1) < K1*K2*E2) :
                index = c.index - index_jump - N1*K1 - N2*K2 - K1*K1*E1
                ##print index, K1, K2, index_jump
                c.name = 'y_%d_%d-%d_%d-%d' % ( f_index, edges_obj_skel[int(index/(K1*K2)),0] ,edges_obj_skel[int(index/(K1*K2)),1] , int((index%(K1*K2))/K2)+1 , int((index%(K1*K2))%K2)+1)

            c.bounds = 0.0, 1.0    # Set bound 0 <= xi <= 1
       
        for i in xrange(row_index_jump, row_index_jump + 2*E1*K1*K1):
            lp.rows[i].bounds = 0, None
        for i in xrange(row_index_jump + 2*E1*K1*K1, row_index_jump + 3*E1*K1*K1):
            lp.rows[i].bounds = None,1
        for i in xrange(row_index_jump + 3*E1*K1*K1, row_index_jump + 3*E1*K1*K1 +2*E2*K1*K2):
            lp.rows[i].bounds = 0, None
        for i in xrange(row_index_jump + 3*E1*K1*K1 + 2*E2*K1*K2, row_index_jump + 3*E1*K1*K1 + 3*E2*K1*K2):
            lp.rows[i].bounds = None,1
        for i in xrange(row_index_jump + 3*E1*K1*K1 + 3*E2*K1*K2 ,row_index_jump + 3*E1*K1*K1 + 3*E2*K1*K2 + N1):
            if (LE == False) :
                lp.rows[i].bounds = 1,1  ##SUM = 1
            else:
                lp.rows[i].bounds = None,1  ## SUM = 1 is changed to SUM<= 1
        for i in xrange(row_index_jump + 3*E1*K1*K1 + 3*E2*K1*K2 +N1 ,row_index_jump + 3*E1*K1*K1 + 3*E2*K1*K2 + N1 + N2):
            if (LE == False) :
                lp.rows[i].bounds = 1,1  ##SUM = 1
            else:
                lp.rows[i].bounds = None,1  ## SUM = 1 is changed to SUM<= 1


        for e in xrange(0,E1):
            u = edges_obj_obj[e,0]
            v = edges_obj_obj[e,1]
            n = -1
            for i in xrange(0,K1):
                for j in xrange(0,K1):
                    n += 1
                    a = int(u*K1 + i) + index_jump
                    b = int(v*K1 + j) + index_jump
                    c = N1*K1 + N2*K2 + e*K1*K1 + i*K1 + j + index_jump
                    ec = e*K1*K1 + n + row_index_jump
                    t.append((ec,a,1))
                    t.append((ec,c,-1))
                    ec += E1*K1*K1
                    t.append((ec,b,1))
                    t.append((ec,c,-1))
                    ec += E1*K1*K1
                    t.append((ec,a,1))
                    t.append((ec,b,1))
                    t.append((ec,c,-1))
        for e in xrange(0,E2):
            u = edges_obj_skel[e,0]
            v = edges_obj_skel[e,1]
            n = 3*E1*K1*K1 -1
            for i in xrange(0,K1):
                for j in xrange(0,K2):
                    n += 1
                    a = int(u*K1 + i) + index_jump
                    b = int(K1*N1+ v*K2 + j) + index_jump
                    c = N1*K1 + N2*K2 + E1*K1*K1 + e*K1*K2 + i*K2 + j + index_jump
                    ec = e*K1*K2 + n + row_index_jump
                    t.append((ec,a,1))
                    t.append((ec,c,-1))
                    ec += E2*K1*K2
                    t.append((ec,b,1))
                    t.append((ec,c,-1))
                    ec += E2*K1*K2
                    t.append((ec,a,1))
                    t.append((ec,b,1))
                    t.append((ec,c,-1))
        for e in xrange(0,N1):
            r = 3*E1*K1*K1 + 3*E2*K1*K2 +e + row_index_jump
            for i in xrange(0,K1):
                c = e*K1+i + index_jump
                t.append((r,c,1))
        for e in xrange(0,N2):
            r = 3*E1*K1*K1 + 3*E2*K1*K2 + N1 +e + row_index_jump
            for i in xrange(0,K2):
                c = K1*N1+ e*K2+i + index_jump
                t.append((r,c,1))

        row_index_jump += 3*E1*K1*K1+3*E2*K1*K2+N1+N2
        index_jump += N1*K1 + N2*K2 + K1*K1*E1 + K1*K2*E2

    ##print len(t)
    ##print t
    lp.matrix = t
    retval=lp.simplex();

    lpFin = time.clock()
    print "Time for LP:", (lpFin-start)

    assert retval == None
    labeling = asmatrix(array([c.primal for c in lp.cols]))
    ##print labeling
    index_jump =0
    for f_index in xrange(0,num_frames):
        E1 = X[1][f_index].shape[0]
        E2 = X[2][f_index].shape[0]
        N1 = X[3][f_index]
        N2 = 1
        if(NODEONLY == "true"):
            N2 =  0
        for c in lp.cols:      # Iterate over all columns
            if (c.index - index_jump < N1*K1 + N2*K2 and c.index > index_jump) :
                c.kind=int
        index_jump += N1*K1 + N2*K2 + K1*K1*E1 + K1*K2*E2

    retval=lp.integer(tm_lim=300000)


    MIPFin = time.clock()
    print "Time for MIP:", (MIPFin-lpFin)

    assert retval == None or retval == "tmlim"
  #  #print 'Z = %g;' % lp.obj.value,  # Retrieve and #print obj func value
   # #print '; '.join('%s = %g' % (c.name, c.primal) for c in lp.cols)
                       # #print struct variable names and primal val
    if(retval == None):
        labeling = asmatrix(array([c.primal for c in lp.cols]))

    #print labeling.T
    #print labeling.shape
   # ymax = (csr_matrix(labeling.T,dtype='d'),N1_list,E1_list,num_frames)
    ymax = (csr_matrix(labeling.T,dtype='d'),N1_list,E1_list,num_frames,num_temporal_edges, obj_map_list)

    #print ymax
    c1 = 0
    c0= 0
    ch =0
    cr = 0
    for c in lp.cols:
        if (c.primal == 1):
            c1 += 1
        elif(c.primal ==0):
            c0 += 1
        elif (c.primal == 0.5):
            ch += 1
        else:
            cr +=1
    #print 'number of 1s: %d' % c1
    #print 'number of 0s: %d' % c0
    #print 'number of 0.5s: %d' % ch
    #print 'number of 0s: %d' % cr
    #score = asarray((w_mat*x*ymax[0]).todense())[0][0];
    score2 = 0#sm.svm_model.classify(psi(x,ymax,sm,sparm))
    #print "objective value = ", round(lp.obj.value,2)
    #print '\n score : ' , round(score,2), ' score2: ',score2;
    #if(lp.obj.value  > 1.1):
    #  assert (round(lp.obj.value,2) ==  round(score,2))
    return ymax

def lp_inference_divmbest_temp(X,sm,sparm,LE):
    global NUM_CLASSES_OBJ
    global NUM_CLASSES_SKEL
    # Parameters:
    nummodes = 1
    lamb = 0.9

    #Get the example and prepare data
    K1 = NUM_CLASSES_OBJ
    K2 = NUM_CLASSES_SKEL
    num_frames = X[5]
    num_temporal_edges = X[8]
    obj_map_list = X[10]
    aid = X[11]

    w = sm.w
    x = X[0]
    N1_list = X[3]
    E1_list = X[4]
    temporal_frame_list = X[7]

    w_list = [w[i] for i in xrange(0,x.shape[0])]
    
    w_mat = csr_matrix(asmatrix(array(w_list)),dtype='d')
    ##print w_mat.shape , x.shape
    lp = glpk.LPX()        # Create empty problem instance
    lp.name = 'inference'     # Assign symbolic name to problem
    lp.obj.maximize = True # Set this as a maximization problem
    lp.cols.add(X[0].shape[1])         # Append columns to this instance
    lp.obj[:] = (asarray((w_mat*x).todense())[0]).tolist()


    ##print lp.obj[:]
    num_rows = 0
    for f_index in xrange(0,num_frames):
        E1 = X[1][f_index].shape[0]
        E2 = X[2][f_index].shape[0]
        N1 = N1_list[f_index]
        N2 = 1
        num_rows += 3*E1*K1*K1+3*E2*K1*K2+N1+N2

        #might not be in final version
    for e_index in xrange(0, num_temporal_edges):
        E3 = X[6][e_index].shape[0]
        E4 = 1
        num_rows += 3*E3*K1*K1+3*E4*K2*K2

    lp.rows.add(num_rows)
    for r in lp.rows:      # Iterate over all rows
        r.name = 'p%d' %  r.index # Name them

    index_jump =0
    row_index_jump = 0
    t = []
    index_jump_map = {} #Might not be in final version

    for f_index in xrange(0,num_frames):
        index_jump_map[f_index] = index_jump
        edges_obj_obj = X[1][f_index]
        edges_obj_skel = X[2][f_index]
        E1 = edges_obj_obj.shape[0]
        E2 = edges_obj_skel.shape[0]
        N1 = N1_list[f_index]
        N2 = 1
        print "X",f_index,X
        print "P",E1,edges_obj_obj
        for cnum in xrange(index_jump,index_jump + N1*K1):      # Iterate over all columns
            index = cnum  - index_jump
            lp.cols[cnum].name = 'y_obj_%d_%d_%d' % ( f_index, index/K1 , (index%K1)+1) # Name them y_obj_0_1, etc
            lp.cols[cnum].kind=int
            lp.cols[cnum].bounds = 0.0, 1.0
        for cnum in xrange(index_jump + N1*K1, index_jump + N1*K1 + N2*K2):
            index = cnum  - index_jump - N1*K1
            lp.cols[cnum].name = 'y_skel_%d_%d_%d' % ( f_index, index/K2 , (index%K2) + 1 ) # name them y_skel_0_1 etc
            lp.cols[cnum].kind = int
            lp.cols[cnum].bounds = 0.0, 1.0
        for cnum in xrange(index_jump + N1*K1 + N2*K2, index_jump + N1*K1 + N2*K2 + K1*K1*E1):
            index = cnum - index_jump - N1*K1 - N2*K2
            lp.cols[cnum].name = 'y_%d_%d-%d_%d-%d' % ( f_index, edges_obj_obj[int(index/(K1*K1)),0] ,edges_obj_obj[int(index/(K1*K1)),1] , int((index%(K1*K1))/K1)+1 , int((index%(K1*K1))%K1)+1)
            lp.cols[cnum].bounds = 0.0, 1.0
        for cnum in xrange(index_jump + N1*K1 + N2*K2 + K1*K1*E1, index_jump + N1*K1 + N2*K2 + K1*K1*E1 + K1*K2*E2) :
            index = cnum - index_jump - N1*K1 - N2*K2 - K1*K1*E1
            lp.cols[cnum].name = 'y_%d_%d-%d_%d-%d' % ( f_index, edges_obj_skel[int(index/(K1*K2)),0] ,edges_obj_skel[int(index/(K1*K2)),1] , int((index%(K1*K2))/K2)+1 , int((index%(K1*K2))%K2)+1)
            lp.cols[cnum].bounds = 0.0, 1.0    # Set bound 0 <= xi <= 1

        for i in xrange(row_index_jump, row_index_jump + 2*E1*K1*K1):
            lp.rows[i].bounds = 0, None
        for i in xrange(row_index_jump + 2*E1*K1*K1, row_index_jump + 3*E1*K1*K1):
            lp.rows[i].bounds = None,1
        for i in xrange(row_index_jump + 3*E1*K1*K1, row_index_jump + 3*E1*K1*K1 +2*E2*K1*K2):
            lp.rows[i].bounds = 0, None
        for i in xrange(row_index_jump + 3*E1*K1*K1 + 2*E2*K1*K2, row_index_jump + 3*E1*K1*K1 + 3*E2*K1*K2):
            lp.rows[i].bounds = None,1
        for i in xrange(row_index_jump + 3*E1*K1*K1 + 3*E2*K1*K2 ,row_index_jump + 3*E1*K1*K1 + 3*E2*K1*K2 + N1):
            if (LE == False) :
                lp.rows[i].bounds = 1,1  ##SUM = 1
            else:
                lp.rows[i].bounds = None,1  ## SUM = 1 is changed to SUM<= 1
        for i in xrange(row_index_jump + 3*E1*K1*K1 + 3*E2*K1*K2 +N1 ,row_index_jump + 3*E1*K1*K1 + 3*E2*K1*K2 + N1 + N2):
            if (LE == False) :
                lp.rows[i].bounds = 1,1  ##SUM = 1
            else:
                lp.rows[i].bounds = None,1  ## SUM = 1 is changed to SUM<= 1


        for e in xrange(0,E1):
            u = edges_obj_obj[e,0]
            v = edges_obj_obj[e,1]
            n = -1
            for i in xrange(0,K1):
                for j in xrange(0,K1):
                    n += 1
                    a = int(u*K1 + i) + index_jump
                    b = int(v*K1 + j) + index_jump
                    c = N1*K1 + N2*K2 + e*K1*K1 + i*K1 + j + index_jump
                    ec = e*K1*K1 + n + row_index_jump
                    t.append((ec,a,1))
                    t.append((ec,c,-1))
                    ec += E1*K1*K1
                    t.append((ec,b,1))
                    t.append((ec,c,-1))
                    ec += E1*K1*K1
                    t.append((ec,a,1))
                    t.append((ec,b,1))
                    t.append((ec,c,-1))
        for e in xrange(0,E2):
            u = edges_obj_skel[e,0]
            v = edges_obj_skel[e,1]
            n = 3*E1*K1*K1 -1
            for i in xrange(0,K1):
                for j in xrange(0,K2):
                    n += 1
                    a = int(u*K1 + i) + index_jump
                    b = int(K1*N1+ v*K2 + j) + index_jump
                    c = N1*K1 + N2*K2 + E1*K1*K1 + e*K1*K2 + i*K2 + j + index_jump
                    ec = e*K1*K2 + n + row_index_jump
                    t.append((ec,a,1))
                    t.append((ec,c,-1))
                    ec += E2*K1*K2
                    t.append((ec,b,1))
                    t.append((ec,c,-1))
                    ec += E2*K1*K2
                    t.append((ec,a,1))
                    t.append((ec,b,1))
                    t.append((ec,c,-1))
        for e in xrange(0,N1):
            r = 3*E1*K1*K1 + 3*E2*K1*K2 +e + row_index_jump
            for i in xrange(0,K1):
                c = e*K1+i + index_jump
                t.append((r,c,1))
        for e in xrange(0,N2):
            r = 3*E1*K1*K1 + 3*E2*K1*K2 + N1 +e + row_index_jump
            for i in xrange(0,K2):
                c = K1*N1+ e*K2+i + index_jump
                t.append((r,c,1))

        row_index_jump += 3*E1*K1*K1+3*E2*K1*K2+N1+N2
        index_jump += N1*K1 + N2*K2 + K1*K1*E1 + K1*K2*E2

    for e_index in xrange(0, num_temporal_edges):
        edges_obj_temporal = X[6][e_index]
        E3 = edges_obj_temporal.shape[0]
        E4 = 1
        if(NODEONLY == "true"):
            E4 =  0
        for cnum in xrange(index_jump, index_jump +  K1*K1*E3):
            index = cnum - index_jump
            lp.cols[cnum].name = 'y_t_o_%d_%d_%d-%d' % ( e_index, edges_obj_temporal[int(index/(K1*K1)),0] , int((index%(K1*K1))/K1)+1 , int((index%(K1*K1))%K1)+1)
            lp.cols[cnum].bounds = 0.0, 1.0
        for cnum in xrange(index_jump +  K1*K1*E3, index_jump + K1*K1*E3 + K2*K2*E4) :
            index = cnum - index_jump - N1*K1 - N2*K2 - K1*K1*E1
            ##print index, K1, K2, index_jump
            lp.cols[cnum].name = 'y_t_s_%d_%d-%d' % ( e_index, int((index%(K2*K2))/K2)+1 , int((index%(K2*K2))%K2)+1)
            lp.cols[cnum].bounds = 0.0, 1.0    # Set bound 0 <= xi <= 1


        for i in xrange(row_index_jump, row_index_jump + 2*E3*K1*K1):
            lp.rows[i].bounds = 0, None
        for i in xrange(row_index_jump + 2*E3*K1*K1, row_index_jump + 3*E3*K1*K1):
            lp.rows[i].bounds = None,1
        for i in xrange(row_index_jump + 3*E3*K1*K1, row_index_jump + 3*E3*K1*K1 +2*E4*K2*K2):
            lp.rows[i].bounds = 0, None
        for i in xrange(row_index_jump + 3*E3*K1*K1 + 2*E4*K2*K2, row_index_jump + 3*E3*K1*K1 + 3*E4*K2*K2):
            lp.rows[i].bounds = None,1

        for e in xrange(0,E3):
            u = edges_obj_temporal[e,0]
            index_jump_u = index_jump_map[temporal_frame_list[e_index][0]]
            index_jump_v = index_jump_map[temporal_frame_list[e_index][1]]
            n = -1
            for i in xrange(0,K1):
                for j in xrange(0,K1):
                    n += 1
                    a = int(u*K1 + i) + index_jump_u
                    b = int(u*K1 + j) + index_jump_v
                    c = e*K1*K1 + i*K1 + j + index_jump
                    ec = e*K1*K1 + n + row_index_jump
                    t.append((ec,a,1))
                    t.append((ec,c,-1))
                    ec += E3*K1*K1
                    t.append((ec,b,1))
                    t.append((ec,c,-1))
                    ec += E3*K1*K1
                    t.append((ec,a,1))
                    t.append((ec,b,1))
                    t.append((ec,c,-1))
        for e in xrange(0,E4):
            u = 0 # only one sub activity node
            index_jump_u = index_jump_map[temporal_frame_list[e_index][0]]
            index_jump_v = index_jump_map[temporal_frame_list[e_index][1]]
            n = 3*E3*K1*K1 -1
            for i in xrange(0,K2):
                for j in xrange(0,K2):
                    n += 1
                    a = int(K1*N1+ u*K2 + i) + index_jump_u
                    b = int(K1*N1+ u*K2 + j) + index_jump_v
                    c =  E3*K1*K1 + e*K2*K2 + i*K2 + j + index_jump
                    ec = e*K2*K2 + n + row_index_jump
                    t.append((ec,a,1))
                    t.append((ec,c,-1))
                    ec += E4*K2*K2
                    t.append((ec,b,1))
                    t.append((ec,c,-1))
                    ec += E4*K2*K2
                    t.append((ec,a,1))
                    t.append((ec,b,1))
                    t.append((ec,c,-1))
        row_index_jump += 3*E3*K1*K1 + 3*E4*K2*K2
        index_jump += K1*K1*E3 + K2*K2*E4
    ##print len(t)
    ##print t
    lp.matrix = t


###########################################
###########################################
###########################################
    retval=lp.simplex();

    assert retval == None
    labeling = asmatrix(array([c.primal for c in lp.cols]))
    ##print labeling
    index_jump =0
    for f_index in xrange(0,num_frames):
        E1 = X[1][f_index].shape[0]
        E2 = X[2][f_index].shape[0]
        N1 = X[3][f_index]
        N2 = 1
        for c in lp.cols:      # Iterate over all columns
            if (c.index - index_jump < N1*K1 + N2*K2 and c.index > index_jump) :
                c.kind=int
        index_jump += N1*K1 + N2*K2 + K1*K1*E1 + K1*K2*E2

    retval=lp.integer(tm_lim=300000)

    MIPFin = time.clock()
    print "Time for MIP:", (MIPFin-lpFin)

    assert retval == None or retval == "tmlim"
  #  #print 'Z = %g;' % lp.obj.value,  # Retrieve and #print obj func value
   # #print '; '.join('%s = %g' % (c.name, c.primal) for c in lp.cols)
                       # #print struct variable names and primal val
    if(retval == None):
        labeling = asmatrix(array([c.primal for c in lp.cols]))

    #print labeling.T
    #print labeling.shape
    ymax = (csr_matrix(labeling.T,dtype='d'),N1_list,E1_list,num_frames,num_temporal_edges, obj_map_list)
    #print ymax
    c1 = 0
    c0= 0
    ch =0
    cr = 0
    for c in lp.cols:
        if (c.primal == 1):
            c1 += 1
        elif(c.primal ==0):
            c0 += 1
        elif (c.primal == 0.5):
            ch += 1
        else:
            cr +=1
    #print 'number of 1s: %d' % c1
    #print 'number of 0s: %d' % c0
    #print 'number of 0.5s: %d' % ch
    #print 'number of 0s: %d' % cr
    #score = asarray((w_mat*x*ymax[0]).todense())[0][0];
    score2 = 0#sm.svm_model.classify(psi(x,ymax,sm,sparm))
    #print "objective value = ", round(lp.obj.value,2)
    #print '\n score : ' , round(score,2), ' score2: ',score2;
    #if(lp.obj.value  > 1.1):
    #  assert (round(lp.obj.value,2) ==  round(score,2))
    score = (w_mat*x*ymax[0])[0,0]
    print "score:", score
    f = open('predscore_'+aid+'.txt','w')
    f.write(repr(score))
    f.close()
##############################################################################
##############################################################################
##############################################################################
    start = time.clock()
    for i in xrange(0,nummodes):
        # run the inference
        print "NO"
        # apply the laggrange update 

    return ymax


def lp_inference_divmbest_not(X,sm,sparm,LE):
    global NUM_CLASSES_OBJ
    global NUM_CLASSES_SKEL
    # Parameters:
    nummodes = 20
    lamb = -0.04

    #Get the example and prepare data
    K1 = NUM_CLASSES_OBJ
    K2 = NUM_CLASSES_SKEL
    num_frames = X[5]
    num_temporal_edges = X[8]
    obj_map_list = X[10]
    aid = X[11] #Might delete later

    w = sm.w
    x = X[0]
    N1_list = X[3]
    E1_list = X[4]

    w_list = [w[i] for i in xrange(0,x.shape[0])]
    
    w_mat = csr_matrix(asmatrix(array(w_list)),dtype='d')

    lp = glpk.LPX()        # Create empty problem instance
    lp.name = 'inference'     # Assign symbolic name to problem
    lp.obj.maximize = True # Set this as a maximization problem
    lp.cols.add(X[0].shape[1])         # Append three columns to this instance
    lp.obj[:] = (asarray((w_mat*x).todense())[0]).tolist()
    
    num_rows = 0

    for f_index in xrange(0,num_frames):
        E1 = X[1][f_index].shape[0]
        E2 = X[2][f_index].shape[0]
        N1 = N1_list[f_index]
        N2 = 1
        num_rows += 3*E1*K1*K1+3*E2*K1*K2+N1+N2

    lp.rows.add(num_rows)
    for r in lp.rows:      # Iterate over all rows
        r.name = 'p%d' %  r.index # Name them

    index_jump =0
    row_index_jump = 0
    t = []

    for f_index in xrange(0,num_frames):
        edges_obj_obj = X[1][f_index]
        edges_obj_skel = X[2][f_index]
        E1 = edges_obj_obj.shape[0]
        E2 = edges_obj_skel.shape[0]
        N1 = N1_list[f_index]
        N2 = 1
        for c in lp.cols:      # Iterate over all columns
            if (c.index < index_jump + N1*K1):
                index = c.index  - index_jump
                c.name = 'y_obj_%d_%d_%d' % ( f_index, index/K1 , (index%K1)+1) # Name them y_obj_0_1, etc
                c.kind=int
            elif((c.index - index_jump - N1*K1) < N2*K2) :
                index = c.index  - index_jump - N1*K1
                c.name = 'y_skel_%d_%d_%d' % ( f_index, index/K2 , (index%K2) + 1 ) # name them y_skel_0_1 etc
                c.kind = int
            elif((c.index - index_jump - N1*K1 - N2*K2) < K1*K1*E1):
                index = c.index - index_jump - N1*K1 - N2*K2
                c.name = 'y_%d_%d-%d_%d-%d' % ( f_index, edges_obj_obj[int(index/(K1*K1)),0] ,edges_obj_obj[int(index/(K1*K1)),1] , int((index%(K1*K1))/K1)+1 , int((index%(K1*K1))%K1)+1)
            elif((c.index - index_jump - N1*K1 - N2*K2 - K1*K1*E1) < K1*K2*E2) :
                index = c.index - index_jump - N1*K1 - N2*K2 - K1*K1*E1
                ##print index, K1, K2, index_jump
                c.name = 'y_%d_%d-%d_%d-%d' % ( f_index, edges_obj_skel[int(index/(K1*K2)),0] ,edges_obj_skel[int(index/(K1*K2)),1] , int((index%(K1*K2))/K2)+1 , int((index%(K1*K2))%K2)+1)

            c.bounds = 0.0, 1.0    # Set bound 0 <= xi <= 1

        for i in xrange(row_index_jump, row_index_jump + 2*E1*K1*K1):
            lp.rows[i].bounds = 0, None
        for i in xrange(row_index_jump + 2*E1*K1*K1, row_index_jump + 3*E1*K1*K1):
            lp.rows[i].bounds = None,1
        for i in xrange(row_index_jump + 3*E1*K1*K1, row_index_jump + 3*E1*K1*K1 +2*E2*K1*K2):
            lp.rows[i].bounds = 0, None
        for i in xrange(row_index_jump + 3*E1*K1*K1 + 2*E2*K1*K2, row_index_jump + 3*E1*K1*K1 + 3*E2*K1*K2):
            lp.rows[i].bounds = None,1
        for i in xrange(row_index_jump + 3*E1*K1*K1 + 3*E2*K1*K2 ,row_index_jump + 3*E1*K1*K1 + 3*E2*K1*K2 + N1):
            if (LE == False) :
                lp.rows[i].bounds = 1,1  ##SUM = 1
            else:
                lp.rows[i].bounds = None,1  ## SUM = 1 is changed to SUM<= 1
        for i in xrange(row_index_jump + 3*E1*K1*K1 + 3*E2*K1*K2 +N1 ,row_index_jump + 3*E1*K1*K1 + 3*E2*K1*K2 + N1 + N2):
            if (LE == False) :
                lp.rows[i].bounds = 1,1  ##SUM = 1
            else:
                lp.rows[i].bounds = None,1  ## SUM = 1 is changed to SUM<= 1

        for e in xrange(0,E1):
            u = edges_obj_obj[e,0]
            v = edges_obj_obj[e,1]
            n = -1
            for i in xrange(0,K1):
                for j in xrange(0,K1):
                    n += 1
                    a = int(u*K1 + i) + index_jump
                    b = int(v*K1 + j) + index_jump
                    c = N1*K1 + N2*K2 + e*K1*K1 + i*K1 + j + index_jump
                    ec = e*K1*K1 + n + row_index_jump
                    t.append((ec,a,1))
                    t.append((ec,c,-1))
                    ec += E1*K1*K1
                    t.append((ec,b,1))
                    t.append((ec,c,-1))
                    ec += E1*K1*K1
                    t.append((ec,a,1))
                    t.append((ec,b,1))
                    t.append((ec,c,-1))


        for e in xrange(0,E2):
            u = edges_obj_skel[e,0]
            v = edges_obj_skel[e,1]
            n = 3*E1*K1*K1 -1
            for i in xrange(0,K1):
                for j in xrange(0,K2):
                    n += 1
                    a = int(u*K1 + i) + index_jump
                    b = int(K1*N1+ v*K2 + j) + index_jump
                    c = N1*K1 + N2*K2 + E1*K1*K1 + e*K1*K2 + i*K2 + j + index_jump
                    ec = e*K1*K2 + n + row_index_jump
                    t.append((ec,a,1))
                    t.append((ec,c,-1))
                    ec += E2*K1*K2
                    t.append((ec,b,1))
                    t.append((ec,c,-1))
                    ec += E2*K1*K2
                    t.append((ec,a,1))
                    t.append((ec,b,1))
                    t.append((ec,c,-1))

        for e in xrange(0,N1):
            r = 3*E1*K1*K1 + 3*E2*K1*K2 +e + row_index_jump
            for i in xrange(0,K1):
                c = e*K1+i + index_jump
                t.append((r,c,1))
        for e in xrange(0,N2):
            r = 3*E1*K1*K1 + 3*E2*K1*K2 + N1 +e + row_index_jump
            for i in xrange(0,K2):
                c = K1*N1+ e*K2+i + index_jump
                t.append((r,c,1))
        row_index_jump += 3*E1*K1*K1+3*E2*K1*K2+N1+N2
        index_jump += N1*K1 + N2*K2 + K1*K1*E1 + K1*K2*E2

    ##print len(t)
    ##print t
    lp.matrix = t


###########################################
###########################################
###########################################
    start = time.clock()
    retval=lp.simplex();

#    print "Time for LP:", (lpFin-start)

    assert retval == None
    labeling = asmatrix(array([c.primal for c in lp.cols]))
    ##print labeling
    index_jump =0
    for f_index in xrange(0,num_frames):
        E1 = X[1][f_index].shape[0]
        E2 = X[2][f_index].shape[0]
        N1 = X[3][f_index]
        N2 = 1
        if(NODEONLY == "true"):
            N2 =  0
        for c in lp.cols:      # Iterate over all columns
            if (c.index - index_jump < N1*K1 + N2*K2 and c.index > index_jump) :
                c.kind=int
        index_jump += N1*K1 + N2*K2 + K1*K1*E1 + K1*K2*E2

    Ob = lp.obj[:]
    DMB = {'val':[],'labels':[]}
    for i in xrange(0,nummodes):
        lpFin = time.clock()
        retval=lp.integer(tm_lim=300000)
        MIPFin = time.clock()
        print "Time for DBEST MIP:", (MIPFin-lpFin)
        print "R",retval
        assert retval == None or retval == "tmlim"
        if(retval == None):
            labeling = asmatrix(array([c.primal for c in lp.cols]))
        ymax = (csr_matrix(labeling.T,dtype='d'),N1_list,E1_list,num_frames,num_temporal_edges, obj_map_list)

        if (i==0):
            ymax_d=ymax

        index_jump = 0
        labelss = []
        for f_index in xrange(0,num_frames):    
            N1 = ymax[1][f_index]
            N2 = 1
            E1 = ymax[2][f_index][0]
            E2 =ymax[2][f_index][1]

            if(N1>0):
                y_obj = ymax[0][index_jump : index_jump + N1*NUM_CLASSES_OBJ]
                lbss =  get_labels_dm(y_obj,NUM_CLASSES_OBJ,N1)
                labelss.append(lbss)
            if(N2>0):
                y_skel = ymax[0][index_jump + N1*NUM_CLASSES_OBJ: index_jump + N1*NUM_CLASSES_OBJ+N2*NUM_CLASSES_SKEL]
                lbss =  get_labels_dm(y_skel,NUM_CLASSES_SKEL,1)
                labelss.append(lbss)
            index_jump += N1*K1+N2*K2+ E1*K1*K1 + E2*K1*K2


        #Update labels
        row_le=N1*K1+N2*K2+ E1*K1*K1 + E2*K1*K2
        val =0;
        gygy=ymax[0].todense()[:];
        for b in xrange(len(Ob)):
            val = val + Ob[b]*gygy[b]

        DMB['labels'].append(labelss)
        DMB['val'].append(val)
        for f_index in xrange(0,num_frames):
            for id_o in xrange(0,N1):
                lp.obj[f_index*row_le+id_o*NUM_CLASSES_OBJ+ (labelss[f_index*2][id_o]-1)] = lp.obj[f_index*row_le+id_o*NUM_CLASSES_OBJ+ (labelss[f_index*2][id_o]-1)] + lamb
            lp.obj[f_index*(row_le)+N1*NUM_CLASSES_OBJ+(labelss[f_index*2+1][0]-1)] = lp.obj[f_index*(row_le)+N1*NUM_CLASSES_OBJ+(labelss[f_index*2+1][0]-1)] + lamb            
        
    score2 = 0
    f = open(aid+'_dmbest.bn', 'wb')
    pickle.dump(DMB,f)
    f.close()
    return ymax_d


def lp_inference_temporal_sum1_IP(X,sm,sparm,LE):
    global NUM_CLASSES_OBJ
    global NUM_CLASSES_SKEL
    start = time.clock()
    K1 = NUM_CLASSES_OBJ
    K2 = NUM_CLASSES_SKEL
    num_frames = X[5]
    num_temporal_edges = X[8]
    obj_map_list = X[10]
    aid = X[11]

   # print X[3],X[4]

    w = sm.w

    x = X[0]
    N1_list = X[3]
    E1_list = X[4]
    temporal_frame_list = X[7]

    #x = (X[0]).todense()
    #print len(w)
    #print x.shape[0]
    w_list = [w[i] for i in xrange(0,x.shape[0])]
    ##print w_list

    w_mat = csr_matrix(asmatrix(array(w_list)),dtype='d')
    ##print w_mat.shape , x.shape
    lp = glpk.LPX()        # Create empty problem instance
    lp.name = 'inference'     # Assign symbolic name to problem
    lp.obj.maximize = True # Set this as a maximization problem
    lp.cols.add(X[0].shape[1])         # Append three columns to this instance
    lp.obj[:] = (asarray((w_mat*x).todense())[0]).tolist()
    ##print lp.obj[:]
    num_rows = 0
    for f_index in xrange(0,num_frames):
        E1 = X[1][f_index].shape[0]
        E2 = X[2][f_index].shape[0]
        N1 = N1_list[f_index]
        N2 = 1
        if(NODEONLY == "true"):
            N2 =  0
        num_rows += 3*E1*K1*K1+3*E2*K1*K2+N1+N2

    for e_index in xrange(0, num_temporal_edges):
        E3 = X[6][e_index].shape[0]
        E4 = 1
        if(NODEONLY == "true"):
            E4 =  0
        num_rows += 3*E3*K1*K1+3*E4*K2*K2

    lp.rows.add(num_rows)
    for r in lp.rows:      # Iterate over all rows
        r.name = 'p%d' %  r.index # Name them

    index_jump =0
    row_index_jump = 0
    t = []
    index_jump_map = {}
    for f_index in xrange(0,num_frames):
        index_jump_map[f_index] = index_jump
        edges_obj_obj = X[1][f_index]
        edges_obj_skel = X[2][f_index]
        E1 = edges_obj_obj.shape[0]
        E2 = edges_obj_skel.shape[0]
        N1 = N1_list[f_index]
        N2 = 1
        if(NODEONLY == "true"):
            N2 =  0

        #lp.cols.add(X[0].get_shape()[1])         # Append three columns to this instance
        #count_t= 0
        for cnum in xrange(index_jump,index_jump + N1*K1):      # Iterate over all columns
            index = cnum  - index_jump
            lp.cols[cnum].name = 'y_obj_%d_%d_%d' % ( f_index, index/K1 , (index%K1)+1) # Name them y_obj_0_1, etc
            lp.cols[cnum].kind=int
            lp.cols[cnum].bounds = 0.0, 1.0
        for cnum in xrange(index_jump + N1*K1, index_jump + N1*K1 + N2*K2):
            index = cnum  - index_jump - N1*K1
            lp.cols[cnum].name = 'y_skel_%d_%d_%d' % ( f_index, index/K2 , (index%K2) + 1 ) # name them y_skel_0_1 etc
            lp.cols[cnum].kind = int
            lp.cols[cnum].bounds = 0.0, 1.0
        for cnum in xrange(index_jump + N1*K1 + N2*K2, index_jump + N1*K1 + N2*K2 + K1*K1*E1):
            index = cnum - index_jump - N1*K1 - N2*K2
            lp.cols[cnum].name = 'y_%d_%d-%d_%d-%d' % ( f_index, edges_obj_obj[int(index/(K1*K1)),0] ,edges_obj_obj[int(index/(K1*K1)),1] , int((index%(K1*K1))/K1)+1 , int((index%(K1*K1))%K1)+1)
            lp.cols[cnum].bounds = 0.0, 1.0
        for cnum in xrange(index_jump + N1*K1 + N2*K2 + K1*K1*E1, index_jump + N1*K1 + N2*K2 + K1*K1*E1 + K1*K2*E2) :
            index = cnum - index_jump - N1*K1 - N2*K2 - K1*K1*E1
            ##print index, K1, K2, index_jump
            lp.cols[cnum].name = 'y_%d_%d-%d_%d-%d' % ( f_index, edges_obj_skel[int(index/(K1*K2)),0] ,edges_obj_skel[int(index/(K1*K2)),1] , int((index%(K1*K2))/K2)+1 , int((index%(K1*K2))%K2)+1)
            lp.cols[cnum].bounds = 0.0, 1.0    # Set bound 0 <= xi <= 1

        for i in xrange(row_index_jump, row_index_jump + 2*E1*K1*K1):
            lp.rows[i].bounds = 0, None
        for i in xrange(row_index_jump + 2*E1*K1*K1, row_index_jump + 3*E1*K1*K1):
            lp.rows[i].bounds = None,1
        for i in xrange(row_index_jump + 3*E1*K1*K1, row_index_jump + 3*E1*K1*K1 +2*E2*K1*K2):
            lp.rows[i].bounds = 0, None
        for i in xrange(row_index_jump + 3*E1*K1*K1 + 2*E2*K1*K2, row_index_jump + 3*E1*K1*K1 + 3*E2*K1*K2):
            lp.rows[i].bounds = None,1
        for i in xrange(row_index_jump + 3*E1*K1*K1 + 3*E2*K1*K2 ,row_index_jump + 3*E1*K1*K1 + 3*E2*K1*K2 + N1):
            if (LE == False) :
                lp.rows[i].bounds = 1,1  ##SUM = 1
            else:
                lp.rows[i].bounds = None,1  ## SUM = 1 is changed to SUM<= 1
        for i in xrange(row_index_jump + 3*E1*K1*K1 + 3*E2*K1*K2 +N1 ,row_index_jump + 3*E1*K1*K1 + 3*E2*K1*K2 + N1 + N2):
            if (LE == False) :
                lp.rows[i].bounds = 1,1  ##SUM = 1
            else:
                lp.rows[i].bounds = None,1  ## SUM = 1 is changed to SUM<= 1


        for e in xrange(0,E1):
            u = edges_obj_obj[e,0]
            v = edges_obj_obj[e,1]
            n = -1
            for i in xrange(0,K1):
                for j in xrange(0,K1):
                    n += 1
                    a = int(u*K1 + i) + index_jump
                    b = int(v*K1 + j) + index_jump
                    c = N1*K1 + N2*K2 + e*K1*K1 + i*K1 + j + index_jump
                    ec = e*K1*K1 + n + row_index_jump
                    t.append((ec,a,1))
                    t.append((ec,c,-1))
                    ec += E1*K1*K1
                    t.append((ec,b,1))
                    t.append((ec,c,-1))
                    ec += E1*K1*K1
                    t.append((ec,a,1))
                    t.append((ec,b,1))
                    t.append((ec,c,-1))
        for e in xrange(0,E2):
            u = edges_obj_skel[e,0]
            v = edges_obj_skel[e,1]
            n = 3*E1*K1*K1 -1
            for i in xrange(0,K1):
                for j in xrange(0,K2):
                    n += 1
                    a = int(u*K1 + i) + index_jump
                    b = int(K1*N1+ v*K2 + j) + index_jump
                    c = N1*K1 + N2*K2 + E1*K1*K1 + e*K1*K2 + i*K2 + j + index_jump
                    ec = e*K1*K2 + n + row_index_jump
                    t.append((ec,a,1))
                    t.append((ec,c,-1))
                    ec += E2*K1*K2
                    t.append((ec,b,1))
                    t.append((ec,c,-1))
                    ec += E2*K1*K2
                    t.append((ec,a,1))
                    t.append((ec,b,1))
                    t.append((ec,c,-1))
        for e in xrange(0,N1):
            r = 3*E1*K1*K1 + 3*E2*K1*K2 +e + row_index_jump
            for i in xrange(0,K1):
                c = e*K1+i + index_jump
                t.append((r,c,1))
        for e in xrange(0,N2):
            r = 3*E1*K1*K1 + 3*E2*K1*K2 + N1 +e + row_index_jump
            for i in xrange(0,K2):
                c = K1*N1+ e*K2+i + index_jump
                t.append((r,c,1))

        row_index_jump += 3*E1*K1*K1+3*E2*K1*K2+N1+N2
        index_jump += N1*K1 + N2*K2 + K1*K1*E1 + K1*K2*E2

    for e_index in xrange(0, num_temporal_edges):
        edges_obj_temporal = X[6][e_index]
        E3 = edges_obj_temporal.shape[0]
        E4 = 1
        if(NODEONLY == "true"):
            E4 =  0
        for cnum in xrange(index_jump, index_jump +  K1*K1*E3):
            index = cnum - index_jump
            lp.cols[cnum].name = 'y_t_o_%d_%d_%d-%d' % ( e_index, edges_obj_temporal[int(index/(K1*K1)),0] , int((index%(K1*K1))/K1)+1 , int((index%(K1*K1))%K1)+1)
            lp.cols[cnum].bounds = 0.0, 1.0
        for cnum in xrange(index_jump +  K1*K1*E3, index_jump + K1*K1*E3 + K2*K2*E4) :
            index = cnum - index_jump - N1*K1 - N2*K2 - K1*K1*E1
            ##print index, K1, K2, index_jump
            lp.cols[cnum].name = 'y_t_s_%d_%d-%d' % ( e_index, int((index%(K2*K2))/K2)+1 , int((index%(K2*K2))%K2)+1)
            lp.cols[cnum].bounds = 0.0, 1.0    # Set bound 0 <= xi <= 1


        for i in xrange(row_index_jump, row_index_jump + 2*E3*K1*K1):
            lp.rows[i].bounds = 0, None
        for i in xrange(row_index_jump + 2*E3*K1*K1, row_index_jump + 3*E3*K1*K1):
            lp.rows[i].bounds = None,1
        for i in xrange(row_index_jump + 3*E3*K1*K1, row_index_jump + 3*E3*K1*K1 +2*E4*K2*K2):
            lp.rows[i].bounds = 0, None
        for i in xrange(row_index_jump + 3*E3*K1*K1 + 2*E4*K2*K2, row_index_jump + 3*E3*K1*K1 + 3*E4*K2*K2):
            lp.rows[i].bounds = None,1

        for e in xrange(0,E3):
            u = edges_obj_temporal[e,0]
            index_jump_u = index_jump_map[temporal_frame_list[e_index][0]]
            index_jump_v = index_jump_map[temporal_frame_list[e_index][1]]
            n = -1
            for i in xrange(0,K1):
                for j in xrange(0,K1):
                    n += 1
                    a = int(u*K1 + i) + index_jump_u
                    b = int(u*K1 + j) + index_jump_v
                    c = e*K1*K1 + i*K1 + j + index_jump
                    ec = e*K1*K1 + n + row_index_jump
                    t.append((ec,a,1))
                    t.append((ec,c,-1))
                    ec += E3*K1*K1
                    t.append((ec,b,1))
                    t.append((ec,c,-1))
                    ec += E3*K1*K1
                    t.append((ec,a,1))
                    t.append((ec,b,1))
                    t.append((ec,c,-1))
        for e in xrange(0,E4):
            u = 0 # only one sub activity node
            index_jump_u = index_jump_map[temporal_frame_list[e_index][0]]
            index_jump_v = index_jump_map[temporal_frame_list[e_index][1]]
            n = 3*E3*K1*K1 -1
            for i in xrange(0,K2):
                for j in xrange(0,K2):
                    n += 1
                    a = int(K1*N1+ u*K2 + i) + index_jump_u
                    b = int(K1*N1+ u*K2 + j) + index_jump_v
                    c =  E3*K1*K1 + e*K2*K2 + i*K2 + j + index_jump
                    ec = e*K2*K2 + n + row_index_jump
                    t.append((ec,a,1))
                    t.append((ec,c,-1))
                    ec += E4*K2*K2
                    t.append((ec,b,1))
                    t.append((ec,c,-1))
                    ec += E4*K2*K2
                    t.append((ec,a,1))
                    t.append((ec,b,1))
                    t.append((ec,c,-1))
        row_index_jump += 3*E3*K1*K1 + 3*E4*K2*K2
        index_jump += K1*K1*E3 + K2*K2*E4
    ##print len(t)
    ##print t
    lp.matrix = t
    retval=lp.simplex();

    lpFin = time.clock()
    #print "Time for LP:", (lpFin-start)

    assert retval == None
    labeling = asmatrix(array([c.primal for c in lp.cols]))
    ##print labeling
    index_jump =0
    for f_index in xrange(0,num_frames):
        E1 = X[1][f_index].shape[0]
        E2 = X[2][f_index].shape[0]
        N1 = X[3][f_index]
        N2 = 1
        if(NODEONLY == "true"):
            N2 =  0
        for c in lp.cols:      # Iterate over all columns
            if (c.index - index_jump < N1*K1 + N2*K2 and c.index > index_jump) :
                c.kind=int
        index_jump += N1*K1 + N2*K2 + K1*K1*E1 + K1*K2*E2

    retval=lp.integer(tm_lim=300000)


    MIPFin = time.clock()
    print "Time for MRF MIP:", (MIPFin-lpFin)

    assert retval == None or retval == "tmlim"
  #  #print 'Z = %g;' % lp.obj.value,  # Retrieve and #print obj func value
   # #print '; '.join('%s = %g' % (c.name, c.primal) for c in lp.cols)
                       # #print struct variable names and primal val
    if(retval == None):
        labeling = asmatrix(array([c.primal for c in lp.cols]))

    #print labeling.T
    #print labeling.shape
    ymax = (csr_matrix(labeling.T,dtype='d'),N1_list,E1_list,num_frames,num_temporal_edges, obj_map_list)
##########################################
##########################################
    index_jump = 0
    labelss = []
    for f_index in xrange(0,num_frames):    
        N1 = ymax[1][f_index]
        N2 = 1
        E1 = ymax[2][f_index][0]
        E2 =ymax[2][f_index][1]

        if(N1>0):
            y_obj = ymax[0][index_jump : index_jump + N1*NUM_CLASSES_OBJ]
            lbss =  get_labels_dm(y_obj,NUM_CLASSES_OBJ,N1)
            labelss.append(lbss)
        if(N2>0):
            y_skel = ymax[0][index_jump + N1*NUM_CLASSES_OBJ: index_jump + N1*NUM_CLASSES_OBJ+N2*NUM_CLASSES_SKEL]
            lbss =  get_labels_dm(y_skel,NUM_CLASSES_SKEL,1)
            labelss.append(lbss)
        index_jump += N1*K1+N2*K2+ E1*K1*K1 + E2*K1*K2

    DMB = {'val':[],'labels':[]}
    DMB['labels'].append(labelss)
    DMB['val'].append(lp.obj.value)
    f = open(aid+'_mrf.bn', 'wb')
    pickle.dump(DMB,f)
    f.close()

###############################################
##################################################33



    #print ymax
    c1 = 0
    c0= 0
    ch =0
    cr = 0
    for c in lp.cols:
        if (c.primal == 1):
            c1 += 1
        elif(c.primal ==0):
            c0 += 1
        elif (c.primal == 0.5):
            ch += 1
        else:
            cr +=1
    #print 'number of 1s: %d' % c1
    #print 'number of 0s: %d' % c0
    #print 'number of 0.5s: %d' % ch
    #print 'number of 0s: %d' % cr
    #score = asarray((w_mat*x*ymax[0]).todense())[0][0];
    score2 = 0#sm.svm_model.classify(psi(x,ymax,sm,sparm))
    #print "objective value = ", round(lp.obj.value,2)
    #print '\n score : ' , round(score,2), ' score2: ',score2;
    #if(lp.obj.value  > 1.1):
    #  assert (round(lp.obj.value,2) ==  round(score,2))
    score = (w_mat*x*ymax[0])[0,0]
    print "score:", score
    f = open('predscore_'+aid+'.txt','w')
    f.write(repr(score))
    f.close()
    return ymax





def classification_score(x,y,sm,sparm):
    """Return an example, label pair discriminant score."""
    # Utilize the svmapi.Model convenience method 'classify'.
    score = sm.svm_model.classify(psi(x,y,sm,sparm))
    global thecount
    thecount += 1
    if (sum(abs(w) for w in sm.w)):
        import pdb; pdb.set_trace()
    return score

def getScore(x,sm,sparm):
    return 0

def classify_example_weight(x, sm, sparm):
    fileptr = open('modelweights_filtered', 'w')
    w_list = [sm.w[i] for i in xrange(0,sm.size_psi)]
    for w in w_list:
      print>>fileptr,w

def classify_example(x, sm, sparm):
    """Returns the classification of an example 'x'."""
    global CLASSIFY_METHOD
    global SINGLE_FRAME
    global TEMPORAL
    global HALLUCINATION
    global DIVMBEST
    print "D",DIVMBEST
    #y = (mat(ones((1,x[0].shape[1]))),x[2],sm.num_classes)
    #l = lp_inference(x,sm,sparm)
    if(SINGLE_FRAME == "true"):
        l = lp_inference_sum1_IP(x,sm,sparm,False)
    elif(DIVMBEST == "true"):
#        l = lp_inference_temporal_sum1_IP(x, sm, sparm, False)
        l = lp_inference_divmbest_not(x, sm, sparm, False)
    elif(TEMPORAL == "false"):
        l = lp_inference_multiple_frames_sum1_IP(x, sm, sparm, False)
    else:
        l = lp_inference_temporal_sum1_IP(x, sm, sparm, False)
        
    '''if(CLASSIFY_METHOD == "sum1.IP"):
        l = lp_inference_sum1_IP(x,sm,sparm,False)
    elif(CLASSIFY_METHOD == "sumLE1.IP"):
        l = lp_inference_sum1_IP(x,sm,sparm,True)
    elif(CLASSIFY_METHOD == "sum1"):
        l = lp_inference_sum1(x,sm,sparm)
    elif(CLASSIFY_METHOD == "qbpo.sum1.IP"):
        l = lp_inference_qbpo_sum1_IP(x,sm,sparm)
    elif(CLASSIFY_METHOD == "qbpo"):
        l = lp_inference_qbpo(x,sm,sparm)'''
   
    return l

def areEqualVectors(V1,V2):
    for i in xrange(0,V1.shape[0]):
        assert(round(V1[i,0]*2, 0)==round(V2[i,0]*2, 0))
        

def psi(x, y, sm, sparm):
    
    """Returns the combined feature vector Psi(x,y)."""
    # Return the product of x and y

    #start = time.clock()
    b = coo_matrix(x[0]*y[0])
    #fin = time.clock()
    #print "Time for matrix multiplication:", (fin-start)

    c = [(b.row[i],b.data[i]) for i in xrange(0,len(b.row))]
    #fin1 = time.clock()
    #print "Time for todense:", (fin1-fin)
    #a = svmapi.Sparse((b.todense()))
    a = svmapi.Sparse((c))
    #fin2 = time.clock()
    #print "Time for psi:", (fin2-fin1)
    return a
    
def loss(Y, Ybar, sparm):
    #global LOSS_METHOD
    global SINGLE_FRAME
    if (SINGLE_FRAME == "true"):
        return loss_micro(Y, Ybar, sparm);
    else:
        return loss_multiple_frame_micro(Y, Ybar, sparm)

def loss_micro(Y, Ybar, sparm):
    """Loss is 1 if the labels are different, 0 if they are the same."""
    global NUM_CLASSES_OBJ
    global NUM_CLASSES_SKEL
    global NODEONLY
    K1 = NUM_CLASSES_OBJ
    K2 = NUM_CLASSES_SKEL
    N2 = 1
    if(NODEONLY == "true"):
        N2 =  0
    N1 = Y[1]
    E1 = Y[2]
    y= Y[0]
    
    #print "N:",N," K: ", K #,y.shape[0],y.shape[1]
    ybar = Ybar[0]
    
    yDiff=y- ybar;
    sum=0.0;
    size=N1*K1 + N2*K2
    

    for index_n in xrange(0,N1):
        for index_k in xrange(0,K1):
            if (y[index_n*K1 : index_n*K1+K1,0].sum() == 1):
                #print 'diff:', yDiff[index_n*K1+index_k,0]
                if yDiff[index_n*K1+index_k,0]>0:
                    sum+=yDiff[index_n*K1+index_k,0]/(N1*K1)
                else:
                    sum-=yDiff[index_n*K1+index_k,0]/(N1*K1)
                #print "sum:", sum
    #print "------"
    #print y
    #print N1*K1, N2*K2
    for index_n in xrange(0,N2):
        for index_k in xrange(0,K2):
            if (y[ N1*K1+ index_n*K2 : N1*K1 + index_n*K2+K2,0].sum() == 1):
                #print 'diff:', yDiff[index_n*K1+index_k,0]
                if yDiff[N1*K1 + index_n*K2+index_k,0]>0:
                    sum+=yDiff[N1*K1 + index_n*K2+index_k,0]/(N2*K2)
                else:
                    sum-=yDiff[ N1*K1 + index_n*K2+index_k,0]/(N2*K2)
                #print "sum:", sum
    #print sum
    return sum;

def loss_multiple_frame_micro(Y, Ybar, sparm):
    """Loss is 1 if the labels are different, 0 if they are the same."""
    global NUM_CLASSES_OBJ
    global NUM_CLASSES_SKEL
    global NODEONLY
    K1 = NUM_CLASSES_OBJ
    K2 = NUM_CLASSES_SKEL
    N2 = 1
    if(NODEONLY == "true"):
        N2 =  0
    N1_list = Y[1]
    E_list = Y[2]
    num_frames = Y[3]
    y= Y[0]

    #print "N:",N," K: ", K #,y.shape[0],y.shape[1]
    ybar = Ybar[0]

    yDiff=y- ybar;
    sum=0.0;
    index_jump = 0

    for f_index in xrange(0,num_frames):
        N1 = N1_list[f_index]
        E1 = E_list[f_index][0]
        E2 = E_list[f_index][1]

        for index_n in xrange(0,N1):
            for index_k in xrange(0,K1):
                if (y[ index_jump + index_n*K1 : index_jump + index_n*K1+K1,0].sum() == 1):
                    #print 'diff:', yDiff[index_n*K1+index_k,0]
                    if yDiff[index_jump + index_n*K1+index_k,0]>0:
                        sum+=yDiff[index_jump + index_n*K1+index_k,0]/(N1*K1)
                    else:
                        sum-=yDiff[index_jump + index_n*K1+index_k,0]/(N1*K1)
                #print "sum:", sum
        #print "------"
        #print y
        #print N1*K1, N2*K2
        for index_n in xrange(0,N2):
            for index_k in xrange(0,K2):
                if (y[ index_jump + N1*K1+ index_n*K2 : index_jump + N1*K1 + index_n*K2+K2,0].sum() == 1):
                    #print 'diff:', yDiff[index_n*K1+index_k,0]
                    if yDiff[index_jump + N1*K1 + index_n*K2+index_k,0]>0:
                        sum+=yDiff[index_jump + N1*K1 + index_n*K2+index_k,0]/(N2*K2)
                    else:
                        sum-=yDiff[ index_jump + N1*K1 + index_n*K2+index_k,0]/(N2*K2)
                    #print "sum:", sum
        index_jump += N1*K1 + N2*K2 + E1*K1*K1 + E2*K1*K2
    #print sum
    return sum;

def write_label_ (fileptr, y):
    return

def write_label(fileptr, y):
    global SINGLE_FRAME
    global NUM_CLASSES_SKEL
    global NUM_CLASSES_OBJ
    global NODEONLY
    if(SINGLE_FRAME == "true"):
        K1 = NUM_CLASSES_OBJ
        K2 = NUM_CLASSES_SKEL
        #K= y[2]
        N1 = y[1]
        N2 = 1
        obj_map = y[3]
        #for node in xrange(0,N):
        #    for label in xrange(0,K):
         #       if(y[0][node*K+label,0] != 0):
         #           s = repr(node+1)+':'+repr(label+1)+':'+repr(y[0][node*K+label,0])
         #           print>>fileptr,s,
        #print>>fileptr
        if(N1>0):
           y_obj = y[0][0 : N1*NUM_CLASSES_OBJ]
        if(N2>0):
           y_skel = y[0][N1*NUM_CLASSES_OBJ:  N1*NUM_CLASSES_OBJ+N2*NUM_CLASSES_SKEL]
        s = ""
        for node in xrange(0,N1):
                maxscore = 0
                maxlabelList =[]
                for label in xrange(0,K1):
                    if(y_obj[node*K1+label,0] > maxscore):
                        maxscore = y_obj[node*K1+label,0]
                for label in xrange(0,K1):
                    if(y_obj[node*K1+label,0] == maxscore):
                        maxlabelList.append(label)
                maxlabel = maxlabelList[random.randint(0,len(maxlabelList))]
                objId =  [k for k, v in obj_map.iteritems() if v == node][0]
                s += repr(int(objId))+':'+repr(maxlabel+1)+':'+repr(maxscore)
                s += ","
        for node in xrange(0,N2):
                maxscore = 0
                maxlabelList = []
                for label in xrange(0,K2):
                    if(y_skel[node*K2+label,0] >maxscore):
                        maxscore = y_skel[node*K2+label,0]
                for label in xrange(0,K2):
                    if(y_skel[node*K2+label,0] == maxscore):
                        maxlabelList.append(label)
                maxlabel =  maxlabelList[random.randint(0,len(maxlabelList))]
                s += repr(node+1)+':'+repr(maxlabel+1)+':'+repr(maxscore)

        s+=";"
        print>>fileptr,s,
        print>>fileptr
    else:
        num_frames = y[3]
        obj_map_list = y[5]
        K1 = NUM_CLASSES_OBJ
        K2 = NUM_CLASSES_SKEL
        #per segment/frame print the objectid:object label; actid:actlabel
        index_jump =0
        s= ""
        for f_index in xrange(0,num_frames):

            N1 = y[1][f_index]
            N2 = 1
            if(NODEONLY == "true"):
                N2 =  0
            E1 = y[2][f_index][0]
            E2 = y[2][f_index][1]
            if(N1>0):
                y_obj = y[0][index_jump : index_jump + N1*NUM_CLASSES_OBJ]
            if(N2>0):
                y_skel = y[0][index_jump + N1*NUM_CLASSES_OBJ: index_jump + N1*NUM_CLASSES_OBJ+N2*NUM_CLASSES_SKEL]
            for node in xrange(0,N1):
                maxscore = 0
                maxlabelList =[]
                for label in xrange(0,K1):
                    if(y_obj[node*K1+label,0] > maxscore):
                        maxscore = y_obj[node*K1+label,0]
                for label in xrange(0,K1):
                    if(y_obj[node*K1+label,0] == maxscore):
                        maxlabelList.append(label)
                maxlabel = maxlabelList[random.randint(0,len(maxlabelList))]
                objId =  [k for k, v in obj_map_list[f_index].iteritems() if v == node][0]
                s += repr(int(objId))+':'+repr(maxlabel+1)+':'+repr(maxscore)
                s += ","
            for node in xrange(0,N2):
                maxscore = 0
                maxlabelList = []
                for label in xrange(0,K2):
                    if(y_skel[node*K2+label,0] >maxscore):
                        maxscore = y_skel[node*K2+label,0]
                for label in xrange(0,K2):
                    if(y_skel[node*K2+label,0] == maxscore):
                        maxlabelList.append(label)
                maxlabel =  maxlabelList[random.randint(0,len(maxlabelList))]
                s += repr(node+1)+':'+repr(maxlabel+1)+':'+repr(maxscore)
                        
            s+=";"
            index_jump += N1*K1+N2*K2+ E1*K1*K1 + E2*K1*K2
        print>>fileptr,s,
        print>>fileptr


def print_iteration_stats(ceps, cached_constraint, sample, sm,
                          cset, alpha, sparm):
    """Called just before the end of each cutting plane iteration.

    This is called just before the end of each cutting plane
    iteration, primarily to #print statistics.  The 'ceps' argument is
    how much the most violated constraint was violated by.  The
    'cached_constraint' argument is true if this constraint was
    constructed from the cache.
    
    The default behavior is that nothing is #printed."""
    # for every 10th iteration save the model file 
    global ITER
    ITER += 1;
    if(ITER%100 == 0):
        filename = "imodels/model.c"+ `sparm.c`  + ".m" + `ITER`;
        print "writing intermediate model: ", filename
        write_model(filename, sm, sparm)
    # #printig the weight vector
    #w_list = [sm.w[i] for i in xrange(0,sm.size_psi)]
    ##print w_list

def write_model(filename, sm, sparm):
    import cPickle, bz2
    cPickle.dump(sm,file(filename,'w'))

def read_model(filename, sparm):
    import cPickle, bz2
    return cPickle.load(file(filename))

def get_labels_dm(Y,K,N):
    y = Y
    lbs = []
    for node in xrange(0,N):
        ym=-1
        for label in xrange(0,K):
            if(y[node*K+label,0]>ym):
                ym=y[node*K+label,0]
        for label in xrange(0,K):
            if(y[node*K+label,0] == ym):
                lbs.append(label+1)
    return lbs



def evaluation_class_pr_sum1(Y,Ybar,K,N,sparm):
    y = Y
    ybar = Ybar
    truecount = zeros((K,1))
    predcount = zeros((K,1))
    singlepredcount = zeros((K,1))
    tpcount = zeros((K,1))
    confusionMatrix=zeros((K,K))
    confusionMatrixWMultiple=zeros((K,K))
    multipleClasses=zeros((K,1))
    zeroClasses=zeros((K,1))
    prec = zeros((K,1))
    recall = zeros((K,1))
    f = open('pred.txt','a')
    ##print y,ybar
    for node in xrange(0,N):
        flag = 0;
        numPositives=0;
        predClass=-1;
        actualClass=-1;
        maxYBar=-1;
        countMax=0
        for label in xrange(0,K):
            ##print node,label
            if(y[node*K+label,0] == 1):
                truecount[label,0] += 1;
                actualClass=label
            if(maxYBar<ybar[node*K+label,0]):
                maxYBar=ybar[node*K+label,0]
                countMax=0
            if(maxYBar==ybar[node*K+label,0]):
                countMax+=1
        if(actualClass == -1):
            continue;
        maxLabelList=[];
        for label in xrange(0,K):
            if(ybar[node*K+label,0] == maxYBar and maxYBar>0): #suboptimal way, but who cares!
                maxLabelList.append(label);
                predcount[label,0] += 1;
                numPositives+=1;
                predClass=label;
                confusionMatrixWMultiple[label,actualClass]+=1.0/countMax;

        if(numPositives==0):
            zeroClasses[actualClass,0]+=1
        elif(numPositives>=1):
#            multipleClasses[actualClass,0]+=1
#        else:
            if(numPositives>1):
                prediction=maxLabelList[random.randint(0,countMax)]
            else:
                prediction=maxLabelList[0]
            confusionMatrix[prediction,actualClass]+=1
            singlepredcount[prediction,0] += 1;
            if(actualClass==prediction):
                tpcount[prediction,0] += 1
                flag = 1;
				
        print>>f, node+1,flag         
    print>>f, "\n"
    for label in xrange(0,K):
        if(singlepredcount[label,0] != 0):
            prec[label,0] = tpcount[label,0]/float(singlepredcount[label,0])
        if(truecount[label,0] !=0):
            recall[label,0] = tpcount[label,0]/float(truecount[label,0])
    return (tpcount,truecount,predcount,confusionMatrix,zeroClasses,multipleClasses,confusionMatrixWMultiple,singlepredcount)

def evaluation_prec_recall(Y, Ybar, K, N ,sparm):
    y = Y[0]
    ybar = Ybar[0]
    prec = 0.0
    recall = 0.0
    for node in xrange(0,N):
        tp_fn = 0.0
        tp_fp = 0.0
        tp= 0.0 #multiply(y[node*K:node*K+K],ybar[node*K:node*K+K])
        for label in xrange(0,K):
            tp_fn += y[node*K+label,0]*y[node*K+label,0]
            tp_fp += ybar[node*K+label,0]*ybar[node*K+label,0]
            tp += y[node*K+label,0]*ybar[node*K+label,0]
        
        if( tp_fp > 0):
            prec += tp/tp_fp; 
        else:
            prec += 0;
        if( tp_fn > 0):
            recall += tp/tp_fn; 
        else:
            recall += 0;

    ##print "similarity is", sim/N
    return (prec/N, recall/N);





def eval_prediction(exnum, (x, y), ypred, sm, sparm, teststats):
    """Accumulate statistics about a single training example.

    Allows accumulated statistics regarding how well the predicted
    label ypred for pattern x matches the true label y.  The first
    time this function is called teststats is None.  This function's
    return value will be passed along to the next call to
    eval_prediction.  After all test predictions are made, the last
    value returned will be passed along to #print_testing_stats.

    On the first call, that is, when exnum==0, teststats==None.  The
    default behavior is that the function does nothing."""
    global SINGLE_FRAME
    
    if exnum==0: teststats = []
    
    if(SINGLE_FRAME == "true"):
        teststats = eval_prediction_single_frame(exnum, (x, y), ypred, sm, sparm, teststats)
    else:
        teststats= eval_prediction_multi_frame(exnum, (x, y), ypred, sm, sparm, teststats)
    return teststats

def eval_prediction_single_frame(exnum, (x, y), ypred, sm, sparm, teststats):
    global NUM_CLASSES_SKEL
    global NUM_CLASSES_OBJ
    global NODEONLY
    if exnum==0: teststats = []

    N1 = y[1]
    N2 = 1
    if(NODEONLY == "true"):
        N2 =  0
    y_obj = y[0][0:N1*NUM_CLASSES_OBJ]
    y_skel = y[0][N1*NUM_CLASSES_OBJ:N1*NUM_CLASSES_OBJ+N2*NUM_CLASSES_SKEL]

    ypred_obj = ypred[0][0:N1*NUM_CLASSES_OBJ]
    ypred_skel = ypred[0][N1*NUM_CLASSES_OBJ:N1*NUM_CLASSES_OBJ+N2*NUM_CLASSES_SKEL]

    obj_res = evaluation_class_pr_sum1(y_obj, ypred_obj, NUM_CLASSES_OBJ , N1, sparm)
    skel_res = evaluation_class_pr_sum1(y_skel, ypred_skel, NUM_CLASSES_SKEL , N2, sparm)
    teststats.append((obj_res,skel_res))
    return teststats


def eval_prediction_multi_frame(exnum, (x, y), ypred, sm, sparm, teststats):
    global NUM_CLASSES_SKEL
    global NUM_CLASSES_OBJ
    global NODEONLY
    if exnum==0: teststats = []
    num_frames = y[3]
    K1 = NUM_CLASSES_OBJ
    K2 = NUM_CLASSES_SKEL
    index_jump =0
    for f_index in xrange(0,num_frames):
        
        N1 = y[1][f_index]
        N2 = 1
        if(NODEONLY == "true"):
            N2 =  0
        E1 = y[2][f_index][0]
        E2 =y[2][f_index][1]
        if(N1>0):
            y_obj = y[0][index_jump : index_jump + N1*NUM_CLASSES_OBJ]
        if(N2>0):
            y_skel = y[0][index_jump + N1*NUM_CLASSES_OBJ: index_jump + N1*NUM_CLASSES_OBJ+N2*NUM_CLASSES_SKEL]
        if(N1>0):
            ypred_obj = ypred[0][index_jump : index_jump + N1*NUM_CLASSES_OBJ]
        if(N2>0):
            ypred_skel = ypred[0][index_jump + N1*NUM_CLASSES_OBJ: index_jump + N1*NUM_CLASSES_OBJ+N2*NUM_CLASSES_SKEL]
        obj_res = []
        skel_res = []
        if(N1>0):
            obj_res = evaluation_class_pr_sum1(y_obj, ypred_obj, NUM_CLASSES_OBJ , N1, sparm)
        if(N2>0):
            skel_res = evaluation_class_pr_sum1(y_skel, ypred_skel, NUM_CLASSES_SKEL , N2, sparm)
        teststats.append((obj_res,skel_res))
        index_jump += N1*K1+N2*K2+ E1*K1*K1 + E2*K1*K2
    return teststats

def print_testing_stats_objects( K, teststats):

    avgp = zeros((K,1))
    avgr = zeros((K,1))
    tpcount = zeros((K,1))
    truecount = zeros((K,1))
    predcount = zeros((K,1))
    singlepredcount = zeros((K,1))
    aggConfusionMatrix=zeros((K,K),dtype='i')
    aggConfusionMatrixWMultiple=zeros((K,K),dtype='i')
    aggZeroPreds=zeros((K,1))
    aggMultiplePreds=zeros((K,1))
    for t in teststats:
        if (len(t) == 0):
            return
        tpcount += t[0]
        truecount += t[1]
        predcount += t[2]
        singlepredcount += t[7]
        aggConfusionMatrix = aggConfusionMatrix + t[3];
        aggZeroPreds +=t[4];
        aggMultiplePreds +=t[5];
        aggConfusionMatrixWMultiple = aggConfusionMatrixWMultiple + t[6];
#pointer to changes

    total_tc  = 0
    total_pc = 0
    total_tp = 0
    for label in xrange(0,K):
        if(singlepredcount[label,0] != 0):
            avgp[label,0] = tpcount[label,0]/float(singlepredcount[label,0])
        if(truecount[label,0] !=0):
            avgr[label,0] = tpcount[label,0]/float(truecount[label,0])
        #print "label ",label+1, " prec: " , avgp[label,0], " recall: " ,avgr[label,0], " tp: ", tpcount[label,0], " tc: ", truecount[label,0], " pc: ", singlepredcount[label,0]
        total_tc +=  truecount[label,0]
        total_pc += singlepredcount[label,0]
        total_tp += tpcount[label,0]
    print "prec: ", total_tp/total_pc , "recall: ",total_tp/total_tc ,"tp: ", total_tp, " pc: ", total_pc, "tc: ", total_tc
    print "confusion matrix:"
    print aggConfusionMatrix;
    savetxt('conf.txt',aggConfusionMatrix,fmt='%d');

    print "confusion matrix with multiple semantics:"
    print aggConfusionMatrixWMultiple;
    savetxt('confm.txt',aggConfusionMatrixWMultiple,fmt='%d');

    print "num Zeros:"
    print aggZeroPreds;

    print "num Multiples:"
    print aggMultiplePreds;


def print_testing_stats(sample, sm, sparm, teststats):
    """#print statistics once classification has finished.

    This is called after all test predictions are made to allow the
    display of any summary statistics that have been accumulated in
    the teststats object through use of the eval_prediction function.

    The default behavior is that nothing is #printed."""

    teststats_obj = [a[0] for a in teststats]
    teststats_skel = [a[1] for a in teststats]
    if(len(teststats_obj[0])>0):
        print "Object detection : "
        print_testing_stats_objects(NUM_CLASSES_OBJ,teststats_obj )
    print "Activity detection : "
    print_testing_stats_objects(NUM_CLASSES_SKEL, teststats_skel)
