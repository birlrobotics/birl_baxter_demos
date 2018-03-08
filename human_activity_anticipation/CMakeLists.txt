cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
project(MY_GRAND_PROJECT)

SET( CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -g -Wall -O0" )
SET( CMAKE_C_FLAGS  "${CMAKE_C_FLAGS} -g -Wall -O0" )

find_package(PCL 1.6 REQUIRED COMPONENTS common io search)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


#add_executable(pcd_write_test src/pcd_write.cpp)
#add_executable(reachabilityMap src/reachabilityMap.cpp)


#target_link_libraries(pcd_write_test ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES})
#target_link_libraries(reachabilityMap ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES})


set(SVM_LIGHT_SRC_DIR "src/svm_light")

INCLUDE_DIRECTORIES(${SVM_LIGHT_SRC_DIR})
ADD_LIBRARY(
    svm_lib_light
    ${SVM_LIGHT_SRC_DIR}/svm_learn.c
    ${SVM_LIGHT_SRC_DIR}/svm_common.c
    ${SVM_LIGHT_SRC_DIR}/svm_hideo.c
)

set(SVM_STRUCT_SRC_DIR "src/svm_struct")

INCLUDE_DIRECTORIES(${SVM_STRUCT_SRC_DIR})

ADD_LIBRARY(
    svm_lib_struct
    ${SVM_STRUCT_SRC_DIR}/svm_struct_common.c
    ${SVM_STRUCT_SRC_DIR}/svm_struct_main.c
    ${SVM_STRUCT_SRC_DIR}/svm_struct_classify_stream.c
)

FIND_PACKAGE(PythonInterp)
FIND_PACKAGE(PythonLibs)
INCLUDE_DIRECTORIES(${PYTHON_INCLUDE_PATH})
INCLUDE_DIRECTORIES("/usr/local/include")
set(SVM_PY_SRC_DIR "src/pyobjs/")

INCLUDE_DIRECTORIES(${SVM_PY_SRC_DIR})

ADD_LIBRARY(
    svm_lib_py
    ${SVM_PY_SRC_DIR}/array.c
    ${SVM_PY_SRC_DIR}/default.c
    ${SVM_PY_SRC_DIR}/kernelparm.c
    ${SVM_PY_SRC_DIR}/sample.c
    ${SVM_PY_SRC_DIR}/sparse.c
    ${SVM_PY_SRC_DIR}/svmapi.c
    ${SVM_PY_SRC_DIR}/constraints.c
    ${SVM_PY_SRC_DIR}/document.c
    ${SVM_PY_SRC_DIR}/model.c
    ${SVM_PY_SRC_DIR}/sparm.c
    ${SVM_PY_SRC_DIR}/structmodel.c
)


set(SVM_API_SRC_DIR "src/")

INCLUDE_DIRECTORIES(${SVM_API_SRC_DIR})

ADD_LIBRARY(
    svm_lib_api
    ${SVM_API_SRC_DIR}/svm_struct_api.c
    ${SVM_API_SRC_DIR}/svm_struct_api.h
    ${SVM_API_SRC_DIR}/svm_struct_api_types.h
    ${SVM_API_SRC_DIR}/svm_struct/svm_struct_common.h
)
add_executable (predict src/streamingPrediction.cpp)
add_dependencies(predict svm_lib_api )
add_executable (predict_seg src/streamingPrediction_segmented.cpp)
add_dependencies(predict_seg svm_lib_api )


add_executable (frameFeats src/featureGenerationForFramesSampled.cpp)

FIND_PACKAGE( OpenCV REQUIRED )

TARGET_LINK_LIBRARIES( svm_lib_struct svm_lib_api svm_lib_light svm_lib_py ${PYTHON_LIBRARIES})
TARGET_LINK_LIBRARIES( svm_lib_api svm_lib_light svm_lib_py ${PYTHON_LIBRARIES})
TARGET_LINK_LIBRARIES(predict svm_lib_api svm_lib_struct svm_lib_light svm_lib_py ${PYTHON_LIBRARIES} ${OpenCV_LIBS} ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} ${PCL_KDTREE_LIBRARY} ${PCL_SEARCH_LIBRARY} ${PCL_SURFACE_LIBRARY} ${PCL_FEATURES_LIBRARY})
TARGET_LINK_LIBRARIES(predict_seg svm_lib_api svm_lib_struct svm_lib_light svm_lib_py ${PYTHON_LIBRARIES} ${OpenCV_LIBS} ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} ${PCL_KDTREE_LIBRARY} ${PCL_SEARCH_LIBRARY} ${PCL_SURFACE_LIBRARY} ${PCL_FEATURES_LIBRARY})
TARGET_LINK_LIBRARIES(frameFeats ${PCL_LIBRARIES}  ${OpenCV_LIBS})
