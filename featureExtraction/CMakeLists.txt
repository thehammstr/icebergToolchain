CMAKE_MINIMUM_REQUIRED(VERSION 2.8)

PROJECT(extractIcebergFeatures_project)

SET(LOCALINCLUDE_DIR "../Generic" CACHE STRING "My include dir
You may add additional search paths here. Use ; to separate multiple
paths.") 

find_package( OpenCV REQUIRED )

FIND_PACKAGE(Ceres REQUIRED )
find_package(PCL 1.8 REQUIRED)
INCLUDE_DIRECTORIES(${CERES_INCLUDE_DIRS})
INCLUDE_DIRECTORIES(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
# Google flags
find_library(GFLAGS_LIB gflags REQUIRED)
find_path(GFLAGS_INCLUDE gflags.h PATH_SUFFIXES gflags)
include_directories(${GFLAGS_INCLUDE})
include_directories(${LOCALINCLUDE_DIR})

# helloworld
ADD_EXECUTABLE(extractFeatures extractFeaturesFromCleanedData.cc ../Generic/NodeDefinitions.cpp  cloudManipulation.cpp)
ADD_EXECUTABLE(testHarness testHarness.cc ../Generic/NodeDefinitions.cpp  cloudManipulation.cpp )
TARGET_LINK_LIBRARIES(extractFeatures ${CERES_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS} gflags)
TARGET_LINK_LIBRARIES(testHarness ${CERES_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS} gflags)
