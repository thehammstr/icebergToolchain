cmake_minimum_required (VERSION 2.6 FATAL_ERROR)

project      (manualCorrespondenceGUI)

# Find includes in corresponding build directories
#set(CMAKE_INCLUDE_CURRENT_DIR ON)
# Instruct CMake to run moc automatically when needed.
#set(CMAKE_AUTOMOC ON)

# Find the QtWidgets library
find_package (Qt4  REQUIRED)
find_package (VTK REQUIRED)
find_package( OpenCV 2.4 REQUIRED )
find_package (PCL 1.7 REQUIRED)
FIND_PACKAGE (Ceres REQUIRED)

include_directories ("../../Generic")
include_directories ("../../ceres")
INCLUDE_DIRECTORIES(${CERES_INCLUDE_DIRS})
include_directories (${OpenCV_INCLUDE_DIRS})
link_directories (${OpenCV_LIBRARY_DIRS})
add_definitions (${OpenCV_DEFINITIONS})
include_directories (${PCL_INCLUDE_DIRS} "../../Generic")
link_directories    (${PCL_LIBRARY_DIRS})
add_definitions     (${PCL_DEFINITIONS})
# Google flags
find_library(GFLAGS_LIB gflags REQUIRED)
find_path(GFLAGS_INCLUDE gflags.h PATH_SUFFIXES gflags)
include_directories(${GFLAGS_INCLUDE})
include_directories(${LOCALINCLUDE_DIR})
set  (project_SOURCES main.cpp corrGUI.cpp ../../Generic/NodeDefinitions.cpp ../../Generic/cloudManipulation.cpp)
set  (project_HEADERS corrGUI.h)
set  (project_FORMS   corrGUI.ui)
set  (VTK_LIBRARIES   vtkRendering vtkGraphics vtkHybrid QVTK)

QT4_WRAP_CPP (project_HEADERS_MOC   ${project_HEADERS})
QT4_WRAP_UI  (project_FORMS_HEADERS ${project_FORMS})

INCLUDE         (${QT_USE_FILE})
ADD_DEFINITIONS (${QT_DEFINITIONS})

ADD_EXECUTABLE  (correspondenceGUI ${project_SOURCES}
                                ${project_FORMS_HEADERS}
                                ${project_HEADERS_MOC})

TARGET_LINK_LIBRARIES (correspondenceGUI ${QT_LIBRARIES} ${CERES_LIBRARIES} ${PCL_LIBRARIES} ${VTK_LIBRARIES} ${OpenCV_LIBS} gflags)

