cmake_minimum_required (VERSION 2.6 FATAL_ERROR)

project      (manualCorrespondenceGUI)

# Find includes in corresponding build directories
#set(CMAKE_INCLUDE_CURRENT_DIR ON)
# Instruct CMake to run moc automatically when needed.
#set(CMAKE_AUTOMOC ON)

# Find the QtWidgets library
find_package (Qt4  REQUIRED)
find_package (VTK REQUIRED)
find_package (PCL 1.7 REQUIRED)

include_directories (${PCL_INCLUDE_DIRS})
link_directories    (${PCL_LIBRARY_DIRS})
add_definitions     (${PCL_DEFINITIONS})

set  (project_SOURCES main.cpp corrGUI.cpp)
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

TARGET_LINK_LIBRARIES (correspondenceGUI ${QT_LIBRARIES} ${PCL_LIBRARIES} ${VTK_LIBRARIES})

