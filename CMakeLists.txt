cmake_minimum_required (VERSION 2.6 FATAL_ERROR)

project      (QTPCL)

find_package (Qt5Widgets REQUIRED)
find_package (VTK REQUIRED)
find_package (PCL REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")
#include(${VTK_USE_FILE})

set (CMAKE_CXX_STANDARD 11)

set  (project_SOURCES main.cpp  viewer.cpp interactor_style_actor.cpp)
set  (project_HEADERS viewer.h  interactor_style_actor.h)
set  (project_FORMS   viewer.ui )

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)
#INCLUDE         (${QT_USE_FILE})
ADD_DEFINITIONS (${QT_DEFINITIONS})

ADD_EXECUTABLE  (QTPCL ${project_SOURCES}
                                ${project_FORMS_HEADERS}
                                ${project_HEADERS_MOC} qrc.qrc)

TARGET_LINK_LIBRARIES (QTPCL ${QT_LIBRARIES} ${PCL_LIBRARIES})


