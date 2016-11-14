cmake_minimum_required (VERSION 2.6 FATAL_ERROR)

project      (QTPCL)

find_package (Qt4 REQUIRED)
find_package (VTK REQUIRED)
find_package (PCL 1.7 REQUIRED)

include_directories (${PCL_INCLUDE_DIRS})
link_directories    (${PCL_LIBRARY_DIRS})
add_definitions     (${PCL_DEFINITIONS})


set (CMAKE_CXX_STANDARD 11)
set (CMAKE_AUTORCC ON)
set  (project_SOURCES main.cpp  viewer.cpp )
set  (project_HEADERS viewer.h )
set  (project_FORMS   viewer.ui )
set  (VTK_LIBRARIES   vtkRendering vtkGraphics vtkHybrid QVTK)



QT4_WRAP_CPP (project_HEADERS_MOC   ${project_HEADERS})
QT4_WRAP_UI  (project_FORMS_HEADERS ${project_FORMS})
INCLUDE         (${QT_USE_FILE})
ADD_DEFINITIONS (${QT_DEFINITIONS})

ADD_EXECUTABLE  (QTPCL ${project_SOURCES}
                                ${project_FORMS_HEADERS}
                                ${project_HEADERS_MOC} qrc.qrc)

TARGET_LINK_LIBRARIES (QTPCL ${QT_LIBRARIES} ${PCL_LIBRARIES} ${VTK_LIBRARIES})


