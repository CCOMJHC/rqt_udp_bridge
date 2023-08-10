cmake_minimum_required(VERSION 2.8.3)
project(rqt_udp_bridge)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    roslint
    rqt_gui
    rqt_gui_cpp
    udp_bridge
)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

if(CMAKE_VERSION VERSION_LESS "3.7.0")
  set(CMAKE_INCLUDE_CURRENT_DIR ON)
endif()

find_package(Qt5 COMPONENTS Widgets REQUIRED)

roslint_cpp()

set(rqt_udp_bridge_SRCS
  src/udp_bridge_plugin.cpp
  src/bridge_node.cpp
)

set(rqt_udp_bridge_HDRS
  include/rqt_udp_bridge/udp_bridge_plugin.h
  include/rqt_udp_bridge/bridge_node.h
)

set(rqt_udp_bridge_UIS
  src/udp_bridge_plugin.ui
  src/add_remote_dialog.ui
  src/subscribe_dialog.ui
)

set(rqt_udp_bridge_INCLUDE_DIRECTORIES
  include
  "${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION}"
)
if(NOT EXISTS "${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION}")
  file(MAKE_DIRECTORY "${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION}")
endif()

catkin_package(
  INCLUDE_DIRS ${rqt_udp_bridge_INCLUDE_DIRECTORIES}
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp rqt_gui_cpp rqt_gui udp_bridge
#  DEPENDS system_lib
)

qt5_wrap_cpp(rqt_udp_bridge_MOCS ${rqt_udp_bridge_HDRS})

set(_cmake_current_binary_dir "${CMAKE_CURRENT_BINARY_DIR}")
set(CMAKE_CURRENT_BINARY_DIR "${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION}")
qt5_wrap_ui(rqt_udp_bridge_UIS_H ${rqt_udp_bridge_UIS})
set(CMAKE_CURRENT_BINARY_DIR "${_cmake_current_binary_dir}")

include_directories(
    ${rqt_udp_bridge_INCLUDE_DIRECTORIES}
    ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
    ${rqt_udp_bridge_SRCS}
    ${rqt_udp_bridge_MOCS}
    ${rqt_udp_bridge_UIS_H}
)

qt5_use_modules(${PROJECT_NAME} Widgets)

target_link_libraries(${PROJECT_NAME}
    ${QT_LIBRARIES}
    ${catkin_LIBRARIES}
)

find_package(class_loader)
class_loader_hide_library_symbols(${PROJECT_NAME})


install(FILES
  plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
