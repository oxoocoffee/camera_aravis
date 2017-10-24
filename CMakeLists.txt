cmake_minimum_required(VERSION 2.8.3)
project(camera_aravis)

#Add custom (non compiling) targets so launch scripts and python files show up in QT Creator's project view.
file(GLOB EXTRA_FILES *.xml scripts/*.py launch/*.launch include/*/*.h action/*.action msg/*.msg)
add_custom_target(${PROJECT_NAME}_OTHER_FILES ALL WORKING_DIRECTORY ${PROJECT_SOURCE_DIR} SOURCES ${EXTRA_FILES})

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules)
find_package(catkin REQUIRED COMPONENTS
              roscpp
              sensor_msgs
              image_transport
              camera_info_manager
              dynamic_reconfigure
              tf
              dynamic_reconfigure
              nodelet
              nodelet_topic_tools
)
find_package(Aravis REQUIRED)
find_package(GLIB2 REQUIRED)

find_package(Boost REQUIRED COMPONENTS thread)

include_directories(include cfg/cpp
                    ${Boost_INCLUDE_DIR}
                    ${catkin_INCLUDE_DIRS}
                    ${Aravis_INCLUDE_PATH}
                    ${GLIB2_INCLUDE_DIRS}
                    )

generate_dynamic_reconfigure_options(cfg/CameraAravisConfig.cfg)


catkin_package(
    DEPENDS Aravis GLIB2
    CATKIN_DEPENDS
    roscpp
    sensor_msgs
    image_transport
    camera_info_manager
    dynamic_reconfigure
    nodelet
    nodelet_topic_tools
    tf
    INCLUDE_DIRS include
    LIBRARIES camera_nodelet
)

message(${Aravis_LIBRARIES})

add_library(camera_nodelet src/camera_nodelet.cpp)
target_link_libraries(camera_nodelet ${catkin_LIBRARIES} ${Aravis_LIBRARIES} glib-2.0 gmodule-2.0 gobject-2.0)
add_dependencies(camera_nodelet ${PROJECT_NAME}_gencfg)
class_loader_hide_library_symbols(camera_nodelet)

add_executable(camera_node src/camera_node.cpp)
target_link_libraries(camera_node camera_nodelet ${catkin_LIBRARIES} ${Aravis_LIBRARIES} glib-2.0 gmodule-2.0 gobject-2.0)
add_dependencies(camera_node camera_nodelet)

install(TARGETS camera_node
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(TARGETS camera_nodelet
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(DIRECTORY launch
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(FILES camera_aravis_plugins.xml
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
