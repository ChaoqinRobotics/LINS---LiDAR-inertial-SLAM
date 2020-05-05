cmake_minimum_required(VERSION 2.8.3)
project(lins)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3")

find_package(catkin REQUIRED COMPONENTS
    cloud_msgs
    cv_bridge
    geometry_msgs
    image_transport
    nav_msgs
    pcl_conversions
    pcl_ros
    roscpp
    rospy
    sensor_msgs
    std_msgs
    tf
)

find_package(GTSAM REQUIRED QUIET)
find_package(PCL REQUIRED QUIET)
find_package(OpenCV REQUIRED QUIET)
find_package(Eigen3 REQUIRED)

catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS cloud_msgs
    DEPENDS PCL
)


include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${GTSAM_INCLUDE_DIR}
    ${EIGEN3_INCLUDE_DIR}
)

link_directories(
    include
    ${OpenCV_LIBRARY_DIRS}
    ${PCL_LIBRARY_DIRS}
    ${GTSAM_LIBRARY_DIRS}
)


list(APPEND SOURCE_FILES
    ${PROJECT_SOURCE_DIR}/src/lib/parameters.cpp
)

list(APPEND LINS_FILES
    ${PROJECT_SOURCE_DIR}/src/lins_fusion_node.cpp
    ${PROJECT_SOURCE_DIR}/src/lib/Estimator.cpp
)

list(APPEND LINK_LIBS 
    ${OpenCV_LIBS}
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
    ${OpenCV_LIBRARY_DIRS}
)

add_executable(lins_fusion_node ${LINS_FILES} ${SOURCE_FILES})
target_link_libraries(lins_fusion_node ${LINK_LIBS})

add_executable(image_projection_node src/image_projection_node.cpp ${SOURCE_FILES})
add_dependencies(image_projection_node ${catkin_EXPORTED_TARGETS} cloud_msgs_gencpp)
target_link_libraries(image_projection_node ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(lidar_mapping_node src/lidar_mapping_node.cpp ${SOURCE_FILES})
target_link_libraries(lidar_mapping_node ${LINK_LIBS} gtsam)

add_executable(transform_fusion_node src/transform_fusion_node.cpp ${SOURCE_FILES})
target_link_libraries(transform_fusion_node ${LINK_LIBS})
