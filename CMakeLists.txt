cmake_minimum_required(VERSION 2.8.3)
project(aloam_velodyne)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

set(Pangolin_DIR "${PROJECT_SOURCE_DIR}/3rdParty/lib/cmake/Pangolin")
set(Ceres_DIR "${PROJECT_SOURCE_DIR}/3rdParty/lib/cmake/Ceres")

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  sensor_msgs
  roscpp
  rospy
  rosbag
  std_msgs
  image_transport
  cv_bridge
  tf
)

#find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Ceres REQUIRED)

include_directories(
  include
	${catkin_INCLUDE_DIRS} 
	${PCL_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS})

catkin_package(
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs
  DEPENDS EIGEN3 PCL 
  INCLUDE_DIRS include
)


add_executable(ascanRegistration src/scanRegistration.cpp)
target_link_libraries(ascanRegistration ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(ascanRegistration_bpearl src/scanRegistration_bpearl.cpp)
target_link_libraries(ascanRegistration_bpearl ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(alaserOdometry src/laserOdometry.cpp)
target_link_libraries(alaserOdometry ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${CERES_LIBRARIES})

add_executable(alaserMapping src/laserMapping.cpp)
target_link_libraries(alaserMapping ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${CERES_LIBRARIES})

add_executable(kittiHelper src/kittiHelper.cpp)
target_link_libraries(kittiHelper ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS})

find_package(Pangolin REQUIRED)
include_directories(${Pangolin_INCLUDE_DIRS})

add_executable(visualizer
		src/Visualization_node.cpp
		src/RosVisualization.cpp)
target_link_libraries(visualizer ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${Pangolin_LIBRARIES} ${OpenCV_LIBS})


#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## 添加可执行文件或者库文件，Mark executables and/or libraries for installation
install(TARGETS ascanRegistration alaserOdometry alaserMapping visualizer
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES
        ${PROJECT_SOURCE_DIR}/3rdParty/lib/libceres.so
        ${PROJECT_SOURCE_DIR}/3rdParty/lib/libceres.so.1
        ${PROJECT_SOURCE_DIR}/3rdParty/lib/libceres.so.1.14.0
        ${PROJECT_SOURCE_DIR}/3rdParty/lib/libpangolin.so
  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

## 添加资源文件．Mark other files for installation (e.g. launch and bag files, etc.)
install(FILES
launch/aloam_zhkj_robosense_bpearl5.launch
DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

