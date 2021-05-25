cmake_minimum_required(VERSION 3.0.2)
project(struct_slam)

set(CMAKE_BUILD_TYPE Debug)
## Compile as C++11, supported in ROS Kinetic and newer
 add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  cv_bridge
  message_filters
  pcl_ros
  geometry_msgs
  std_msgs
  dynamic_reconfigure
  message_generation
  sensor_msgs
  tf
)

find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)

##link_directories(${PCL_LIBRARY_DIRS})


## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
add_message_files(
   FILES
   plm.msg
   plm0.msg
   waku.msg
   wakuhairetu.msg
   ImageMatchingData.msg
   MaskImageData.msg
   ImageFlow2D.msg
 )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs 
   geometry_msgs 
   sensor_msgs   # Or other packages containing msgs
 )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
 generate_dynamic_reconfigure_options(
   cfg/Depth_pcl.cfg
   cfg/imageMatching.cfg
 )


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(

CATKIN_DEPENDS roscpp sensor_msgs message_runtime 


#  LIBRARIES Struct_SLAM
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

##link_directories(${PCL_LIBRARY_DIRS})

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/Struct_SLAM.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/Struct_SLAM_node.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# catkin_install_python(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_Struct_SLAM.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

add_executable(Hough src/Hough.cpp)
target_link_libraries(Hough ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(Hough_test src/Hough_test.cpp)
target_link_libraries(Hough_test ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(Hough_camera src/Hough_camera.cpp)
target_link_libraries(Hough_camera ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(Sobel_edge src/Sobel_edge.cpp)
target_link_libraries(Sobel_edge ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(HoughP_camera src/HoughP_camera.cpp)
target_link_libraries(HoughP_camera ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(HoughP_Hough_camera src/HoughP_Hough_camera.cpp)
target_link_libraries(HoughP_Hough_camera ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(HoughP_depth src/HoughP_depth.cpp)
target_link_libraries(HoughP_depth ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(HoughP_depth1_Ver2 src/HoughP_depth1_Ver2.cpp)
target_link_libraries(HoughP_depth1_Ver2 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(HoughP_depth1_Ver2_1 src/HoughP_depth1_Ver2_1.cpp)
target_link_libraries(HoughP_depth1_Ver2_1 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(HoughP_depth1_Ver2_2 src/HoughP_depth1_Ver2_2.cpp)
target_link_libraries(HoughP_depth1_Ver2_2 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(HoughP_depth1_Ver2_3 src/HoughP_depth1_Ver2_3.cpp)
target_link_libraries(HoughP_depth1_Ver2_3 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(HoughP_depth1_Ver3 src/HoughP_depth1_Ver3.cpp)
target_link_libraries(HoughP_depth1_Ver3 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(testtest src/testtest.cpp)
target_link_libraries(testtest ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})


add_executable(testH src/testH.cpp)
target_link_libraries(testH ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(testH_ver2 src/testH_ver2.cpp)
target_link_libraries(testH_ver2 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(testH_ver3 src/testH_ver3.cpp)
target_link_libraries(testH_ver3 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(test_pcl src/test_pcl.cpp)
target_link_libraries(test_pcl ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(pcl src/pcl.cpp)
target_link_libraries(pcl ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})



add_executable(depth_pcl4_20201127 src/depth_pcl4_20201127.cpp)
add_dependencies(depth_pcl4_20201127 ${PROJECT_NAME}_gencfg)
add_dependencies(depth_pcl4_20201127 ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(depth_pcl4_20201127 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#20210521 FLDver
add_executable(depth_pcl4_20210521 src/depth_pcl4_20210521.cpp)
add_dependencies(depth_pcl4_20210521 ${PROJECT_NAME}_gencfg)
add_dependencies(depth_pcl4_20210521 ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(depth_pcl4_20210521 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(depth_pcl4_sub_20201127 src/depth_pcl4_sub_20201127.cpp)
add_dependencies(depth_pcl4_sub_20201127 ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(depth_pcl4_sub_20201127 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(HoughP_pcl5 src/HoughP_pcl5.cpp)
add_dependencies(HoughP_pcl5 ${PROJECT_NAME}_gencfg)
add_dependencies(HoughP_pcl5 ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(HoughP_pcl5 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(HoughP_pcl5_sub src/HoughP_pcl5_sub.cpp)
add_dependencies(HoughP_pcl5_sub ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(HoughP_pcl5_sub ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})


add_executable(test_pcl1 src/test_pcl1.cpp)
add_dependencies(test_pcl1 ${PROJECT_NAME}_gencfg)
add_dependencies(test_pcl1 ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(test_pcl1 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(test_cl src/test_cl.cpp)
add_dependencies(test_cl ${PROJECT_NAME}_gencfg)
add_dependencies(test_cl ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(test_cl ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(basic_simple_talker src/basic_simple_talker.cpp)
target_link_libraries(basic_simple_talker ${catkin_LIBRARIES})

add_executable(basic_simple_talker1 src/basic_simple_talker1.cpp)
target_link_libraries(basic_simple_talker1 ${catkin_LIBRARIES})

add_executable(basic_simple_listener src/basic_simple_listener.cpp)
target_link_libraries(basic_simple_listener ${catkin_LIBRARIES})

add_executable(info_marker_publisher1 src/info_marker_publisher1.cpp)
target_link_libraries(info_marker_publisher1 ${catkin_LIBRARIES})

add_executable(points_and_lines src/points_and_lines.cpp)
target_link_libraries(points_and_lines ${catkin_LIBRARIES})

add_executable(lines src/lines.cpp)
target_link_libraries(lines ${catkin_LIBRARIES})

add_executable(lines_hough src/lines_hough.cpp)
add_dependencies(lines_hough ${PROJECT_NAME}_gencfg)
add_dependencies(lines_hough ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(lines_hough ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#20210125_test_file
add_executable(lines_hough_test src/lines_hough_test.cpp)
add_dependencies(lines_hough_test ${PROJECT_NAME}_gencfg)
add_dependencies(lines_hough_test ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(lines_hough_test ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(carib src/carib.cpp)
target_link_libraries(carib ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(srccap src/srccap.cpp)
target_link_libraries(srccap ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(findessential src/findessential.cpp)
target_link_libraries(findessential ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(findessential2 src/findessential2.cpp)
target_link_libraries(findessential2 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(findessential3 src/findessential3.cpp)
target_link_libraries(findessential3 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(findessential3_2 src/findessential3_2.cpp)
target_link_libraries(findessential3_2 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#add_executable(findessential4 src/findessential4.cpp)
#target_link_libraries(findessential4 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#add_executable(SfM src/smp/SfM.cpp)
#target_link_libraries(SfM ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(${PROJECT_NAME}_im # image matching
   src/imageMatching/constructor.cpp
   src/imageMatching/methods.cpp
   src/imageMatching/debug_main.cpp
   src/imageMatching/debug_methods.cpp
   src/imageMatching/property.cpp
src/imageMatching/property.cpp
 )
add_dependencies(${PROJECT_NAME}_im ${TARGET_NAME}${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(${PROJECT_NAME}_im ${PROJECT_NAME}_gencfg)
target_link_libraries(${PROJECT_NAME}_im ${catkin_LIBRARIES} ${OpenCV_LIBRARIES}) # image matching

#20210121
add_executable(lines_prm src/lines_prm.cpp)
add_dependencies(lines_prm ${PROJECT_NAME}_gencfg)
add_dependencies(lines_prm ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(lines_prm ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#20210125
add_executable(lines_prm2 src/lines_prm2.cpp)
add_dependencies(lines_prm2 ${PROJECT_NAME}_gencfg)
add_dependencies(lines_prm2 ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(lines_prm2 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#20210208 copy of lines_hough
add_executable(lines_hough3 src/lines_hough3.cpp)
add_dependencies(lines_hough3 ${PROJECT_NAME}_gencfg)
add_dependencies(lines_hough3 ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(lines_hough3 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#20210210 copy of depth_pcl4_20201127
add_executable(depth_pcl4_test src/depth_pcl4_test.cpp)
add_dependencies(depth_pcl4_test ${PROJECT_NAME}_gencfg)
add_dependencies(depth_pcl4_test ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(depth_pcl4_test ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#20210210 copy of depth_pcl4_20201127 and lines_hough3
add_executable(depth_pcl4_lines_hough3 src/depth_pcl4_lines_hough3.cpp)
add_dependencies(depth_pcl4_lines_hough3 ${PROJECT_NAME}_gencfg)
add_dependencies(depth_pcl4_lines_hough3 ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(depth_pcl4_lines_hough3 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#20210217 copy of lines_hough3 next ver
#add_executable(lines_hough4 src/lines_hough4.cpp)
#add_dependencies(lines_hough4 ${PROJECT_NAME}_gencfg)
#add_dependencies(lines_hough4 ${PROJECT_NAME}_generate_messages_cpp)
#target_link_libraries(lines_hough4 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#20210223 copy of lines_hough3 next ver
add_executable(lines_hough5 src/lines_hough5.cpp)
add_dependencies(lines_hough5 ${PROJECT_NAME}_gencfg)
add_dependencies(lines_hough5 ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(lines_hough5 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#20210224 LSD 
add_executable(20210224_LSD1 src/20210224_LSD1.cpp src/lsd.h src/lsd.c)
add_dependencies(20210224_LSD1 ${PROJECT_NAME}_gencfg)
add_dependencies(20210224_LSD1 ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(20210224_LSD1 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#20210226 FLD(Fast Line Detector) 
add_executable(20210226_FLD1 src/20210226_FLD1.cpp)
add_dependencies(20210226_FLD1 ${PROJECT_NAME}_gencfg)
add_dependencies(20210226_FLD1 ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(20210226_FLD1 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#20210226 FLD(Fast Line Detector) 
add_executable(20210226_FLD_camera src/20210226_FLD_camera.cpp)
add_dependencies(20210226_FLD_camera ${PROJECT_NAME}_gencfg)
add_dependencies(20210226_FLD_camera ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(20210226_FLD_camera ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#20210226 FLD(Fast Line Detector) copy
add_executable(20210226_FLD_camera2 src/FLD/20210226_FLD_camera2.cpp)
add_dependencies(20210226_FLD_camera2 ${PROJECT_NAME}_gencfg)
add_dependencies(20210226_FLD_camera2 ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(20210226_FLD_camera2 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#20210226 FLD(Fast Line Detector) copy
add_executable(20210226_FLD_camera_Hough src/FLD/20210226_FLD_camera_Hough.cpp)
add_dependencies(20210226_FLD_camera_Hough ${PROJECT_NAME}_gencfg)
add_dependencies(20210226_FLD_camera_Hough ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(20210226_FLD_camera_Hough ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#20210227 FLD(Fast Line Detector) copy
add_executable(20210226_FLD_camera_Hough2 src/FLD/20210226_FLD_camera_Hough2.cpp)
add_dependencies(20210226_FLD_camera_Hough2 ${PROJECT_NAME}_gencfg)
add_dependencies(20210226_FLD_camera_Hough2 ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(20210226_FLD_camera_Hough2 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#20210227 FLD(Fast Line Detector) copy
add_executable(20210226_FLD_camera_Hough3 src/FLD/20210226_FLD_camera_Hough3.cpp)
add_dependencies(20210226_FLD_camera_Hough3 ${PROJECT_NAME}_gencfg)
add_dependencies(20210226_FLD_camera_Hough3 ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(20210226_FLD_camera_Hough3 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#20210420 saisyounizyouhou
add_executable(L2_sp src/L2_sp3/L2_sp.cpp)
add_dependencies(L2_sp ${PROJECT_NAME}_gencfg)
add_dependencies(L2_sp ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(L2_sp ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#20210420 saisyounizyouhou_testVer
#add_executable(L2_sp_test src/L2_sp3/L2_sp_test.cpp)
#add_dependencies(L2_sp_test ${PROJECT_NAME}_gencfg)
#add_dependencies(L2_sp_test ${PROJECT_NAME}_generate_messages_cpp)
#target_link_libraries(L2_sp_test ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#20210422 test_Eigen
add_executable(test_Eigen src/test_Eigen.cpp)
add_dependencies(test_Eigen ${PROJECT_NAME}_gencfg)
add_dependencies(test_Eigen ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(test_Eigen ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#20210426 saisyou zizyouhou _test_3honnVer
add_executable(20210426_L2_sp3_test src/L2_sp3/20210426_L2_sp3_test.cpp)
add_dependencies(20210426_L2_sp3_test ${PROJECT_NAME}_gencfg)
add_dependencies(20210426_L2_sp3_test ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(20210426_L2_sp3_test ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#20210429 test_Eigen2
add_executable(test_Eigen2 src/test_Eigen2.cpp)
add_dependencies(test_Eigen2 ${PROJECT_NAME}_gencfg)
add_dependencies(test_Eigen2 ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(test_Eigen2 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#20210503 test_Eigen2 copy
add_executable(test_Eigen3 src/test_Eigen3.cpp)
add_dependencies(test_Eigen3 ${PROJECT_NAME}_gencfg)
add_dependencies(test_Eigen3 ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(test_Eigen3 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#20210503 20210426_L2_sp3_test copy 
add_executable(20210503_L2_sp3_test src/L2_sp3/20210503_L2_sp3_test.cpp)
add_dependencies(20210503_L2_sp3_test ${PROJECT_NAME}_gencfg)
add_dependencies(20210503_L2_sp3_test ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(20210503_L2_sp3_test ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#20210503 test_Eigen_saisyou copy
add_executable(test_Eigen_saisyou src/test_Eigen_saisyou.cpp)
add_dependencies(test_Eigen_saisyou ${PROJECT_NAME}_gencfg)
add_dependencies(test_Eigen_saisyou ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(test_Eigen_saisyou ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#20210507 20210226_FLD_camera_Hough3 copy
add_executable(20210507_FLD_camera_Hough3 src/FLD/20210507_FLD_camera_Hough3.cpp)
add_dependencies(20210507_FLD_camera_Hough3 ${PROJECT_NAME}_gencfg)
add_dependencies(20210507_FLD_camera_Hough3 ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(20210507_FLD_camera_Hough3 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#20210510 20210503_L2_sp3_test copy Ver_test
add_executable(20210510_L2_sp3_test src/L2_sp3/20210510_L2_sp3_test.cpp)
add_dependencies(20210510_L2_sp3_test ${PROJECT_NAME}_gencfg)
add_dependencies(20210510_L2_sp3_test ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(20210510_L2_sp3_test ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#20210510 20210510_L2_sp3_test copy Ver_realtime 
add_executable(20210510_L2_sp3_realt src/L2_sp3/20210510_L2_sp3_realt.cpp)
add_dependencies(20210510_L2_sp3_realt ${PROJECT_NAME}_gencfg)
add_dependencies(20210510_L2_sp3_realt ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(20210510_L2_sp3_realt ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#20210511 20210510_L2_sp3_test copy Ver_test2
add_executable(20210511_L2_sp3_test src/L2_sp3/20210511_L2_sp3_test.cpp)
add_dependencies(20210511_L2_sp3_test ${PROJECT_NAME}_gencfg)
add_dependencies(20210511_L2_sp3_test ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(20210511_L2_sp3_test ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#20210511 20210507_FLD_camera_Hough3 copy
add_executable(20210511_FLD_camera_Hough3 src/FLD/20210511_FLD_camera_Hough3.cpp)
add_dependencies(20210511_FLD_camera_Hough3 ${PROJECT_NAME}_gencfg)
add_dependencies(20210511_FLD_camera_Hough3 ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(20210511_FLD_camera_Hough3 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#20210517 20210511_FLD_camera_Hough3 copy ver_depth
add_executable(20210517_FLD_depth src/FLD/20210517_FLD_depth.cpp)
add_dependencies(20210517_FLD_depth ${PROJECT_NAME}_gencfg)
add_dependencies(20210517_FLD_depth ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(20210517_FLD_depth ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#20210518 lines_prm2 copy VerFLD+HoughP
add_executable(lines_prm3 src/lines_prm3.cpp)
add_dependencies(lines_prm3 ${PROJECT_NAME}_gencfg)
add_dependencies(lines_prm3 ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(lines_prm3 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#20210519 RealsenseD435 Depth adjustment program
add_executable(D435_test src/test/D435_test.cpp)
add_dependencies(D435_test ${PROJECT_NAME}_gencfg)
add_dependencies(D435_test ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(D435_test ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#20210520 20210517_FLD_depth + D435_test
add_executable(20210520_FLD_depth2 src/FLD/20210520_FLD_depth2.cpp)
add_dependencies(20210520_FLD_depth2 ${PROJECT_NAME}_gencfg)
add_dependencies(20210520_FLD_depth2 ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(20210520_FLD_depth2 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#20210521 lines_prm3 copy Ver lines_prm3 + D435_test
#add_executable(lines_prm4 src/prm/lines_prm4.cpp)
#add_dependencies(lines_prm4 ${PROJECT_NAME}_gencfg)
#add_dependencies(lines_prm4 ${PROJECT_NAME}_generate_messages_cpp)
#target_link_libraries(lines_prm4 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#20210521 lines_prm4 copy Ver OpenCV->Eigen
add_executable(lines_prm5 src/prm/lines_prm5.cpp)
add_dependencies(lines_prm5 ${PROJECT_NAME}_gencfg)
add_dependencies(lines_prm5 ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(lines_prm5 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#20210524 lines_prm6 copy Ver lines_prm5 + FLD(tate,yoko,naname)
add_executable(lines_prm6 src/prm/lines_prm6.cpp)
add_dependencies(lines_prm6 ${PROJECT_NAME}_gencfg)
add_dependencies(lines_prm6 ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(lines_prm6 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#20210524 20210520_FLD_depth copy Ver A[][][][]
add_executable(20210524_FLD_depth2 src/FLD/20210524_FLD_depth2.cpp)
add_dependencies(20210524_FLD_depth2 ${PROJECT_NAME}_gencfg)
add_dependencies(20210524_FLD_depth2 ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(20210524_FLD_depth2 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#20210525 lines_prm6 copy Ver Eigen
add_executable(lines_prm7 src/prm/lines_prm7.cpp)
add_dependencies(lines_prm7 ${PROJECT_NAME}_gencfg)
add_dependencies(lines_prm7 ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(lines_prm7 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
