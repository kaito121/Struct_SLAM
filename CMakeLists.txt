cmake_minimum_required(VERSION 3.0.2)
project(struct_slam)

set(CMAKE_BUILD_TYPE Debug)
## Compile as C++11, supported in ROS Kinetic and newer(C++11としてコンパイルされ、ROS Kinetic以降でサポートされています。)
 add_compile_options(-std=c++11)

## Find catkin macros and libraries(catkinのマクロとライブラリを探す)
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
##if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz) のようなCOMPONENTSリストが使われた場合、
## is used, also find other catkin packages
##他のcatkinパッケージも検索します。

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  cv_bridge
  image_transport
  message_filters
  pcl_ros
  geometry_msgs
  std_msgs
  dynamic_reconfigure
  message_generation
  sensor_msgs
  tf
  tf2
  tf2_ros
  nav_msgs
)

find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)

find_package(PkgConfig)
pkg_check_modules(RealSense2 REQUIRED realsense2)
pkg_check_modules(OpenCV REQUIRED opencv)
include_directories(${RealSense2_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
link_directories(${RealSense2_LIBRARY_DIRS} ${OpenCV_LIBRARY_DIRS})

##link_directories(${PCL_LIBRARY_DIRS})


## System dependencies are found with CMake's conventions(システムの依存性は、CMakeの規約に基づいて検出されます。)
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
## このパッケージの中でメッセージ、サービス、アクションを宣言し、構築するには、以下の手順に従います。
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * MSG_DEP_SETを、メッセージ/サービス/アクションで使用するメッセージタイプを持つパッケージのセットとします（例：std_msgs、actionlib_msgs、...）
## * In the file package.xml:
## * package.xmlというファイルの中で
##   * add a build_depend tag for "message_generation"
##   * "message_generation" の build_depend タグを追加します。
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
   ClassificationData.msg
   ClassificationElement.msg
   opt_point.msg
   marker_tf.msg

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
   nav_msgs
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
   cfg/classification.cfg
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
#CATKIN_DEPENDS roscpp rospy tf2


#  LIBRARIES Struct_SLAM
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files(ヘッダーファイルの追加場所の指定)
## Your package locations should be listed before other locations(パッケージの位置は他の場所よりも先に表示されるべき)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}

  ##------------
  #${PROJECT_SOURCE_DIR}
  #${PROJECT_SOURCE_DIR}/include/include
  #${PROJECT_SOURCE_DIR}/include/include/bm
  #${PROJECT_SOURCE_DIR}/include/include/kdtree++
  ##-------------
)

##link_directories(${PCL_LIBRARY_DIRS})

## Declare a C++ library(C++ライブラリを宣言する)
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/Struct_SLAM.cpp
# )

##----------
#set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
#
#add_library(${PROJECT_NAME} SHARED
#src/src/JLinkage.cpp
#src/src/RandomSampler.cpp
#src/src/Utilities.cpp
#src/src/VPCluster.cpp
#src/src/VPSample.cpp
#src/src/updator.cpp
#)
##--------------

## Add cmake target dependencies of the library(ライブラリのcmakeターゲット依存関係の追加)
## as an example, code may need to be generated before libraries(例として、ライブラリの前にコードを生成する必要がある場合があります)
## either from message generation or dynamic reconfigure(メッセージ生成または動的再構成のいずれかから)
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable(C++の実行ファイルを宣言する)
## With catkin_make all packages are built within a single CMake context(catkin_make では、すべてのパッケージが単一の CMake コンテキストでビルドされます。)
## The recommended prefix ensures that target names across packages don't collide(推奨プレフィックスにより、パッケージ間のターゲット名が衝突しないようにする)
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

#add_executable(Hough src/Hough.cpp)
#target_link_libraries(Hough ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#add_executable(Hough_test src/Hough_test.cpp)
#target_link_libraries(Hough_test ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#add_executable(Hough_camera src/Hough_camera.cpp)
#src/src/updator.cpp
#)
##--------------

## Add cmake target dependencies of the library(ライブラリのcmakeターゲット依存関係の追加)
## as an example, code may need to be generated before libraries(例として、ライブラリの前にコードを生成する必要がある場合があります)
## either from message generation or dynamic reconfigure(メッセージ生成または動的再構成のいずれかから)
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable(C++の実行ファイルを宣言する)
## With catkin_make all packages are built within a single CMake context(catkin_make では、すべてのパッケージが単一の CMake コンテキストでビルドされます。)
## The recommended prefix ensures that target names across packages don't collide(推奨プレフィックスにより、パッケージ間のターゲット名が衝突しないようにする)
# add_executable(${PROJECT_NAME}_node src/Struct_SLAM_node.cpp)

## Rename C++ executable without prefix
#add_executable(HoughP_Hough_camera src/HoughP_Hough_camera.cpp)
#target_link_libraries(HoughP_Hough_camera ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#add_executable(HoughP_depth src/HoughP_depth.cpp)
#target_link_libraries(HoughP_depth ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#add_executable(HoughP_depth1_Ver2 src/HoughP_depth1_Ver2.cpp)
#target_link_libraries(HoughP_depth1_Ver2 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#add_executable(HoughP_depth1_Ver2_1 src/HoughP_depth1_Ver2_1.cpp)
#target_link_libraries(HoughP_depth1_Ver2_1 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#add_executable(HoughP_depth1_Ver2_2 src/HoughP_depth1_Ver2_2.cpp)
#target_link_libraries(HoughP_depth1_Ver2_2 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#add_executable(HoughP_depth1_Ver2_3 src/HoughP_depth1_Ver2_3.cpp)
#target_link_libraries(HoughP_depth1_Ver2_3 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#add_executable(HoughP_depth1_Ver3 src/HoughP_depth1_Ver3.cpp)
#target_link_libraries(HoughP_depth1_Ver3 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#Open CV gyouretu test
#add_executable(testtest src/testtest.cpp)
#target_link_libraries(testtest ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#add_executable(testH src/testH.cpp)
#target_link_libraries(testH ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#add_executable(testH_ver2 src/testH_ver2.cpp)
#target_link_libraries(testH_ver2 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#add_executable(testH_ver3 src/testH_ver3.cpp)
#target_link_libraries(testH_ver3 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#add_executable(test_pcl src/test_pcl.cpp)
#target_link_libraries(test_pcl ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#add_executable(pcl src/pcl.cpp)
#target_link_libraries(pcl ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

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

#add_executable(HoughP_pcl5 src/HoughP_pcl5.cpp)
#add_dependencies(HoughP_pcl5 ${PROJECT_NAME}_gencfg)
#add_dependencies(HoughP_pcl5 ${PROJECT_NAME}_generate_messages_cpp)
#target_link_libraries(HoughP_pcl5 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#add_executable(HoughP_pcl5_sub src/HoughP_pcl5_sub.cpp)
#add_dependencies(HoughP_pcl5_sub ${PROJECT_NAME}_generate_messages_cpp)
#target_link_libraries(HoughP_pcl5_sub ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})


#add_executable(test_pcl1 src/test_pcl1.cpp)
#add_dependencies(test_pcl1 ${PROJECT_NAME}_gencfg)
#add_dependencies(test_pcl1 ${PROJECT_NAME}_generate_messages_cpp)
#target_link_libraries(test_pcl1 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#add_executable(test_cl src/test_cl.cpp)
#add_dependencies(test_cl ${PROJECT_NAME}_gencfg)
#add_dependencies(test_cl ${PROJECT_NAME}_generate_messages_cpp)
#target_link_libraries(test_cl ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#add_executable(basic_simple_talker src/basic_simple_talker.cpp)
#target_link_libraries(basic_simple_talker ${catkin_LIBRARIES})

#add_executable(basic_simple_talker1 src/basic_simple_talker1.cpp)
#target_link_libraries(basic_simple_talker1 ${catkin_LIBRARIES})

#dd_executable(basic_simple_listener src/basic_simple_listener.cpp)
#arget_link_libraries(basic_simple_listener ${catkin_LIBRARIES})

#add_executable(info_marker_publisher1 src/info_marker_publisher1.cpp)
#target_link_libraries(info_marker_publisher1 ${catkin_LIBRARIES})

#add_executable(points_and_lines src/points_and_lines.cpp)
#target_link_libraries(points_and_lines ${catkin_LIBRARIES})

#add_executable(lines src/lines.cpp)
#target_link_libraries(lines ${catkin_LIBRARIES})

#20210618 Ver odometry
#add_executable(lines_hough src/lines_hough.cpp)
#add_dependencies(lines_hough ${PROJECT_NAME}_gencfg)
#add_dependencies(lines_hough ${PROJECT_NAME}_generate_messages_cpp)
#target_link_libraries(lines_hough ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#20210125_test_file
#add_executable(lines_hough_test src/lines_hough_test.cpp)
#add_dependencies(lines_hough_test ${PROJECT_NAME}_gencfg)
#add_dependencies(lines_hough_test ${PROJECT_NAME}_generate_messages_cpp)
#target_link_libraries(lines_hough_test ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#12gatu calmanfilter note
#add_executable(carib src/carib.cpp)
#target_link_libraries(carib ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#add_executable(srccap src/srccap.cpp)
#target_link_libraries(srccap ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#add_executable(findessential src/findessential.cpp)
#target_link_libraries(findessential ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#add_executable(findessential2 src/findessential2.cpp)
#target_link_libraries(findessential2 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#add_executable(findessential3 src/findessential3.cpp)
#target_link_libraries(findessential3 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#add_executable(findessential3_2 src/findessential3_2.cpp)
#target_link_libraries(findessential3_2 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#add_executable(findessential4 src/findessential4.cpp)
#target_link_libraries(findessential4 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#add_executable(SfM src/smp/SfM.cpp)
#target_link_libraries(SfM ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#add_executable(${PROJECT_NAME}_im # image matching
#   src/imageMatching/constructor.cpp
#   src/imageMatching/methods.cpp
#   src/imageMatching/debug_main.cpp
#   src/imageMatching/debug_methods.cpp
#   src/imageMatching/property.cpp
#src/imageMatching/property.cpp
# )
#add_dependencies(${PROJECT_NAME}_im ${TARGET_NAME}${PROJECT_NAME}_generate_messages_cpp)
#add_dependencies(${PROJECT_NAME}_im ${PROJECT_NAME}_gencfg)
#target_link_libraries(${PROJECT_NAME}_im ${catkin_LIBRARIES} ${OpenCV_LIBRARIES}) # image matching

#20210121
#add_executable(lines_prm src/lines_prm.cpp)
#add_dependencies(lines_prm ${PROJECT_NAME}_gencfg)
#add_dependencies(lines_prm ${PROJECT_NAME}_generate_messages_cpp)
#target_link_libraries(lines_prm ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#20210125
#add_executable(lines_prm2 src/lines_prm2.cpp)
#add_dependencies(lines_prm2 ${PROJECT_NAME}_gencfg)
#add_dependencies(lines_prm2 ${PROJECT_NAME}_generate_messages_cpp)
#target_link_libraries(lines_prm2 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#20210208 copy of lines_hough
#add_executable(lines_hough3 src/lines_hough3.cpp)
#add_dependencies(lines_hough3 ${PROJECT_NAME}_gencfg)
#add_dependencies(lines_hough3 ${PROJECT_NAME}_generate_messages_cpp)
#target_link_libraries(lines_hough3 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#20210210 copy of depth_pcl4_20201127
#add_executable(depth_pcl4_test src/depth_pcl4_test.cpp)
#add_dependencies(depth_pcl4_test ${PROJECT_NAME}_gencfg)
#add_dependencies(depth_pcl4_test ${PROJECT_NAME}_generate_messages_cpp)
#target_link_libraries(depth_pcl4_test ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#20210210 copy of depth_pcl4_20201127 and lines_hough3
#add_executable(depth_pcl4_lines_hough3 src/depth_pcl4_lines_hough3.cpp)
#add_dependencies(depth_pcl4_lines_hough3 ${PROJECT_NAME}_gencfg)
#add_dependencies(depth_pcl4_lines_hough3 ${PROJECT_NAME}_generate_messages_cpp)
#target_link_libraries(depth_pcl4_lines_hough3 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#20210217 copy of lines_hough3 next ver
#add_executable(lines_hough4 src/lines_hough4.cpp)
#add_dependencies(lines_hough4 ${PROJECT_NAME}_gencfg)
#add_dependencies(lines_hough4 ${PROJECT_NAME}_generate_messages_cpp)
#target_link_libraries(lines_hough4 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#20210223 copy of lines_hough3 next ver
#add_executable(lines_hough5 src/lines_hough5.cpp)
#add_dependencies(lines_hough5 ${PROJECT_NAME}_gencfg)
#add_dependencies(lines_hough5 ${PROJECT_NAME}_generate_messages_cpp)
#target_link_libraries(lines_hough5 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#20210224 LSD 
#add_executable(20210224_LSD1 src/20210224_LSD1.cpp src/lsd.h src/lsd.c)
#add_dependencies(20210224_LSD1 ${PROJECT_NAME}_gencfg)
#add_dependencies(20210224_LSD1 ${PROJECT_NAME}_generate_messages_cpp)
#target_link_libraries(20210224_LSD1 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#20210226 FLD(Fast Line Detector) 
#add_executable(20210226_FLD1 src/20210226_FLD1.cpp)
#add_dependencies(20210226_FLD1 ${PROJECT_NAME}_gencfg)
#add_dependencies(20210226_FLD1 ${PROJECT_NAME}_generate_messages_cpp)
#target_link_libraries(20210226_FLD1 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#20210226 FLD(Fast Line Detector) 
#add_executable(20210226_FLD_camera src/20210226_FLD_camera.cpp)
#add_dependencies(20210226_FLD_camera ${PROJECT_NAME}_gencfg)
#add_dependencies(20210226_FLD_camera ${PROJECT_NAME}_generate_messages_cpp)
#target_link_libraries(20210226_FLD_camera ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#20210226 FLD(Fast Line Detector) copy
#add_executable(20210226_FLD_camera2 src/FLD/20210226_FLD_camera2.cpp)
#add_dependencies(20210226_FLD_camera2 ${PROJECT_NAME}_gencfg)
#add_dependencies(20210226_FLD_camera2 ${PROJECT_NAME}_generate_messages_cpp)
#target_link_libraries(20210226_FLD_camera2 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#20210226 FLD(Fast Line Detector) copy
#add_executable(20210226_FLD_camera_Hough src/FLD/20210226_FLD_camera_Hough.cpp)
#add_dependencies(20210226_FLD_camera_Hough ${PROJECT_NAME}_gencfg)
#add_dependencies(20210226_FLD_camera_Hough ${PROJECT_NAME}_generate_messages_cpp)
#target_link_libraries(20210226_FLD_camera_Hough ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#20210227 FLD(Fast Line Detector) copy
#add_executable(20210226_FLD_camera_Hough2 src/FLD/20210226_FLD_camera_Hough2.cpp)
#add_dependencies(20210226_FLD_camera_Hough2 ${PROJECT_NAME}_gencfg)
#add_dependencies(20210226_FLD_camera_Hough2 ${PROJECT_NAME}_generate_messages_cpp)
#target_link_libraries(20210226_FLD_camera_Hough2 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#20210227 FLD(Fast Line Detector) copy
#add_executable(20210226_FLD_camera_Hough3 src/FLD/20210226_FLD_camera_Hough3.cpp)
#add_dependencies(20210226_FLD_camera_Hough3 ${PROJECT_NAME}_gencfg)
#add_dependencies(20210226_FLD_camera_Hough3 ${PROJECT_NAME}_generate_messages_cpp)
#target_link_libraries(20210226_FLD_camera_Hough3 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#20210420 saisyounizyouhou
#add_executable(L2_sp src/L2_sp3/L2_sp.cpp)
#add_dependencies(L2_sp ${PROJECT_NAME}_gencfg)
#add_dependencies(L2_sp ${PROJECT_NAME}_generate_messages_cpp)
#target_link_libraries(L2_sp ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#20210420 saisyounizyouhou_testVer
#add_executable(L2_sp_test src/L2_sp3/L2_sp_test.cpp)
#add_dependencies(L2_sp_test ${PROJECT_NAME}_gencfg)
#add_dependencies(L2_sp_test ${PROJECT_NAME}_generate_messages_cpp)
#target_link_libraries(L2_sp_test ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#20210422 test_Eigen
#add_executable(test_Eigen src/test_Eigen.cpp)
#add_dependencies(test_Eigen ${PROJECT_NAME}_gencfg)
#add_dependencies(test_Eigen ${PROJECT_NAME}_generate_messages_cpp)
#target_link_libraries(test_Eigen ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#20210426 saisyou zizyouhou _test_3honnVer
#add_executable(20210426_L2_sp3_test src/L2_sp3/20210426_L2_sp3_test.cpp)
#add_dependencies(20210426_L2_sp3_test ${PROJECT_NAME}_gencfg)
#add_dependencies(20210426_L2_sp3_test ${PROJECT_NAME}_generate_messages_cpp)
#target_link_libraries(20210426_L2_sp3_test ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#20210429 test_Eigen2
#add_executable(test_Eigen2 src/test_Eigen2.cpp)
#add_dependencies(test_Eigen2 ${PROJECT_NAME}_gencfg)
#add_dependencies(test_Eigen2 ${PROJECT_NAME}_generate_messages_cpp)
#target_link_libraries(test_Eigen2 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#20210503 test_Eigen2 copy
#add_executable(test_Eigen3 src/test_Eigen3.cpp)
#add_dependencies(test_Eigen3 ${PROJECT_NAME}_gencfg)
#add_dependencies(test_Eigen3 ${PROJECT_NAME}_generate_messages_cpp)
#target_link_libraries(test_Eigen3 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#20210503 20210426_L2_sp3_test copy 
#add_executable(20210503_L2_sp3_test src/L2_sp3/20210503_L2_sp3_test.cpp)
#add_dependencies(20210503_L2_sp3_test ${PROJECT_NAME}_gencfg)
#add_dependencies(20210503_L2_sp3_test ${PROJECT_NAME}_generate_messages_cpp)
#target_link_libraries(20210503_L2_sp3_test ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#20210503 test_Eigen_saisyou copy
#add_executable(test_Eigen_saisyou src/test_Eigen_saisyou.cpp)
#add_dependencies(test_Eigen_saisyou ${PROJECT_NAME}_gencfg)
#add_dependencies(test_Eigen_saisyou ${PROJECT_NAME}_generate_messages_cpp)
#target_link_libraries(test_Eigen_saisyou ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#20210507 20210226_FLD_camera_Hough3 copy
#add_executable(20210507_FLD_camera_Hough3 src/FLD/20210507_FLD_camera_Hough3.cpp)
#add_dependencies(20210507_FLD_camera_Hough3 ${PROJECT_NAME}_gencfg)
#add_dependencies(20210507_FLD_camera_Hough3 ${PROJECT_NAME}_generate_messages_cpp)
#target_link_libraries(20210507_FLD_camera_Hough3 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#20210510 20210503_L2_sp3_test copy Ver_test
#add_executable(20210510_L2_sp3_test src/L2_sp3/20210510_L2_sp3_test.cpp)
#add_dependencies(20210510_L2_sp3_test ${PROJECT_NAME}_gencfg)
#add_dependencies(20210510_L2_sp3_test ${PROJECT_NAME}_generate_messages_cpp)
#target_link_libraries(20210510_L2_sp3_test ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#20210510 20210510_L2_sp3_test copy Ver_realtime 
#add_executable(20210510_L2_sp3_realt src/L2_sp3/20210510_L2_sp3_realt.cpp)
#add_dependencies(20210510_L2_sp3_realt ${PROJECT_NAME}_gencfg)
#add_dependencies(20210510_L2_sp3_realt ${PROJECT_NAME}_generate_messages_cpp)
#target_link_libraries(20210510_L2_sp3_realt ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#20210511 20210510_L2_sp3_test copy Ver_test2+ Ver_Randamu
add_executable(20210511_L2_sp3_simulation src/L2_sp3/20210511_L2_sp3_simulation.cpp)
add_dependencies(20210511_L2_sp3_simulation ${PROJECT_NAME}_gencfg)
add_dependencies(20210511_L2_sp3_simulation ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(20210511_L2_sp3_simulation ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#20210511 20210507_FLD_camera_Hough3 copy
#add_executable(20210511_FLD_camera_Hough3 src/FLD/20210511_FLD_camera_Hough3.cpp)
#add_dependencies(20210511_FLD_camera_Hough3 ${PROJECT_NAME}_gencfg)
#add_dependencies(20210511_FLD_camera_Hough3 ${PROJECT_NAME}_generate_messages_cpp)
#target_link_libraries(20210511_FLD_camera_Hough3 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#20210517 20210511_FLD_camera_Hough3 copy ver_depth
#add_executable(20210517_FLD_depth src/FLD/20210517_FLD_depth.cpp)
#add_dependencies(20210517_FLD_depth ${PROJECT_NAME}_gencfg)
#add_dependencies(20210517_FLD_depth ${PROJECT_NAME}_generate_messages_cpp)
#target_link_libraries(20210517_FLD_depth ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#20210518 lines_prm2 copy VerFLD+HoughP
#add_executable(lines_prm3 src/lines_prm3.cpp)
#add_dependencies(lines_prm3 ${PROJECT_NAME}_gencfg)
#add_dependencies(lines_prm3 ${PROJECT_NAME}_generate_messages_cpp)
#target_link_libraries(lines_prm3 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#20210519 RealsenseD435 Depth adjustment program
#add_executable(D435_test src/test/D435_test.cpp)
#add_dependencies(D435_test ${PROJECT_NAME}_gencfg)
#add_dependencies(D435_test ${PROJECT_NAME}_generate_messages_cpp)
#target_link_libraries(D435_test ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#20210519 RealsenseD435 Depth adjustment program
#add_executable(D435_test2 src/test/D435_test2.cpp)
#add_dependencies(D435_test2 ${PROJECT_NAME}_gencfg)
#add_dependencies(D435_test2 ${PROJECT_NAME}_generate_messages_cpp)
#target_link_libraries(D435_test2 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#20210520 20210517_FLD_depth + D435_test
#add_executable(20210520_FLD_depth2 src/FLD/20210520_FLD_depth2.cpp)
#add_dependencies(20210520_FLD_depth2 ${PROJECT_NAME}_gencfg)
#add_dependencies(20210520_FLD_depth2 ${PROJECT_NAME}_generate_messages_cpp)
#target_link_libraries(20210520_FLD_depth2 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#20210521 lines_prm3 copy Ver lines_prm3 + D435_test
#add_executable(lines_prm4 src/prm/lines_prm4.cpp)
#add_dependencies(lines_prm4 ${PROJECT_NAME}_gencfg)
#add_dependencies(lines_prm4 ${PROJECT_NAME}_generate_messages_cpp)
#target_link_libraries(lines_prm4 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#20210521 lines_prm4 copy Ver OpenCV->Eigen
#add_executable(lines_prm5 src/prm/lines_prm5.cpp)
#add_dependencies(lines_prm5 ${PROJECT_NAME}_gencfg)
#add_dependencies(lines_prm5 ${PROJECT_NAME}_generate_messages_cpp)
#target_link_libraries(lines_prm5 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#20210524 lines_prm6 copy Ver lines_prm5 + FLD(tate,yoko,naname)
#add_executable(lines_prm6 src/prm/lines_prm6.cpp)
#add_dependencies(lines_prm6 ${PROJECT_NAME}_gencfg)
#add_dependencies(lines_prm6 ${PROJECT_NAME}_generate_messages_cpp)
#target_link_libraries(lines_prm6 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#20210524 20210520_FLD_depth copy Ver A[][][][]
#add_executable(20210524_FLD_depth2 src/FLD/20210524_FLD_depth2.cpp)
#add_dependencies(20210524_FLD_depth2 ${PROJECT_NAME}_gencfg)
#add_dependencies(20210524_FLD_depth2 ${PROJECT_NAME}_generate_messages_cpp)
#target_link_libraries(20210524_FLD_depth2 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#20210527 OpticalFlow test
#add_executable(OpticalFlow0 src/OptF/OpticalFlow0.cpp)
#add_dependencies(OpticalFlow0 ${PROJECT_NAME}_gencfg)
#add_dependencies(OpticalFlow0 ${PROJECT_NAME}_generate_messages_cpp)
#target_link_libraries(OpticalFlow0 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#20210527 OpticalFlow test
#add_executable(OpticalFlow1 src/OptF/OpticalFlow1.cpp)
#add_dependencies(OpticalFlow1 ${PROJECT_NAME}_gencfg)
#add_dependencies(OpticalFlow1 ${PROJECT_NAME}_generate_messages_cpp)
#target_link_libraries(OpticalFlow1 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#20210527 OpticalFlow test
#add_executable(OpticalFlow2 src/OptF/OpticalFlow2.cpp)
#add_dependencies(OpticalFlow2 ${PROJECT_NAME}_gencfg)
#add_dependencies(OpticalFlow2 ${PROJECT_NAME}_generate_messages_cpp)
#target_link_libraries(OpticalFlow2 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#20210529 20210510_L2_sp3_test copy Ver_realtime
#add_executable(20210529_L2_sp3_camera src/L2_sp3/20210529_L2_sp3_camera.cpp)
#add_dependencies(20210529_L2_sp3_camera ${PROJECT_NAME}_gencfg)
#add_dependencies(20210529_L2_sp3_camera ${PROJECT_NAME}_generate_messages_cpp)
#target_link_libraries(20210529_L2_sp3_camera ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})     

#20210531 Kalman filter test
add_executable(Kal_t1 src/OptF/Kal_t1.cpp)
add_dependencies(Kal_t1 ${PROJECT_NAME}_gencfg)
add_dependencies(Kal_t1 ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(Kal_t1 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

##20210601 camera_video_save 
#add_executable(CVS src/OptF/CVS.cpp)
#add_dependencies(CVS ${PROJECT_NAME}_gencfg)
#add_dependencies(CVS ${PROJECT_NAME}_generate_messages_cpp)
#target_link_libraries(CVS ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#20210601 tuiseki http://cvwww.ee.ous.ac.jp/opencv_practice5/#3.1
#add_executable(tuiseki src/OptF/tuiseki.cpp)
#add_dependencies(tuiseki ${PROJECT_NAME}_gencfg)
#add_dependencies(tuiseki ${PROJECT_NAME}_generate_messages_cpp)
#target_link_libraries(tuiseki ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#20210601 tuiseki Ver realtime http://cvwww.ee.ous.ac.jp/opencv_practice5/#3.1
#add_executable(tuiseki_camera src/OptF/tuiseki_camera.cpp)
#add_dependencies(tuiseki_camera ${PROJECT_NAME}_gencfg)
#add_dependencies(tuiseki_camera ${PROJECT_NAME}_generate_messages_cpp)
#target_link_libraries(tuiseki_camera ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#20210601 matching  http://cvwww.ee.ous.ac.jp/opencv_practice5/#3.1
#add_executable(matching_camera src/OptF/matching_camera.cpp)
#add_dependencies(matching_camera ${PROJECT_NAME}_gencfg)
#add_dependencies(matching_camera ${PROJECT_NAME}_generate_messages_cpp)
#target_link_libraries(matching_camera ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#20210601 ORB_matching Ver realtime
#add_executable(lines_prm_ORB src/OptF/lines_prm_ORB.cpp)
#add_dependencies(lines_prm_ORB ${PROJECT_NAME}_gencfg)
#add_dependencies(lines_prm_ORB ${PROJECT_NAME}_generate_messages_cpp)
#target_link_libraries(lines_prm_ORB ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#20210602 ORB_matching Ver realtime
#add_executable(lines_prm_ORB2 src/prm/lines_prm_ORB2.cpp)
#add_dependencies(lines_prm_ORB2 ${PROJECT_NAME}_gencfg)
#add_dependencies(lines_prm_ORB2 ${PROJECT_NAME}_generate_messages_cpp)
#target_link_libraries(lines_prm_ORB2 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#20210602 template1_matching 
#add_executable(template1 src/OptF/template1.cpp)
#add_dependencies(template1 ${PROJECT_NAME}_gencfg)
#add_dependencies(template1 ${PROJECT_NAME}_generate_messages_cpp)
#target_link_libraries(template1 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#20210602 tuiseki Ver realtime1 http://cvwww.ee.ous.ac.jp/opencv_practice5/#3.1
add_executable(tuiseki_camera1 src/OptF/tuiseki_camera1.cpp)
add_dependencies(tuiseki_camera1 ${PROJECT_NAME}_gencfg)
add_dependencies(tuiseki_camera1 ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(tuiseki_camera1 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#20210602 tuiseki Ver realtime1_1 http://cvwww.ee.ous.ac.jp/opencv_practice5/#3.1
#add_executable(tuiseki_camera1_1 src/OptF/tuiseki_camera1_1.cpp)
#add_dependencies(tuiseki_cadd_dependencies(${PROJECT_NAME}_cc ${PROJECT_NAME}_gencfg)
#ki_camera1_1 ${PROJECT_NAME}_generate_messages_cpp)
#target_link_libraries(tuiseki_camera1_1 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#20210602 tuiseki Ver realtime2 http://cvwww.ee.ous.ac.jp/opencv_practice5/#3.1
#add_executable(tuiseki_camera2 src/OptF/tuiseki_camera2.cpp)
#add_dependencies(tuiseki_camera2 ${PROJECT_NAME}_gencfg)
#add_dependencies(tuiseki_camera2 ${PROJECT_NAME}_generate_messages_cpp)
#target_link_libraries(tuiseki_camera2 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#add_executable(test_prm src/OptF/test_prm.cpp)
#add_dependencies(test_prm ${PROJECT_NAME}_gencfg)
#add_dependencies(test_prm ${PROJECT_NAME}_generate_messages_cpp)
#target_link_libraries(test_prm ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#20210608 lines_prm_ORB2 copy ver Depth
#add_executable(lines_prm_ORB3 src/prm/lines_prm_ORB3.cpp)
#add_dependencies(lines_prm_ORB3 ${PROJECT_NAME}_gencfg)
#add_dependencies(lines_prm_ORB3 ${PROJECT_NAME}_generate_messages_cpp)
#target_link_libraries(lines_prm_ORB3 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#20210610 lines_prm_ORB3 copy ver float
#add_executable(lines_prm_ORB4 src/prm/lines_prm_ORB4.cpp)
#add_dependencies(lines_prm_ORB4 ${PROJECT_NAME}_gencfg)
#add_dependencies(lines_prm_ORB4 ${PROJECT_NAME}_generate_messages_cpp)
#target_link_libraries(lines_prm_ORB4 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#20210617 lines_prm_ORB4 copy ver NotZeroLength
#add_executable(lines_prm_ORB5 src/prm/lines_prm_ORB5.cpp)
#add_dependencies(lines_prm_ORB5 ${PROJECT_NAME}_gencfg)
#add_dependencies(lines_prm_ORB5 ${PROJECT_NAME}_generate_messages_cpp)
#target_link_libraries(lines_prm_ORB5 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#20210618 SfM_sample1
#add_executable(sfm1 src/sfm/sfm1.cpp)
#add_dependencies(sfm1 ${PROJECT_NAME}_gencfg)
#add_dependencies(sfm1 ${PROJECT_NAME}_generate_messages_cpp)
#target_link_libraries(sfm1 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#20210619 template1_matching copy Ver Realtime(ZNCC) 
add_executable(template2 src/OptF/template2.cpp)
add_dependencies(template2 ${PROJECT_NAME}_gencfg)
add_dependencies(template2 ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(template2 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#20210622 template2 copy  
#add_executable(template3 src/OptF/template3.cpp)
#add_dependencies(template3 ${PROJECT_NAME}_gencfg)
#add_dependencies(template3 ${PROJECT_NAME}_generate_messages_cpp)
#target_link_libraries(template3 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#20210623 template3 copy_Ver tokutyoutenn_Limited  
#add_executable(template4_limit src/OptF/template4_limit.cpp)
#add_dependencies(template4_limit ${PROJECT_NAME}_gencfg)
#add_dependencies(template4_limit ${PROJECT_NAME}_generate_messages_cpp)
#target_link_libraries(template4_limit ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#20210627 Kal_t1 copy Ver_Realtime 
#add_executable(Kal_t2 src/OptF/Kal_t2.cpp)
#add_dependencies(Kal_t2 ${PROJECT_NAME}_gencfg)
#add_dependencies(Kal_t2 ${PROJECT_NAME}_generate_messages_cpp)
#target_link_libraries(Kal_t2 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#20210628 Kal_t3 copy Ver_hukusuu 
#add_executable(Kal_t3 src/OptF/Kal_t3.cpp)
#add_dependencies(Kal_t3 ${PROJECT_NAME}_gencfg)
#add_dependencies(Kal_t3 ${PROJECT_NAME}_generate_messages_cpp)
#target_link_libraries(Kal_t3 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#20210628 Kal_t4 copy Ver_hukusuu 
#add_executable(Kal_t4 src/OptF/Kal_t4.cpp)
#add_dependencies(Kal_t4 ${PROJECT_NAME}_gencfg)
#add_dependencies(Kal_t4 ${PROJECT_NAME}_generate_messages_cpp)
#target_link_libraries(Kal_t4 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#20210701 Kal_t2 copy Ver_+TemplateMaching 
#add_executable(Kal_t2_template src/OptF/Kal_t2_template.cpp)
#add_dependencies(Kal_t2_template ${PROJECT_NAME}_gencfg)
#add_dependencies(Kal_t2_template ${PROJECT_NAME}_generate_messages_cpp)
#target_link_libraries(Kal_t2_template ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#20210702
#add_executable(Saisyouzizyou_gaibu_pram src/L2_sp3/Saisyouzizyou_gaibu_pram.cpp)
#add_dependencies(Saisyouzizyou_gaibu_pram ${PROJECT_NAME}_gencfg)
#add_dependencies(Saisyouzizyou_gaibu_pram ${PROJECT_NAME}_generate_messages_cpp)
#target_link_libraries(Saisyouzizyou_gaibu_pram ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#20210706 Kal_t2_template copy Ver_+TemplateMaching (hukusuu)
#add_executable(Kal_t3_template src/OptF/Kal_t3_template.cpp)
#add_dependencies(Kal_t3_template ${PROJECT_NAME}_gencfg)
#add_dependencies(Kal_t3_template ${PROJECT_NAME}_generate_messages_cpp)
#target_link_libraries(Kal_t3_template ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#20210707 Kal_t3_template copy Ver_+TemplateMaching (hukusuu)
#add_executable(Kal_t3_template2 src/OptF/Kal_t3_template2.cpp)
#add_dependencies(Kal_t3_template2 ${PROJECT_NAME}_gencfg)
#add_dependencies(Kal_t3_template2 ${PROJECT_NAME}_generate_messages_cpp)
#target_link_libraries(Kal_t3_template2 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#20210707 Kal_t3_template copy Ver_+TemplateMaching 
#add_executable(Kal_t4_template src/OptF/Kal_t4_template.cpp)
#add_dependencies(Kal_t4_template ${PROJECT_NAME}_gencfg)
#add_dependencies(Kal_t4_template ${PROJECT_NAME}_generate_messages_cpp)
#target_link_libraries(Kal_t4_template ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#20210708 Kal_t4_template copy Ver_Algorithm narabikae
add_executable(Kal_t4_template2 src/OptF/Kal_t4_template2.cpp)
add_dependencies(Kal_t4_template2 ${PROJECT_NAME}_gencfg)
add_dependencies(Kal_t4_template2 ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(Kal_t4_template2 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#20210707 Kal_t3_template2 copy Ver_kahennsuu_taiou
#add_executable(Kal_t3_template3 src/OptF/Kal_t3_template3.cpp)
#add_dependencies(Kal_t3_template3 ${PROJECT_NAME}_gencfg)
#add_dependencies(Kal_t3_template3 ${PROJECT_NAME}_generate_messages_cpp)
#target_link_libraries(Kal_t3_template3 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#20210709 Kal_t4_template2 copy Ver_Error_repair
add_executable(Kal_t5_template src/OptF/Kal_t5_template.cpp)
add_dependencies(Kal_t5_template ${PROJECT_NAME}_gencfg)
add_dependencies(Kal_t5_template ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(Kal_t5_template ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#20210712 20210524_FLD_depth2 copy Ver + Kal_t4_template2
#add_executable(20210712_FLD src/FLD/20210712_FLD.cpp)
#add_dependencies(20210712_FLD ${PROJECT_NAME}_gencfg)
#add_dependencies(20210712_FLD ${PROJECT_NAME}_generate_messages_cpp)
#target_link_libraries(20210712_FLD ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#20210714 Kal_t5_template copy Ver_bunkatu(Roi) 
add_executable(Kal_t5_template2 src/OptF/Kal_t5_template2.cpp)
add_dependencies(Kal_t5_template2 ${PROJECT_NAME}_gencfg)
add_dependencies(Kal_t5_template2 ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(Kal_t5_template2 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#20210715 EKF 
#add_executable(EKF_test src/EKF/EKF_test.cpp)
#add_dependencies(EKF_test ${PROJECT_NAME}_gencfg)
#add_dependencies(EKF_test ${PROJECT_NAME}_generate_messages_cpp)
#target_link_libraries(EKF_test ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#20210715 EKF 
#add_executable(EKF_test0 src/EKF/EKF_test0.cpp)
#add_dependencies(EKF_test0 ${PROJECT_NAME}_gencfg)
#add_dependencies(EKF_test0 ${PROJECT_NAME}_generate_messages_cpp)
#target_link_libraries(EKF_test0 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#20210715 EKF 
add_executable(EKF_test1 src/EKF/EKF_test1.cpp)
add_dependencies(EKF_test1 ${PROJECT_NAME}_gencfg)
add_dependencies(EKF_test1 ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(EKF_test1 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#20210715 EKF 
add_executable(KF_test src/EKF/KF_test.cpp)
add_dependencies(KF_test ${PROJECT_NAME}_gencfg)
add_dependencies(KF_test ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(KF_test ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})


#20210716 EKF 
#add_executable(EKF_test2 src/EKF/EKF_test2.cpp)
#add_dependencies(EKF_test2 ${PROJECT_NAME}_gencfg)
#add_dependencies(EKF_test2 ${PROJECT_NAME}_generate_messages_cpp)
#target_link_libraries(EKF_test2 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#20210716 EKF 
#add_executable(EKF_testi src/EKF/EKF_testi.cpp)
#add_dependencies(EKF_testi ${PROJECT_NAME}_gencfg)
#add_dependencies(EKF_testi ${PROJECT_NAME}_generate_messages_cpp)
#target_link_libraries(EKF_testi ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#20210717 lines_prm_ORB5 copy Ver opticalFlow + Vpoint 
#add_executable(Vpoint_1 src/L2_sp3/Vpoint_1.cpp)
#add_dependencies(Vpoint_1 ${PROJECT_NAME}_gencfg)
#add_dependencies(Vpoint_1 ${PROJECT_NAME}_generate_messages_cpp)
#target_link_libraries(Vpoint_1 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#20210719 Particle_Filter_SampleTest 
#add_executable(particle_test src/L2_sp3/particle_test.cpp)
#add_dependencies(particle_test ${PROJECT_NAME}_gencfg)
#add_dependencies(particle_test ${PROJECT_NAME}_generate_messages_cpp)
#target_link_libraries(particle_test ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#20210719 histogram test
#add_executable(histogram src/FLD/histogram.cpp)
#add_dependencies(histogram ${PROJECT_NAME}_gencfg)
#add_dependencies(histogram ${PROJECT_NAME}_generate_messages_cpp)
#target_link_libraries(histogram ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#add_executable(${PROJECT_NAME}_cc # classification
#  src/classification/constructor.cpp
#  src/classification/methods.cpp
#  src/classification/debug_main.cpp
#  src/classification/debug_methods.cpp
#  src/classification/property.cpp
#)
#add_dependencies(${PROJECT_NAME}_cc ${PROJECT_NAME}_gencfg) # classification
#target_link_libraries(${PROJECT_NAME}_cc ${catkin_LIBRARIES}) # classification

#20210726 Vpoint_1 copy Ver clustering(depth_pcl4_20201127 sannkou) 
add_executable(Vpoint_1_clustering src/L2_sp3/Vpoint_1_clustering.cpp)
add_dependencies(Vpoint_1_clustering  ${PROJECT_NAME}_gencfg)
add_dependencies(Vpoint_1_clustering  ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(Vpoint_1_clustering  ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#20210726 20210712_FLD copy Ver clustering
#add_executable(FLD_clustering src/FLD/FLD_clustering.cpp)
#add_dependencies(FLD_clustering ${PROJECT_NAME}_gencfg)
#add_dependencies(FLD_clustering ${PROJECT_NAME}_generate_messages_cpp)
#target_link_libraries(FLD_clustering ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#20210726 13-01
#add_executable(13-01 src/FLD/13-01.cpp)
#add_dependencies(13-01 ${PROJECT_NAME}_gencfg)
#add_dependencies(13-01 ${PROJECT_NAME}_generate_messages_cpp)
#target_link_libraries(13-01 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#20210726 13-01
#add_executable(k-means src/FLD/k-means.cpp)
#add_dependencies(k-means ${PROJECT_NAME}_gencfg)
#add_dependencies(k-means ${PROJECT_NAME}_generate_messages_cpp)
#target_link_libraries(k-means ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#20210729 20210712_FLD copy Ver clustering_2
#add_executable(FLD_clustering_2 src/FLD/FLD_clustering_2.cpp)
#add_dependencies(FLD_clustering_2 ${PROJECT_NAME}_gencfg)
#add_dependencies(FLD_clustering_2 ${PROJECT_NAME}_generate_messages_cpp)
#target_link_libraries(FLD_clustering_2 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#20210712 20210712_FLD copy Ver +Vpoint(suitei)
#add_executable(20210712_FLD_Vpoint1 src/L2_sp3/20210712_FLD_Vpoint1.cpp)
#add_dependencies(20210712_FLD_Vpoint1 ${PROJECT_NAME}_gencfg)
#add_dependencies(20210712_FLD_Vpoint1 ${PROJECT_NAME}_generate_messages_cpp)
#target_link_libraries(20210712_FLD_Vpoint1 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#20210730 depth_pcl4_20210521 copy depth_pcl4_20210521 + lines_prm_ORB5
add_executable(lines_prm_ORB6 src/prm/lines_prm_ORB6.cpp)
add_dependencies(lines_prm_ORB6 ${PROJECT_NAME}_gencfg)
add_dependencies(lines_prm_ORB6 ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(lines_prm_ORB6 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#20210730 lines_prm_ORB6
#add_executable(lines_prm_ORB6_2 src/prm/lines_prm_ORB6_2.cpp)
#add_dependencies(lines_prm_ORB6_2 ${PROJECT_NAME}_gencfg)
#add_dependencies(lines_prm_ORB6_2 ${PROJECT_NAME}_generate_messages_cpp)
#target_link_libraries(lines_prm_ORB6_2 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#20210730 sample 
#add_executable(05-01 src/sample/05-01.cpp)
#add_dependencies(05-01 ${PROJECT_NAME}_gencfg)
#add_dependencies(05-01 ${PROJECT_NAME}_generate_messages_cpp)
#target_link_libraries(05-01 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#20210730 sample 
#add_executable(05-02_2 src/sample/05-02_2.cpp)
#add_dependencies(05-02_2 ${PROJECT_NAME}_gencfg)
#add_dependencies(05-02_2 ${PROJECT_NAME}_generate_messages_cpp)
#target_link_libraries(05-02_2 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#20210730 sample 
#add_executable(05-02 src/sample/05-02.cpp)
#add_dependencies(05-02 ${PROJECT_NAME}_gencfg)
#add_dependencies(05-02 ${PROJECT_NAME}_generate_messages_cpp)
#target_link_libraries(05-02 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#20210731 sample 
#add_executable(vis_tf_broadcast src/tf/vis_tf_broadcast.cpp)
#add_dependencies(vis_tf_broadcast ${PROJECT_NAME}_gencfg)
#add_dependencies(vis_tf_broadcast ${PROJECT_NAME}_generate_messages_cpp)
#target_link_libraries(vis_tf_broadcast ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#20210731 sample 
#add_executable(vis_tf_listen src/tf/vis_tf_listen.cpp)
#add_dependencies(vis_tf_listen ${PROJECT_NAME}_gencfg)
#add_dependencies(vis_tf_listen ${PROJECT_NAME}_generate_messages_cpp)
#target_link_libraries(vis_tf_listen ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#20210731 sample 
#add_executable(image_publisher src/test/image_publisher.cpp)
#add_dependencies(image_publisher ${PROJECT_NAME}_gencfg)
#add_dependencies(image_publisher ${PROJECT_NAME}_generate_messages_cpp)
#target_link_libraries(image_publisher ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#20210731 sample 
#add_executable(image_subscriber src/test/image_subscriber.cpp)
#add_dependencies(image_subscriber ${PROJECT_NAME}_gencfg)
#add_dependencies(image_subscriber ${PROJECT_NAME}_generate_messages_cpp)
#target_link_libraries(image_subscriber ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#20210801 FLD_clustering_2 copy Ver Average_clustering_yokoline
#add_executable(FLD_clustering_3 src/FLD/FLD_clustering_3.cpp)
#add_dependencies(FLD_clustering_3 ${PROJECT_NAME}_gencfg)
#add_dependencies(FLD_clustering_3 ${PROJECT_NAME}_generate_messages_cpp)
#target_link_libraries(FLD_clustering_3 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#20210803 Marker
#add_executable(marker1 src/marker/marker1.cpp)
#add_dependencies(marker1 ${PROJECT_NAME}_gencfg)
#add_dependencies(marker1 ${PROJECT_NAME}_generate_messages_cpp)
#target_link_libraries(marker1 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#20210803 Marker
#add_executable(marker1_make src/marker/marker1_make.cpp)
#add_dependencies(marker1_make ${PROJECT_NAME}_gencfg)
#add_dependencies(marker1_make ${PROJECT_NAME}_generate_messages_cpp)
#target_link_libraries(marker1_make ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#20210803 sample 
add_executable(robot_tf_broadcast src/tf/robot_tf_broadcast.cpp)
add_dependencies(robot_tf_broadcast ${PROJECT_NAME}_gencfg)
add_dependencies(robot_tf_broadcast ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(robot_tf_broadcast ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#20210803 sample 
add_executable(robot_tf_listen src/tf/robot_tf_listen.cpp)
add_dependencies(robot_tf_listen ${PROJECT_NAME}_gencfg)
add_dependencies(robot_tf_listen ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(robot_tf_listen ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#20210803 sample 
#add_executable(robot_tf_listen_broad src/tf/robot_tf_listen_broad.cpp)
#add_dependencies(robot_tf_listen_broad ${PROJECT_NAME}_gencfg)
#add_dependencies(robot_tf_listen_broad ${PROJECT_NAME}_generate_messages_cpp)
#target_link_libraries(robot_tf_listen_broad ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#20210805 Marker https://github.com/naruya/aruco/blob/master/aruco.ipynb
#add_executable(marker2 src/marker/marker2.cpp)
#add_dependencies(marker2 ${PROJECT_NAME}_gencfg)
#add_dependencies(marker2 ${PROJECT_NAME}_generate_messages_cpp)
#target_link_libraries(marker2 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#20210805 Marker https://github.com/naruya/aruco/blob/master/aruco.ipynb
#add_executable(marker2_listen src/marker/marker2_listen.cpp)
#add_dependencies(marker2_listen ${PROJECT_NAME}_gencfg)
#add_dependencies(marker2_listen ${PROJECT_NAME}_generate_messages_cpp)
#target_link_libraries(marker2_listen ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#20210808 Marker https://github.com/naruya/aruco/blob/master/aruco.ipynb
#add_executable(marker3 src/marker/marker3.cpp)
#add_dependencies(marker3 ${PROJECT_NAME}_gencfg)
#add_dependencies(marker3 ${PROJECT_NAME}_generate_messages_cpp)
#target_link_libraries(marker3 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#20210809 Marker saisyouizyouhou gaibu_pram
#add_executable(marker4 src/marker/marker4.cpp)
#add_dependencies(marker4 ${PROJECT_NAME}_gencfg)
#add_dependencies(marker4 ${PROJECT_NAME}_generate_messages_cpp)
#target_link_libraries(marker4 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#20210810 Marker saisyouizyouhou gaibu_pram (marker4 copy)
#add_executable(marker04 src/marker/marker04.cpp)
#add_dependencies(marker04 ${PROJECT_NAME}_gencfg)
#add_dependencies(marker04 ${PROJECT_NAME}_generate_messages_cpp)
#target_link_libraries(marker04 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#20210814 Marker Depth_Error_repair (marker04 copy)
#add_executable(marker04_1 src/marker/marker04_1.cpp)
#add_dependencies(marker04_1 ${PROJECT_NAME}_gencfg)
#add_dependencies(marker04_1 ${PROJECT_NAME}_generate_messages_cpp)
#target_link_libraries(marker04_1 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#20210814 Marker Depth_Error_repair (marker04 copy)
#add_executable(marker04_2 src/marker/marker04_2.cpp)
#add_dependencies(marker04_2 ${PROJECT_NAME}_gencfg)
#add_dependencies(marker04_2 ${PROJECT_NAME}_generate_messages_cpp)
#target_link_libraries(marker04_2 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#20210816 Marker Depth_Error_repair (marker04_1 copy)
#add_executable(marker04_3 src/marker/marker04_3.cpp)
#add_dependencies(marker04_3 ${PROJECT_NAME}_gencfg)
#add_dependencies(marker04_3 ${PROJECT_NAME}_generate_messages_cpp)
#target_link_libraries(marker04_3 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#20210810 Marker4 copy Ver hukusuu
#add_executable(marker5 src/marker/marker5.cpp)
#add_dependencies(marker5 ${PROJECT_NAME}_gencfg)
#add_dependencies(marker5 ${PROJECT_NAME}_generate_messages_cpp)
#target_link_libraries(marker5 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#20210816 hidaka
#add_executable(marker_hidaka src/marker/hidaka/marker_hidaka.cpp)
#add_dependencies(marker_hidaka ${PROJECT_NAME}_gencfg)
#add_dependencies(marker_hidaka ${PROJECT_NAME}_generate_messages_cpp)
#target_link_libraries(marker_hidaka ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#20210818 hidaka
#add_executable(marker_hidaka_1 src/marker/hidaka/marker_hidaka_1.cpp)
#add_dependencies(marker_hidaka_1 ${PROJECT_NAME}_gencfg)
#add_dependencies(marker_hidaka_1 ${PROJECT_NAME}_generate_messages_cpp)
#target_link_libraries(marker_hidaka_1 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#20210818 hidaka marker+Feature point
add_executable(marker_hidaka_point src/marker/hidaka/marker_hidaka_point.cpp)
add_dependencies(marker_hidaka_point ${PROJECT_NAME}_gencfg)
add_dependencies(marker_hidaka_point ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(marker_hidaka_point ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#20210818 hidaka marker+Feature point
add_executable(marker_hidaka_point0 src/marker/hidaka/marker_hidaka_point0.cpp)
add_dependencies(marker_hidaka_point0 ${PROJECT_NAME}_gencfg)
add_dependencies(marker_hidaka_point0 ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(marker_hidaka_point0 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#20210826 EKF_camera
#add_executable(adv_time src/test/adv_time.cpp)
#add_dependencies(adv_time ${PROJECT_NAME}_gencfg)
#add_dependencies(adv_time ${PROJECT_NAME}_generate_messages_cpp)
#target_link_libraries(adv_time ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#20210826 hidaka marker+Feature point2
#add_executable(marker_hidaka_point2 src/marker/hidaka/marker_hidaka_point2.cpp)
#add_dependencies(marker_hidaka_point2 ${PROJECT_NAME}_gencfg)
#add_dependencies(marker_hidaka_point2 ${PROJECT_NAME}_generate_messages_cpp)
#target_link_libraries(marker_hidaka_point2 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#20210828 hidaka marker bunsan test
add_executable(marker_hidaka_bunsan src/marker/hidaka/marker_hidaka_bunsan.cpp)
add_dependencies(marker_hidaka_bunsan ${PROJECT_NAME}_gencfg)
add_dependencies(marker_hidaka_bunsan ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(marker_hidaka_bunsan ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#20210831 Vpoint
#add_executable(Vpoint_test2 src/Vpoint/Vpoint_test2.cpp)
#add_dependencies(Vpoint_test2 ${PROJECT_NAME}_gencfg)
#add_dependencies(Vpoint_test2 ${PROJECT_NAME}_generate_messages_cpp)
#target_link_libraries(Vpoint_test2 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#add_executable(Vpoint_test src/Vpoint/Vpoint_test.cpp)
#add_dependencies(Vpoint_test ${PROJECT_NAME}_gencfg)
#add_dependencies(Vpoint_test ${PROJECT_NAME}_generate_messages_cpp)
#target_link_libraries(Vpoint_test ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})


#add_executable(main ./main.cpp)
#target_link_libraries(main ${catkin_LIBRARIES} ${PROJECT_NAME}${OpenCV_LIBRARIES})

#20210901 RANSC Line
#add_executable(ransac_test src/test/ransac_test.cpp)
#add_dependencies(ransac_test ${PROJECT_NAME}_gencfg)
#add_dependencies(ransac_test ${PROJECT_NAME}_generate_messages_cpp)
#target_link_libraries(ransac_test ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#20210901 RANSC Line+FLD
#add_executable(FLD_ransac src/FLD/FLD_ransac.cpp)
#add_dependencies(FLD_ransac ${PROJECT_NAME}_gencfg)
#add_dependencies(FLD_ransac ${PROJECT_NAME}_generate_messages_cpp)
#target_link_libraries(FLD_ransac ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#20210901 FLD only copy Ver Average_clustering_yokoline
#add_executable(FLD_clustering_04 src/FLD/FLD_clustering_04.cpp)
#add_dependencies(FLD_clustering_04 ${PROJECT_NAME}_gencfg)
#add_dependencies(FLD_clustering_04 ${PROJECT_NAME}_generate_messages_cpp)
#target_link_libraries(FLD_clustering_04 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#20210901 FLD only copy Ver Average_clustering_yokoline
#add_executable(FLD_clustering_4 src/FLD/FLD_clustering_4.cpp)
#add_dependencies(FLD_clustering_4 ${PROJECT_NAME}_gencfg)
#add_dependencies(FLD_clustering_4 ${PROJECT_NAME}_generate_messages_cpp)
#target_link_libraries(FLD_clustering_4 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#20210902 Vpoint_test2+Vpoint(suitei)
#add_executable(20210902_FLD_Vpoint2 src/L2_sp3/20210902_FLD_Vpoint2.cpp)
#add_dependencies(20210902_FLD_Vpoint2 ${PROJECT_NAME}_gencfg)
#add_dependencies(20210902_FLD_Vpoint2 ${PROJECT_NAME}_generate_messages_cpp)
#target_link_libraries(20210902_FLD_Vpoint2 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#20210902 Vpoint_test2+Vpoint(suitei) Ver RANSAC
#add_executable(20210902_FLD_Vpoint2_ransac src/L2_sp3/20210902_FLD_Vpoint2_ransac.cpp)
#add_dependencies(20210902_FLD_Vpoint2_ransac ${PROJECT_NAME}_gencfg)
#add_dependencies(20210902_FLD_Vpoint2_ransac ${PROJECT_NAME}_generate_messages_cpp)
#target_link_libraries(20210902_FLD_Vpoint2_ransac ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#20210901 FLD_clustering_4 + 20210902_FLD_Vpoint2
add_executable(FLD_clustering_5 src/FLD/FLD_clustering_5.cpp)
add_dependencies(FLD_clustering_5 ${PROJECT_NAME}_gencfg)
add_dependencies(FLD_clustering_5 ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(FLD_clustering_5 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#20210902 特徴点検出テスト
add_executable(point_test src/test/point_test.cpp)
add_dependencies(point_test ${PROJECT_NAME}_gencfg)
add_dependencies(point_test ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(point_test ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#20210910 FLD_clustering_5 Ver2　画像コーナー検出
add_executable(FLD_clustering_5_2 src/FLD/FLD_clustering_5_2.cpp)
add_dependencies(FLD_clustering_5_2 ${PROJECT_NAME}_gencfg)
add_dependencies(FLD_clustering_5_2 ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(FLD_clustering_5_2 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#20210910 Depth画像のカラー化
add_executable(depth_test src/test/depth_test.cpp)
add_dependencies(depth_test ${PROJECT_NAME}_gencfg)
add_dependencies(depth_test ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(depth_test ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#20211119 マーカーのDepth値確認
add_executable(depth_marker_test src/test/depth_marker_test.cpp)
add_dependencies(depth_marker_test ${PROJECT_NAME}_gencfg)
add_dependencies(depth_marker_test ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(depth_marker_test ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#20210825 EKF_camera
#add_executable(EKF_test_marker src/EKF/EKF_test_marker.cpp)
#add_dependencies(EKF_test_marker ${PROJECT_NAME}_gencfg)
#add_dependencies(EKF_test_marker ${PROJECT_NAME}_generate_messages_cpp)
#target_link_libraries(EKF_test_marker ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#20210914 EKF_camera行列を点の数だけ拡大した（間違ってるかも？）
#add_executable(EKF_test_marker2 src/EKF/EKF_test_marker2.cpp)
#add_dependencies(EKF_test_marker2 ${PROJECT_NAME}_gencfg)
#add_dependencies(EKF_test_marker2 ${PROJECT_NAME}_generate_messages_cpp)
#target_link_libraries(EKF_test_marker2 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#20210914 EKF_camera行列ではなく式の個数を点群の数だけ作る
#add_executable(EKF_test_marker2_2 src/EKF/EKF_test_marker2_2.cpp)
#add_dependencies(EKF_test_marker2_2 ${PROJECT_NAME}_gencfg)
#add_dependencies(EKF_test_marker2_2 ${PROJECT_NAME}_generate_messages_cpp)
#target_link_libraries(EKF_test_marker2_2 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})


#add_executable(opencv_cap2 src/test/opencv_cap2.cpp)
#target_link_libraries(opencv_cap2 ${catkin_LIBRARIES} ${RealSense2_LIBRARIES} ${OpenCV_LIBRARIES})

#20210916 床面除去
#add_executable(depth_test_ransac src/test/depth_test_ransac.cpp)
#add_dependencies(depth_test_ransac ${PROJECT_NAME}_gencfg)
#add_dependencies(depth_test_ransac ${PROJECT_NAME}_generate_messages_cpp)
#target_link_libraries(depth_test_ransac ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#20210920 EKF_test_marker2_2のカルマンマーカーID対応配列化+マーカーの中心座標の１点のみ使用
#add_executable(EKF_test_marker2_3 src/EKF/EKF_test_marker2_3.cpp)
#add_dependencies(EKF_test_marker2_3 ${PROJECT_NAME}_gencfg)
#add_dependencies(EKF_test_marker2_3 ${PROJECT_NAME}_generate_messages_cpp)
#target_link_libraries(EKF_test_marker2_3 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#20210921 ロボットビジョンの手法
#add_executable(EKF_test_marker3 src/EKF/EKF_test_marker3.cpp)
#add_dependencies(EKF_test_marker3 ${PROJECT_NAME}_gencfg)
#add_dependencies(EKF_test_marker3 ${PROJECT_NAME}_generate_messages_cpp)
#target_link_libraries(EKF_test_marker3 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#20210921 EKF_test_marker3の日高先生手法を組み込んだバージョン
#add_executable(EKF_test_marker2_4 src/EKF/EKF_test_marker2_4.cpp)
#add_dependencies(EKF_test_marker2_4 ${PROJECT_NAME}_gencfg)
#add_dependencies(EKF_test_marker2_4 ${PROJECT_NAME}_generate_messages_cpp)
#target_link_libraries(EKF_test_marker2_4 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#20210922 EKF_test_marker2_4+特徴点外部パラメータ推定
#add_executable(EKF_test_marker2_5 src/EKF/EKF_test_marker2_5.cpp)
#add_dependencies(EKF_test_marker2_5 ${PROJECT_NAME}_gencfg)
#add_dependencies(EKF_test_marker2_5 ${PROJECT_NAME}_generate_messages_cpp)
#target_link_libraries(EKF_test_marker2_5 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#20210922 EKF_test_marker2_5　EKFを使用せずに観測データのみで運動パラメータ推定を行う(外部パラメータを使用した自己位置推定)
#add_executable(NO_EKF_test_marker2_5 src/EKF/NO_EKF_test_marker2_5.cpp)
#add_dependencies(NO_EKF_test_marker2_5 ${PROJECT_NAME}_gencfg)
#add_dependencies(NO_EKF_test_marker2_5 ${PROJECT_NAME}_generate_messages_cpp)
#target_link_libraries(NO_EKF_test_marker2_5 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#20210924 マーカー全範囲のDepth分散取得プログラム
add_executable(marker_depth_bunsan src/marker/marker_depth_bunsan.cpp)
add_dependencies(marker_depth_bunsan ${PROJECT_NAME}_gencfg)
add_dependencies(marker_depth_bunsan ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(marker_depth_bunsan ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#20210924 マーカー全範囲のDepth分散取得プログラム
#add_executable(marker_depth_bunsan2 src/marker/marker_depth_bunsan2.cpp)
#add_dependencies(marker_depth_bunsan2 ${PROJECT_NAME}_gencfg)
#add_dependencies(marker_depth_bunsan2 ${PROJECT_NAME}_generate_messages_cpp)
#target_link_libraries(marker_depth_bunsan2 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#20210922 NO_EKF_test_marker2_5のマーカー世界座標を利用した自己位置推定(まだできていない)
#add_executable(NO_EKF_test_marker2_6 src/EKF/NO_EKF_test_marker2_6.cpp)
#add_dependencies(NO_EKF_test_marker2_6 ${PROJECT_NAME}_gencfg)
#add_dependencies(NO_EKF_test_marker2_6 ${PROJECT_NAME}_generate_messages_cpp)
#target_link_libraries(NO_EKF_test_marker2_6 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#20210929 特徴点追跡を２回行う外パラ自己位置推定＋マーカー追跡（EKFなし)
#add_executable(point_test_marker src/Vpoint/point_test_marker.cpp)
#add_dependencies(point_test_marker ${PROJECT_NAME}_gencfg)
#add_dependencies(point_test_marker ${PROJECT_NAME}_generate_messages_cpp)
#target_link_libraries(point_test_marker ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#20210929 特徴点追跡を２回行う外パラ自己位置推定
#add_executable(point_test_NOmarker src/Vpoint/point_test_NOmarker.cpp)
#add_dependencies(point_test_NOmarker ${PROJECT_NAME}_gencfg)
#add_dependencies(point_test_NOmarker ${PROJECT_NAME}_generate_messages_cpp)
#target_link_libraries(point_test_NOmarker ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#20210929 tfテスト roslaunch struct_slam start_demo.launch 
#add_executable(turtle_tf_broadcaster src/tf/turtle_tf_broadcaster.cpp)
#target_link_libraries(turtle_tf_broadcaster ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
##20210929 tfテスト:http://wiki.ros.org/ja/tf/Tutorials/Writing%20a%20tf%20listener%20%28C%2B%2B%29
#add_executable(turtle_tf_listener src/tf/turtle_tf_listener.cpp)
#target_link_libraries(turtle_tf_listener ${catkin_LIBRARIES})
##20210929 tfテスト:http://wiki.ros.org/ja/tf/Tutorials/Adding%20a%20frame%20%28C%2B%2B%29
#add_executable(frame_tf_broadcaster src/tf/frame_tf_broadcaster.cpp)
#target_link_libraries(frame_tf_broadcaster ${catkin_LIBRARIES})

#20211005 特徴点追跡中に特徴点検出を行い自己位置推定
#add_executable(point_test_NOmarker2 src/Vpoint/point_test_NOmarker2.cpp)
#add_dependencies(point_test_NOmarker2 ${PROJECT_NAME}_gencfg)
#add_dependencies(point_test_NOmarker2 ${PROJECT_NAME}_generate_messages_cpp)
#target_link_libraries(point_test_NOmarker2 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#20211005 特徴点追跡中に特徴点検出を行い自己位置推定(オプティカルフローのpoint位置違い)
#add_executable(point_test_NOmarker3 src/Vpoint/point_test_NOmarker3.cpp)
#add_dependencies(point_test_NOmarker3 ${PROJECT_NAME}_gencfg)
#add_dependencies(point_test_NOmarker3 ${PROJECT_NAME}_generate_messages_cpp)
#target_link_libraries(point_test_NOmarker3 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#20211005 ORB特徴点検出+追跡テスト(記述子テスト)
add_executable(ORB_point_test src/test/ORB_point_test.cpp)
add_dependencies(ORB_point_test ${PROJECT_NAME}_gencfg)
add_dependencies(ORB_point_test ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(ORB_point_test ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#20211006 テンプレートマッチング(差分二乗和)(これでマッチングの認識が可能)
add_executable(template_test src/test/template_test.cpp)
add_dependencies(template_test ${PROJECT_NAME}_gencfg)
add_dependencies(template_test ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(template_test ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#20211006 テンプレートマッチング(複数)+特徴点
add_executable(template_point_test src/test/template_point_test.cpp)
add_dependencies(template_point_test ${PROJECT_NAME}_gencfg)
add_dependencies(template_point_test ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(template_point_test ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#20211006 テンプレートマッチング(複数)+特徴点の高速化
#add_executable(template_point_test3 src/test/template_point_test3.cpp)
#add_dependencies(template_point_test3 ${PROJECT_NAME}_gencfg)
#add_dependencies(template_point_test3 ${PROJECT_NAME}_generate_messages_cpp)
#target_link_libraries(template_point_test3 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#20211006 FLD_clustering_5の構造線を代表線にまとめる
add_executable(FLD_clustering_6 src/FLD/FLD_clustering_6.cpp)
add_dependencies(FLD_clustering_6 ${PROJECT_NAME}_gencfg)
add_dependencies(FLD_clustering_6 ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(FLD_clustering_6 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#20211008 FLD_clustering_6の線中点をテンプレートとしたテンプレートマッチングを行う
#add_executable(FLD_clustering_6_1 src/FLD/FLD_clustering_6_1.cpp)
#add_dependencies(FLD_clustering_6_1 ${PROJECT_NAME}_gencfg)
#add_dependencies(FLD_clustering_6_1 ${PROJECT_NAME}_generate_messages_cpp)
#target_link_libraries(FLD_clustering_6_1 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#20211005 ORB特徴点検出で運動パラメータ推定
#add_executable(ORB_point_test1 src/test/ORB_point_test1.cpp)
#add_dependencies(ORB_point_test1 ${PROJECT_NAME}_gencfg)
#add_dependencies(ORB_point_test1 ${PROJECT_NAME}_generate_messages_cpp)
#target_link_libraries(ORB_point_test1 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#20211011 テンプレートマッチング高速化(うまくできない。多分残差逐次検定法と画像ピラミッドを使ってる)
#add_executable(Hi_speed_template_matching src/test/Hi_speed_template_matching.cpp)
#add_dependencies(Hi_speed_template_matching ${PROJECT_NAME}_gencfg)
#add_dependencies(Hi_speed_template_matching ${PROJECT_NAME}_generate_messages_cpp)
#target_link_libraries(Hi_speed_template_matching ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#20211008 FLD_clustering_6_1の続き(線追跡法)
#add_executable(FLD_clustering_6_2 src/FLD/FLD_clustering_6_2.cpp)
#add_dependencies(FLD_clustering_6_2 ${PROJECT_NAME}_gencfg)
#add_dependencies(FLD_clustering_6_2 ${PROJECT_NAME}_generate_messages_cpp)
#target_link_libraries(FLD_clustering_6_2 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#20211010 FLD_clustering_6_1のテンプレマッチ高速化(画像クロップ法)
#add_executable(FLD_clustering_6_1_2 src/FLD/FLD_clustering_6_1_2.cpp)
#add_dependencies(FLD_clustering_6_1_2 ${PROJECT_NAME}_gencfg)
#add_dependencies(FLD_clustering_6_1_2 ${PROJECT_NAME}_generate_messages_cpp)
#target_link_libraries(FLD_clustering_6_1_2 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#20211012 FLD_clustering_6_1のテンプレマッチ高速化(画像クロップ法)
#add_executable(Kal_point src/test/Kal_point.cpp)
#add_dependencies(Kal_point ${PROJECT_NAME}_gencfg)
#add_dependencies(Kal_point ${PROJECT_NAME}_generate_messages_cpp)
#target_link_libraries(Kal_point ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#20211008 FLD_clustering_6_2の続き(線追跡法)
#add_executable(FLD_clustering_6_2_1 src/FLD/FLD_clustering_6_2_1.cpp)
#add_dependencies(FLD_clustering_6_2_1 ${PROJECT_NAME}_gencfg)
#add_dependencies(FLD_clustering_6_2_1 ${PROJECT_NAME}_generate_messages_cpp)
#target_link_libraries(FLD_clustering_6_2_1 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#20211012 特徴点追加＋マーカー追跡（EKFなし)
#add_executable(point_test_marker2 src/Vpoint/point_test_marker2.cpp)
#add_dependencies(point_test_marker2 ${PROJECT_NAME}_gencfg)
#add_dependencies(point_test_marker2 ${PROJECT_NAME}_generate_messages_cpp)
#target_link_libraries(point_test_marker2 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#20211012 特徴点追加＋マーカー追跡（EKF有り)
#add_executable(EKF_point_test_marker2 src/EKF/EKF_point_test_marker2.cpp)
#add_dependencies(EKF_point_test_marker2 ${PROJECT_NAME}_gencfg)
#add_dependencies(EKF_point_test_marker2 ${PROJECT_NAME}_generate_messages_cpp)
#target_link_libraries(EKF_point_test_marker2 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#20211012 特徴点追加（特徴点のEKF)
#add_executable(EKF_point_test1 src/EKF/EKF_point_test1.cpp)
#add_dependencies(EKF_point_test1 ${PROJECT_NAME}_gencfg)
#add_dependencies(EKF_point_test1 ${PROJECT_NAME}_generate_messages_cpp)
#target_link_libraries(EKF_point_test1 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#20211013 マーカーの真値を利用した外部パラメータ推定(EKFなし)
#add_executable(marker_pose1 src/Vpoint/marker_pose1.cpp)
#add_dependencies(marker_pose1 ${PROJECT_NAME}_gencfg)
#add_dependencies(marker_pose1 ${PROJECT_NAME}_generate_messages_cpp)
#target_link_libraries(marker_pose1 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#20211013 点群クラスタリングテスト
#add_executable(clustring_test src/test/clustring_test.cpp)
#add_dependencies(clustring_test ${PROJECT_NAME}_gencfg)
#add_dependencies(clustring_test ${PROJECT_NAME}_generate_messages_cpp)
#target_link_libraries(clustring_test ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#20211013 点群クラスタリングテスト
#add_executable(EKF_marker_4_test src/test/EKF_marker_4_test.cpp)
#add_dependencies(EKF_marker_4_test ${PROJECT_NAME}_gencfg)
#add_dependencies(EKF_marker_4_test ${PROJECT_NAME}_generate_messages_cpp)
#target_link_libraries(EKF_marker_4_test ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#20211015 ロボットのトピックを確認し取得する+ロボットの状態方程式
#add_executable(robot_test1 src/robot/robot_test1.cpp)
#add_dependencies(robot_test1 ${PROJECT_NAME}_gencfg)
#add_dependencies(robot_test1 ${PROJECT_NAME}_generate_messages_cpp)
#target_link_libraries(robot_test1 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#20211015 robot_test1にEKFを導入
#add_executable(robot_test2 src/robot/robot_test2.cpp)
#add_dependencies(robot_test2 ${PROJECT_NAME}_gencfg)
#add_dependencies(robot_test2 ${PROJECT_NAME}_generate_messages_cpp)
#target_link_libraries(robot_test2 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#20211018 ロボットに指令を与えるプログラム(ver0)
#add_executable(robot_teleop_test0 src/robot/robot_teleop_test0.cpp)
#add_dependencies(robot_teleop_test0 ${PROJECT_NAME}_gencfg)
#add_dependencies(robot_teleop_test0 ${PROJECT_NAME}_generate_messages_cpp)
#target_link_libraries(robot_teleop_test0 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#20211018 ロボットを指令値1m動かすプログラム(調整済み)
add_executable(robot_teleop_test src/robot/robot_teleop_test.cpp)
add_dependencies(robot_teleop_test ${PROJECT_NAME}_gencfg)
add_dependencies(robot_teleop_test ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(robot_teleop_test ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#20211019 センサー情報を取得するプログラム(観測情報に使用)
add_executable(robot_sensor1 src/robot/robot_sensor1.cpp)
add_dependencies(robot_sensor1 ${PROJECT_NAME}_gencfg)
add_dependencies(robot_sensor1 ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(robot_sensor1 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#20211019 センサー情報を同期させるプログラム(message_filtersr利用)(1よりこっちのほうが遅い...)
add_executable(robot_sensor2 src/robot/robot_sensor2.cpp)
add_dependencies(robot_sensor2 ${PROJECT_NAME}_gencfg)
add_dependencies(robot_sensor2 ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(robot_sensor2 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#20211019 カルマンフィルタ実装(message_filtersr利用)
add_executable(robot_test3 src/robot/robot_test3.cpp)
add_dependencies(robot_test3 ${PROJECT_NAME}_gencfg)
add_dependencies(robot_test3 ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(robot_test3 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#20211021 robot_test3に特徴点を追加
add_executable(robot_test4 src/robot/robot_test4.cpp)
add_dependencies(robot_test4 ${PROJECT_NAME}_gencfg)
add_dependencies(robot_test4 ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(robot_test4 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#20211021 ORB追跡テスト
add_executable(ORB_tracking_test src/test/ORB_tracking_test.cpp)
add_dependencies(ORB_tracking_test ${PROJECT_NAME}_gencfg)
add_dependencies(ORB_tracking_test ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(ORB_tracking_test ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#20211023 特徴点世界座標判別要素の導入
add_executable(robot_test5 src/robot/robot_test5.cpp)
add_dependencies(robot_test5 ${PROJECT_NAME}_gencfg)
add_dependencies(robot_test5 ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(robot_test5 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#20211023 特徴点世界座標判別要素の導入
add_executable(robot_test6 src/robot/robot_test6.cpp)
add_dependencies(robot_test6 ${PROJECT_NAME}_gencfg)
add_dependencies(robot_test6 ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(robot_test6 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#20211024 オドメトリーを利用して特徴点の運動復元
add_executable(robot_opt_test src/robot/robot_opt_test.cpp)
add_dependencies(robot_opt_test ${PROJECT_NAME}_gencfg)
add_dependencies(robot_opt_test ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(robot_opt_test ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#20211025 オドメトリーを利用してマッチングの範囲予測
add_executable(robot_opt_test2 src/robot/robot_opt_test2.cpp)
add_dependencies(robot_opt_test2 ${PROJECT_NAME}_gencfg)
add_dependencies(robot_opt_test2 ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(robot_opt_test2 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#20211026 テンプレートの保存テスト(時間別3枚の画像を保存してテンプレート確保するテスト)
add_executable(template_point_test0 src/test/template_point_test0.cpp)
add_dependencies(template_point_test0 ${PROJECT_NAME}_gencfg)
add_dependencies(template_point_test0 ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(template_point_test0 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#20211026 テンプレートの保存テスト(マッチング数が少なくなったら画像を保存してテンプレート確保するテスト)
add_executable(template_point_test0_1 src/test/template_point_test0_1.cpp)
add_dependencies(template_point_test0_1 ${PROJECT_NAME}_gencfg)
add_dependencies(template_point_test0_1 ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(template_point_test0_1 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#20211026 オドメトリーを利用してマッチングの範囲予測2(2回目までの動作)
add_executable(robot_opt_test3 src/robot/robot_opt_test3.cpp)
add_dependencies(robot_opt_test3 ${PROJECT_NAME}_gencfg)
add_dependencies(robot_opt_test3 ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(robot_opt_test3 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#20211027 3回目以降の動作導入
add_executable(robot_opt_test4 src/robot/robot_opt_test4.cpp)
add_dependencies(robot_opt_test4 ${PROJECT_NAME}_gencfg)
add_dependencies(robot_opt_test4 ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(robot_opt_test4 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#20211028 予測範囲を中心に移動
add_executable(robot_opt_test5 src/robot/robot_opt_test5.cpp)
add_dependencies(robot_opt_test5 ${PROJECT_NAME}_gencfg)
add_dependencies(robot_opt_test5 ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(robot_opt_test5 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#20211026 テンプレートの保存テスト(分割テンプレートマッチングのテスト)
add_executable(template_point_test_bunkatu src/test/template_point_test_bunkatu.cpp)
add_dependencies(template_point_test_bunkatu ${PROJECT_NAME}_gencfg)
add_dependencies(template_point_test_bunkatu ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(template_point_test_bunkatu ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#20211102 分割テンプレートマッチングのテスト(探査範囲変更)
add_executable(template_point_test_bunkatu2 src/test/template_point_test_bunkatu2.cpp)
add_dependencies(template_point_test_bunkatu2 ${PROJECT_NAME}_gencfg)
add_dependencies(template_point_test_bunkatu2 ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(template_point_test_bunkatu2 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#20211102 分割テンプレートマッチングのテスト(静止画)
add_executable(template_point_test_bunkatu2_0 src/test/template_point_test_bunkatu2_0.cpp)
add_dependencies(template_point_test_bunkatu2_0 ${PROJECT_NAME}_gencfg)
add_dependencies(template_point_test_bunkatu2_0 ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(template_point_test_bunkatu2_0 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#20211029 マッチング方法を差分二乗和から正規化相互相関に変更
add_executable(robot_opt_test6 src/robot/robot_opt_test6.cpp)
add_dependencies(robot_opt_test6 ${PROJECT_NAME}_gencfg)
add_dependencies(robot_opt_test6 ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(robot_opt_test6 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#20211029 img_template1をMP_prev_Tempに変更
add_executable(robot_opt_test6_1 src/robot/robot_opt_test6_1.cpp)
add_dependencies(robot_opt_test6_1 ${PROJECT_NAME}_gencfg)
add_dependencies(robot_opt_test6_1 ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(robot_opt_test6_1 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#20211029 テンプレート消去アルゴリズムの導入とZNCCへの変更
add_executable(robot_opt_test7 src/robot/robot_opt_test7.cpp)
add_dependencies(robot_opt_test7 ${PROJECT_NAME}_gencfg)
add_dependencies(robot_opt_test7 ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(robot_opt_test7 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#20211101 エッジ画像でのテンプレートマッチングテスト
add_executable(template_point_edge src/test/template_point_edge.cpp)
add_dependencies(template_point_edge ${PROJECT_NAME}_gencfg)
add_dependencies(template_point_edge ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(template_point_edge ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#20211102 世界座標移植アルゴリズムの導入
add_executable(robot_opt_test8 src/robot/robot_opt_test8.cpp)
add_dependencies(robot_opt_test8 ${PROJECT_NAME}_gencfg)
add_dependencies(robot_opt_test8 ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(robot_opt_test8 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#20211102 カルマンフィルタの自己位置推定テスト
add_executable(robot_opt_test9 src/robot/robot_opt_test9.cpp)
add_dependencies(robot_opt_test9 ${PROJECT_NAME}_gencfg)
add_dependencies(robot_opt_test9 ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(robot_opt_test9 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#20211105 マーカー未観測時の世界座標推定を導入(特徴点の自己位置推定完成)
add_executable(robot_opt_test10 src/robot/robot_opt_test10.cpp)
add_dependencies(robot_opt_test10 ${PROJECT_NAME}_gencfg)
add_dependencies(robot_opt_test10 ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(robot_opt_test10 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#Realsenseテストプログラム
add_executable(D435_test0 src/test/D435_test0.cpp)
add_dependencies(D435_test0 ${PROJECT_NAME}_gencfg)
add_dependencies(D435_test0 ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(D435_test0 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#20211105 マーカーのみで自己位置推定(世界座標未知マーカー使用)
add_executable(robot_test_marker src/robot/robot_test_marker.cpp)
add_dependencies(robot_test_marker ${PROJECT_NAME}_gencfg)
add_dependencies(robot_test_marker ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(robot_test_marker ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#20211108 FLD_clustering_6+マーカー検出プログラム(特徴線の自己位置推定完成)(仮)+更新動作追加済み
add_executable(FLD_marker1 src/FLD/FLD_marker1.cpp)
add_dependencies(FLD_marker1 ${PROJECT_NAME}_gencfg)
add_dependencies(FLD_marker1 ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(FLD_marker1 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#20211108 FLD_clustering_6+マーカー検出プログラム(特徴線の自己位置推定完成)(仮)+更新動作追加済み(表示コメント少)
add_executable(FLD_marker1_1 src/FLD/FLD_marker1_1.cpp)
add_dependencies(FLD_marker1_1 ${PROJECT_NAME}_gencfg)
add_dependencies(FLD_marker1_1 ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(FLD_marker1_1 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#20211109 テンプレートの更新動作を追加(特徴点)
add_executable(robot_opt_test11 src/robot/robot_opt_test11.cpp)
add_dependencies(robot_opt_test11 ${PROJECT_NAME}_gencfg)
add_dependencies(robot_opt_test11 ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(robot_opt_test11 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#20211110 特徴線テンプレマッチ自己位置推定
add_executable(robot_FLD1 src/robot/robot_FLD1.cpp)
add_dependencies(robot_FLD1 ${PROJECT_NAME}_gencfg)
add_dependencies(robot_FLD1 ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(robot_FLD1 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#20211115 特徴点＋特徴線テンプレマッチ自己位置推定(世界座標はそれぞれの要素で推定)(FLD_marker1_1+(robot_opt_test11)
add_executable(robot_FLD_point1 src/robot/robot_FLD_point1.cpp)
add_dependencies(robot_FLD_point1 ${PROJECT_NAME}_gencfg)
add_dependencies(robot_FLD_point1 ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(robot_FLD_point1 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#20211116 特徴点＋特徴線テンプレマッチ自己位置推定(世界座標データは特徴点基準で推定)
add_executable(robot_FLD_point2 src/robot/robot_FLD_point2.cpp)
add_dependencies(robot_FLD_point2 ${PROJECT_NAME}_gencfg)
add_dependencies(robot_FLD_point2 ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(robot_FLD_point2 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#20211116 特徴点＋特徴線テンプレマッチ自己位置推定(世界座標データは特徴点基準で推定)(高速化:コメント少)
add_executable(robot_FLD_point2_1 src/robot/robot_FLD_point2_1.cpp)
add_dependencies(robot_FLD_point2_1 ${PROJECT_NAME}_gencfg)
add_dependencies(robot_FLD_point2_1 ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(robot_FLD_point2_1 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#20211116 FLD_marker1_1のテンプレート中点高精度化
add_executable(FLD_marker2 src/FLD/FLD_marker2.cpp)
add_dependencies(FLD_marker2 ${PROJECT_NAME}_gencfg)
add_dependencies(FLD_marker2 ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(FLD_marker2 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#20211116 rosbag取得用プログラム
add_executable(robot_odom1 src/robot/robot_odom1.cpp)
add_dependencies(robot_odom1 ${PROJECT_NAME}_gencfg)
add_dependencies(robot_odom1 ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(robot_odom1 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#20211201 rosbag取得用プログラムVer2
add_executable(robot_odom2 src/robot/robot_odom2.cpp)
add_dependencies(robot_odom2 ${PROJECT_NAME}_gencfg)
add_dependencies(robot_odom2 ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(robot_odom2 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#20211201 rosbag取得用プログラムVer2
add_executable(robot_odom3 src/robot/robot_odom3.cpp)
add_dependencies(robot_odom3 ${PROJECT_NAME}_gencfg)
add_dependencies(robot_odom3 ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(robot_odom3 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#20211116 特徴点追跡不能時、最後の特徴点の世界座標をオドメトリーから推定するプログラム導入
add_executable(robot_opt_test12 src/robot/robot_opt_test12.cpp)
add_dependencies(robot_opt_test12 ${PROJECT_NAME}_gencfg)
add_dependencies(robot_opt_test12 ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(robot_opt_test12 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#20211117 robot_opt_test12のカメラ座標系を元の形に戻す(通常の左手系)
add_executable(robot_opt_test13 src/robot/robot_opt_test13.cpp)
add_dependencies(robot_opt_test13 ${PROJECT_NAME}_gencfg)
add_dependencies(robot_opt_test13 ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(robot_opt_test13 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#20211117 ロボット推定位置からLMのカメラ座標推定(精度悪い　間違えている可能性がある)
add_executable(robot_opt_test14 src/robot/robot_opt_test14.cpp)
add_dependencies(robot_opt_test14 ${PROJECT_NAME}_gencfg)
add_dependencies(robot_opt_test14 ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(robot_opt_test14 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#20211119 ロボット推定位置から特徴点の世界座標推定
add_executable(robot_opt_test14_2 src/robot/robot_opt_test14_2.cpp)
add_dependencies(robot_opt_test14_2 ${PROJECT_NAME}_gencfg)
add_dependencies(robot_opt_test14_2 ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(robot_opt_test14_2 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#20211118 LMのカメラ座標を自己位置から推定し、観測カメラ座標と比較テストを行う
add_executable(robot_LM_test1 src/robot/robot_LM_test1.cpp)
add_dependencies(robot_LM_test1 ${PROJECT_NAME}_gencfg)
add_dependencies(robot_LM_test1 ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(robot_LM_test1 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#20211120  オプティカルフローでのロボットの自己位置推定(テンプレマッチとの比較検討用)
add_executable(robot_test7 src/robot/robot_test7.cpp)
add_dependencies(robot_test7 ${PROJECT_NAME}_gencfg)
add_dependencies(robot_test7 ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(robot_test7 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#20211202 ロボットの動かし方変更(robot_odom2)(特徴線自己位置推定プログラム)
add_executable(FLD_marker2_2 src/FLD/FLD_marker2_2.cpp)
add_dependencies(FLD_marker2_2 ${PROJECT_NAME}_gencfg)
add_dependencies(FLD_marker2_2 ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(FLD_marker2_2 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#20211202 robot_opt_test11のロボットの動かし方変更(robot_odom2)(特徴点自己位置推定プログラム)
add_executable(robot_opt_test11_2 src/robot/robot_opt_test11_2.cpp)
add_dependencies(robot_opt_test11_2 ${PROJECT_NAME}_gencfg)
add_dependencies(robot_opt_test11_2 ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(robot_opt_test11_2 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#20211209 移植失敗時一つ前のデータを利用(特徴線自己位置推定プログラム)
add_executable(FLD_marker2_3 src/FLD/FLD_marker2_3.cpp)
add_dependencies(FLD_marker2_3 ${PROJECT_NAME}_gencfg)
add_dependencies(FLD_marker2_3 ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(FLD_marker2_3 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#20211209 移植失敗時一つ前のデータを利用+エンコーダ推定(Des→Act)(特徴線自己位置推定プログラム)
add_executable(FLD_marker2_4 src/FLD/FLD_marker2_4.cpp)
add_dependencies(FLD_marker2_4 ${PROJECT_NAME}_gencfg)
add_dependencies(FLD_marker2_4 ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(FLD_marker2_4 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#20211209 エンコーダ推定(Des→Act)(特徴線自己位置推定プログラム)
add_executable(FLD_marker2_2_4 src/FLD/FLD_marker2_2_4.cpp)
add_dependencies(FLD_marker2_2_4 ${PROJECT_NAME}_gencfg)
add_dependencies(FLD_marker2_2_4 ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(FLD_marker2_2_4 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#20211209 移植失敗時一つ前のデータを利用+エンコーダ推定(Des→Act)(FLD_marker2_2_4→FLD_marker2_2_5)
add_executable(FLD_marker2_2_5 src/FLD/FLD_marker2_2_5.cpp)
add_dependencies(FLD_marker2_2_5 ${PROJECT_NAME}_gencfg)
add_dependencies(FLD_marker2_2_5 ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(FLD_marker2_2_5 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#20211202 移植失敗時一つ前のデータを利用+エンコーダ推定(Des→Act)(robot_opt_test11_2→robot_opt_test11_3)(特徴点自己位置推定プログラム)
add_executable(robot_opt_test11_3 src/robot/robot_opt_test11_3.cpp)
add_dependencies(robot_opt_test11_3 ${PROJECT_NAME}_gencfg)
add_dependencies(robot_opt_test11_3 ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(robot_opt_test11_3 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})


