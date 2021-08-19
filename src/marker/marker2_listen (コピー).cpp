//観測マーカーと世界座標マーカーとのズレを観測する
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_listener.h>
#include <std_msgs/String.h>

#include <string>
#include <math.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "marker2_listen");
  ros::NodeHandle nh;

  tf::TransformListener ln_MAP_MarkertoC_Marker;

  ros::Rate loop_rate(10);

    while (ros::ok())
  {
    //観測マーカー
    geometry_msgs::PoseStamped source_marker_pose;
    source_marker_pose.header.frame_id = "marker_link";
    source_marker_pose.header.stamp = ros::Time::now();
    source_marker_pose.pose.orientation.w = 1.0;

    geometry_msgs::PoseStamped MAP_MarkertoC_Marker_pose;//観測マーカーと地図上のマーカーとのズレを保管

    //MAP_Marker
    std::string MAP_MarkertoC_Marker_frame = "MAP_Marker_link";
    try
    {

      ln_MAP_MarkertoC_Marker.waitForTransform(source_marker_pose.header.frame_id, MAP_MarkertoC_Marker_frame, source_marker_pose.header.stamp, ros::Duration(1.0));//map_linkとbady2_linkとの差を計算
      ln_MAP_MarkertoC_Marker.transformPose(MAP_MarkertoC_Marker_frame, source_marker_pose, MAP_MarkertoC_Marker_pose);


      std::cout << "観測マーカーと地図上のマーカーとの差を計算={"<< MAP_MarkertoC_Marker_pose.pose.position.x <<","<<MAP_MarkertoC_Marker_pose.pose.position.y<<","<<MAP_MarkertoC_Marker_pose.pose.position.z<<"}"<< std::endl;

    }
    catch (...)
    {
      ROS_INFO("tf error");
    }
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}