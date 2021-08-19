#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_listener.h>

#include <string>
#include <math.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "vis_tf_listen");
  ros::NodeHandle nh;

  tf::TransformListener ln;
  tf::TransformListener ln3;
  tf::TransformListener ln2_4;
  tf::TransformListener ln3_4;
  tf::TransformListener ln_marker_robot;
  tf::TransformListener ln_marker2_robot;


  ros::Rate loop_rate(10);
/*  
  while (ros::ok())
  {
    geometry_msgs::PoseStamped source_pose;
    source_pose.header.frame_id = "body2_link";
    source_pose.header.stamp = ros::Time::now();
    source_pose.pose.orientation.w = 1.0;

    geometry_msgs::PoseStamped source3_pose;
    source3_pose.header.frame_id = "body3_link";
    source3_pose.header.stamp = ros::Time::now();
    source3_pose.pose.orientation.w = 1.0;
    

    geometry_msgs::PoseStamped target_pose;
    geometry_msgs::PoseStamped target3_pose;
    geometry_msgs::PoseStamped target_pose2_4;
    geometry_msgs::PoseStamped target_pose3_4;



    std::string target_frame = "map_link";
    std::string target4_frame = "body4_link";
    try
    {
      //ln.waitForTransform(source_pose.header.frame_id, "/robot1/camera_link", source_pose.header.stamp, ros::Duration(1.0));
      //ln.transformPose("/robot1/camera_link", source_pose, target_pose);

      ln.waitForTransform(source_pose.header.frame_id, target_frame, source_pose.header.stamp, ros::Duration(1.0));//map_linkとbady2_linkとの差を計算
      ln.transformPose(target_frame, source_pose, target_pose);

      ln3.waitForTransform(source3_pose.header.frame_id, target_frame, source3_pose.header.stamp, ros::Duration(1.0));//map_linkとbady3_linkとの差を計算
      ln3.transformPose(target_frame, source3_pose, target3_pose);

      ln2_4.waitForTransform(source_pose.header.frame_id, target4_frame, source_pose.header.stamp, ros::Duration(1.0));//bady2_linkとbady4_linkとの差を計算
      ln2_4.transformPose(target4_frame, source_pose, target_pose2_4);

      ln3_4.waitForTransform(source3_pose.header.frame_id, target4_frame, source3_pose.header.stamp, ros::Duration(1.0));//bady3_linkとbady4_linkとの差を計算
      ln3_4.transformPose(target4_frame, source3_pose, target_pose3_4);

      //std::cout << "map_linkとbady2_linkとの差を計算={"<< target_pose.pose.position.x <<","<<target_pose.pose.position.y<<","<<target_pose.pose.position.z<<"}"<< std::endl;
      //std::cout << "map_linkとbady3_linkとの差を計算={"<< target3_pose.pose.position.x <<","<<target3_pose.pose.position.y<<","<<target3_pose.pose.position.z<<"}"<< std::endl;
      std::cout << "bady2_linkとbady4_linkとの差を計算={"<< target_pose2_4.pose.position.x <<","<<target_pose2_4.pose.position.y<<","<<target_pose2_4.pose.position.z<<"}"<< std::endl;
      std::cout << "bady3_linkとbady4_linkとの差を計算={"<< target_pose3_4.pose.position.x <<","<<target_pose3_4.pose.position.y<<","<<target_pose3_4.pose.position.z<<"}"<< std::endl;

    }
    catch (...)
    {
      ROS_INFO("tf error");
    }
    ros::spinOnce();
    loop_rate.sleep();
  }
*/  

    while (ros::ok())
  {
    geometry_msgs::PoseStamped source_marker_pose;
    source_marker_pose.header.frame_id = "marker_link";
    source_marker_pose.header.stamp = ros::Time::now();
    source_marker_pose.pose.orientation.w = 1.0;

    geometry_msgs::PoseStamped source_marker2_pose;
    source_marker2_pose.header.frame_id = "marker2_link";
    source_marker2_pose.header.stamp = ros::Time::now();
    source_marker2_pose.pose.orientation.w = 1.0;
    

    geometry_msgs::PoseStamped target_robot_pose;
    geometry_msgs::PoseStamped target2_robot_pose;


    std::string target_frame = "robot_link";
    //std::string target2_frame = "body4_link";
    try
    {
      //ln.waitForTransform(source_pose.header.frame_id, "/robot1/camera_link", source_pose.header.stamp, ros::Duration(1.0));
      //ln.transformPose("/robot1/camera_link", source_pose, target_pose);

      ln_marker_robot.waitForTransform(source_marker_pose.header.frame_id, target_frame, source_marker_pose.header.stamp, ros::Duration(1.0));//map_linkとbady2_linkとの差を計算
      ln_marker_robot.transformPose(target_frame, source_marker_pose, target_robot_pose);

      ln_marker2_robot.waitForTransform(source_marker2_pose.header.frame_id, target_frame, source_marker2_pose.header.stamp, ros::Duration(1.0));//map_linkとbady3_linkとの差を計算
      ln_marker2_robot.transformPose(target_frame, source_marker2_pose, target2_robot_pose);


      //std::cout << "map_linkとbady2_linkとの差を計算={"<< target_pose.pose.position.x <<","<<target_pose.pose.position.y<<","<<target_pose.pose.position.z<<"}"<< std::endl;
      //std::cout << "map_linkとbady3_linkとの差を計算={"<< target3_pose.pose.position.x <<","<<target3_pose.pose.position.y<<","<<target3_pose.pose.position.z<<"}"<< std::endl;
      std::cout << "marker_linkとrobot_linkとの差を計算={"<< target_robot_pose.pose.position.x <<","<<target_robot_pose.pose.position.y<<","<<target_robot_pose.pose.position.z<<"}"<< std::endl;
      std::cout << "marke2r_linkとrobot_linkとの差を計算={"<< target2_robot_pose.pose.position.x <<","<<target2_robot_pose.pose.position.y<<","<<target2_robot_pose.pose.position.z<<"}"<< std::endl;

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