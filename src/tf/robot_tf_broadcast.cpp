#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>

#include <string>
#include <math.h>
std::string source_frame = "map_link";//マップフレーム

int main(int argc, char** argv)
{
  ros::init(argc, argv, "vis_tf_publisher");
  ros::NodeHandle nh;

  ros::Rate loop_rate(10);
  int kaisu = 0;
  while (ros::ok())
  {
    float xyz[3] = { 0, 0, 0 };
    xyz[0] = cos((float)kaisu / 5.0);
    xyz[1] = sin((float)kaisu / 5.0);

    
/*
    std::string target_frame = "body_link";
    geometry_msgs::Pose t_pose;
    t_pose.position.x = xyz[0];
    t_pose.position.y = xyz[1];
    t_pose.orientation.w = 1.0;

    static tf::TransformBroadcaster br;
    tf::Transform transform;
    poseMsgToTF(t_pose, transform);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), source_frame, target_frame));

    std::string target_frame2 = "body2_link";
    geometry_msgs::Pose t2_pose;
    t2_pose.position.x = -xyz[0];
    t2_pose.position.y = -xyz[1];
    t2_pose.position.z = 1.5;
    t2_pose.orientation.w = 1.0;

    static tf::TransformBroadcaster br2;
    tf::Transform transform2;
    poseMsgToTF(t2_pose, transform2);
    br2.sendTransform(tf::StampedTransform(transform2, ros::Time::now(), source_frame, target_frame2));//mapとtarget_frame2をくっつけてる

    std::string target_frame3 = "body3_link";
    geometry_msgs::Pose t3_pose;
    t3_pose.position.x = -cos((float)kaisu / 3.0)/1.5;
    t3_pose.position.y = -sin((float)kaisu / 3.0)/1.5;
    t3_pose.position.z = 1;
    t3_pose.orientation.w = 1.0+20;

    static tf::TransformBroadcaster br3;
    tf::Transform transform3;
    poseMsgToTF(t3_pose, transform3);
    br3.sendTransform(tf::StampedTransform(transform3, ros::Time::now(), target_frame2, target_frame3));//targt_frame2とtarget_frame3をくっつけてる

    std::string target_frame4 = "body4_link";
    geometry_msgs::Pose t4_pose;
    t4_pose.position.x = -cos((float)kaisu / 3.0)/1.5;
    t4_pose.position.y = -sin((float)kaisu / 3.0)/1.5;
    t4_pose.position.z = -0.5;
    t4_pose.orientation.w = 1.0+20;

    static tf::TransformBroadcaster br4;
    tf::Transform transform4;
    poseMsgToTF(t4_pose, transform4);
    br4.sendTransform(tf::StampedTransform(transform4, ros::Time::now(), target_frame3, target_frame4));//targt_frame3とtarget_frame4をくっつけてる

    std::cout << "bady2_linkとbady4_linkとの差を計算={"<< t4_pose.position.x-t2_pose.position.x <<","<<t4_pose.position.y-t2_pose.position.y<<","<<t4_pose.position.z-t2_pose.position.z<<"}"<< std::endl;
    std::cout << "bady3_linkとbady4_linkとの差を計算={"<< t4_pose.position.x-t3_pose.position.x <<","<<t4_pose.position.y-t3_pose.position.y<<","<<t4_pose.position.z-t3_pose.position.x<<"}"<< std::endl;
*/
    /*std::string target_robot_frame = "robot_link";
    geometry_msgs::Pose robot_pose;
    robot_pose.position.x = 0+(kaisu*0.01);//赤
    robot_pose.position.y = 0;//緑
    robot_pose.position.z = 1;//青
    robot_pose.orientation.w = 1.0;

    static tf::TransformBroadcaster br_robot;
    tf::Transform transform_robot;
    poseMsgToTF(robot_pose, transform_robot);
    br_robot.sendTransform(tf::StampedTransform(transform_robot, ros::Time::now(), source_frame, target_robot_frame));*/

    std::string target_marker_frame = "marker_link";
    geometry_msgs::Pose marker_pose;
    marker_pose.position.x = 1;//赤
    marker_pose.position.y = 1;//緑
    marker_pose.position.z = 1;//青
    marker_pose.orientation.w = 1.0;

    static tf::TransformBroadcaster br_marker;
    tf::Transform transform_marker;
    poseMsgToTF(marker_pose, transform_marker);
    br_marker.sendTransform(tf::StampedTransform(transform_marker, ros::Time::now(), source_frame, target_marker_frame));

    std::string target_marker2_frame = "marker2_link";
    geometry_msgs::Pose marker2_pose;
    marker2_pose.position.x = 2;//赤
    marker2_pose.position.y = 1;//緑
    marker2_pose.position.z = 1;//青
    marker2_pose.orientation.w = 1.0;

    static tf::TransformBroadcaster br_marker2;
    tf::Transform transform_marker2;
    poseMsgToTF(marker2_pose, transform_marker2);
    br_marker2.sendTransform(tf::StampedTransform(transform_marker2, ros::Time::now(), source_frame, target_marker2_frame));

    std::string target_robot_frame = "robot_link";//ロボットはマーカーに対して動いてる（ここで与えてるのはマーカーに対するロボットの位置変化量のみ）
    geometry_msgs::Pose robot_pose;
    robot_pose.position.x = 0+(kaisu*0.01);//赤//ここはマーカーとの差が書いてある（つまり座標ではない）
    robot_pose.position.y = 0;//緑
    robot_pose.position.z = 0;//青
    robot_pose.orientation.w = 1.0;

    static tf::TransformBroadcaster br_robot;
    tf::Transform transform_robot;
    poseMsgToTF(robot_pose, transform_robot);
    br_robot.sendTransform(tf::StampedTransform(transform_robot, ros::Time::now(), target_marker_frame, target_robot_frame));





    kaisu++;
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
