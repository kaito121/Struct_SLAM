#include <ros/ros.h>
#include <iostream>
#include <string>
#include <vector>
#include <opencv2/opencv.hpp>
#include <opencv/highgui.h>
#include <cv_bridge/cv_bridge.h>//画像変換のヘッダ
#include <sensor_msgs/Image.h>//センサーデータ形式ヘッダ
#include <sensor_msgs/image_encodings.h>//エンコードのためのヘッダ
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/CameraInfo.h>//camera_infoを獲得するためのヘッダー
#include <struct_slam/marker_tf.h>//自作メッセージ用ヘッダ#include<develのファイル名/メッセージファイル名>
#include <time.h>//処理の時間を出力する



int main(int argc, char** argv)
{
  ros::init(argc, argv, "adv_time");
  ros::NodeHandle n;

  //ros::Time::waitForValid();
  ros::Time ros_begin = ros::Time::now();
  ros::WallTime wall_begin = ros::WallTime::now();

  //ros::Rate loop_rate(1);
  for(int i=0; i<10000; i++)
  {
    ROS_INFO("TICK");

    ros::Time ros_now = ros::Time::now();
    ros::Duration ros_duration = ros_now - ros_begin;
    ROS_INFO("ROS: %u.09%u", ros_duration.sec, ros_duration.nsec);

    ros::WallTime wall_now = ros::WallTime::now();
    ros::WallDuration wall_duration = wall_now - wall_begin;
    ROS_INFO("WALL:%u.%09u", wall_duration.sec, wall_duration.nsec);

    char date[64];
    time_t t = ros::Time::now().sec;
    strftime(date, sizeof(date), "%Y/%m/%d %a %H:%M:%S", localtime(&t));
    ROS_INFO("%s", date);

    //ros::spinOnce();
    //loop_rate.sleep();
  }
  return 0;
}


/*int main(int argc, char** argv)
{
	ros::init(argc, argv, "mesureing_computation_time");
	ros::NodeHandle nh;

	ros::Time t_start = ros::Time::now();
    ROS_INFO("%lf",ros::Time::now().toSec());

	//Start
	for(int i=0; i<10000; i++){
         ROS_INFO("a_%lf",ros::Time::now().toSec());
		std::cout << "i = " << i << std::endl;
        double secs =ros::Time::now().toSec();
        ros::Time ros_now = ros::Time::now();
        ros::Duration ros_duration = ros_now - t_start;
        ROS_INFO("ROS: %u.09%u", ros_duration.sec, ros_duration.nsec);
        std::cout << std::setprecision(20) << ros::Time::now().toSec() << std::endl;
        //ros::Duration d(0.6);
        //secs = d.toSec();
		std::cout << "secs = " << secs << std::endl;

		std::cout << "time[s] = " << (ros::Time::now() - t_start).toSec() << std::endl;
	}
	//End

	std::cout << "Computation is done" << std::endl;
	std::cout << "Computation time[s] = " << (ros::Time::now() - t_start).toSec() << std::endl;

	return 0;
}*/