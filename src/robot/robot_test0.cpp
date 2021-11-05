//特徴点検出のテストプログラム
#define _CRT_SECURE_NO_WARNINGS
#include <ros/ros.h>
#include <iostream>
#include <string>
#include <vector>
#include <opencv2/opencv.hpp>
#include <opencv/highgui.h>
#include <cv_bridge/cv_bridge.h>//画像変換のヘッダ
#include <sensor_msgs/Image.h>//センサーデータ形式ヘッダ
#include <sensor_msgs/image_encodings.h>//エンコードのためのヘッダ
#include <sensor_msgs/CameraInfo.h>//camera_infoを獲得するためのヘッダー
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <time.h>//処理の時間を出力する
#include <sys/time.h>

#include <geometry_msgs/PoseStamped.h>//tf
#include <tf/transform_broadcaster.h>//tf
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <geometry_msgs/Twist.h>//ロボットの指令値(速度)用ヘッダー
#include <nav_msgs/Odometry.h>


std::string win_src = "src";
std::string win_dst = "dst";
std::string win_point = "point";
ros::Subscriber robot_sub;
ros::Subscriber odom_sub;


using namespace std;
using namespace cv;


cv::Mat img_dst;//画像定義
vector<cv::Point2f> points_prev, points_curr;//特徴点定義
vector<cv::Point3f> camera_point_p,camera_point_c;//特徴点定義
sensor_msgs::CameraInfo camera_info;//CameraInfo受け取り用
geometry_msgs::Twist robot_velocity;//指令速度
nav_msgs::Odometry robot_odometry;//ロボットのオドメトリー

int kaisu=0,kaisu2=0;


ros::Time ros_begin;//プログラム時間
ros::WallTime wall_begin = ros::WallTime::now();//プログラム開始時の時間
ros::WallDuration wall_prev;//一つ前の時間
ros::WallDuration wall_systemtime;//サンプリング時間
//ros::Time ros_begin = ros::Time::now();
struct timeval startTime, endTime;  // 構造体宣言
struct timeval startTime2, endTime2;  // 構造体宣言
float realsec,realsec2;//サンプリング時間（C++)
float ALLrealsec,ALLrealsec2;//サンプリング時間（C++)

double position_masterX=0,position_masterY=0,position_masterZ=0;//ロボットの指令位置(理想値)
double orientation_masterX=0,orientation_masterY=0,orientation_masterZ=0;//ロボットの指令角度(理想値)(ここの計算間違えているので注意)
double robot_positionX,robot_positionY,robot_positionZ;//ロボットの観測位置(エンコーダー観測値)
double robot_orientationX,robot_orientationY,robot_orientationZ;//ロボットの姿勢観測値(エンコーダー観測値)(オイラー角)

double roll, pitch, yaw;//クオータニオン→オイラー角変換用
double RobotX=0,RobotY=0,RobotTH=0;//ロボットの状態方程式
double RobotV;//ロボットの速度ベクトル


//コールバック関数
void callback(const sensor_msgs::Image::ConstPtr& rgb_msg,const sensor_msgs::Image::ConstPtr& depth_msg, const sensor_msgs::CameraInfo::ConstPtr& cam_info)
{
  ROS_INFO("callback_functionが呼ばれたよ");

  ////サンプリング時間取得(C言語の方法)(こっちのほうが正確らしい)
  //gettimeofday(&startTime, NULL);// 開始時刻取得
  //if(kaisu!=0){
  //  time_t diffsec = difftime(startTime.tv_sec, endTime.tv_sec);    // 秒数の差分を計算
  //  suseconds_t diffsub = startTime.tv_usec - endTime.tv_usec;      // マイクロ秒部分の差分を計算
  //  realsec = diffsec+diffsub*1e-6;                          // 実時間を計算
  //  printf("処理の時間=%f\n", realsec);
  //}

//サンプリング時間取得(ROS)
  ros::WallTime wall_now = ros::WallTime::now();
  ros::WallDuration wall_duration = wall_now - wall_begin;
  ROS_INFO("WALL:%u.%09u", wall_duration.sec, wall_duration.nsec);
  wall_systemtime = wall_duration - wall_prev;
  //ROS_INFO("systemtime:%u.%09u", wall_systemtime.sec, wall_systemtime.nsec);
std::cout << "wall_systemtime=" <<wall_systemtime<< std::endl;//サンプリング時間

    //変数宣言
  cv_bridge::CvImagePtr bridgeImage;//クラス::型//cv_brigeは画像変換するとこ
  cv_bridge::CvImagePtr bridgedepthImage;//クラス::型//cv_brigeは画像変換するとこ
  cv::Mat RGBimage,depthimage,image;
//
    try{//MAT形式変換
       bridgeImage=cv_bridge::toCvCopy(rgb_msg, sensor_msgs::image_encodings::BGR8);//MAT形式に変える
       ROS_INFO("callBack");//printと秒数表示
    }
	  //エラー処理
    catch(cv_bridge::Exception& e) {//エラー処理(失敗)成功ならスキップ
        std::cout<<"depth_image_callback Error \n";
        ROS_ERROR("Could not convert from '%s' to 'BGR8'.",rgb_msg->encoding.c_str());
        return ;
    }
    try{//MAT形式変換
       bridgedepthImage=cv_bridge::toCvCopy(depth_msg, sensor_msgs::image_encodings::TYPE_32FC1);//MAT形式に変える
       ROS_INFO("callBack");}//printと秒数表示
	  //エラー処理
    catch(cv_bridge::Exception& e) {//エラー処理(失敗)成功ならスキップ
        std::cout<<"depth_image_callback Error \n";
        ROS_ERROR("Could not convert from '%s' to '32FC1'.",depth_msg->encoding.c_str());
        return ;}
//  
  	camera_info=*cam_info;//CameraInfo受け取り
   
  	image = bridgeImage->image.clone();//image変数に変換した画像データを代入
  	depthimage = bridgedepthImage->image.clone();//image変数に変換した画像データを代入

	  img_dst = image.clone();
    img_dst = cv::Scalar(255,255,255);

  
    // 画面表示
    cv::imshow(win_src, image);
    cv::imshow(win_dst, img_dst);
	kaisu++;

    cv::waitKey(1);//ros::spinにジャンプする
}

// 指令値の並進、回転速度の表示。(並進速度(m/s)。前方向が正,回転速度(rad/s)。反時計回りが正)                                   
void callback2(const geometry_msgs::Twist::ConstPtr& vel) {

  //サンプリング時間取得(C言語の方法)(こっちのほうが正確らしい)
  gettimeofday(&startTime, NULL);// 開始時刻取得
  if(kaisu!=0){
    time_t diffsec = difftime(startTime.tv_sec,endTime.tv_sec);    // 秒数の差分を計算
    suseconds_t diffsub = startTime.tv_usec - endTime.tv_usec;      // マイクロ秒部分の差分を計算
    realsec = diffsec+diffsub*1e-6;                          // 実時間を計算
    ALLrealsec=ALLrealsec+realsec;
    printf("処理の時間=%f\n", realsec);
    printf("処理時間合計=%f\n", ALLrealsec);
  }
  robot_velocity=*vel;
  std::cout << "robot_velocity.linear.x" <<robot_velocity.linear.x<< std::endl;
  std::cout << "robot_velocity.linear.y" <<robot_velocity.linear.y<< std::endl;
  std::cout << "robot_velocity.linear.z" <<robot_velocity.linear.z<< std::endl;
  std::cout << "robot_velocity.angular.x" <<robot_velocity.angular.x<< std::endl;
  std::cout << "robot_velocity.angular.y" <<robot_velocity.angular.y<< std::endl;
  std::cout << "robot_velocity.angular.z" <<robot_velocity.angular.z<< std::endl;

  //cout << "Linear.x :" << vel->linear.x << endl;
  //cout << "Linear.y :" << vel->linear.y << endl;
  //cout << "Linear.z :" << vel->linear.z << endl;
  //cout << "Angular.x:" << vel->angular.x << endl;
  //cout << "Angular.y:" << vel->angular.y << endl;
  //cout << "Angular.z:" << vel->angular.z << endl;
  //cout << "Angular.w:" << vel->angular.w << endl;

  position_masterX=position_masterX+(robot_velocity.linear.x*realsec);
  position_masterY=position_masterY+(robot_velocity.linear.y*realsec);
  position_masterZ=position_masterZ+(robot_velocity.linear.z*realsec);
  orientation_masterX=orientation_masterX+(robot_velocity.angular.z*realsec);
  orientation_masterY=orientation_masterY+(robot_velocity.angular.y*realsec);
  orientation_masterZ=orientation_masterZ+(robot_velocity.angular.z*realsec);

  std::cout << "position_masterX=" <<position_masterX<< std::endl;
  std::cout << "position_masterY=" <<position_masterY<< std::endl;
  std::cout << "position_masterZ=" <<position_masterZ<< std::endl;
  std::cout << "orientation_masterX=" <<orientation_masterX<< std::endl;
  std::cout << "orientation_masterY=" <<orientation_masterY<< std::endl;
  std::cout << "orientation_masterZ=" <<orientation_masterZ<< std::endl;
  std::cout << "robot_positionX=" <<robot_positionX<< std::endl;
  std::cout << "robot_positionY=" <<robot_positionY<< std::endl;
  std::cout << "robot_positionZ=" <<robot_positionZ<< std::endl;
  std::cout << "robot_orientationX=" <<robot_orientationX<< std::endl;
  std::cout << "robot_orientationY=" <<robot_orientationY<< std::endl;
  std::cout << "robot_orientationZ=" <<robot_orientationZ<< std::endl;

  kaisu++;
  endTime=startTime;//動作終了時刻取得
  cv::waitKey(1);//ros::spinにジャンプする
}

//ロボットのオドメトリー(ロボットの観測値)
void odomcallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  //複数関数を分けた時の処理時間の取得方法について聞く
   //サンプリング時間取得(C言語の方法)(こっちのほうが正確らしい)
  gettimeofday(&startTime2, NULL);// 開始時刻取得
  if(kaisu2!=0){
    time_t diffsec = difftime(startTime2.tv_sec,endTime2.tv_sec);    // 秒数の差分を計算
    suseconds_t diffsub = startTime2.tv_usec - endTime2.tv_usec;      // マイクロ秒部分の差分を計算
    realsec2 = diffsec+diffsub*1e-6;                          // 実時間を計算
    ALLrealsec2=ALLrealsec2+realsec2;
    printf("処理の時間=%f\n", realsec2);
    printf("処理時間合計=%f\n", ALLrealsec2);
  }

  robot_odometry=*msg;
  ROS_INFO("Seq: [%d]", msg->header.seq);
  //ロボットの実測位置
  ROS_INFO("Position-> x: [%f], y: [%f], z: [%f]", msg->pose.pose.position.x,msg->pose.pose.position.y, msg->pose.pose.position.z);
  //ロボットの姿勢実測値
  ROS_INFO("Orientation-> x: [%f], y: [%f], z: [%f], w: [%f]", msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
  //ロボットの速度と回転速度(実測値)
  ROS_INFO("Vel-> Linear: [%f], Angular: [%f]", msg->twist.twist.linear.x,msg->twist.twist.angular.z);

  //std::cout << "robot_odometry.pose.pose.position.x=" <<robot_odometry.pose.pose.position.x<< std::endl;
  //std::cout << "robot_odometry.pose.pose.position.y=" <<robot_odometry.pose.pose.position.y<< std::endl;
  //std::cout << "robot_odometry.pose.pose.position.z=" <<robot_odometry.pose.pose.position.z<< std::endl;
  //std::cout << "robot_odometry.pose.pose.orientation.x=" <<robot_odometry.pose.pose.orientation.x<< std::endl;
  //std::cout << "robot_odometry.pose.pose.orientation.y=" <<robot_odometry.pose.pose.orientation.y<< std::endl;
  //std::cout << "robot_odometry.pose.pose.orientation.z=" <<robot_odometry.pose.pose.orientation.z<< std::endl;
  //std::cout << "robot_odometry.pose.pose.orientation.w=" <<robot_odometry.pose.pose.orientation.w<< std::endl;

  robot_positionX=robot_odometry.pose.pose.position.x;
  robot_positionY=robot_odometry.pose.pose.position.y;
  robot_positionZ=robot_odometry.pose.pose.position.z;
  tf::Quaternion quat(robot_odometry.pose.pose.orientation.x,robot_odometry.pose.pose.orientation.y,robot_odometry.pose.pose.orientation.z,robot_odometry.pose.pose.orientation.w);
  tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);//クォータニオン→オイラー角
  robot_orientationX=yaw;
  robot_orientationY=pitch;
  robot_orientationZ=roll;

  //std::cout << "robot_positionX=" <<robot_positionX<< std::endl;
  //std::cout << "robot_positionY=" <<robot_positionY<< std::endl;
  //std::cout << "robot_positionZ=" <<robot_positionZ<< std::endl;


  //robot_linear = (cv::Mat_<double>(1, 2) << robot_odometry.twist.twist.linear.x,robot_odometry.twist.twist.linear.y);

  std::cout << "robot_odometry.twist.twist.linear.x=" <<robot_odometry.twist.twist.linear.x<< std::endl;
  std::cout << "robot_odometry.twist.twist.linear.y=" <<robot_odometry.twist.twist.linear.y<< std::endl;
  std::cout << "robot_odometry.twist.twist.linear.z=" <<robot_odometry.twist.twist.linear.z<< std::endl;
  std::cout << "robot_odometry.twist.twist.angular.x=" <<robot_odometry.twist.twist.angular.x<< std::endl;
  std::cout << "robot_odometry.twist.twist.angular.y=" <<robot_odometry.twist.twist.angular.y<< std::endl;
  std::cout << "robot_odometry.twist.twist.angular.z=" <<robot_odometry.twist.twist.angular.z<< std::endl;
  std::cout << "robot_positionX=" <<robot_positionX<< std::endl;
  std::cout << "robot_positionY=" <<robot_positionY<< std::endl;
  std::cout << "robot_positionZ=" <<robot_positionZ<< std::endl;
  std::cout << "robot_orientationX=" <<robot_orientationX<< std::endl;
  std::cout << "robot_orientationY=" <<robot_orientationY<< std::endl;
  std::cout << "robot_orientationZ=" <<robot_orientationZ<< std::endl;



  RobotV=robot_odometry.twist.twist.linear.x+robot_odometry.twist.twist.linear.y;//速度ベクトルの合成
  std::cout << "RobotV=" <<RobotV<< std::endl;

  //ロボットの状態方程式
  if(robot_odometry.twist.twist.angular.z==0){
    RobotX=RobotX+(RobotV*cos(RobotTH)*realsec2);
    RobotY=RobotY+(RobotV*sin(RobotTH)*realsec2);
    RobotTH=RobotTH+(robot_odometry.twist.twist.angular.z*realsec2);
  }
  else{
    RobotX=RobotX+((RobotV/robot_odometry.twist.twist.angular.z)*(sin(RobotTH+robot_odometry.twist.twist.angular.z*realsec2)-sin(RobotTH)));
    RobotY=RobotY+((RobotV/robot_odometry.twist.twist.angular.z)*(-cos(RobotTH+robot_odometry.twist.twist.angular.z*realsec2)+cos(RobotTH)));
    RobotTH=RobotTH+(robot_odometry.twist.twist.angular.z*realsec2);
  }

  std::cout << "RobotX=" <<RobotX<< std::endl;
  std::cout << "RobotY=" <<RobotY<< std::endl;
  std::cout << "RobotTH=" <<RobotTH<< std::endl;

  endTime2=startTime2;//動作終了時刻取得
  kaisu2++;
  cv::waitKey(1);//ros::spinにジャンプする

}


//メイン関数
int main(int argc,char **argv){

	ros::init(argc,argv,"marker2");//rosを初期化
	ros::NodeHandle nhSub;//ノードハンドル
  if(kaisu!=0){ros_begin = ros::Time::now();}
	//subscriber関連
  //Realsensesの時(roslaunch realsense2_camera rs_camera.launch align_depth:=true)(Depth修正版なのでこっちを使うこと)
	message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nhSub, "/camera/color/image_raw", 1);//センサーメッセージを使うときは対応したヘッダーが必要
	message_filters::Subscriber<sensor_msgs::Image> depth_sub(nhSub, "/camera/aligned_depth_to_color/image_raw", 1);
	message_filters::Subscriber<sensor_msgs::CameraInfo> info_sub(nhSub, "/camera/color/camera_info", 1);

  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,sensor_msgs::Image,sensor_msgs::CameraInfo> MySyncPolicy;	
	message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10),rgb_sub, depth_sub, info_sub);
	sync.registerCallback(boost::bind(&callback,_1, _2, _3));

  ros::NodeHandle velonh;
  //subscriberの作成。トピック/cmd_velを購読する。バッファ数は10。               
  ros::Subscriber robot_sub = velonh.subscribe("/robot1/mobile_base/commands/velocity", 10, callback2);//ロボットの指令値

  //subscriberの作成。トピック/cmd_velを購読する。バッファ数は10。               
  ros::NodeHandle odomnh;
  ros::Subscriber odom_sub = odomnh.subscribe("/robot1/odom", 10, odomcallback);//ロボットのオドメトリー
  
	ros::spin();//トピック更新待機
			
	return 0;
}