//ロボットを指令値1m動かすプログラム
//Ver0では入力と出力の周波数が異なっていたため、このプログラムで出力を確認してから入力を出すようにしている
//そうすることで入力(指令値)と出力(観測値)の時間を合わせている。
//また入力開始と出力開始時にラグが生じるため、ifでラグの調整を行なっている
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
#include <nav_msgs/Path.h>//経路情報を記録する

#include <geometry_msgs/Twist.h>//ロボットの指令値(速度)用ヘッダー
#include <nav_msgs/Odometry.h>
#include<fstream>

std::string win_src = "src";
std::string win_dst = "dst";
std::string win_point = "point";
std::string source_frame = "map";//mapフレーム

ros::Subscriber odom_sub;//ロボットオドメトリー受信用
ros::Publisher  pub;//速度送信用
ros::Publisher Des_pub_plan;//カメラ経路送信用(指令値)
ros::Publisher Act_pub_plan;//カメラ経路送信用(観測値)

bool X_25=false;
bool TH_90=false;
bool Y_05=false;

using namespace std;
using namespace cv;


cv::Mat img_dst;//画像定義
vector<cv::Point2f> points_prev, points_curr;//特徴点定義
vector<cv::Point3f> camera_point_p,camera_point_c;//特徴点定義
sensor_msgs::CameraInfo camera_info;//CameraInfo受け取り用
geometry_msgs::Twist robot_velocity;//指令速度
nav_msgs::Odometry robot_odometry;//ロボットのオドメトリー

int kaisu=0,kaisu2=0,kaisuM1=0,kaisuV1=0;

ros::Time ros_begin;//プログラム時間
ros::WallTime wall_begin = ros::WallTime::now();//プログラム開始時の時間
ros::WallDuration wall_prev;//一つ前の時間
ros::WallDuration wall_systemtime;//サンプリング時間
//ros::Time ros_begin = ros::Time::now();
struct timeval startTime, endTime;  // 構造体宣言
struct timeval startTime2, endTime2;  // 構造体宣言
struct timeval startTimeV1, endTimeV1;  // 構造体宣言
struct timeval startTimeM1, endTimeM1;  // 構造体宣言

float realsec,realsec2;//サンプリング時間（C++)
float ALLrealsec,ALLrealsec2;//サンプリング時間（C++)
float realsecM1,ALLrealsecM1;//サンプリング時間（C++)
float realsecV1,ALLrealsecV1;//サンプリング時間（C++)

double position_masterX=0,position_masterY=0,position_masterZ=0;//ロボットの指令位置(理想値)
double orientation_masterX=0,orientation_masterY=0,orientation_masterZ=0;//ロボットの指令角度(理想値)(ここの計算間違えているので注意)
double robot_positionX,robot_positionY,robot_positionZ;//ロボットの観測位置(エンコーダー観測値)
double robot_orientationX,robot_orientationY,robot_orientationZ;//ロボットの姿勢観測値(エンコーダー観測値)(オイラー角)

double roll, pitch, yaw;//クオータニオン→オイラー角変換用
double Act_RobotX=0,Act_RobotY=0,Act_RobotTH=0;//ロボットの状態方程式(実際の状態)
double Des_RobotX=0,Des_RobotY=0,Des_RobotTH=0;//ロボットの状態方程式(理想状態)
double Est_RobotX=0,Est_RobotY=0,Est_RobotTH=0;//ロボットの状態方程式(推定状態)
double Act_RobotV,Des_RobotV;//ロボットの速度ベクトル(実測値,指令値)

ofstream act_v("/home/fuji/catkin_ws/src/Struct_SLAM/src/robot/ACT_V.txt");
ofstream act_u("/home/fuji/catkin_ws/src/Struct_SLAM/src/robot/ACT_ω.txt");
ofstream act_robot("/home/fuji/catkin_ws/src/Struct_SLAM/src/robot/ACT_robot.txt");
ofstream act_time("/home/fuji/catkin_ws/src/Struct_SLAM/src/robot/ACT_time.txt");
ofstream act_ALLtime("/home/fuji/catkin_ws/src/Struct_SLAM/src/robot/ACT_ALLtime.txt");
ofstream des_v("/home/fuji/catkin_ws/src/Struct_SLAM/src/robot/DES_V.txt");
ofstream des_u("/home/fuji/catkin_ws/src/Struct_SLAM/src/robot/DES_ω.txt");
ofstream des_robot("/home/fuji/catkin_ws/src/Struct_SLAM/src/robot/DES_robot.txt");
ofstream des_time("/home/fuji/catkin_ws/src/Struct_SLAM/src/robot/DES_time.txt");
ofstream des_ALLtime("/home/fuji/catkin_ws/src/Struct_SLAM/src/robot/DES_ALLtime.txt");
ofstream V1_time("/home/fuji/catkin_ws/src/Struct_SLAM/src/robot/V1_time.txt");
ofstream M1_time("/home/fuji/catkin_ws/src/Struct_SLAM/src/robot/M1_time.txt");
ofstream V1_ALLtime("/home/fuji/catkin_ws/src/Struct_SLAM/src/robot/V1_ALLtime.txt");
ofstream M1_ALLtime("/home/fuji/catkin_ws/src/Struct_SLAM/src/robot/M1_ALLtime.txt");

geometry_msgs::Pose Des_Robot_pose;//ロボットtf(指令値)
geometry_msgs::Pose Act_Robot_pose;//ロボットtf(観測値)

nav_msgs::Path Des_path;//カメラ経路表示設定
nav_msgs::Path Act_path;//カメラ経路表示設定
geometry_msgs::PoseStamped Des_pose;//ロボット姿勢(指令値)
geometry_msgs::PoseStamped Act_pose;//ロボット姿勢(観測値)

//ロボットのオドメトリー(ロボットの観測値)
void odomcallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    ros::NodeHandle nh;
    robot_odometry=*msg;
    pub= nh.advertise<geometry_msgs::Twist>("/robot1/mobile_base/commands/velocity", 10);
 
    //サンプリング時間取得(C言語の方法)(こっちのほうが正確らしい)
      gettimeofday(&startTime, NULL);// 開始時刻取得
      if(kaisu!=0){
        time_t diffsec = difftime(startTime.tv_sec,endTime.tv_sec);    // 秒数の差分を計算
        suseconds_t diffsub = startTime.tv_usec - endTime.tv_usec;      // マイクロ秒部分の差分を計算
        realsec = diffsec+diffsub*1e-6;                          // 実時間を計算
        ALLrealsec=ALLrealsec+realsec;
        //printf("処理の時間=%f\n", realsec);
        //printf("処理時間合計=%f\n", ALLrealsec);
      }
      //3回転
      /*if(X_25==false&&TH_90==false&&Y_05==false){
          robot_velocity.linear.x  =  0.0;
          robot_velocity.angular.z  =  -0.5;//(0.5)
        if(Des_RobotTH<=-3.141592653*6){
          TH_90=true;
        }
      }
      else{
          robot_velocity.linear.x  =  0.0;
          robot_velocity.angular.z  = 0.0;//(0.5)
      }
      pub.publish(robot_velocity);    // 速度指令メッセージをパブリッシュ（送信）*/


      //2.5m直進後、45度回転し、50cm進む
      //xが2.0m以下の時実行→xに2.0m進める
      std::cout << "X_25=" <<X_25<<",TH_90=" <<TH_90<<",Y_05=" <<Y_05<< std::endl;
      if(X_25==false&&TH_90==false&&Y_05==false){
        gettimeofday(&startTimeV1, NULL);// 直進動作開始時刻
        if(kaisuV1!=0){
          time_t diffsecV1 = difftime(startTimeV1.tv_sec,endTimeV1.tv_sec);    // 秒数の差分を計算
          suseconds_t diffsubV1 = startTimeV1.tv_usec - endTimeV1.tv_usec;      // マイクロ秒部分の差分を計算
          realsecV1 = diffsecV1+diffsubV1*1e-6;                          // 実時間を計算
          ALLrealsecV1=ALLrealsecV1+realsecV1;
          V1_time<<realsecV1<<"\n";
          V1_ALLtime<<ALLrealsecV1<<"\n";
        }

        robot_velocity.linear.x  = 0.1;//(0.1)
        robot_velocity.angular.z = 0.0; // 回転速度の初期化}//xが1以上になったら終了
        if(Des_RobotX>=2.7){
          X_25=true;
        }
        kaisuV1++;
      }
      if(X_25==true&&TH_90==false&&Y_05==false){
        gettimeofday(&startTimeM1, NULL);//回転動作開始時刻
        if(kaisuM1!=0){
          time_t diffsecM1 = difftime(startTimeM1.tv_sec,endTimeM1.tv_sec);    // 秒数の差分を計算
          suseconds_t diffsubM1 = startTimeM1.tv_usec - endTimeM1.tv_usec;      // マイクロ秒部分の差分を計算
          realsecM1 = diffsecM1+diffsubM1*1e-6;                          // 実時間を計算
          ALLrealsecM1=ALLrealsecM1+realsecM1;//経過時刻
          M1_time<<realsecM1<<"\n";
          M1_ALLtime<<ALLrealsecM1<<"\n";
        }
        robot_velocity.linear.x  =  0.0;
        //robot_velocity.angular.z  =  0.5;//(0.5)
        robot_velocity.angular.z  =  -(0.5+(0.0176*ALLrealsecM1+0.11));//(0.5)
        //if(Des_RobotTH>=3.141592653/2){
        if(Des_RobotTH<=-3.141592653/2){
          TH_90=true;
          //for(int j=0;j<=10;j++){
            robot_velocity.linear.x  = 0.0; // 並進速度vの初期化
            robot_velocity.angular.z = 0.0; // 回転速度ωの初期化}//xが1以上になったら終了
            pub.publish(robot_velocity);    // 速度指令メッセージをパブリッシュ（送信）
            usleep(7*100000);//0.5秒ストップ(マイクロ秒)
          //}
        }
        kaisuM1++;
      }

      if(X_25==true&&TH_90==true&&Y_05==false){
        robot_velocity.linear.x  = 0.1;//(0.1)
        robot_velocity.angular.z = 0.0; // 回転速度の初期化}//xが1以上になったら終了
        //if(Des_RobotY>=2.5){
        if(Des_RobotY<=-2.0){
          Y_05=true;
        }
      }
      if(X_25==true&&TH_90==true&&Y_05==true){
          robot_velocity.linear.x  = 0.0; // 並進速度vの初期化
          robot_velocity.angular.z = 0.0; // 回転速度ωの初期化}//xが1以上になったら終了
      }

    pub.publish(robot_velocity);    // 速度指令メッセージをパブリッシュ（送信）

      if(X_25==false&&TH_90==false&&Y_05==false){
        robot_velocity.linear.x  = 0.1;//(0.1)
        robot_velocity.angular.z = 0.0; // 回転速度の初期化}//xが1以上になったら終了
      }
      if(X_25==true&&TH_90==false&&Y_05==false){
        robot_velocity.linear.x  =  0.0;
        //robot_velocity.angular.z  =  0.5;//(0.5)
        robot_velocity.angular.z  =  -0.5;//(0.5)
      }
      if(X_25==true&&TH_90==true&&Y_05==false){
        robot_velocity.linear.x  = 0.1;//(0.1)
        robot_velocity.angular.z = 0.0; // 回転速度の初期化}//xが1以上になったら終了
      }
      if(X_25==true&&TH_90==true&&Y_05==true){
          robot_velocity.linear.x  = 0.0; // 並進速度vの初期化
          robot_velocity.angular.z = 0.0; // 回転速度ωの初期化}//xが1以上になったら終了
      }

      //1m前進後、1m後進する(観測値で制御する)
      /*std::cout << "X_25=" <<X_25<<",TH_90=" <<TH_90<<",Y_05=" <<Y_05<< std::endl;
      if(X_25==false&&TH_90==false&&Y_05==false){
        robot_velocity.linear.x  = 0.1+0.01816;//(0.1)
        robot_velocity.angular.z = 0.0; // 回転速度の初期化}//xが1以上になったら終了
        if(Act_RobotX>=1.0){
          X_25=true;
          robot_velocity.linear.x  = 0.0; // 並進速度vの初期化
          robot_velocity.angular.z = 0.0; // 回転速度ωの初期化}//xが1以上になったら終了
          pub.publish(robot_velocity);    // 速度指令メッセージをパブリッシュ（送信）
          usleep(5 * 10000);//0.05秒ストップ(マイクロ秒)
        }
      }
      if(X_25==true&&TH_90==false&&Y_05==false){
        robot_velocity.linear.x  =  -(0.1+0.01816);
        robot_velocity.angular.z  =  0;//(0.5)
        //if(Des_RobotTH<=-3.141592653/2){
        if(Act_RobotX<=0){
          TH_90=true;
        }
      }
      if(X_25==true&&TH_90==true&&Y_05==false){
          robot_velocity.linear.x  = 0.0; // 並進速度vの初期化
          robot_velocity.angular.z = 0.0; // 回転速度ωの初期化}//xが1以上になったら終了
      }
      pub.publish(robot_velocity);    // 速度指令メッセージをパブリッシュ（送信）
      if(X_25==false&&TH_90==false&&Y_05==false){
        robot_velocity.linear.x  = 0.1+0.01816;//(0.1)
        robot_velocity.angular.z = 0.0; // 回転速度の初期化}//xが1以上になったら終了
      }
      if(X_25==true&&TH_90==false&&Y_05==false){
        robot_velocity.linear.x  =  -(0.1+0.01816);
        robot_velocity.angular.z  =  0;//(0.5)
      }
      if(X_25==true&&TH_90==true&&Y_05==false){
          robot_velocity.linear.x  = 0.0; // 並進速度vの初期化
          robot_velocity.angular.z = 0.0; // 回転速度ωの初期化}//xが1以上になったら終了
      }*/





    Act_RobotV=robot_odometry.twist.twist.linear.x+robot_odometry.twist.twist.linear.y;//速度ベクトルの合成
    std::cout << "Act_RobotV=" <<Act_RobotV<< std::endl;
    act_v<<Act_RobotV <<"\n";
    act_u<<robot_odometry.twist.twist.angular.z <<"\n";

    //ロボットの状態方程式
    //雑音の影響を考慮した実際の状態(カルマン推定前)
    if(robot_odometry.twist.twist.angular.z==0){
      Act_RobotX=Act_RobotX+(Act_RobotV*cos(Act_RobotTH)*realsec);
      Act_RobotY=Act_RobotY+(Act_RobotV*sin(Act_RobotTH)*realsec);
      Act_RobotTH=Act_RobotTH+(robot_odometry.twist.twist.angular.z*realsec);
    }
    else{
      Act_RobotX=Act_RobotX+((Act_RobotV/robot_odometry.twist.twist.angular.z)*(sin(Act_RobotTH+robot_odometry.twist.twist.angular.z*realsec)-sin(Act_RobotTH)));
      Act_RobotY=Act_RobotY+((Act_RobotV/robot_odometry.twist.twist.angular.z)*(-cos(Act_RobotTH+robot_odometry.twist.twist.angular.z*realsec)+cos(Act_RobotTH)));
      Act_RobotTH=Act_RobotTH+(robot_odometry.twist.twist.angular.z*realsec);
    }

    std::cout << "Act_RobotX=" <<Act_RobotX<< std::endl;
    std::cout << "Act_RobotY=" <<Act_RobotY<< std::endl;
    std::cout << "Act_RobotTH=" <<Act_RobotTH<< std::endl;
    act_robot<< "ALLrealsec=" <<ALLrealsec<< " ,realsec=" <<realsec<<" ,Act_RobotX=" <<Act_RobotX<<",Act_RobotY=" <<Act_RobotY<<",Act_RobotTH=" <<Act_RobotTH<<"\n";
    act_time<<realsec<<"\n";
    act_ALLtime<<ALLrealsec<<"\n";


    Des_RobotV=robot_velocity.linear.x+robot_velocity.linear.y;//速度ベクトルの合成
    std::cout << "Des_RobotV=" <<Des_RobotV<< std::endl;
    des_v<<Des_RobotV <<"\n";
    des_u<<robot_velocity.angular.z <<"\n";

    if(Act_RobotX>0||Act_RobotY>0||Act_RobotTH>0){//ラグの調整
      ////目標指令状態
      if(robot_velocity.angular.z==0){
        Des_RobotX=Des_RobotX+(Des_RobotV*cos(Des_RobotTH)*realsec);
        Des_RobotY=Des_RobotY+(Des_RobotV*sin(Des_RobotTH)*realsec);
        Des_RobotTH=Des_RobotTH+(robot_velocity.angular.z*realsec);
      }
      else{
        Des_RobotX=Des_RobotX+((Des_RobotV/robot_velocity.angular.z)*(sin(Des_RobotTH+robot_velocity.angular.z*realsec)-sin(Des_RobotTH)));
        Des_RobotY=Des_RobotY+((Des_RobotV/robot_velocity.angular.z)*(-cos(Des_RobotTH+robot_velocity.angular.z*realsec)+cos(Des_RobotTH)));
        Des_RobotTH=Des_RobotTH+(robot_velocity.angular.z*realsec);
      }
    }

    std::cout << "Des_RobotX=" <<Des_RobotX<< std::endl;
    std::cout << "Des_RobotY=" <<Des_RobotY<< std::endl;
    std::cout << "Des_RobotTH=" <<Des_RobotTH<< std::endl;
    des_robot<< "ALLrealsec=" <<ALLrealsec<<" ,realsec2=" <<realsec<<" ,Des_RobotX=" <<Des_RobotX<<",Des_RobotY=" <<Des_RobotY<<",Des_RobotTH=" <<Des_RobotTH<<"\n";
    des_time<<realsec<<"\n";
    des_ALLtime<<ALLrealsec<<"\n";

    robot_velocity.linear.x  = 0.0; // 並進速度の初期化
    robot_velocity.angular.z = 0.0; // 回転速度の初期化
  if(kaisu>1){
    //tf(map Des_Robot間のlink)-----------------------------------------------------------------------------------------
      //ここがカメラの姿勢部分
      std::string MaptoDes_Robot_frame = "MaptoDes_Robot_link";
      //微小区間回転行列
      double RollX=0,PitchY=Des_RobotTH,YawZ=0;

      //カメラ位置(理論値)
      Des_Robot_pose.position.x = Des_RobotX;//赤
      Des_Robot_pose.position.y = Des_RobotY;//緑
      Des_Robot_pose.position.z = 0;//青(tf:Z,画像:Y)
      Des_Robot_pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(YawZ,RollX,PitchY);

      static tf::TransformBroadcaster br_Des_Robot_pose;
      tf::Transform Des_Robot_transform;
      poseMsgToTF(Des_Robot_pose, Des_Robot_transform);
      br_Des_Robot_pose.sendTransform(tf::StampedTransform(Des_Robot_transform, ros::Time::now(), source_frame, MaptoDes_Robot_frame));
      std::cout <<"Des_Robot_pose.position.x="<<Des_Robot_pose.position.x<< std::endl;
      std::cout <<"Des_Robot_pose.position.y="<<Des_Robot_pose.position.y<< std::endl;
      std::cout <<"Des_Robot_pose.position.z="<<Des_Robot_pose.position.z<< std::endl;
      std::cout <<"Des_Robot_pose.orientation.x="<<Des_Robot_pose.orientation.x<< std::endl;
      std::cout <<"Des_Robot_pose.orientation.y="<<Des_Robot_pose.orientation.y<< std::endl;
      std::cout <<"Des_Robot_pose.orientation.z="<<Des_Robot_pose.orientation.z<< std::endl;
      std::cout <<"Des_Robot_pose.orientation.w="<<Des_Robot_pose.orientation.w<< std::endl;

      //tf(Des_Robot camera_link間のlink)-----------------------------------------------------------------------------------------
      geometry_msgs::Pose Des_Robot_Link_pose;
      std::string Des_Robot_camera_frame = "Des_Robot_camera_link";
      Des_Robot_Link_pose.position.x = 0;
      Des_Robot_Link_pose.position.y = 0;
      Des_Robot_Link_pose.position.z = 0.67;//ここに実際のカメラの高さを入れる
      Des_Robot_Link_pose.orientation.w = 1.0;
      static tf::TransformBroadcaster br_Des_Robot_Link_pose;
      tf::Transform Des_Robot_Link_transform;
      poseMsgToTF(Des_Robot_Link_pose, Des_Robot_Link_transform);
      br_Des_Robot_Link_pose.sendTransform(tf::StampedTransform(Des_Robot_Link_transform, ros::Time::now(), MaptoDes_Robot_frame, Des_Robot_camera_frame));
    
    //tf(map Des_Robot間のlink)-----------------------------------------------------------------------------------------
      //ここがカメラの姿勢部分
      std::string MaptoAct_Robot_frame = "MaptoAct_Robot_link";
      //微小区間回転行列
      RollX=0,PitchY=Act_RobotTH,YawZ=0;

      //カメラ位置(理論値)
      Act_Robot_pose.position.x = Act_RobotX;//赤
      Act_Robot_pose.position.y = Act_RobotY;//緑
      Act_Robot_pose.position.z = 0;//青(tf:Z,画像:Y)
      Act_Robot_pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(YawZ,RollX,PitchY);

      static tf::TransformBroadcaster br_Act_Robot_pose;
      tf::Transform Act_Robot_transform;
      poseMsgToTF(Act_Robot_pose, Act_Robot_transform);
      br_Act_Robot_pose.sendTransform(tf::StampedTransform(Act_Robot_transform, ros::Time::now(), source_frame, MaptoAct_Robot_frame));
      std::cout <<"Act_Robot_pose.position.x="<<Act_Robot_pose.position.x<< std::endl;
      std::cout <<"Act_Robot_pose.position.y="<<Act_Robot_pose.position.y<< std::endl;
      std::cout <<"Act_Robot_pose.position.z="<<Act_Robot_pose.position.z<< std::endl;
      std::cout <<"Act_Robot_pose.orientation.x="<<Act_Robot_pose.orientation.x<< std::endl;
      std::cout <<"Act_Robot_pose.orientation.y="<<Act_Robot_pose.orientation.y<< std::endl;
      std::cout <<"Act_Robot_pose.orientation.z="<<Act_Robot_pose.orientation.z<< std::endl;
      std::cout <<"Act_Robot_pose.orientation.w="<<Act_Robot_pose.orientation.w<< std::endl;

      //tf(Act_Robot camera_link間のlink)-----------------------------------------------------------------------------------------
      geometry_msgs::Pose Act_Robot_Link_pose;
      std::string Act_Robot_camera_frame = "Act_Robot_camera_link";

      //std::string Act_Robot_Link_Link_frame = "Act_Robot_Link_Link_link";
      Act_Robot_Link_pose.position.x = 0;
      Act_Robot_Link_pose.position.y = 0;
      Act_Robot_Link_pose.position.z = 0.67;//ここに実際のカメラの高さを入れる
      Act_Robot_Link_pose.orientation.w = 1.0;
      static tf::TransformBroadcaster br_Act_Robot_Link_pose;
      tf::Transform Act_Robot_Link_transform;
      poseMsgToTF(Act_Robot_Link_pose, Act_Robot_Link_transform);
      br_Act_Robot_Link_pose.sendTransform(tf::StampedTransform(Act_Robot_Link_transform, ros::Time::now(), MaptoAct_Robot_frame, Act_Robot_camera_frame));
    }
  
  else{
    Des_Robot_pose.position.x = 0;
    Des_Robot_pose.position.y = 0;
    Des_Robot_pose.position.z = 0;
    Des_Robot_pose.orientation.x=0;
    Des_Robot_pose.orientation.y=0;
    Des_Robot_pose.orientation.z=0;
    Des_Robot_pose.orientation.w=0;

    Act_Robot_pose.position.x = 0;
    Act_Robot_pose.position.y = 0;
    Act_Robot_pose.position.z = 0;
    Act_Robot_pose.orientation.x=0;
    Act_Robot_pose.orientation.y=0;
    Act_Robot_pose.orientation.z=0;
    Act_Robot_pose.orientation.w=0;
  }
  //経路描写-------------------------------------------------------------
    Des_pose.header.stamp = ros::Time::now();
    Des_pose.header.frame_id = source_frame;
    Des_pose.pose.position = Des_Robot_pose.position;
    Des_pose.pose.orientation = Des_Robot_pose.orientation;
    Des_path.header.stamp = ros::Time::now();
    Des_path.header.frame_id = source_frame;
    Des_path.poses.push_back(Des_pose);
    Des_pub_plan.publish(Des_path);

    Act_pose.header.stamp = ros::Time::now();
    Act_pose.header.frame_id = source_frame;
    Act_pose.pose.position = Act_Robot_pose.position;
    Act_pose.pose.orientation = Act_Robot_pose.orientation;
    Act_path.header.stamp = ros::Time::now();
    Act_path.header.frame_id = source_frame;
    Act_path.poses.push_back(Act_pose);
    Act_pub_plan.publish(Act_path);




    endTime=startTime;//動作終了時刻取得
    endTimeV1=startTimeV1;//動作終了時刻取得
    endTimeM1=startTimeM1;//動作終了時刻取得
    kaisu++;

    cv::waitKey(1);//ros::spinにジャンプする

}


//メイン関数
int main(int argc,char **argv){
  ros::init(argc,argv,"marker2");//rosを初期化
	ros::NodeHandle nhSub;//ノードハンドル

  if(kaisu!=0){ros_begin = ros::Time::now();}

	//subscriber関連
  //subscriberの作成。トピック/cmd_velを購読する。バッファ数は10。               
  ros::NodeHandle odomnh;
  ros::Subscriber odom_sub = odomnh.subscribe("/robot1/odom", 10, odomcallback);//ロボットのオドメトリー

  ros::NodeHandle nhPub;
  //Des_pub_plan = nhPub.advertise<nav_msgs::Path>("/get_multi_path",1000);
  Des_pub_plan = nhPub.advertise<nav_msgs::Path>("/Des_path",1000);
  Act_pub_plan = nhPub.advertise<nav_msgs::Path>("/Act_path",1000);

	ros::spin();//トピック更新待機
			
	return 0;
}