//線検出(FLD)→Depth取得不可能な線を削除
//まずナナメの線のみを取り出し、取り出した後のナナメの線が含まれていないグループを作る
//その後作成したグループからタテ、ヨコ線の検出を行う
//20210908 青線がばーって表示されるバグを直したが、kskの個数が０個になるとSegmentationFolutになるのが確認できた（たまに起こる）
//そのためkskの対策が必要

//rosのヘッダ
//#define _CRT_SECURE_NO_WARNINGS
#include <ros/ros.h>
#include <iostream>
#include <string>
#include <vector>
#include <cv_bridge/cv_bridge.h>//画像変換のヘッダ
#include <sensor_msgs/Image.h>//センサーデータ形式ヘッダ
#include <sensor_msgs/image_encodings.h>//エンコードのためのヘッダ
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <math.h>
#include <highgui.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/ximgproc/fast_line_detector.hpp>//FLD
#include <Eigen/Dense>
#include <opencv2/aruco/charuco.hpp>//マーカー検出
#include <time.h>//処理の時間を出力する
#include <sys/time.h>

#include <geometry_msgs/PoseStamped.h>//tf
#include <tf/transform_broadcaster.h>//tf
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <opencv2/aruco/charuco.hpp>//マーカー検出
#include <nav_msgs/Path.h>//経路情報を記録する

#include <geometry_msgs/Twist.h>//ロボットの指令値(速度)用ヘッダー
#include <nav_msgs/Odometry.h>
#include<fstream>


ros::Subscriber sub;//データをsubcribeする奴
ros::Subscriber odom_sub;//ロボットオドメトリー受信用
ros::Publisher  pub;//速度送信用
ros::Publisher Des_pub_plan;//カメラ経路送信用(指令値)
ros::Publisher Act_pub_plan;//カメラ経路送信用(観測値)
ros::Publisher Est_pub_plan;//カメラ経路送信用(推定値)
std::string source_frame = "map";//mapフレーム

using namespace std;
using namespace cv;
using Eigen::MatrixXd;

int kaisu=0;
// reset == TRUE のとき特徴点検出を行う
// 最初のフレームで必ず特徴点検bool reset = true;//初回プログラム用（特徴点検出&特徴点再検出用)出を行うように、初期値を TRUE にする
bool time0 = false;//カルマン推定データ初回回避用(カルマン動作した時TRUE)
bool X_25=false;
bool TH_90=false;
bool Y_05=false;

double VX,omegaZ,LX,THZ,LY;//ロボットの指令値パラメータ

//時間取得用
ros::Time ros_begin;//プログラム時間
ros::WallTime wall_begin = ros::WallTime::now();//プログラム開始時の時間
ros::WallDuration wall_prev;//一つ前の時間
ros::WallDuration wall_systemtime;//サンプリング時間
//ros::Time ros_begin = ros::Time::now();
struct timeval startTime, endTime;  // 構造体宣言
float realsec,ALLrealsec;//サンプリング時間（C++)

geometry_msgs::Twist robot_velocity;//指令速度
nav_msgs::Odometry robot_odometry;//ロボットのオドメトリー

//ロボット動作関連
double roll, pitch, yaw;//クオータニオン→オイラー角変換用
double Act_RobotX=0,Act_RobotY=0,Act_RobotTH=0;//ロボットの状態方程式(実際の状態)
double Des_RobotX=0,Des_RobotY=0,Des_RobotTH=0;//ロボットの状態方程式(理想状態)
double Est_RobotX=0,Est_RobotY=0,Est_RobotTH=0;//ロボットの状態方程式(推定状態)
double Act_RobotV,Des_RobotV;//ロボットの速度ベクトル(実測値,指令値)

//カルマンフィルタ関連
cv::Mat ACT_Robot,DES_Robot,EST_Robot;//状態方程式の行列化
cv::Mat At,Mt,Ft,Cov,Qt,K,Ht,I,hu;

geometry_msgs::Pose Des_Robot_pose;//ロボットtf(指令値)
geometry_msgs::Pose Act_Robot_pose;//ロボットtf(観測値)
geometry_msgs::Pose Est_Robot_pose;//ロボットtf(推定値)
nav_msgs::Path Des_path;//カメラ経路表示設定
nav_msgs::Path Act_path;//カメラ経路表示設定
nav_msgs::Path Est_path;//カメラ経路表示設定
geometry_msgs::PoseStamped Des_pose;//ロボット姿勢(指令値)
geometry_msgs::PoseStamped Act_pose;//ロボット姿勢(観測値)
geometry_msgs::PoseStamped Est_pose;//ロボット姿勢(推定値)

ofstream Des_V("/home/fuji/catkin_ws/src/Struct_SLAM/src/robot/date_odom/Des_V.txt");//指令値速度
ofstream Des_omega("/home/fuji/catkin_ws/src/Struct_SLAM/src/robot/date_odom/Des_omega.txt");//指令値角速度
ofstream Act_V("/home/fuji/catkin_ws/src/Struct_SLAM/src/robot/date_odom/Act_V.txt");//観測値速度
ofstream Act_omega("/home/fuji/catkin_ws/src/Struct_SLAM/src/robot/date_odom/Act_omega.txt");//観測値角速度
ofstream robot_sec("/home/fuji/catkin_ws/src/Struct_SLAM/src/robot/date_odom/robot_sec.txt");
ofstream Des_X("/home/fuji/catkin_ws/src/Struct_SLAM/src/robot/date_odom/Des_X.txt");
ofstream Des_Y("/home/fuji/catkin_ws/src/Struct_SLAM/src/robot/date_odom/Des_Y.txt");
ofstream Des_TH("/home/fuji/catkin_ws/src/Struct_SLAM/src/robot/date_odom/Des_TH.txt");
ofstream Act_X("/home/fuji/catkin_ws/src/Struct_SLAM/src/robot/date_odom/Act_X.txt");
ofstream Act_Y("/home/fuji/catkin_ws/src/Struct_SLAM/src/robot/date_odom/Act_Y.txt");
ofstream Act_TH("/home/fuji/catkin_ws/src/Struct_SLAM/src/robot/date_odom/Act_Th.txt");
ofstream Des2_V("/home/fuji/catkin_ws/src/Struct_SLAM/src/robot/date_odom/Des2_V.txt");//指令値速度
ofstream Des2_omega("/home/fuji/catkin_ws/src/Struct_SLAM/src/robot/date_odom/Des2_omega.txt");//指令値角速度

//コールバック関数
void callback(const nav_msgs::Odometry::ConstPtr& msg,const sensor_msgs::Image::ConstPtr& rgb_msg,const sensor_msgs::Image::ConstPtr& depth_msg)
{
  //サンプリング時間取得(C言語の方法)(こっちのほうが正確らしい)
  gettimeofday(&startTime, NULL);// 開始時刻取得
  if(time0 != false){
    time_t diffsec = difftime(startTime.tv_sec,endTime.tv_sec);    // 秒数の差分を計算
    suseconds_t diffsub = startTime.tv_usec - endTime.tv_usec;      // マイクロ秒部分の差分を計算
    realsec = diffsec+diffsub*1e-6;                          // 実時間を計算
    ALLrealsec=ALLrealsec+realsec;
    printf("処理の時間=%f\n", realsec);
    printf("処理時間合計=%f\n", ALLrealsec);
  }

	//変数宣言
	cv_bridge::CvImagePtr bridgeImage;//クラス::型//cv_brigeは画像変換するとこ
  cv_bridge::CvImagePtr bridgedepthImage;//クラス::型//cv_brigeは画像変換するとこ
  cv::Mat RGBimage;//opencvの画像
  cv::Mat depthimage;//opencvの画像
	cv::Mat image;//opencvの画像
	ROS_INFO("callback_functionが呼ばれたよ");
	
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

  cv::Mat img_src = bridgeImage->image.clone();//image変数に変換した画像データを代入
  cv::Mat img_depth = bridgedepthImage->image.clone();//image変数に変換した画像データを代入

//ロボット指令-------------------------------------------------------------------
    robot_odometry=*msg;
    //回転動作
    /*if(X_25==false&&TH_90==false&&Y_05==false){
      robot_velocity.linear.x  =  0;
      robot_velocity.angular.z  =  -THZ+(Des_RobotTH-Act_RobotTH)*2.2;//実行指令値
      Des2_V<<robot_velocity.linear.x<<"\n";
      Des2_omega<<robot_velocity.angular.z<<"\n";
      pub.publish(robot_velocity);    // 速度指令メッセージをパブリッシュ（送信）
      robot_velocity.linear.x  =  0.0;
      robot_velocity.angular.z  =  -THZ;
      //if(Act_RobotTH<=0.01){robot_velocity.angular.z  =  0;}//指令値の遅れを考慮(推定に使用する指令値)
      //else{robot_velocity.angular.z  =  THZ;} 
      std::cout << "Des_RobotTH=" <<Des_RobotTH<< std::endl;
      std::cout << "-3.141592653*2=" <<-3.141592653*2<< std::endl;

        
      if(Des_RobotTH<=-3.141592653*2){//廊下
        X_25=true;
        robot_velocity.linear.x  = 0.0;
        robot_velocity.angular.z = 0.0;
        pub.publish(robot_velocity);    // 速度指令メッセージをパブリッシュ（送信）
        usleep(7*100000);//7秒ストップ(マイクロ秒)
      }
    }
    else if(X_25==true&&TH_90==false&&Y_05==false){
      robot_velocity.linear.x  = 0.0; // 並進速度vの初期化
      robot_velocity.angular.z = 0.0; // 回転速度ωの初期化}//xが1以上になったら終了
      pub.publish(robot_velocity);    // 速度指令メッセージをパブリッシュ（送信）
      usleep(7*100000);//0.5秒ストップ(マイクロ秒)
    }*/
    /*//廊下動作1-----------------------------------------------------------------------------------------------------------------
      std::cout << "X_25=" <<X_25<<",TH_90=" <<TH_90<<",Y_05=" <<Y_05<< std::endl;
      //直進動作
      if(X_25==false&&TH_90==false&&Y_05==false){
        robot_velocity.linear.x  = VX+2.2*(Des_RobotX-Act_RobotX)+2.2*(Des_RobotY-Act_RobotY);//実行指令値
        robot_velocity.angular.z = (Des_RobotTH-Act_RobotTH)*2.2;

        pub.publish(robot_velocity);    // 速度指令メッセージをパブリッシュ（送信）

        if(Act_RobotV<=0.1){robot_velocity.linear.x=0;}//指令値の遅れを考慮(推定に使用する指令値)
        else{robot_velocity.linear.x  = VX;} 
        robot_velocity.angular.z = 0.0;
        if(Des_RobotX>=LX){//停止命令
          X_25=true;
          robot_velocity.linear.x  = 0.0; // 並進速度vの初期化
          robot_velocity.angular.z = 0.0; // 回転速度ωの初期化
          pub.publish(robot_velocity);    // 速度指令メッセージをパブリッシュ（送信）
          usleep(7*100000);//0.5秒ストップ(マイクロ秒)
        }
      }
      else if(X_25==true&&TH_90==false&&Y_05==false){
        robot_velocity.linear.x  = 0.0; // 並進速度vの初期化
        robot_velocity.angular.z = 0.0; // 回転速度ωの初期化}//xが1以上になったら終了
        pub.publish(robot_velocity);    // 速度指令メッセージをパブリッシュ（送信）
      }*/
   
     //廊下動作1(反時計回り)-----------------------------------------------------------------------------------------------------------------
      std::cout << "X_25=" <<X_25<<",TH_90=" <<TH_90<<",Y_05=" <<Y_05<< std::endl;
      //直進動作
      if(X_25==false&&TH_90==false&&Y_05==false){
        robot_velocity.linear.x  = VX+2.2*(Des_RobotX-Act_RobotX)+2.2*(Des_RobotY-Act_RobotY);//実行指令値
        robot_velocity.angular.z=(Des_RobotTH-Act_RobotTH)*2.2;

        pub.publish(robot_velocity);    // 速度指令メッセージをパブリッシュ（送信）

        if(Act_RobotV<=0.1){robot_velocity.linear.x=0;}//指令値の遅れを考慮(推定に使用する指令値)
        else{robot_velocity.linear.x  = VX;} 
        robot_velocity.angular.z = 0.0;
        if(Des_RobotX>=LX){//停止命令
          X_25=true;
          robot_velocity.linear.x  = 0.0; // 並進速度vの初期化
          robot_velocity.angular.z = 0.0; // 回転速度ωの初期化
          pub.publish(robot_velocity);    // 速度指令メッセージをパブリッシュ（送信）
          usleep(7*100000);//0.5秒ストップ(マイクロ秒)
        }
      }
      //回転動作(反時計回り)
      else if(X_25==true&&TH_90==false&&Y_05==false){
        robot_velocity.linear.x  =  0;
        robot_velocity.angular.z  =  THZ+(Des_RobotTH-Act_RobotTH)*2.2;//実行指令値

        pub.publish(robot_velocity);    // 速度指令メッセージをパブリッシュ（送信）
        robot_velocity.linear.x  =  0.0;
        robot_velocity.angular.z  =  THZ; 

        if(Act_RobotTH<=0.0001){robot_velocity.angular.z  =  0;}//指令値の遅れを考慮(推定に使用する指令値)
        else{robot_velocity.angular.z  =  THZ;} 
        
        if(Des_RobotTH>=3.141592653/omegaZ){//廊下
          TH_90=true;
          robot_velocity.linear.x  = 0.0; // 並進速度vの初期化
          robot_velocity.angular.z = 0.0; // 回転速度ωの初期化}//xが1以上になったら終了
          pub.publish(robot_velocity);    // 速度指令メッセージをパブリッシュ（送信）
          usleep(7*100000);//0.5秒ストップ(マイクロ秒)
        }
      }
      else if(X_25==true&&TH_90==true&&Y_05==false){
        robot_velocity.linear.x  = VX+2.2*(Des_RobotX-Act_RobotX)+2.2*(Des_RobotY-Act_RobotY);
        robot_velocity.angular.z=(Des_RobotTH-Act_RobotTH)*2.2;

        pub.publish(robot_velocity);    // 速度指令メッセージをパブリッシュ（送信）
        if(Act_RobotV<=0.1){robot_velocity.linear.x=0;}
        else{robot_velocity.linear.x  = VX;} // 並進速度vの初期化
        robot_velocity.angular.z = 0.0; // 回転速度ωの初期化}//xが1以上になったら終了
        if(Des_RobotY>=LY){
          Y_05=true;
          robot_velocity.linear.x  = 0.0; // 並進速度vの初期化
          robot_velocity.angular.z = 0.0; // 回転速度ωの初期化}//xが1以上になったら終了
          pub.publish(robot_velocity);    // 速度指令メッセージをパブリッシュ（送信）
          usleep(7*100000);//0.5秒ストップ(マイクロ秒)
        }
      }
      else if(X_25==true&&TH_90==true&&Y_05==true){
        robot_velocity.linear.x  = 0.0; // 並進速度vの初期化
        robot_velocity.angular.z = 0.0; // 回転速度ωの初期化}//xが1以上になったら終了
        pub.publish(robot_velocity);    // 速度指令メッセージをパブリッシュ（送信）
      }

      

      //研究室動作-------------------------------------------------------------------------------------
      /*std::cout << "X_25=" <<X_25<<",TH_90=" <<TH_90<<",Y_05=" <<Y_05<< std::endl;
      if(X_25==false&&TH_90==false&&Y_05==false){
        gettimeofday(&startTimeV1, NULL);// 直進動作開始時刻
        if(kaisuV1!=0){
          time_t diffsecV1 = difftime(startTimeV1.tv_sec,endTimeV1.tv_sec);    // 秒数の差分を計算
          suseconds_t diffsubV1 = startTimeV1.tv_usec - endTimeV1.tv_usec;      // マイクロ秒部分の差分を計算
          realsecV1 = diffsecV1+diffsubV1*1e-6;                          // 実時間を計算
          ALLrealsecV1=ALLrealsecV1+realsecV1;
        }
        robot_velocity.linear.x  = VX;//(0.1)
        robot_velocity.angular.z = 0.0; // 回転速度の初期化}//xが1以上になったら終了
        if(Des_RobotX>=LX){X_25=true;}
        kaisuV1++;
      }
      if(X_25==true&&TH_90==false&&Y_05==false){
        gettimeofday(&startTimeM1, NULL);//回転動作開始時刻
        if(kaisuM1!=0){
          time_t diffsecM1 = difftime(startTimeM1.tv_sec,endTimeM1.tv_sec);    // 秒数の差分を計算
          suseconds_t diffsubM1 = startTimeM1.tv_usec - endTimeM1.tv_usec;      // マイクロ秒部分の差分を計算
          realsecM1 = diffsecM1+diffsubM1*1e-6;                          // 実時間を計算
          ALLrealsecM1=ALLrealsecM1+realsecM1;//経過時刻
        }
        robot_velocity.linear.x  =  0.0;
        robot_velocity.angular.z  =  -((THZ-0.1)+(0.0176*ALLrealsecM1+0.11));//(0.5)研究室
        if(Des_RobotTH<=-3.141592653/omegaZ){//研究室
          TH_90=true;
          robot_velocity.linear.x  = 0.0; // 並進速度vの初期化
          robot_velocity.angular.z = 0.0; // 回転速度ωの初期化}//xが1以上になったら終了
          pub.publish(robot_velocity);    // 速度指令メッセージをパブリッシュ（送信）
          usleep(7*100000);//0.5秒ストップ(マイクロ秒)
        }
        kaisuM1++;
      }
      if(X_25==true&&TH_90==true&&Y_05==false){
        robot_velocity.linear.x  = VX;//(0.1)
        robot_velocity.angular.z = 0.0; // 回転速度の初期化}//xが1以上になったら終了
        if(Des_RobotY<=-2.0){Y_05=true;}//研究室
      }
      if(X_25==true&&TH_90==true&&Y_05==true){
          robot_velocity.linear.x  = 0.0; // 並進速度vの初期化
          robot_velocity.angular.z = 0.0; // 回転速度ωの初期化}//xが1以上になったら終了
      }
    pub.publish(robot_velocity);    // 速度指令メッセージをパブリッシュ（送信）
    if(X_25==false&&TH_90==false&&Y_05==false){
      robot_velocity.linear.x  = VX;//(0.1)
      robot_velocity.angular.z = 0.0; // 回転速度の初期化}//xが1以上になったら終了
    }
    if(X_25==true&&TH_90==false&&Y_05==false){
      robot_velocity.linear.x  =  0.0;
      robot_velocity.angular.z  =  -THZ;//(0.5)研究室
    }
    if(X_25==true&&TH_90==true&&Y_05==false){
      robot_velocity.linear.x  = VX;//(0.1)
      robot_velocity.angular.z = 0.0; // 回転速度の初期化}//xが1以上になったら終了
    }
    if(X_25==true&&TH_90==true&&Y_05==true){
      robot_velocity.linear.x  = 0.0; // 並進速度vの初期化
      robot_velocity.angular.z = 0.0; // 回転速度ωの初期化}//xが1以上になったら終了
    }*/
    robot_sec<<ALLrealsec<<"\n";
    Act_RobotV=robot_odometry.twist.twist.linear.x+robot_odometry.twist.twist.linear.y;//速度ベクトルの合成
    std::cout << "Act_RobotV=" <<Act_RobotV<< std::endl;
    Act_V<<Act_RobotV<<"\n";
    Act_omega<<robot_odometry.twist.twist.angular.z<<"\n";

    Des_RobotV=robot_velocity.linear.x+robot_velocity.linear.y;//速度ベクトルの合成
    std::cout << "Des_RobotV=" <<Des_RobotV<< std::endl;
    Des_V<<Des_RobotV<<"\n";
    Des_omega<<robot_velocity.angular.z<<"\n";



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

    ACT_Robot= (cv::Mat_<double>(3,1) <<
      Act_RobotX,
      Act_RobotY,
      Act_RobotTH);
      
    Act_X<<Act_RobotX<<"\n";
    Act_Y<<Act_RobotY<<"\n";
    Act_TH<<Act_RobotTH<<"\n";

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

    DES_Robot= (cv::Mat_<double>(3,1) <<
      Des_RobotX,
      Des_RobotY,
      Des_RobotTH);

    Des_X<<Des_RobotX<<"\n";
    Des_Y<<Des_RobotY<<"\n";
    Des_TH<<Des_RobotTH<<"\n";

  //Rviz関連
  if(kaisu>1){
    //指令値状態tf(map Des_Robot間のlink)-----------------------------------------------------------------------------------------
    //ここがカメラの姿勢部分
    std::string MaptoDes_Robot_frame = "MaptoDes_Robot_link";
    double RollX=0,PitchY=Des_RobotTH,YawZ=0;//微小区間回転行列

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
    
    //観測状態tf(map Des_Robot間のlink)-----------------------------------------------------------------------------------------
    //ここがカメラの姿勢部分
    std::string MaptoAct_Robot_frame = "MaptoAct_Robot_link";
    RollX=0,PitchY=Act_RobotTH,YawZ=0;//微小区間回転行列

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
    Des_Robot_pose.position.x = 0, Des_Robot_pose.position.y = 0, Des_Robot_pose.position.z = 0;
    Act_Robot_pose.position.x = 0, Act_Robot_pose.position.y = 0, Act_Robot_pose.position.z = 0;
    Des_Robot_pose.orientation.x=0, Des_Robot_pose.orientation.y=0, Des_Robot_pose.orientation.z=0, Des_Robot_pose.orientation.w=0;
    Act_Robot_pose.orientation.x=0, Act_Robot_pose.orientation.y=0, Act_Robot_pose.orientation.z=0, Act_Robot_pose.orientation.w=0;
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

  robot_velocity.linear.x  = 0.0; // 並進速度の初期化
  robot_velocity.angular.z = 0.0; // 回転速度の初期化
  kaisu++;

  time0=true;//一回目スキップ
  endTime=startTime;//動作終了時刻取得
	cv::waitKey(1);//ros::spinにジャンプする
}

//メイン関数
int main(int argc,char **argv){
	ros::init(argc,argv,"FLD_clustering_3");//rosを初期化
	ros::NodeHandle nhSub;//ノードハンドル
  if(kaisu!=0){ros_begin = ros::Time::now();}

  //Realsensesの時(roslaunch realsense2_camera rs_camera.launch align_depth:=true)(Depth修正版なのでこっちを使うこと)
	message_filters::Subscriber<nav_msgs::Odometry> odom_sub(nhSub, "/robot1/odom", 1);//センサーメッセージを使うときは対応したヘッダーが必要
	message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nhSub, "/camera/color/image_raw", 1);//センサーメッセージを使うときは対応したヘッダーが必要
	message_filters::Subscriber<sensor_msgs::Image> depth_sub(nhSub, "/camera/aligned_depth_to_color/image_raw", 1);

  typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry,sensor_msgs::Image,sensor_msgs::Image> MySyncPolicy;	
	message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10),odom_sub,rgb_sub, depth_sub);
	sync.registerCallback(boost::bind(&callback,_1, _2, _3));

  ros::NodeHandle nhPub;
  Des_pub_plan = nhPub.advertise<nav_msgs::Path>("/Des_path",1000);
  Act_pub_plan = nhPub.advertise<nav_msgs::Path>("/Act_path",1000);
  Est_pub_plan = nhPub.advertise<nav_msgs::Path>("/Est_path",1000);

  ros::NodeHandle nh;//パブリッシュ用
  pub= nh.advertise<geometry_msgs::Twist>("/robot1/mobile_base/commands/velocity", 10);

  //LX=2.0,VX=0.25,omegaZ=2,THZ=0.25,LY=5.0;
  LX=2.5,VX=0.25,omegaZ=2.05,THZ=0.20,LY=5.0;

	ros::spin();//トピック更新待機
	return 0;
}