//特徴点検出のテストプログラム
#define _CRT_SECURE_NO_WARNINGS
#include <ros/ros.h>
#include <iostream>
#include <string>
#include <vector>
#include <math.h>
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
#include <opencv2/aruco/charuco.hpp>//マーカー検出
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
ros::Publisher Est_pub_plan;//カメラ経路送信用(推定値)

bool time0 = false;//カルマン推定データ初回回避用(カルマン動作した時TRUE)

using namespace std;
using namespace cv;


cv::Mat img_dst;//画像定義
vector<cv::Point2f> points_prev, points_curr;//特徴点定義
vector<cv::Point3f> camera_point_p,camera_point_c;//特徴点定義
sensor_msgs::CameraInfo camera_info;//CameraInfo受け取り用
geometry_msgs::Twist robot_velocity;//指令速度
nav_msgs::Odometry robot_odometry;//ロボットのオドメトリー

int kaisu=0;

ros::Time ros_begin;//プログラム時間
ros::WallTime wall_begin = ros::WallTime::now();//プログラム開始時の時間
ros::WallDuration wall_prev;//一つ前の時間
ros::WallDuration wall_systemtime;//サンプリング時間
//ros::Time ros_begin = ros::Time::now();
struct timeval startTime, endTime;  // 構造体宣言
struct timeval startTime2, endTime2;  // 構造体宣言
float realsec;//サンプリング時間（C++)
float ALLrealsec;//サンプリング時間（C++)


double robot_orientationX,robot_orientationY,robot_orientationZ;//ロボットの姿勢観測値(エンコーダー観測値)(オイラー角)

double roll, pitch, yaw;//クオータニオン→オイラー角変換用
double Act_RobotX=0,Act_RobotY=0,Act_RobotTH=0;//ロボットの状態方程式(実際の状態)
double Des_RobotX=0,Des_RobotY=0,Des_RobotTH=0;//ロボットの状態方程式(理想状態)
double Est_RobotX=0,Est_RobotY=0,Est_RobotTH=0;//ロボットの状態方程式(推定状態)
double Act_RobotV,Des_RobotV;//ロボットの速度ベクトル(実測値,指令値)

int ALLMarker=40;//全マーカー個数
float MC_point_prve[50][4];//一つ前のカメラ座標
float pixel[50][2],depth[100],MC_point[50][4],x,y,r2,f,ux,uy;//画像→カメラ座標変換
//double CameraLM[50][2];//カメラから見たマーカーまでの距離[0]と角度[1]

cv::Mat ACT_Robot,DES_Robot,EST_Robot;//状態方程式の行列化
cv::Mat At,Mt,Ft,Cov,Qt,K,Ht,I,hu;

double LM[1][2],CameraLM[50][2];//ランドマーク世界座標
double siguma;//センサーの共分散(仮)
cv::Mat zu;//センサーの観測値(仮)


ofstream act_v("/home/fuji/catkin_ws/src/Struct_SLAM/src/robot/ACT_V.txt");
ofstream act_u("/home/fuji/catkin_ws/src/Struct_SLAM/src/robot/ACT_ω.txt");
ofstream act_robot("/home/fuji/catkin_ws/src/Struct_SLAM/src/robot/ACT_robot.txt");
ofstream des_v("/home/fuji/catkin_ws/src/Struct_SLAM/src/robot/DES_V.txt");
ofstream des_u("/home/fuji/catkin_ws/src/Struct_SLAM/src/robot/DES_ω.txt");
ofstream des_robot("/home/fuji/catkin_ws/src/Struct_SLAM/src/robot/DES_robot.txt");
ofstream camera_robot("/home/fuji/catkin_ws/src/Struct_SLAM/src/robot/sensor_date.txt");

geometry_msgs::Pose Des_Robot_pose;//ロボットtf(指令値)
geometry_msgs::Pose Act_Robot_pose;//ロボットtf(観測値)
geometry_msgs::Pose Est_Robot_pose;//ロボットtf(推定値)

nav_msgs::Path Des_path;//カメラ経路表示設定
nav_msgs::Path Act_path;//カメラ経路表示設定
nav_msgs::Path Est_path;//カメラ経路表示設定
geometry_msgs::PoseStamped Des_pose;//ロボット姿勢(指令値)
geometry_msgs::PoseStamped Act_pose;//ロボット姿勢(観測値)
geometry_msgs::PoseStamped Est_pose;//ロボット姿勢(推定値)


//コールバック関数
void callback(const nav_msgs::Odometry::ConstPtr& msg,const sensor_msgs::Image::ConstPtr& rgb_msg,const sensor_msgs::Image::ConstPtr& depth_msg)
{
  ROS_INFO("callback_functionが呼ばれたよ");

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
   
  	image = bridgeImage->image.clone();//image変数に変換した画像データを代入
  	depthimage = bridgedepthImage->image.clone();//image変数に変換した画像データを代入

	  img_dst = image.clone();
    //img_dst = cv::Scalar(255,255,255);
    cv::Mat_<float> intrinsic_K= cv::Mat_<float>(3, 3);

    //マーカー検出+外部パラメータ推定-------------------------------------------------------------------------------------------  
  //カメラ内部パラメータ読み込み
  cv::Mat cameraMatrix;
  cv::FileStorage fs;
  fs.open("/home/fuji/catkin_ws/src/Struct_SLAM/src/marker/realsense_para.xml", cv::FileStorage::READ);
  fs["intrinsic"]>>cameraMatrix;
  //std::cout << "内部パラメータcameraMatrix=\n" << cameraMatrix << std::endl;
  intrinsic_K=cameraMatrix;

  //カメラの歪みパラメータ読み込み
  cv::Mat distCoeffs;
  cv::FileStorage fd;
  fd.open("/home/fuji/catkin_ws/src/Struct_SLAM/src/marker/realsense_para.xml", cv::FileStorage::READ);
  fd["distortion"]>>distCoeffs;
  //std::cout << "ねじれパラメータdistCoeffs=\n" << distCoeffs << std::endl;

  //マーカ辞書作成 6x6マスのマーカを250種類生成
  cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);

  //charucoボード生成 10x7マスのチェスボード、グリッドのサイズ0.04f、グリッド内マーカのサイズ0.02f
  cv::Ptr<cv::aruco::CharucoBoard> board = cv::aruco::CharucoBoard::create(10, 7, 0.04f, 0.02f, dictionary);

  //マーカー検出時メソッドを指定
  cv::Ptr<cv::aruco::DetectorParameters> params = cv::aruco::DetectorParameters::create();
  params->cornerRefinementMethod = cv::aruco::CORNER_REFINE_NONE;

  //マーカー検出
  std::vector<int> markerIds;
  std::vector<std::vector<cv::Point2f> > markerCorners;
  cv::aruco::detectMarkers(image, board->dictionary, markerCorners, markerIds, params);

  std::vector<cv::Vec3d> rvecs,tvecs;//マーカーの姿勢(回転ベクトル、並進ベクトル)
  cv::Mat_<double> rvecs2[50],jacobian[50];
  int MarkerC[50][4][2];//マーカーのコーナー座標を記録する([マーカーID][コーナー番号][XorY])
  float MCx[50],MCy[50];//マーカーの中心座標
  float depth0,depthFT;//Depth修正用
  
  //MCpoint[i][3],MC_point_prve[][][3]はデータ有無の要素（[][][3]=1ならデータ有り,0ならデータ無し)
  if(kaisu==0){//初回のみ全部初期化
    for(int i=0;i<ALLMarker;i++){
      for(int j=0;j<4;j++){
        MC_point[i][3]=0;//全マーカーのデータ確認用要素初期化
        MC_point_prve[i][3]=0;
        //xEst_prev_clP[i]=0;
      }
    }
  }
  //毎回最新pointのみ初期化
  for(int i=0;i<ALLMarker;i++){
    for(int j=0;j<4;j++){
      MC_point[i][3]=0;//全マーカーのデータ確認用要素初期化
    }
  }

  //マーカー観測可能
  if (markerIds.size() > 0) {
    cv::aruco::drawDetectedMarkers(img_dst, markerCorners, markerIds);//マーカー位置を描画
    cv::aruco::estimatePoseSingleMarkers(markerCorners, 0.05, cameraMatrix, distCoeffs, rvecs, tvecs);//マーカーの姿勢推定

    for(int i=0;i<markerIds.size();i++){
      std::cout <<"マーカーの個数:markerIds.size()="<<markerIds.size() << std::endl;//マーカー個数
      //std::cout <<"markerIds("<<i<<")="<< markerIds.at(i) << std::endl;//マーカーID
      
      MarkerC[markerIds.at(i)][0][0]=markerCorners[i][0].x, MarkerC[markerIds.at(i)][0][1]=markerCorners[i][0].y;//コーナーの画像座標ID対応化
      MarkerC[markerIds.at(i)][1][0]=markerCorners[i][1].x, MarkerC[markerIds.at(i)][1][1]=markerCorners[i][1].y;
      MarkerC[markerIds.at(i)][2][0]=markerCorners[i][2].x, MarkerC[markerIds.at(i)][2][1]=markerCorners[i][2].y;
      MarkerC[markerIds.at(i)][3][0]=markerCorners[i][3].x, MarkerC[markerIds.at(i)][3][1]=markerCorners[i][3].y;
      MCx[markerIds.at(i)]=(MarkerC[markerIds.at(i)][0][0]+MarkerC[markerIds.at(i)][1][0])/2;//マーカー中心座標(x座標)
      MCy[markerIds.at(i)]=(MarkerC[markerIds.at(i)][0][1]+MarkerC[markerIds.at(i)][2][1])/2;//マーカー中心座標(y座標)
		  cv::circle(img_dst, cv::Point(MCx[markerIds.at(i)],MCy[markerIds.at(i)]), 3, Scalar(0,255,0),  -1, cv::LINE_AA);//緑点
      cv::aruco::drawAxis(img_dst,cameraMatrix,distCoeffs,rvecs[i], tvecs[i], 0.1);//マーカーの姿勢描写
      cv::Rodrigues(rvecs[i],rvecs2[i],jacobian[i]);//回転ベクトルから回転行列への変換

      //画像→カメラ座標変換(マーカーの中心座標を使用)----------------------------------------------------------------------
      pixel[markerIds.at(i)][0]=MCx[markerIds.at(i)];
      pixel[markerIds.at(i)][1]=MCy[markerIds.at(i)];
      std::cout <<"MCx["<<markerIds.at(i)<<"]="<<MCx[markerIds.at(i)]<<",MCy["<<markerIds.at(i)<<"]="<<MCy[markerIds.at(i)]<< std::endl;
      depth[markerIds.at(i)] = depthimage.at<float>(cv::Point(pixel[markerIds.at(i)][0],pixel[markerIds.at(i)][1]));
      //Depth値を修正
      depth0=depth[markerIds.at(i)]*0.001;
      depthFT=39.215*depth0*depth0-27.793*depth0-7.7718;
      depth[markerIds.at(i)]=depth[markerIds.at(i)]-depthFT;

      //Depthが取得できないコーナーを削除する+Depthの外れ値を除く
      if(depth[markerIds.at(i)]>0&&depth[markerIds.at(i)]<10000){
        x = (pixel[markerIds.at(i)][0] - 324.473) / 615.337;//ここで正規化座標もしてる
        y = (pixel[markerIds.at(i)][1] - 241.696) / 615.458;
        //camera_info.K[0]=615.337,camera_info.K[2]=324.473,camera_info.K[4]=615.458,camera_info.K[5]=241.696//内部パラメータ

        MC_point[markerIds.at(i)][0] = depth[markerIds.at(i)] * x/1000;//メートル表示変換
        MC_point[markerIds.at(i)][1] = depth[markerIds.at(i)] * y/1000;
        MC_point[markerIds.at(i)][2] = depth[markerIds.at(i)]/1000;
        MC_point[markerIds.at(i)][3] = 1;//データ取得可能なら1
        std::cout << "マーカーのカメラ座標:MC_point["<<markerIds.at(i)<<"]={"<< MC_point[markerIds.at(i)][0] <<","<<MC_point[markerIds.at(i)][1]<<","<<MC_point[markerIds.at(i)][2]<<"}"<< std::endl;
        //std::cout <<"X="<< MC_point[markerIds.at(i)][0]/MC_point[markerIds.at(i)][2]<< std::endl;
        //std::cout <<"Y="<< MC_point[markerIds.at(i)][1]/MC_point[markerIds.at(i)][2]<< std::endl;
      }
      //マーカーまでの距離と角度を求める(観測値)
      CameraLM[markerIds.at(i)][0]=sqrt((MC_point[markerIds.at(i)][0]*MC_point[markerIds.at(i)][0])+(MC_point[markerIds.at(i)][2]*MC_point[markerIds.at(i)][2]));
      CameraLM[markerIds.at(i)][1]=atan2(MC_point[markerIds.at(i)][0],MC_point[markerIds.at(i)][2]);
      std::cout <<"CameraLM["<<markerIds.at(i)<<"][0]="<<CameraLM[markerIds.at(i)][0]<< std::endl;//マーカーまでの距離
      std::cout <<"CameraLM["<<markerIds.at(i)<<"][1]="<<CameraLM[markerIds.at(i)][1]<< std::endl;//マーカーまでの角度
      camera_robot<< "ALLrealsec=" <<ALLrealsec<< " ,realsec=" <<realsec<<" ,CameraLM["<<markerIds.at(i)<<"][0]=" <<CameraLM[markerIds.at(i)][0]<<",CameraLM["<<markerIds.at(i)<<"][1]=" <<CameraLM[markerIds.at(i)][1]<<"\n";

      zu = (cv::Mat_<double>(2,1) <<
        CameraLM[markerIds.at(i)][0],
        CameraLM[markerIds.at(i)][1]);

      }
    }
    //ロボット指令-------------------------------------------------------------------
    ros::NodeHandle nh;
    robot_odometry=*msg;
    pub= nh.advertise<geometry_msgs::Twist>("/robot1/mobile_base/commands/velocity", 10);
 

    if(Act_RobotX>=0.50){//xが1以上になったら終了    
      robot_velocity.linear.x  = 0.0; // 並進速度の初期化
      robot_velocity.angular.z = 0.0; // 回転速度の初期化}
    }
    else{
      robot_velocity.linear.x  =  0.1;//(0.1)
      robot_velocity.angular.z  =  0;
    }
    pub.publish(robot_velocity);    // 速度指令メッセージをパブリッシュ（送信）

    Act_RobotV=robot_odometry.twist.twist.linear.x+robot_odometry.twist.twist.linear.y;//速度ベクトルの合成
    std::cout << "Act_RobotV=" <<Act_RobotV<< std::endl;
    act_v<<Act_RobotV <<"\n";
    act_u<<robot_odometry.twist.twist.angular.z <<"\n";

    Des_RobotV=robot_velocity.linear.x+robot_velocity.linear.y;//速度ベクトルの合成
    std::cout << "Des_RobotV=" <<Des_RobotV<< std::endl;
    des_v<<Des_RobotV <<"\n";
    des_u<<robot_velocity.angular.z <<"\n";

    //この下にカルマンフィルタを実装する(できたら別関数にしたい)
    //カルマンフィルタ初期設定---------------------------------------------------------------------------------------------------------
    if(kaisu==0){
      LM[0][0]=2.00;//マーカーの世界座標x
      LM[0][1]=0.00;//マーカーの世界座標y
      DES_Robot = cv::Mat_<double>::zeros(3, 1);//目標指令状態
      ACT_Robot = cv::Mat_<double>::zeros(3, 1);//雑音の影響を考慮した実際の状態
      EST_Robot = cv::Mat_<double>::zeros(3, 1);//推定状態
      //z[0]=0;//出力方程式
      //z[1]=0;
      //z[2]=0;//出力方程式
      //z[3]=0;
      Cov = (cv::Mat_<double>(3,3) << //共分散Q
        1e-10, 0,      0,
        0,     1e-10,  0,
        0,     0,      1e-10);
      I = cv::Mat_<double>::eye(3, 3);//単位行列
      //z = cv::Mat_<double>::zeros(2, 1);//センサーの観測値(仮)
      std::cout <<"初期設定"<< std::endl;
    }

  if(time0 != false){
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

    ACT_Robot= (cv::Mat_<double>(3,1) <<
      Act_RobotX,
      Act_RobotY,
      Act_RobotTH);

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
    des_robot<< "ALLrealsec=" <<ALLrealsec<<" ,realsec=" <<realsec<<" ,Des_RobotX=" <<Des_RobotX<<",Des_RobotY=" <<Des_RobotY<<",Des_RobotTH=" <<Des_RobotTH<<"\n";

    DES_Robot= (cv::Mat_<double>(3,1) <<
      Des_RobotX,
      Des_RobotY,
      Des_RobotTH);

  //予測ステップ-----------------------------------------------------------------------------------------------------
  std::cout <<"予測ステップ"<< std::endl;
  //Mt誤差共分散(指令値に発生するノイズ)
  //指令値に事前分散をかけたもの
  Mt= (cv::Mat_<double>(2,2) << 
      ((2.97694E-5*abs(Des_RobotV))+(0.000126947*abs(robot_velocity.angular.z)))/realsec, 0,
      0, ((3.30119E-6*abs(Des_RobotV))+(0.00110641*abs(robot_velocity.angular.z)))/realsec);
  double At00,At01,At10,At11,At20,At21;

  //Atは速度、角速度は指令値、角度θ(状態)は一つ前の実測状態を使用
  if(robot_velocity.angular.z==0){
    At00=cos(Act_RobotTH)*realsec;
    At01=0;
    At10=sin(Act_RobotTH)*realsec;
    At11=0;
    At20=0;
    At21=realsec;
  }
  else{
    At00=(1/robot_velocity.angular.z)*(sin(Act_RobotTH+robot_velocity.angular.z*realsec)-sin(Act_RobotTH));
    At01=-(Des_RobotV/(robot_velocity.angular.z*robot_velocity.angular.z))*(sin(Act_RobotTH+robot_velocity.angular.z*realsec)-sin(Act_RobotTH))
        +((Des_RobotV/robot_velocity.angular.z)*realsec*cos(Act_RobotTH+robot_velocity.angular.z*realsec));
    At10=(1/robot_velocity.angular.z)*(-cos(Act_RobotTH+robot_velocity.angular.z*realsec)+cos(Act_RobotTH));
    At11=-(Des_RobotV/(robot_velocity.angular.z*robot_velocity.angular.z))*(-cos(Act_RobotTH+robot_velocity.angular.z*realsec)+cos(Act_RobotTH))
        +((Des_RobotV/robot_velocity.angular.z)*realsec*sin(Act_RobotTH+robot_velocity.angular.z*realsec));
    At20=0;
    At21=realsec;
  }

  //実際の出力と指令値との誤差を入れるためのヤコビアン
  At= (cv::Mat_<double>(3,2) <<
      At00, At01,
      At10, At11,
      At20, At21);

  //Ftは速度、角速度は指令値、角度θは一つ前の推定状態を使用
  if(robot_velocity.angular.z==0){
    Ft= (cv::Mat_<double>(3,3) << 
      1, 0, -Act_RobotV*sin(Est_RobotTH)*realsec,
      0, 1, Act_RobotV*cos(Est_RobotTH)*realsec,
      0, 0, 1);
    }
  else{
    Ft= (cv::Mat_<double>(3,3) << 
      1, 0, (Act_RobotV/robot_velocity.angular.z)*(cos(Est_RobotTH+robot_velocity.angular.z*realsec)-cos(Est_RobotTH)),
      0, 1, (Act_RobotV/robot_velocity.angular.z)*(sin(Est_RobotTH+robot_velocity.angular.z*realsec)-sin(Est_RobotTH)),
      0, 0, 1);
  }
  //信念分布の共分散行列式Σ(3☓3)(発生する誤差はここにまとまっている)
  std::cout <<"Cov="<<Cov<< std::endl;
  std::cout <<"Ft="<<Ft<< std::endl;
  std::cout <<"At="<<At<< std::endl;
  std::cout <<"Mt="<<Mt<< std::endl;

  Cov=Ft*Cov*Ft.t()+At*Mt*At.t();
  std::cout <<"Cov="<<Cov<< std::endl;


  //信念分布の中心(平均)μ(一つ前の推定位置(信念分布の中心)と指令値)
  if(robot_velocity.angular.z==0){
    Est_RobotX=Est_RobotX+(Des_RobotV*cos(Est_RobotTH)*realsec);
    Est_RobotY=Est_RobotY+(Des_RobotV*sin(Est_RobotTH)*realsec);
    Est_RobotTH=Est_RobotTH+(robot_velocity.angular.z*realsec);
  }
  else{
    Est_RobotX=Est_RobotX+((Des_RobotV/robot_velocity.angular.z)*(sin(Est_RobotTH+robot_velocity.angular.z*realsec)-sin(Est_RobotTH)));
    Est_RobotY=Est_RobotY+((Des_RobotV/robot_velocity.angular.z)*(-cos(Est_RobotTH+robot_velocity.angular.z*realsec)+cos(Est_RobotTH)));
    Est_RobotTH=Est_RobotTH+(robot_velocity.angular.z*realsec);
  }
  std::cout << "Est_RobotX=" <<Est_RobotX<< std::endl;
  std::cout << "Est_RobotY=" <<Est_RobotY<< std::endl;
  std::cout << "Est_RobotTH=" <<Est_RobotTH<< std::endl;

  EST_Robot= (cv::Mat_<double>(3,1) <<
    Est_RobotX,
    Est_RobotY,
    Est_RobotTH);

  //更新ステップ------------------------------------------------------------------
  std::cout <<"更新ステップ"<< std::endl;
  //更新ステップはランドーマークの数だけ更新を行う
  for(){
    //観測方程式(信念分布の中心位置から見たLMまでの距離と角度)(理想推定)
    hu = (cv::Mat_<double>(2,1) <<
      sqrt((Est_RobotX-LM[0][0])*(Est_RobotX-LM[0][0]) + (Est_RobotY-LM[0][1])*(Est_RobotY-LM[0][1])),
      atan2(LM[0][1]-Est_RobotY,LM[0][0]-Est_RobotX) - Est_RobotTH);

    double lu=sqrt((Est_RobotX-LM[0][0])*(Est_RobotX-LM[0][0])+(Est_RobotY-LM[0][1])*(Est_RobotY-LM[0][1]));
    std::cout <<"test1"<< std::endl;


    //観測方程式のヤコビ行列Ht信念分布の中心と世界座標ランドマーク位置(LM[0][0],LM[0][1])(理想推定)
    Ht= (cv::Mat_<double>(2,3) << //共分散Ht
      (Est_RobotX-LM[0][0])/lu,       (Est_RobotY-LM[0][1])/lu,       0,
      (LM[0][1]-Est_RobotY)/(lu*lu),  (Est_RobotX-LM[0][0])/(lu*lu), -1);

    //センサーの誤差共分散(直進と角度の誤差共分散を代入)
    Qt= (cv::Mat_<double>(2,2) <<
      (lu*0.14)*(lu*0.14),  0,
      0,            0.05*0.05);

    //カルマンゲインK
    cv::Mat tempK=Qt+(Ht*Cov*Ht.t());
    K=Cov*Ht.t()*tempK.inv();

    //センサー値を反映するための更新式
    //信念分布の分散Σ
    Cov=(I-K*Ht)*Cov;

    //信念分布の中心位置(推定自己位置)μ=EST_Robot
    //zはセンサーから取得したマーカーまでの距離と角度データ
    //hは推定位置からみたマーカーまでの距離と角度(理想推定値)
    //多分カルマンゲインをかけることで2次から3次への拡張が可能
    EST_Robot=K*(zu-hu)+EST_Robot;
  }

  Est_RobotX=EST_Robot.at<double>(0);
  Est_RobotY=EST_Robot.at<double>(1);
  Est_RobotTH=EST_Robot.at<double>(2);

  std::cout <<"test4"<< std::endl;
  }//if(time0 != false)→end; //プログラム初回時カルマンは実行しない(Δtがないから)
    // 画面表示
    cv::imshow(win_src, image);
    cv::imshow(win_dst, img_dst);

    robot_velocity.linear.x  = 0.0; // 並進速度の初期化
    robot_velocity.angular.z = 0.0; // 回転速度の初期化
    //Rviz関連
    if(kaisu>1){
      //指令値状態tf(map Des_Robot間のlink)-----------------------------------------------------------------------------------------
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

      //推定状態tf(map Des_Robot間のlink)-----------------------------------------------------------------------------------------
      //ここがカメラの姿勢部分
      std::string MaptoEst_Robot_frame = "MaptoEst_Robot_link";
      //微小区間回転行列
      RollX=0,PitchY=Est_RobotTH,YawZ=0;

      //カメラ位置(理論値)
      Est_Robot_pose.position.x = Est_RobotX;//赤
      Est_Robot_pose.position.y = Est_RobotY;//緑
      Est_Robot_pose.position.z = 0;//青(tf:Z,画像:Y)
      Est_Robot_pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(YawZ,RollX,PitchY);

      static tf::TransformBroadcaster br_Est_Robot_pose;
      tf::Transform Est_Robot_transform;
      poseMsgToTF(Est_Robot_pose, Est_Robot_transform);
      br_Est_Robot_pose.sendTransform(tf::StampedTransform(Est_Robot_transform, ros::Time::now(), source_frame, MaptoEst_Robot_frame));
      std::cout <<"Est_Robot_pose.position.x="<<Est_Robot_pose.position.x<< std::endl;
      std::cout <<"Est_Robot_pose.position.y="<<Est_Robot_pose.position.y<< std::endl;
      std::cout <<"Est_Robot_pose.orientation.z="<<Est_Robot_pose.orientation.z<< std::endl;
      std::cout <<"Est_Robot_pose.orientation.w="<<Est_Robot_pose.orientation.w<< std::endl;

      //tf(Est_Robot camera_link間のlink)-----------------------------------------------------------------------------------------
      geometry_msgs::Pose Est_Robot_Link_pose;
      std::string Est_Robot_camera_frame = "Est_Robot_camera_link";

      //std::string Est_Robot_Link_Link_frame = "Est_Robot_Link_Link_link";
      Est_Robot_Link_pose.position.x = 0;
      Est_Robot_Link_pose.position.y = 0;
      Est_Robot_Link_pose.position.z = 0.67;//ここに実際のカメラの高さを入れる
      Est_Robot_Link_pose.orientation.w = 1.0;
      static tf::TransformBroadcaster br_Est_Robot_Link_pose;
      tf::Transform Est_Robot_Link_transform;
      poseMsgToTF(Est_Robot_Link_pose, Est_Robot_Link_transform);
      br_Est_Robot_Link_pose.sendTransform(tf::StampedTransform(Est_Robot_Link_transform, ros::Time::now(), MaptoEst_Robot_frame, Est_Robot_camera_frame));
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

      Est_Robot_pose.position.x = 0;
      Est_Robot_pose.position.y = 0;
      Est_Robot_pose.position.z = 0;
      Est_Robot_pose.orientation.x=0;
      Est_Robot_pose.orientation.y=0;
      Est_Robot_pose.orientation.z=0;
      Est_Robot_pose.orientation.w=0;
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

      Est_pose.header.stamp = ros::Time::now();
      Est_pose.header.frame_id = source_frame;
      Est_pose.pose.position = Est_Robot_pose.position;
      Est_pose.pose.orientation = Est_Robot_pose.orientation;
      Est_path.header.stamp = ros::Time::now();
      Est_path.header.frame_id = source_frame;
      Est_path.poses.push_back(Est_pose);
      Est_pub_plan.publish(Est_path);

    kaisu++;
    time0=true;//一回目スキップ
    endTime=startTime;//動作終了時刻取得
    cv::waitKey(1);//ros::spinにジャンプする
}


//メイン関数
int main(int argc,char **argv){
	ros::init(argc,argv,"marker2");//rosを初期化
	ros::NodeHandle nhSub;//ノードハンドル
  if(kaisu!=0){ros_begin = ros::Time::now();}
	//subscriber関連
  //Realsensesの時(roslaunch realsense2_camera rs_camera.launch align_depth:=true)(Depth修正版なのでこっちを使うこと)
	message_filters::Subscriber<nav_msgs::Odometry> odom_sub(nhSub, "/robot1/odom", 1);//センサーメッセージを使うときは対応したヘッダーが必要
	message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nhSub, "/camera/color/image_raw", 1);//センサーメッセージを使うときは対応したヘッダーが必要
	message_filters::Subscriber<sensor_msgs::Image> depth_sub(nhSub, "/camera/aligned_depth_to_color/image_raw", 1);

  typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry,sensor_msgs::Image,sensor_msgs::Image> MySyncPolicy;	
	message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10),odom_sub,rgb_sub, depth_sub);
	sync.registerCallback(boost::bind(&callback,_1, _2, _3));

  ros::NodeHandle nhPub;
  //Des_pub_plan = nhPub.advertise<nav_msgs::Path>("/get_multi_path",1000);
  Des_pub_plan = nhPub.advertise<nav_msgs::Path>("/Des_path",1000);
  Act_pub_plan = nhPub.advertise<nav_msgs::Path>("/Act_path",1000);
  Est_pub_plan = nhPub.advertise<nav_msgs::Path>("/Est_path",1000);
    
	ros::spin();//トピック更新待機
			
	return 0;
}