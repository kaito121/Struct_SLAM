//20211029 テンプレートの拡大を導入
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

using namespace std;
using namespace cv;

std::string win_src = "src";
std::string win_dst = "dst";
std::string win_dst2 = "dst2";//追跡チェック用
std::string win_point = "point";
std::string win_master_temp = "master_temp";
std::string source_frame = "map";//mapフレーム

ros::Subscriber odom_sub;//ロボットオドメトリー受信用
ros::Publisher  pub;//速度送信用
ros::Publisher Des_pub_plan;//カメラ経路送信用(指令値)
ros::Publisher Act_pub_plan;//カメラ経路送信用(観測値)
ros::Publisher Est_pub_plan;//カメラ経路送信用(推定値)

bool time0 = false;//カルマン推定データ初回回避用(カルマン動作した時TRUE)
bool X_25=false;
bool TH_90=false;
bool Y_05=false;
bool reset = true;//初回プログラム用（特徴点検出&特徴点再検出用)
bool swap_on = true;//初回プログラム実行時はswapしない(データの浅いコピーの影響を考慮)
bool Tracking = false;//3回目テンプレートマッチング動作

int kaisu=0,kaisuM1=0,kaisuV1=0;
int depth_point_prev_ok=0; //depth取得可能な特徴点の数
int depth_point_curr_ok=0; //depth取得可能な特徴点の数
int depth_points_currA_ok=0; //depth取得可能な特徴点の数

//特徴点検出関連
cv::Mat img_dst,image_curr,image_prev,img_dst2;//画像定義
cv::Mat FeaturePoint_curr[100],FeaturePoint_prev[100];//特徴点周囲の切り取った画像(テンプレート画像)
cv::Mat EST_scope[100];//特徴点周囲の切り取った画像(予測範囲画像)
cv::Mat img_template1,img_master_temp;

vector<cv::Point2f> points_prev, points_curr;//特徴点定義
vector<cv::Point3f> camera_point_p,camera_point_c;//特徴点定義(カメラ座標系)
vector<cv::Point3f> world_point_p,world_point_c;//特徴点定義(世界座標系)
vector<cv::Point3f> Est_point;//特徴点定義(運動復元
vector<cv::Point2f> Est_pixel;//特徴点定義(運動復元
vector<cv::Point2f> Matching_pixel_curr,Matching_pixel_prev;//テンプレートの中心座標
int matchC_curr=0,matchC_prev=0;//マッチングしたテンプレート数
cv::Mat Matching_Templ_curr[500],Matching_Templ_prev[500];//マッチングしたテンプレート画像キープ用

int template_size=10;
double cropx=10*2.5;//予測範囲の範囲(template_size*n)
double cropy=10*2.5;//予測範囲の範囲(template_size*n)

//int template_size=10;
//double cropx=10*3;//予測範囲の範囲(template_size*n)
//double cropy=10*3;//予測範囲の範囲(template_size*n)
cv::Point min_pt1[100], max_pt1[100];//テンプレートマッチング用変数
double min_val1[100], max_val1[100];

//テンプレートマッチ用変数
cv::Mat MP_curr_Templ[100];//マッチテンプレート画像
vector<cv::Point2f> MP_curr_pixel;//マッチテンプレートの画像座標系
vector<cv::Point3f> MP_curr_camera;//マッチテンプレートのカメラ座標系
vector<cv::Point3f> MP_curr_world;//マッチテンプレートの世界座標系
vector<cv::Point3f> MP_curr_world2;//マッチテンプレートの世界座標系
double DMP_curr[600];//Depth_Matching_pixel_prev(マッチテンプレート座標のDepth)
int DMP_curr_ok=0;//マッチテンプレートのDepth取得可能数

//テンプレートマッチ用変数(3回目動作用)
cv::Mat MP_curr2_Templ[100];//マッチテンプレート画像
vector<cv::Point2f> MP_curr2_pixel;//マッチテンプレートの画像座標系
vector<cv::Point3f> MP_curr2_camera;//マッチテンプレートのカメラ座標系
vector<cv::Point3f> MP_curr2_world;//マッチテンプレートの世界座標
double DMP_curr2[600];//Depth_Matching_pixel_prev(マッチテンプレート座標のDepth)
int DMP_curr2_ok=0;//マッチテンプレートのDepth取得可能数
double length;//テンプレートの距離比較用(追加更新動作)

double MPcoix[300],MPcoiy[300];
vector<cv::Point3f> Est_MP_point;//特徴点定義(運動復元
vector<cv::Point2f> Est_MP_pixel;//特徴点定義(運動復元
cv::Mat EST_MP_scope[100];//予測範囲クロップ
int EST_MP_ok=0;//画面内のテンプレート数
cv::Mat MP_prev_Templ[100];//マッチテンプレート画像
vector<cv::Point2f> MP_prev_pixel;//マッチテンプレート座標の画像座標系
vector<cv::Point3f> MP_prev_camera;//マッチテンプレート座標のカメラ座標系
vector<cv::Point3f> MP_prev_world;//マッチテンプレート座標の世界座標系
double DMP_prev[600];//Depth_Matching_pixel_prev(マッチテンプレート座標のDepth)
int DMP_prev_ok=0;//マッチテンプレートのDepth取得可能数

//マッチング追跡不可能時使用要素-------------------------
vector<cv::Point3f> MP_keep_camera;//推定カメラ座標
vector<cv::Point3f> MP_keep_world;//推定世界座標
vector<cv::Point3f> MP_Est_prev_camera;//世界座標推定用カメラ座標
vector<cv::Point3f> MP_Est_prev_world;//世界座標推定用世界座標
vector<cv::Point3f> MP_Est_curr_camera;//推定カメラ座標
vector<cv::Point3f> MP_Est_curr_world;//推定世界座標

float depth_point_prev[600],depth_point_curr[600];
double CameraLMP[2];

sensor_msgs::CameraInfo camera_info;//CameraInfo受け取り用
geometry_msgs::Twist robot_velocity;//指令速度
nav_msgs::Odometry robot_odometry;//ロボットのオドメトリー


ros::Time ros_begin;//プログラム時間
ros::WallTime wall_begin = ros::WallTime::now();//プログラム開始時の時間
ros::WallDuration wall_prev;//一つ前の時間
ros::WallDuration wall_systemtime;//サンプリング時間
//ros::Time ros_begin = ros::Time::now();
struct timeval startTime, endTime;  // 構造体宣言
struct timeval startTime2, endTime2;  // 構造体宣言
struct timeval startTimeV1, endTimeV1;  // 構造体宣言
struct timeval startTimeM1, endTimeM1;  // 構造体宣言

float realsec;//サンプリング時間（C++)
float ALLrealsec;//サンプリング時間（C++)
float realsecV1,ALLrealsecV1;//直進動作時間（C++)
float realsecM1,ALLrealsecM1;//回転動作時間（C++)

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
cv::Mat Zu[40],Zu_P[600];//センサーの観測値(仮)
//マーカーの世界座標登録--------------------------------------------------------
cv::Mat_<float>MarkerW[40]=cv::Mat_<float>(3, 1);//マーカーの世界座標

double ZNL[15];
double pixelX,pixelY,Xv,Yv;

ofstream act_v("/home/fuji/catkin_ws/src/Struct_SLAM/src/robot/date/ACT_V.txt");
ofstream act_u("/home/fuji/catkin_ws/src/Struct_SLAM/src/robot/date/ACT_ω.txt");
ofstream des_v("/home/fuji/catkin_ws/src/Struct_SLAM/src/robot/date/DES_V.txt");
ofstream des_u("/home/fuji/catkin_ws/src/Struct_SLAM/src/robot/date/DES_ω.txt");
ofstream act_robot("/home/fuji/catkin_ws/src/Struct_SLAM/src/robot/date/ACT_robot.txt");
ofstream des_robot("/home/fuji/catkin_ws/src/Struct_SLAM/src/robot/date/DES_robot.txt");
ofstream est_robot("/home/fuji/catkin_ws/src/Struct_SLAM/src/robot/date/EST_robot.txt");
ofstream camera_robot("/home/fuji/catkin_ws/src/Struct_SLAM/src/robot/date/sensor_date.txt");
ofstream opt_pointx("/home/fuji/catkin_ws/src/Struct_SLAM/src/robot/date/opt_point1x.txt");
ofstream opt_pointy("/home/fuji/catkin_ws/src/Struct_SLAM/src/robot/date/opt_point1y.txt");
ofstream opt_pointz("/home/fuji/catkin_ws/src/Struct_SLAM/src/robot/date/opt_point1z.txt");
ofstream zn_pointl("/home/fuji/catkin_ws/src/Struct_SLAM/src/robot/date/zn_pointl.txt");
ofstream zn_pointth("/home/fuji/catkin_ws/src/Struct_SLAM/src/robot/date/zn_pointth.txt");
ofstream hu_pointl("/home/fuji/catkin_ws/src/Struct_SLAM/src/robot/date/hu_pointl.txt");
ofstream hu_pointth("/home/fuji/catkin_ws/src/Struct_SLAM/src/robot/date/hu_pointth.txt");
ofstream znL("/home/fuji/catkin_ws/src/Struct_SLAM/src/robot/date/znL.txt");
ofstream ESTL("/home/fuji/catkin_ws/src/Struct_SLAM/src/robot/date/ESTL.txt");
ofstream Cameralm("/home/fuji/catkin_ws/src/Struct_SLAM/src/robot/date/Cameralm.txt");
ofstream Camerath("/home/fuji/catkin_ws/src/Struct_SLAM/src/robot/date/Camerath.txt");

ofstream W_point("/home/fuji/catkin_ws/src/Struct_SLAM/src/robot/date/W_point.txt");
ofstream W_point2("/home/fuji/catkin_ws/src/Struct_SLAM/src/robot/date/W_point2.txt");

ofstream Realsec("/home/fuji/catkin_ws/src/Struct_SLAM/src/robot/date/opt_test11_sec.txt");
ofstream EST("/home/fuji/catkin_ws/src/Struct_SLAM/src/robot/date/Est_point.txt");


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
   
  	image = bridgeImage->image.clone();//image変数に変換した画像データを代入
  	depthimage = bridgedepthImage->image.clone();//image変数に変換した画像データを代入

	  img_dst = image.clone();
	  img_dst2 = image.clone();
    img_dst2 = cv::Scalar(255,255,255);
    

  //マーカー検出+外部パラメータ推定-------------------------------------------------------------------------------------------  
  //カメラ内部パラメータ読み込み
  cv::Mat cameraMatrix;
  cv::FileStorage fs;
  cv::Mat_<float> intrinsic_K= cv::Mat_<float>(3, 3);
  fs.open("/home/fuji/catkin_ws/src/Struct_SLAM/src/marker/realsense_para.xml", cv::FileStorage::READ);
  fs["intrinsic"]>>cameraMatrix;
  intrinsic_K=cameraMatrix;

  //カメラの歪みパラメータ読み込み
  cv::Mat distCoeffs;
  cv::FileStorage fd;
  fd.open("/home/fuji/catkin_ws/src/Struct_SLAM/src/marker/realsense_para.xml", cv::FileStorage::READ);
  fd["distortion"]>>distCoeffs;

  //マーカ辞書作成 6x6マスのマーカを250種類生成
  cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);

  //charucoボード生成 10x7マスのチェスボード、グリッドのサイズ0.04f、グリッド内マーカのサイズ0.02f
  cv::Ptr<cv::aruco::CharucoBoard> board = cv::aruco::CharucoBoard::create(10, 7, 0.04f, 0.02f, dictionary);
  cv::Ptr<cv::aruco::DetectorParameters> params = cv::aruco::DetectorParameters::create();//マーカー検出時メソッドを指定
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

        //ロボット座標系(tf座標系)とカメラ観測座標系ではxとyの符号が逆なので注意(ロボット(tf)座標:Y = カメラ観測座標:-X,ロボット(tf)座標:Z = カメラ観測座標:-Y )
        std::cout << "マーカーのカメラ座標:MC_point["<<markerIds.at(i)<<"]={"<< -MC_point[markerIds.at(i)][0] <<","<<-MC_point[markerIds.at(i)][1]<<","<<MC_point[markerIds.at(i)][2]<<"}"<< std::endl;
        //std::cout <<"X="<< MC_point[markerIds.at(i)][0]/MC_point[markerIds.at(i)][2]<< std::endl;
        //std::cout <<"Y="<< MC_point[markerIds.at(i)][1]/MC_point[markerIds.at(i)][2]<< std::endl;
      }
      //マーカーまでの距離と角度を求める(観測値)
      CameraLM[markerIds.at(i)][0]=sqrt((-MC_point[markerIds.at(i)][0]*-MC_point[markerIds.at(i)][0])+(MC_point[markerIds.at(i)][2]*MC_point[markerIds.at(i)][2]));
      CameraLM[markerIds.at(i)][1]=atan2(-MC_point[markerIds.at(i)][0],MC_point[markerIds.at(i)][2]);
      std::cout <<"CameraLM["<<markerIds.at(i)<<"][0]="<<CameraLM[markerIds.at(i)][0]<< std::endl;//マーカーまでの距離
      std::cout <<"CameraLM["<<markerIds.at(i)<<"][1]="<<CameraLM[markerIds.at(i)][1]<< std::endl;//マーカーまでの角度
      camera_robot<< "ALLrealsec=" <<ALLrealsec<< " ,realsec=" <<realsec<<" ,CameraLM["<<markerIds.at(i)<<"][0]=" <<CameraLM[markerIds.at(i)][0]<<",CameraLM["<<markerIds.at(i)<<"][1]=" <<CameraLM[markerIds.at(i)][1]<<"\n";

      Zu[markerIds.at(i)] = (cv::Mat_<double>(2,1) <<
        CameraLM[markerIds.at(i)][0],
        CameraLM[markerIds.at(i)][1]);
    }
    Cameralm<<CameraLM[markerIds.at(0)][0]<<"\n";
    Camerath<<CameraLM[markerIds.at(0)][1]<<"\n";
  }
  else{std::cout <<"マーカー観測不能"<< std::endl;}
  //ここまでがマーカー観測----------------------------------------------

  //特徴点検出--------------------------------------------------------------------------------------------------------------
	cv::cvtColor(image, image_curr, cv::COLOR_BGR2GRAY);//グレースケール化
  //初回検出プログラム-----------------------------------------------------------------------------------------------------

  //20211026   
  //初回は特徴点検出とテンプレート作成のみ行う
  //二回目以降検出した特徴点を使用してオドメトリー範囲予測を行い、テンプレマッチを行う。→マッチしたテンプレートをキープ
  //また2回目以降も初回と同様に特徴点検出を行いテンプレートを作成する。
  //3回目以降前の動作でキープしたテンプレートと作成したテンプレートの座標を使用し、オドメトリー範囲予測を行い、テンプレマッチ→マッチした新規テンプレートのみ保存
  //3回目も同様に特徴点検出を行いテンプレート作成

  //予測範囲が画面外に行った場合は画面外に行ったテンプレートを削除（このとき番号は詰める）
  //全ての予測範囲が画面外に行った場合、もしくはキープするテンプレートがなくなった場合は初回動作に戻る

  //特徴点検出とテンプレート作成
	    std::cout <<"初回検出プログラム"<< std::endl;
      depth_point_curr_ok=0; //depth取得可能な特徴点の数(初期化)
      swap_on=false;
      //cv::goodFeaturesToTrack(今画像, 今の特徴点, 特徴点の個数, 0.01, 10, cv::Mat(), 3, 3, 0, 0.04);
		  cv::goodFeaturesToTrack(image_curr, points_curr, 10, 0.01, 10, cv::Mat(), 3, 3, 0, 0.04);// 特徴点検出(グレースケール画像から特徴点検出)
		  cv::cornerSubPix(image_curr, points_curr, cv::Size(10, 10), cv::Size(-1, -1), cv::TermCriteria(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, 20, 0.03));
      camera_point_c.resize(points_curr.size());//要素数初期設定(座標変換用)
	    std::cout <<"test2_points_curr.size()="<<points_curr.size()<< std::endl;
      
      image.copyTo(img_master_temp);//切り取り場所を表示
      //特徴点を検出したら特徴点の周りの画像をクロップして保存
      //テンプレートマッチングの不具合を考慮(外枠8Pixelの特徴点を検出しない)＋Depthが取得不能な点を削除
      for (int i = 0; i < points_curr.size(); i++) {
        if(template_size<points_curr[i].y&&points_curr[i].y<480-template_size){
          if(template_size<points_curr[i].x&&points_curr[i].x<640-template_size){
            depth_point_curr[i] = depthimage.at<float>(cv::Point(points_curr[i].x,points_curr[i].y));
            //Depthが取得できない特徴点を削除する+Depthの外れ値を除く
            if(depth_point_curr[i]>0.001&&depth_point_curr[i]<10000){
              camera_point_c[depth_point_curr_ok].x = depth_point_curr[i] * ((points_curr[i].x - 324.473) / 615.337)/1000;//メートル表示変換
              camera_point_c[depth_point_curr_ok].y = depth_point_curr[i] * ((points_curr[i].y - 241.696) / 615.458)/1000;
              camera_point_c[depth_point_curr_ok].z = depth_point_curr[i]/1000;
              points_curr[depth_point_curr_ok] = points_curr[i];
			        cv::circle(img_dst, points_curr[i], 6, Scalar(255,0,0), -1, cv::LINE_AA);
              //テンプレート作成----------------------------------------------------------------------------------------
              std::cout <<"特徴点画像クロッププログラム["<<i<<"]"<< std::endl;//最初のフレーム
              cv::Rect roi2(cv::Point(points_curr[i].x-template_size,points_curr[i].y-template_size), cv::Size(template_size*2, template_size*2));//特徴点を中心とした16☓16pixelの画像を切り取る
              FeaturePoint_curr[depth_point_curr_ok] = image(roi2); // 切り出し画像
              cv::rectangle(img_master_temp, roi2,cv::Scalar(255, 255, 255), 2);//テンプレート位置
              //cv::rectangle(img_master_temp, cv::Point(points_curr[i].x-template_size,points_curr[i].y+template_size), 
              //cv::Point(points_curr[i].x+template_size,points_curr[i].y-template_size), cv::Scalar(255, 255, 255), 2, cv::LINE_AA);//四角形を描写(白)
              std::cout <<"特徴点の画像座標(depth_point_curr_ok)["<<depth_point_curr_ok<<"]="<<points_curr[depth_point_curr_ok]<< std::endl;//特徴点の座標(範囲制限後)
              depth_point_curr_ok=depth_point_curr_ok+1;//Depth取得可能+テンプレ制限範囲内の個数をカウント
            }
          }
        }
		  }
      points_curr.resize(depth_point_curr_ok);//Depth取得可能数でリサイズ(二次元)
      points_prev.resize(depth_point_curr_ok);//Depth取得可能数でリサイズ(二次元)
      camera_point_c.resize(depth_point_curr_ok);//Depth取得可能数でリサイズ(三次元カメラ座標)
      camera_point_p.resize(depth_point_curr_ok);//Depth取得可能数でリサイズ(三次元カメラ座標)
      world_point_c.resize(depth_point_curr_ok);//Depth取得可能数でリサイズ(世界座標)
      Est_point.resize(depth_point_curr_ok);//Depth取得可能数でリサイズ(運動復元カメラ座標)
      Est_pixel.resize(depth_point_curr_ok);//Depth取得可能数でリサイズ(運動復元画像座標)
      //Matching_pixel.resize(depth_point_curr_ok);//Depth取得可能数でリサイズ(運動復元画像座標)
  
      reset = false;//if文切り替え
	    std::cout <<"初回検出プログラム終了"<< std::endl;

    //ロボット指令-------------------------------------------------------------------
    ros::NodeHandle nh;
    robot_odometry=*msg;
    pub= nh.advertise<geometry_msgs::Twist>("/robot1/mobile_base/commands/velocity", 10);
    //if(Des_RobotX<=0.50){//xが1以上になったら終了  
    /*if(Des_RobotTH>=-3.141592653/2){//研究室
      robot_velocity.linear.x  = 0.0; // 並進速度の初期化
      //robot_velocity.angular.z = 0.0; // 回転速度の初期化
      robot_velocity.angular.z = -0.0; // 回転速度の初期化
      //robot_velocity.angular.z  =  -(0.5+(0.0176*ALLrealsecM1+0.11));//(0.5)研究室
    }
    else{
      robot_velocity.linear.x  =  0.0;//(0.1)
      robot_velocity.angular.z  =  0;
    }*/
    //pub.publish(robot_velocity);    // 速度指令メッセージをパブリッシュ（送信）
    

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
        }
        robot_velocity.linear.x  = 0.3;//(0.1)
        robot_velocity.angular.z = 0.0; // 回転速度の初期化}//xが1以上になったら終了
        if(Des_RobotX>=4.5){
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
        }
        robot_velocity.linear.x  =  0.0;
        robot_velocity.angular.z  =  0.2+(0.0176*ALLrealsecM1+0.11);//(0.5)廊下
        //robot_velocity.angular.z  =  -(0.2+(0.0176*ALLrealsecM1+0.11));//(0.5)研究室
        if(Des_RobotTH>=3.141592653/2){//廊下
        //if(Des_RobotTH<=-3.141592653/2.4){//研究室
          TH_90=true;
          robot_velocity.linear.x  = 0.0; // 並進速度vの初期化
          robot_velocity.angular.z = 0.0; // 回転速度ωの初期化}//xが1以上になったら終了
          pub.publish(robot_velocity);    // 速度指令メッセージをパブリッシュ（送信）
          usleep(7*100000);//0.5秒ストップ(マイクロ秒)
        }
        kaisuM1++;
      }

      if(X_25==true&&TH_90==true&&Y_05==false){
        robot_velocity.linear.x  = 0.3;//(0.1)
        robot_velocity.angular.z = 0.0; // 回転速度の初期化}//xが1以上になったら終了
        //if(Des_RobotY>=2.0){//研究室
        if(Des_RobotY>=10.0){//廊下
          Y_05=true;
        }
      }
      if(X_25==true&&TH_90==true&&Y_05==true){
          robot_velocity.linear.x  = 0.0; // 並進速度vの初期化
          robot_velocity.angular.z = 0.0; // 回転速度ωの初期化}//xが1以上になったら終了
      }

    pub.publish(robot_velocity);    // 速度指令メッセージをパブリッシュ（送信）

      if(X_25==false&&TH_90==false&&Y_05==false){
        robot_velocity.linear.x  = 0.3;//(0.1)
        robot_velocity.angular.z = 0.0; // 回転速度の初期化}//xが1以上になったら終了
      }
      if(X_25==true&&TH_90==false&&Y_05==false){
        robot_velocity.linear.x  =  0.0;
        robot_velocity.angular.z  =  0.3;//(0.5)廊下はこっち
        //robot_velocity.angular.z  =  -0.5;//(0.5)研究室
        //robot_velocity.angular.z  =  -0.3;//(0.5)研究室(rosbag_0.2)
      }
      if(X_25==true&&TH_90==true&&Y_05==false){
        robot_velocity.linear.x  = 0.3;//(0.1)
        robot_velocity.angular.z = 0.0; // 回転速度の初期化}//xが1以上になったら終了
      }
      if(X_25==true&&TH_90==true&&Y_05==true){
          robot_velocity.linear.x  = 0.0; // 並進速度vの初期化
          robot_velocity.angular.z = 0.0; // 回転速度ωの初期化}//xが1以上になったら終了
      }

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
      DES_Robot = cv::Mat_<double>::zeros(3, 1);//目標指令状態
      ACT_Robot = cv::Mat_<double>::zeros(3, 1);//雑音の影響を考慮した実際の状態
      EST_Robot = cv::Mat_<double>::zeros(3, 1);//推定状態

      Cov = (cv::Mat_<double>(3,3) << //共分散Q
        1e-10, 0,      0,
        0,     1e-10,  0,
        0,     0,      1e-10);
      I = cv::Mat_<double>::eye(3, 3);//単位行列
      std::cout <<"初期設定"<< std::endl;
    }

    Matching_pixel_curr.resize(1000);//Matching_pixel_curr定義(ここの大きさを超えるとエラーになる)
    //テンプレート範囲予測とテンプレートマッチング----------------------------------------------------------------------
    //テンプレートマッチングは1つ前のテンプレートを使用(テンプレート:t+n-1,マッチング画像:t+n)
  if(time0 != false){
    if (reset == false) {//初回以降動作
      //ロボットの動きから特徴点の運動復元を行う(特徴点の状態方程式を使用)
      std::cout <<"二回目動作:depth_point_prev_ok="<<depth_point_prev_ok<< std::endl;

      matchC_curr=0;
      for(int i=0;i<depth_point_prev_ok;i++){
        std::cout <<"\n"<< std::endl;
        std::cout <<"camera_point_p["<<i<<"]="<<camera_point_p[i]<< std::endl;
        std::cout <<"robot_odometry.twist.twist.angular.z="<<robot_odometry.twist.twist.angular.z<< std::endl;
        std::cout <<"realsec="<<realsec<< std::endl;
        std::cout <<"Act_RobotV="<<Act_RobotV<< std::endl;

        Est_point[i].x=camera_point_p[i].x+robot_odometry.twist.twist.angular.z*realsec*camera_point_p[i].z-Act_RobotV*sin(-Act_RobotTH)*realsec;
        Est_point[i].y=camera_point_p[i].y;
        Est_point[i].z=camera_point_p[i].z-robot_odometry.twist.twist.angular.z*realsec*camera_point_p[i].x-Act_RobotV*cos(-Act_RobotTH)*realsec;

        Est_pixel[i].x=324.473+(Est_point[i].x/Est_point[i].z)*615.337;
        Est_pixel[i].y=241.696+(Est_point[i].y/Est_point[i].z)*615.458;

        //cv::circle(img_dst, cv::Point(Est_pixel[i].x,Est_pixel[i].y), 6, Scalar(0,255,255), -1, cv::LINE_AA);//一つ前の画像の座標
        cv::line(img_dst,cv::Point(points_prev[i].x,points_prev[i].y),cv::Point(Est_pixel[i].x,Est_pixel[i].y),cv::Scalar(0,0,255), 1, cv::LINE_AA);

        std::cout <<"Est_point["<<i<<"]=\n"<<Est_point[i]<< std::endl;//予測カメラ座標
        std::cout <<"Est_pixel["<<i<<"]=\n"<<Est_pixel[i]<< std::endl;//予測画像座標

        //求めた運動復元結果からテンプレートマッチングの予測範囲を作る(とりあえずタテヨコ2倍)
        std::cout << "マッチング範囲限定クロッププログラム"<< std::endl;

        //予測範囲が全て画面内の時
        if(cropx<=Est_pixel[i].x&&Est_pixel[i].x<=640-cropx&&cropy<=Est_pixel[i].y&&Est_pixel[i].y<=480-cropy){
          std::cout << "予測範囲が全て画面内の時["<<i<<"]"<< std::endl;
          cv::Rect roiEST(cv::Point(Est_pixel[i].x-cropx,Est_pixel[i].y-cropx), cv::Size(cropx*2, cropx*2));//線の中点を中心とした線の画像を切り取る
          EST_scope[i] = image(roiEST); // 切り出し画像
          cv::rectangle(img_dst, roiEST,cv::Scalar(0, 255, 255), 2);//マッチング予測範囲
          cv::imshow("EST_scope", EST_scope[i]);//黄色の特徴点を中心としたクロップ画像
        }
        //左側
        else if(0<=Est_pixel[i].x&&Est_pixel[i].x<cropx){
          //左上(xとyどちらもはみでる)
          if(0<=Est_pixel[i].y&&Est_pixel[i].y<cropy){
            std::cout << "左上(xとyどちらもはみでる)["<<i<<"]"<<std::endl;
            cv::Rect roiEST(cv::Point(0,0), cv::Size(Est_pixel[i].x+cropx, Est_pixel[i].y+cropy));//線の中点を中心とした線の画像を切り取る
            EST_scope[i] = image(roiEST); // 切り出し画像
           cv::rectangle(img_dst, roiEST,cv::Scalar(0, 255, 255), 2);//マッチング予測範囲
          }
          //左側(xははみ出ない)
          else if(cropy<=Est_pixel[i].y&&Est_pixel[i].y<=480-cropy){
            std::cout << "左側(xははみ出ない)["<<i<<"]"<<std::endl;
            cv::Rect roiEST(cv::Point(0,Est_pixel[i].y-cropy), cv::Size(Est_pixel[i].x+cropx, cropx*2));//線の中点を中心とした線の画像を切り取る
            EST_scope[i] = image(roiEST); // 切り出し画像
            cv::rectangle(img_dst, roiEST,cv::Scalar(0, 255, 255), 2);//マッチング予測範囲
          }
          //左下(xとyどちらもはみでる)
          else if(480-cropy<Est_pixel[i].y&&Est_pixel[i].y<=480){
            std::cout << "左下(xとyどちらもはみでる)["<<i<<"]"<<std::endl;
            cv::Rect roiEST(cv::Point(0,Est_pixel[i].y-cropy), cv::Size(Est_pixel[i].x+cropx, 480-Est_pixel[i].y+cropy));//線の中点を中心とした線の画像を切り取る
            EST_scope[i] = image(roiEST); // 切り出し画像
            cv::rectangle(img_dst, roiEST,cv::Scalar(0, 255, 255), 2);//マッチング予測範囲
          }
        }
        //上側(yははみ出ない)
        else if(cropx<=Est_pixel[i].x&&Est_pixel[i].x<=640-cropx&&0<=Est_pixel[i].y&&Est_pixel[i].y<cropy){
          std::cout << "上側(yははみ出ない)["<<i<<"]"<<std::endl;
          cv::Rect roiEST(cv::Point(Est_pixel[i].x-cropx,0), cv::Size(cropx*2, Est_pixel[i].y+cropy));//線の中点を中心とした線の画像を切り取る
          EST_scope[i] = image(roiEST); // 切り出し画像
          cv::rectangle(img_dst, roiEST,cv::Scalar(0, 255, 255), 2);//マッチング予測範囲
        }
        //右側
        else if(640-cropx<Est_pixel[i].x&&Est_pixel[i].x<=640){
          //右上(xとyどちらもはみでる)
          if(0<=Est_pixel[i].y&&Est_pixel[i].y<cropy){
            std::cout << "右上(xとyどちらもはみでる)["<<i<<"]"<<std::endl;
            cv::Rect roiEST(cv::Point(Est_pixel[i].x-cropx,0), cv::Size(640-Est_pixel[i].x+cropx, Est_pixel[i].y+cropy));//線の中点を中心とした線の画像を切り取る
            EST_scope[i] = image(roiEST); // 切り出し画像
            cv::rectangle(img_dst, roiEST,cv::Scalar(0, 255, 255), 2);//マッチング予測範囲
          }
          //右側(xははみ出ない)
          else if(cropy<=Est_pixel[i].y&&Est_pixel[i].y<=480-cropy){
            std::cout << "右側(xははみ出ない)["<<i<<"]"<<std::endl;
            cv::Rect roiEST(cv::Point(Est_pixel[i].x-cropx,Est_pixel[i].y-cropy), cv::Size(640-Est_pixel[i].x+cropx, cropy*2));//線の中点を中心とした線の画像を切り取る
            EST_scope[i] = image(roiEST); // 切り出し画像
            cv::rectangle(img_dst, roiEST,cv::Scalar(0, 255, 255), 2);//マッチング予測範囲
          }
          //右下(xとyどちらもはみでる)
          else if(480-cropy<Est_pixel[i].y&&Est_pixel[i].y<=480){
            std::cout << "右下(xとyどちらもはみでる)["<<i<<"]"<<std::endl;
            cv::Rect roiEST(cv::Point(Est_pixel[i].x-cropx,Est_pixel[i].y-cropy), cv::Size(640-Est_pixel[i].x+cropx, 480-Est_pixel[i].y+cropy));//線の中点を中心とした線の画像を切り取る
            EST_scope[i] = image(roiEST); // 切り出し画像
            cv::rectangle(img_dst, roiEST,cv::Scalar(0, 255, 255), 2);//マッチング予測範囲
          }
        }
        //下側(yははみ出ない)
        else if(cropx<=Est_pixel[i].x&&Est_pixel[i].x<=640-cropx&&480-cropy<=Est_pixel[i].y&&Est_pixel[i].y<480){
          std::cout << "下側(yははみ出ない)["<<i<<"]"<<std::endl;
          cv::Rect roiEST(cv::Point(Est_pixel[i].x-cropx,Est_pixel[i].y-cropy), cv::Size(cropx*2, 480-Est_pixel[i].y+cropy));//線の中点を中心とした線の画像を切り取る
          EST_scope[i] = image(roiEST); // 切り出し画像
          cv::rectangle(img_dst, roiEST,cv::Scalar(0, 255, 255), 2);//マッチング予測範囲
        }
        else{
          std::cout << "画面外["<<i<<"]"<< std::endl;
        }

        std::cout <<"FeaturePoint_prev[i].cols="<<FeaturePoint_prev[i].cols<<"FeaturePoint_prev[i].rows="<<FeaturePoint_prev[i].rows<< std::endl;//最初のフレーム
        std::cout <<"EST_scope[i].cols="<<EST_scope[i].cols<<"EST_scope[i].rows="<<EST_scope[i].rows<< std::endl;//最初のフレーム

        //予測範囲に対しテンプレートマッチングを行う
        std::cout << "テンプレートマッチングプログラム["<<i<<"]"<< std::endl;
        FeaturePoint_prev[i].copyTo(img_template1); // 切り出し画像
        //cv::imshow("win_FeaturePoint_prev", FeaturePoint_prev[i]);//黄色の特徴点を中心としたクロップ画像

        cv::Mat img_minmax1;
        // テンプレートマッチング
        cv::matchTemplate(EST_scope[i], img_template1, img_minmax1, cv::TM_CCOEFF_NORMED);//正規化相互相関(ZNCC)
        cv::minMaxLoc(img_minmax1, &min_val1[i], &max_val1[i], &min_pt1[i], &max_pt1[i]);
        std::cout << "min_val1(白)["<<i<<"]=" << min_val1[i] << std::endl;//一致度が上がると値が小さくなる
        std::cout << "max_val1(白)["<<i<<"]=" << max_val1[i] << std::endl;
        if(0.83<max_val1[i]){//最小値がしきい値以下なら表示
          //予測範囲が全て画面内の時
          if(cropx<=Est_pixel[i].x&&Est_pixel[i].x<=640-cropx&&cropy<=Est_pixel[i].y&&Est_pixel[i].y<=480-cropy){
            std::cout << "マッチング:全て画面内の時["<<i<<"]"<< std::endl;
            cv::rectangle(img_dst, cv::Rect(Est_pixel[i].x-cropx+max_pt1[i].x, Est_pixel[i].y-cropy+max_pt1[i].y, img_template1.cols, img_template1.rows), cv::Scalar(0, 255, 0), 3);//白枠
            cv::circle(img_dst, cv::Point(Est_pixel[i].x-cropx+max_pt1[i].x+(img_template1.cols/2), Est_pixel[i].y-cropy+max_pt1[i].y+(img_template1.rows/2)), 5, cv::Scalar(0, 255, 0), -1);//テンプレートの中心座標
            Matching_pixel_curr[matchC_curr].x=Est_pixel[i].x-cropx+max_pt1[i].x+(img_template1.cols/2);//マッチング中心座標(画像全体座標)
            Matching_pixel_curr[matchC_curr].y=Est_pixel[i].y-cropy+max_pt1[i].y+(img_template1.rows/2);//マッチング中心座標(画像全体座標)
            Matching_Templ_curr[matchC_curr]=img_template1;//マッチングしたテンプレート画像
            std::cout <<"マッチングの中心座標[matchC_curr="<<matchC_curr<<"]="<<Matching_pixel_curr[matchC_curr]<< std::endl;
            matchC_curr=matchC_curr+1;//マッチングの中心座標個数
          }
             //左側
          else if(0<=Est_pixel[i].x&&Est_pixel[i].x<cropx){
            //左上(xとyどちらもはみでる)
            if(0<=Est_pixel[i].y&&Est_pixel[i].y<cropy){
              std::cout << "マッチング:左上(xとyどちらもはみでる)["<<i<<"]"<<std::endl;
              cv::rectangle(img_dst, cv::Rect(Est_pixel[i].x-cropx+max_pt1[i].x, Est_pixel[i].y-cropy+max_pt1[i].y, img_template1.cols, img_template1.rows), cv::Scalar(0, 255, 0), 3);//白枠
              cv::circle(img_dst, cv::Point(Est_pixel[i].x-cropx+max_pt1[i].x+(img_template1.cols/2), Est_pixel[i].y-cropy+max_pt1[i].y+(img_template1.rows/2)), 5, cv::Scalar(0, 255, 0), -1);//テンプレートの中心座標
              Matching_pixel_curr[matchC_curr].x=Est_pixel[i].x-cropx+max_pt1[i].x+(img_template1.cols/2);//マッチング中心座標(画像全体座標)
              Matching_pixel_curr[matchC_curr].y=Est_pixel[i].y-cropy+max_pt1[i].y+(img_template1.rows/2);//マッチング中心座標(画像全体座標)
              Matching_Templ_curr[matchC_curr]=img_template1;//マッチングしたテンプレート画像
              std::cout <<"マッチングの中心座標[matchC_curr="<<matchC_curr<<"]="<<Matching_pixel_curr[matchC_curr]<< std::endl;
              matchC_curr=matchC_curr+1;//マッチングの中心座標個数
            }
            //左側(xははみ出ない)
            else if(cropy<=Est_pixel[i].y&&Est_pixel[i].y<=480-cropy){
              std::cout << "マッチング:左側(xははみ出ない)["<<i<<"]"<<std::endl;
            cv::rectangle(img_dst, cv::Rect(Est_pixel[i].x-cropx+max_pt1[i].x, Est_pixel[i].y-cropy+max_pt1[i].y, img_template1.cols, img_template1.rows), cv::Scalar(0, 255, 0), 3);//白枠
            cv::circle(img_dst, cv::Point(Est_pixel[i].x-cropx+max_pt1[i].x+(img_template1.cols/2), Est_pixel[i].y-cropy+max_pt1[i].y+(img_template1.rows/2)), 5, cv::Scalar(0, 255, 0), -1);//テンプレートの中心座標
            Matching_pixel_curr[matchC_curr].x=Est_pixel[i].x-cropx+max_pt1[i].x+(img_template1.cols/2);//マッチング中心座標(画像全体座標)
            Matching_pixel_curr[matchC_curr].y=Est_pixel[i].y-cropy+max_pt1[i].y+(img_template1.rows/2);//マッチング中心座標(画像全体座標)
            Matching_Templ_curr[matchC_curr]=img_template1;//マッチングしたテンプレート画像
            std::cout <<"マッチングの中心座標[matchC_curr="<<matchC_curr<<"]="<<Matching_pixel_curr[matchC_curr]<< std::endl;
            matchC_curr=matchC_curr+1;//マッチングの中心座標個数
            }
            //左下(xとyどちらもはみでる)
            else if(480-cropy<Est_pixel[i].y&&Est_pixel[i].y<=480){
              std::cout << "マッチング:左下(xとyどちらもはみでる)["<<i<<"]"<<std::endl;
            cv::rectangle(img_dst, cv::Rect(Est_pixel[i].x-cropx+max_pt1[i].x, Est_pixel[i].y-cropy+max_pt1[i].y, img_template1.cols, img_template1.rows), cv::Scalar(0, 255, 0), 3);//白枠
            cv::circle(img_dst, cv::Point(Est_pixel[i].x-cropx+max_pt1[i].x+(img_template1.cols/2), Est_pixel[i].y-cropy+max_pt1[i].y+(img_template1.rows/2)), 5, cv::Scalar(0, 255, 0), -1);//テンプレートの中心座標
            Matching_pixel_curr[matchC_curr].x=Est_pixel[i].x-cropx+max_pt1[i].x+(img_template1.cols/2);//マッチング中心座標(画像全体座標)
            Matching_pixel_curr[matchC_curr].y=Est_pixel[i].y-cropy+max_pt1[i].y+(img_template1.rows/2);//マッチング中心座標(画像全体座標)
            Matching_Templ_curr[matchC_curr]=img_template1;//マッチングしたテンプレート画像
            std::cout <<"マッチングの中心座標[matchC_curr="<<matchC_curr<<"]="<<Matching_pixel_curr[matchC_curr]<< std::endl;
            matchC_curr=matchC_curr+1;//マッチングの中心座標個数
            }
          }
          //上側(yははみ出ない)
          else if(cropx<=Est_pixel[i].x&&Est_pixel[i].x<=640-cropx&&0<=Est_pixel[i].y&&Est_pixel[i].y<cropy){
            std::cout << "マッチング:上側(yははみ出ない)["<<i<<"]"<<std::endl;
            cv::rectangle(img_dst, cv::Rect(Est_pixel[i].x-cropx+max_pt1[i].x, Est_pixel[i].y-cropy+max_pt1[i].y, img_template1.cols, img_template1.rows), cv::Scalar(0, 255, 0), 3);//白枠
            cv::circle(img_dst, cv::Point(Est_pixel[i].x-cropx+max_pt1[i].x+(img_template1.cols/2), Est_pixel[i].y-cropy+max_pt1[i].y+(img_template1.rows/2)), 5, cv::Scalar(0, 255, 0), -1);//テンプレートの中心座標
            Matching_pixel_curr[matchC_curr].x=Est_pixel[i].x-cropx+max_pt1[i].x+(img_template1.cols/2);//マッチング中心座標(画像全体座標)
            Matching_pixel_curr[matchC_curr].y=Est_pixel[i].y-cropy+max_pt1[i].y+(img_template1.rows/2);//マッチング中心座標(画像全体座標)
            Matching_Templ_curr[matchC_curr]=img_template1;//マッチングしたテンプレート画像
            std::cout <<"マッチングの中心座標[matchC_curr="<<matchC_curr<<"]="<<Matching_pixel_curr[matchC_curr]<< std::endl;
            matchC_curr=matchC_curr+1;//マッチングの中心座標個数
          }
          //右側
          else if(640-cropx<Est_pixel[i].x&&Est_pixel[i].x<=640){
            //右上(xとyどちらもはみでる)
            if(0<=Est_pixel[i].y&&Est_pixel[i].y<cropy){
              std::cout << "マッチング:右上(xとyどちらもはみでる)["<<i<<"]"<<std::endl;
            cv::rectangle(img_dst, cv::Rect(Est_pixel[i].x-cropx+max_pt1[i].x, Est_pixel[i].y-cropy+max_pt1[i].y, img_template1.cols, img_template1.rows), cv::Scalar(0, 255, 0), 3);//白枠
            cv::circle(img_dst, cv::Point(Est_pixel[i].x-cropx+max_pt1[i].x+(img_template1.cols/2), Est_pixel[i].y-cropy+max_pt1[i].y+(img_template1.rows/2)), 5, cv::Scalar(0, 255, 0), -1);//テンプレートの中心座標
            Matching_pixel_curr[matchC_curr].x=Est_pixel[i].x-cropx+max_pt1[i].x+(img_template1.cols/2);//マッチング中心座標(画像全体座標)
            Matching_pixel_curr[matchC_curr].y=Est_pixel[i].y-cropy+max_pt1[i].y+(img_template1.rows/2);//マッチング中心座標(画像全体座標)
            Matching_Templ_curr[matchC_curr]=img_template1;//マッチングしたテンプレート画像
            std::cout <<"マッチングの中心座標[matchC_curr="<<matchC_curr<<"]="<<Matching_pixel_curr[matchC_curr]<< std::endl;
            matchC_curr=matchC_curr+1;//マッチングの中心座標個数
            }
            //右側(xははみ出ない)
            else if(cropy<=Est_pixel[i].y&&Est_pixel[i].y<=480-cropy){
              std::cout << "マッチング:右側(xははみ出ない)["<<i<<"]"<<std::endl;
            cv::rectangle(img_dst, cv::Rect(Est_pixel[i].x-cropx+max_pt1[i].x, Est_pixel[i].y-cropy+max_pt1[i].y, img_template1.cols, img_template1.rows), cv::Scalar(0, 255, 0), 3);//白枠
            cv::circle(img_dst, cv::Point(Est_pixel[i].x-cropx+max_pt1[i].x+(img_template1.cols/2), Est_pixel[i].y-cropy+max_pt1[i].y+(img_template1.rows/2)), 5, cv::Scalar(0, 255, 0), -1);//テンプレートの中心座標
            Matching_pixel_curr[matchC_curr].x=Est_pixel[i].x-cropx+max_pt1[i].x+(img_template1.cols/2);//マッチング中心座標(画像全体座標)
            Matching_pixel_curr[matchC_curr].y=Est_pixel[i].y-cropy+max_pt1[i].y+(img_template1.rows/2);//マッチング中心座標(画像全体座標)
            Matching_Templ_curr[matchC_curr]=img_template1;//マッチングしたテンプレート画像
            std::cout <<"マッチングの中心座標[matchC_curr="<<matchC_curr<<"]="<<Matching_pixel_curr[matchC_curr]<< std::endl;
            matchC_curr=matchC_curr+1;//マッチングの中心座標個数
            }
            //右下(xとyどちらもはみでる)
            else if(480-cropy<Est_pixel[i].y&&Est_pixel[i].y<=480){
              std::cout << "マッチング:右下(xとyどちらもはみでる)["<<i<<"]"<<std::endl;
            cv::rectangle(img_dst, cv::Rect(Est_pixel[i].x-cropx+max_pt1[i].x, Est_pixel[i].y-cropy+max_pt1[i].y, img_template1.cols, img_template1.rows), cv::Scalar(0, 255, 0), 3);//白枠
            cv::circle(img_dst, cv::Point(Est_pixel[i].x-cropx+max_pt1[i].x+(img_template1.cols/2), Est_pixel[i].y-cropy+max_pt1[i].y+(img_template1.rows/2)), 5, cv::Scalar(0, 255, 0), -1);//テンプレートの中心座標
            Matching_pixel_curr[matchC_curr].x=Est_pixel[i].x-cropx+max_pt1[i].x+(img_template1.cols/2);//マッチング中心座標(画像全体座標)
            Matching_pixel_curr[matchC_curr].y=Est_pixel[i].y-cropy+max_pt1[i].y+(img_template1.rows/2);//マッチング中心座標(画像全体座標)
            Matching_Templ_curr[matchC_curr]=img_template1;//マッチングしたテンプレート画像
            std::cout <<"マッチングの中心座標[matchC_curr="<<matchC_curr<<"]="<<Matching_pixel_curr[matchC_curr]<< std::endl;
            matchC_curr=matchC_curr+1;//マッチングの中心座標個数
            }
          }
          //下側(yははみ出ない)
          else if(cropx<=Est_pixel[i].x&&Est_pixel[i].x<=640-cropx&&480-cropy<=Est_pixel[i].y&&Est_pixel[i].y<480){
            std::cout << "マッチング:下側(yははみ出ない)["<<i<<"]"<<std::endl;
            cv::rectangle(img_dst, cv::Rect(Est_pixel[i].x-cropx+max_pt1[i].x, Est_pixel[i].y-cropy+max_pt1[i].y, img_template1.cols, img_template1.rows), cv::Scalar(0, 255, 0), 3);//白枠
            cv::circle(img_dst, cv::Point(Est_pixel[i].x-cropx+max_pt1[i].x+(img_template1.cols/2), Est_pixel[i].y-cropy+max_pt1[i].y+(img_template1.rows/2)), 5, cv::Scalar(0, 255, 0), -1);//テンプレートの中心座標
            Matching_pixel_curr[matchC_curr].x=Est_pixel[i].x-cropx+max_pt1[i].x+(img_template1.cols/2);//マッチング中心座標(画像全体座標)
            Matching_pixel_curr[matchC_curr].y=Est_pixel[i].y-cropy+max_pt1[i].y+(img_template1.rows/2);//マッチング中心座標(画像全体座標)
            Matching_Templ_curr[matchC_curr]=img_template1;//マッチングしたテンプレート画像
            std::cout <<"マッチングの中心座標[matchC_curr="<<matchC_curr<<"]="<<Matching_pixel_curr[matchC_curr]<< std::endl;
            matchC_curr=matchC_curr+1;//マッチングの中心座標個数
          }
          else{
            std::cout << "画面外["<<i<<"]"<< std::endl;
          }
        }//if(min_val1[i]<max_val1[i]*0.05)→end(テンプレートマッチング)
      }//for(int i=0;i<depth_point_prev_ok;i++)→end (範囲予測+テンプレートマッチング)
      std::cout <<"matchC_curr="<<matchC_curr<< std::endl;
      Matching_pixel_curr.resize(matchC_curr);//Depth取得可能数でリサイズ(運動復元画像座標)
      MP_curr_pixel.resize(1000);//配列数初期設定(Depth取得可能なマッチング中心画像座標)
      MP_curr_camera.resize(1000);//配列数初期設定(Depth取得可能なマッチング中心カメラ座標)
      std::cout <<"test"<< std::endl;

      //マッチングしたテンプレートをマッチテンプレートとしてキープする
      //マッチテンプレートのDepthが取得不能な点を削除
      DMP_curr_ok=0;
      for (int i = 0; i < matchC_curr; i++) {
        DMP_curr[i] = depthimage.at<float>(cv::Point(Matching_pixel_curr[i].x,Matching_pixel_curr[i].y));//DMP_curr=Depth_Matching_pixel_curr
        //Depthが取得できない特徴点を削除する+Depthの外れ値を除く
        if(DMP_curr[i]>0.001&&DMP_curr[i]<10000){
          MP_curr_Templ[DMP_curr_ok] = Matching_Templ_curr[i];//Depth取得可能なマッチテンプレート
          MP_curr_pixel[DMP_curr_ok] = Matching_pixel_curr[i];//Depth取得可能なマッチング中心画像座標
          MP_curr_camera[DMP_curr_ok].x = DMP_curr[i] * ((Matching_pixel_curr[i].x - 324.473) / 615.337)/1000;//カメラ座標変換
          MP_curr_camera[DMP_curr_ok].y = DMP_curr[i] * ((Matching_pixel_curr[i].y - 241.696) / 615.458)/1000;
          MP_curr_camera[DMP_curr_ok].z = DMP_curr[i]/1000;
          std::cout <<"MP_curr_pixel["<<DMP_curr_ok<<"]="<<MP_curr_pixel[DMP_curr_ok]<< std::endl;
          std::cout <<"MP_curr_camera["<<DMP_curr_ok<<"]="<<MP_curr_camera[DMP_curr_ok]<< std::endl;
          //std::cout <<"MP_curr_Templ["<<DMP_curr_ok<<"].cols="<<MP_curr_Templ[DMP_curr_ok].cols<<"MP_curr_Templ["<<DMP_curr_ok<<"].rows="<<MP_curr_Templ[DMP_curr_ok].rows<< std::endl;//最初のフレーム
          DMP_curr_ok=DMP_curr_ok+1;//Depthが取得可能なマッチテンプレート数
          //DMP_prev_ok1=DMP_prev_ok1+1;//新規テンプレート数
        }
		  }
      std::cout <<"新規テンプレート数:DMP_curr_ok="<<DMP_curr_ok<< std::endl;
      //類似するテンプレートを削除する
      int notsimil=0;
      for (int i = 0; i < DMP_curr_ok; i++) {
        int similar=0;
        for (int j = i+1; j < DMP_curr_ok; j++) {
          length=sqrt((MP_curr_pixel[i].x-MP_curr_pixel[j].x)*(MP_curr_pixel[i].x-MP_curr_pixel[j].x)
            +(MP_curr_pixel[i].y-MP_curr_pixel[j].y)*(MP_curr_pixel[i].y-MP_curr_pixel[j].y));
          if(template_size*2>length){
            similar=similar+1;
          }
        }
        //類似するテンプレートを繰り上げ削除
        if(similar==0){
          MP_curr_Templ[notsimil]=MP_curr_Templ[i];
          MP_curr_pixel[notsimil]=MP_curr_pixel[i];
          MP_curr_camera[notsimil]=MP_curr_camera[i];
          notsimil=notsimil+1;
        }
      }
      DMP_curr_ok=notsimil;
      std::cout <<"新規テンプレート数(修正後):DMP_curr_ok="<<DMP_curr_ok<< std::endl;

      MP_curr_pixel.resize(DMP_curr_ok);//Depth取得可能数でリサイズ(マッチング中心画像座標)
      MP_curr_camera.resize(DMP_curr_ok);//Depth取得可能数でリサイズ(マッチング中心カメラ座標)

      Est_MP_point.resize(1000);//配列初期設定
      Est_MP_pixel.resize(1000);//配列初期設定

      //世界座標推定(マーカー観測時)------------------------------------------------------------------
      //テンプレートに一番近いマーカーの座標を使用する(カメラ観測座標で)
      //(マーカー観測が不可能な場合は3回目動作後に推定する)
      MP_curr_world.resize(1000);//初期設定
      if(markerIds.size() > 0){
        std::cout <<"マーカー観測可能"<< std::endl;
        double minlengh=1000000;
        int minMC;
        for (int i = 0; i < DMP_curr_ok; i++) {
          for(int j = 0; j < markerIds.size(); j++){
            length=sqrt((-MP_curr_camera[i].x+MC_point[j][0])*(-MP_curr_camera[i].x+MC_point[j][0])
            +(-MP_curr_camera[i].y+MC_point[j][1])*(-MP_curr_camera[i].y+MC_point[j][1])
            +(MP_curr_camera[i].z-MC_point[j][2])*(MP_curr_camera[i].z-MC_point[j][2]));
            if(minlengh>length){
              minMC=j;
              minlengh=length;
            }
          }
          MP_curr_world[i].x=MarkerW[markerIds.at(minMC)].at<float>(0)+(-MP_curr_camera[i].x+MC_point[markerIds.at(minMC)][0])*cos(-Est_RobotTH)-(MP_curr_camera[i].z-MC_point[markerIds.at(minMC)][2])*sin(-Est_RobotTH);
          MP_curr_world[i].y=MarkerW[markerIds.at(minMC)].at<float>(1)+(-MP_curr_camera[i].y+MC_point[markerIds.at(minMC)][1]);
          MP_curr_world[i].z=MarkerW[markerIds.at(minMC)].at<float>(2)+(MP_curr_camera[i].x-MC_point[markerIds.at(minMC)][0])*sin(-Est_RobotTH)+(MP_curr_camera[i].z-MC_point[markerIds.at(minMC)][2])*cos(-Est_RobotTH);

          std::cout <<"MarkerW[markerIds.at(minMC)].at<float>(0)="<<MarkerW[markerIds.at(minMC)].at<float>(0)<<", MarkerW[markerIds.at(minMC)].at<float>(1)="<<MarkerW[markerIds.at(minMC)].at<float>(1)<<", MarkerW[markerIds.at(minMC)].at<float>(2)="<<MarkerW[markerIds.at(minMC)].at<float>(2)<< std::endl;
          std::cout <<"MP_curr_world[i].x= "<<MP_curr_world[i].x<<", MP_curr_world[i].y="<<MP_curr_world[i].y<<", MP_curr_world[i].z="<<MP_curr_world[i].z<< std::endl;
          std::cout <<"MC_point[markerIds.at(minMC)][0]="<<MC_point[markerIds.at(minMC)][0]<<", MC_point[markerIds.at(minMC)][0]="<<MC_point[markerIds.at(minMC)][0]<<", MC_point[markerIds.at(minMC)][0]="<<MC_point[markerIds.at(minMC)][0]<< std::endl;
          std::cout <<"MP_curr_camera[i].x="<<MP_curr_camera[i].x<<", MP_curr_camera[i].y="<<MP_curr_camera[i].y<<", MP_curr_camera[i].z="<<MP_curr_camera[i].z<< std::endl;

          //MP_curr_world[i].x=MarkerW[markerIds.at(minMC)].at<float>(0)+MP_curr_camera[i].x-MC_point[markerIds.at(minMC)][0];
          //MP_curr_world[i].y=MarkerW[markerIds.at(minMC)].at<float>(1)+MP_curr_camera[i].y-MC_point[markerIds.at(minMC)][1];
          //MP_curr_world[i].z=MarkerW[markerIds.at(minMC)].at<float>(2)+MP_curr_camera[i].z-MC_point[markerIds.at(minMC)][2];
          std::cout <<"MC_point["<<markerIds.at(minMC)<<"]=("<<-MC_point[markerIds.at(minMC)][0]<<", "<<-MC_point[markerIds.at(minMC)][1]<<", "<<MC_point[markerIds.at(minMC)][2]<<")"<< std::endl;
          std::cout <<"MP_curr_camera["<<i<<"]="<<MP_curr_camera[i]<< std::endl;

          std::cout <<"MarkerW["<<markerIds.at(minMC)<<"]=("<<MarkerW[markerIds.at(minMC)].at<float>(0)<<", "<<MarkerW[markerIds.at(minMC)].at<float>(1)<<", "<<MarkerW[markerIds.at(minMC)].at<float>(2)<<")"<< std::endl;
          std::cout <<"MP_curr_world["<<i<<"]="<<MP_curr_world[i]<< std::endl;
          W_point<<"観測可能MC_point["<<markerIds.at(minMC)<<"]=("<<-MC_point[markerIds.at(minMC)][0]<<", "<<-MC_point[markerIds.at(minMC)][1]<<", "<<MC_point[markerIds.at(minMC)][2]<<")  ";
          W_point<<"観測可能MP_curr_camera["<<i<<"]= ("<<MP_curr_camera[i].x<<", "<<MP_curr_camera[i].y<<", "<<MP_curr_camera[i].z<<")\n";
          W_point<<"観測可能MarkerW["<<markerIds.at(minMC)<<"]=("<<MarkerW[markerIds.at(minMC)].at<float>(0)<<", "<<MarkerW[markerIds.at(minMC)].at<float>(1)<<", "<<MarkerW[markerIds.at(minMC)].at<float>(2)<<")  ";
          W_point<<"観測可能MP_curr_world["<<i<<"]= ("<<MP_curr_world[i].x<<", "<<MP_curr_world[i].y<<", "<<MP_curr_world[i].z<<")\n";
        }
        W_point<<"\n";
        MP_curr_world.resize(DMP_curr_ok);//Depth取得可能数でリサイズ(マッチング中心世界座標)
        for (int i = 0; i < DMP_curr_ok; i++) {
          std::cout <<"新規テンプレートの世界座標:MP_curr_world["<<i<<"]="<<MP_curr_world[i]<< std::endl;
        }
      }

      //3回目以降動作
      if(Tracking == true){
        std::cout <<"3回目以降動作-------------------------------------------------------"<< std::endl;
        //ロボットの動きから一つ前のマッチング座標の運動復元を行う(特徴点の状態方程式を使用)
        EST<<"確認用:DMP_prev_ok="<<DMP_prev_ok<<"\n";

        EST_MP_ok=0;
        for(int i=0;i<DMP_prev_ok;i++){
          std::cout <<"\n"<< std::endl;
          std::cout <<"MP_prev_world["<<i<<"]="<<MP_prev_world[i]<< std::endl;
          std::cout <<"MP_prev_camera["<<i<<"]="<<MP_prev_camera[i]<< std::endl;
          std::cout <<"MP_prev_pixel["<<i<<"]="<<MP_prev_pixel[i]<< std::endl;
          std::cout <<"robot_odometry.twist.twist.angular.z="<<robot_odometry.twist.twist.angular.z<< std::endl;
          std::cout <<"realsec="<<realsec<< std::endl;
          std::cout <<"Act_RobotV="<<Act_RobotV<< std::endl;

          Est_MP_point[i].x=MP_prev_camera[i].x+robot_odometry.twist.twist.angular.z*realsec*MP_prev_camera[i].z-Act_RobotV*sin(-Act_RobotTH)*realsec;
          Est_MP_point[i].y=MP_prev_camera[i].y;
          Est_MP_point[i].z=MP_prev_camera[i].z-robot_odometry.twist.twist.angular.z*realsec*MP_prev_camera[i].x-Act_RobotV*cos(-Act_RobotTH)*realsec;

          Est_MP_pixel[i].x=324.473+(Est_MP_point[i].x/Est_MP_point[i].z)*615.337;
          Est_MP_pixel[i].y=241.696+(Est_MP_point[i].y/Est_MP_point[i].z)*615.458;

          //cv::circle(img_dst, cv::Point(Est_pixel[i].x,Est_pixel[i].y), 6, Scalar(0,255,255), -1, cv::LINE_AA);//一つ前の画像の座標
          //一つ前のマッチ画像座標と次の推定画像座標
          cv::line(img_dst,cv::Point(MP_prev_pixel[i].x,MP_prev_pixel[i].y),cv::Point(Est_MP_pixel[i].x,Est_MP_pixel[i].y),cv::Scalar(0,0,255), 1, cv::LINE_AA);//180度(180*3)

          std::cout <<"Est_MP_point["<<i<<"]=\n"<<Est_MP_point[i]<< std::endl;//予測カメラ座標
          std::cout <<"Est_MP_pixel["<<i<<"]=\n"<<Est_MP_pixel[i]<< std::endl;//予測画像座標

          //求めた運動復元結果からテンプレートマッチングの予測範囲を作る(とりあえずタテヨコ2倍)
          //ここで予測点が画面外に行ったらそのテンプレートを削除する
          std::cout << "マッチング範囲限定クロッププログラム"<< std::endl;
          //MPcoix[i]=Est_MP_pixel[i].x+template_size;//予測範囲の中心座標
          //MPcoiy[i]=Est_MP_pixel[i].y+template_size;//予測範囲の中心座標
          //予測範囲が全て画面内の時
          if(cropx<=Est_MP_pixel[i].x&&Est_MP_pixel[i].x<=640-cropx&&cropy<=Est_MP_pixel[i].y&&Est_MP_pixel[i].y<=480-cropy){
            std::cout << "予測範囲が全て画面内の時["<<i<<"]"<< std::endl;
            cv::Rect roiEST(cv::Point(Est_MP_pixel[i].x-cropx,Est_MP_pixel[i].y-cropx), cv::Size(cropx*2, cropx*2));//線の中点を中心とした線の画像を切り取る
            EST_MP_scope[EST_MP_ok] = image(roiEST); // 切り出し画像
            MPcoix[EST_MP_ok]=Est_MP_pixel[i].x;
            MPcoiy[EST_MP_ok]=Est_MP_pixel[i].y;
            Est_MP_pixel[EST_MP_ok] = Est_MP_pixel[i];
            Est_MP_point[EST_MP_ok] = Est_MP_point[i];
            MP_prev_pixel[EST_MP_ok]=MP_prev_pixel[i];
            MP_prev_camera[EST_MP_ok]=MP_prev_camera[i];
            MP_prev_world[EST_MP_ok]=MP_prev_world[i];
            MP_prev_Templ[EST_MP_ok]=MP_prev_Templ[i];
            cv::rectangle(img_dst, roiEST,cv::Scalar(0, 50, 255), 2);//マッチング予測範囲
            cv::imshow("EST_MP_scope", EST_MP_scope[EST_MP_ok]);//黄色の特徴点を中心としたクロップ画像
            EST_MP_ok=EST_MP_ok+1;//予測範囲が画面内のテンプレート数
          }
           //左側
          else if(0<=Est_MP_pixel[i].x&&Est_MP_pixel[i].x<cropx){
            //左上(xとyどちらもはみでる)
            if(0<=Est_MP_pixel[i].y&&Est_MP_pixel[i].y<cropy){
              std::cout << "左上(xとyどちらもはみでる)["<<i<<"]"<<std::endl;
              cv::Rect roiEST(cv::Point(0,0), cv::Size(Est_MP_pixel[i].x+cropx, Est_MP_pixel[i].y+cropy));//線の中点を中心とした線の画像を切り取る
              EST_MP_scope[EST_MP_ok] = image(roiEST); // 切り出し画像
              MPcoix[EST_MP_ok]=Est_MP_pixel[i].x;
              MPcoiy[EST_MP_ok]=Est_MP_pixel[i].y;
              Est_MP_pixel[EST_MP_ok] = Est_MP_pixel[i];
              Est_MP_point[EST_MP_ok] = Est_MP_point[i];
              MP_prev_pixel[EST_MP_ok]=MP_prev_pixel[i];
              MP_prev_camera[EST_MP_ok]=MP_prev_camera[i];
              MP_prev_world[EST_MP_ok]=MP_prev_world[i];
              MP_prev_Templ[EST_MP_ok]=MP_prev_Templ[i];
              cv::rectangle(img_dst, roiEST,cv::Scalar(0, 50, 255), 2);//マッチング予測範囲
              EST_MP_ok=EST_MP_ok+1;//予測範囲が画面内のテンプレート数
            }
            //左側(xははみ出ない)
            else if(cropy<=Est_MP_pixel[i].y&&Est_MP_pixel[i].y<=480-cropy){
              std::cout << "左側(xははみ出ない)["<<i<<"]"<<std::endl;
              cv::Rect roiEST(cv::Point(0,Est_MP_pixel[i].y-cropy), cv::Size(Est_MP_pixel[i].x+cropx, cropx*2));//線の中点を中心とした線の画像を切り取る
              EST_MP_scope[EST_MP_ok] = image(roiEST); // 切り出し画像
              MPcoix[EST_MP_ok]=Est_MP_pixel[i].x;
              MPcoiy[EST_MP_ok]=Est_MP_pixel[i].y;
              Est_MP_pixel[EST_MP_ok] = Est_MP_pixel[i];
              Est_MP_point[EST_MP_ok] = Est_MP_point[i];
              MP_prev_pixel[EST_MP_ok]=MP_prev_pixel[i];
              MP_prev_camera[EST_MP_ok]=MP_prev_camera[i];
              MP_prev_world[EST_MP_ok]=MP_prev_world[i];
              MP_prev_Templ[EST_MP_ok]=MP_prev_Templ[i];
              cv::rectangle(img_dst, roiEST,cv::Scalar(0, 50, 255), 2);//マッチング予測範囲
              EST_MP_ok=EST_MP_ok+1;//予測範囲が画面内のテンプレート数
            }
            //左下(xとyどちらもはみでる)
            else if(480-cropy<Est_MP_pixel[i].y&&Est_MP_pixel[i].y<=480){
              std::cout << "左下(xとyどちらもはみでる)["<<i<<"]"<<std::endl;
              cv::Rect roiEST(cv::Point(0,Est_MP_pixel[i].y-cropy), cv::Size(Est_MP_pixel[i].x+cropx, 480-Est_MP_pixel[i].y+cropy));//線の中点を中心とした線の画像を切り取る
              EST_MP_scope[EST_MP_ok] = image(roiEST); // 切り出し画像
              MPcoix[EST_MP_ok]=Est_MP_pixel[i].x;
              MPcoiy[EST_MP_ok]=Est_MP_pixel[i].y;
              Est_MP_pixel[EST_MP_ok] = Est_MP_pixel[i];
              Est_MP_point[EST_MP_ok] = Est_MP_point[i];
              MP_prev_pixel[EST_MP_ok]=MP_prev_pixel[i];
              MP_prev_camera[EST_MP_ok]=MP_prev_camera[i];
              MP_prev_world[EST_MP_ok]=MP_prev_world[i];
              MP_prev_Templ[EST_MP_ok]=MP_prev_Templ[i];
              cv::rectangle(img_dst, roiEST,cv::Scalar(0, 50, 255), 2);//マッチング予測範囲
              EST_MP_ok=EST_MP_ok+1;//予測範囲が画面内のテンプレート数
            }
          }
          //上側(yははみ出る)
          else if(cropx<=Est_MP_pixel[i].x&&Est_MP_pixel[i].x<=640-cropx&&0<=Est_MP_pixel[i].y&&Est_MP_pixel[i].y<cropy){
            std::cout << "上側(yははみ出ない)["<<i<<"]"<<std::endl;
            cv::Rect roiEST(cv::Point(Est_MP_pixel[i].x-cropx,0), cv::Size(cropx*2, Est_MP_pixel[i].y+cropy));//線の中点を中心とした線の画像を切り取る
            EST_MP_scope[EST_MP_ok] = image(roiEST); // 切り出し画像
            MPcoix[EST_MP_ok]=Est_MP_pixel[i].x;
            MPcoiy[EST_MP_ok]=Est_MP_pixel[i].y;
            Est_MP_pixel[EST_MP_ok] = Est_MP_pixel[i];
            Est_MP_point[EST_MP_ok] = Est_MP_point[i];
            MP_prev_pixel[EST_MP_ok]=MP_prev_pixel[i];
            MP_prev_camera[EST_MP_ok]=MP_prev_camera[i];
            MP_prev_world[EST_MP_ok]=MP_prev_world[i];
            MP_prev_Templ[EST_MP_ok]=MP_prev_Templ[i];
            cv::rectangle(img_dst, roiEST,cv::Scalar(0, 50, 255), 2);//マッチング予測範囲
            EST_MP_ok=EST_MP_ok+1;//予測範囲が画面内のテンプレート数
          }
          //右側
          else if(640-cropx<Est_MP_pixel[i].x&&Est_MP_pixel[i].x<=640){
            //右上(xとyどちらもはみでる)
            if(0<=Est_MP_pixel[i].y&&Est_MP_pixel[i].y<cropy){
              std::cout << "右上(xとyどちらもはみでる)["<<i<<"]"<<std::endl;
              cv::Rect roiEST(cv::Point(Est_MP_pixel[i].x-cropx,0), cv::Size(640-Est_MP_pixel[i].x+cropx, Est_MP_pixel[i].y+cropy));//線の中点を中心とした線の画像を切り取る
              EST_MP_scope[EST_MP_ok] = image(roiEST); // 切り出し画像
              MPcoix[EST_MP_ok]=Est_MP_pixel[i].x;
              MPcoiy[EST_MP_ok]=Est_MP_pixel[i].y;
              Est_MP_pixel[EST_MP_ok] = Est_MP_pixel[i];
              Est_MP_point[EST_MP_ok] = Est_MP_point[i];
              MP_prev_pixel[EST_MP_ok]=MP_prev_pixel[i];
              MP_prev_camera[EST_MP_ok]=MP_prev_camera[i];
              MP_prev_world[EST_MP_ok]=MP_prev_world[i];
              MP_prev_Templ[EST_MP_ok]=MP_prev_Templ[i];
              cv::rectangle(img_dst, roiEST,cv::Scalar(0, 50, 255), 2);//マッチング予測範囲
              EST_MP_ok=EST_MP_ok+1;//予測範囲が画面内のテンプレート数
            }
            //右側(xははみ出ない)
            else if(cropy<=Est_MP_pixel[i].y&&Est_MP_pixel[i].y<=480-cropy){
              std::cout << "右側(xははみ出ない)["<<i<<"]"<<std::endl;
              cv::Rect roiEST(cv::Point(Est_MP_pixel[i].x-cropx,Est_MP_pixel[i].y-cropy), cv::Size(640-Est_MP_pixel[i].x+cropx, cropy*2));//線の中点を中心とした線の画像を切り取る
              EST_MP_scope[EST_MP_ok] = image(roiEST); // 切り出し画像
              MPcoix[EST_MP_ok]=Est_MP_pixel[i].x;
              MPcoiy[EST_MP_ok]=Est_MP_pixel[i].y;
              Est_MP_pixel[EST_MP_ok] = Est_MP_pixel[i];
              Est_MP_point[EST_MP_ok] = Est_MP_point[i];
              MP_prev_pixel[EST_MP_ok]=MP_prev_pixel[i];
              MP_prev_camera[EST_MP_ok]=MP_prev_camera[i];
              MP_prev_world[EST_MP_ok]=MP_prev_world[i];
              MP_prev_Templ[EST_MP_ok]=MP_prev_Templ[i];
              cv::rectangle(img_dst, roiEST,cv::Scalar(0, 50, 255), 2);//マッチング予測範囲
              EST_MP_ok=EST_MP_ok+1;//予測範囲が画面内のテンプレート数
            }
            //右下(xとyどちらもはみでる)
            else if(480-cropy<Est_MP_pixel[i].y&&Est_MP_pixel[i].y<=480){
              std::cout << "右下(xとyどちらもはみでる)["<<i<<"]"<<std::endl;
              cv::Rect roiEST(cv::Point(Est_MP_pixel[i].x-cropx,Est_MP_pixel[i].y-cropy), cv::Size(640-Est_MP_pixel[i].x+cropx, 480-Est_MP_pixel[i].y+cropy));//線の中点を中心とした線の画像を切り取る
              EST_MP_scope[EST_MP_ok] = image(roiEST); // 切り出し画像
              MPcoix[EST_MP_ok]=Est_MP_pixel[i].x;
              MPcoiy[EST_MP_ok]=Est_MP_pixel[i].y;
              Est_MP_pixel[EST_MP_ok] = Est_MP_pixel[i];
              Est_MP_point[EST_MP_ok] = Est_MP_point[i];
              MP_prev_pixel[EST_MP_ok]=MP_prev_pixel[i];
              MP_prev_camera[EST_MP_ok]=MP_prev_camera[i];
              MP_prev_world[EST_MP_ok]=MP_prev_world[i];
              MP_prev_Templ[EST_MP_ok]=MP_prev_Templ[i];
              cv::rectangle(img_dst, roiEST,cv::Scalar(0, 50, 255), 2);//マッチング予測範囲
              EST_MP_ok=EST_MP_ok+1;//予測範囲が画面内のテンプレート数
            }
          }
          //下側(yははみ出ない)
          else if(cropx<=Est_MP_pixel[i].x&&Est_MP_pixel[i].x<=640-cropx&&480-cropy<=Est_MP_pixel[i].y&&Est_MP_pixel[i].y<480){
            std::cout << "下側(yははみ出ない)["<<i<<"]"<<std::endl;
            cv::Rect roiEST(cv::Point(Est_MP_pixel[i].x-cropx,Est_MP_pixel[i].y-cropy), cv::Size(cropx*2, 480-Est_MP_pixel[i].y+cropy));//線の中点を中心とした線の画像を切り取る
            EST_MP_scope[EST_MP_ok] = image(roiEST); // 切り出し画像
            MPcoix[EST_MP_ok]=Est_MP_pixel[i].x;
            MPcoiy[EST_MP_ok]=Est_MP_pixel[i].y;
            Est_MP_pixel[EST_MP_ok] = Est_MP_pixel[i];
            Est_MP_point[EST_MP_ok] = Est_MP_point[i];
            MP_prev_pixel[EST_MP_ok]=MP_prev_pixel[i];
            MP_prev_camera[EST_MP_ok]=MP_prev_camera[i];
            MP_prev_world[EST_MP_ok]=MP_prev_world[i];
            MP_prev_Templ[EST_MP_ok]=MP_prev_Templ[i];
            cv::rectangle(img_dst, roiEST,cv::Scalar(0, 50, 255), 2);//マッチング予測範囲
            EST_MP_ok=EST_MP_ok+1;//予測範囲が画面内のテンプレート数
          }
          else{
            std::cout << "画面外["<<i<<"]"<< std::endl;
          }
        }//for(int i=0;i<DMP_prev_ok;i++)→end (範囲予測+テンプレートマッチング)
        std::cout << "EST_MP_ok="<<EST_MP_ok<< std::endl;
        Est_MP_pixel.resize(EST_MP_ok);//リサイズ
        Est_MP_point.resize(EST_MP_ok);//リサイズ
        MP_prev_pixel.resize(EST_MP_ok);//リサイズ
        MP_prev_camera.resize(EST_MP_ok);//リサイズ
        MP_prev_world.resize(EST_MP_ok);//リサイズ
        Matching_pixel_prev.resize(1000);//配列初期設定

        //テンプレートマッチングが原因っぽい
        //予測範囲に対しテンプレートマッチングを行う
        matchC_prev=0;
        for(int i=0;i<EST_MP_ok;i++){
          std::cout <<"\n"<< std::endl;
          std::cout << "テンプレートマッチングプログラム["<<i<<"]"<< std::endl;
          //MP_prev_Templ[i].copyTo(img_template1); // 切り出し画像(多分これが原因)
          img_template1 = MP_prev_Templ[i].clone();

          //テンプレートの拡大-----------------------------------------------
          double scale=sqrt(Est_MP_point[i].x*Est_MP_point[i].x+Est_MP_point[i].y*Est_MP_point[i].y+Est_MP_point[i].z*Est_MP_point[i].z)
          /sqrt(MP_prev_camera[i].x*MP_prev_camera[i].x+MP_prev_camera[i].y*MP_prev_camera[i].y+MP_prev_camera[i].z*MP_prev_camera[i].z);
          //cv::resize(img_template1, img_template1, cv::Size(), scale, scale);

          //cv::imshow("win_img_template1", MP_prev_Templ[i]);//黄色の特徴点を中心としたクロップ画像
          std::cout <<"img_template1.cols="<<img_template1.cols<<"img_template1.rows="<<img_template1.rows<< std::endl;//最初のフレーム
          std::cout <<"EST_MP_scope[i].cols="<<EST_MP_scope[i].cols<<"EST_MP_scope[i].rows="<<EST_MP_scope[i].rows<< std::endl;//最初のフレーム

          cv::Mat img_minmax1;
          // テンプレートマッチング(マッチングした際の座標を利用してマッチングテンプレートを作成し更新する)
          cv::matchTemplate(EST_MP_scope[i], img_template1, img_minmax1, cv::TM_CCOEFF_NORMED);//正規化相互相関(ZNCC)
          cv::minMaxLoc(img_minmax1, &min_val1[i], &max_val1[i], &min_pt1[i], &max_pt1[i]);
          std::cout << "min_val1(白)["<<i<<"]=" << min_val1[i] << std::endl;//一致度が上がると値が小さくなる
          std::cout << "max_val1(白)["<<i<<"]=" << max_val1[i] << std::endl;

          if(0.83<max_val1[i]){//最小値がしきい値以下なら表示
            //予測範囲が全て画面内の時
            if(cropx<=Est_MP_pixel[i].x&&Est_MP_pixel[i].x<=640-cropx&&cropy<=Est_MP_pixel[i].y&&Est_MP_pixel[i].y<=480-cropy){
              std::cout << "マッチング:全て画面内の時["<<i<<"]"<< std::endl;
              cv::rectangle(img_dst, cv::Rect(Est_MP_pixel[i].x-cropx+max_pt1[i].x, Est_MP_pixel[i].y-cropy+max_pt1[i].y, img_template1.cols, img_template1.rows), cv::Scalar(0, 255, 0), 3);//白枠
              cv::circle(img_dst, cv::Point(Est_MP_pixel[i].x-cropx+max_pt1[i].x+(img_template1.cols/2), Est_MP_pixel[i].y-cropy+max_pt1[i].y+(img_template1.rows/2)), 5, cv::Scalar(0, 255, 0), -1);//テンプレートの中心座標

              Matching_pixel_prev[matchC_prev].x=Est_MP_pixel[i].x-cropx+max_pt1[i].x+(img_template1.cols/2);//マッチング中心座標(画像全体座標)
              Matching_pixel_prev[matchC_prev].y=Est_MP_pixel[i].y-cropy+max_pt1[i].y+(img_template1.rows/2);//マッチング中心座標(画像全体座標)
              std::cout <<"マッチングの中心座標[matchC_prev="<<matchC_prev<<"]="<<Matching_pixel_prev[matchC_prev]<< std::endl;
              
              cv::Rect roi_match(cv::Point(Est_MP_pixel[i].x-cropx+max_pt1[i].x, Est_MP_pixel[i].y-cropy+max_pt1[i].y), cv::Size(template_size*2, template_size*2));//テンプレートの更新
              Matching_Templ_prev[matchC_prev] = image(roi_match); // 切り出し画像

             // Matching_Templ_prev[matchC_prev]=img_template1;//マッチングしたテンプレート画像

              MP_prev_pixel[matchC_prev]=MP_prev_pixel[i];
              MP_prev_camera[matchC_prev]=MP_prev_camera[i];
              MP_prev_world[matchC_prev]=MP_prev_world[i];
              MP_prev_Templ[matchC_prev]=MP_prev_Templ[i];
              matchC_prev=matchC_prev+1;//マッチングの中心座標個数
            }
            //左側
            else if(0<=Est_MP_pixel[i].x&&Est_MP_pixel[i].x<cropx){
              //左上(xとyどちらもはみでる)
              if(0<=Est_MP_pixel[i].y&&Est_MP_pixel[i].y<cropy){
                std::cout << "マッチング:左上(xとyどちらもはみでる)["<<i<<"]"<<std::endl;
                cv::rectangle(img_dst, cv::Rect(max_pt1[i].x, max_pt1[i].y, img_template1.cols, img_template1.rows), cv::Scalar(0, 255, 0), 3);//白枠
                cv::circle(img_dst, cv::Point(max_pt1[i].x+(img_template1.cols/2), max_pt1[i].y+(img_template1.rows/2)), 5, cv::Scalar(0, 255, 0), -1);//テンプレートの中心座標

                Matching_pixel_prev[matchC_prev].x=max_pt1[i].x+(img_template1.cols/2);//マッチング中心座標(画像全体座標)
                Matching_pixel_prev[matchC_prev].y=max_pt1[i].y+(img_template1.rows/2);//マッチング中心座標(画像全体座標)
                std::cout <<"マッチングの中心座標[matchC_prev="<<matchC_prev<<"]="<<Matching_pixel_prev[matchC_prev]<< std::endl;
                cv::Rect roi_match(cv::Point(max_pt1[i].x, max_pt1[i].y), cv::Size(template_size*2, template_size*2));//テンプレートの更新
                Matching_Templ_prev[matchC_prev] = image(roi_match); // 切り出し画像

                //Matching_Templ_prev[matchC_prev]=img_template1;//マッチングしたテンプレート画像
                MP_prev_pixel[matchC_prev]=MP_prev_pixel[i];
                MP_prev_camera[matchC_prev]=MP_prev_camera[i];
                MP_prev_world[matchC_prev]=MP_prev_world[i];
                MP_prev_Templ[matchC_prev]=MP_prev_Templ[i];
                matchC_prev=matchC_prev+1;//マッチングの中心座標個数
              }
              //左側(xははみ出ない)
              else if(cropy<=Est_MP_pixel[i].y&&Est_MP_pixel[i].y<=480-cropy){
                std::cout << "マッチング:左側(xははみ出ない)["<<i<<"]"<<std::endl;
                cv::rectangle(img_dst, cv::Rect(max_pt1[i].x, Est_MP_pixel[i].y-cropy+max_pt1[i].y, img_template1.cols, img_template1.rows), cv::Scalar(0, 255, 0), 3);//白枠
                cv::circle(img_dst, cv::Point(max_pt1[i].x+(img_template1.cols/2), Est_MP_pixel[i].y-cropy+max_pt1[i].y+(img_template1.rows/2)), 5, cv::Scalar(0, 255, 0), -1);//テンプレートの中心座標
    
                Matching_pixel_prev[matchC_prev].x=max_pt1[i].x+(img_template1.cols/2);//マッチング中心座標(画像全体座標)
                Matching_pixel_prev[matchC_prev].y=Est_MP_pixel[i].y-cropy+max_pt1[i].y+(img_template1.rows/2);//マッチング中心座標(画像全体座標)
                std::cout <<"マッチングの中心座標[matchC_prev="<<matchC_prev<<"]="<<Matching_pixel_prev[matchC_prev]<< std::endl;
                cv::Rect roi_match(cv::Point(max_pt1[i].x, Est_MP_pixel[i].y-cropy+max_pt1[i].y), cv::Size(template_size*2, template_size*2));//テンプレートの更新
                Matching_Templ_prev[matchC_prev] = image(roi_match); // 切り出し画像

                //Matching_Templ_prev[matchC_prev]=img_template1;//マッチングしたテンプレート画像
                MP_prev_pixel[matchC_prev]=MP_prev_pixel[i];
                MP_prev_camera[matchC_prev]=MP_prev_camera[i];
                MP_prev_world[matchC_prev]=MP_prev_world[i];
                MP_prev_Templ[matchC_prev]=MP_prev_Templ[i];
                matchC_prev=matchC_prev+1;//マッチングの中心座標個数
              }
              //左下(xとyどちらもはみでる)
              else if(480-cropy<Est_MP_pixel[i].y&&Est_MP_pixel[i].y<=480){
                std::cout << "マッチング:左下(xとyどちらもはみでる)["<<i<<"]"<<std::endl;
                cv::rectangle(img_dst, cv::Rect(max_pt1[i].x, Est_MP_pixel[i].y-cropy+max_pt1[i].y, img_template1.cols, img_template1.rows), cv::Scalar(0, 255, 0), 3);//白枠
                cv::circle(img_dst, cv::Point(max_pt1[i].x+(img_template1.cols/2), Est_MP_pixel[i].y-cropy+max_pt1[i].y+(img_template1.rows/2)), 5, cv::Scalar(0, 255, 0), -1);//テンプレートの中心座標

                Matching_pixel_prev[matchC_prev].x=max_pt1[i].x+(img_template1.cols/2);//マッチング中心座標(画像全体座標)
                Matching_pixel_prev[matchC_prev].y=Est_MP_pixel[i].y-cropy+max_pt1[i].y+(img_template1.rows/2);//マッチング中心座標(画像全体座標)
                std::cout <<"マッチングの中心座標[matchC_prev="<<matchC_prev<<"]="<<Matching_pixel_prev[matchC_prev]<< std::endl;
                cv::Rect roi_match(cv::Point(max_pt1[i].x, Est_MP_pixel[i].y-cropy+max_pt1[i].y), cv::Size(template_size*2, template_size*2));//テンプレートの更新
                Matching_Templ_prev[matchC_prev] = image(roi_match); // 切り出し画像

                //Matching_Templ_prev[matchC_prev]=img_template1;//マッチングしたテンプレート画像
                MP_prev_pixel[matchC_prev]=MP_prev_pixel[i];
                MP_prev_camera[matchC_prev]=MP_prev_camera[i];
                MP_prev_world[matchC_prev]=MP_prev_world[i];
                MP_prev_Templ[matchC_prev]=MP_prev_Templ[i];
                matchC_prev=matchC_prev+1;//マッチングの中心座標個数
              }
            }
            //上側(yははみ出る)
            else if(cropx<=Est_MP_pixel[i].x&&Est_MP_pixel[i].x<=640-cropx&&0<=Est_MP_pixel[i].y&&Est_MP_pixel[i].y<cropy){
              std::cout << "マッチング:上側["<<i<<"](yははみ出る)"<<std::endl;
              cv::rectangle(img_dst, cv::Rect(Est_MP_pixel[i].x-cropx+max_pt1[i].x, max_pt1[i].y, img_template1.cols, img_template1.rows), cv::Scalar(0, 255, 0), 3);//白枠
              cv::circle(img_dst, cv::Point(Est_MP_pixel[i].x-cropx+max_pt1[i].x+(img_template1.cols/2), max_pt1[i].y+(img_template1.rows/2)), 5, cv::Scalar(0, 255, 0), -1);//テンプレートの中心座標

              Matching_pixel_prev[matchC_prev].x=Est_MP_pixel[i].x-cropx+max_pt1[i].x+(img_template1.cols/2);//マッチング中心座標(画像全体座標)
              Matching_pixel_prev[matchC_prev].y=max_pt1[i].y+(img_template1.rows/2);//マッチング中心座標(画像全体座標)
              std::cout <<"マッチングの中心座標[matchC_prev="<<matchC_prev<<"]="<<Matching_pixel_prev[matchC_prev]<< std::endl;
              cv::Rect roi_match(cv::Point(Est_MP_pixel[i].x-cropx+max_pt1[i].x,max_pt1[i].y), cv::Size(img_template1.cols, img_template1.rows));//テンプレートの更新
              Matching_Templ_prev[matchC_prev] = image(roi_match); // 切り出し画像

              //Matching_Templ_prev[matchC_prev]=img_template1;//マッチングしたテンプレート画像
              MP_prev_pixel[matchC_prev]=MP_prev_pixel[i];
              MP_prev_camera[matchC_prev]=MP_prev_camera[i];
              MP_prev_world[matchC_prev]=MP_prev_world[i];
              MP_prev_Templ[matchC_prev]=MP_prev_Templ[i];
              matchC_prev=matchC_prev+1;//マッチングの中心座標個数
            }
            //右側
            else if(640-cropx<Est_MP_pixel[i].x&&Est_MP_pixel[i].x<=640){
              //右上(xとyどちらもはみでる)
              if(0<=Est_MP_pixel[i].y&&Est_MP_pixel[i].y<cropy){
                std::cout << "マッチング:右上(xとyどちらもはみでる)["<<i<<"]"<<std::endl;
                cv::rectangle(img_dst, cv::Rect(Est_MP_pixel[i].x-cropx+max_pt1[i].x, max_pt1[i].y, img_template1.cols, img_template1.rows), cv::Scalar(0, 255, 0), 3);//白枠
                cv::circle(img_dst, cv::Point(Est_MP_pixel[i].x-cropx+max_pt1[i].x+(img_template1.cols/2), max_pt1[i].y+(img_template1.rows/2)), 5, cv::Scalar(0, 255, 0), -1);//テンプレートの中心座標

                Matching_pixel_prev[matchC_prev].x=Est_MP_pixel[i].x-cropx+max_pt1[i].x+(img_template1.cols/2);//マッチング中心座標(画像全体座標)
                Matching_pixel_prev[matchC_prev].y=max_pt1[i].y+(img_template1.rows/2);//マッチング中心座標(画像全体座標)
                std::cout <<"マッチングの中心座標[matchC_prev="<<matchC_prev<<"]="<<Matching_pixel_prev[matchC_prev]<< std::endl;
                cv::Rect roi_match(cv::Point(Est_MP_pixel[i].x-cropx+max_pt1[i].x, max_pt1[i].y), cv::Size(template_size*2, template_size*2));//テンプレートの更新
                Matching_Templ_prev[matchC_prev] = image(roi_match); // 切り出し画像

                //Matching_Templ_prev[matchC_prev]=img_template1;//マッチングしたテンプレート画像
                MP_prev_pixel[matchC_prev]=MP_prev_pixel[i];
                MP_prev_camera[matchC_prev]=MP_prev_camera[i];
                MP_prev_world[matchC_prev]=MP_prev_world[i];
                MP_prev_Templ[matchC_prev]=MP_prev_Templ[i];
                matchC_prev=matchC_prev+1;//マッチングの中心座標個数
              }
              //右側(xははみ出ない)
              else if(cropy<=Est_MP_pixel[i].y&&Est_MP_pixel[i].y<=480-cropy){
                std::cout << "マッチング:右側(xははみ出ない)["<<i<<"]"<<std::endl;
                cv::rectangle(img_dst, cv::Rect(Est_MP_pixel[i].x-cropx+max_pt1[i].x, Est_MP_pixel[i].y-cropy+max_pt1[i].y, img_template1.cols, img_template1.rows), cv::Scalar(0, 255, 0), 3);//白枠
                cv::circle(img_dst, cv::Point(Est_MP_pixel[i].x-cropx+max_pt1[i].x+(img_template1.cols/2), Est_MP_pixel[i].y-cropy+max_pt1[i].y+(img_template1.rows/2)), 5, cv::Scalar(0, 255, 0), -1);//テンプレートの中心座標

                Matching_pixel_prev[matchC_prev].x=Est_MP_pixel[i].x-cropx+max_pt1[i].x+(img_template1.cols/2);//マッチング中心座標(画像全体座標)
                Matching_pixel_prev[matchC_prev].y=Est_MP_pixel[i].y-cropy+max_pt1[i].y+(img_template1.rows/2);//マッチング中心座標(画像全体座標)
                std::cout <<"マッチングの中心座標[matchC_prev="<<matchC_prev<<"]="<<Matching_pixel_prev[matchC_prev]<< std::endl;
                cv::Rect roi_match(cv::Point(Matching_pixel_prev[matchC_prev].x-(img_template1.cols/2),Matching_pixel_prev[matchC_prev].y-(img_template1.rows/2)), cv::Size(template_size*2, template_size*2));//テンプレートの更新
                Matching_Templ_prev[matchC_prev] = image(roi_match); // 切り出し画像

                //Matching_Templ_prev[matchC_prev]=img_template1;//マッチングしたテンプレート画像
                MP_prev_pixel[matchC_prev]=MP_prev_pixel[i];
                MP_prev_camera[matchC_prev]=MP_prev_camera[i];
                MP_prev_world[matchC_prev]=MP_prev_world[i];
                MP_prev_Templ[matchC_prev]=MP_prev_Templ[i];
                matchC_prev=matchC_prev+1;//マッチングの中心座標個数
              }
              //右下(xとyどちらもはみでる)
              else if(480-cropy<Est_MP_pixel[i].y&&Est_MP_pixel[i].y<=480){
                std::cout << "マッチング:右下(xとyどちらもはみでる)["<<i<<"]"<<std::endl;
                cv::rectangle(img_dst, cv::Rect(Est_MP_pixel[i].x-cropx+max_pt1[i].x, Est_MP_pixel[i].y-cropy+max_pt1[i].y, img_template1.cols, img_template1.rows), cv::Scalar(0, 255, 0), 3);//白枠
                cv::circle(img_dst, cv::Point(Est_MP_pixel[i].x-cropx+max_pt1[i].x+(img_template1.cols/2), Est_MP_pixel[i].y-cropy+max_pt1[i].y+(img_template1.rows/2)), 5, cv::Scalar(0, 255, 0), -1);//テンプレートの中心座標

                Matching_pixel_prev[matchC_prev].x=Est_MP_pixel[i].x-cropx+max_pt1[i].x+(img_template1.cols/2);//マッチング中心座標(画像全体座標)
                Matching_pixel_prev[matchC_prev].y=Est_MP_pixel[i].y-cropy+max_pt1[i].y+(img_template1.rows/2);//マッチング中心座標(画像全体座標)
                std::cout <<"マッチングの中心座標[matchC_prev="<<matchC_prev<<"]="<<Matching_pixel_prev[matchC_prev]<< std::endl;
                cv::Rect roi_match(cv::Point(Matching_pixel_prev[matchC_prev].x-(img_template1.cols/2),Matching_pixel_prev[matchC_prev].y-(img_template1.rows/2)), cv::Size(template_size*2, template_size*2));//テンプレートの更新
                Matching_Templ_prev[matchC_prev] = image(roi_match); // 切り出し画像

                //Matching_Templ_prev[matchC_prev]=img_template1;//マッチングしたテンプレート画像
                MP_prev_pixel[matchC_prev]=MP_prev_pixel[i];
                MP_prev_camera[matchC_prev]=MP_prev_camera[i];
                MP_prev_world[matchC_prev]=MP_prev_world[i];
                MP_prev_Templ[matchC_prev]=MP_prev_Templ[i];
                matchC_prev=matchC_prev+1;//マッチングの中心座標個数
              }
            }
            //下側(yははみ出ない)
            else if(cropx<=Est_MP_pixel[i].x&&Est_MP_pixel[i].x<=640-cropx&&480-cropy<=Est_MP_pixel[i].y&&Est_MP_pixel[i].y<480){
              std::cout << "マッチング:下側(yははみ出ない)["<<i<<"]"<<std::endl;
              cv::rectangle(img_dst, cv::Rect(Est_MP_pixel[i].x-cropx+max_pt1[i].x, Est_MP_pixel[i].y-cropy+max_pt1[i].y, img_template1.cols, img_template1.rows), cv::Scalar(0, 255, 0), 3);//白枠
              cv::circle(img_dst, cv::Point(Est_MP_pixel[i].x-cropx+max_pt1[i].x+(img_template1.cols/2), Est_MP_pixel[i].y-cropy+max_pt1[i].y+(img_template1.rows/2)), 5, cv::Scalar(0, 255, 0), -1);//テンプレートの中心座標

              Matching_pixel_prev[matchC_prev].x=Est_MP_pixel[i].x-cropx+max_pt1[i].x+(img_template1.cols/2);//マッチング中心座標(画像全体座標)
              Matching_pixel_prev[matchC_prev].y=Est_MP_pixel[i].y-cropy+max_pt1[i].y+(img_template1.rows/2);//マッチング中心座標(画像全体座標)
              std::cout <<"マッチングの中心座標[matchC_prev="<<matchC_prev<<"]="<<Matching_pixel_prev[matchC_prev]<< std::endl;
              cv::Rect roi_match(cv::Point(Matching_pixel_prev[matchC_prev].x-(img_template1.cols/2),Matching_pixel_prev[matchC_prev].y-(img_template1.rows/2)), cv::Size(template_size*2, template_size*2));//テンプレートの更新
              Matching_Templ_prev[matchC_prev] = image(roi_match); // 切り出し画像

              //Matching_Templ_prev[matchC_prev]=img_template1;//マッチングしたテンプレート画像
              MP_prev_pixel[matchC_prev]=MP_prev_pixel[i];
              MP_prev_camera[matchC_prev]=MP_prev_camera[i];
              MP_prev_world[matchC_prev]=MP_prev_world[i];
              MP_prev_Templ[matchC_prev]=MP_prev_Templ[i];
              matchC_prev=matchC_prev+1;//マッチングの中心座標個数
            }
            else{
              std::cout << "画面外["<<i<<"]"<< std::endl;
            }
          }//if(min_val1[i]<max_val1[i]*0.05)→end(テンプレートマッチング)
          else{
              std::cout << "マッチしない["<<i<<"]((0.99<max_val1["<<i<<"])="<<0.99<<"<"<<max_val1[i]<< std::endl;
          }
        }//for(int i=0;i<EST_MP_ok;i++)→end (範囲予測+テンプレートマッチング)

        std::cout <<"matchC_prev="<<matchC_prev<< std::endl;
        Matching_pixel_prev.resize(matchC_prev);//Depth取得可能数でリサイズ(運動復元画像座標)
        MP_prev_pixel.resize(matchC_prev);//リサイズ
        MP_prev_camera.resize(matchC_prev);//リサイズ
        MP_prev_world.resize(matchC_prev);//リサイズ
        MP_curr2_pixel.resize(1000);//配列数初期設定(Depth取得可能なマッチング中心画像座標)
        MP_curr2_camera.resize(1000);//配列数初期設定(Depth取得可能なマッチング中心カメラ座標)
        MP_curr2_world.resize(1000);//配列数初期設定(Depth取得可能なマッチング中心カメラ座標)
        std::cout <<"test"<< std::endl;

        //マッチングしたテンプレートをマッチテンプレートとしてキープする
        //マッチテンプレートのDepthが取得不能な点を削除
        DMP_curr2_ok=0;
        for (int i = 0; i < matchC_prev; i++) {
          DMP_curr2[i] = depthimage.at<float>(cv::Point(Matching_pixel_prev[i].x,Matching_pixel_prev[i].y));//DMP_prev=Depth_Matching_pixel_prev
          //Depthが取得できない特徴点を削除する+Depthの外れ値を除く
          if(DMP_curr2[i]>0.001&&DMP_curr2[i]<10000){
            MP_curr2_Templ[DMP_curr2_ok] = Matching_Templ_prev[i];//Depth取得可能なマッチテンプレート
            MP_curr2_pixel[DMP_curr2_ok] = Matching_pixel_prev[i];//Depth取得可能なマッチング中心画像座標
            MP_curr2_camera[DMP_curr2_ok].x = DMP_curr2[i] * ((Matching_pixel_prev[i].x - 324.473) / 615.337)/1000;//カメラ座標変換
            MP_curr2_camera[DMP_curr2_ok].y = DMP_curr2[i] * ((Matching_pixel_prev[i].y - 241.696) / 615.458)/1000;
            MP_curr2_camera[DMP_curr2_ok].z = DMP_curr2[i]/1000;
            MP_curr2_world[DMP_curr2_ok] = MP_prev_world[i];//Depth取得可能なマッチング中心世界座標
            std::cout <<"MP_curr2_pixel["<<DMP_curr2_ok<<"]="<<MP_curr2_pixel[DMP_curr2_ok]<< std::endl;
            std::cout <<"MP_curr2_camera["<<DMP_curr2_ok<<"]="<<MP_curr2_camera[DMP_curr2_ok]<< std::endl;
            std::cout <<"MP_curr2_world["<<DMP_curr2_ok<<"]="<<MP_curr2_world[DMP_curr2_ok]<< std::endl;
            DMP_curr2_ok=DMP_curr2_ok+1;//Depthが取得可能な全マッチテンプレート数
            //DMP_prev_ok3=DMP_prev_ok3+1;//保持テンプレート数
          }
		    }
/*        std::cout <<"新規テンプレート数(2回目動作結果):DMP_curr_ok="<<DMP_curr_ok<< std::endl;
        for (int i = 0; i < DMP_curr_ok; i++) {
          std::cout <<"MP_curr_pixel["<<i<<"]="<<MP_curr_pixel[i]<< std::endl;
          //std::cout <<"MP_curr_camera["<<i<<"]="<<MP_curr_camera[i]<< std::endl;
        }
        std::cout <<"3回目テンプレート数:DMP_curr2_ok="<<DMP_curr2_ok<< std::endl;
        for (int i = 0; i < DMP_curr2_ok; i++) {
          std::cout <<"MP_curr2_pixel["<<i<<"]="<<MP_curr2_pixel[i]<< std::endl;
          //std::cout <<"MP_curr2_camera["<<i<<"]="<<MP_curr2_camera[i]<< std::endl;
        }
        std::cout <<"一つ前のテンプレート数(調整済み):matchC_prev="<<matchC_prev<< std::endl;
        for (int i = 0; i < matchC_prev; i++) {
          std::cout <<"MP_prev_pixel["<<i<<"]="<<MP_prev_pixel[i]<< std::endl;
        }
        cv::line(img_dst2,cv::Point(MP_curr2_pixel[0].x,MP_curr2_pixel[0].y),cv::Point(MP_prev_pixel[0].x,MP_prev_pixel[0].y),cv::Scalar(0,0,0), 2, cv::LINE_AA);
        cv::line(img_dst2,cv::Point(MP_curr2_pixel[1].x,MP_curr2_pixel[1].y),cv::Point(MP_prev_pixel[1].x,MP_prev_pixel[1].y),cv::Scalar(0,0,255), 2, cv::LINE_AA);
*/
        //世界座標推定(マーカー観測不能時)------------------------------------------------------
        //マーカーが観測できない場合(観測可能な場合は2回目動作で世界座標を推定する)
        //新規テンプレートに一番近いキープテンプレートの世界座標を使用する(カメラ観測座標で)
        MP_curr_world.resize(1000);//初期設定
        if(markerIds.size() <= 0){
          std::cout <<"マーカー観測不能時"<< std::endl;
          double minlengh=1000000;
          int minMC;
          for (int i = 0; i < DMP_curr_ok; i++) {
            for(int j = 0; j < DMP_curr2_ok; j++){
              length=sqrt((-MP_curr_camera[i].x+MP_curr2_camera[j].x)*(-MP_curr_camera[i].x+MP_curr2_camera[j].x)
              +(-MP_curr_camera[i].y+MP_curr2_camera[j].y)*(-MP_curr_camera[i].y+MP_curr2_camera[j].y)
              +(MP_curr_camera[i].z-MP_curr2_camera[j].z)*(MP_curr_camera[i].z-MP_curr2_camera[j].z));
              if(minlengh>length){
                minMC=j;
                minlengh=length;
              }
            }
            MP_curr_world[i].x=MP_curr2_world[minMC].x+(-MP_curr_camera[i].x+MP_curr2_camera[minMC].x)*cos(-Est_RobotTH)-(MP_curr_camera[i].z-MP_curr2_camera[minMC].z)*sin(-Est_RobotTH);
            MP_curr_world[i].y=MP_curr2_world[minMC].y+(-MP_curr_camera[i].y+MP_curr2_camera[minMC].y);
            MP_curr_world[i].z=MP_curr2_world[minMC].z+(MP_curr_camera[i].x-MP_curr2_camera[minMC].x)*sin(-Est_RobotTH)+(MP_curr_camera[i].z-MP_curr2_camera[minMC].z)*cos(-Est_RobotTH);

            //MP_curr_world[i].x=MP_curr2_world[minMC].x+MP_curr_camera[i].x-MP_curr2_camera[minMC].x;
            //MP_curr_world[i].y=MP_curr2_world[minMC].y+MP_curr_camera[i].y-MP_curr2_camera[minMC].y;
            //MP_curr_world[i].z=MP_curr2_world[minMC].z+MP_curr_camera[i].z-MP_curr2_camera[minMC].z;
            std::cout <<"MP_curr2_camera["<<minMC<<"]= "<<MP_curr2_camera[minMC]<< std::endl;
            std::cout <<"MP_curr_camera["<<i<<"]= "<<MP_curr_camera[i]<< std::endl;
            std::cout <<"MP_curr2_world["<<minMC<<"]= "<<MP_curr2_world[minMC]<< std::endl;
            std::cout <<"MP_curr_world["<<i<<"]=  "<<MP_curr_world[i]<< std::endl;
            W_point<<"観測不可MP_curr2_camera["<<minMC<<"]= ("<<MP_curr2_camera[minMC].x<<", "<<MP_curr2_camera[minMC].y<<", "<<MP_curr2_camera[minMC].z<<")  ";
            W_point<<"観測不可MP_curr_camera["<<i<<"]= ("<<MP_curr_camera[i].x<<", "<<MP_curr_camera[i].y<<", "<<MP_curr_camera[i].z<<")\n";
            W_point<<"観測不可MP_curr2_world["<<minMC<<"]= ("<<MP_curr2_world[minMC].x<<", "<<MP_curr2_world[minMC].y<<", "<<MP_curr2_world[minMC].z<<")  ";
            W_point<<"観測不可MP_curr_world["<<i<<"]= ("<<MP_curr_world[i].x<<", "<<MP_curr_world[i].y<<", "<<MP_curr_world[i].z<<")\n";
          }
          W_point<<"\n";
          MP_curr_world.resize(DMP_curr_ok);//Depth取得可能数でリサイズ(マッチング中心世界座標)
          for (int i = 0; i < DMP_curr_ok; i++) {
            std::cout <<"新規テンプレートの世界座標:MP_curr_world["<<i<<"]="<<MP_curr_world[i]<< std::endl;
            W_point2 <<"新規テンプレートの世界座標:MP_curr_world["<<i<<"]="<<MP_curr_world[i]<<"\n";
          }
        }//if(markerIds.size() <= 0)→END

        //新規テンプレート追加動作-------------------------------------------------------------------------------------------------------------
        //同じ所に作成されたテンプレート削除する(画像座標を使って比較)
        //3回目のマッチング結果(Prev)と2回目に作成した新規テンプレート(curr)を比較する
        //新規テンプレートと旧テンプレートを比較して全ての旧テンプレートと距離がテンプレートサイズ以上離れていたら追加する
        //距離がテンプレートサイズ以内ならばPrev(旧テンプレート)を削除し新規テンプレートを追加する
        //複数重なる場合は最後
        for (int i = 0; i < DMP_curr2_ok; i++) {
          double remave=0,minlengh=1000000;
          int minj;
          std::cout <<"\n"<< std::endl;
          for (int j = 0; j < DMP_curr_ok; j++) {
            length=sqrt((MP_curr2_pixel[i].x-MP_curr_pixel[j].x)*(MP_curr2_pixel[i].x-MP_curr_pixel[j].x)
                        +(MP_curr2_pixel[i].y-MP_curr_pixel[j].y)*(MP_curr2_pixel[i].y-MP_curr_pixel[j].y));

            std::cout <<"length="<<length<< std::endl;
            std::cout <<"template_size="<<template_size<< std::endl;
            std::cout <<"更新前:MP_curr_pixel["<<j<<"]="<<MP_curr_pixel[j]<< std::endl;
            std::cout <<"更新前:MP_curr2_pixel["<<i<<"]="<<MP_curr2_pixel[i]<< std::endl;
            
            //更新動作(テンプレートのかぶりがあるとき→最も距離が近いテンプレートで更新する)
            if(length<template_size*2){
              remave=1;
              if(minlengh>length){
                minj=j;//最も距離が近いテンプレートの配列を保存
                minlengh=length;//最小距離の更新
              }
            }
          }//for (int j = 0; j < DMP_curr_ok; j++)→end
          //更新動作(テンプレートのかぶりが存在する時)
          if(remave==1){
            std::cout <<"更新動作------------------------------------------------------------------------"<<std::endl;
            MP_curr2_Templ[i] = MP_curr_Templ[minj];//Depth取得可能なマッチテンプレート
            MP_curr2_pixel[i] = MP_curr_pixel[minj];//Depth取得可能なマッチング中心画像座標
            MP_curr2_camera[i] = MP_curr_camera[minj];
            MP_curr2_world[i] = MP_curr_world[minj];
            //更新したCurrを削除する(重複防止)
            int j,k;
            for ( j = k = 0; j < DMP_curr_ok; j++) {
              if(j!=minj){
                MP_curr_Templ[k]=MP_curr_Templ[j];
                MP_curr_pixel[k]=MP_curr_pixel[j];
                MP_curr_camera[k]=MP_curr_camera[j];
                MP_curr_world[k++]=MP_curr_world[j];
              }
            }
            DMP_curr_ok=k;
          }
        }//for (int i = 0; i < DMP_curr2_ok; i++) →end

        std::cout <<"DMP_prev_ok="<<DMP_prev_ok<< std::endl;
        if(DMP_prev_ok>0||markerIds.size()>0){//1つ前のマッチング要素が存在するもしくはマーカー観測可能時(マッチング追跡可能時)
          //追加動作(残ったCurrをCurr2に追加する→残ったcurrには更新要素が無いためテンプレートがかぶってない)
          for (int j = 0; j < DMP_curr_ok; j++) {
            MP_curr2_Templ[DMP_curr2_ok] = MP_curr_Templ[j];//Depth取得可能なマッチテンプレート
            MP_curr2_pixel[DMP_curr2_ok] = MP_curr_pixel[j];//Depth取得可能なマッチング中心画像座標
            MP_curr2_camera[DMP_curr2_ok] = MP_curr_camera[j];
            MP_curr2_world[DMP_curr2_ok] = MP_curr_world[j];
            std::cout <<"追加動作:MP_curr2_pixel["<<DMP_curr2_ok<<"]="<<MP_curr2_pixel[DMP_curr2_ok]<< std::endl;
            //std::cout <<"追加動作:MP_curr2_camera["<<DMP_curr2_ok<<"]="<<MP_curr2_camera[DMP_curr2_ok]<< std::endl;
            DMP_curr2_ok=DMP_curr2_ok+1;//Depthが取得可能な全マッチテンプレート数
          }

          MP_curr2_pixel.resize(DMP_curr2_ok);//Depth取得可能数でリサイズ(マッチング中心画像座標)
          MP_curr2_camera.resize(DMP_curr2_ok);//Depth取得可能数でリサイズ(マッチング中心カメラ座標)
          MP_curr2_world.resize(DMP_curr2_ok);//Depth取得可能数でリサイズ(マッチング中心世界座標)
          std::cout <<"全テンプレート数:DMP_prev_ok="<<DMP_curr2_ok<< std::endl;

          for (int i = 0; i < DMP_curr2_ok; i++) {
            std::cout <<"MP_curr2_pixel["<<i<<"]="<<MP_curr2_pixel[i]<< std::endl;
            //テンプレートまでの距離と角度を求める(観測値)
            CameraLMP[0]=sqrt((-MP_curr2_camera[i].x*-MP_curr2_camera[i].x)+(MP_curr2_camera[i].z*MP_curr2_camera[i].z));
            CameraLMP[1]=atan2(-MP_curr2_camera[i].x,MP_curr2_camera[i].z);
            Zu_P[i] = (cv::Mat_<double>(2,1) <<
              CameraLMP[0],
              CameraLMP[1]);
          }
          //2回目動作追加後もデータ数が0だった場合、一つ前のマッチングデータを世界座標推定用配列に代入する
          //このデータ数が0の時はマッチング追跡ができないため、世界座標データが消えてしまう
          //なので一つ前のマッチングデータと次のオドメトリー情報を利用して、推定世界座標を求める
          //そしてその推定した世界座標を利用して自己位置推定を行なっていく(マーカーが観測されるまで)
          if(DMP_curr2_ok<=0){
            cv::swap(MP_keep_world, MP_Est_prev_world);//一つ前のマッチ世界座標→推定用世界座標
            cv::swap(MP_keep_camera, MP_Est_prev_camera);//一つ前のマッチカメラ座標→推定用カメラ座標
            EST<<"推定用配列代入:DMP_curr2_ok="<<DMP_curr2_ok<<"\n";
            EST<<"推定用配列代入:DMP_prev_ok="<<DMP_prev_ok<<"\n";
            EST<<"推定用配列代入:MP_keep_world.size()="<<MP_keep_world.size()<<"\n";
            EST<<"推定用配列代入:MP_Est_prev_world.size()="<<MP_Est_prev_world.size()<<"\n";
          }
        }

        //一つ前のマッチング要素がなくマッチング追跡不可能なとき
        /*else{
          std::cout <<"マッチング追跡不可能555555555555555555555555555555555555555555555555555555555555555555555555555555"<< std::endl;
          EST<<"マッチング追跡不可能"<<"\n";
          EST<<"MP_Est_prev_world.size()="<<MP_Est_prev_world.size()<<"\n";

          //追加動作
          for (int j = 0; j < DMP_curr_ok; j++) {
            MP_curr2_Templ[DMP_curr2_ok] = MP_curr_Templ[j];//Depth取得可能なマッチテンプレート
            MP_curr2_pixel[DMP_curr2_ok] = MP_curr_pixel[j];//Depth取得可能なマッチング中心画像座標
            MP_curr2_camera[DMP_curr2_ok] = MP_curr_camera[j];
            DMP_curr2_ok=DMP_curr2_ok+1;//Depthが取得可能な全マッチテンプレート数
          }
          MP_curr2_pixel.resize(DMP_curr2_ok);//Depth取得可能数でリサイズ(マッチング中心画像座標)
          MP_curr2_camera.resize(DMP_curr2_ok);//Depth取得可能数でリサイズ(マッチング中心カメラ座標)
          MP_Est_curr_world.resize(1000);//初期設定
          MP_Est_curr_camera.resize(1000);//初期設定
          //一つ前のマッチングデータと次のオドメトリー情報を利用して、推定世界座標と推定カメラ座標を求める
          for (int i = 0; i < MP_Est_prev_world.size(); i++) {
            //MP_Est_curr_world[i].x=MP_Est_prev_world[i].x-robot_odometry.twist.twist.angular.z*realsec*-MP_Est_prev_world[i].z+Act_RobotV*sin(-Act_RobotTH)*realsec;
            //MP_Est_curr_world[i].y=MP_Est_prev_world[i].y;
            //MP_Est_curr_world[i].z= MP_Est_prev_world[i].z-robot_odometry.twist.twist.angular.z*realsec*-MP_Est_prev_world[i].x+Act_RobotV*cos(-Act_RobotTH)*realsec;
            MP_Est_curr_world[i].x=MP_Est_prev_world[i].x;
            MP_Est_curr_world[i].y=MP_Est_prev_world[i].y;
            MP_Est_curr_world[i].z=MP_Est_prev_world[i].z;

            MP_Est_curr_camera[i].x=MP_Est_prev_camera[i].x+robot_odometry.twist.twist.angular.z*realsec*MP_Est_prev_camera[i].z-Act_RobotV*sin(-Act_RobotTH)*realsec;
            MP_Est_curr_camera[i].y=MP_Est_prev_camera[i].y;
            MP_Est_curr_camera[i].z=MP_Est_prev_camera[i].z-robot_odometry.twist.twist.angular.z*realsec*MP_Est_prev_camera[i].x-Act_RobotV*cos(-Act_RobotTH)*realsec;
            EST<<"MP_Est_prev_world["<<i<<"].x="<<MP_Est_prev_world[i].x<<",MP_Est_prev_world["<<i<<"].y="<<MP_Est_prev_world[i].y<<", MP_Est_prev_world["<<i<<"].z="<<MP_Est_prev_world[i].z<<"\n";
            EST<<"MP_Est_curr_world["<<i<<"].x="<<MP_Est_curr_world[i].x<<",MP_Est_curr_world["<<i<<"].y="<<MP_Est_curr_world[i].y<<", MP_Est_curr_world["<<i<<"].z="<<MP_Est_curr_world[i].z<<"\n";
            EST<<"-MP_Est_prev_camera["<<i<<"].x="<<-MP_Est_prev_camera[i].x<<",-MP_Est_prev_camera["<<i<<"].y="<<-MP_Est_prev_camera[i].y<<", MP_Est_prev_camera["<<i<<"].z="<<MP_Est_prev_camera[i].z<<"\n";
            EST<<"-MP_Est_curr_camera["<<i<<"].x="<<-MP_Est_curr_camera[i].x<<",-MP_Est_curr_camera["<<i<<"].y="<<-MP_Est_curr_camera[i].y<<", MP_Est_curr_camera["<<i<<"].z="<<MP_Est_curr_camera[i].z<<"\n";
          }
          MP_Est_curr_world.resize(MP_Est_prev_world.size());//初期設定
          MP_Est_curr_camera.resize(MP_Est_prev_world.size());//初期設定
          MP_curr2_world.resize(1000);//初期設定
          double minlengh=1000000;
          int minMC;
          for (int i = 0; i < DMP_curr2_ok; i++) {
            for(int j = 0; j < MP_Est_prev_world.size(); j++){
              length=sqrt((-MP_curr2_camera[i].x+MP_Est_curr_camera[j].x)*(-MP_curr2_camera[i].x+MP_Est_curr_camera[j].x)
                +(-MP_curr2_camera[i].y+MP_Est_curr_camera[j].y)*(-MP_curr2_camera[i].y+MP_Est_curr_camera[j].y)
                +(MP_curr2_camera[i].z-MP_Est_curr_camera[j].z)*(MP_curr2_camera[i].z-MP_Est_curr_camera[j].z));
              if(minlengh>length){
                minMC=j,minlengh=length;
              }
            }
            MP_curr2_world[i].x=MP_Est_curr_world[minMC].x+(-MP_curr2_camera[i].x+MP_Est_curr_camera[minMC].x)*cos(-Est_RobotTH)-(MP_curr2_camera[i].z-MP_Est_curr_camera[minMC].z)*sin(-Est_RobotTH);
            MP_curr2_world[i].y=MP_Est_curr_world[minMC].y+(-MP_curr2_camera[i].y+MP_Est_curr_camera[minMC].y);
            MP_curr2_world[i].z=MP_Est_curr_world[minMC].z+(MP_curr2_camera[i].x-MP_Est_curr_camera[minMC].x)*sin(-Est_RobotTH)+(MP_curr2_camera[i].z-MP_Est_curr_camera[minMC].z)*cos(-Est_RobotTH);

            std::cout <<"MP_Est_curr_camera["<<minMC<<"]= "<<MP_Est_curr_camera[minMC]<< std::endl;
            std::cout <<"MP_curr2_camera["<<i<<"]= "<<MP_curr2_camera[i]<< std::endl;
            std::cout <<"MP_Est_curr_world["<<minMC<<"]= "<<MP_Est_curr_world[minMC]<< std::endl;
            std::cout <<"MP_curr2_world["<<i<<"]=  "<<MP_curr2_world[i]<< std::endl;
          }
          MP_curr2_world.resize(DMP_curr2_ok);//Depth取得可能数でリサイズ(マッチング中心世界座標)
          std::cout <<"全テンプレート数:DMP_prev_ok="<<DMP_curr2_ok<< std::endl;

          for (int i = 0; i < DMP_curr2_ok; i++) {
            std::cout <<"MP_curr2_pixel["<<i<<"]="<<MP_curr2_pixel[i]<< std::endl;
            //テンプレートまでの距離と角度を求める(観測値)
            CameraLMP[0]=sqrt((-MP_curr2_camera[i].x*-MP_curr2_camera[i].x)+(MP_curr2_camera[i].z*MP_curr2_camera[i].z));
            CameraLMP[1]=atan2(-MP_curr2_camera[i].x,MP_curr2_camera[i].z);
            Zu_P[i] = (cv::Mat_<double>(2,1) <<
              CameraLMP[0],
              CameraLMP[1]);
          }
          //連続でマーカー観測されない時
          if(DMP_curr2_ok<=0){
            cv::swap(MP_Est_curr_world , MP_Est_prev_world);//推定した世界座標→推定用世界座標
            cv::swap(MP_Est_curr_camera, MP_Est_prev_camera);//推定したカメラ座標→推定用カメラ座標
          }
        }*/
      }//if(Tracking == true)→end
    }//if (reset == false)→end

    //マーカーTF-------------------------------------------------------------------------------------------
    /*std::string target_maker_frame = "marker_world_link";//cameraとマーカー間のリンク
    geometry_msgs::Pose maker_pose;

    maker_pose.position.x = MarkerW[markerIds.at(0)](2,0);//Rvizと画像は座標系が異なるので注意
    maker_pose.position.y = MarkerW[markerIds.at(0)](1,0);
    maker_pose.position.z = MarkerW[markerIds.at(0)](0,0);
    maker_pose.orientation.w = 1.0;

    static tf::TransformBroadcaster br_maker;
    tf::Transform maker_transform;
    poseMsgToTF(maker_pose, maker_transform);
    br_maker.sendTransform(tf::StampedTransform(maker_transform, ros::Time::now(), source_frame, target_maker_frame));*/

    if(Tracking == true){
      //tf(観測特徴点)-------------------------------------------------------------------------------------------------観測マーカー
      std::string target_maker_frame1 = "MP_curr2_world";//cameraとマーカー間のリンク
      geometry_msgs::Pose maker_pose1;

      std::cout << "tf特徴点の世界座標:MP_curr2_world[i]={x="<< MP_curr2_world[0].x <<",y="<<MP_curr2_world[0].y<<",z="<<MP_curr2_world[0].z<<"}"<< std::endl;
      maker_pose1.position.x = MP_curr2_world[0].z;//Rvizと画像は座標系が異なるので注意
      maker_pose1.position.y = MP_curr2_world[0].x;
      maker_pose1.position.z = MP_curr2_world[0].y;
      maker_pose1.orientation.w = 1.0;

      static tf::TransformBroadcaster br_maker1;
      tf::Transform maker_transform1;
      poseMsgToTF(maker_pose1, maker_transform1);
      br_maker1.sendTransform(tf::StampedTransform(maker_transform1, ros::Time::now(), source_frame, target_maker_frame1));
    }

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

    if(Act_RobotX>0||Act_RobotY>0||Act_RobotTH>0){//ラグの調整
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

      ESTL<<sqrt(Est_RobotX*Est_RobotX-Est_RobotY*Est_RobotY)<<"\n";

      //更新ステップ------------------------------------------------------------------
      //更新ステップはランドーマークの数だけ更新を行う
      for(int i=0;i<markerIds.size();i++){
        std::cout <<"更新ステップ(マーカー)"<< std::endl;
        //観測方程式(信念分布の中心位置から見たLMまでの距離と角度)(理想推定)
        hu = (cv::Mat_<double>(2,1) <<
          sqrt((Est_RobotX-MarkerW[markerIds.at(i)](2,0))*(Est_RobotX-MarkerW[markerIds.at(i)](2,0)) + (Est_RobotY-MarkerW[markerIds.at(i)](0,0))*(Est_RobotY-MarkerW[markerIds.at(i)](0,0))),
          atan2(MarkerW[markerIds.at(i)](0,0)-Est_RobotY,MarkerW[markerIds.at(i)](2,0)-Est_RobotX) - Est_RobotTH);

        double lu=sqrt((Est_RobotX-MarkerW[markerIds.at(i)](2,0))*(Est_RobotX-MarkerW[markerIds.at(i)](2,0))+(Est_RobotY-MarkerW[markerIds.at(i)](0,0))*(Est_RobotY-MarkerW[markerIds.at(i)](0,0)));

        //観測方程式のヤコビ行列Ht信念分布の中心と世界座標ランドマーク位置(MarkerW[markerIds.at(i)](0,0),MarkerW[markerIds.at(i)](2,0))(理想推定)
        Ht= (cv::Mat_<double>(2,3) << //共分散Ht
          (Est_RobotX-MarkerW[markerIds.at(i)](2,0))/lu,       (Est_RobotY-MarkerW[markerIds.at(i)](0,0))/lu,       0,
          (MarkerW[markerIds.at(i)](0,0)-Est_RobotY)/(lu*lu),  (Est_RobotX-MarkerW[markerIds.at(i)](2,0))/(lu*lu), -1);

        Qt= (cv::Mat_<double>(2,2) <<//センサーの誤差共分散(直進と角度の誤差共分散を代入)
          (lu*lu)*0.0006,  0,
          0,            0.00008);

        cv::Mat tempK=Qt+(Ht*Cov*Ht.t());
        K=Cov*Ht.t()*tempK.inv();//カルマンゲインK

        //センサー値を反映するための更新式
        Cov=(I-K*Ht)*Cov;//信念分布の分散Σ

        //信念分布の中心位置(推定自己位置)μ=EST_Robot
        //zはセンサーから取得したマーカーまでの距離と角度データ
        //hは推定位置からみたマーカーまでの距離と角度(理想推定値)
        //多分カルマンゲインをかけることで2次から3次への拡張が可能
        EST_Robot=K*(Zu[markerIds.at(i)]-hu)+EST_Robot;
      }

      //更新ステップは特徴点の数だけ更新を行う
      for(int i=0;i<DMP_curr2_ok;i++){
        std::cout <<"更新ステップ(特徴点)"<< std::endl;
        //観測方程式(信念分布の中心位置から見たLMまでの距離と角度)(理想推定)
        hu = (cv::Mat_<double>(2,1) <<
          sqrt((Est_RobotX-MP_curr2_world[i].z)*(Est_RobotX-MP_curr2_world[i].z) + (Est_RobotY-MP_curr2_world[i].x)*(Est_RobotY-MP_curr2_world[i].x)),
          atan2(MP_curr2_world[i].x-Est_RobotY,MP_curr2_world[i].z-Est_RobotX) - Est_RobotTH);
        hu_pointl<<"Hu_P["<<i<<"].at<double>(0)="<<hu.at<double>(0)<<"\n";
        hu_pointth<<"Hu_P["<<i<<"].at<double>(1)="<<hu.at<double>(1)<<"\n";

        double lu=sqrt((Est_RobotX-MP_curr2_world[i].z)*(Est_RobotX-MP_curr2_world[i].z)+(Est_RobotY-MP_curr2_world[i].x)*(Est_RobotY-MP_curr2_world[i].x));

        //観測方程式のヤコビ行列Ht信念分布の中心と世界座標ランドマーク位置(MP_curr2_world[i].x,MP_curr2_world[i].z)(理想推定)
        Ht= (cv::Mat_<double>(2,3) << //共分散Ht
          (Est_RobotX-MP_curr2_world[i].z)/lu,       (Est_RobotY-MP_curr2_world[i].x)/lu,       0,
          (MP_curr2_world[i].x-Est_RobotY)/(lu*lu),  (Est_RobotX-MP_curr2_world[i].z)/(lu*lu), -1);

        Qt= (cv::Mat_<double>(2,2) <<//センサーの誤差共分散(直進と角度の誤差共分散を代入)
          (lu*lu)*0.0006,  0,
          0,            0.00008);

        //カルマンゲインK
        cv::Mat tempK=Qt+(Ht*Cov*Ht.t());
        K=Cov*Ht.t()*tempK.inv();

        //センサー値を反映するための更新式
        Cov=(I-K*Ht)*Cov;//信念分布の分散Σ

        //信念分布の中心位置(推定自己位置)μ=EST_Robot
        //zはセンサーから取得したマーカーまでの距離と角度データ
        //hは推定位置からみたマーカーまでの距離と角度(理想推定値)
        //多分カルマンゲインをかけることで2次から3次への拡張が可能
        EST_Robot=K*(Zu_P[i]-hu)+EST_Robot;
      }

      Est_RobotX=EST_Robot.at<double>(0);
      Est_RobotY=EST_Robot.at<double>(1);
      Est_RobotTH=EST_Robot.at<double>(2);
      est_robot<< "ALLrealsec=" <<ALLrealsec<< " ,realsec=" <<realsec<<" ,Est_RobotX=" <<Est_RobotX<<",Est_RobotY=" <<Est_RobotY<<",Est_RobotTH=" <<Est_RobotTH<<"\n";
      std::cout <<"test4"<< std::endl;
    }
  }//if(time0 != false)→END
  
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
    std::cout <<"tst1"<< std::endl;

    //推定状態tf(map Des_Robot間のlink)-----------------------------------------------------------------------------------------
    //ここがカメラの姿勢部分
    std::string MaptoEst_Robot_frame = "MaptoEst_Robot_link";
    RollX=0,PitchY=Est_RobotTH,YawZ=0;//微小区間回転行列

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
    std::cout <<"tst2"<< std::endl;

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
    std::cout <<"tst3"<< std::endl;

    // 画面表示
    cv::imshow(win_src, image);
    cv::imshow(win_dst, img_dst);
    cv::imshow(win_dst2, img_dst2);
    cv::imshow(win_master_temp, img_master_temp);

    cv::swap(image_curr, image_prev);// image_curr を image_prev に移す（交換する）
    cv::swap(points_curr, points_prev);//二次元画像座標を保存(points_curr→points_prev)
    cv::swap(camera_point_c, camera_point_p);//三次元カメラ座標を保存(camera_point_c→camera_point_p)
    cv::swap(FeaturePoint_curr, FeaturePoint_prev);//今のテンプレートを一つ前のテンプレートとして保存
    depth_point_prev_ok=depth_point_curr_ok;//テンプレート取得数を一つ前の取得数として保存
    /*//2回目動作時
    if(Tracking==false){ 
      DMP_prev_ok=DMP_curr_ok;//Depth取得可能マッチテンプレート数をキープ+(3回目以降はこれに一つ前のテンプレート要素を追加する)
      cv::swap(MP_curr_world, MP_prev_world);//マッチ中心世界座標
      cv::swap(MP_curr_camera, MP_prev_camera);//マッチ中心カメラ座標
      cv::swap(MP_curr_pixel, MP_prev_pixel);//マッチ中心画像座標
      cv::swap(MP_curr_Templ, MP_prev_Templ);//マッチテンプレート画像
      Tracking=true;//3回目動作
    }
    //3回目以降動作時
    else if(Tracking == true){
      //Depth取得可能マッチング要素キープ
      DMP_prev_ok=DMP_curr2_ok;
      cv::swap(MP_curr2_world, MP_prev_world);//マッチ中心世界座標
      cv::swap(MP_curr2_camera, MP_prev_camera);//マッチ中心カメラ座標
      cv::swap(MP_curr2_pixel, MP_prev_pixel);//マッチ中心画像座標
      cv::swap(MP_curr2_Templ, MP_prev_Templ);//マッチテンプレート画像
    }*/

   //2回目動作時
    if(Tracking==false){ 
      DMP_prev_ok=DMP_curr_ok;//Depth取得可能マッチテンプレート数をキープ+(3回目以降はこれに一つ前のテンプレート要素を追加する)
      cv::swap(MP_curr_world, MP_prev_world);//マッチ中心世界座標
      cv::swap(MP_curr_camera, MP_prev_camera);//マッチ中心カメラ座標
      cv::swap(MP_curr_pixel, MP_prev_pixel);//マッチ中心画像座標
      cv::swap(MP_curr_Templ, MP_prev_Templ);//マッチ座標
    }
    //3回目以降動作時
    if(Tracking == true){
      //Depth取得可能マッチング要素キープ
      DMP_prev_ok=DMP_curr2_ok;
      cv::swap(MP_curr2_world, MP_prev_world);//マッチ中心世界座標
      cv::swap(MP_curr2_camera, MP_prev_camera);//マッチ中心カメラ座標
      cv::swap(MP_curr2_pixel, MP_prev_pixel);//マッチ中心画像座標
      cv::swap(MP_curr2_Templ, MP_prev_Templ);//マッチ座標
      MP_keep_world.resize(1000);
      MP_keep_camera.resize(1000);
      for(int i=0;i<DMP_prev_ok;i++){
        MP_keep_world[i]=MP_prev_world[i];
        MP_keep_camera[i]=MP_prev_camera[i];
      }
      //std::copy(MP_prev_world.begin(), MP_prev_world.end(), back_inserter(MP_Est_prev_world) );
      //std::copy(MP_prev_camera.begin(), MP_prev_camera.end(), back_inserter(MP_Est_prev_camera) );
      MP_keep_world.resize(DMP_prev_ok);
      MP_keep_camera.resize(DMP_prev_ok);
    }
    std::cout <<"tst4"<< std::endl;

    if (kaisu >=1){
      Tracking=true;//3回目動作
    }

    if(swap_on ==false){//初回プログラム実行時はswapしない(データの浅いコピーの影響を考慮)
        //cv::swap(points_curr, points_prev);//二次元画像座標を保存(points_curr→points_prev)
        //cv::swap(camera_point_c, camera_point_p);//三次元カメラ座標を保存(camera_point_c→camera_point_p)
      //cv::swap(world_point_c, world_point_p);//特徴点の世界座標を保存(camera_point_c→camera_point_p)
    }

    robot_velocity.linear.x  = 0.0; // 並進速度の初期化
    robot_velocity.angular.z = 0.0; // 回転速度の初期化

    kaisu++;
    if(time0 != false){
      std::cout << "処理時間=" <<realsec<<",  処理経過時間="<<ALLrealsec<< std::endl;//サンプリング時間
      Realsec<<realsec<<"\n";
    }
    time0=true;//一回目スキップ
    endTime=startTime;//動作終了時刻取得
    endTimeV1=startTimeV1;//動作終了時刻取得
    endTimeM1=startTimeM1;//動作終了時刻取得
    std::cout <<"tst5"<< std::endl;


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
  Des_pub_plan = nhPub.advertise<nav_msgs::Path>("/Des_path",1000);
  Act_pub_plan = nhPub.advertise<nav_msgs::Path>("/Act_path",1000);
  Est_pub_plan = nhPub.advertise<nav_msgs::Path>("/Est_path",1000);


  //(x,y,z)ここはカメラ座標系で記述(中の値はロボット座標に変換済み)
  //MarkerW[4]= (cv::Mat_<float>(3, 1) <<-0.20, 0.28, 3.0);//(x,y,z)ここはカメラ座標系で記述(中の値はロボット座標に変換済み)
  //MarkerW[5]= (cv::Mat_<float>(3, 1) <<0.20, 0.28, 3.0);
  //MarkerW[3]= (cv::Mat_<float>(3, 1) <<0.40, 0.28, 3.0);
  //MarkerW[6]= (cv::Mat_<float>(3, 1) <<0, 0.28, 3.0);
  //MarkerW[0]= (cv::Mat_<float>(3, 1) <<0, 0.5, 3.0);
  //MarkerW[1]= (cv::Mat_<float>(3, 1) <<0.2, 0.5, 3.0);

  
  //マーカーの世界座標登録(単位はmm)(20210930のrobag)
  //MarkerW[0]= (cv::Mat_<float>(3, 1) <<-470, 300, 1050);
  //MarkerW[1]= (cv::Mat_<float>(3, 1) <<570, 280, 2090);
  //MarkerW[2]= (cv::Mat_<float>(3, 1) <<-450, 325, 3250);
  //MarkerW[3]= (cv::Mat_<float>(3, 1) <<580, 300, 3680);
  //MarkerW[5]= (cv::Mat_<float>(3, 1) <<-410, 300, 4800);
  //MarkerW[6]= (cv::Mat_<float>(3, 1) <<120, 280, 6100);
  //MarkerW[7]= (cv::Mat_<float>(3, 1) <<1440, 650, 6060);
  //MarkerW[4]= (cv::Mat_<float>(3, 1) <<2300, 460, 6000);
  //MarkerW[8]= (cv::Mat_<float>(3, 1) <<3000, 880, 5250);
  //MarkerW[9]= (cv::Mat_<float>(3, 1) <<2800, 830, 3600);

  //MarkerW[3]= (cv::Mat_<float>(3, 1) <<0.0, 0.28, 3.0);//20211030研究室(直進2.7m,回転-3.141592653/2.4,速度0.1,角速度-(0.2+(0.0176*ALLrealsecM1+0.11)))
  //MarkerW[4]= (cv::Mat_<float>(3, 1) <<-2.3, 0.28, 2.74);
  //MarkerW[8]= (cv::Mat_<float>(3, 1) <<0.62, 0.73, 2.60);

  //MarkerW[3]= (cv::Mat_<float>(3, 1) <<0.0, 0.28, 2.0);//20211104研究室(直進1.7m,回転-3.141592653/2.2,速度0.1,角速度-(0.2+(0.0176*ALLrealsecM1+0.11)))
  //MarkerW[4]= (cv::Mat_<float>(3, 1) <<-1.5, 0.28, 2.3);
  //MarkerW[5]= (cv::Mat_<float>(3, 1) <<-2.3, 0.28, 1.74);

  //MarkerW[4]= (cv::Mat_<float>(3, 1) <<0.5, 0.28, 3.0);//20211109廊下(直進2m,回転3.141592653/2.0,速度0.1,角速度(0.2+(0.0176*ALLrealsecM1+0.11)))
  //MarkerW[5]= (cv::Mat_<float>(3, 1) <<1.5, 0.28, 2.9);//実測値(X:3.42,Y:1.99)
  //MarkerW[3]= (cv::Mat_<float>(3, 1) <<4.0, 0.28, 2.0);
  //MarkerW[3]= (cv::Mat_<float>(3, 1) <<2.2, 0.280, 0.79);

  //MarkerW[3]= (cv::Mat_<float>(3, 1) <<1.0, 0.28, 3.0);//20211110廊下(直進40m,速度0.3)
  //MarkerW[4]= (cv::Mat_<float>(3, 1) <<1.0, 0.28, 12.95);//実測値(X:3.42,Y:1.99)
  //MarkerW[5]= (cv::Mat_<float>(3, 1) <<1.0, 0.28, 27.67);

  //廊下回転動作実験
  //MarkerW[3]= (cv::Mat_<float>(3, 1) <<1.0, 0.28, 3.0);//20211117_V1廊下(直進2m,速度0.3,回転90度,回転速度0.2,直進10m,速度0.3)
  //MarkerW[4]= (cv::Mat_<float>(3, 1) <<9.52, 0.28, 0.77);//実測値(X:1.06,Y:7.26)

  //廊下回転動作実験
  //MarkerW[3]= (cv::Mat_<float>(3, 1) <<1.0, 0.28, 3.0);//20211117_V2廊下(直進2m,速度0.3,回転90度,回転速度0.2,直進10m,速度0.3)
  //MarkerW[4]= (cv::Mat_<float>(3, 1) <<9.52,0.28, 2.9);//実測値(X:0.98,Y:6.23)

  //廊下回転動作実験
  MarkerW[3]= (cv::Mat_<float>(3, 1) <<1.165, 0.28, 3.0);//20211118_V1廊下(直進4.5m,速度0.3,回転90度,回転速度0.2,直進10m,速度0.3)
  MarkerW[4]= (cv::Mat_<float>(3, 1) <<-0.61, 0.28, 5.8);
  MarkerW[5]= (cv::Mat_<float>(3, 1) <<7.57, 0.28, 3.0);


    
	ros::spin();//トピック更新待機
			
	return 0;
}