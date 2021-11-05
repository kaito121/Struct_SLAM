///マーカー検出プログラム https://qiita.com/noueezy/items/58730f09f6aa6a97faf5
//マーカー検出時の赤四角が左上になるようにマーカーを配置すると正しくマーカーの中心座標を求められる
//外部パラメータを推定するためには最低でも３点の情報が必要→そこでマーカーのコーナー４点を使用して外部パラメータを算出する

//20210826 システムのサンプリング時間を取得したいがros time nowがうまく使えない（何故か値が出ない）
//そこで今はWalltimeで代用するが、ここで問題が発生する可能性があるので今後修繕が必要
//https://8ttyan.hatenablog.com/entry/2015/02/03/003428
//20210921 ロボットビジョンの手法と日高先生の手法を組み合わせたバージョン

//特徴点検出を交互に行うことで常に特徴点を追跡できるようにしたい
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
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/CameraInfo.h>//camera_infoを獲得するためのヘッダー
#include <struct_slam/marker_tf.h>//自作メッセージ用ヘッダ#include<develのファイル名/メッセージファイル名>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <opencv2/aruco/charuco.hpp>//マーカー検出

#include <geometry_msgs/PoseStamped.h>//tf
#include <tf/transform_broadcaster.h>//tf

#include "struct_slam/rs.h"

#include "struct_slam/Conversions/EulerAngle.h"//座標変換用
#include "struct_slam/Conversions/Quaternion.h"
#include "struct_slam/Conversions/RotationMatrix.h"
#include "struct_slam/Conversions/Vector3.h"
#include "struct_slam/Conversions/conversion.h"

#include <nav_msgs/Path.h>//経路情報を記録する
#include <time.h>//処理の時間を出力する
#include <sys/time.h>

#include<fstream>//ファイル出力用ヘッダー

std::string win_src = "src";
std::string win_dst = "dst";
std::string win_depth = "win_depth";

std::string source_frame = "map";//mapフレーム
ros::Subscriber marker_sub;
ros::Publisher pub_plan;//カメラ経路送信用

using namespace std;
using namespace cv;
float kaisu = 0;//tf用
int kosuu=0,PT_kosuu=0,PT2_kosuu=0,points_kosuu=0,PTCLkosuu=0;
int KAL_Wait=0;//カルマン推定マーカー再認識初回回避用(1以下はカルマンスキップ)
int depth_point_prev_ok=0; //depth取得可能な特徴点の数
int depth_point_curr_ok=0; //depth取得可能な特徴点の数
int depth_points_currA_ok=0; //depth取得可能な特徴点の数
int depth_point_prev2_ok=0; //depth取得可能な特徴点の数
int depth_point_curr2_ok=0; //depth取得可能な特徴点の数

bool KAL = false;//カルマン推定データ初回回避用(カルマン動作した時TRUE)
bool reset = true;//初回プログラム用（特徴点検出&特徴点再検出用)
bool swap_on = true;//初回プログラム実行時はswapしない(データの浅いコピーの影響を考慮)
bool reset2 = true;//初回プログラム用（特徴点検出&特徴点再検出用)
bool swap_on2 = true;//初回プログラム実行時はswapしない(データの浅いコピーの影響を考慮)
bool PLAS_POINT= true;//特徴点追加動作判断用


cv::Mat image_curr,image_prev,img_dst,img_dst1;//画像定義
vector<cv::Point2f> points_prev, points_curr;//特徴点定義
vector<cv::Point2f> ALL_points_prev, ALL_points_curr;//特徴点定義
vector<cv::Point2f> points_prevA, points_currA;//特徴点定義
vector<cv::Point2f> points_prev2, points_curr2;//特徴点定義
vector<cv::Point3f> camera_point_p,camera_point_c;//特徴点定義
vector<cv::Point3f> ALL_camera_point_p,ALL_camera_point_c;//特徴点定義
vector<cv::Point3f> camera_point_pA,camera_point_cA;//特徴点定義
vector<cv::Point3f> camera_point_p2,camera_point_c2;//特徴点定義
float depth_point_prev[600],depth_point_curr[600];
float depth_points_prevA[600],depth_points_currA[600];
float depth_point_prev2[600],depth_point_curr2[600];
cv::Mat_<float> F_Xp = cv::Mat_<float>(4, 1);//外部パラメータ（日高手法 特徴点)
cv::Mat_<float> F_XpPLAS = cv::Mat_<float>::zeros(4, 1);
cv::Mat_<float>PT_At0,PT_Et0;
cv::Mat_<float> PT_Vt;//外部パラメータ並進ベクトル(特徴点)
cv::Mat_<float> PT_Ft;//外部パラメータ回転行列(特徴点)
cv::Mat_<float> PT_Tt=cv::Mat_<float>(6, 1);//教科書運動パラメータ(特徴点のみで推定)

cv::Mat_<float>PT2_At0,PT2_Et0;
cv::Mat_<float> PT2_Vt;//外部パラメータ並進ベクトル(特徴点)
cv::Mat_<float> PT2_Ft;//外部パラメータ回転行列(特徴点)
cv::Mat_<float> PT2_Tt=cv::Mat_<float>(6, 1);//教科書運動パラメータ(特徴点のみで推定)

cv::Mat_<float>At;//外部パラメータ推定
cv::Mat_<float>Et;//外部パラメータ推定
cv::Mat_<float>AWt;//外部パラメータ推定
cv::Mat_<float>ALL_At;//外部パラメータ推定
cv::Mat_<float>ALL_Et;//外部パラメータ推定

//マーカー関連
sensor_msgs::CameraInfo camera_info;//CameraInfo受け取り用
static struct_slam::marker_tf marker_tf;//観測マーカーのメッセージ
float Rotation;//外部パラメータ(仮)
float MC_point_prve[50][4];//一つ前のカメラ座標
cv::Mat_<float> Xp = cv::Mat_<float>(4, 1);//外部パラメータ（日高手法)
cv::Mat_<float> XpPLAS = cv::Mat_<float>::zeros(4, 1);
float pixel[50][2],depth[100],MC_point[50][4],x,y,r2,f,ux,uy;//画像→カメラ座標変換

ros::Time ros_begin;//プログラム時間
ros::WallTime wall_begin = ros::WallTime::now();//プログラム開始時の時間
ros::WallDuration wall_prev;//一つ前の時間
ros::WallDuration wall_systemtime;//サンプリング時間
struct timeval startTime, endTime;  // 構造体宣言
float realsec;//サンプリング時間（C++)

int ALLMarker=40;//全マーカー個数
//カルマンフィルタ
cv::Mat_<float> Vt;//外部パラメータ並進ベクトル
cv::Mat_<float> Ft;//外部パラメータ回転行列
cv::Mat_<float> ALL_Vt;//外部パラメータ並進ベクトル(マーカー+特徴点)
cv::Mat_<float> ALL_Ft;//外部パラメータ回転行列(マーカー+特徴点)
cv::Mat_<float> ALL_Tt=cv::Mat_<float>(6, 1);//教科書運動パラメータ

////カルマン定義xEst0_points_prev_cl
//cv::Mat_<float>xEst0_cl[40]=cv::Mat_<float>(3, 1);//今観測したマーカー座標(カルマン用)
//cv::Mat_<float>xEst0_points_prev_cl[40]=cv::Mat_<float>(3,1);//一つ前に観測したマーカー座標（カルマン用)
//cv::Mat_<float>xEst_points_cl[40]=cv::Mat_<float>(3, 1);//今推定したマーカー座標(カルマン用)
//cv::Mat_<float>xEst_points_prev_cl[40]=cv::Mat_<float>(3, 1);//一つ前に推定したマーカー座標（カルマン用)
//cv::Mat_<float>pixel_points_cl[40]=cv::Mat_<float>(2, 1);//追跡可能なマーカーの画像座標（カルマン用)
//int xEst0_points_prev_clP[40],xEst0_clP[40],xEst_points_prev_clP[40];//要素があるかどうか(マッチング用)
//cv::Mat_<float>Gt[40]=cv::Mat_<float>(3, 4);
//cv::Mat_<float> Mt_cl[40] = cv::Mat_<float>::eye(3, 3);//誤差共分散(予測式)
//cv::Mat_<float> Pt_cl[40] = cv::Mat_<float>::eye(3, 3);//誤差共分散(更新式)
//cv::Mat_<float> Yt_cl[40] = cv::Mat_<float>::zeros(2, 1);//観測残差
//cv::Mat_<float> Ht_cl[40] = cv::Mat_<float>::zeros(2, 3);//Htヤコビアン行列
//cv::Mat_<float> Ut_cl[40] = cv::Mat_<float>::zeros(2, 1);//共分散
//cv::Mat_<float> St[40] = cv::Mat_<float>::zeros(2, 2);//共分散
//cv::Mat_<float> Kt_cl[40] = cv::Mat_<float>::zeros(3, 2);//カルマンゲイン
//cv::Mat_<float> u_ = cv::Mat_<float>(2, 2);

//特徴点カルマン定義
cv::Mat_<float>xEst0_points_cl[1000]=cv::Mat_<float>(3, 1);//今観測したマーカー座標(カルマン用)
cv::Mat_<float>xEst0_points_prev_cl[1000]=cv::Mat_<float>(3,1);//一つ前に観測したマーカー座標（カルマン用)
cv::Mat_<float>xEst_points_cl[1000]=cv::Mat_<float>(3, 1);//今推定したマーカー座標(カルマン用)
cv::Mat_<float>xEst_points_prev_cl[1000]=cv::Mat_<float>(3, 1);//一つ前に推定したマーカー座標（カルマン用)
cv::Mat_<float>pixel_points_cl[1000]=cv::Mat_<float>(2, 1);//追跡可能なマーカーの画像座標（カルマン用)
//int xEst0_points_prev_clP[40],xEst0_clP[40],xEst_points_prev_clP[40];//要素があるかどうか(マッチング用)
cv::Mat_<float>Gt[1000]=cv::Mat_<float>(3, 4);
cv::Mat_<float> Mt_cl[1000] = cv::Mat_<float>::eye(3, 3);//誤差共分散(予測式)
cv::Mat_<float> Pt_cl[1000] = cv::Mat_<float>::eye(3, 3);//誤差共分散(更新式)
cv::Mat_<float> Yt_cl[1000] = cv::Mat_<float>::zeros(2, 1);//観測残差
cv::Mat_<float> Ht_cl[1000] = cv::Mat_<float>::zeros(2, 3);//Htヤコビアン行列
cv::Mat_<float> Ut_cl[1000] = cv::Mat_<float>::zeros(2, 1);//共分散
cv::Mat_<float> St[1000] = cv::Mat_<float>::zeros(2, 2);//共分散
cv::Mat_<float> Kt_cl[1000] = cv::Mat_<float>::zeros(3, 2);//カルマンゲイン
cv::Mat_<float> u_ = cv::Mat_<float>(2, 2);

//cv::Mat_<float> u_ = (cv::Mat_<float>(2, 2) <<
//    0.2, 0,
//    0, 0.2);

//ofstream outputfile3("/home/fuji/catkin_ws/src/Struct_SLAM/src/EKF/marker_test_NO_ALL_xEst[3].txt");//出力ファイルパス
//ofstream outputfile11("/home/fuji/catkin_ws/src/Struct_SLAM/src/EKF/marker_test_NO_ALL_xEst[11].txt");//出力ファイルパス
//ofstream outputfile6("/home/fuji/catkin_ws/src/Struct_SLAM/src/EKF/xEst[6].txt");//出力ファイルパス
//ofstream outputfile11("/home/fuji/catkin_ws/src/Struct_SLAM/src/EKF/xEst[11].txt");//出力ファイルパス
//ofstream outputfile27("/home/fuji/catkin_ws/src/Struct_SLAM/src/EKF/xEst[27].txt");//出力ファイルパス
ofstream outputfileTF("/home/fuji/catkin_ws/src/Struct_SLAM/src/EKF/no_EKF_TF/TFposition_gaipal.txt");//出力ファイルパス
ofstream outputfileALL_Vt("/home/fuji/catkin_ws/src/Struct_SLAM/src/EKF/no_EKF_TF/TF_ALL_Vt_gaipal.txt");//出力ファイルパス

struct Camera_Base{
    float x;
    float y;
    float z;
};
struct Camera_Base camera_base;//カメラ姿勢(tf)の初期設定
geometry_msgs::Pose camera_base_pose;

nav_msgs::Path path;//カメラ経路表示設定
geometry_msgs::PoseStamped pose;

float camera_prmX=0,camera_prmY=0,camera_prmZ=0;

//コールバック関数
void callback(const sensor_msgs::Image::ConstPtr& rgb_msg,const sensor_msgs::Image::ConstPtr& depth_msg, const sensor_msgs::CameraInfo::ConstPtr& cam_info)
{
  ROS_INFO("callback_functionが呼ばれたよ");

  //サンプリング時間取得(C言語の方法)(こっちのほうが正確らしい)
  gettimeofday(&startTime, NULL);// 開始時刻取得
  if(kaisu!=0){
    time_t diffsec = difftime(startTime.tv_sec, endTime.tv_sec);    // 秒数の差分を計算
    suseconds_t diffsub = startTime.tv_usec - endTime.tv_usec;      // マイクロ秒部分の差分を計算
    realsec = diffsec+diffsub*1e-6;                          // 実時間を計算
    printf("処理の時間=%f\n", realsec);
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
  struct_slam::marker_tf marker_tf2=marker_tf;//tfのデータ一時保存(更新を防ぐため)

  std::array<float, 9>  array;
  Quaternion quaternionVal(1,2,3,4);
  RotationMatrix rotationMatrixVal(array);

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
  
  camera_info=*cam_info;//CameraInfo受け取り
   
  image = bridgeImage->image.clone();//image変数に変換した画像データを代入
  depthimage = bridgedepthImage->image.clone();//image変数に変換した画像データを代入

  //Depth画像の色付けを行なっている
  //double min,max;
  //cv::minMaxIdx(depthimage, &min, &max);
  //cv::Mat adjimg_depth;
  //// Histogram Equalization
  //float scale = 255 / (max-min);
  //depthimage.convertTo(adjimg_depth,CV_8UC1, scale, -min*scale); 
  //cv::Mat falseColorsimg_depth;
  //applyColorMap(adjimg_depth, falseColorsimg_depth, cv::COLORMAP_WINTER);//ここのcvで色を変えられる
  //cv::imshow("Out", falseColorsimg_depth);

  image.copyTo(img_dst);//

  cv::Mat imageCopy = image.clone();
  cv::Mat_<float> intrinsic_K= cv::Mat_<float>(3, 3);
	std::cout <<"画像取り込み"<< std::endl;
  //特徴点----------------------------------------------------------------------------------------------------------
  //特徴点検出--------------------------------------------------------------------------------------------------------------
  //一つ目前の特徴点の数と現在の特徴点の数を合わせる必要がある。ここでは追跡可能か,それぞれDepthデータ取得可能であるかでサイズ調整を行っている
	cv::cvtColor(image, image_curr, cv::COLOR_BGR2GRAY);//グレースケール化
  //初回検出プログラム-----------------------------------------------------------------------------------------------------
  if (reset == true) {
	  std::cout <<"初回検出プログラム"<< std::endl;
    depth_point_prev_ok=0; //depth取得可能な特徴点の数(初期化)
    swap_on=false;
    //cv::goodFeaturesToTrack(今画像, 今の特徴点, 特徴点の個数, 0.01, 10, cv::Mat(), 3, 3, 0, 0.04);
		cv::goodFeaturesToTrack(image_curr, points_curr, 600, 0.01, 10, cv::Mat(), 3, 3, 0, 0.04);// 特徴点検出(グレースケール画像から特徴点検出)
		cv::cornerSubPix(image_curr, points_curr, cv::Size(10, 10), cv::Size(-1, -1), cv::TermCriteria(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, 20, 0.03));
		points_prev = points_curr;//データのコピ
    camera_point_p.resize(points_prev.size());//要素数初期設定(座標変換用)
	  std::cout <<"test2_points_prev.size()="<<points_prev.size()<< std::endl;
    
		for (int i = 0; i < points_prev.size(); i++) {
			cv::circle(imageCopy, points_prev[i], 6, Scalar(255,0,0), -1, cv::LINE_AA);
      //画像→カメラ座標変換----------------------------------------------------------------------
      depth_point_prev[i] = depthimage.at<float>(cv::Point(points_prev[i].x,points_prev[i].y));
      //Depthが取得できない特徴点を削除する+Depthの外れ値を除く
      if(depth_point_prev[i]>0.001&&depth_point_prev[i]<10000){
        camera_point_p[depth_point_prev_ok].x = depth_point_prev[i] * ((points_prev[i].x - camera_info.K[2]) / camera_info.K[0])/1000;//メートル表示変換
        camera_point_p[depth_point_prev_ok].y = depth_point_prev[i] * ((points_prev[i].y - camera_info.K[5]) / camera_info.K[4])/1000;
        camera_point_p[depth_point_prev_ok].z = depth_point_prev[i]/1000;

        //std::cout << "特徴点のカメラ座標:camera_point_p[depth_point_prev_ok="<<depth_point_prev_ok<<"]={"<< camera_point_p[depth_point_prev_ok].x <<","<<camera_point_p[depth_point_prev_ok].y<<","<<camera_point_p[depth_point_prev_ok].z<<"}"<< std::endl;
        points_prev[depth_point_prev_ok] = points_prev[i];
        depth_point_prev_ok=depth_point_prev_ok+1;//Depth取得可能の個数をカウント
      }
		}
    points_prev.resize(depth_point_prev_ok);//Depth取得可能数でリサイズ(二次元)
    camera_point_p.resize(depth_point_prev_ok);//Depth取得可能数でリサイズ(三次元カメラ座標)
    reset = false;//if文切り替え
	  std::cout <<"初回検出プログラム終了"<< std::endl;
  }
  //オプティカルフロー-------------------------------------------------------------------------------------
  else{
	  std::cout <<"オプティカルフロー"<< std::endl;// 特徴点追跡(二回目のフレーム)
	  vector<uchar> status;//特徴点の数
	  vector<float> err;
    swap_on = true;
    depth_point_curr_ok=0; //depth取得可能な特徴点の数(初期化)

    cv::calcOpticalFlowPyrLK(image_prev, image_curr, points_prev, points_curr, status, err);//オプティカルフロー
    //一つ前のOPTで追跡可能だったpoints_prevと
    //cv::calcOpticalFlowPyrLK(image_prev, image_curr, points_curr, points_prev, status, err);//オプティカルフロー(points_currは追跡可能points)
	  std::cout <<"status.size()="<<status.size()<< std::endl;//特徴点の個数

	  // 追跡できなかった特徴点をリストから削除する
	  int i, k,n,j=0;
	  for (i = k = n =0; i < status.size(); i++){
	  	//std::cout <<"status["<<i<<"]="<<status[i]<< std::endl;//0以外の時は変なマークがでる
	  	//ここで検索出来なかったものを消し探索の範囲を狭めている(statusが0の時)
	  	//statusが0以外の時値を更新する(0は追跡不可能)
	  	if (status[i] != 0) {	
	  	  points_prev[k] = points_prev[i];
        camera_point_p[k] = camera_point_p[i];
	  	  points_curr[k++] = points_curr[i];
	  	}
    }
	  points_curr.resize(k);//ここでkの個数でリサイズ
	  points_prev.resize(k);
	  camera_point_p.resize(k);
	  camera_point_c.resize(k);

    for (j= n = 0; j < points_curr.size(); j++){
      //追跡距離が極端に長い追跡を削除(追跡距離が100以下の時のみ使用)
      if(abs(points_prev[j].x-points_curr[j].x)<50||abs(points_prev[j].y-points_curr[j].y)<50){
        	points_prev[n] = points_prev[j];
          camera_point_p[n] = camera_point_p[j];
	    	  points_curr[n++] = points_curr[j];
      }
    }
    points_curr.resize(n);//ここでkの個数でリサイズ
	  points_prev.resize(n);
	  camera_point_p.resize(n);
	  camera_point_c.resize(n);
		

	  // 特徴点を丸で描く-------------------------------------------------------------------------------------------
	  for (int i = 0; i < points_curr.size(); i++) {
      //std::cout <<"OPT後マッチングの中心座標["<<i<<"]="<<points_curr[i]<< std::endl;
      cv::circle(imageCopy, points_prev[i], 3, Scalar(255,255,255), -1, cv::LINE_AA);//一つ前の画像の座標
	    cv::circle(imageCopy, points_curr[i], 4, Scalar(0, 255, 0), -1, cv::LINE_AA);//今の座標情報
	    //cv::line(imageCopy,cv::Point(points_curr[i]),cv::Point(points_prev[i]),cv::Scalar(0,255,255), 1, cv::LINE_AA);//線を描写する
      cv::line(imageCopy,cv::Point(points_curr[i].x,points_prev[i].y),cv::Point(points_prev[i]),cv::Scalar(255,0,0), 1, cv::LINE_AA);//線を描写する
      cv::line(imageCopy,cv::Point(points_prev[i].x,points_curr[i].y),cv::Point(points_prev[i]),cv::Scalar(0,255,0), 1, cv::LINE_AA);//線を描写する
    
      //画像→カメラ座標変換----------------------------------------------------------------------
      depth_point_curr[i] = depthimage.at<float>(cv::Point(points_curr[i].x,points_curr[i].y));
      //Depthが取得できない特徴点を削除する+Depthの外れ値を除く
      if(depth_point_curr[i]>0.001&&depth_point_curr[i]<10000){
        camera_point_c[depth_point_curr_ok].x = depth_point_curr[i] * ((points_curr[i].x - camera_info.K[2]) / camera_info.K[0])/1000;//メートル表示変換
        camera_point_c[depth_point_curr_ok].y = depth_point_curr[i] * ((points_curr[i].y - camera_info.K[5]) / camera_info.K[4])/1000;
        camera_point_c[depth_point_curr_ok].z = depth_point_curr[i]/1000;

        //std::cout << "特徴点のカメラ座標:camera_point_c[depth_point_curr_ok="<<depth_point_curr_ok<<"]={"<< camera_point_c[depth_point_curr_ok].x <<","<<camera_point_c[depth_point_curr_ok].y<<","<<camera_point_c[depth_point_curr_ok].z<<"}"<< std::endl;
        
        camera_point_p[depth_point_curr_ok]=camera_point_p[i];
        points_prev[depth_point_curr_ok] = points_prev[i];
        points_curr[depth_point_curr_ok] = points_curr[i];
        xEst_points_prev_cl[depth_point_curr_ok]=xEst_points_prev_cl[i];
        depth_point_curr_ok=depth_point_curr_ok+1;//Depth取得可能の個数をカウント
      }
    }

    cv::circle(imageCopy, points_prev[0], 6, Scalar(0,0,255), -1, cv::LINE_AA);//一つ前の画像の座標
	  cv::circle(imageCopy, points_curr[0], 6, Scalar(255, 0, 0), -1, cv::LINE_AA);//今の座標情報


    std::cout <<"depth_point_curr_ok="<<depth_point_curr_ok<< std::endl;
    camera_point_p.resize(depth_point_curr_ok);//Depth取得可能数でリサイズ(三次元カメラ座標)
    camera_point_c.resize(depth_point_curr_ok);//Depth取得可能数でリサイズ(三次元カメラ座標)
    points_prev.resize(depth_point_curr_ok);//Depth取得可能数でリサイズ(二次元座標)
    points_curr.resize(depth_point_curr_ok);//Depth取得可能数でリサイズ(二次元座標)

    depth_points_currA_ok=0;//depth取得可能な特徴点の数(初期化)
    if(camera_point_p.size()<50||k<50){//特徴点が100個以下になったら再び特徴点を検出する
      //特徴点の追加（今現在の画像から検出してオプティカルフロー追跡点に足して、次回のオプティカルフローに使用する)
		  cv::goodFeaturesToTrack(image_curr, points_currA, 600, 0.01, 10, cv::Mat(), 3, 3, 0, 0.04);// 特徴点検出(グレースケール画像から特徴点検出)
		  cv::cornerSubPix(image_curr, points_currA, cv::Size(10, 10), cv::Size(-1, -1), cv::TermCriteria(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, 20, 0.03));
		  //points_prev = points_curr;//データのコピ
      camera_point_cA.resize(points_currA.size());//要素数初期設定(座標変換用)
	    std::cout <<"test2_points_prev.size()="<<points_prev.size()<< std::endl;
      
		  for (int i = 0; i < points_currA.size(); i++) {
        //画像→カメラ座標変換----------------------------------------------------------------------
        depth_points_currA[i] = depthimage.at<float>(cv::Point(points_currA[i].x,points_currA[i].y));
        //Depthが取得できない特徴点を削除する+Depthの外れ値を除く
        if(depth_points_currA[i]>0.001&&depth_points_currA[i]<10000){
          camera_point_cA[depth_points_currA_ok].x = depth_points_currA[i] * ((points_currA[i].x - camera_info.K[2]) / camera_info.K[0])/1000;//メートル表示変換
          camera_point_cA[depth_points_currA_ok].y = depth_points_currA[i] * ((points_currA[i].y - camera_info.K[5]) / camera_info.K[4])/1000;
          camera_point_cA[depth_points_currA_ok].z = depth_points_currA[i]/1000;
          //std::cout << "特徴点のカメラ座標:camera_point_p[depth_points_currA_ok="<<depth_points_currA_ok<<"]={"<< camera_point_p[depth_points_currA_ok].x <<","<<camera_point_p[depth_points_currA_ok].y<<","<<camera_point_p[depth_points_currA_ok].z<<"}"<< std::endl;
          points_currA[depth_points_currA_ok] = points_currA[i];
          depth_points_currA_ok=depth_points_currA_ok+1;//Depth取得可能の個数をカウント
		  	  cv::circle(imageCopy, points_currA[i], 3, Scalar(255,255,0), -1, cv::LINE_AA);
        }
		  }
      points_currA.resize(depth_points_currA_ok);//Depth取得可能数でリサイズ(二次元)
      camera_point_cA.resize(depth_points_currA_ok);//Depth取得可能数でリサイズ(三次元カメラ座標)
    }

    //if(camera_point_p.size()<5||k<5){//特徴点が100個以下になったら再び特徴点を検出する
    //  reset = true;
    //  std::cout <<" 特徴点再検出リセット"<<std::endl;}

    PT_At0=cv::Mat_<float>(depth_point_curr_ok*3, 6);
    PT_Et0=cv::Mat_<float>(depth_point_curr_ok*3, 1);
    PT_Tt=cv::Mat_<float>(6, 1);//教科書運動パラメータ

    PT_kosuu=0;
	    std::cout <<"test0"<<std::endl;


    //最小二乗法を用いた外部パラメータの算出(日高手法 特徴点)-----------------------------------------------------------------------------
    /*for(int i=0;i<depth_point_curr_ok;i++){
      PT_At0(PT_kosuu,0)=-1,  PT_At0(PT_kosuu,1)=0,   PT_At0(PT_kosuu,2)=0,   PT_At0(PT_kosuu,3)=0,   PT_At0(PT_kosuu,4)=-camera_point_p[i].z,    PT_At0(PT_kosuu,5)=0;
      PT_At0(PT_kosuu+1,0)=0, PT_At0(PT_kosuu+1,1)=-1,PT_At0(PT_kosuu+1,2)=0, PT_At0(PT_kosuu+1,3)=0, PT_At0(PT_kosuu+1,4)=0,                     PT_At0(PT_kosuu+1,5)=0;
      PT_At0(PT_kosuu+2,0)=0, PT_At0(PT_kosuu+2,1)=0, PT_At0(PT_kosuu+2,2)=-1,PT_At0(PT_kosuu+2,3)=0, PT_At0(PT_kosuu+2,4)=camera_point_p[i].x,   PT_At0(PT_kosuu+2,5)=0;

      PT_Et0(PT_kosuu,0)=camera_point_c[i].x-camera_point_p[i].x;
      PT_Et0(PT_kosuu+1,0)=camera_point_c[i].y-camera_point_p[i].y;
      PT_Et0(PT_kosuu+2,0)=camera_point_c[i].z-camera_point_p[i].z;

      PT_kosuu=PT_kosuu+3;
    }*/
  }

  //リサイズ前定義
  cv::Mat_<float>Pt0=cv::Mat_<float>(depth_point_curr_ok*2, 1);
  cv::Mat_<float>Ot0_1=cv::Mat_<float>(depth_point_curr_ok*2, 6);
  cv::Mat_<float>At0=cv::Mat_<float>(depth_point_curr_ok*3, 6);
  cv::Mat_<float>Et0=cv::Mat_<float>(depth_point_curr_ok*3, 1);
  cv::Mat_<float>AWt0=cv::Mat_<float>(depth_point_curr_ok*3, 4);//誤差共分散を求めるときに使用する式AW
  cv::Mat_<float>delta0=cv::Mat_<float>(depth_point_curr_ok*3, 1);//共分散deltaの内部要素
  float Xp,Yp;
  float deltaX=0,deltaY=0,deltaZ=0,Ave_pointX=0,Ave_pointY=0,Ave_pointZ=0,Ave_point_prvX=0,Ave_point_prvY=0,Ave_point_prvZ=0;
  
  PTCLkosuu=0;
  //最小二乗法を用いた外部パラメータの算出(ロボットビジョン教科書手法)-----------------------------------------------------------------------------
  if(kaisu!=0){
      if(KAL==false){//カルマン推定前はこっち
        for(int i=0;i<depth_point_curr_ok;i++){
          xEst0_points_prev_cl[i](0,0)=camera_point_p[i].x;//一つ前に観測した特徴点のカメラ座標
          xEst0_points_prev_cl[i](1,0)=camera_point_p[i].y;
          xEst0_points_prev_cl[i](2,0)=camera_point_p[i].z;
          xEst0_points_cl[i](0,0)=camera_point_c[i].x;//今の特徴点の観測カメラ座標
          xEst0_points_cl[i](1,0)=camera_point_c[i].y;
          xEst0_points_cl[i](2,0)=camera_point_c[i].z;
          pixel_points_cl[i](0,0)=points_curr[i].x - camera_info.K[2];//今観測した特徴点の正規化画像座標(カルマン用)
          pixel_points_cl[i](1,0)=points_curr[i].y - camera_info.K[5];

           //xz平面上を移動する条件下の時
          At0(PTCLkosuu,0)=-1,  At0(PTCLkosuu,1)=0,   At0(PTCLkosuu,2)=0,   At0(PTCLkosuu,3)=0,   At0(PTCLkosuu,4)=-xEst0_points_prev_cl[i](2,0),  At0(PTCLkosuu,5)=0;
          At0(PTCLkosuu+1,0)=0, At0(PTCLkosuu+1,1)=-1,At0(PTCLkosuu+1,2)=0, At0(PTCLkosuu+1,3)=0, At0(PTCLkosuu+1,4)=0,                      At0(PTCLkosuu+1,5)=0;
          At0(PTCLkosuu+2,0)=0, At0(PTCLkosuu+2,1)=0, At0(PTCLkosuu+2,2)=-1,At0(PTCLkosuu+2,3)=0, At0(PTCLkosuu+2,4)=xEst0_points_prev_cl[i](0,0), At0(PTCLkosuu+2,5)=0;

          Et0(PTCLkosuu,0)=xEst0_points_cl[i](0,0)-xEst0_points_prev_cl[i](0,0);
          Et0(PTCLkosuu+1,0)=xEst0_points_cl[i](1,0)-xEst0_points_prev_cl[i](1,0);
          Et0(PTCLkosuu+2,0)=xEst0_points_cl[i](2,0)-xEst0_points_prev_cl[i](2,0);

          //誤差共分散を求めるときに使用する式AW
          AWt0(PTCLkosuu,0)=-1,   AWt0(PTCLkosuu,1)=0,    AWt0(PTCLkosuu,2)=0,    AWt0(PTCLkosuu,3)=-xEst0_points_prev_cl[i](2,0);
          AWt0(PTCLkosuu+1,0)=0,  AWt0(PTCLkosuu+1,1)=-1, AWt0(PTCLkosuu+1,2)=0,  AWt0(PTCLkosuu+1,3)=0;
          AWt0(PTCLkosuu+2,0)=0,  AWt0(PTCLkosuu+2,1)=0,  AWt0(PTCLkosuu+2,2)=-1, AWt0(PTCLkosuu+2,3)=xEst0_points_prev_cl[i](0,0);
            
          //共分散の内部要素(観測値ー前の推定値)
          delta0(PTCLkosuu,0)=(xEst0_points_cl[i](0,0)-xEst0_points_prev_cl[i](0,0))*(xEst0_points_cl[i](0,0)-xEst0_points_prev_cl[i](0,0));
          delta0(PTCLkosuu+1,0)=(xEst0_points_cl[i](1,0)-xEst0_points_prev_cl[i](1,0))*(xEst0_points_cl[i](1,0)-xEst0_points_prev_cl[i](1,0));
          delta0(PTCLkosuu+2,0)=(xEst0_points_cl[i](2,0)-xEst0_points_prev_cl[i](2,0))*(xEst0_points_cl[i](2,0)-xEst0_points_prev_cl[i](2,0));

          PTCLkosuu=PTCLkosuu+3;
        }
      }
      else{
        for(int i=0;i<depth_point_curr_ok;i++){
          xEst0_points_prev_cl[i](0,0)=xEst_points_prev_cl[i](0,0);//一つ前に観測した特徴点のカメラ座標
          xEst0_points_prev_cl[i](1,0)=xEst_points_prev_cl[i](1,0);
          xEst0_points_prev_cl[i](2,0)=xEst_points_prev_cl[i](2,0);
          xEst0_points_cl[i](0,0)=camera_point_c[i].x;//今の特徴点の観測カメラ座標
          xEst0_points_cl[i](1,0)=camera_point_c[i].y;
          xEst0_points_cl[i](2,0)=camera_point_c[i].z;
          pixel_points_cl[i](0,0)=points_curr[i].x - camera_info.K[2];//今観測した特徴点の正規化画像座標(カルマン用)
          pixel_points_cl[i](1,0)=points_curr[i].y - camera_info.K[5];

           //xz平面上を移動する条件下の時
          At0(PTCLkosuu,0)=-1,  At0(PTCLkosuu,1)=0,   At0(PTCLkosuu,2)=0,   At0(PTCLkosuu,3)=0,   At0(PTCLkosuu,4)=-xEst0_points_prev_cl[i](2,0),  At0(PTCLkosuu,5)=0;
          At0(PTCLkosuu+1,0)=0, At0(PTCLkosuu+1,1)=-1,At0(PTCLkosuu+1,2)=0, At0(PTCLkosuu+1,3)=0, At0(PTCLkosuu+1,4)=0,                      At0(PTCLkosuu+1,5)=0;
          At0(PTCLkosuu+2,0)=0, At0(PTCLkosuu+2,1)=0, At0(PTCLkosuu+2,2)=-1,At0(PTCLkosuu+2,3)=0, At0(PTCLkosuu+2,4)=xEst0_points_prev_cl[i](0,0), At0(PTCLkosuu+2,5)=0;

          Et0(PTCLkosuu,0)=xEst0_points_cl[i](0,0)-xEst0_points_prev_cl[i](0,0);
          Et0(PTCLkosuu+1,0)=xEst0_points_cl[i](1,0)-xEst0_points_prev_cl[i](1,0);
          Et0(PTCLkosuu+2,0)=xEst0_points_cl[i](2,0)-xEst0_points_prev_cl[i](2,0);

          //誤差共分散を求めるときに使用する式AW
          AWt0(PTCLkosuu,0)=-1,   AWt0(PTCLkosuu,1)=0,    AWt0(PTCLkosuu,2)=0,    AWt0(PTCLkosuu,3)=-xEst0_points_prev_cl[i](2,0);
          AWt0(PTCLkosuu+1,0)=0,  AWt0(PTCLkosuu+1,1)=-1, AWt0(PTCLkosuu+1,2)=0,  AWt0(PTCLkosuu+1,3)=0;
          AWt0(PTCLkosuu+2,0)=0,  AWt0(PTCLkosuu+2,1)=0,  AWt0(PTCLkosuu+2,2)=-1, AWt0(PTCLkosuu+2,3)=xEst0_points_prev_cl[i](0,0);
            
          //共分散の内部要素(観測値ー前の推定値)
          delta0(PTCLkosuu,0)=(xEst0_points_cl[i](0,0)-xEst0_points_prev_cl[i](0,0))*(xEst0_points_cl[i](0,0)-xEst0_points_prev_cl[i](0,0));
          delta0(PTCLkosuu+1,0)=(xEst0_points_cl[i](1,0)-xEst0_points_prev_cl[i](1,0))*(xEst0_points_cl[i](1,0)-xEst0_points_prev_cl[i](1,0));
          delta0(PTCLkosuu+2,0)=(xEst0_points_cl[i](2,0)-xEst0_points_prev_cl[i](2,0))*(xEst0_points_cl[i](2,0)-xEst0_points_prev_cl[i](2,0));

          PTCLkosuu=PTCLkosuu+3;
        }
      }
      //for(int i=0;i<depth_point_curr_ok;i++){
      //    std::cout << "xEst0_points_prev_cl["<<i<<"]=\n"<<xEst0_points_prev_cl[i]<< std::endl;
      //    std::cout << "xEst0_points_cl["<<i<<"]=\n"<<xEst0_points_cl[i]<< std::endl;
      //}
      //特徴点1を使用した外部パラメータ推定)
      //std::cout <<"特徴点1を使用した外部パラメータ推定"<<std::endl;

      //リサイズ用行列定義
      cv::Mat_<float>At=cv::Mat_<float>(PTCLkosuu, 6);
      cv::Mat_<float>Et=cv::Mat_<float>(PTCLkosuu, 1);
      cv::Mat_<float>AWt=cv::Mat_<float>(PTCLkosuu, 4);//誤差共分散を求めるときに使用する式AW
      cv::Mat_<float>delta=cv::Mat_<float>(PTCLkosuu, 1);//共分散deltaの内部要素

      cv::Mat_<float>Tt=cv::Mat_<float>(6, 1);//教科書運動パラメータ

      At0.rowRange(0, PTCLkosuu).copyTo(At.rowRange(0, PTCLkosuu));
	    Et0.rowRange(0, PTCLkosuu).copyTo(Et.rowRange(0, PTCLkosuu));
      AWt0.rowRange(0, PTCLkosuu).copyTo(AWt.rowRange(0, PTCLkosuu));
	    delta0.rowRange(0, PTCLkosuu).copyTo(delta.rowRange(0, PTCLkosuu));

      ALL_At=cv::Mat_<float>(PTCLkosuu, 6);
      ALL_Et=cv::Mat_<float>(PTCLkosuu, 1);
      At.rowRange(0, PTCLkosuu).copyTo(ALL_At.rowRange(0, PTCLkosuu));
	    Et.rowRange(0, PTCLkosuu).copyTo(ALL_Et.rowRange(0, PTCLkosuu));
    
      //すべてのマーカー座標を利用して最小二乗法外部パラメータ推定(初回)
      ALL_Tt=ALL_At.inv(cv::DECOMP_SVD)*ALL_Et;
      std::cout <<"ALL_Tt=\n"<<ALL_Tt<< std::endl;//推定外部パラメータ

      ALL_Ft = cv::Mat_<float>(3, 3);//回転行列(日高手法)
      ALL_Ft(0,0)=1,             ALL_Ft(0,1)=-ALL_Tt(5,0),  ALL_Ft(0,2)=-ALL_Tt(4,0);
      ALL_Ft(1,0)=-ALL_Tt(5,0),  ALL_Ft(1,1)=1,             ALL_Ft(1,2)=ALL_Tt(3,0),
      ALL_Ft(2,0)=ALL_Tt(4,0),   ALL_Ft(2,1)=-ALL_Tt(3,0),  ALL_Ft(2,2)=1;
      std::cout <<"回転行列(日高手法)ALL_Ft=\n"<<ALL_Ft<< std::endl;//回転行列

      ALL_Vt = cv::Mat_<float>(4, 1);//並進ベクトル+y軸回転(日高手法)
      ALL_Vt(0,0)=ALL_Tt(0,0);//Vx
      ALL_Vt(1,0)=ALL_Tt(1,0);//Vy
      ALL_Vt(2,0)=ALL_Tt(2,0);//Vz
      ALL_Vt(3,0)=ALL_Tt(4,0);//Ωy
      std::cout <<"並進ベクトル+y軸回転(日高手法)ALL_Vt=\n"<<ALL_Vt<< std::endl;//並進ベクトル

      //カルマン初期設定(カルマン初回のみ実行)
      if(KAL==false){
         Gt[depth_point_curr_ok]=cv::Mat_<float>(3, 4);
         Mt_cl[depth_point_curr_ok] = cv::Mat_<float>::zeros(3, 3);//誤差共分散(予測式)
         Yt_cl[depth_point_curr_ok] = cv::Mat_<float>::zeros(2, 1);//観測残差
         Ht_cl[depth_point_curr_ok] = cv::Mat_<float>::zeros(2, 3);//Htヤコビアン行列
         Ut_cl[depth_point_curr_ok] = cv::Mat_<float>::zeros(2, 1);//共分散
         St[depth_point_curr_ok] = cv::Mat_<float>::zeros(2, 2);//共分散
         Kt_cl[depth_point_curr_ok] = cv::Mat_<float>::zeros(3, 2);//カルマンゲイン
         u_ = (cv::Mat_<float>(2, 2) <<
            0.083, 0,
            0, 0.083);
      }

      //カルマンフィルタでマーカーの座標修正(座標推定)
      //予測ステップ----------------------------------------------------------------------------------------------------
      //状態モデルの予測式
      for(int i=0;i<depth_point_curr_ok;i++){
        //if(xEst0_clP[i]==1){//要素がある時のみ実行

          Gt[i](0,0)=-1,Gt[i](0,1)=0, Gt[i](0,2)=0, Gt[i](0,3)=-xEst0_points_prev_cl[i](2,0);
          Gt[i](1,0)=0, Gt[i](1,1)=-1,Gt[i](1,2)=0, Gt[i](1,3)=-0;
          Gt[i](2,0)=0, Gt[i](2,1)=0, Gt[i](2,2)=-1,Gt[i](2,3)=xEst0_points_prev_cl[i](0,0);
          //std::cout <<"Gt["<<i<<"]=\n"<<Gt[i]<< std::endl;
          //std::cout <<"xEst0_points_prev_cl["<<i<<"]=\n"<<xEst0_points_prev_cl[i]<< std::endl;
          //std::cout <<"ALL_Vt=\n"<<ALL_Vt<< std::endl;
          //std::cout <<"ALL_Ft=\n"<<ALL_Ft<< std::endl;


          xEst_points_cl[i]=ALL_Ft*xEst0_points_prev_cl[i]+Gt[i]*ALL_Vt;//カルマン初回動作はこっち
          //std::cout <<"xEst_points_cl["<<i<<"]=\n"<<xEst_points_cl[i]<< std::endl;
        //}
      }
      //誤差共分散の予測式-----------------------------------
      cv::Mat eye = cv::Mat_<float>::eye(PTCLkosuu, PTCLkosuu);// 単位行列kosuu, 4
      cv::Mat_<float>eyetemp=cv::Mat_<float>(1, PTCLkosuu);
      cv::Mat_<float>deltaeye=cv::Mat_<float>(PTCLkosuu, PTCLkosuu);
      cv::Mat_<float>AWt_1=cv::Mat_<float>(PTCLkosuu, 4);
      cv::Mat_<float>Wt = cv::Mat_<float>::zeros(4, 4);

      int kosuuNO;
      for(int i=0;i<PTCLkosuu;i++){//共分散の対角化
        eye.rowRange(i, i+1).copyTo(eyetemp.rowRange(0, 1));
        //if(i%3==0){
        //  if(MC_point[i][2]>3
        //}
        if(i%3==0){eyetemp=eyetemp*delta(i,0)*30;}
        else if(i%3==1){eyetemp=eyetemp*delta(i,0)*30;}
        else if(i%3==2){eyetemp=eyetemp*delta(i,0)*30;}
        eyetemp.rowRange(0, 1).copyTo(deltaeye.rowRange(i, i+1));
      }
      //std::cout <<"deltaeye=\n"<<deltaeye<< std::endl;

      AWt_1=AWt.inv(cv::DECOMP_SVD);//擬似逆行列
      //std::cout <<"AWt_1=\n"<<AWt_1<< std::endl;
      Wt=AWt_1*deltaeye*AWt_1.t();
      //std::cout <<"Wt=\n"<<Wt<< std::endl;

      for(int i=0;i<depth_point_curr_ok;i++){
        //if(xEst0_clP[i]==1){//要素がある時のみ実行
          if(KAL==false){//カルマン初回実行時はこっち
            Mt_cl[i]= (cv::Mat_<float>(3, 3) <<
              0.1, 0, 0,
              0, 0.1, 0,
              0, 0, 0.1);//Mt_cl[i]の初期値
          }
          else{Mt_cl[i]=ALL_Ft*Pt_cl[i]*ALL_Ft.t()+Gt[i]*Wt*Gt[i].t();}
          //Mt_cl[i]=ALL_Ft*Pt_cl[i]*ALL_Ft.t()+Gt[i]*Wt*Gt[i].t();
          //Mt_cl[i]=Ft*Pt_cl[i]*Ft.t()+Gt[i]*Wt*Gt[i].t();
          //std::cout <<"Pt_cl[i="<<i<<"]=\n"<<Pt_cl[i]<< std::endl;
          //std::cout <<"Mt_cl["<<i<<"]=\n"<<Mt_cl[i]<< std::endl;//誤差共分散の予測式Mt
        //}
      }
      //更新ステップ----------------------------------------------------------------------
      //観測残差
      for(int i=0;i<depth_point_curr_ok;i++){
        //if(xEst0_clP[i]==1){//要素がある時のみ実行
          Ht_cl[i]= (cv::Mat_<float>(2, 3) <<
            camera_info.K[0]/xEst0_points_prev_cl[i](2,0), 0,                                      -camera_info.K[0]*(xEst0_points_prev_cl[i](0,0)/(xEst0_points_prev_cl[i](2,0)*xEst0_points_prev_cl[i](2,0))),
            0,                                      camera_info.K[0]/xEst0_points_prev_cl[i](2,0), -camera_info.K[0]*(xEst0_points_prev_cl[i](1,0)/(xEst0_points_prev_cl[i](2,0)*xEst0_points_prev_cl[i](2,0))));

          Ut_cl[i]= (cv::Mat_<float>(2, 1) <<
            camera_info.K[0]*(xEst0_points_prev_cl[i](0,0)/xEst0_points_prev_cl[i](2,0)),
            camera_info.K[0]*(xEst0_points_prev_cl[i](1,0)/xEst0_points_prev_cl[i](2,0)));

          //std::cout <<"Ht_cl["<<i<<"]=\n"<<Ht_cl[i]<< std::endl;
          //std::cout <<"Ut_cl["<<i<<"]=\n"<<Ut_cl[i]<< std::endl;

          Yt_cl[i]=pixel_points_cl[i]-(Ht_cl[i]*xEst_points_cl[i]+Ut_cl[i]);
          //std::cout <<"pixel["<<i<<"][0]="<<pixel[i][0]<< std::endl;
          //std::cout <<"pixel["<<i<<"][1]="<<pixel[i][1]<< std::endl;
          //std::cout <<"pixel_points_cl["<<i<<"]=\n"<<pixel_points_cl[i]<< std::endl;
          //std::cout <<"xEst_points_cl["<<i<<"]=\n"<<xEst_points_cl[i]<< std::endl;
          //std::cout <<"Ht_cl["<<i<<"]*xEst_points_cl["<<i<<"]=\n"<<Ht_cl[i]*xEst_points_cl[i]<< std::endl;
          //std::cout <<"Ht_cl["<<i<<"]*xEst_points_cl["<<i<<"]+Ut_cl["<<i<<"]=\n"<<Ht_cl[i]*xEst_points_cl[i]+Ut_cl[i]<< std::endl;

          //std::cout <<"観測残差:Yt_cl["<<i<<"]=\n"<<Yt_cl[i]<< std::endl;//観測残差

          St[i]=Ht_cl[i]*Mt_cl[i]*Ht_cl[i].t()+u_;
          //std::cout <<"観測残差の共分散:St["<<i<<"]=\n"<<St[i]<< std::endl;

          Pt_cl[i]=Mt_cl[i].inv()+(Ht_cl[i].t()*u_.inv()*Ht_cl[i]);
          Pt_cl[i]=Pt_cl[i].inv();
          //std::cout <<"誤差共分散の更新:Pt_cl["<<i<<"]=\n"<<Pt_cl[i]<< std::endl;//誤差共分散の更新

          //Kt_cl[i]=Pt_cl[i]*Ht_cl[i].t()*u_.inv();
          Kt_cl[i]=Pt_cl[i]*Ht_cl[i].t()*St[i].inv();
          //std::cout <<"u_.inv()=\n"<<u_.inv()<< std::endl;
          //std::cout <<"カルマンゲイン:Kt_cl["<<i<<"]=\n"<<Kt_cl[i]<< std::endl;//カルマンゲイン

          xEst_points_cl[i]=xEst_points_cl[i]+(Kt_cl[i]*Yt_cl[i]);
          //std::cout <<"状態モデルの更新:xEst_points_cl["<<i<<"]=\n"<<xEst_points_cl[i]<< std::endl;//状態モデルの更新

          xEst_points_prev_cl[i]=xEst_points_cl[i];//現在の推定を保存
          //xEst_points_prev_clP[i]=1;
          //std::cout <<"xEst_points_prev_clP["<<i<<"]="<<xEst_points_prev_clP[i]<< std::endl;
          KAL=true;//次の動作の時カルマンの推定結果と観測値の追跡を確認する
        //}
        //新しくマーカーが観測された時
        //else if(MC_point[i][3]==1&&xEst_points_prev_clP[i]==0){
        //  xEst_points_prev_clP[i]=1;
        //  std::cout <<"新しくマーカーが観測された時 xEst_points_prev_clP["<<i<<"]="<<xEst_points_prev_clP[i]<< std::endl;
        //}
        ////更新データが存在しない時
        //else{xEst_points_prev_clP[i]=0;
        //  //std::cout <<"ELSE xEst_points_prev_clP["<<i<<"]="<<xEst_points_prev_clP[i]<< std::endl;
        //}
      }
  }//if(kaisu!=0)→end(初回は動作しない)
  

    if(kaisu>1){
      //ALL_Tt=(ALL_Tt+ALL_prve_Tt)*realsec/2;
      //camera_prmX = camera_prmX+ALL_Tt(5,0)*camera_prmY-ALL_Tt(4,0)*camera_prmZ+ALL_Tt(2,0);
      //camera_prmY = -ALL_Tt(5,0)*camera_prmX+camera_prmY+ALL_Tt(3,0)*camera_prmZ+ALL_Tt(0,0);
      //camera_prmZ = ALL_Tt(4,0)*camera_prmX-ALL_Tt(5,0)*camera_prmY+camera_prmZ+ALL_Tt(1,0);

      camera_prmX = -camera_prmX-ALL_Tt(5,0)*camera_prmY+ALL_Tt(4,0)*camera_prmZ+ALL_Tt(2,0);
      camera_prmY = ALL_Tt(5,0)*camera_prmX-camera_prmY-ALL_Tt(3,0)*camera_prmZ+ALL_Tt(0,0);
      camera_prmZ = -ALL_Tt(4,0)*camera_prmX+ALL_Tt(5,0)*camera_prmY-camera_prmZ+ALL_Tt(1,0);


     //tf(map camera_base間のlink)-----------------------------------------------------------------------------------------
      //ここがカメラの姿勢部分
      std::string MaptoCamera_Base_frame = "MaptoCamera_Base_link";
      //微小区間回転行列
      double RollX=ALL_Ft(2,1),PitchY=ALL_Ft(0,2),YawZ=ALL_Ft(1,0);

      //カメラ位置
      camera_base_pose.position.x = camera_prmZ;//赤(tf:X,画像:Z)
      camera_base_pose.position.y = camera_prmX;//緑(tf:Y,画像:X)
      camera_base_pose.position.z = 0;//青(tf:Z,画像:Y)
      camera_base_pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(YawZ,RollX,PitchY);

      outputfileALL_Vt<<"X:ALL_Vt(2,0)="<<ALL_Vt(2,0)<<" Y:ALL_Vt(0,0)="<<ALL_Vt(0,0)<<" Z:ALL_Vt(1,0)="<<ALL_Vt(1,0)<<"\n";

      static tf::TransformBroadcaster br_camera_base_pose;
      tf::Transform camera_base_transform;
      poseMsgToTF(camera_base_pose, camera_base_transform);
      br_camera_base_pose.sendTransform(tf::StampedTransform(camera_base_transform, ros::Time::now(), source_frame, MaptoCamera_Base_frame));
      std::cout <<"camera_base_pose.position.x="<<camera_base_pose.position.x<< std::endl;
      std::cout <<"camera_base_pose.position.y="<<camera_base_pose.position.y<< std::endl;
      std::cout <<"camera_base_pose.position.z="<<camera_base_pose.position.z<< std::endl;
      std::cout <<"camera_base_pose.orientation.x="<<camera_base_pose.orientation.x<< std::endl;
      std::cout <<"camera_base_pose.orientation.y="<<camera_base_pose.orientation.y<< std::endl;
      std::cout <<"camera_base_pose.orientation.z="<<camera_base_pose.orientation.z<< std::endl;
      std::cout <<"camera_base_pose.orientation.w="<<camera_base_pose.orientation.w<< std::endl;
      outputfileTF<<"camera_base_pose.position.x="<<camera_base_pose.position.x<<" camera_base_pose.position.y="<<camera_base_pose.position.y<<" camera_base_pose.position.z="<<camera_base_pose.position.z<<"\n";

      //pose.header.stamp = ros::Time::now();
      //pose.header.frame_id = source_frame;
      ////pose.header.frame_id = MaptoCamera_Base_frame;
      //pose.pose.position = camera_base_pose.position;
      //pose.pose.orientation = camera_base_pose.orientation;
      //path.header.stamp = ros::Time::now();
      //path.header.frame_id = source_frame;
      ////path.header.frame_id = MaptoCamera_Base_frame;
      //path.poses.push_back(pose);
      //pub_plan.publish(path);
      //tf(camera_base camera_link間のlink)-----------------------------------------------------------------------------------------
      geometry_msgs::Pose Camera_BasetoCamera_Link_pose;

      //std::string Camera_BasetoCamera_Link_frame = "Camera_BasetoCamera_Link_link";
      Camera_BasetoCamera_Link_pose.position.x = 0;
      Camera_BasetoCamera_Link_pose.position.y = 0;
      Camera_BasetoCamera_Link_pose.position.z = 0.67;//ここに実際のカメラの高さを入れる
      Camera_BasetoCamera_Link_pose.orientation.w = 1.0;
      //Camera_BasetoCamera_Link_pose.position.z = 0.9;//ここに実際のカメラの高さを入れる
      //Camera_BasetoCamera_Link_pose.orientation.x = quaternionVal.x;
      //Camera_BasetoCamera_Link_pose.orientation.y = quaternionVal.y;
      //Camera_BasetoCamera_Link_pose.orientation.z = quaternionVal.z;
      //Camera_BasetoCamera_Link_pose.orientation.w = quaternionVal.w;
      //Camera_BasetoCamera_Link_pose.orientation.w = quaternionVal.w;
      static tf::TransformBroadcaster br_Camera_BasetoCamera_Link_pose;

      tf::Transform Camera_BasetoCamera_transform;
      poseMsgToTF(Camera_BasetoCamera_Link_pose, Camera_BasetoCamera_transform);
      br_Camera_BasetoCamera_Link_pose.sendTransform(tf::StampedTransform(Camera_BasetoCamera_transform, ros::Time::now(), MaptoCamera_Base_frame, "camera_link"));
    }
  

  else{
    camera_base_pose.position.x = 0;
    camera_base_pose.position.y = 0;
    camera_base_pose.position.z = 0;
    camera_base_pose.orientation.x=0;
    camera_base_pose.orientation.y=0;
    camera_base_pose.orientation.z=0;
    camera_base_pose.orientation.w=0;
  }
  //経路描写-------------------------------------------------------------
    pose.header.stamp = ros::Time::now();
    pose.header.frame_id = source_frame;
    pose.pose.position = camera_base_pose.position;
    pose.pose.orientation = camera_base_pose.orientation;
    path.header.stamp = ros::Time::now();
    path.header.frame_id = source_frame;
    path.poses.push_back(pose);
    pub_plan.publish(path);
  

  // 画面表示
  cv::imshow(win_src, image);
  cv::imshow(win_dst, imageCopy);
  //cv::imshow(win_depth, depthimage);


  cv::swap(image_curr, image_prev);// image_curr を image_prev に移す（交換する）

  if(swap_on ==true){//初回プログラム実行時はswapしない(データの浅いコピーの影響を考慮)
    points_kosuu=0;
    std::cout <<"points_kosuu="<<points_kosuu<<std::endl;
    std::cout <<"depth_point_curr_ok="<<depth_point_curr_ok<<std::endl;
    std::cout <<"depth_points_currA_ok="<<depth_points_currA_ok<<std::endl;

    ALL_points_curr.resize(depth_point_curr_ok+depth_points_currA_ok);
    ALL_camera_point_c.resize(depth_point_curr_ok+depth_points_currA_ok);
    for(int i=0;i<depth_point_curr_ok;i++){
      ALL_points_curr[points_kosuu]=points_curr[i];//追跡可能点に検出点をプラスする
      ALL_camera_point_c[points_kosuu]=camera_point_c[i];
      points_kosuu=points_kosuu+1;
    }
    //同じ場所のpointを消したい
    for(int i=0;i<depth_points_currA_ok;i++){
      PLAS_POINT=true;
      for(int j=0;j<depth_point_curr_ok;j++){
        //検出1点に対してすべての追跡点で検査→１つでもかぶる点があるならその検出点を消去
        if(abs(points_curr[j].x-points_currA[i].x)<1.5&&abs(points_curr[j].y-points_currA[i].y)<1.5){//差が0.1以下なら同じ点とみなして消去
          PLAS_POINT=false;//更新オフ
          continue;//かぶる点が検出された時for分終了 
        }
      }
      if(PLAS_POINT==true){//かぶる点が無いとき追加動作実行
        ALL_points_curr[points_kosuu]=points_currA[i];//追跡可能点に検出点をプラスする
        ALL_camera_point_c[points_kosuu]=camera_point_cA[i];
        points_kosuu=points_kosuu+1;
      }
    }
    cv::swap(ALL_points_curr, points_prev);//二次元画像座標を保存(points_curr→points_prev)(追跡可能点をpoints_prevに保存)
    cv::swap(ALL_camera_point_c, camera_point_p);//三次元カメラ座標を保存(camera_point_c→camera_point_p)
  }


  wall_prev=wall_duration;
  kaisu++;
  KAL_Wait++;
  //startTime=endTime;
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
	message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nhSub, "/camera/color/image_raw", 1);//センサーメッセージを使うときは対応したヘッダーが必要
	message_filters::Subscriber<sensor_msgs::Image> depth_sub(nhSub, "/camera/aligned_depth_to_color/image_raw", 1);
	message_filters::Subscriber<sensor_msgs::CameraInfo> info_sub(nhSub, "/camera/color/camera_info", 1);

  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,sensor_msgs::Image,sensor_msgs::CameraInfo> MySyncPolicy;	
	message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10),rgb_sub, depth_sub, info_sub);
	sync.registerCallback(boost::bind(&callback,_1, _2, _3));

  ros::NodeHandle nhPub;
  pub_plan = nhPub.advertise<nav_msgs::Path>("/get_multi_path",1000);

  
	ros::spin();//トピック更新待機
			
	return 0;
}






