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
int kosuu=0,PT_kosuu=0,PT2_kosuu=0;

bool KAL = false;//カルマン推定データ初回回避用(カルマン動作した時TRUE)
bool reset = true;//初回プログラム用（特徴点検出&特徴点再検出用)
bool swap_on = true;//初回プログラム実行時はswapしない(データの浅いコピーの影響を考慮)
bool reset2 = true;//初回プログラム用（特徴点検出&特徴点再検出用)
bool swap_on2 = true;//初回プログラム実行時はswapしない(データの浅いコピーの影響を考慮)

cv::Mat image_curr,image_prev,img_dst,img_dst1;//画像定義
vector<cv::Point2f> points_prev, points_curr;//特徴点定義
vector<cv::Point2f> points_prev2, points_curr2;//特徴点定義
vector<cv::Point3f> camera_point_p,camera_point_c;//特徴点定義
vector<cv::Point3f> camera_point_p2,camera_point_c2;//特徴点定義
float depth_point_prev[300],camera_point_prev[300][3],depth_point_curr[300],camera_point_curr[300][3];
float depth_point_prev2[300],camera_point_prev2[300][3],depth_point_curr2[300],camera_point_curr2[300][3];
cv::Mat_<float> F_Xp = cv::Mat_<float>(4, 1);//外部パラメータ（日高手法 特徴点)
cv::Mat_<float> F_XpPLAS = cv::Mat_<float>::zeros(4, 1);
cv::Mat_<float>PT_At0,PT_Et0;
cv::Mat_<float> PT_Vt;//外部パラメータ並進ベクトル(特徴点)
cv::Mat_<float> PT_Ft;//外部パラメータ回転行列(特徴点)
cv::Mat_<float> PT_Tt=cv::Mat_<float>(6, 1);//教科書運動パラメータ(特徴点のみで推定)
std::array<float, 9> External_R;//外部パラメータ(回転行列のみ)(クォータニオン変換用)

cv::Mat_<float>PT2_At0,PT2_Et0;
cv::Mat_<float> PT2_Vt;//外部パラメータ並進ベクトル(特徴点)
cv::Mat_<float> PT2_Ft;//外部パラメータ回転行列(特徴点)
cv::Mat_<float> PT2_Tt=cv::Mat_<float>(6, 1);//教科書運動パラメータ(特徴点のみで推定)

cv::Mat_<float>ALL_At;//外部パラメータ推定
cv::Mat_<float>ALL_Et;//外部パラメータ推定

//マーカー関連
sensor_msgs::CameraInfo camera_info;//CameraInfo受け取り用
static struct_slam::marker_tf marker_tf;//観測マーカーのメッセージ
//float Rotation;//外部パラメータ(仮)
//float point_prve[50][4];//一つ前のカメラ座標
//cv::Mat_<float> Xp = cv::Mat_<float>(4, 1);//外部パラメータ（日高手法)
//cv::Mat_<float> XpPLAS = cv::Mat_<float>::zeros(4, 1);
//float pixel[50][2],depth[100],point[50][4],x,y,r2,f,ux,uy;//画像→カメラ座標変換

ros::Time ros_begin;//プログラム時間
ros::WallTime wall_begin = ros::WallTime::now();//プログラム開始時の時間
ros::WallDuration wall_prev;//一つ前の時間
ros::WallDuration wall_systemtime;//サンプリング時間
struct timeval startTime, endTime;  // 構造体宣言
float realsec;//サンプリング時間（C++)

int ALLMarker=40;//全マーカー個数
//カルマンフィルタ
//cv::Mat_<float> Vt;//外部パラメータ並進ベクトル
//cv::Mat_<float> Ft;//外部パラメータ回転行列
cv::Mat_<float> ALL_Vt;//外部パラメータ並進ベクトル(マーカー+特徴点)
cv::Mat_<float> ALL_Ft;//外部パラメータ回転行列(マーカー+特徴点)
cv::Mat_<float> ALL_Tt=cv::Mat_<float>(6, 1);//教科書運動パラメータ

////カルマン定義
//cv::Mat_<float>xEst0_cl[40]=cv::Mat_<float>(3, 1);//今観測したマーカー座標(カルマン用)
//cv::Mat_<float>xEst0_prev_cl[40]=cv::Mat_<float>(3, 4);//一つ前に観測したマーカー座標（カルマン用)
//cv::Mat_<float>xEst_cl[40]=cv::Mat_<float>(3, 1);//今推定したマーカー座標(カルマン用)
//cv::Mat_<float>xEst_prev_cl[40]=cv::Mat_<float>(3, 1);//一つ前に推定したマーカー座標（カルマン用)
//cv::Mat_<float>pixel_cl[40]=cv::Mat_<float>(2, 1);//追跡可能なマーカーの画像座標（カルマン用)
//int xEst0_prev_clP[40],xEst0_clP[40],xEst_prev_clP[40];//要素があるかどうか(マッチング用)
//cv::Mat_<float>Gt[40]=cv::Mat_<float>(3, 3);
//cv::Mat_<float> Mt_cl[40] = cv::Mat_<float>::eye(3, 3);//誤差共分散(予測式)
//cv::Mat_<float> Pt_cl[40] = cv::Mat_<float>::eye(3, 3);//誤差共分散(更新式)
//cv::Mat_<float> Yt_cl[40] = cv::Mat_<float>::zeros(2, 1);//観測残差
//cv::Mat_<float> Ht_cl[40] = cv::Mat_<float>::zeros(2, 3);//Htヤコビアン行列
//cv::Mat_<float> Ut_cl[40] = cv::Mat_<float>::zeros(2, 1);//共分散
//cv::Mat_<float> St[40] = cv::Mat_<float>::zeros(2, 2);//共分散
//cv::Mat_<float> Kt_cl[40] = cv::Mat_<float>::zeros(3, 2);//カルマンゲイン
//cv::Mat_<float> u_ = cv::Mat_<float>(2, 2);

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

  ////Depth画像の色付けを行なっている
  //double min,max;
  //cv::minMaxIdx(depthimage, &min, &max);
  //cv::Mat adjimg_depth;
  //// Histogram Equalization
  //float scale = 255 / (max-min);
  //depthimage.convertTo(adjimg_depth,CV_8UC1, scale, -min*scale); 
  //cv::Mat falseColorsimg_depth;
  //applyColorMap(adjimg_depth, falseColorsimg_depth, cv::COLORMAP_WINTER);//ここのcvで色を変えられる
  //cv::imshow("Out", falseColorsimg_depth);
  //image.copyTo(img_dst);//

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

    int depth_point_prev_ok=0; //depth取得可能な特徴点の数
    swap_on=false;
    //cv::goodFeaturesToTrack(今画像, 今の特徴点, 特徴点の個数, 0.01, 10, cv::Mat(), 3, 3, 0, 0.04);
		cv::goodFeaturesToTrack(image_curr, points_curr, 300, 0.01, 10, cv::Mat(), 3, 3, 0, 0.04);// 特徴点検出(グレースケール画像から特徴点検出)
		cv::cornerSubPix(image_curr, points_curr, cv::Size(10, 10), cv::Size(-1, -1), cv::TermCriteria(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, 20, 0.03));
		points_prev = points_curr;//データのコピ
    camera_point_p.resize(points_prev.size());//要素数初期設定
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
    int depth_point_curr_ok=0; //depth取得可能な特徴点の数

    cv::calcOpticalFlowPyrLK(image_prev, image_curr, points_curr, points_prev, status, err);//オプティカルフロー
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
		
	  // 特徴点を丸で描く-------------------------------------------------------------------------------------------
	  for (int i = 0; i < points_curr.size(); i++) {
      //std::cout <<"OPT後マッチングの中心座標["<<i<<"]="<<points_curr[i]<< std::endl;
      cv::circle(imageCopy, points_prev[i], 3, Scalar(255,255,255), -1, cv::LINE_AA);//一つ前の画像の座標
	    cv::circle(imageCopy, points_curr[i], 4, Scalar(0, 255, 0), -1, cv::LINE_AA);//今の座標情報
	    //cv::line(imageCopy,cv::Point(points_curr[i]),cv::Point(points_prev[i]),cv::Scalar(0,255,255), 1, cv::LINE_AA);//線を描写する
      //cv::line(imageCopy,cv::Point(points_curr[i].x,points_prev[i].y),cv::Point(points_prev[i]),cv::Scalar(255,0,0), 1, cv::LINE_AA);//線を描写する
      //cv::line(imageCopy,cv::Point(points_prev[i].x,points_curr[i].y),cv::Point(points_prev[i]),cv::Scalar(0,255,0), 1, cv::LINE_AA);//線を描写する
    
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
        depth_point_curr_ok=depth_point_curr_ok+1;//Depth取得可能の個数をカウント
      }
    }

    std::cout <<"depth_point_curr_ok="<<depth_point_curr_ok<< std::endl;
    camera_point_p.resize(depth_point_curr_ok);//Depth取得可能数でリサイズ(三次元カメラ座標)
    camera_point_c.resize(depth_point_curr_ok);//Depth取得可能数でリサイズ(三次元カメラ座標)
    points_prev.resize(depth_point_curr_ok);//Depth取得可能数でリサイズ(二次元座標)
    points_curr.resize(depth_point_curr_ok);//Depth取得可能数でリサイズ(二次元座標)

    if(camera_point_p.size()<5||k<5){//特徴点が100個以下になったら再び特徴点を検出する
      reset = true;
      std::cout <<" 特徴点再検出リセット"<<std::endl;}

    PT_At0=cv::Mat_<float>(depth_point_curr_ok*3, 6);
    PT_Et0=cv::Mat_<float>(depth_point_curr_ok*3, 1);
    PT_Tt=cv::Mat_<float>(6, 1);//教科書運動パラメータ

    PT_kosuu=0;

    //最小二乗法を用いた外部パラメータの算出(日高手法 特徴点)-----------------------------------------------------------------------------
    for(int i=0;i<depth_point_curr_ok;i++){
      PT_At0(PT_kosuu,0)=-1,  PT_At0(PT_kosuu,1)=0,   PT_At0(PT_kosuu,2)=0,   PT_At0(PT_kosuu,3)=0,   PT_At0(PT_kosuu,4)=-camera_point_p[i].z,    PT_At0(PT_kosuu,5)=0;
      PT_At0(PT_kosuu+1,0)=0, PT_At0(PT_kosuu+1,1)=-1,PT_At0(PT_kosuu+1,2)=0, PT_At0(PT_kosuu+1,3)=0, PT_At0(PT_kosuu+1,4)=0,                     PT_At0(PT_kosuu+1,5)=0;
      PT_At0(PT_kosuu+2,0)=0, PT_At0(PT_kosuu+2,1)=0, PT_At0(PT_kosuu+2,2)=-1,PT_At0(PT_kosuu+2,3)=0, PT_At0(PT_kosuu+2,4)=camera_point_p[i].x,   PT_At0(PT_kosuu+2,5)=0;

      PT_Et0(PT_kosuu,0)=camera_point_c[i].x-camera_point_p[i].x;
      PT_Et0(PT_kosuu+1,0)=camera_point_c[i].y-camera_point_p[i].y;
      PT_Et0(PT_kosuu+2,0)=camera_point_c[i].z-camera_point_p[i].z;

      PT_kosuu=PT_kosuu+3;
    }

    ////特徴点1を利用して最小二乗法外部パラメータ推定(初回)
    //PT_Tt=PT_At0.inv(cv::DECOMP_SVD)*PT_Et0;
    ////std::cout <<"PT_Tt=\n"<<PT_Tt<< std::endl;//推定外部パラメータ
    //PT_Ft = cv::Mat_<float>(3, 3);//回転行列(日高手法)
    //PT_Ft(0,0)=1,            PT_Ft(0,1)=PT_Tt(5,0),   PT_Ft(0,2)=-PT_Tt(4,0);
    //PT_Ft(1,0)=-PT_Tt(5,0),  PT_Ft(1,1)=1,             PT_Ft(1,2)=PT_Tt(3,0),
    //PT_Ft(2,0)=PT_Tt(4,0),   PT_Ft(2,1)=PT_Tt(3,0),    PT_Ft(2,2)=1;
    //PT_Vt = cv::Mat_<float>(4, 1);//並進ベクトル+y軸回転(日高手法)
    //PT_Vt(0,0)=PT_Tt(0,0);//Vx
    //PT_Vt(1,0)=PT_Tt(1,0);//Vy
    //PT_Vt(2,0)=PT_Tt(2,0);//Vz
    //PT_Vt(3,0)=PT_Tt(4,0);//Ωy
    ////std::cout <<"並進ベクトル+y軸回転(日高手法)PT_Vt=\n"<<PT_Vt<< std::endl;//並進ベクトル
  }

  //二個目特徴点検出--------------------------------------------------------------------------------------------------------------
  //一つ目前の特徴点の数と現在の特徴点の数を合わせる必要がある。ここでは追跡可能か,それぞれDepthデータ取得可能であるかでサイズ調整を行っている
	cv::cvtColor(image, image_curr, cv::COLOR_BGR2GRAY);//グレースケール化
  //初回検出プログラム-----------------------------------------------------------------------------------------------------
  if (reset2 == true) {
	  std::cout <<"初回２回目検出プログラム"<< std::endl;

    int depth_point_prev2_ok=0; //depth取得可能な特徴点の数
    swap_on2=false;
    //cv::goodFeaturesToTrack(今画像, 前画像, 特徴点の個数, 0.01, 10, cv::Mat(), 3, 3, 0, 0.04);
		cv::goodFeaturesToTrack(image_curr, points_curr2, 300, 0.01, 10, cv::Mat(), 3, 3, 0, 0.04);// 特徴点検出(グレースケール画像から特徴点検出)
		cv::cornerSubPix(image_curr, points_curr2, cv::Size(10, 10), cv::Size(-1, -1), cv::TermCriteria(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, 20, 0.03));
		points_prev2 = points_curr2;//データのコピ
    camera_point_p2.resize(points_prev2.size());//要素数初期設定
	  //std::cout <<"test2_points_prev2.size()="<<points_prev2.size()<< std::endl;
    
		for (int i = 0; i < points_prev2.size(); i++) {
			cv::circle(imageCopy, points_prev2[i], 6, Scalar(255,0,0), -1, cv::LINE_AA);
      //画像→カメラ座標変換----------------------------------------------------------------------
      depth_point_prev2[i] = depthimage.at<float>(cv::Point(points_prev2[i].x,points_prev2[i].y));

      //Depthが取得できない特徴点を削除する+Depthの外れ値を除く
      if(depth_point_prev2[i]>0.001&&depth_point_prev2[i]<10000){
        camera_point_p2[depth_point_prev2_ok].x = depth_point_prev2[i] * ((points_prev2[i].x - camera_info.K[2]) / camera_info.K[0])/1000;//メートル表示変換
        camera_point_p2[depth_point_prev2_ok].y = depth_point_prev2[i] * ((points_prev2[i].y - camera_info.K[5]) / camera_info.K[4])/1000;
        camera_point_p2[depth_point_prev2_ok].z = depth_point_prev2[i]/1000;

        //std::cout << "特徴点のカメラ座標:camera_point_p2[depth_point_prev2_ok="<<depth_point_prev2_ok<<"]={"<< camera_point_p2[depth_point_prev2_ok].x <<","<<camera_point_p2[depth_point_prev2_ok].y<<","<<camera_point_p2[depth_point_prev2_ok].z<<"}"<< std::endl;
        points_prev2[depth_point_prev2_ok] = points_prev2[i];
        depth_point_prev2_ok=depth_point_prev2_ok+1;//Depth取得可能の個数をカウント
      }
		}
    points_prev2.resize(depth_point_prev2_ok);//Depth取得可能数でリサイズ(二次元)
    camera_point_p2.resize(depth_point_prev2_ok);//Depth取得可能数でリサイズ(三次元カメラ座標)
    reset2 = false;//if文切り替え
	  //std::cout <<"初回2回目検出プログラム終了"<< std::endl;
  }
  //オプティカルフロー2-------------------------------------------------------------------------------------
  else{
	  std::cout <<"オプティカルフロー2"<< std::endl;// 特徴点追跡(二回目のフレーム)
	  vector<uchar> status2;//特徴点の数
	  vector<float> err2;
    swap_on2 = true;
    int depth_point_curr2_ok=0; //depth取得可能な特徴点の数

    cv::calcOpticalFlowPyrLK(image_prev, image_curr, points_curr2, points_prev2, status2, err2);//オプティカルフロー
	  std::cout <<"status.size()="<<status2.size()<< std::endl;//特徴点の個数

	  // 追跡できなかった特徴点をリストから削除する
	  int i, k,n,j=0;
	  for (i = k = n =0; i < status2.size(); i++){
	  	//std::cout <<"status["<<i<<"]="<<status[i]<< std::endl;//0以外の時は変なマークがでる
	  	//ここで検索出来なかったものを消し探索の範囲を狭めている(statusが0の時)
	  	//statusが0以外の時値を更新する(0は追跡不可能)
	  	if (status2[i] != 0) {	
	  	  points_prev2[k] = points_prev2[i];
        camera_point_p2[k] = camera_point_p2[i];
	  	  points_curr2[k++] = points_curr2[i];
	  	}
    }
	  points_curr2.resize(k);//ここでkの個数でリサイズ
	  points_prev2.resize(k);
	  camera_point_p2.resize(k);
	  camera_point_c2.resize(k);
		
	  // 特徴点を丸で描く-------------------------------------------------------------------------------------------
	  for (int i = 0; i < points_curr2.size(); i++) {
      //std::cout <<"OPT後マッチングの中心座標["<<i<<"]="<<points_curr[i]<< std::endl;
      cv::circle(imageCopy, points_prev2[i], 3, Scalar(255,255,255), -1, cv::LINE_AA);//一つ前の画像の座標
	    cv::circle(imageCopy, points_curr2[i], 4, Scalar(255, 0, 255), -1, cv::LINE_AA);//今の座標情報
	    //cv::line(imageCopy,cv::Point(points_curr2[i]),cv::Point(points_prev2[i]),cv::Scalar(0,255,255), 1, cv::LINE_AA);//線を描写する
      //cv::line(imageCopy,cv::Point(points_curr2[i].x,points_prev2[i].y),cv::Point(points_prev2[i]),cv::Scalar(255,0,0), 1, cv::LINE_AA);//線を描写する
      //cv::line(imageCopy,cv::Point(points_prev2[i].x,points_curr2[i].y),cv::Point(points_prev2[i]),cv::Scalar(0,255,0), 1, cv::LINE_AA);//線を描写する
    
      //画像→カメラ座標変換----------------------------------------------------------------------
      depth_point_curr2[i] = depthimage.at<float>(cv::Point(points_curr2[i].x,points_curr2[i].y));
      //Depthが取得できない特徴点を削除する+Depthの外れ値を除く
      if(depth_point_curr2[i]>0.001&&depth_point_curr2[i]<10000){
        camera_point_c2[depth_point_curr2_ok].x = depth_point_curr2[i] * ((points_curr2[i].x - camera_info.K[2]) / camera_info.K[0])/1000;//メートル表示変換
        camera_point_c2[depth_point_curr2_ok].y = depth_point_curr2[i] * ((points_curr2[i].y - camera_info.K[5]) / camera_info.K[4])/1000;
        camera_point_c2[depth_point_curr2_ok].z = depth_point_curr2[i]/1000;

        //std::cout << "特徴点のカメラ座標:camera_point_c2[depth_point_curr2_ok="<<depth_point_curr2_ok<<"]={"<< camera_point_c2[depth_point_curr2_ok].x <<","<<camera_point_c2[depth_point_curr2_ok].y<<","<<camera_point_c2[depth_point_curr2_ok].z<<"}"<< std::endl;
        
        camera_point_p2[depth_point_curr2_ok]=camera_point_p2[i];
        points_prev2[depth_point_curr2_ok] = points_prev2[i];
        points_curr2[depth_point_curr2_ok] = points_curr2[i];
        depth_point_curr2_ok=depth_point_curr2_ok+1;//Depth取得可能の個数をカウント
      }
    }

    std::cout <<"depth_point_curr2_ok="<<depth_point_curr2_ok<< std::endl;
    camera_point_p2.resize(depth_point_curr2_ok);//Depth取得可能数でリサイズ(三次元カメラ座標)
    camera_point_c2.resize(depth_point_curr2_ok);//Depth取得可能数でリサイズ(三次元カメラ座標)
    points_prev2.resize(depth_point_curr2_ok);//Depth取得可能数でリサイズ(二次元座標)
    points_curr2.resize(depth_point_curr2_ok);//Depth取得可能数でリサイズ(二次元座標)

    if(camera_point_p2.size()<10||k<10){//特徴点が100個以下になったら再び特徴点を検出する
      reset2 = true;
      std::cout <<" 特徴点再検出リセット"<<std::endl;}

    PT2_At0=cv::Mat_<float>(depth_point_curr2_ok*3, 6);
    PT2_Et0=cv::Mat_<float>(depth_point_curr2_ok*3, 1);
    PT2_Tt=cv::Mat_<float>(6, 1);//教科書運動パラメータ

    PT2_kosuu=0;

    //最小二乗法を用いた外部パラメータの算出(日高手法 特徴点)-----------------------------------------------------------------------------
    for(int i=0;i<depth_point_curr2_ok;i++){
      PT2_At0(PT2_kosuu,0)=-1,  PT2_At0(PT2_kosuu,1)=0,   PT2_At0(PT2_kosuu,2)=0,   PT2_At0(PT2_kosuu,3)=0,   PT2_At0(PT2_kosuu,4)=-camera_point_p2[i].z,    PT2_At0(PT2_kosuu,5)=0;
      PT2_At0(PT2_kosuu+1,0)=0, PT2_At0(PT2_kosuu+1,1)=-1,PT2_At0(PT2_kosuu+1,2)=0, PT2_At0(PT2_kosuu+1,3)=0, PT2_At0(PT2_kosuu+1,4)=0,                     PT2_At0(PT2_kosuu+1,5)=0;
      PT2_At0(PT2_kosuu+2,0)=0, PT2_At0(PT2_kosuu+2,1)=0, PT2_At0(PT2_kosuu+2,2)=-1,PT2_At0(PT2_kosuu+2,3)=0, PT2_At0(PT2_kosuu+2,4)=camera_point_p2[i].x,   PT2_At0(PT2_kosuu+2,5)=0;

      PT2_Et0(PT2_kosuu,0)=camera_point_c2[i].x-camera_point_p2[i].x;
      PT2_Et0(PT2_kosuu+1,0)=camera_point_c2[i].y-camera_point_p2[i].y;
      PT2_Et0(PT2_kosuu+2,0)=camera_point_c2[i].z-camera_point_p2[i].z;

      PT2_kosuu=PT2_kosuu+3;
    }

    ////すべてのマーカー座標を利用して最小二乗法外部パラメータ推定(初回)
    //PT2_Tt=PT2_At0.inv(cv::DECOMP_SVD)*PT2_Et0;
    ////std::cout <<"PT2_Tt=\n"<<PT2_Tt<< std::endl;//推定外部パラメータ
    //PT2_Ft = cv::Mat_<float>(3, 3);//回転行列(日高手法)
    //PT2_Ft(0,0)=1,            PT2_Ft(0,1)=PT2_Tt(5,0),   PT2_Ft(0,2)=-PT2_Tt(4,0);
    //PT2_Ft(1,0)=-PT2_Tt(5,0),  PT2_Ft(1,1)=1,             PT2_Ft(1,2)=PT2_Tt(3,0),
    //PT2_Ft(2,0)=PT2_Tt(4,0),   PT2_Ft(2,1)=PT2_Tt(3,0),    PT2_Ft(2,2)=1;
    //PT2_Vt = cv::Mat_<float>(4, 1);//並進ベクトル+y軸回転(日高手法)
    //PT2_Vt(0,0)=PT2_Tt(0,0);//Vx
    //PT2_Vt(1,0)=PT2_Tt(1,0);//Vy
    //PT2_Vt(2,0)=PT2_Tt(2,0);//Vz
    //PT2_Vt(3,0)=PT2_Tt(4,0);//Ωy
    ////std::cout <<"並進ベクトル+y軸回転(日高手法)PT2_Vt=\n"<<PT2_Vt<< std::endl;//並進ベクトル
  }
  if(kaisu!=0){
    std::cout <<"swap_on="<<swap_on<<std::endl;
    std::cout <<"swap_on2="<<swap_on2<<std::endl;
  //特徴点1検出時
  if (swap_on==false&&swap_on2==true) {
    std::cout <<"特徴点2を使用した外部パラメータ推定"<<std::endl;
    //特徴点1を使用した外部パラメータ推定
    ALL_At=cv::Mat_<float>(PT2_kosuu, 6);
    ALL_Et=cv::Mat_<float>(PT2_kosuu, 1);

    PT2_At0.rowRange(0, PT2_kosuu).copyTo(ALL_At.rowRange(0, PT2_kosuu));
	  PT2_Et0.rowRange(0, PT2_kosuu).copyTo(ALL_Et.rowRange(0, PT2_kosuu));
  }
  //特徴点2検出時
  if (swap_on==true&&swap_on2==false) {
    //std::cout <<"特徴点1を使用した外部パラメータ推定"<<std::endl;
    //特徴点1を使用した外部パラメータ推定
    ALL_At=cv::Mat_<float>(PT_kosuu, 6);
    ALL_Et=cv::Mat_<float>(PT_kosuu, 1);
    std::cout <<"PT_kosuu="<<PT_kosuu<<std::endl;

    PT_At0.rowRange(0, PT_kosuu).copyTo(ALL_At.rowRange(0, PT_kosuu));
	  PT_Et0.rowRange(0, PT_kosuu).copyTo(ALL_Et.rowRange(0, PT_kosuu));
  }

  if (swap_on==true&&swap_on2==true) {
    //std::cout <<"特徴点1+特徴点2を使用した外部パラメータ推定"<<std::endl;
    //特徴点+特徴点2を使用した外部パラメータ推定
    ALL_At=cv::Mat_<float>(PT_kosuu+PT2_kosuu, 6);
    ALL_Et=cv::Mat_<float>(PT_kosuu+PT2_kosuu, 1);
    PT_At0.rowRange(0, PT_kosuu).copyTo(ALL_At.rowRange(0, PT_kosuu));
	  PT_Et0.rowRange(0, PT_kosuu).copyTo(ALL_Et.rowRange(0, PT_kosuu));
    PT2_At0.rowRange(0, PT2_kosuu).copyTo(ALL_At.rowRange(PT_kosuu, PT_kosuu+PT2_kosuu));
	  PT2_Et0.rowRange(0, PT2_kosuu).copyTo(ALL_Et.rowRange(PT_kosuu, PT_kosuu+PT2_kosuu));
  }
      //std::cout <<"ALL_At=\n"<<ALL_At<< std::endl;
      //std::cout <<"ALL_Et=\n"<<ALL_Et<< std::endl;

      //すべてのマーカー座標を利用して最小二乗法外部パラメータ推定(初回)
      ALL_Tt=ALL_At.inv(cv::DECOMP_SVD)*ALL_Et;
      //At_1=At.t()*At;
      //Tt=At_1.inv()*At.t()*Et;
      std::cout <<"ALL_Tt=\n"<<ALL_Tt<< std::endl;//推定外部パラメータ

      ALL_Ft = cv::Mat_<float>(3, 3);//回転行列(日高手法)
      ALL_Ft(0,0)=1,             ALL_Ft(0,1)=-ALL_Tt(5,0),  ALL_Ft(0,2)=-ALL_Tt(4,0);
      ALL_Ft(1,0)=-ALL_Tt(5,0),  ALL_Ft(1,1)=1,             ALL_Ft(1,2)=ALL_Tt(3,0),
      ALL_Ft(2,0)=ALL_Tt(4,0),   ALL_Ft(2,1)=-ALL_Tt(3,0),  ALL_Ft(2,2)=1;

      //ALL_Ft(0,0)=1,        ALL_Ft(0,1)=0,        ALL_Ft(0,2)=-ALL_Tt(4,0);
      //ALL_Ft(1,0)=0,        ALL_Ft(1,1)=1,        ALL_Ft(1,2)=0,
      //ALL_Ft(2,0)=ALL_Tt(4,0),  ALL_Ft(2,1)=0,        ALL_Ft(2,2)=1;
      std::cout <<"回転行列(日高手法)ALL_Ft=\n"<<ALL_Ft<< std::endl;//回転行列

      ALL_Vt = cv::Mat_<float>(4, 1);//並進ベクトル+y軸回転(日高手法)
      ALL_Vt(0,0)=ALL_Tt(0,0);//Vx
      ALL_Vt(1,0)=ALL_Tt(1,0);//Vy
      ALL_Vt(2,0)=ALL_Tt(2,0);//Vz
      ALL_Vt(3,0)=ALL_Tt(4,0);//Ωy
      std::cout <<"並進ベクトル+y軸回転(日高手法)ALL_Vt=\n"<<ALL_Vt<< std::endl;//並進ベクトル

      //camera_base_pose.position.x = camera_base_pose.position.x-ALL_Tt(5,0)*camera_base_pose.position.y-ALL_Tt(4,0)*camera_base_pose.position.z+ALL_Tt(2,0);
      //camera_base_pose.position.y = -ALL_Tt(5,0)*camera_base_pose.position.x+camera_base_pose.position.y+ALL_Tt(3,0)*camera_base_pose.position.z+ALL_Tt(0,0);
      //camera_base_pose.position.z = ALL_Tt(4,0)*camera_base_pose.position.x-ALL_Tt(5,0)*camera_base_pose.position.y+camera_base_pose.position.z+ALL_Tt(1,0);

      camera_prmX = camera_prmX-ALL_Tt(5,0)*camera_prmY-ALL_Tt(4,0)*camera_prmZ+ALL_Tt(2,0);
      camera_prmY = -ALL_Tt(5,0)*camera_prmX+camera_prmY+ALL_Tt(3,0)*camera_prmZ+ALL_Tt(0,0);
      camera_prmZ = ALL_Tt(4,0)*camera_prmX-ALL_Tt(5,0)*camera_prmY+camera_prmZ+ALL_Tt(1,0);


  
   //tf(map camera_base間のlink)-----------------------------------------------------------------------------------------
  //ここがカメラの姿勢部分

    std::string MaptoCamera_Base_frame = "MaptoCamera_Base_link";

    //if (markerIds.size() > 0) {//マーカーが認識できる時
      //オイラー角→クオータニオン変換
      float cosRoll = cos(ALL_Ft(1,2)*realsec/ 2.0);
      float sinRoll = sin(ALL_Ft(1,2)*realsec / 2.0);
      float cosPitch = cos(ALL_Ft(0,2)*realsec / 2.0);
      float sinPitch = sin(ALL_Ft(0,2)*realsec / 2.0);
      float cosYaw = cos(ALL_Ft(0,1)*realsec / 2.0);
      float sinYaw = sin(ALL_Ft(0,1)*realsec / 2.0);

      float q0 = cosRoll * cosPitch * cosYaw + sinRoll * sinPitch * sinYaw;
      float q1 = sinRoll * cosPitch * cosYaw - cosRoll * sinPitch * sinYaw;
      float q2 = cosRoll * sinPitch * cosYaw + sinRoll * cosPitch * sinYaw;
      float q3 = cosRoll * cosPitch * sinYaw - sinRoll * sinPitch * cosYaw;

      //微小区間回転行列
      double RollX=ALL_Ft(2,1),PitchY=ALL_Ft(0,2),YawZ=ALL_Ft(1,0);

            //回転行列のみにする
      //External_prm.row(0).copyTo(External_R.row(0));
      //External_prm.row(1).copyTo(External_R.row(1));
      //External_prm.row(2).copyTo(External_R.row(2));

      //std::cout <<"External_R=\n"<< External_R << std::endl;//外部パラメータの合計

      std::array<float, 9> External_R{//外部パラメータ回転行列(column-major)
        //ALL_Ft(0,0),  ALL_Ft(0,1),  ALL_Ft(0,2),
        //ALL_Ft(1,0),  ALL_Ft(1,1),  ALL_Ft(1,2),
        //ALL_Ft(2,0),  ALL_Ft(2,1),  ALL_Ft(2,2)};
        ALL_Ft(0,0),  ALL_Ft(1,0),  ALL_Ft(2,0),
        ALL_Ft(0,1),  ALL_Ft(1,1),  ALL_Ft(2,1),
        ALL_Ft(0,2),  ALL_Ft(1,2),  ALL_Ft(2,2)};

        //External_prm(0,0),External_prm(1,0),External_prm(2,0),
        //External_prm(0,1),External_prm(1,1),External_prm(2,1),
        //External_prm(0,2),External_prm(1,2),External_prm(2,2)};

      rotationMatrixVal=External_R;
      quaternionVal=toQuaternion(rotationMatrixVal);//回転行列→クォータニオン変換
      
      std::cout <<" quaternionVal.x="<< quaternionVal.x << std::endl;//クォータニオン表示
      std::cout <<" quaternionVal.y="<< quaternionVal.y << std::endl;
      std::cout <<" quaternionVal.z="<< quaternionVal.z << std::endl;
      std::cout <<" quaternionVal.w="<< quaternionVal.w << std::endl;

      //カメラ位置
      //camera_base_pose.position.x = camera_base_pose.position.x+ALL_Vt(2,0);//赤(tf:X,画像:Z)
      //camera_base_pose.position.y = camera_base_pose.position.y+ALL_Vt(0,0);//緑(tf:Y,画像:X)
      //camera_base_pose.position.z = camera_base_pose.position.z+ALL_Vt(1,0);//青(tf:Z,画像:Y)
      //camera_base_pose.orientation.x = camera_base_pose.orientation.x+quaternionVal.z;
      //camera_base_pose.orientation.y = camera_base_pose.orientation.y+quaternionVal.x;
      //camera_base_pose.orientation.z = camera_base_pose.orientation.z+quaternionVal.y;
      //camera_base_pose.orientation.w = camera_base_pose.orientation.w+quaternionVal.w;

      camera_base_pose.position.x = camera_prmZ;//赤(tf:X,画像:Z)
      camera_base_pose.position.y = camera_prmX;//緑(tf:Y,画像:X)
      camera_base_pose.position.z = 0;//青(tf:Z,画像:Y)
      
      //camera_base_pose.position.x = camera_base_pose.position.x-ALL_Tt(5,0)*camera_base_pose.position.y-ALL_Tt(4,0)*camera_base_pose.position.z+ALL_Tt(2,0);
      //camera_base_pose.position.y = -ALL_Tt(5,0)*camera_base_pose.position.x+camera_base_pose.position.y+ALL_Tt(3,0)*camera_base_pose.position.z+ALL_Tt(0,0);
      //camera_base_pose.position.z = ALL_Tt(4,0)*camera_base_pose.position.x-ALL_Tt(5,0)*camera_base_pose.position.y+camera_base_pose.position.z+ALL_Tt(1,0);

      camera_base_pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(YawZ,RollX,PitchY);
      //camera_base_pose.orientation = tf::createQuaternionFromRPY(double RollX,double PitchY,double YawZ);
      //camera_base_pose.orientation = tf::createQuaternionFromRPY(YawZ,RollX,PitchY);
      //camera_base_pose.orientation.x = quaternionVal.z;
      //camera_base_pose.orientation.y = quaternionVal.x;
      //camera_base_pose.orientation.z = quaternionVal.y;
      //camera_base_pose.orientation.w = quaternionVal.w;

      outputfileALL_Vt<<"X:ALL_Vt(2,0)="<<ALL_Vt(2,0)<<" Y:ALL_Vt(0,0)="<<ALL_Vt(0,0)<<" Z:ALL_Vt(1,0)="<<ALL_Vt(1,0)<<"\n";
    //}
    //else{//マーカーが認識されない時（点群情報のみで自己位置推定を行う
    //  //オイラー角→クオータニオン変換
    //  float cosRoll = cos(PT_Ft(1,2)/ 2.0);
    //  float sinRoll = sin(PT_Ft(1,2) / 2.0);
    //  float cosPitch = cos(PT_Ft(0,2) / 2.0);
    //  float sinPitch = sin(PT_Ft(0,2) / 2.0);
    //  float cosYaw = cos(PT_Ft(0,1) / 2.0);
    //  float sinYaw = sin(PT_Ft(0,1) / 2.0);
    //  float q0 = cosRoll * cosPitch * cosYaw + sinRoll * sinPitch * sinYaw;
    //  float q1 = sinRoll * cosPitch * cosYaw - cosRoll * sinPitch * sinYaw;
    //  float q2 = cosRoll * sinPitch * cosYaw + sinRoll * cosPitch * sinYaw;
    //  float q3 = cosRoll * cosPitch * sinYaw - sinRoll * sinPitch * cosYaw;
    //  //カメラ位置
    //  camera_base_pose.position.x = camera_base_pose.position.x+PT_Vt(2,0);//赤(xは進行方向)
    //  camera_base_pose.position.y = camera_base_pose.position.y+PT_Vt(0,0);//緑
    //  camera_base_pose.position.z = camera_base_pose.position.z+PT_Vt(1,0);//青
    //  camera_base_pose.orientation.x = camera_base_pose.orientation.x+q0;
    //  camera_base_pose.orientation.y = camera_base_pose.orientation.y+q1;
    //  camera_base_pose.orientation.z = camera_base_pose.orientation.z+q2;
    //  camera_base_pose.orientation.w = camera_base_pose.orientation.w+q3;
    //  outputfileALL_Vt<<"X:PT_Vt(2,0)="<<PT_Vt(2,0)<<" Y:PT_Vt(0,0)="<<PT_Vt(0,0)<<" Z:PT_Vt(1,0)="<<PT_Vt(1,0)<<"\n";
    //}

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
    cv::swap(points_curr, points_prev);//二次元画像座標を保存(points_curr→points_prev)
    cv::swap(camera_point_c, camera_point_p);//三次元カメラ座標を保存(camera_point_c→camera_point_p)
    }
  if(swap_on2 ==true){//初回プログラム実行時はswapしない(データの浅いコピーの影響を考慮)
  cv::swap(points_curr2, points_prev2);//二次元画像座標を保存(points_curr2→points_prev2)
  cv::swap(camera_point_c2, camera_point_p2);//三次元カメラ座標を保存(camera_point_c2→camera_point_p2)
  }
  wall_prev=wall_duration;
  kaisu++;
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





