///マーカー検出プログラム https://qiita.com/noueezy/items/58730f09f6aa6a97faf5
//マーカー検出時の赤四角が左上になるようにマーカーを配置すると正しくマーカーの中心座標を求められる
//外部パラメータを推定するためには最低でも３点の情報が必要→そこでマーカーのコーナー４点を使用して外部パラメータを算出する

//日高先生の手法で外部パラメータの算出を行ってみる
//この手法ではカメラは台車に固定されている事を前提としているため、y方向は動かず,x,z方向のみになっている。
//20210826 システムのサンプリング時間を取得したいがros time nowがうまく使えない（何故か値が出ない）
//そこで今はWalltimeで代用するが、ここで問題が発生する可能性があるので今後修繕が必要
//https://8ttyan.hatenablog.com/entry/2015/02/03/003428
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

std::string win_src = "src";
std::string win_dst = "dst";
std::string win_depth = "win_depth";

std::string source_frame = "map";//mapフレーム
ros::Subscriber marker_sub;
cv::KalmanFilter KF(4, 4);//(状態ベクトルの次元数,観測ベクトルの次元数)
cv::KalmanFilter KF2(2, 1);//(状態ベクトルの次元数,観測ベクトルの次元数)

using namespace std;
using namespace cv;
float kaisu = 0;//tf用
int kaisu2 = 0;//tf用
int kosuu=0;

// reset == TRUE のとき特徴点検出を行う
// 最初のフレームで必ず特徴点検出を行うように、初期値を TRUE にする
bool reset = true;//初回プログラム用（特徴点検出&特徴点再検出用)
bool swap_on = true;//初回プログラム実行時はswapしない(データの浅いコピーの影響を考慮)


cv::Mat image_curr,image_prev,img_dst,img_dst1;//画像定義
vector<cv::Point2f> points_prev, points_curr;//特徴点定義
vector<cv::Point3f> camera_point_p,camera_point_c;//特徴点定義
float depth_point_prev[300],camera_point_prev[300][3],depth_point_curr[300],camera_point_curr[300][3];
cv::Mat_<float> F_Xp = cv::Mat_<float>(4, 1);//外部パラメータ（日高手法 特徴点)
cv::Mat_<float> F_XpPLAS = cv::Mat_<float>::zeros(4, 1);


sensor_msgs::CameraInfo camera_info;//CameraInfo受け取り用
static struct_slam::marker_tf marker_tf;//観測マーカーのメッセージ
float MLength_prve[50],MAngle_prve[50];//マーカーまでの距離と角度
float Rotation;//外部パラメータ(仮)
cv::Mat_<float> MarkerCamera[50]=cv::Mat_<float>(4, 4);//マーカーのカメラ座標の行列
cv::Mat_<float> MarkerWorld[50]=cv::Mat_<float>(4, 3);//マーカーの世界座標の行列
cv::Mat_<float>MarkerCameraT=cv::Mat_<float>(4, 4);//マーカーの世界座標の行列
cv::Mat_<float> External_prm=cv::Mat_<float>(4, 3);//外部パラメータ
static cv::Mat_<float> External_prmPLAS=cv::Mat_<float>(4, 3);//外部パラメータ
//static cv::Mat_<float> External_R=cv::Mat_<float>(3, 3);//外部パラメータ(回転行列のみ)
std::array<float, 9> External_R;//外部パラメータ(回転行列のみ)(クォータニオン変換用)
static cv::Mat_<float> External_T=cv::Mat_<float>(1, 3);//外部パラメータ(回転行列のみ)
float point_prve[50][4][3];//一つ前のカメラ座標
cv::Mat_<float> Xp = cv::Mat_<float>(4, 1);//外部パラメータ（日高手法)
cv::Mat_<float> XpPLAS = cv::Mat_<float>::zeros(4, 1);
float pixel[50][4][2],depth[100],point[50][4][3],x,y,r2,f,ux,uy;//画像→カメラ座標変換

ros::Time ros_begin;//プログラム時間
ros::WallTime wall_begin = ros::WallTime::now();//プログラム開始時の時間
ros::WallDuration wall_prev;//一つ前の時間
ros::WallDuration wall_systemtime;//サンプリング時間
//ros::Time ros_begin = ros::Time::now();
struct timeval startTime, endTime;  // 構造体宣言
float realsec;//サンプリング時間（C++)

cv::Mat_<float> xEst = cv::Mat_<float>::zeros(3, 1);//状態方程式（カメラ）
cv::Mat_<float> Ft = cv::Mat_<float>(3, 3);//回転行列
cv::Mat_<float> Vt = cv::Mat_<float>(3, 1);//並進ベクトル




struct Camera_Base{
    float x;
    float y;
    float z;
};
struct Camera_Base camera_base;

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
  //gettimeofday(&endTime, NULL);// 開始時刻取得
  //if(kaisu!=0){
  //  time_t diffsec = difftime(endTime.tv_sec, startTime.tv_sec);    // 秒数の差分を計算
  //  suseconds_t diffsub = endTime.tv_usec - startTime.tv_usec;      // マイクロ秒部分の差分を計算
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
	
  //if(kaisu==0){ros_begin = ros::Time::now();}
    //ros::Time ros_now = ros::Time::now();
    //ros::Duration ros_duration = ros_now - ros_begin;
    //ROS_INFO("ROS: %u.09%u", ros_duration.sec, ros_duration.nsec);
    //ROS_INFO("%lf",ros::Time::now().toSec());
    //double  ros_nowt = ros::Time::now().toSec();
    //fout << t << std::endl;


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
  //std::cout << "camera_info.K[0]=" <<camera_info.K[0]<< std::endl;//内部パラメータ
  //std::cout << "camera_info.K[1]=" <<camera_info.K[1]<< std::endl;
  //std::cout << "camera_info.K[2]=" <<camera_info.K[2]<< std::endl;
  //std::cout << "camera_info.K[3]=" <<camera_info.K[3]<< std::endl;
  //std::cout << "camera_info.K[4]=" <<camera_info.K[4]<< std::endl;
  //std::cout << "camera_info.K[5]=" <<camera_info.K[5]<< std::endl;
   
  image = bridgeImage->image.clone();//image変数に変換した画像データを代入
  depthimage = bridgedepthImage->image.clone();//image変数に変換した画像データを代入

  //Depth画像の色付けを行なっている
  double min;
  double max;
  cv::minMaxIdx(depthimage, &min, &max);
  cv::Mat adjimg_depth;
  // Histogram Equalization
  float scale = 255 / (max-min);
  depthimage.convertTo(adjimg_depth,CV_8UC1, scale, -min*scale); 
  cv::Mat falseColorsimg_depth;
  applyColorMap(adjimg_depth, falseColorsimg_depth, cv::COLORMAP_WINTER);//ここのcvで色を変えられる
  cv::imshow("Out", falseColorsimg_depth);

  image.copyTo(img_dst);//
  image.copyTo(img_dst1);//特徴点検出結果表示用

  cv::Mat imageCopy = image.clone();
  cv::Mat_<float> intrinsic_K= cv::Mat_<float>(3, 3);
	std::cout <<"画像取り込み"<< std::endl;


  //カルマンフィルタ初期設定---------------------------------------------------------------------------------------------------------
  if(kaisu==0){
    KF.statePre = cv::Mat_<float>::zeros(4, 4); //状態の推定値(x'(k))
    KF.statePost = cv::Mat_<float>::zeros(4, 4);// 更新された状態の推定値 (x(k))
    // 運動モデル(システムの時間遷移に関する線形モデル)(A)
    KF.transitionMatrix = (cv::Mat_<float>(4, 4) << // 等速直線運動（速度利用あり）
      1, 0, 1, 0,
      0, 1, 0, 1,
      0, 0, 1, 0,
      0, 0, 0, 1);

    KF.measurementMatrix = (cv::Mat_<float>(2, 4) << //観測行列 (H)
      1, 0, 0, 0,
      0, 1, 0, 0);

    KF.measurementNoiseCov = (cv::Mat_<float>(2, 2) << //観測ノイズの共分散行列 (R)
      0.1, 0,
      0, 0.1);

    KF.gain = cv::Mat_<float>::zeros(4, 2);//カルマンゲイン(K)
    setIdentity(KF.processNoiseCov, cv::Scalar::all(1e-1));//プロセスノイズ（時間遷移に関するノイズ）の共分散行列 (Q)
    setIdentity(KF.errorCovPost, cv::Scalar::all(1e-1));//前回更新された誤差共分散(P'(k))

    KF2.statePre = cv::Mat_<float>::zeros(2, 4); //状態の推定値(x'(k))
    KF2.statePost = cv::Mat_<float>::zeros(2, 4);// 更新された状態の推定値 (x(k))

    // 運動モデル(システムの時間遷移に関する線形モデル)(A)
    KF2.transitionMatrix = (cv::Mat_<float>(2, 2) << // 等速直線運動（速度利用あり）
      1, 0,
      0, 1);

    KF2.measurementMatrix = (cv::Mat_<float>(1, 2) << //観測行列 (H)
      1, 0);

    KF2.measurementNoiseCov = (cv::Mat_<float>(1, 1) << //観測ノイズの共分散行列 (R)
      0.1);

    KF2.gain = cv::Mat_<float>::zeros(2, 1);//カルマンゲイン(K)
    setIdentity(KF2.processNoiseCov, cv::Scalar::all(1e-1));//プロセスノイズ（時間遷移に関するノイズ）の共分散行列 (Q)
    setIdentity(KF2.errorCovPost, cv::Scalar::all(1e-1));//前回更新された誤差共分散(P'(k))
  }
//---------------------------------------------------------------------------------------------------------------------------
  //特徴点検出--------------------------------------------------------------------------------------------------------------
  //一つ目前の特徴点の数と現在の特徴点の数を合わせる必要がある。ここでは追跡可能か,それぞれDepthデータ取得可能であるかでサイズ調整を行っている
	cv::cvtColor(image, image_curr, cv::COLOR_BGR2GRAY);//グレースケール化
  //初回検出プログラム-----------------------------------------------------------------------------------------------------
  if (reset == true) {
	  std::cout <<"初回検出プログラム"<< std::endl;
	  std::cout <<"test1"<< std::endl;


    int depth_point_prev_ok=0; //depth取得可能な特徴点の数
    swap_on=false;
    //cv::goodFeaturesToTrack(今画像, 前画像, 特徴点の個数, 0.01, 10, cv::Mat(), 3, 3, 0, 0.04);
		cv::goodFeaturesToTrack(image_curr, points_curr, 100, 0.01, 10, cv::Mat(), 3, 3, 0, 0.04);// 特徴点検出(グレースケール画像から特徴点検出)
		cv::cornerSubPix(image_curr, points_curr, cv::Size(10, 10), cv::Size(-1, -1), cv::TermCriteria(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, 20, 0.03));
		points_prev = points_curr;//データのコピ
    camera_point_p.resize(points_prev.size());//要素数初期設定
	  std::cout <<"test2_points_prev.size()="<<points_prev.size()<< std::endl;

    
		for (int i = 0; i < points_prev.size(); i++) {
			cv::circle(imageCopy, points_prev[i], 6, Scalar(255,0,0), -1, cv::LINE_AA);
      //画像→カメラ座標変換----------------------------------------------------------------------
      depth_point_prev[i] = depthimage.at<float>(cv::Point(points_prev[i].x,points_prev[i].y));
	    std::cout <<"test3_i="<<i<< std::endl;

      //Depthが取得できない特徴点を削除する+Depthの外れ値を除く
      if(depth_point_prev[i]>0.001&&depth_point_prev[i]<10000){
        camera_point_p[depth_point_prev_ok].x = depth_point_prev[i] * ((points_prev[i].x - camera_info.K[2]) / camera_info.K[0])/1000;//メートル表示変換
        camera_point_p[depth_point_prev_ok].y = depth_point_prev[i] * ((points_prev[i].y - camera_info.K[5]) / camera_info.K[4])/1000;
        camera_point_p[depth_point_prev_ok].z = depth_point_prev[i]/1000;

        //std::cout << "特徴点のカメラ座標:camera_point_p[depth_point_prev_ok="<<depth_point_prev_ok<<"]={"<< camera_point_p[depth_point_prev_ok].x <<","<<camera_point_p[depth_point_prev_ok].y<<","<<camera_point_p[depth_point_prev_ok].z<<"}"<< std::endl;
        points_prev[depth_point_prev_ok] = points_prev[i];
        depth_point_prev_ok=depth_point_prev_ok+1;//Depth取得可能の個数をカウント
	      std::cout <<"test4"<< std::endl;

      }
		}
    points_prev.resize(depth_point_prev_ok);//Depth取得可能数でリサイズ(二次元)
	  std::cout <<"test5"<< std::endl;
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
	    cv::circle(imageCopy, points_curr[i], 4, Scalar(0, 255, 0), -1, cv::LINE_AA);//今の座標情報
	    cv::circle(imageCopy, points_prev[i], 3, Scalar(255,255,255), -1, cv::LINE_AA);//一つ前の画像の座標
	    cv::line(imageCopy,cv::Point(points_curr[i]),cv::Point(points_prev[i]),cv::Scalar(0,255,255), 1, cv::LINE_AA);//線を描写する
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
        depth_point_curr_ok=depth_point_curr_ok+1;//Depth取得可能の個数をカウント
      }
    }

    std::cout <<"depth_point_curr_ok="<<depth_point_curr_ok<< std::endl;
    camera_point_p.resize(depth_point_curr_ok);//Depth取得可能数でリサイズ(三次元カメラ座標)
    camera_point_c.resize(depth_point_curr_ok);//Depth取得可能数でリサイズ(三次元カメラ座標)
    points_prev.resize(depth_point_curr_ok);//Depth取得可能数でリサイズ(二次元座標)
    points_curr.resize(depth_point_curr_ok);//Depth取得可能数でリサイズ(二次元座標)

    if(camera_point_p.size()<30||k<30){//特徴点が100個以下になったら再び特徴点を検出する
      reset = true;
      std::cout <<" 特徴点再検出リセット"<<std::endl;}

    cv::Mat_<float>F_Pt=cv::Mat_<float>(depth_point_curr_ok*2, 1);
    cv::Mat_<float>F_Xt=cv::Mat_<float>(depth_point_curr_ok*2, 1);
    cv::Mat_<float>F_Ot_1=cv::Mat_<float>(depth_point_curr_ok*2, 4);
    kosuu=0;

    //最小二乗法を用いた外部パラメータの算出(日高手法 特徴点)-----------------------------------------------------------------------------
    for(int i=0;i<depth_point_curr_ok;i++){
      F_Pt(kosuu,0)=camera_point_c[i].x;//マーカーのカメラ座標(観測)
      F_Pt(kosuu+1,0)=camera_point_c[i].z;

      F_Ot_1(kosuu,0)=camera_point_p[i].x,F_Ot_1(kosuu,1)=camera_point_p[i].z,F_Ot_1(kosuu,2)=1,F_Ot_1(kosuu,3)=0;
      F_Ot_1(kosuu+1,0)=camera_point_p[i].z,F_Ot_1(kosuu+1,1)=-camera_point_p[i].x,F_Ot_1(kosuu+1,2)=0,F_Ot_1(kosuu+1,3)=1;

      kosuu=kosuu+2;
    }
          
    //std::cout <<"特徴点F_Pt=\n"<<F_Pt<< std::endl;
    //std::cout <<"特徴点F_Ot_1=\n"<<F_Ot_1<< std::endl;
      
    cv::Mat_<float> F_Ot_1T = cv::Mat_<float>(4, 4);
    F_Ot_1T=F_Ot_1.t()*F_Ot_1;
    F_Xp=F_Ot_1T.inv(cv::DECOMP_SVD)*F_Ot_1.t()*F_Pt;
    //std::cout <<"F_Ot_1T=\n"<<F_Ot_1T<< std::endl;
    std::cout <<"F_Xp=\n"<<F_Xp<< std::endl;
    F_XpPLAS=F_XpPLAS+F_Xp;
    std::cout <<"F_XpPLAS=\n"<<F_XpPLAS<< std::endl;
    
  }


//マーカー検出+外部パラメータ推定-------------------------------------------------------------------------------------------  
  //カメラ内部パラメータ読み込み
  cv::Mat cameraMatrix;
  cv::FileStorage fs;
  fs.open("/home/fuji/catkin_ws/src/Struct_SLAM/src/marker/realsense_para.xml", cv::FileStorage::READ);
  fs["intrinsic"]>>cameraMatrix;
  std::cout << "内部パラメータcameraMatrix=\n" << cameraMatrix << std::endl;
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
  cv::Mat_<float> Screenp[50] = cv::Mat_<float>(3, 1);//点Pの画像座標系（u1,v1,1)T※斉次化
  cv::Mat_<float> Camerap[50] = cv::Mat_<float>(3, 1);//点Pのカメラ座標系（xc,yc,zc)T
  float MLength[50],MAngle[50];//マーカーまでの距離と角度
  int depth_error[50];//各マーカーごとにDepthが取得不可能なコーナー数をカウント
  int depth_ok[50],ALL_depth_ok=0;//各マーカーごとにDepthが取得可能なコーナー数をカウント
  float World_point[50][4][3];//各マーカーの世界座標

  //cv::Mat measurement(2, 1, CV_32F);// カルマンフィルタ(観測)
  cv::Mat measurement(2, 4, CV_32F);
  cv::Mat measurement2(1, 4, CV_32F);// カルマンフィルタ(観測)
  
  kosuu=0;

  if (markerIds.size() > 0) {
    //マーカー位置を描画
    cv::aruco::drawDetectedMarkers(imageCopy, markerCorners, markerIds);
    //マーカーの姿勢推定
    cv::aruco::estimatePoseSingleMarkers(markerCorners, 0.05, cameraMatrix, distCoeffs, rvecs, tvecs);

    for(int i=0;i<markerIds.size();i++){
      std::cout <<"マーカーの個数:markerIds.size()="<<markerIds.size() << std::endl;//マーカー個数
      std::cout <<"markerIds("<<i<<")="<< markerIds.at(i) << std::endl;//マーカーID
      //std::cout <<"markerCorners["<<i<<"]=\n"<< markerCorners[i] << std::endl;//マーカーのコーナー座標が出てくる
      //std::cout <<"markerCorners["<<i<<"]="<< markerCorners[i][0].x << std::endl;//マーカーのコーナー座標がx出てくる
      //std::cout <<"markerCorners["<<i<<"]="<< markerCorners[i][0].y << std::endl;//マーカーのコーナー座標がy出てくる
      MarkerC[markerIds.at(i)][0][0]=markerCorners[i][0].x, MarkerC[markerIds.at(i)][0][1]=markerCorners[i][0].y;//コーナーの画像座標ID対応化
      MarkerC[markerIds.at(i)][1][0]=markerCorners[i][1].x, MarkerC[markerIds.at(i)][1][1]=markerCorners[i][1].y;
      MarkerC[markerIds.at(i)][2][0]=markerCorners[i][2].x, MarkerC[markerIds.at(i)][2][1]=markerCorners[i][2].y;
      MarkerC[markerIds.at(i)][3][0]=markerCorners[i][3].x, MarkerC[markerIds.at(i)][3][1]=markerCorners[i][3].y;

      //std::cout <<"MarkerC["<<markerIds.at(i)<<"][0][0]="<< MarkerC[markerIds.at(i)][0][0]<< std::endl;//コーナーの画像座標(ID対応)
      //std::cout <<"MarkerC["<<markerIds.at(i)<<"][0][1]="<< MarkerC[markerIds.at(i)][0][1]<< std::endl;//コーナーの画像座標(ID対応)
		  cv::circle(imageCopy, cv::Point(MarkerC[markerIds.at(i)][0][0], MarkerC[markerIds.at(i)][0][1]), 3, Scalar(0,255,0),   -1, cv::LINE_AA);//緑点
		  cv::circle(imageCopy, cv::Point(MarkerC[markerIds.at(i)][1][0], MarkerC[markerIds.at(i)][1][1]), 3, Scalar(255,255,0), -1, cv::LINE_AA);//水色
		  cv::circle(imageCopy, cv::Point(MarkerC[markerIds.at(i)][2][0], MarkerC[markerIds.at(i)][2][1]), 3, Scalar(255,0,0),   -1, cv::LINE_AA);//青
		  cv::circle(imageCopy, cv::Point(MarkerC[markerIds.at(i)][3][0], MarkerC[markerIds.at(i)][3][1]), 3, Scalar(255,0,255), -1, cv::LINE_AA);//紫

      cv::aruco::drawAxis(imageCopy,cameraMatrix,distCoeffs,rvecs[i], tvecs[i], 0.1);//マーカーの姿勢描写
      //std::cout <<"rvecs["<<i<<"]="<< rvecs[i] << std::endl;//回転ベクトル
      //std::cout <<"tvecs["<<i<<"]="<< tvecs[i] << std::endl;//並進ベクトル:tvecs=(x,y,z)カメラ座標

      cv::Rodrigues(rvecs[i],rvecs2[i],jacobian[i]);//回転ベクトルから回転行列への変換

      //std::cout <<"回転行列rvecs2["<<i<<"]="<< rvecs2[i] << std::endl;//回転行列

      depth_ok[markerIds.at(i)]=0;//Depth取得可能数初期化

      //画像→カメラ座標変換(コーナすべて変換する)----------------------------------------------------------------------
      for(int j=0;j<4;j++){
        pixel[markerIds.at(i)][j][0]=MarkerC[markerIds.at(i)][j][0];
        pixel[markerIds.at(i)][j][1]=MarkerC[markerIds.at(i)][j][1];
        std::cout <<"MarkerC[markerIds.at(i)="<<markerIds.at(i)<<"][j="<<j<<"][0]="<<MarkerC[markerIds.at(i)][j][0]<<",MarkerC[markerIds.at(i)="<<markerIds.at(i)<<"][j="<<j<<"][1]="<<MarkerC[markerIds.at(i)][j][1]<< std::endl;
        depth[j] = depthimage.at<float>(cv::Point(pixel[markerIds.at(i)][j][0],pixel[markerIds.at(i)][j][1]));
        //std::cout <<"depth["<<markerIds.at(i)<<"]["<<j<<"]="<< depth[j]<< std::endl;

        //Depthが取得できないコーナーを削除する+Depthの外れ値を除く
        if(depth[j]>0&&depth[j]<10000){
          x = (pixel[markerIds.at(i)][j][0] - camera_info.K[2]) / camera_info.K[0];
          y = (pixel[markerIds.at(i)][j][1] - camera_info.K[5]) / camera_info.K[4];
          //camera_info.K[0]=615.337,camera_info.K[2]=324.473,camera_info.K[4]=615.458,camera_info.K[5]=241.696//内部パラメータ

          point[markerIds.at(i)][j][0] = depth[j] * x/1000;//メートル表示変換
          point[markerIds.at(i)][j][1] = depth[j] * y/1000;
          point[markerIds.at(i)][j][2] = depth[j]/1000;
          std::cout << "特徴点のカメラ座標:point["<<markerIds.at(i)<<"]["<<j<<"]={"<< point[markerIds.at(i)][j][0] <<","<<point[markerIds.at(i)][j][1]<<","<<point[markerIds.at(i)][j][2]<<"}"<< std::endl;
          depth_ok[markerIds.at(i)]=depth_ok[markerIds.at(i)]+1;//Depth取得可能の個数をカウント
          ALL_depth_ok=ALL_depth_ok+1;//全Depth取得可能の個数をカウント
          //std::cout <<"X="<< point[markerIds.at(i)][j][0]/point[markerIds.at(i)][j][2]<< std::endl;
          //std::cout <<"Y="<< point[markerIds.at(i)][j][1]/point[markerIds.at(i)][j][2]<< std::endl;
        }
      }
      std::cout <<"depth_ok["<<markerIds.at(i)<<"]="<< depth_ok[markerIds.at(i)]<< std::endl;

    }
    cv::Mat_<float> MarkerCameraALL=cv::Mat_<float>(ALL_depth_ok, 4);//マーカーのカメラ座標の行列
    cv::Mat_<float> MarkerWorldALL=cv::Mat_<float>(ALL_depth_ok, 3);//マーカーの世界座標の行列

    cv::Mat_<float>Pt=cv::Mat_<float>(markerIds.size()*8, 1);
    cv::Mat_<float>Ot_1=cv::Mat_<float>(markerIds.size()*8, 4);
  
    //最小二乗法を用いた外部パラメータの算出(日高手法)-----------------------------------------------------------------------------
    if(kaisu!=0){
      for(int i=0;i<markerIds.size();i++){
        for(int j=0;j<4;j++){
        //for(int j=0;j<depth_ok[markerIds.at(i)];j++){//これを考慮する必要がある
          Pt(kosuu,0)=point[markerIds.at(i)][j][0];//マーカーのカメラ座標(観測)
          Pt(kosuu+1,0)=point[markerIds.at(i)][j][2];

          Ot_1(kosuu,0)=point_prve[markerIds.at(i)][j][0],Ot_1(kosuu,1)=point_prve[markerIds.at(i)][j][2],Ot_1(kosuu,2)=1,Ot_1(kosuu,3)=0;
          Ot_1(kosuu+1,0)=point_prve[markerIds.at(i)][j][2],Ot_1(kosuu+1,1)=-point_prve[markerIds.at(i)][j][0],Ot_1(kosuu+1,2)=0,Ot_1(kosuu+1,3)=1;

          kosuu=kosuu+2;
        }
      }
      
      std::cout <<"Pt=\n"<<Pt<< std::endl;
      std::cout <<"Ot_1=\n"<<Ot_1<< std::endl;
      
      cv::Mat_<float> Ot_1T = cv::Mat_<float>(4, 4);
      Ot_1T=Ot_1.t()*Ot_1;
      Xp=Ot_1T.inv(cv::DECOMP_SVD)*Ot_1.t()*Pt;
      //std::cout <<"Ot_1T=\n"<<Ot_1T<< std::endl;
      std::cout <<"Xp=\n"<<Xp<< std::endl;//推定外部パラメータ[cos,sin,tx,ty]
      XpPLAS=XpPLAS+Xp;
      std::cout <<"XpPLAS=\n"<<XpPLAS<< std::endl;

      //外部パラメータから角速度を求める
      float tan=Xp(0,1)/Xp(0,0);//tan=sin/cos
      std::cout <<"tan="<<tan<< std::endl;
      float theta=atan(tan);
      std::cout <<"theta="<<theta<< std::endl;
      float omega=theta/wall_systemtime.toSec ();
      std::cout <<"omega="<<omega<< std::endl;
      float omega2=theta/realsec;
      std::cout <<"omega2="<<omega2<< std::endl;
      
      Ft(0,0)=1,Ft(0,1)=0,Ft(0,2)=-omega2;
      Ft(1,0)=0,Ft(1,1)=1,Ft(2,2)=0,
      Ft(2,0)=omega2,Ft(2,1)=0,Ft(2,2)=1;
      std::cout <<"Ft=\n"<<Ft<< std::endl;//回転行列

      //外部パラメータから速度を求める
      Vt(0,0)=Xp(0,2)/realsec;//Vx
      Vt(1,0)=Xp(0,3)/realsec;//Vz
      std::cout <<"vx="<<Vt(0,0)<<",vz="<<Vt(1,0)<< std::endl;//並進ベクトル

      //状態方程式（自己位置推定）
      xEst=Ft*xEst-Vt;
      xEst(1,0)=0;
      std::cout <<"xEst=\n"<<xEst<< std::endl;
      
    }

/*
//最小二乗法を用いた外部パラメータの算出-----------------------------------------------------------------------------
 
      //最小二乗法を用いて外部パラメータを算出する(マーカーがひとつの時限定（試し）)(逆行列は擬似逆行列を使用している)
      External_prm=MarkerCameraT.inv(cv::DECOMP_SVD)*MarkerCamera[markerIds.at(i)].t()*MarkerWorld[markerIds.at(i)];

      std::cout <<" External_prm=\n"<< External_prm << std::endl;//外部パラメータ
      External_prmPLAS=External_prmPLAS+External_prm;
      std::cout <<" External_prmPLAS=\n"<< External_prmPLAS << std::endl;//外部パラメータの合計

      //回転行列のみにする
      //External_prm.row(0).copyTo(External_R.row(0));
      //External_prm.row(1).copyTo(External_R.row(1));
      //External_prm.row(2).copyTo(External_R.row(2));

      //std::cout <<"External_R=\n"<< External_R << std::endl;//外部パラメータの合計

      std::array<float, 9> External_R{//外部パラメータ回転行列(column-major)
        External_prm(0,0),External_prm(1,0),External_prm(2,0),
        External_prm(0,1),External_prm(1,1),External_prm(2,1),
        External_prm(0,2),External_prm(1,2),External_prm(2,2)};

      rotationMatrixVal=External_R;
      quaternionVal=toQuaternion(rotationMatrixVal);//回転行列→クォータニオン変換
      
      std::cout <<" quaternionVal.x="<< quaternionVal.x << std::endl;//クォータニオン表示
      std::cout <<" quaternionVal.y="<< quaternionVal.y << std::endl;
      std::cout <<" quaternionVal.z="<< quaternionVal.z << std::endl;
      std::cout <<" quaternionVal.w="<< quaternionVal.w << std::endl;


     //マーカーの世界座標配列化
      //MarkerWorld[markerIds.at(i)](0,0)=MarkerCamera[markerIds.at(i)](0,0);
      //MarkerWorld[markerIds.at(i)](0,1)=MarkerCamera[markerIds.at(i)](0,1);
      //MarkerWorld[markerIds.at(i)](0,2)=MarkerCamera[markerIds.at(i)](0,2);
      //std::cout <<" MarkerWorld["<<markerIds.at(i)<<"]=\n"<< MarkerWorld[markerIds.at(i)] << std::endl;

      kaisu2++;

      //マーカーまでの距離と角度を求める
      MLength[markerIds.at(i)]=point[markerIds.at(i)][0][2];
      MAngle[markerIds.at(i)]=atan(point[markerIds.at(i)][0][0]/point[markerIds.at(i)][0][2]);

      std::cout <<"距離MLemght["<<markerIds.at(i)<<"]="<< MLength[markerIds.at(i)] << std::endl;
      std::cout <<"角度MAngle["<<markerIds.at(i)<<"]="<< MAngle[markerIds.at(i)] << std::endl;

      //マーカーの一つ前の座標と今の座標から最小二乗法を用いて外部パラメータを算出する
      //座標配列化
      //初回*/



      //MLength_prve[markerIds.at(i)]=MLength[markerIds.at(i)];//一つ前の距離に保存
      //MAngle_prve[markerIds.at(i)]=MAngle[markerIds.at(i)];//一つ前の角度に保存
    
    for(int i=0;i<markerIds.size();i++){
      for(int j=0;j<4;j++){
        point_prve[markerIds.at(i)][j][0]=point[markerIds.at(i)][j][0];//今のカメラ座標を保存
        point_prve[markerIds.at(i)][j][1]=point[markerIds.at(i)][j][1];
        point_prve[markerIds.at(i)][j][2]=point[markerIds.at(i)][j][2];
      }
      //tf(観測マーカー)-------------------------------------------------------------------------------------------------観測マーカー
      std::string target_maker_frame = "marker_link";//cameraとマーカー間のリンク
      geometry_msgs::Pose maker_pose;

      std::cout << "tf特徴点のカメラ座標:point["<<markerIds.at(i)<<"][0]={x="<< point[markerIds.at(i)][0][2] <<",y="<<-point[markerIds.at(i)][0][0]<<",z="<<-point[markerIds.at(i)][0][1]<<"}"<< std::endl;

      maker_pose.position.x = point[markerIds.at(i)][0][2];//Rvizと画像は座標系が異なるので注意
      maker_pose.position.y = -point[markerIds.at(i)][0][0];
      maker_pose.position.z = -point[markerIds.at(i)][0][1];
      maker_pose.orientation.w = 1.0;

      static tf::TransformBroadcaster br_maker;
      tf::Transform maker_transform;
      poseMsgToTF(maker_pose, maker_transform);
      br_maker.sendTransform(tf::StampedTransform(maker_transform, ros::Time::now(), "/camera_link", target_maker_frame));
      //tf(観測マーカー)-------------------------------------------------------------------------------------------------観測マーカー
      std::string target_maker_frame1 = "marker_link1";//cameraとマーカー間のリンク
      geometry_msgs::Pose maker_pose1;

      std::cout << "tf特徴点のカメラ座標1:point["<<markerIds.at(i)<<"][0]={x="<< point[markerIds.at(i)][0][2] <<",y="<<-point[markerIds.at(i)][0][0]<<",z="<<-point[markerIds.at(i)][0][1]<<"}"<< std::endl;

      maker_pose1.position.x = point[markerIds.at(i)][1][2];//Rvizと画像は座標系が異なるので注意
      maker_pose1.position.y = -point[markerIds.at(i)][1][0];
      maker_pose1.position.z = -point[markerIds.at(i)][1][1];
      maker_pose1.orientation.w = 1.0;

      static tf::TransformBroadcaster br_maker1;
      tf::Transform maker_transform1;
      poseMsgToTF(maker_pose1, maker_transform1);
      br_maker1.sendTransform(tf::StampedTransform(maker_transform1, ros::Time::now(), "/camera_link", target_maker_frame1));
      //tf(観測マーカー)-------------------------------------------------------------------------------------------------観測マーカー
      std::string target_maker_frame2 = "marker_link2";//cameraとマーカー間のリンク
      geometry_msgs::Pose maker_pose2;

      std::cout << "tf特徴点のカメラ座標2:point["<<markerIds.at(i)<<"][0]={x="<< point[markerIds.at(i)][0][2] <<",y="<<-point[markerIds.at(i)][0][0]<<",z="<<-point[markerIds.at(i)][0][1]<<"}"<< std::endl;

      maker_pose2.position.x = point[markerIds.at(i)][2][2];//Rvizと画像は座標系が異なるので注意
      maker_pose2.position.y = -point[markerIds.at(i)][2][0];
      maker_pose2.position.z = -point[markerIds.at(i)][2][1];
      maker_pose2.orientation.w = 1.0;

      static tf::TransformBroadcaster br_maker2;
      tf::Transform maker_transform2;
      poseMsgToTF(maker_pose2, maker_transform2);
      br_maker2.sendTransform(tf::StampedTransform(maker_transform2, ros::Time::now(), "/camera_link", target_maker_frame2));
      //tf(観測マーカー)-------------------------------------------------------------------------------------------------観測マーカー
      std::string target_maker_frame3 = "marker_link3";//cameraとマーカー間のリンク
      geometry_msgs::Pose maker_pose3;

      std::cout << "tf特徴点のカメラ座標3:point["<<markerIds.at(i)<<"][0]={x="<< point[markerIds.at(i)][0][2] <<",y="<<-point[markerIds.at(i)][0][0]<<",z="<<-point[markerIds.at(i)][0][1]<<"}"<< std::endl;

      maker_pose3.position.x = point[markerIds.at(i)][3][2];//Rvizと画像は座標系が異なるので注意
      maker_pose3.position.y = -point[markerIds.at(i)][3][0];
      maker_pose3.position.z = -point[markerIds.at(i)][3][1];
      maker_pose3.orientation.w = 1.0;

      static tf::TransformBroadcaster br_maker3;
      tf::Transform maker_transform3;
      poseMsgToTF(maker_pose3, maker_transform3);
      br_maker3.sendTransform(tf::StampedTransform(maker_transform3, ros::Time::now(), "/camera_link", target_maker_frame3));

    }
  }


//tf(map camera_base間のlink)-----------------------------------------------------------------------------------------
   geometry_msgs::Pose camera_base_pose;

    std::string MaptoCamera_Base_frame = "MaptoCamera_Base_link";
    camera_base_pose.position.x = camera_base.x+Xp(1,0);//ここにカメラオドメトリーが入ると思う
    camera_base_pose.position.y = camera_base.y;
    camera_base_pose.position.z = camera_base.z+Xp(2,0);
    //camera_base_pose.position.x = camera_base.x+External_prm(3,0);//ここにカメラオドメトリーが入ると思う
    //camera_base_pose.position.y = camera_base.y+External_prm(3,1);
    //camera_base_pose.position.z = camera_base.z+External_prm(3,2);
    camera_base_pose.orientation.w = 0.1;
    static tf::TransformBroadcaster br_camera_base_pose;

    tf::Transform camera_base_transform;
    poseMsgToTF(camera_base_pose, camera_base_transform);
    br_camera_base_pose.sendTransform(tf::StampedTransform(camera_base_transform, ros::Time::now(), source_frame, MaptoCamera_Base_frame));

  //tf(camera_base camera_link間のlink)-----------------------------------------------------------------------------------------
    geometry_msgs::Pose Camera_BasetoCamera_Link_pose;

    //std::string Camera_BasetoCamera_Link_frame = "Camera_BasetoCamera_Link_link";
    Camera_BasetoCamera_Link_pose.position.x = 0;
    Camera_BasetoCamera_Link_pose.position.y = 0;
    Camera_BasetoCamera_Link_pose.position.z = 0.14;//ここに実際のカメラの高さを入れる
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

    

    //std::cout << "観測マーカーと地図上のマーカーとの差を計算(座標)={"<< 
    //marker_tf2.Tfodometry.pose.pose.position.x <<","<<marker_tf2.Tfodometry.pose.pose.position.y<<","<<marker_tf2.Tfodometry.pose.pose.position.z<<"}"<< std::endl;

  
    // 画面表示
    cv::imshow(win_src, image);
    cv::imshow(win_dst, imageCopy);
    //cv::imshow(win_depth, depthimage);

    cv::swap(image_curr, image_prev);// image_curr を image_prev に移す（交換する）
    if(swap_on ==true){//初回プログラム実行時はswapしない(データの浅いコピーの影響を考慮)
      cv::swap(points_curr, points_prev);//二次元画像座標を保存(points_curr→points_prev)
      cv::swap(camera_point_c, camera_point_p);//三次元カメラ座標を保存(camera_point_c→camera_point_p)
      }
    wall_prev=wall_duration;
    kaisu++;
    //startTime=endTime;
    endTime=startTime;//動作終了時刻取得
    cv::waitKey(1);//ros::spinにジャンプする
}

//観測マーカー調整用関数
void callback2(const struct_slam::marker_tf::ConstPtr& marker_tf_msg){
  marker_tf=*marker_tf_msg;
  std::cout << "観測マーカーと地図上のマーカーとの差を計算(座標)={"<< 
  marker_tf.Tfodometry.pose.pose.position.x <<","<<marker_tf.Tfodometry.pose.pose.position.y<<","<<marker_tf.Tfodometry.pose.pose.position.z<<"}"<< std::endl;
  std::cout << "観測マーカーと地図上のマーカーとの差を計算(姿勢)={"<< 
  marker_tf.Tfodometry.pose.pose.orientation.x <<","<<marker_tf.Tfodometry.pose.pose.orientation.y<<","<<marker_tf.Tfodometry.pose.pose.orientation.z<<","<<marker_tf.Tfodometry.pose.pose.orientation.w<<"}"<< std::endl;
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

	ros::NodeHandle nhSubTf;//ノードハンドル
	//message_filters::Subscriber<struct_slam::marker_tf> marker_sub(nhSubTf, "marker_tf", 1);

  marker_sub = nhSubTf.subscribe("marker_tf", 1, callback2);

  
	ros::spin();//トピック更新待機
			
	return 0;
}






