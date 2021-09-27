///マーカー検出プログラム https://qiita.com/noueezy/items/58730f09f6aa6a97faf5
//マーカー検出時の赤四角が左上になるようにマーカーを配置すると正しくマーカーの中心座標を求められる
//外部パラメータを推定するためには最低でも３点の情報が必要→そこでマーカーのコーナー４点を使用して外部パラメータを算出する

//20210826 システムのサンプリング時間を取得したいがros time nowがうまく使えない（何故か値が出ない）
//そこで今はWalltimeで代用するが、ここで問題が発生する可能性があるので今後修繕が必要
//https://8ttyan.hatenablog.com/entry/2015/02/03/003428
//20210921 ロボットビジョンの手法と日高先生の手法を組み合わせたバージョン
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

using namespace std;
using namespace cv;
float kaisu = 0;//tf用
int kosuu=0,PT_kosuu=0;

bool KAL = false;//カルマン推定データ初回回避用(カルマン動作した時TRUE)
bool reset = true;//初回プログラム用（特徴点検出&特徴点再検出用)
bool swap_on = true;//初回プログラム実行時はswapしない(データの浅いコピーの影響を考慮)


cv::Mat image_curr,image_prev,img_dst,img_dst1;//画像定義
vector<cv::Point2f> points_prev, points_curr;//特徴点定義
vector<cv::Point3f> camera_point_p,camera_point_c;//特徴点定義
float depth_point_prev[300],camera_point_prev[300][3],depth_point_curr[300],camera_point_curr[300][3];
cv::Mat_<float> F_Xp = cv::Mat_<float>(4, 1);//外部パラメータ（日高手法 特徴点)
cv::Mat_<float> F_XpPLAS = cv::Mat_<float>::zeros(4, 1);
cv::Mat_<float>PT_At0,PT_Et0,PT_Tt;

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


////カルマン定義
cv::Mat_<float>xEst0_cl[40]=cv::Mat_<float>(3, 1);//今観測したマーカー座標(カルマン用)
cv::Mat_<float>xEst0_prev_cl[40]=cv::Mat_<float>(3, 1);//一つ前に観測したマーカー座標（カルマン用)
cv::Mat_<float>xEst_cl[40]=cv::Mat_<float>(3, 1);//今推定したマーカー座標(カルマン用)
cv::Mat_<float>xEst_prev_cl[40]=cv::Mat_<float>(3, 1);//一つ前に推定したマーカー座標（カルマン用)
cv::Mat_<float>pixel_cl[40]=cv::Mat_<float>(2, 1);//追跡可能なマーカーの画像座標（カルマン用)
int xEst0_prev_clP[40],xEst0_clP[40],xEst_prev_clP[40];//要素があるかどうか(マッチング用)
cv::Mat_<float>Gt[40]=cv::Mat_<float>(3, 4);
cv::Mat_<float> Mt_cl[40] = cv::Mat_<float>::eye(3, 3);//誤差共分散(予測式)
cv::Mat_<float> Pt_cl[40] = cv::Mat_<float>::eye(3, 3);//誤差共分散(更新式)
cv::Mat_<float> Yt_cl[40] = cv::Mat_<float>::zeros(2, 1);//観測残差
cv::Mat_<float> Ht_cl[40] = cv::Mat_<float>::zeros(2, 3);//Htヤコビアン行列
cv::Mat_<float> Ut_cl[40] = cv::Mat_<float>::zeros(2, 1);//共分散
cv::Mat_<float> St[40] = cv::Mat_<float>::zeros(2, 2);//共分散
cv::Mat_<float> Kt_cl[40] = cv::Mat_<float>::zeros(3, 2);//カルマンゲイン
cv::Mat_<float> u_ = cv::Mat_<float>(2, 2);

//cv::Mat_<float> u_ = (cv::Mat_<float>(2, 2) <<
//    0.083, 0,
//    0, 0.083);

//ofstream outputfile3("/home/fuji/catkin_ws/src/Struct_SLAM/src/EKF/marker_test_ALL_xEst[3].txt");//出力ファイルパス
ofstream outputfile11("/home/fuji/catkin_ws/src/Struct_SLAM/src/EKF/marker_4_ALL_xEst[11].txt");//出力ファイルパス
ofstream outputfile0("/home/fuji/catkin_ws/src/Struct_SLAM/src/EKF/marker_4_ALL_xEst[11]_delta.txt");//出力ファイルパス
//ofstream outputfile6("/home/fuji/catkin_ws/src/Struct_SLAM/src/EKF/xEst[6].txt");//出力ファイルパス
//ofstream outputfile11("/home/fuji/catkin_ws/src/Struct_SLAM/src/EKF/xEst[11].txt");//出力ファイルパス
//ofstream outputfile27("/home/fuji/catkin_ws/src/Struct_SLAM/src/EKF/xEst[27].txt");//出力ファイルパス



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
  double min,max;
  cv::minMaxIdx(depthimage, &min, &max);
  cv::Mat adjimg_depth;
  // Histogram Equalization
  float scale = 255 / (max-min);
  depthimage.convertTo(adjimg_depth,CV_8UC1, scale, -min*scale); 
  cv::Mat falseColorsimg_depth;
  applyColorMap(adjimg_depth, falseColorsimg_depth, cv::COLORMAP_WINTER);//ここのcvで色を変えられる
  cv::imshow("Out", falseColorsimg_depth);

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
    float depth_point0,depth_pointFT;

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
      //Depth値を修正
      //depth_point0=depth_point_curr[i]*0.001;
      //depth_pointFT=39.215*depth_point0*depth_point0-27.793*depth_point0-7.7718;
      //depth_point_curr[i]=depth_point_curr[i]-depth_pointFT;

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

    //cv::Mat_<float>F_Pt=cv::Mat_<float>(depth_point_curr_ok*2, 1);
    //cv::Mat_<float>F_Xt=cv::Mat_<float>(depth_point_curr_ok*2, 1);
    //cv::Mat_<float>F_Ot_1=cv::Mat_<float>(depth_point_curr_ok*2, 4);

    PT_At0=cv::Mat_<float>(depth_point_curr_ok*3, 6);
    PT_Et0=cv::Mat_<float>(depth_point_curr_ok*3, 1);
    PT_Tt=cv::Mat_<float>(6, 1);//教科書運動パラメータ

    PT_kosuu=0;

    //最小二乗法を用いた外部パラメータの算出(日高手法 特徴点)-----------------------------------------------------------------------------
    for(int i=0;i<depth_point_curr_ok;i++){
      //F_Pt(kosuu,0)=camera_point_c[i].x;//マーカーのカメラ座標(観測)
      //F_Pt(kosuu+1,0)=camera_point_c[i].z;

      //F_Ot_1(kosuu,0)=camera_point_p[i].x,F_Ot_1(kosuu,1)=camera_point_p[i].z,F_Ot_1(kosuu,2)=1,F_Ot_1(kosuu,3)=0;
      //F_Ot_1(kosuu+1,0)=camera_point_p[i].z,F_Ot_1(kosuu+1,1)=-camera_point_p[i].x,F_Ot_1(kosuu+1,2)=0,F_Ot_1(kosuu+1,3)=1;

      PT_At0(PT_kosuu,0)=-1,  PT_At0(PT_kosuu,1)=0,   PT_At0(PT_kosuu,2)=0,   PT_At0(PT_kosuu,3)=0,                       PT_At0(PT_kosuu,4)=-camera_point_p[i].z,  PT_At0(PT_kosuu,5)=-camera_point_p[i].y;
      PT_At0(PT_kosuu+1,0)=0, PT_At0(PT_kosuu+1,1)=-1,PT_At0(PT_kosuu+1,2)=0, PT_At0(PT_kosuu+1,3)=camera_point_p[i].z, PT_At0(PT_kosuu+1,4)=0,                     PT_At0(PT_kosuu+1,5)=-camera_point_p[i].x;
      PT_At0(PT_kosuu+2,0)=0, PT_At0(PT_kosuu+2,1)=0, PT_At0(PT_kosuu+2,2)=-1,PT_At0(PT_kosuu+2,3)=-camera_point_p[i].y,PT_At0(PT_kosuu+2,4)=camera_point_p[i].x, PT_At0(PT_kosuu+2,5)=0;

      PT_Et0(PT_kosuu,0)=camera_point_c[i].x-camera_point_p[i].x;
      PT_Et0(PT_kosuu+1,0)=camera_point_c[i].y-camera_point_p[i].y;
      PT_Et0(PT_kosuu+2,0)=camera_point_c[i].z-camera_point_p[i].z;

      PT_kosuu=PT_kosuu+3;
    }
          
    //std::cout <<"特徴点F_Pt=\n"<<F_Pt<< std::endl;
    //std::cout <<"特徴点F_Ot_1=\n"<<F_Ot_1<< std::endl;
      
    //cv::Mat_<float> F_Ot_1T = cv::Mat_<float>(4, 4);
    //F_Ot_1T=F_Ot_1.t()*F_Ot_1;
    //F_Xp=F_Ot_1T.inv(cv::DECOMP_SVD)*F_Ot_1.t()*F_Pt;
    ////std::cout <<"F_Ot_1T=\n"<<F_Ot_1T<< std::endl;
    //std::cout <<"F_Xp=\n"<<F_Xp<< std::endl;
    //F_XpPLAS=F_XpPLAS+F_Xp;
    //std::cout <<"F_XpPLAS=\n"<<F_XpPLAS<< std::endl;

    std::cout <<"PT_At0=\n"<<PT_At0<< std::endl;
    std::cout <<"PT_Et0=\n"<<PT_Et0<< std::endl;
    //すべてのマーカー座標を利用して最小二乗法外部パラメータ推定(初回)
    PT_Tt=PT_At0.inv(cv::DECOMP_SVD)*PT_Et0;
    std::cout <<"PT_Tt=\n"<<PT_Tt<< std::endl;//推定外部パラメータ
    
  }

  
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
  
  //MC_point[i][3],MC_point_prve[][][3]はデータ有無の要素（[][][3]=1ならデータ有り,0ならデータ無し)
  if(kaisu==0){//初回のみ全部初期化
    for(int i=0;i<ALLMarker;i++){
      for(int j=0;j<4;j++){
        MC_point[i][3]=0;//全マーカーのデータ確認用要素初期化
        MC_point_prve[i][3]=0;
        xEst_prev_clP[i]=0;
      }
    }
  }
  //毎回最新MC_pointのみ初期化
  for(int i=0;i<ALLMarker;i++){
    for(int j=0;j<4;j++){
      MC_point[i][3]=0;//全マーカーのデータ確認用要素初期化
    }
  }

  //マーカー観測可能
  if (markerIds.size() > 0) {
    cv::aruco::drawDetectedMarkers(imageCopy, markerCorners, markerIds);//マーカー位置を描画
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
		  cv::circle(imageCopy, cv::Point(MCx[markerIds.at(i)],MCy[markerIds.at(i)]), 3, Scalar(0,255,0),  -1, cv::LINE_AA);//緑点
      cv::aruco::drawAxis(imageCopy,cameraMatrix,distCoeffs,rvecs[i], tvecs[i], 0.1);//マーカーの姿勢描写
      cv::Rodrigues(rvecs[i],rvecs2[i],jacobian[i]);//回転ベクトルから回転行列への変換

      //画像→カメラ座標変換(コーナすべて変換する)----------------------------------------------------------------------
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
        x = (pixel[markerIds.at(i)][0] - camera_info.K[2]) / camera_info.K[0];//ここで正規化座標もしてる
        y = (pixel[markerIds.at(i)][1] - camera_info.K[5]) / camera_info.K[4];
        //camera_info.K[0]=615.337,camera_info.K[2]=324.473,camera_info.K[4]=615.458,camera_info.K[5]=241.696//内部パラメータ

        MC_point[markerIds.at(i)][0] = depth[markerIds.at(i)] * x/1000;//メートル表示変換
        MC_point[markerIds.at(i)][1] = depth[markerIds.at(i)] * y/1000;
        MC_point[markerIds.at(i)][2] = depth[markerIds.at(i)]/1000;
        MC_point[markerIds.at(i)][3] = 1;//データ取得可能なら1
        std::cout << "特徴点のカメラ座標:MC_point["<<markerIds.at(i)<<"]={"<< MC_point[markerIds.at(i)][0] <<","<<MC_point[markerIds.at(i)][1]<<","<<MC_point[markerIds.at(i)][2]<<"}"<< std::endl;
        //std::cout <<"X="<< MC_point[markerIds.at(i)][0]/MC_point[markerIds.at(i)][2]<< std::endl;
        //std::cout <<"Y="<< MC_point[markerIds.at(i)][1]/MC_point[markerIds.at(i)][2]<< std::endl;
      }
    }
    //リサイズ前定義
    cv::Mat_<float>Pt0=cv::Mat_<float>(markerIds.size()*2, 1);
    cv::Mat_<float>Ot0_1=cv::Mat_<float>(markerIds.size()*2, 6);
    cv::Mat_<float>At0=cv::Mat_<float>(markerIds.size()*3, 6);
    cv::Mat_<float>Et0=cv::Mat_<float>(markerIds.size()*3, 1);
    cv::Mat_<float>AWt0=cv::Mat_<float>(markerIds.size()*3, 4);//誤差共分散を求めるときに使用する式AW
    cv::Mat_<float>delta0=cv::Mat_<float>(markerIds.size()*3, 1);//共分散deltaの内部要素
    float Xp,Yp;
    float deltaX=0,deltaY=0,deltaZ=0,Ave_pointX=0,Ave_pointY=0,Ave_pointZ=0,Ave_point_prvX=0,Ave_point_prvY=0,Ave_point_prvZ=0;

    kosuu=0;
    //最小二乗法を用いた外部パラメータの算出(ロボットビジョン教科書手法)-----------------------------------------------------------------------------
    //マーカーが追跡可能かどうかを見る(pointとMC_point_prveで同じマーカーがあれば追跡可能)
    if(kaisu!=0){
      if(KAL==false){//カルマン推定前はこっち
        for(int i=0;i<ALLMarker;i++){//全マーカー分回す
          if(MC_point[i][3]!=0){//最新要素が無いものを除外
            if(MC_point[i][3]==MC_point_prve[i][3]){//追跡可能かどうか
            std::cout << "point     ["<<i<<"]={"<< MC_point[i][0] <<","<<MC_point[i][1]<<","<<MC_point[i][2]<<","<<MC_point[i][3]<<"}"<< std::endl;
            std::cout << "point_prev["<<i<<"]={"<< MC_point_prve[i][0] <<","<<MC_point_prve[i][1]<<","<<MC_point_prve[i][2]<<","<<MC_point_prve[i][3]<<"}"<< std::endl;

            xEst0_prev_cl[i](0,0)=MC_point_prve[i][0];//一つ前に観測したマーカーのカメラ座標（カルマン用)
            xEst0_prev_cl[i](1,0)=MC_point_prve[i][1];
            xEst0_prev_cl[i](2,0)=MC_point_prve[i][2];
            xEst0_prev_clP[i]=1;//要素がある時は1(マッチング確認用)

            xEst0_cl[i](0,0)=MC_point[i][0];//今観測したマーカーのカメラ座標(カルマン用)
            xEst0_cl[i](1,0)=MC_point[i][1];
            xEst0_cl[i](2,0)=MC_point[i][2];
            xEst0_clP[i]=1;//要素があるときは1;(マッチング確認用)

            pixel_cl[i](0,0)=pixel[i][0] - camera_info.K[2];//今観測したマーカーの正規化画像座標(カルマン用)
            pixel_cl[i](1,0)=pixel[i][1] - camera_info.K[5];

            At0(kosuu,0)=-1,  At0(kosuu,1)=0,   At0(kosuu,2)=0,   At0(kosuu,3)=0,                       At0(kosuu,4)=-xEst0_prev_cl[i](2,0),  At0(kosuu,5)=-xEst0_prev_cl[i](1,0);
            At0(kosuu+1,0)=0, At0(kosuu+1,1)=-1,At0(kosuu+1,2)=0, At0(kosuu+1,3)=xEst0_prev_cl[i](2,0), At0(kosuu+1,4)=0,                     At0(kosuu+1,5)=-xEst0_prev_cl[i](0,0);
            At0(kosuu+2,0)=0, At0(kosuu+2,1)=0, At0(kosuu+2,2)=-1,At0(kosuu+2,3)=-xEst0_prev_cl[i](1,0),At0(kosuu+2,4)=xEst0_prev_cl[i](0,0), At0(kosuu+2,5)=0;

            Et0(kosuu,0)=xEst0_cl[i](0,0)-xEst0_prev_cl[i](0,0);
            Et0(kosuu+1,0)=xEst0_cl[i](1,0)-xEst0_prev_cl[i](1,0);
            Et0(kosuu+2,0)=xEst0_cl[i](2,0)-xEst0_prev_cl[i](2,0);

            //誤差共分散を求めるときに使用する式AW
            AWt0(kosuu,0)=-1,   AWt0(kosuu,1)=0,    AWt0(kosuu,2)=0,    AWt0(kosuu,3)=-xEst0_prev_cl[i](2,0);
            AWt0(kosuu+1,0)=0,  AWt0(kosuu+1,1)=-1, AWt0(kosuu+1,2)=0,  AWt0(kosuu+1,3)=0;
            AWt0(kosuu+2,0)=0,  AWt0(kosuu+2,1)=0,  AWt0(kosuu+2,2)=-1, AWt0(kosuu+2,3)=xEst0_prev_cl[i](0,0);

            //共分散の内部要素(観測値ー前の推定値)
            delta0(kosuu,0)=(xEst0_cl[i](0,0)-xEst0_prev_cl[i](0,0))*(xEst0_cl[i](0,0)-xEst0_prev_cl[i](0,0));
            delta0(kosuu+1,0)=(xEst0_cl[i](1,0)-xEst0_prev_cl[i](1,0))*(xEst0_cl[i](1,0)-xEst0_prev_cl[i](1,0));
            delta0(kosuu+2,0)=(xEst0_cl[i](2,0)-xEst0_prev_cl[i](2,0))*(xEst0_cl[i](2,0)-xEst0_prev_cl[i](2,0));

            kosuu=kosuu+3;
            }
            //追跡できない時
            else{xEst0_prev_clP[i]=0,xEst0_clP[i]=0;}//要素がない時は0(マッチング確認用)
          }
          //最新要素が無い時
          else{xEst0_prev_clP[i]=0,xEst0_clP[i]=0;}//要素がない時は0(マッチング確認用)
        }
      }
      else{//カルマン推定後
      std::cout << "カルマン推定後"<< std::endl;
        for(int i=0;i<ALLMarker;i++){//全マーカー分回す
          if(MC_point[i][3]!=0){//最新要素が無いものを除外
            std::cout << "MC_point[i="<<i<<"][3]="<<MC_point[i][3]<<",xEst_prev_clP[i="<<i<<"]="<<xEst_prev_clP[i]<< std::endl;
            if(MC_point[i][3]==xEst_prev_clP[i]){//追跡可能かどうか
              std::cout << "point       ["<<i<<"]={"<< MC_point[i][0] <<","<<MC_point[i][1]<<","<<MC_point[i][2]<<","<<MC_point[i][3]<<"}"<< std::endl;
              std::cout << "xEst_prev_cl["<<i<<"]={"<< xEst_prev_cl[i](0,0) <<","<<xEst_prev_cl[i](1,0)<<","<<xEst_prev_cl[i](2,0)<<","<<xEst_prev_clP[i]<<"}"<< std::endl;
              std::cout << "Pt_cl       ["<<i<<"]=\n"<<Pt_cl[i]<< std::endl;//１つ前の推定誤差共分散

              xEst0_prev_cl[i](0,0)=xEst_prev_cl[i](0,0);//一つ前に推定したマーカーのカメラ座標（カルマン用)
              xEst0_prev_cl[i](1,0)=xEst_prev_cl[i](1,0);
              xEst0_prev_cl[i](2,0)=xEst_prev_cl[i](2,0);

              xEst0_cl[i](0,0)=MC_point[i][0];//今観測したマーカーのカメラ座標(カルマン用)
              xEst0_cl[i](1,0)=MC_point[i][1];
              xEst0_cl[i](2,0)=MC_point[i][2];
              xEst0_clP[i]=1;//要素があるときは1;(マッチング確認用)

              pixel_cl[i](0,0)=pixel[i][0] - camera_info.K[2];//今観測したマーカーの正規化画像座標
              pixel_cl[i](1,0)=pixel[i][1] - camera_info.K[5];

             At0(kosuu,0)=-1,  At0(kosuu,1)=0,   At0(kosuu,2)=0,   At0(kosuu,3)=0,                       At0(kosuu,4)=-xEst0_prev_cl[i](2,0),  At0(kosuu,5)=-xEst0_prev_cl[i](1,0);
             At0(kosuu+1,0)=0, At0(kosuu+1,1)=-1,At0(kosuu+1,2)=0, At0(kosuu+1,3)=xEst0_prev_cl[i](2,0), At0(kosuu+1,4)=0,                     At0(kosuu+1,5)=-xEst0_prev_cl[i](0,0);
             At0(kosuu+2,0)=0, At0(kosuu+2,1)=0, At0(kosuu+2,2)=-1,At0(kosuu+2,3)=-xEst0_prev_cl[i](1,0),At0(kosuu+2,4)=xEst0_prev_cl[i](0,0), At0(kosuu+2,5)=0;

             Et0(kosuu,0)=xEst0_cl[i](0,0)-xEst0_prev_cl[i](0,0);
             Et0(kosuu+1,0)=xEst0_cl[i](1,0)-xEst0_prev_cl[i](1,0);
             Et0(kosuu+2,0)=xEst0_cl[i](2,0)-xEst0_prev_cl[i](2,0);

             AWt0(kosuu,0)=-1,   AWt0(kosuu,1)=0,    AWt0(kosuu,2)=0,    AWt0(kosuu,3)=-xEst0_prev_cl[i](2,0);
             AWt0(kosuu+1,0)=0,  AWt0(kosuu+1,1)=-1, AWt0(kosuu+1,2)=0,  AWt0(kosuu+1,3)=0;
             AWt0(kosuu+2,0)=0,  AWt0(kosuu+2,1)=0,  AWt0(kosuu+2,2)=-1, AWt0(kosuu+2,3)=xEst0_prev_cl[i](0,0);

             //共分散の内部要素
             delta0(kosuu,0)=(xEst0_cl[i](0,0)-xEst0_prev_cl[i](0,0))*(xEst0_cl[i](0,0)-xEst0_prev_cl[i](0,0));
             delta0(kosuu+1,0)=(xEst0_cl[i](1,0)-xEst0_prev_cl[i](1,0))*(xEst0_cl[i](1,0)-xEst0_prev_cl[i](1,0));
             delta0(kosuu+2,0)=(xEst0_cl[i](2,0)-xEst0_prev_cl[i](2,0))*(xEst0_cl[i](2,0)-xEst0_prev_cl[i](2,0));

             kosuu=kosuu+3;
            }
            //追跡できない時
            else{xEst0_prev_clP[i]=0,xEst0_clP[i]=0;}//要素がない時は0(マッチング確認用)
            //新しくマーカーが観測された時
            if(MC_point[i][3]==1&&xEst_prev_clP[i]==0){
              xEst_prev_cl[i](0,0)=MC_point[i][0];//今観測したマーカーのカメラ座標（カルマン用)
              xEst_prev_cl[i](1,0)=MC_point[i][1];
              xEst_prev_cl[i](2,0)=MC_point[i][2];
            }   
          }
          //最新要素が無い時
          else{xEst0_prev_clP[i]=0,xEst0_clP[i]=0;}//要素がない時は0(マッチング確認用)
        }
      }
      for(int i=0;i<ALLMarker;i++){
        if(xEst0_clP[i]==1){
          std::cout << "xEst0_prev_cl["<<i<<"]=\n"<<xEst0_prev_cl[i]<< std::endl;
          std::cout << "xEst0_cl["<<i<<"]=\n"<<xEst0_cl[i]<< std::endl;
        }
      }
      //outputfile3<<"　　推定結果　　 "<<"               "<<"　観測データ\n";
      //outputfile3<<"xEst0_prev_cl[11]="<<xEst0_prev_cl[11](0,0)<<",xEst0_cl[11]="<<xEst0_cl[11](0,0)<<"\n";
      //outputfile3<<"                  "<<xEst0_prev_cl[11](1,0)<<"              "<<xEst0_cl[11](1,0)<<"\n";
      //outputfile3<<"                  "<<xEst0_prev_cl[11](2,0)<<"              "<<xEst0_cl[11](2,0)<<"\n";
      


      //リサイズ用行列定義
      cv::Mat_<float>At=cv::Mat_<float>(kosuu, 6);
      cv::Mat_<float>At_1=cv::Mat_<float>(kosuu, kosuu);
      cv::Mat_<float>Et=cv::Mat_<float>(kosuu, 1);
      cv::Mat_<float>AWt=cv::Mat_<float>(kosuu, 4);//誤差共分散を求めるときに使用する式AW
      cv::Mat_<float>delta=cv::Mat_<float>(kosuu, 1);//共分散deltaの内部要素

      cv::Mat_<float>Tt=cv::Mat_<float>(6, 1);//教科書運動パラメータ

      // Pt0の第0行から第kosuu行をPtの第0行から第kosuu行に代入(リサイズ)
	    At0.rowRange(0, kosuu).copyTo(At.rowRange(0, kosuu));
	    Et0.rowRange(0, kosuu).copyTo(Et.rowRange(0, kosuu));
	    AWt0.rowRange(0, kosuu).copyTo(AWt.rowRange(0, kosuu));
	    delta0.rowRange(0, kosuu).copyTo(delta.rowRange(0, kosuu));

      std::cout <<"At=\n"<<At<< std::endl;
      std::cout <<"Et=\n"<<Et<< std::endl;
      std::cout <<"AWt=\n"<<AWt<< std::endl;

      /*//すべてのマーカー座標を利用して最小二乗法外部パラメータ推定(初回)
      Tt=At.inv(cv::DECOMP_SVD)*Et;
      //At_1=At.t()*At;
      //Tt=At_1.inv()*At.t()*Et;
      std::cout <<"Tt=\n"<<Tt<< std::endl;//推定外部パラメータ
      
      Ft = cv::Mat_<float>(3, 3);//回転行列(日高手法)
      Ft(0,0)=1,        Ft(0,1)=0,        Ft(0,2)=-Tt(4,0);
      Ft(1,0)=0,        Ft(1,1)=1,        Ft(1,2)=0,
      Ft(2,0)=Tt(4,0),  Ft(2,1)=0,        Ft(2,2)=1;
      std::cout <<"回転行列(日高手法)Ft=\n"<<Ft<< std::endl;//回転行列

      Vt = cv::Mat_<float>(4, 1);//並進ベクトル+y軸回転(日高手法)
      Vt(0,0)=Tt(0,0);//Vx
      Vt(1,0)=Tt(1,0);//Vy
      Vt(2,0)=Tt(2,0);//Vz
      Vt(3,0)=Tt(4,0);//Ωy
      std::cout <<"並進ベクトル+y軸回転(日高手法)Vt=\n"<<Vt<< std::endl;//並進ベクトル*/

      //マーカー+特徴点を使用した外部パラメータ推定
      cv::Mat_<float>ALL_At=cv::Mat_<float>(kosuu+PT_kosuu, 6);
      cv::Mat_<float>ALL_Et=cv::Mat_<float>(kosuu+PT_kosuu, 1);
      cv::Mat_<float>ALL_Tt=cv::Mat_<float>(6, 1);//教科書運動パラメータ

      At.rowRange(0, kosuu).copyTo(ALL_At.rowRange(0, kosuu));
	    Et.rowRange(0, kosuu).copyTo(ALL_Et.rowRange(0, kosuu));
      PT_At0.rowRange(0, PT_kosuu).copyTo(ALL_At.rowRange(kosuu, kosuu+PT_kosuu));
	    PT_Et0.rowRange(0, PT_kosuu).copyTo(ALL_Et.rowRange(kosuu, kosuu+PT_kosuu));

      //std::cout <<"ALL_At=\n"<<ALL_At<< std::endl;
      //std::cout <<"ALL_Et=\n"<<ALL_Et<< std::endl;

      //すべてのマーカー座標を利用して最小二乗法外部パラメータ推定(初回)
      ALL_Tt=ALL_At.inv(cv::DECOMP_SVD)*ALL_Et;
      std::cout <<"ALL_Tt=\n"<<ALL_Tt<< std::endl;//推定外部パラメータ

      ALL_Ft = cv::Mat_<float>(3, 3);//回転行列(日高手法)
      ALL_Ft(0,0)=1,        ALL_Ft(0,1)=0,        ALL_Ft(0,2)=-ALL_Tt(4,0);
      ALL_Ft(1,0)=0,        ALL_Ft(1,1)=1,        ALL_Ft(1,2)=0,
      ALL_Ft(2,0)=ALL_Tt(4,0),  ALL_Ft(2,1)=0,        ALL_Ft(2,2)=1;
      std::cout <<"回転行列(日高手法)ALL_Ft=\n"<<ALL_Ft<< std::endl;//回転行列

      ALL_Vt = cv::Mat_<float>(4, 1);//並進ベクトル+y軸回転(日高手法)
      ALL_Vt(0,0)=ALL_Tt(0,0);//Vx
      ALL_Vt(1,0)=ALL_Tt(1,0);//Vy
      ALL_Vt(2,0)=ALL_Tt(2,0);//Vz
      ALL_Vt(3,0)=0;

      //ALL_Vt(3,0)=ALL_Tt(4,0);//Ωy
      std::cout <<"並進ベクトル+y軸回転(日高手法)ALL_Vt=\n"<<ALL_Vt<< std::endl;//並進ベクトル


      //カルマン初期設定(カルマン初回のみ実行)
      if(KAL==false){
         Gt[ALLMarker]=cv::Mat_<float>(3, 4);
         Mt_cl[ALLMarker] = cv::Mat_<float>::zeros(3, 3);//誤差共分散(予測式)
         Yt_cl[ALLMarker] = cv::Mat_<float>::zeros(2, 1);//観測残差
         Ht_cl[ALLMarker] = cv::Mat_<float>::zeros(2, 3);//Htヤコビアン行列
         Ut_cl[ALLMarker] = cv::Mat_<float>::zeros(2, 1);//共分散
         St[ALLMarker] = cv::Mat_<float>::zeros(2, 2);//共分散
         Kt_cl[ALLMarker] = cv::Mat_<float>::zeros(3, 2);//カルマンゲイン
         u_ = (cv::Mat_<float>(2, 2) <<
            0.0001, 0,
            0, 0.0001);
      }

      //カルマンフィルタでマーカーの座標修正(座標推定)
      //予測ステップ----------------------------------------------------------------------------------------------------
      //状態モデルの予測式
      for(int i=0;i<ALLMarker;i++){
        if(xEst0_clP[i]==1){//要素がある時のみ実行

          Gt[i](0,0)=-1,Gt[i](0,1)=0, Gt[i](0,2)=0, Gt[i](0,3)=-xEst0_prev_cl[i](2,0);
          Gt[i](1,0)=0, Gt[i](1,1)=-1,Gt[i](1,2)=0, Gt[i](1,3)=-0;
          Gt[i](2,0)=0, Gt[i](2,1)=0, Gt[i](2,2)=-1,Gt[i](2,3)=xEst0_prev_cl[i](0,0);
          std::cout <<"Gt["<<i<<"]=\n"<<Gt[i]<< std::endl;

          xEst_cl[i]=ALL_Ft*xEst0_prev_cl[i]+Gt[i]*ALL_Vt;//カルマン初回動作はこっち
          //xEst_cl[i]=Ft*xEst0_prev_cl[i]+Gt[i]*Vt;//カルマン初回動作はこっち
          std::cout <<"xEst_cl["<<i<<"]=\n"<<xEst_cl[i]<< std::endl;
        }
      }


      //誤差共分散の予測式-----------------------------------
      cv::Mat eye = cv::Mat_<float>::eye(kosuu, kosuu);// 単位行列kosuu, 4
      cv::Mat_<float>eyetemp=cv::Mat_<float>(1, kosuu);
      cv::Mat_<float>deltaeye=cv::Mat_<float>(kosuu, kosuu);
      cv::Mat_<float>AWt_1=cv::Mat_<float>(kosuu, 4);
      cv::Mat_<float>Wt = cv::Mat_<float>::zeros(4, 4);

      for(int i=0;i<kosuu;i++){//共分散の対角化
        eye.rowRange(i, i+1).copyTo(eyetemp.rowRange(0, 1));
        eyetemp=eyetemp*delta(i,0)*0.083;
        eyetemp.rowRange(0, 1).copyTo(deltaeye.rowRange(i, i+1));
      }
      std::cout <<"deltaeye=\n"<<deltaeye<< std::endl;
      outputfile0<<"deltaeye=\n"<<deltaeye<<"\n";

      AWt_1=AWt.inv(cv::DECOMP_SVD);//擬似逆行列
      std::cout <<"AWt_1=\n"<<AWt_1<< std::endl;
      Wt=AWt_1*deltaeye*AWt_1.t();
      std::cout <<"Wt=\n"<<Wt<< std::endl;

      for(int i=0;i<ALLMarker;i++){
        if(xEst0_clP[i]==1){//要素がある時のみ実行
          if(KAL==false){//カルマン初回実行時はこっち
            Mt_cl[i]= (cv::Mat_<float>(3, 3) <<
              0.1, 0, 0,
              0, 0.1, 0,
              0, 0, 0.1);//Mt_cl[i]の初期値
          }
          else{Mt_cl[i]=ALL_Ft*Pt_cl[i]*ALL_Ft.t()+Gt[i]*Wt*Gt[i].t();}
          //Mt_cl[i]=ALL_Ft*Pt_cl[i]*ALL_Ft.t()+Gt[i]*Wt*Gt[i].t();
          //Mt_cl[i]=Ft*Pt_cl[i]*Ft.t()+Gt[i]*Wt*Gt[i].t();
          std::cout <<"Pt_cl[i="<<i<<"]=\n"<<Pt_cl[i]<< std::endl;
          std::cout <<"Mt_cl["<<i<<"]=\n"<<Mt_cl[i]<< std::endl;//誤差共分散の予測式Mt
        }
      }
      //更新ステップ----------------------------------------------------------------------
      //観測残差
      for(int i=0;i<ALLMarker;i++){
        if(xEst0_clP[i]==1){//要素がある時のみ実行
          Ht_cl[i]= (cv::Mat_<float>(2, 3) <<
            camera_info.K[0]/xEst0_prev_cl[i](2,0), 0,                                      -camera_info.K[0]*xEst0_prev_cl[i](0,0)/xEst0_prev_cl[i](2,0)*xEst0_prev_cl[i](2,0),
            0,                                      camera_info.K[0]/xEst0_prev_cl[i](2,0), -camera_info.K[0]*xEst0_prev_cl[i](1,0)/xEst0_prev_cl[i](2,0)*xEst0_prev_cl[i](2,0));

          Ut_cl[i]= (cv::Mat_<float>(2, 1) <<
            camera_info.K[0]*xEst0_prev_cl[i](0,0)/xEst0_prev_cl[i](2,0),
            camera_info.K[0]*xEst0_prev_cl[i](1,0)/xEst0_prev_cl[i](2,0));

          std::cout <<"Ht_cl["<<i<<"]=\n"<<Ht_cl[i]<< std::endl;
          std::cout <<"Ut_cl["<<i<<"]=\n"<<Ut_cl[i]<< std::endl;

          Yt_cl[i]=pixel_cl[i]-(Ht_cl[i]*xEst_cl[i]+Ut_cl[i]);
          std::cout <<"pixel["<<i<<"][0]="<<pixel[i][0]<< std::endl;
          std::cout <<"pixel["<<i<<"][1]="<<pixel[i][1]<< std::endl;
          std::cout <<"pixel_cl["<<i<<"]=\n"<<pixel_cl[i]<< std::endl;
          std::cout <<"xEst_cl["<<i<<"]=\n"<<xEst_cl[i]<< std::endl;
          std::cout <<"Ht_cl["<<i<<"]*xEst_cl["<<i<<"]=\n"<<Ht_cl[i]*xEst_cl[i]<< std::endl;
          std::cout <<"Ht_cl["<<i<<"]*xEst_cl["<<i<<"]+Ut_cl["<<i<<"]=\n"<<Ht_cl[i]*xEst_cl[i]+Ut_cl[i]<< std::endl;

          std::cout <<"観測残差:Yt_cl["<<i<<"]=\n"<<Yt_cl[i]<< std::endl;//観測残差

          St[i]=Ht_cl[i]*Mt_cl[i]*Ht_cl[i].t()+u_;
          std::cout <<"観測残差の共分散:St["<<i<<"]=\n"<<St[i]<< std::endl;

          Pt_cl[i]=Mt_cl[i].inv()+Ht_cl[i].t()*u_.inv()*Ht_cl[i];
          Pt_cl[i]=Pt_cl[i].inv();
          std::cout <<"誤差共分散の更新:Pt_cl["<<i<<"]=\n"<<Pt_cl[i]<< std::endl;//誤差共分散の更新

          //Kt_cl[i]=Pt_cl[i]*Ht_cl[i].t()*u_.inv();
          Kt_cl[i]=Pt_cl[i]*Ht_cl[i].t()*St[i].inv();
          std::cout <<"u_.inv()=\n"<<u_.inv()<< std::endl;
          std::cout <<"カルマンゲイン:Kt_cl["<<i<<"]=\n"<<Kt_cl[i]<< std::endl;//カルマンゲイン

          xEst_cl[i]=xEst_cl[i]+Kt_cl[i]*Yt_cl[i];
          std::cout <<"状態モデルの更新:xEst_cl["<<i<<"]=\n"<<xEst_cl[i]<< std::endl;//状態モデルの更新

          xEst_prev_cl[i]=xEst_cl[i];//現在の推定を保存
          xEst_prev_clP[i]=1;
          std::cout <<"xEst_prev_clP["<<i<<"]="<<xEst_prev_clP[i]<< std::endl;
          KAL=true;//次の動作の時カルマンの推定結果と観測値の追跡を確認する
        }
        //新しくマーカーが観測された時
        else if(MC_point[i][3]==1&&xEst_prev_clP[i]==0){
          xEst_prev_clP[i]=1;
          std::cout <<"新しくマーカーが観測された時 xEst_prev_clP["<<i<<"]="<<xEst_prev_clP[i]<< std::endl;
        }
        //更新データが存在しない時
        else{xEst_prev_clP[i]=0;
          //std::cout <<"ELSE xEst_prev_clP["<<i<<"]="<<xEst_prev_clP[i]<< std::endl;
        }
      }
      outputfile11<<"　　推定結果　　 "<<"               "<<"　観測データ\n";
      outputfile11<<"xEst0_prev_cl[11]="<<xEst0_prev_cl[11](0,0)<<",xEst0_cl[11]="<<xEst0_cl[11](0,0)<<",カルマンゲイン:Kt_cl[11]="<<Kt_cl[11](0,0)<<","<<Kt_cl[11](0,1)<<",外部パラメータ推定:Tt="<<Tt(0,0)<<",  "<<Tt(3,0)<<",外部パラメータ推定:ALL_Tt="<<ALL_Tt(0,0)<<",  "<<ALL_Tt(3,0)<<"\n";
      outputfile11<<"                  "<<xEst0_prev_cl[11](1,0)<<"              "<<xEst0_cl[11](1,0)<<"                          "<<Kt_cl[11](1,0)<<","<<Kt_cl[11](1,1)<<"                       "<<Tt(1,0)<<",  "<<Tt(4,0)<<"                           "<<ALL_Tt(1,0)<<",  "<<ALL_Tt(4,0)<<"\n";
      outputfile11<<"                  "<<xEst0_prev_cl[11](2,0)<<"              "<<xEst0_cl[11](2,0)<<"                             "<<Kt_cl[11](2,0)<<","<<Kt_cl[11](2,1)<<"                       "<<Tt(2,0)<<",  "<<Tt(5,0)<<"                           "<<ALL_Tt(2,0)<<",  "<<ALL_Tt(5,0)<<"\n";
      //outputfile3<<"　　推定結果　　 "<<"               "<<"　観測データ\n";
      //outputfile3<<"xEst0_prev_cl[3]="<<xEst0_prev_cl[3](0,0)<<",xEst0_cl[3]="<<xEst0_cl[3](0,0)<<",カルマンゲイン:Kt_cl[3]="<<Kt_cl[3](0,0)<<","<<Kt_cl[3](0,1)<<",外部パラメータ推定:Tt="<<Tt(0,0)<<",  "<<Tt(3,0)<<",外部パラメータ推定:ALL_Tt="<<ALL_Tt(0,0)<<",  "<<ALL_Tt(3,0)<<"\n";
      //outputfile3<<"                 "<<xEst0_prev_cl[3](1,0)<<"             "<<xEst0_cl[3](1,0)<<"                         "<<Kt_cl[3](1,0)<<","<<Kt_cl[3](1,1)<<"                       "<<Tt(1,0)<<",  "<<Tt(4,0)<<"                           "<<ALL_Tt(1,0)<<",  "<<ALL_Tt(4,0)<<"\n";
      //outputfile3<<"                 "<<xEst0_prev_cl[3](2,0)<<"             "<<xEst0_cl[3](2,0)<<"                            "<<Kt_cl[3](2,0)<<","<<Kt_cl[3](2,1)<<"                       "<<Tt(2,0)<<",  "<<Tt(5,0)<<"                           "<<ALL_Tt(2,0)<<",  "<<ALL_Tt(5,0)<<"\n";

    }//if(kaisu!=0)→end(初回は動作しない)
  }//if(markerIds.size() > 0)→end(マーカーが観測されなかった時は動作しない)

  //メモ20210914
  //カルマンフィルタの構築を行なっている。とりあえず初回動作分の構築は書いた（結果は見ていない）
  //次に行うこととして、推定した値を再び予測ステップに戻して再度推定を行う必要がある。
  //これにあたりいくつかわからないことがある
  //１つはカルマン推定後の動作について
  //カルマンで推定した後推定した値を使って外部パラメータを求めるが、カルマン中にもカメラは動作しているのでおかしくならないのか？
  //これについては多分カルマン推定中に観測データを入れているので、そこで動作の更新が起こるからおかしくならないかも？
  //ふたつ目はどうやってカルマン初回かどうかを分けるか
  //またマーカーが観測されなかった時はそのマーカーのカルマン推定は行わないので、観測に応じてカルマンの推定も変えないと行けない
  //つまり推定した値と観測の値で再び追跡可能かどうかを検出し、追跡可能だった場合はカルマンを行う
  //追跡が行えなかった場合はカルマン推定はしない

  //また今現在追跡できるかどうかを見てるが新規のマーカーが現れた時はどういう動作をするかをもう一度分析する必要がある
  //新規マーカーが現れた時の動きは、新規マーカーが確認された時はカルマン推定などは行わず座標のみ保存
  //そして次の動作時に観測され、そのマーカーが追跡可能ならばカルマン推定を行う

  //ちなみに一度追跡が不可能になったらそのマーカーの推定結果は削除する
   
  
  // 画面表示
  cv::imshow(win_src, image);
  cv::imshow(win_dst, imageCopy);
  //cv::imshow(win_depth, depthimage);

  for(int i=0;i<markerIds.size();i++){
    MC_point_prve[markerIds.at(i)][0]=MC_point[markerIds.at(i)][0];//今のカメラ座標を保存
    MC_point_prve[markerIds.at(i)][1]=MC_point[markerIds.at(i)][1];
    MC_point_prve[markerIds.at(i)][2]=MC_point[markerIds.at(i)][2];
    MC_point_prve[markerIds.at(i)][3]=MC_point[markerIds.at(i)][3];//データが取得可能かどうか
  }
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

  
	ros::spin();//トピック更新待機
			
	return 0;
}






