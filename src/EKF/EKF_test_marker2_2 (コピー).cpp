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
float depth_point_prve[300],camera_point_prve[300][3],depth_point_curr[300],camera_point_curr[300][3];
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
float point_prve[50][4][4];//一つ前のカメラ座標
cv::Mat_<float> Xp = cv::Mat_<float>(4, 1);//外部パラメータ（日高手法)
cv::Mat_<float> XpPLAS = cv::Mat_<float>::zeros(4, 1);
float pixel[50][4][2],depth[100],point[50][4][4],x,y,r2,f,ux,uy;//画像→カメラ座標変換

ros::Time ros_begin;//プログラム時間
ros::WallTime wall_begin = ros::WallTime::now();//プログラム開始時の時間
ros::WallDuration wall_prev;//一つ前の時間
ros::WallDuration wall_systemtime;//サンプリング時間
//ros::Time ros_begin = ros::Time::now();
struct timeval startTime, endTime;  // 構造体宣言
float realsec;//サンプリング時間（C++)

//cv::Mat_<float> xEst = cv::Mat_<float>::zeros(3, 1);//状態方程式（カメラ）
cv::Mat_<float> Ft = cv::Mat_<float>(3, 3);//回転行列
//cv::Mat_<float> Vt = cv::Mat_<float>(3, 1);//並進ベクトル

int ALLMarker=40;//全マーカー個数
//カルマンフィルタ
cv::Mat_<float> xEst;
//cv::Mat_<float> xEst_prev;
cv::Mat_<float> Vt;

////カルマン定義
//cv::Mat_<float> Wt = cv::Mat_<float>::zeros(6, 6);
//cv::Mat_<float> Mt_cl[40*4] = cv::Mat_<float>::zeros(3, 3);//誤差共分散(予測式)
//cv::Mat_<float> Pt_cl[40*4] = cv::Mat_<float>::zeros(3, 3);//誤差共分散(更新式)
//cv::Mat_<float> Yt_cl[40*4] = cv::Mat_<float>::zeros(2, 1);//観測残差
//cv::Mat_<float> Ht_cl[40*4] = cv::Mat_<float>::zeros(2, 3);//Htヤコビアン行列
//cv::Mat_<float> Ut_cl[40*4] = cv::Mat_<float>::zeros(2, 1);//共分散
//cv::Mat_<float> Kt_cl[40*4] = cv::Mat_<float>::zeros(3, 2);//カルマンゲイン
//cv::Mat_<float> u_ = (cv::Mat_<float>(2, 2) <<
//    1/12, 0,
//    0, 1/12);


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
  image.copyTo(img_dst1);//特徴点検出結果表示用

  cv::Mat imageCopy = image.clone();
  cv::Mat_<float> intrinsic_K= cv::Mat_<float>(3, 3);
	std::cout <<"画像取り込み"<< std::endl;
  
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
  
  kosuu=0;
  //point[i][j][3],point_prve[][][3]はデータ有無の要素（[][][3]=1ならデータ有り,0ならデータ無し)
  if(kaisu==0){//初回のみ全部初期化
    for(int i=0;i<ALLMarker;i++){
      for(int j=0;j<4;j++){
        point[i][j][3]=0;//全マーカーのデータ確認用要素初期化
        point_prve[i][j][3]=0;
      }
    }
  }
  //毎回最新pointのみ初期化
  for(int i=0;i<ALLMarker;i++){
    for(int j=0;j<4;j++){
      point[i][j][3]=0;//全マーカーのデータ確認用要素初期化
    }
  }

  //マーカー観測可能
  if (markerIds.size() > 0) {
    cv::aruco::drawDetectedMarkers(imageCopy, markerCorners, markerIds);//マーカー位置を描画
    cv::aruco::estimatePoseSingleMarkers(markerCorners, 0.05, cameraMatrix, distCoeffs, rvecs, tvecs);//マーカーの姿勢推定

    for(int i=0;i<markerIds.size();i++){
      std::cout <<"マーカーの個数:markerIds.size()="<<markerIds.size() << std::endl;//マーカー個数
      std::cout <<"markerIds("<<i<<")="<< markerIds.at(i) << std::endl;//マーカーID
      
      MarkerC[markerIds.at(i)][0][0]=markerCorners[i][0].x, MarkerC[markerIds.at(i)][0][1]=markerCorners[i][0].y;//コーナーの画像座標ID対応化
      MarkerC[markerIds.at(i)][1][0]=markerCorners[i][1].x, MarkerC[markerIds.at(i)][1][1]=markerCorners[i][1].y;
      MarkerC[markerIds.at(i)][2][0]=markerCorners[i][2].x, MarkerC[markerIds.at(i)][2][1]=markerCorners[i][2].y;
      MarkerC[markerIds.at(i)][3][0]=markerCorners[i][3].x, MarkerC[markerIds.at(i)][3][1]=markerCorners[i][3].y;

		  cv::circle(imageCopy, cv::Point(MarkerC[markerIds.at(i)][0][0], MarkerC[markerIds.at(i)][0][1]), 3, Scalar(0,255,0),   -1, cv::LINE_AA);//緑点
		  cv::circle(imageCopy, cv::Point(MarkerC[markerIds.at(i)][1][0], MarkerC[markerIds.at(i)][1][1]), 3, Scalar(255,255,0), -1, cv::LINE_AA);//水色
		  cv::circle(imageCopy, cv::Point(MarkerC[markerIds.at(i)][2][0], MarkerC[markerIds.at(i)][2][1]), 3, Scalar(255,0,0),   -1, cv::LINE_AA);//青
		  cv::circle(imageCopy, cv::Point(MarkerC[markerIds.at(i)][3][0], MarkerC[markerIds.at(i)][3][1]), 3, Scalar(255,0,255), -1, cv::LINE_AA);//紫

      cv::aruco::drawAxis(imageCopy,cameraMatrix,distCoeffs,rvecs[i], tvecs[i], 0.1);//マーカーの姿勢描写
      cv::Rodrigues(rvecs[i],rvecs2[i],jacobian[i]);//回転ベクトルから回転行列への変換
      depth_ok[markerIds.at(i)]=0;//Depth取得可能数初期化

      //画像→カメラ座標変換(コーナすべて変換する)----------------------------------------------------------------------
      for(int j=0;j<4;j++){
        pixel[markerIds.at(i)][j][0]=MarkerC[markerIds.at(i)][j][0];
        pixel[markerIds.at(i)][j][1]=MarkerC[markerIds.at(i)][j][1];
        std::cout <<"MarkerC[markerIds.at(i)="<<markerIds.at(i)<<"][j="<<j<<"][0]="<<MarkerC[markerIds.at(i)][j][0]<<",MarkerC[markerIds.at(i)="<<markerIds.at(i)<<"][j="<<j<<"][1]="<<MarkerC[markerIds.at(i)][j][1]<< std::endl;
        depth[j] = depthimage.at<float>(cv::Point(pixel[markerIds.at(i)][j][0],pixel[markerIds.at(i)][j][1]));

        //Depthが取得できないコーナーを削除する+Depthの外れ値を除く
        if(depth[j]>0&&depth[j]<10000){
          x = (pixel[markerIds.at(i)][j][0] - camera_info.K[2]) / camera_info.K[0];//ここで正規化座標もしてる
          y = (pixel[markerIds.at(i)][j][1] - camera_info.K[5]) / camera_info.K[4];
          //camera_info.K[0]=615.337,camera_info.K[2]=324.473,camera_info.K[4]=615.458,camera_info.K[5]=241.696//内部パラメータ

          point[markerIds.at(i)][j][0] = depth[j] * x/1000;//メートル表示変換
          point[markerIds.at(i)][j][1] = depth[j] * y/1000;
          point[markerIds.at(i)][j][2] = depth[j]/1000;
          point[markerIds.at(i)][j][3] = 1;//データ取得可能なら1
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

    cv::Mat_<float>Pt0=cv::Mat_<float>(markerIds.size()*8, 1);
    cv::Mat_<float>Ot0_1=cv::Mat_<float>(markerIds.size()*8, 4);
    cv::Mat_<float>xEst0=cv::Mat_<float>(3,markerIds.size()*8);//今観測したマーカー座標(初回外パラ推定用)
    cv::Mat_<float>xEst_prev0=cv::Mat_<float>(3,markerIds.size()*8);//一つ前に観測したマーカー座標（初回外パラ推定用)
    cv::Mat_<float>xEst0_cl[40*4]=cv::Mat_<float>(3, 1);//今観測したマーカー座標(カルマン用)
    cv::Mat_<float>xEst0_prev_cl[40*4]=cv::Mat_<float>(3, 1);//一つ前に観測したマーカー座標（カルマン用)
    cv::Mat_<float>xEst_cl[40*4]=cv::Mat_<float>(3, 1);//今推定したマーカー座標(カルマン用)
    cv::Mat_<float>xEst_prev_cl[40*4]=cv::Mat_<float>(3, 1);//一つ前に推定したマーカー座標（カルマン用)
    cv::Mat_<float>pixel_cl[40*4]=cv::Mat_<float>(2, 1);//追跡可能なマーカーの画像座標（カルマン用)
    cv::Mat_<float>Gt[40*4]=cv::Mat_<float>(3, 6);

    //最小二乗法を用いた外部パラメータの算出(日高手法)-----------------------------------------------------------------------------
    //マーカーが追跡可能かどうかを見る(pointとpoint_prveで同じマーカーがあれば追跡可能)
    if(kaisu!=0){
      for(int i=0;i<ALLMarker;i++){
        for(int j=0;j<4;j++){
          if(point[i][j][3]!=0){//最新要素が無いものを除外
            if(point[i][j][3]==point_prve[i][j][3]){//追跡可能かどうか
            std::cout << "point     ["<<i<<"]["<<j<<"]={"<< point[i][j][0] <<","<<point[i][j][1]<<","<<point[i][j][2]<<","<<point[i][j][3]<<"}"<< std::endl;
            std::cout << "point_prev["<<i<<"]["<<j<<"]={"<< point_prve[i][j][0] <<","<<point_prve[i][j][1]<<","<<point_prve[i][j][2]<<","<<point_prve[i][j][3]<<"}"<< std::endl;
            Pt0(kosuu,0)=point[i][j][0];//マーカーのカメラ座標(観測)
            Pt0(kosuu+1,0)=point[i][j][2];

            Ot0_1(kosuu,0)=point_prve[i][j][0],Ot0_1(kosuu,1)=point_prve[i][j][2],Ot0_1(kosuu,2)=1,Ot0_1(kosuu,3)=0;
            Ot0_1(kosuu+1,0)=point_prve[i][j][2],Ot0_1(kosuu+1,1)=-point_prve[i][j][0],Ot0_1(kosuu+1,2)=0,Ot0_1(kosuu+1,3)=1;

            xEst_prev0(0,kosuu/2)=point_prve[i][j][0];
            xEst_prev0(1,kosuu/2)=0;
            xEst_prev0(2,kosuu/2)=point_prve[i][j][2];

            xEst0(0,kosuu/2)=point[i][j][0];
            xEst0(1,kosuu/2)=0;
            xEst0(2,kosuu/2)=point[i][j][2];

            xEst0_prev_cl[kosuu/2](0,0)=point_prve[i][j][0];//一つ前に観測したマーカーのカメラ座標（カルマン用)
            xEst0_prev_cl[kosuu/2](1,0)=0;
            xEst0_prev_cl[kosuu/2](2,0)=point_prve[i][j][2];

            xEst0_cl[kosuu/2](0,0)=point[i][j][0];//今観測したマーカーのカメラ座標(カルマン用)
            xEst0_cl[kosuu/2](1,0)=0;
            xEst0_cl[kosuu/2](2,0)=point[i][j][2];

            pixel_cl[kosuu/2](0,0)=pixel[i][j][0] - camera_info.K[2];//今観測したマーカーの正規化画像座標
            pixel_cl[kosuu/2](1,0)=pixel[i][j][1] - camera_info.K[5];

            kosuu=kosuu+2;
            }
          }
        }
      }
      //for(int i=0;i<kosuu/2;i++){
      //std::cout << "xEst0_prev_cl["<<i<<"]=\n"<<xEst0_prev_cl[i]<< std::endl;
      //std::cout << "xEst0_cl["<<i<<"]=\n"<<xEst0_cl[i]<< std::endl;
      //}

      cv::Mat_<float>Pt=cv::Mat_<float>(kosuu, 1);//リサイズ用行列定義
      cv::Mat_<float>Ot_1=cv::Mat_<float>(kosuu, 4);
      cv::Mat_<float>xEst = cv::Mat_<float>::zeros(3, kosuu/2);//状態方程式(マーカーのカメラ座標(初回は観測値))
      cv::Mat_<float>xEst_prev = cv::Mat_<float>::zeros(3, kosuu/2);//状態方程式(マーカーのカメラ座標(初回は観測値))

      // Pt0の第0行から第kosuu行をPtの第0行から第kosuu行に代入(リサイズ)
	    Pt0.rowRange(0, kosuu).copyTo(Pt.rowRange(0, kosuu));
	    Ot0_1.rowRange(0, kosuu).copyTo(Ot_1.rowRange(0, kosuu));
	    xEst_prev0.colRange(0, kosuu/2).copyTo(xEst_prev.colRange(0, kosuu/2));
	    xEst0.colRange(0, kosuu/2).copyTo(xEst.colRange(0, kosuu/2));
      std::cout <<"Pt=\n"<<Pt<< std::endl;
      std::cout <<"Ot_1=\n"<<Ot_1<< std::endl;
      std::cout <<"xEst=\n"<<xEst<< std::endl;
      std::cout <<"xEst_prev=\n"<<xEst_prev<< std::endl;

      //すべてのマーカー座標を利用して最小二乗法外部パラメータ推定(初回)
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

      Vt = cv::Mat_<float>(6, 1);//並進ベクトル(外部パラメータから速度を求める)
      Vt(0,0)=Xp(0,2)/realsec;//Vx
      Vt(1,0)=0;//Vy
      Vt(2,0)=Xp(0,3)/realsec;//Vz
      Vt(3,0)=0;
      Vt(4,0)=0;
      Vt(5,0)=0;
      std::cout <<"Vt=\n"<<Vt<< std::endl;//並進ベクトル
      //カルマン初期設定(カルマン初回のみ実行)
      //if(kaisu<2){                //ここ本来はいるけど今テストのために消している
        cv::Mat Wt = cv::Mat_<float>::zeros(6, 6);
        cv::Mat Mt_cl[kosuu] = cv::Mat_<float>::zeros(3, 3);//誤差共分散(予測式)
        cv::Mat Pt_cl[kosuu] = cv::Mat_<float>::zeros(3, 3);//誤差共分散(更新式)
        cv::Mat Yt_cl[kosuu] = cv::Mat_<float>::zeros(2, 1);//観測残差
        cv::Mat Ht_cl[kosuu] = cv::Mat_<float>::zeros(2, 3);//Htヤコビアン行列
        cv::Mat Ut_cl[kosuu] = cv::Mat_<float>::zeros(2, 1);//共分散
        cv::Mat Kt_cl[kosuu] = cv::Mat_<float>::zeros(3, 2);//カルマンゲイン
        cv::Mat u_ = (cv::Mat_<float>(2, 2) <<
            1/12, 0,
            0, 1/12);
        for(int i=0;i<kosuu/2;i++){
          Gt[i](0,0)=-1,Gt[i](0,1)=0,Gt[i](0,2)=0,Gt[i](0,3)=0,Gt[i](0,4)=-xEst0_prev_cl[i](2,0),Gt[i](0,5)=xEst0_prev_cl[i](1,0);
          Gt[i](1,0)=0,Gt[i](1,1)=-1,Gt[i](1,2)=0,Gt[i](1,3)=xEst0_prev_cl[i](2,0),Gt[i](1,4)=0,Gt[i](1,5)=-xEst0_prev_cl[i](0,0);
          Gt[i](2,0)=0,Gt[i](2,1)=0,Gt[i](2,2)=-1,Gt[i](2,3)=-xEst0_prev_cl[i](1,0),Gt[i](2,4)=xEst0_prev_cl[i](0,0),Gt[i](2,5)=0;
          //std::cout <<"Gt["<<i<<"]=\n"<<Gt[i]<< std::endl;
        }
      //}

      //カルマンフィルタでマーカーの座標修正(座標推定)
      //予測ステップ----------------------------------------------------------------------------------------------------
      //状態モデルの予測式
      for(int i=0;i<kosuu/2;i++){
        xEst_cl[i]=Ft*xEst0_prev_cl[i]+Gt[i]*Vt;//カルマン初回動作はこっち
        xEst_cl[i]=Ft*xEst_prev_cl[i]+Gt[i]*Vt;//初回以降はこっち
        //std::cout <<"xEst_cl["<<i<<"]=\n"<<xEst_cl[i]<< std::endl;
      }
      //誤差共分散の予測式-----------------------------------
      float Xp,Yp;
      int Otcount=0;
      cv::Mat delta = cv::Mat_<float>::eye(kosuu, kosuu);// 単位行列
      cv::Mat_<float>Ot=cv::Mat_<float>(kosuu, 6);
      for(int i=0;i<kosuu/2;i++){
        Xp=camera_info.K[0]*xEst_cl[i](0,0)/xEst_cl[i](2,0);
        Yp=camera_info.K[0]*xEst_cl[i](1,0)/xEst_cl[i](2,0);
        Ot(Otcount,0)=camera_info.K[0],Ot(Otcount,1)=0,Ot(Otcount,2)=-Xp,Ot(Otcount,3)=-Xp*xEst_prev0(1,i),Ot(Otcount,4)=Xp*xEst_prev0(0,i)+camera_info.K[0]*xEst_prev0(2,i),Ot(Otcount,5)=-camera_info.K[0]*xEst_prev0(1,i);
        Ot(Otcount+1,0)=0,Ot(Otcount+1,1)=camera_info.K[0],Ot(Otcount+1,2)=-Yp,Ot(Otcount+1,3)=-Yp*xEst_prev0(1,i)-camera_info.K[0]*xEst_prev0(2,i),Ot(Otcount+1,4)=-Yp*xEst_prev0(0,i),Ot(Otcount+1,5)=camera_info.K[0]*xEst_prev0(0,i);
        Otcount=Otcount+2;
      }
      std::cout <<"Ot=\n"<<Ot<< std::endl;

      cv::Mat_<float>Ot2_1=cv::Mat_<float>(kosuu, kosuu);
      Ot2_1=Ot.t()*Ot;
      std::cout <<"Ot2_1=\n"<<Ot2_1<< std::endl;
      cv::Mat_<float>Ot2_1_T=cv::Mat_<float>(kosuu, kosuu);
      Ot2_1_T=Ot2_1.inv();
      Wt=Ot2_1_T*Ot.t()*delta*0.12*Ot*Ot2_1_T.t();
      std::cout <<"Wt=\n"<<Wt<< std::endl;

      for(int i=0;i<kosuu/2;i++){
        Mt_cl[i]=Ft*Pt_cl[i]*Ft.t()+Gt[i]*Wt*Gt[i].t();
        std::cout <<"Mt_cl["<<i<<"]=\n"<<Mt_cl[i]<< std::endl;//誤差共分散の予測式Mt
      }
      //更新ステップ----------------------------------------------------------------------
      //観測残差
      for(int i=0;i<kosuu/2;i++){
        Ht_cl[i]= (cv::Mat_<float>(2, 3) <<
          camera_info.K[0]/xEst0_prev_cl[i](2,0), 0, -camera_info.K[0]*xEst0_prev_cl[i](0,0)/xEst0_prev_cl[i](2,0)*xEst0_prev_cl[i](2,0),
          0, camera_info.K[0]/xEst0_prev_cl[i](2,0), -camera_info.K[0]*xEst0_prev_cl[i](1,0)/xEst0_prev_cl[i](2,0)*xEst0_prev_cl[i](2,0));

        Ut_cl[i]= (cv::Mat_<float>(2, 1) <<
          camera_info.K[0]*xEst0_prev_cl[i](0,0)/xEst0_prev_cl[i](2,0),
          camera_info.K[0]*xEst0_prev_cl[i](1,0)/xEst0_prev_cl[i](2,0));

        std::cout <<"Ht_cl["<<i<<"]=\n"<<Ht_cl[i]<< std::endl;
        std::cout <<"Ut_cl["<<i<<"]=\n"<<Ut_cl[i]<< std::endl;

        Yt_cl[i]=pixel_cl[i]-(Ht_cl[i]*xEst_cl[i]+Ut_cl[i]);
        std::cout <<"Yt_cl["<<i<<"]=\n"<<Yt_cl[i]<< std::endl;//観測残差

        Kt_cl[i]=Pt_cl[i]*Ht_cl[i].t()*u_.inv();
        std::cout <<"Kt_cl["<<i<<"]=\n"<<Kt_cl[i]<< std::endl;//カルマンゲイン

        xEst_cl[i]=xEst_cl[i]+Kt_cl[i]*Yt_cl[i];
        std::cout <<"xEst_cl["<<i<<"]=\n"<<xEst_cl[i]<< std::endl;//状態モデルの更新

        Pt_cl[i]=Mt_cl[i].inv()+Ht_cl[i].t()*u_.inv()*Ht_cl[i];
        Pt_cl[i]=Pt_cl[i].inv();
        std::cout <<"Pt_cl["<<i<<"]=\n"<<Pt_cl[i]<< std::endl;//誤差共分散の更新 

        xEst_prev_cl[i]=xEst_cl[i];//現在の推定を保存
      }
    }
  }
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
    for(int j=0;j<4;j++){
      point_prve[markerIds.at(i)][j][0]=point[markerIds.at(i)][j][0];//今のカメラ座標を保存
      point_prve[markerIds.at(i)][j][1]=point[markerIds.at(i)][j][1];
      point_prve[markerIds.at(i)][j][2]=point[markerIds.at(i)][j][2];
      point_prve[markerIds.at(i)][j][3]=point[markerIds.at(i)][j][3];//データが取得可能かどうか
    }
  }

  cv::swap(image_curr, image_prev);// image_curr を image_prev に移す（交換する）
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






