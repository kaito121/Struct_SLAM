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

#include<fstream>

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
float pixel[50][2],depth[100],point[50][4],x,y,r2,f,ux,uy;//画像→カメラ座標変換

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
ofstream outputfile1("/home/fuji/catkin_ws/src/Struct_SLAM/src/marker/hidaka/MCx[6]_6.0m.txt");
ofstream outputfile2("/home/fuji/catkin_ws/src/Struct_SLAM/src/marker/hidaka/MCy[6]_6.0m.txt");
ofstream outputfile3("/home/fuji/catkin_ws/src/Struct_SLAM/src/marker/hidaka/MCz[6]_6.0m.txt");
ofstream outputfile4("/home/fuji/catkin_ws/src/Struct_SLAM/src/marker/hidaka/MPx[6]_6.0m.txt");
ofstream outputfile5("/home/fuji/catkin_ws/src/Struct_SLAM/src/marker/hidaka/MPy[6]_6.0m.txt");
ofstream outputfile6("/home/fuji/catkin_ws/src/Struct_SLAM/src/marker/hidaka/MPz[6]_6.0m.txt");

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

  image.copyTo(img_dst);//
  image.copyTo(img_dst1);//特徴点検出結果表示用

  cv::Mat imageCopy = image.clone();
  cv::Mat_<float> intrinsic_K= cv::Mat_<float>(3, 3);
	std::cout <<"画像取り込み"<< std::endl;


//---------------------------------------------------------------------------------------------------------------------------

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
  float MarkerC[50][4][2];//マーカーのコーナー座標を記録する([マーカーID][コーナー番号][XorY])
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
      std::cout <<"MCx[markerIds.at(i)]="<<MCx[markerIds.at(i)]<<",MCy[markerIds.at(i)]="<<MCy[markerIds.at(i)]<< std::endl;
      depth[markerIds.at(i)] = depthimage.at<float>(cv::Point(pixel[markerIds.at(i)][0],pixel[markerIds.at(i)][1]));

      //Depthが取得できないコーナーを削除する+Depthの外れ値を除く
      if(depth[markerIds.at(i)]>0&&depth[markerIds.at(i)]<10000){
        x = (pixel[markerIds.at(i)][0] - camera_info.K[2]) / camera_info.K[0];//ここで正規化座標もしてる
        y = (pixel[markerIds.at(i)][1] - camera_info.K[5]) / camera_info.K[4];
        //camera_info.K[0]=615.337,camera_info.K[2]=324.473,camera_info.K[4]=615.458,camera_info.K[5]=241.696//内部パラメータ

        point[markerIds.at(i)][0] = depth[markerIds.at(i)] * x/1000;//メートル表示変換
        point[markerIds.at(i)][1] = depth[markerIds.at(i)] * y/1000;
        point[markerIds.at(i)][2] = depth[markerIds.at(i)]/1000;
        point[markerIds.at(i)][3] = 1;//データ取得可能なら1
        std::cout << "特徴点のカメラ座標:point["<<markerIds.at(i)<<"]={"<< point[markerIds.at(i)][0] <<","<<point[markerIds.at(i)][1]<<","<<point[markerIds.at(i)][2]<<"}"<< std::endl;
        //std::cout <<"X="<< point[markerIds.at(i)][0]/point[markerIds.at(i)][2]<< std::endl;
        //std::cout <<"Y="<< point[markerIds.at(i)][1]/point[markerIds.at(i)][2]<< std::endl;
      }
    }

      outputfile1<<MCx[6]<<"\n";
      outputfile2<<MCy[6]<<"\n";
      outputfile3<<depthimage.at<float>(cv::Point(MCx[6],MCy[6]))<<"\n";
      outputfile4<<point[6][0]<<"\n";
      outputfile5<<point[6][1]<<"\n";
      outputfile6<<point[6][2]<<"\n";
      std::cout <<"MCx[6]["<<kaisu<<"]="<<MCx[6]<< std::endl;//マーカーの画像座標(ID対応)
      std::cout <<"MCy[6]["<<kaisu<<"]="<<MCy[6]<< std::endl;//マーカーの画像座標(ID対応)
      std::cout <<"MCz[6]["<<kaisu<<"]="<<depthimage.at<float>(cv::Point(MCx[6],MCy[6]))<< std::endl;
      std::cout <<"MPx[6]["<<kaisu<<"]="<<point[6][0]<< std::endl;//コーナーのカメラ座標(ID対応)
      std::cout <<"MPy[6]["<<kaisu<<"]="<<point[6][1]<< std::endl;//コーナーのカメラ座標(ID対応)
      std::cout <<"MPz[6]["<<kaisu<<"]="<<point[6][2]<< std::endl;//コーナーのカメラ座標(ID対応)


    //outputfile.close();

    cv::Mat_<float> MarkerCameraALL=cv::Mat_<float>(ALL_depth_ok, 4);//マーカーのカメラ座標の行列
    cv::Mat_<float> MarkerWorldALL=cv::Mat_<float>(ALL_depth_ok, 3);//マーカーの世界座標の行列

    cv::Mat_<float>Pt=cv::Mat_<float>(markerIds.size()*8, 1);
    cv::Mat_<float>Ot_1=cv::Mat_<float>(markerIds.size()*8, 4);
      
    }
  
    // 画面表示
    cv::imshow(win_src, image);
    cv::imshow(win_dst, imageCopy);
    cv::imshow(win_depth, depthimage);

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

	ros::NodeHandle nhSubTf;//ノードハンドル
	//message_filters::Subscriber<struct_slam::marker_tf> marker_sub(nhSubTf, "marker_tf", 1);


  
	ros::spin();//トピック更新待機
			
	return 0;
}






