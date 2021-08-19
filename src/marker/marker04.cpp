///マーカー検出プログラム https://qiita.com/noueezy/items/58730f09f6aa6a97faf5
//マーカー検出時の赤四角が左上になるようにマーカーを配置すると正しくマーカーの中心座標を求められる
//外部パラメータを推定するためには最低でも３点の情報が必要→そこでマーカーのコーナー４点を使用して外部パラメータを算出する
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


std::string win_src = "src";
std::string win_dst = "dst";
std::string win_depth = "win_depth";

std::string source_frame = "map";//mapフレーム
ros::Subscriber marker_sub;



using namespace std;
using namespace cv;
float kaisu = 0;//tf用
int kaisu2 = 0;//tf用

sensor_msgs::CameraInfo camera_info;//CameraInfo受け取り用
static struct_slam::marker_tf marker_tf;//観測マーカーのメッセージ
float MLength_prve[50],MAngle_prve[50];//マーカーまでの距離と角度
float Rotation;//外部パラメータ(仮)
cv::Mat_<float> MarkerCamera[50]=cv::Mat_<float>(1, 4);//マーカーのカメラ座標の行列
cv::Mat_<float> MarkerWorld[50]=cv::Mat_<float>(1, 3);//マーカーの世界座標の行列
cv::Mat_<float>MarkerCameraT=cv::Mat_<float>(4, 4);//マーカーの世界座標の行列
cv::Mat_<float> External_prm=cv::Mat_<float>(4, 3);//外部パラメータ
static cv::Mat_<float> External_prmPLAS=cv::Mat_<float>(4, 3);//外部パラメータ
//static cv::Mat_<float> External_R=cv::Mat_<float>(3, 3);//外部パラメータ(回転行列のみ)
std::array<float, 9> External_R;//外部パラメータ(回転行列のみ)(クォータニオン変換用)
static cv::Mat_<float> External_T=cv::Mat_<float>(1, 3);//外部パラメータ(回転行列のみ)

struct Camera_Base{
    float x;
    float y;
    float z;
};
struct Camera_Base camera_base;



//コールバック関数
void callback(const sensor_msgs::Image::ConstPtr& rgb_msg,const sensor_msgs::Image::ConstPtr& depth_msg, const sensor_msgs::CameraInfo::ConstPtr& cam_info)
{
	//変数宣言
  cv_bridge::CvImagePtr bridgeImage;//クラス::型//cv_brigeは画像変換するとこ
  cv_bridge::CvImagePtr bridgedepthImage;//クラス::型//cv_brigeは画像変換するとこ
  cv::Mat RGBimage,depthimage,image;
  struct_slam::marker_tf marker_tf2=marker_tf;//tfのデータ一時保存(更新を防ぐため)

  std::array<float, 9>  array;
  Quaternion quaternionVal(1,2,3,4);
  RotationMatrix rotationMatrixVal(array);

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
    
  camera_info=*cam_info;//CameraInfo受け取り

  //std::cout << "CameraInfo:/camera/color/camera_info" << std::endl;//rostopic echo /camera/color/camera_info でも見れる
  //std::cout << "camera_info.height=" <<camera_info.height<< std::endl;
  //std::cout << "camera_info.width= " <<camera_info.width<< std::endl;
  //std::cout << "camera_info.distortion_model=" <<camera_info.distortion_model<< std::endl;
  //std::cout << "camera_info.D=" <<camera_info.D[0]<< std::endl;//RealsenseD435は歪みパラメータが0(キャリブレーション調整されてる)
  //std::cout << "camera_info.K=" <<camera_info.K[0]<< std::endl;
  //std::cout << "camera_info.R=" <<camera_info.R<< std::endl;
  //std::cout << "camera_info.P=" <<camera_info.P<< std::endl;
  //std::cout << "camera_info.binning_x=" <<camera_info.binning_x<< std::endl;
  //std::cout << "camera_info.binning_y=" <<camera_info.binning_y<< std::endl;
    

  image = bridgeImage->image.clone();//image変数に変換した画像データを代入
  depthimage = bridgedepthImage->image.clone();//image変数に変換した画像データを代入

  cv::Mat imageCopy = image.clone();
  cv::Mat_<float> intrinsic_K= cv::Mat_<float>(3, 3);
  
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
  cv::Mat_<float> Screenp[50] = cv::Mat_<float>(3, 1);//点Pの画像座標系（u1,v1,1)T※斉次化
  cv::Mat_<float> Camerap[50] = cv::Mat_<float>(3, 1);//点Pのカメラ座標系（xc,yc,zc)T
  float pixel[50][2],depth,point[50][3],x,y,r2,f,ux,uy;
  float MLength[50],MAngle[50];//マーカーまでの距離と角度
  //float pixel[100][2],depth,point[100][3];


  if (markerIds.size() > 0) {
    //マーカー位置を描画
    cv::aruco::drawDetectedMarkers(imageCopy, markerCorners, markerIds);
    //マーカーの姿勢推定
    cv::aruco::estimatePoseSingleMarkers(markerCorners, 0.05, cameraMatrix, distCoeffs, rvecs, tvecs);

    for(int i=0;i<markerIds.size();i++){
      std::cout <<"markerIds("<<i<<")="<< markerIds.at(i) << std::endl;//マーカーID
      std::cout <<"markerCorners["<<i<<"]=\n"<< markerCorners[i] << std::endl;//マーカーのコーナー座標が出てくる
      std::cout <<"markerCorners["<<i<<"]="<< markerCorners[i][0].x << std::endl;//マーカーのコーナー座標がx出てくる
      std::cout <<"markerCorners["<<i<<"]="<< markerCorners[i][0].y << std::endl;//マーカーのコーナー座標がy出てくる
      MarkerC[markerIds.at(i)][0][0]=markerCorners[i][0].x, MarkerC[markerIds.at(i)][0][1]=markerCorners[i][0].y;//コーナーの画像座標ID対応化
      MarkerC[markerIds.at(i)][1][0]=markerCorners[i][1].x, MarkerC[markerIds.at(i)][1][1]=markerCorners[i][1].y;
      MarkerC[markerIds.at(i)][2][0]=markerCorners[i][2].x, MarkerC[markerIds.at(i)][2][1]=markerCorners[i][2].y;
      MarkerC[markerIds.at(i)][3][0]=markerCorners[i][3].x, MarkerC[markerIds.at(i)][3][1]=markerCorners[i][3].y;

      std::cout <<"MarkerC["<<markerIds.at(i)<<"][0][0]="<< MarkerC[markerIds.at(i)][0][0]<< std::endl;//コーナーの画像座標(ID対応)
      std::cout <<"MarkerC["<<markerIds.at(i)<<"][0][1]="<< MarkerC[markerIds.at(i)][0][1]<< std::endl;//コーナーの画像座標(ID対応)
		  cv::circle(imageCopy, cv::Point(MarkerC[markerIds.at(i)][0][0], MarkerC[markerIds.at(i)][0][1]), 3, Scalar(0,255,0),   -1, cv::LINE_AA);//コーナーの画像座標(ID対応)
		  cv::circle(imageCopy, cv::Point(MarkerC[markerIds.at(i)][1][0], MarkerC[markerIds.at(i)][1][1]), 3, Scalar(255,255,0), -1, cv::LINE_AA);
		  cv::circle(imageCopy, cv::Point(MarkerC[markerIds.at(i)][2][0], MarkerC[markerIds.at(i)][2][1]), 3, Scalar(255,0,0),   -1, cv::LINE_AA);
		  cv::circle(imageCopy, cv::Point(MarkerC[markerIds.at(i)][3][0], MarkerC[markerIds.at(i)][3][1]), 3, Scalar(255,0,255), -1, cv::LINE_AA);
      MCx[markerIds.at(i)]=(MarkerC[markerIds.at(i)][0][0]+MarkerC[markerIds.at(i)][1][0])/2;//マーカー中心座標(x座標)
      MCy[markerIds.at(i)]=(MarkerC[markerIds.at(i)][0][1]+MarkerC[markerIds.at(i)][2][1])/2;//マーカー中心座標(y座標)

		  cv::circle(imageCopy, cv::Point(MCx[markerIds.at(i)], MCy[markerIds.at(i)]), 5, Scalar(0,0,255), -1, cv::LINE_AA);//マーカー中心座標
		  cv::circle(depthimage, cv::Point(MCx[markerIds.at(i)], MCy[markerIds.at(i)]), 5, Scalar(0,0,0), -1, cv::LINE_AA);//マーカー中心座標

      cv::aruco::drawAxis(imageCopy,cameraMatrix,distCoeffs,rvecs[i], tvecs[i], 0.1);//マーカーの姿勢描写
      //std::cout <<"rvecs["<<i<<"]="<< rvecs[i] << std::endl;//回転ベクトル
      //std::cout <<"tvecs["<<i<<"]="<< tvecs[i] << std::endl;//並進ベクトル:tvecs=(x,y,z)カメラ座標

      cv::Rodrigues(rvecs[i],rvecs2[i],jacobian[i]);//回転ベクトルから回転行列への変換

      //std::cout <<"回転行列rvecs2["<<i<<"]="<< rvecs2[i] << std::endl;//回転行列

/*    // 座標変換を行う
      tf::Vector3 position_or = transformer * tf::Vector3(tvecs[i][0], tvecs[i][1], tvecs[i][2]);
      tf::Quaternion attitude_or = transformer * tf::createQuaternionFromRPY(rvecs[i][0], rvecs[i][1], rvecs[i][2]);
      tf::Vector3 velocity_or = velocity_transformer * tf::Vector3(tvecs[i][0], tvecs[i][1], tvecs[i][2]);

      double x_or, y_or, z_or; // 変換後のターゲット位置@ロボット座標系
      double roll_or, pitch_or, yaw_or; // 変換後のターゲット姿勢@ロボット座標系
      double Vx_or, Vy_or, Vz_or; // 変換後のターゲット速度@ロボット座標系
      x_or = position_or.x();
      y_or = position_or.y();
      z_or = position_or.z();
      tf::Matrix3x3(attitude_or).getRPY(roll_or, pitch_or, yaw_or);
      Vx_or = velocity_or.x();
      Vy_or = velocity_or.y();
      Vz_or = velocity_or.z();
*/
    //画像→カメラ座標変換----------------------------------------------------------------------
      //float pixel[2]={MCx[markerIds.at(i)],MCy[markerIds.at(i)]};
      //float depth = depthimage.at<float>(MCx[markerIds.at(i)],MCy[markerIds.at(i)]);
       pixel[markerIds.at(i)][0]=MCx[markerIds.at(i)];
       pixel[markerIds.at(i)][1]=MCy[markerIds.at(i)];
       depth = depthimage.at<float>(MCx[markerIds.at(i)],MCy[markerIds.at(i)]);

      x = (pixel[markerIds.at(i)][0] - camera_info.K[2]) / camera_info.K[0];
      y = (pixel[markerIds.at(i)][1] - camera_info.K[5]) / camera_info.K[4];

      r2  = x*x + y*y;
      f = 1 + camera_info.D[0]*r2 + camera_info.D[1]*r2*r2 + camera_info.D[4]*r2*r2*r2;
      ux = x*f + 2*camera_info.D[2]*x*y + camera_info.D[3]*(r2 + 2*x*x);
      uy = y*f + 2*camera_info.D[3]*x*y + camera_info.D[2]*(r2 + 2*y*y);
      x = ux;
      y = uy;

      point[markerIds.at(i)][0] = depth * x/1000;//メートル表示変換
      point[markerIds.at(i)][1] = depth * y/1000;
      point[markerIds.at(i)][2] = depth/1000;
      std::cout << "特徴点のカメラ座標:point["<<markerIds.at(i)<<"]={"<< point[markerIds.at(i)][0] <<","<<point[markerIds.at(i)][1]<<","<<point[markerIds.at(i)][2]<<"}"<< std::endl;
/*
//最小二乗法を用いた外部パラメータの算出-----------------------------------------------------------------------------
      MarkerCamera[markerIds.at(i)](0,0)=point[markerIds.at(i)][0];//マーカーのカメラ座標配列化
      MarkerCamera[markerIds.at(i)](0,1)=point[markerIds.at(i)][1];
      MarkerCamera[markerIds.at(i)](0,2)=point[markerIds.at(i)][2];
      MarkerCamera[markerIds.at(i)](0,3)=1;
      std::cout <<" MarkerCamera["<<markerIds.at(i)<<"]=\n"<< MarkerCamera[markerIds.at(i)] << std::endl;

      if(kaisu2==0){//マーカーの世界座標配列化(初回はカメラ座標＝世界座標)
        MarkerWorld[markerIds.at(i)](0,0)=point[markerIds.at(i)][0];
        MarkerWorld[markerIds.at(i)](0,1)=point[markerIds.at(i)][1];
        MarkerWorld[markerIds.at(i)](0,2)=point[markerIds.at(i)][2];
      }
      MarkerCameraT=MarkerCamera[markerIds.at(i)].t()*MarkerCamera[markerIds.at(i)];
      std::cout <<" MarkerCameraT=\n"<< MarkerCameraT << std::endl;//外部パラメータ
      std::cout <<" MarkerCameraT.inv()=\n"<< MarkerCameraT.inv(cv::DECOMP_SVD) << std::endl;//擬似逆行列
      std::cout <<" MarkerCamera[markerIds.at(i)].t()=\n"<< MarkerCamera[markerIds.at(i)].t() << std::endl;//外部パラメータ
      std::cout <<" MarkerCameraT.inv()*MarkerCamera[markerIds.at(i)].t()=\n"<< MarkerCameraT.inv(cv::DECOMP_SVD)*MarkerCamera[markerIds.at(i)].t() << std::endl;//外部パラメータ


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
      MarkerWorld[markerIds.at(i)](0,0)=MarkerCamera[markerIds.at(i)](0,0);
      MarkerWorld[markerIds.at(i)](0,1)=MarkerCamera[markerIds.at(i)](0,1);
      MarkerWorld[markerIds.at(i)](0,2)=MarkerCamera[markerIds.at(i)](0,2);
      std::cout <<" MarkerWorld["<<markerIds.at(i)<<"]=\n"<< MarkerWorld[markerIds.at(i)] << std::endl;

      kaisu2++;

      //マーカーまでの距離と角度を求める
      MLength[markerIds.at(i)]=point[markerIds.at(i)][2];
      MAngle[markerIds.at(i)]=atan(point[markerIds.at(i)][0]/point[markerIds.at(i)][2]);

      std::cout <<"距離MLemght["<<markerIds.at(i)<<"]="<< MLength[markerIds.at(i)] << std::endl;
      std::cout <<"角度MAngle["<<markerIds.at(i)<<"]="<< MAngle[markerIds.at(i)] << std::endl;

      //マーカーの一つ前の座標と今の座標から最小二乗法を用いて外部パラメータを算出する
      //座標配列化
      //初回


  //tf(観測マーカー)-------------------------------------------------------------------------------------------------観測マーカー
      std::string target_maker_frame = "marker_link";//cameraとマーカー間のリンク
      geometry_msgs::Pose maker_pose;

      maker_pose.position.x = point[markerIds.at(i)][2];//Rvizと画像は座標系が異なるので注意
      maker_pose.position.y = -point[markerIds.at(i)][0];
      maker_pose.position.z = -point[markerIds.at(i)][1];
      maker_pose.orientation.w = 1.0;

      static tf::TransformBroadcaster br_maker;
      tf::Transform maker_transform;
      poseMsgToTF(maker_pose, maker_transform);
      br_maker.sendTransform(tf::StampedTransform(maker_transform, ros::Time::now(), target_maker_frame, "/camera_link"));
      //br_maker.sendTransform(tf::StampedTransform(maker_transform, ros::Time::now(), "/camera_link", target_maker_frame));
      //br_maker.sendTransform(tf::StampedTransform(maker_transform, ros::Time::now(), "/robot1/camera_link", target_maker_frame));


      MLength_prve[markerIds.at(i)]=MLength[markerIds.at(i)];//一つ前の距離に保存
      MAngle_prve[markerIds.at(i)]=MAngle[markerIds.at(i)];//一つ前の角度に保存
      */
    }
  }
  //tf(map camera_base間のlink)-----------------------------------------------------------------------------------------
   geometry_msgs::Pose camera_base_pose;

    std::string MaptoCamera_Base_frame = "MaptoCamera_Base_link";
    camera_base_pose.position.x = camera_base.x+External_prm(3,0);//ここにカメラオドメトリーが入ると思う
    camera_base_pose.position.y = camera_base.y+External_prm(3,1);
    camera_base_pose.position.z = camera_base.z+External_prm(3,2);
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
    //Camera_BasetoCamera_Link_pose.position.z = 0.9;//ここに実際のカメラの高さを入れる
    Camera_BasetoCamera_Link_pose.orientation.x = quaternionVal.x;
    Camera_BasetoCamera_Link_pose.orientation.y = quaternionVal.y;
    Camera_BasetoCamera_Link_pose.orientation.z = quaternionVal.z;
    Camera_BasetoCamera_Link_pose.orientation.w = quaternionVal.w;
    static tf::TransformBroadcaster br_Camera_BasetoCamera_Link_pose;

    tf::Transform Camera_BasetoCamera_transform;
    poseMsgToTF(Camera_BasetoCamera_Link_pose, Camera_BasetoCamera_transform);
    br_Camera_BasetoCamera_Link_pose.sendTransform(tf::StampedTransform(Camera_BasetoCamera_transform, ros::Time::now(), MaptoCamera_Base_frame, "camera_link"));
 
  

    kaisu++;

    std::cout << "観測マーカーと地図上のマーカーとの差を計算(座標)={"<< 
    marker_tf2.Tfodometry.pose.pose.position.x <<","<<marker_tf2.Tfodometry.pose.pose.position.y<<","<<marker_tf2.Tfodometry.pose.pose.position.z<<"}"<< std::endl;

  
    // 画面表示
    cv::imshow(win_src, image);
    cv::imshow(win_dst, imageCopy);
    cv::imshow(win_depth, depthimage);

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


