//マーカー検出プログラム https://qiita.com/noueezy/items/58730f09f6aa6a97faf5
//マーカー検出時の赤四角が左上になるようにマーカーを配置すると正しくマーカーの中心座標を求められる
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

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <opencv2/aruco/charuco.hpp>//マーカー検出

#include <geometry_msgs/PoseStamped.h>//tf
#include <tf/transform_broadcaster.h>//tf

#include "struct_slam/rs.h"

std::string win_src = "src";
std::string win_dst = "dst";
std::string win_depth = "win_depth";

std::string source_frame = "map";//mapフレーム

using namespace std;
using namespace cv;
float kaisu = 0;//tf用
int kaisu2 = 0;//tf用

sensor_msgs::CameraInfo camera_info;//CameraInfo受け取り用

//コールバック関数
void callback(const sensor_msgs::Image::ConstPtr& rgb_msg,const sensor_msgs::Image::ConstPtr& depth_msg, const sensor_msgs::CameraInfo::ConstPtr& cam_info)
{
	//変数宣言
  cv_bridge::CvImagePtr bridgeImage;//クラス::型//cv_brigeは画像変換するとこ
  cv_bridge::CvImagePtr bridgedepthImage;//クラス::型//cv_brigeは画像変換するとこ
  cv::Mat RGBimage,depthimage,image;
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

  std::cout << "CameraInfo:/camera/color/camera_info" << std::endl;//rostopic echo /camera/color/camera_info でも見れる
  std::cout << "camera_info.height=" <<camera_info.height<< std::endl;
  std::cout << "camera_info.width= " <<camera_info.width<< std::endl;
  std::cout << "camera_info.distortion_model=" <<camera_info.distortion_model<< std::endl;
  std::cout << "camera_info.D=" <<camera_info.D[0]<< std::endl;//RealsenseD435は歪みパラメータが0(キャリブレーション調整されてる)
  std::cout << "camera_info.K=" <<camera_info.K[0]<< std::endl;
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
  int MarkerC[100][4][2];//マーカーのコーナー座標を記録する([マーカーID][コーナー番号][XorY])
  float MCx[100],MCy[100];//マーカーの中心座標
  cv::Mat_<float> Screenp[100] = cv::Mat_<float>(3, 1);//点Pの画像座標系（u1,v1,1)T※斉次化
  cv::Mat_<float> Camerap[100] = cv::Mat_<float>(3, 1);//点Pのカメラ座標系（xc,yc,zc)T


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
      std::cout <<"rvecs["<<i<<"]="<< rvecs[i] << std::endl;//回転ベクトル
      std::cout <<"tvecs["<<i<<"]="<< tvecs[0][0] << std::endl;//並進ベクトル:tvecs=(x,y,z)カメラ座標

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

    }
    //画像→カメラ座標変換----------------------------------------------------------------------
    //float pixel[2]={MCx[markerIds.at(i)],MCy[markerIds.at(i)]};
    //float depth = depthimage.at<float>(MCx[markerIds.at(i)],MCy[markerIds.at(i)]);
    float pixel[2]={MCx[3],MCy[3]};
    float depth = depthimage.at<float>(MCx[3],MCy[3]);
    float point[3];

    float x = (pixel[0] - camera_info.K[2]) / camera_info.K[0];
    float y = (pixel[1] - camera_info.K[5]) / camera_info.K[4];
    
    float r2  = x*x + y*y;
    float f = 1 + camera_info.D[0]*r2 + camera_info.D[1]*r2*r2 + camera_info.D[4]*r2*r2*r2;
    float ux = x*f + 2*camera_info.D[2]*x*y + camera_info.D[3]*(r2 + 2*x*x);
    float uy = y*f + 2*camera_info.D[3]*x*y + camera_info.D[2]*(r2 + 2*y*y);
    x = ux;
    y = uy;
    
    point[0] = depth * x/1000;//メートル表示変換
    point[1] = depth * y/1000;
    point[2] = depth/1000;

    std::cout << "特徴点のカメラ座標:point={"<< point[0] <<","<<point[1]<<","<<point[2]<<"}"<< std::endl;

    //// 座標変換を行う
    //tf::Vector3 position_or = transformer * tf::Vector3(point[0], point[1], point[2]);
    //tf::Quaternion attitude_or = transformer * tf::createQuaternionFromRPY(roll_os, pitch_os, yaw_os);
    //tf::Vector3 velocity_or = velocity_transformer * tf::Vector3(point[0], point[1], point[2]);


    //tf-------------------------------------------------------------------------------------------------
    std::string target_maker_frame = "maker_link";//cameraとマーカー間のリンク
    geometry_msgs::Pose maker_pose;

    maker_pose.position.x = point[2];//Rvizと画像は座標系が異なるので注意
    maker_pose.position.y = -point[0];
    maker_pose.position.z = -point[1];
    maker_pose.orientation.w = 1.0;

    static tf::TransformBroadcaster br_maker;
    tf::Transform maker_transform;
    poseMsgToTF(maker_pose, maker_transform);
    br_maker.sendTransform(tf::StampedTransform(maker_transform, ros::Time::now(), "/camera_link", target_maker_frame));
    //br_maker.sendTransform(tf::StampedTransform(maker_transform, ros::Time::now(), "/robot1/camera_link", target_maker_frame));

  }
  //tf-------------------------------------------------------------------------------------------------------
  
/*    float xyz[3] = { 0, 0, 0 };
    xyz[0] = cos(kaisu / 5.0);
    xyz[1] = sin(kaisu / 5.0);
    
    std::string target_frame = "body_link";
    geometry_msgs::Pose t_pose;
    t_pose.position.x = xyz[0];
    t_pose.position.y = xyz[1];
    t_pose.position.z = 1;
    t_pose.orientation.w = 0.1;

    static tf::TransformBroadcaster br;
    tf::Transform transform;
    
    poseMsgToTF(t_pose, transform);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/camera_link", target_frame));
*/
  //tf(map camera間のlink)-----------------------------------------------------------------------------------------
    geometry_msgs::Pose map_pose;

    map_pose.position.x = 0;//ここにカメラオドメトリーが入ると思う
    map_pose.position.y = 0;
    map_pose.position.z = 0;
    map_pose.orientation.w = 0.1;
    static tf::TransformBroadcaster br_map;

    tf::Transform map_transform;
    poseMsgToTF(map_pose, map_transform);

    br_map.sendTransform(tf::StampedTransform(map_transform, ros::Time::now(), source_frame, "/camera_link"));

/*
    if(kaisu>=100&&kaisu<=300){
      std::string target_frame2 = "body_link2";
      geometry_msgs::Pose t_pose2;

      t_pose2.position.x = xyz[0];
      t_pose2.position.y = xyz[1];
      t_pose2.position.z = -1;
      t_pose2.orientation.w = 1.0;

      static tf::TransformBroadcaster br2;
      tf::Transform transform2;
      poseMsgToTF(t_pose2, transform2);
      br2.sendTransform(tf::StampedTransform(transform2, ros::Time::now(), "/robot1/camera_link", target_frame2));
    }
*/
    


    //br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), source_frame, target_frame));

    kaisu++;
    kaisu2++;
  
    // 画面表示
    cv::imshow(win_src, image);
    cv::imshow(win_dst, imageCopy);
    cv::imshow(win_depth, depthimage);


  cv::waitKey(1);//ros::spinにジャンプする
}
/*

//歪みや逆歪み係数のない画像のピクセル座標と深度が与えられた場合、同じカメラに対する3次元空間の対応点を計算する
static void rs_deproject_pixel_to_point(float point[3], const struct rs_intrinsics * intrin, const float pixel[2], float depth)
{
    assert(intrin->model != RS_DISTORTION_MODIFIED_BROWN_CONRADY); // Cannot deproject from a forward-distorted image//前方に歪んだ画像からのデプロジェクションができない
    assert(intrin->model != RS_DISTORTION_FTHETA); // Cannot deproject to an ftheta image

    float x = (pixel[0] - intrin->ppx) / intrin->fx;
    float y = (pixel[1] - intrin->ppy) / intrin->fy;
    if(intrin->model == RS_DISTORTION_INVERSE_BROWN_CONRADY)
    {
        float r2  = x*x + y*y;
        float f = 1 + intrin->coeffs[0]*r2 + intrin->coeffs[1]*r2*r2 + intrin->coeffs[4]*r2*r2*r2;
        float ux = x*f + 2*intrin->coeffs[2]*x*y + intrin->coeffs[3]*(r2 + 2*x*x);
        float uy = y*f + 2*intrin->coeffs[3]*x*y + intrin->coeffs[2]*(r2 + 2*y*y);
        x = ux;
        y = uy;
    }
    point[0] = depth * x;
    point[1] = depth * y;
    point[2] = depth;
}
*/


//メイン関数
int main(int argc,char **argv){

	ros::init(argc,argv,"marker1");//rosを初期化
	ros::NodeHandle nhSub;//ノードハンドル
	
	//subscriber関連
  //Realsensesの時(roslaunch realsense2_camera rs_camera.launch align_depth:=true)(Depth修正版なのでこっちを使うこと)
	message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nhSub, "/camera/color/image_raw", 1);//センサーメッセージを使うときは対応したヘッダーが必要
	message_filters::Subscriber<sensor_msgs::Image> depth_sub(nhSub, "/camera/aligned_depth_to_color/image_raw", 1);
	message_filters::Subscriber<sensor_msgs::CameraInfo> info_sub(nhSub, "/camera/color/camera_info", 1);

  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,sensor_msgs::Image,sensor_msgs::CameraInfo> MySyncPolicy;	
	message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10),rgb_sub, depth_sub, info_sub);
	sync.registerCallback(boost::bind(&callback,_1, _2, _3));



  //typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,sensor_msgs::Image> MySyncPolicy;	
	//message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10),rgb_sub, depth_sub);
	//sync.registerCallback(boost::bind(&callback,_1, _2));

	ros::spin();//トピック更新待機
			
	return 0;
}

