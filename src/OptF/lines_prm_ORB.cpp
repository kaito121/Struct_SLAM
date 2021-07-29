//内部パラメータ、外部パラメータの検出とカメラ座標から世界座標系への座標変換
//FLD化済み
//Depth調整バージョン(D435_test.cppを参照)
//rosのヘッダ
#include <ros/ros.h>
#include <sensor_msgs/Image.h>//センサーデータ形式ヘッダ
#include <cv_bridge/cv_bridge.h>//画像変換のヘッダ
#include <sensor_msgs/image_encodings.h>//エンコードのためのヘッダ
#include <sensor_msgs/PointCloud.h>
#include <opencv2/opencv.hpp>
#include <opencv/highgui.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl_ros/point_cloud.h>
#include <dynamic_reconfigure/server.h>//reconfig用
#include <mutex>
//#include <OpenCV1/wakuhairetu.h>//自作メッセージ用ヘッダ
#include <algorithm>//並び替え用
#include <math.h>
#include <stdlib.h>//絶対値用関数
#include <highgui.h>
#include <visualization_msgs/Marker.h>//ラインマーカー用
#include <cmath>
#include <struct_slam/MaskImageData.h>//パッケージ名要変更（自分で作ったデータを取り込んで）
#include <struct_slam/ImageMatchingData.h>
#include <opencv2/ximgproc/fast_line_detector.hpp>//FLD
#include <Eigen/Dense>//Eigen用
#include <Eigen/Core>//cvMat->Eigen変換用
#include <opencv2/core/core.hpp>//cvMat->Eigen変換用
#include <opencv2/core/eigen.hpp>//cvMat->Eigen変換用



ros::Subscriber sub;//データをsubcribeする奴
ros::Publisher marker_pub;
ros::Publisher marker_pub_W;
// ros::Publisher image_pub;
std::string win_src = "src";
std::string win_edge = "edge";
std::string win_dst = "dst";
std::string win_dst2 = "dst2";
std::string win_depth = "depth";
std::string win_line = "line";
std::string win_dstm1 = "dstm1";
std::string win_dstm2 = "dstm2";


using namespace std;
using namespace cv;
using namespace Eigen;
using Eigen::MatrixXd;

// 抽出する画像の輝度値の範囲を指定
#define B_MAX 255
#define B_MIN 150
#define G_MAX 255
#define G_MIN 180
#define R_MAX 255
#define R_MIN 180
int best = 30;
int kaisu= 0;
int sec=0;
cv::Mat RGBimage3;//ここで定義する必要がある

void callback(const sensor_msgs::Image::ConstPtr& rgb_msg,const sensor_msgs::Image::ConstPtr& depth_msg)
{
	//変数宣言
	  cv_bridge::CvImagePtr bridgeRGBImage;//クラス::型//cv_brigeは画像変換するとこ
    cv_bridge::CvImagePtr bridgedepthImage;//クラス::型//cv_brigeは画像変換するとこ
    cv::Mat RGBimage;//opencvの画像
    cv::Mat depthimage;//opencvの画像
    

	//ROS_INFO("callback_functionが呼ばれたよ");
	
    try{//MAT形式変換
       bridgeRGBImage=cv_bridge::toCvCopy(rgb_msg, sensor_msgs::image_encodings::BGR8);//MAT形式に変える
       ROS_INFO("callBack");}//printと秒数表示

	//エラー処理
    catch(cv_bridge::Exception& e) {//エラー処理(失敗)成功ならスキップ
        std::cout<<"RGB_image_callback Error \n";
        ROS_ERROR("Could not convert from '%s' to 'BGR8'.",
        rgb_msg->encoding.c_str());
        return ;}

    try{//MAT形式変換
       bridgedepthImage=cv_bridge::toCvCopy(depth_msg, sensor_msgs::image_encodings::TYPE_32FC1);//MAT形式に変える
       ROS_INFO("callBack");}//printと秒数表示
    
	//エラー処理
    catch(cv_bridge::Exception& e) {//エラー処理(失敗)成功ならスキップ
        std::cout<<"depth_image_callback Error \n";
        ROS_ERROR("Could not convert from '%s' to '32FC1'.",
        depth_msg->encoding.c_str());
        return ;}
        
    std::cout << "kaisu=" << kaisu << std::endl;
    RGBimage = bridgeRGBImage->image.clone();//image変数に変換した画像データを代入
    depthimage = bridgedepthImage->image.clone();//image変数に変換した画像データを代入


//画像マッチング（パラメータ推定）
  cv::Mat img_prm[2], img_prmw[2], img_match, img_per, img_reg,img_dstm1,img_dstm2;
  cv::Scalar color[2] = { cv::Scalar(0, 0, 255), cv::Scalar(255, 0, 0) };

   // 画像読み込み(初回はこっち)
    if(kaisu==0){
	img_prm[0] = RGBimage; // 画像読み込み
	img_prm[1] = RGBimage;}// 画像読み込み
	//二回目以降
	else{
	img_prm[0] = RGBimage; // 画像読み込み
	img_prm[1] = RGBimage3;} // 画像読み込み
  
  img_prm[0].copyTo(img_dstm1); // 縦横倍のMatの中央にコピー
  img_prm[1].copyTo(img_dstm2); // 縦横倍のMatの中央にコピー

  //cv::Mat roi1,roi2;


	cv::rectangle(img_prm[0], cv::Point(0, 0), cv::Point(img_prm[0].cols, img_prm[0].rows), color[0], 2); // 外枠
  cv::rectangle(img_prm[1], cv::Point(0, 0), cv::Point(img_prm[1].cols, img_prm[1].rows), color[1], 2); // 外枠

	img_prmw[0] = cv::Mat::zeros(img_prm[0].size() * 2, img_prm[0].type());
  img_prmw[1] = cv::Mat::zeros(img_prm[1].size() * 2, img_prm[1].type());
	cv::Mat roi1 = img_prmw[0](cv::Rect(img_prmw[0].cols / 4, img_prmw[0].rows / 4, img_prm[0].cols, img_prm[0].rows));
  cv::Mat roi2 = img_prmw[1](cv::Rect(img_prmw[1].cols / 4, img_prmw[1].rows / 4, img_prm[1].cols, img_prm[1].rows));
  cv::Mat roi3 = img_prmw[0](cv::Rect(img_prmw[0].cols / 4, img_prmw[0].rows / 4, img_prm[0].cols, img_prm[0].rows));
  cv::Mat roi4 = img_prmw[1](cv::Rect(img_prmw[1].cols / 4, img_prmw[1].rows / 4, img_prm[1].cols, img_prm[1].rows));
	img_prm[0].copyTo(roi1); // 縦横倍のMatの中央にコピー
  img_prm[1].copyTo(roi2); // 縦横倍のMatの中央にコピー
  //img_prm[0].copyTo(img_dstm1); // 縦横倍のMatの中央にコピー
  //img_prm[1].copyTo(img_dstm2); // 縦横倍のMatの中央にコピー

    //matchs:ORBマッチングの結果表示
    //img_dstm1:ORBマッチングの結果表示
    //img_dstm2:ORBマッチングの結果表示
    //img_prmw[0]:roi1の画像を縮小し黒い枠をつけた画像
    //img_prmw[1]:roi2の画像を縮小し黒い枠をつけた画像
    //img_prm[0]:src画像に枠をつけた画像
    //img_prm[1]:src画像に枠をつけた画像
    //roi1:img_prmにマッチング結果を表示させたやつ
    //roi2:img_prmにマッチング結果を表示させたやつ
	
	//cv::imshow("img_prm[0]", img_prmw[0]);
	//cv::imshow("img_prm[1]", img_prmw[1]);

  // (1) 特徴点抽出(ORB)
	cv::Ptr<cv::ORB> detector = cv::ORB::create();// FeatureDetectorオブジェクトの生成
	cv::Ptr<cv::ORB> extractor = cv::ORB::create();// DescriptionExtractorオブジェクトの生成
	cv::BFMatcher matcher(cv::NORM_HAMMING); // DescriptorMatcherオブジェクトの生成
  std::vector<cv::KeyPoint> kpts1, kpts2;// 特徴点情報を格納するための変数(KeyPoint)
  
  detector->detect(img_prmw[0], kpts1);// 特徴点抽出の実行
  detector->detect(img_prmw[1], kpts2);

  cv::Mat desc1, desc2;// 画像の特徴情報を格納するための変数

  extractor->compute(img_prmw[0], kpts1, desc1);// 特徴記述の計算を実行
  extractor->compute(img_prmw[1], kpts2, desc2);

  std::vector<cv::DMatch> matches;// 特徴点のマッチング情報を格納する変数
  matcher.match(desc1, desc2, matches);// 特徴点マッチングの実行
	ROS_INFO("maxtutinngu");//printと秒数表示

  std::cout << "best = " << best << std::endl;
	std::cout << "match size = " << matches.size() << std::endl;
	if (matches.size() < best) {
		std::cout << "few matchpoints" << std::endl;
	}
	
	// 上位best個を採用
	std::nth_element(begin(matches), begin(matches) + best - 1, end(matches));
	matches.erase(begin(matches) + best, end(matches));
	std::cout << "matchs size = " << matches.size() << std::endl;
	
	// 特徴点の対応を表示(img_src1,KeyPoint1,img_src2,KeyPoint2,match,img_dst)
	cv::drawMatches(img_prmw[0], kpts1, img_prmw[1], kpts2, matches, img_match);
	//cv::imshow("matchs", img_match);//ORBマッチングの結果表示
	
	// 特徴点をvectorにまとめる
	std::vector<cv::Point2f> points_src, points_dst;
	for (int i = 0; i < matches.size(); i++) {
		points_src.push_back(kpts1[matches[i].queryIdx].pt);
		points_dst.push_back(kpts2[matches[i].trainIdx].pt);

    //マッチング対応点の座標
    std::cout <<"KeyPoint1["<<i<<"].pt=("<<kpts1[matches[i].queryIdx].pt.x-img_prmw[0].cols / 4<<","<<kpts1[matches[i].queryIdx].pt.y-img_prmw[0].rows / 4<<")"<<std::endl;//対応特徴点の座標1(x,y)
    std::cout <<"KeyPoint2["<<i<<"].pt=("<<kpts2[matches[i].trainIdx].pt.x-img_prmw[0].cols / 4<<","<<kpts2[matches[i].trainIdx].pt.y-img_prmw[0].rows / 4<<")"<<std::endl;//対応特徴点の座標1(x,y)

  
    cv::ellipse(roi1, cv::Point(kpts1[matches[i].queryIdx].pt.x-img_prmw[0].cols / 4,kpts1[matches[i].queryIdx].pt.y-img_prmw[0].rows / 4),cv::Size(5, 5), 0.0, 0.0, 360.0, cv::Scalar(0, 250, 250), 3);//予測の円（黄色）
    cv::ellipse(roi2, cv::Point(kpts2[matches[i].trainIdx].pt.x-img_prmw[1].cols / 4,kpts2[matches[i].trainIdx].pt.y-img_prmw[1].rows / 4),cv::Size(5, 5), 0.0, 0.0, 360.0, cv::Scalar(0, 250, 250), 3);//予測の円（黄色

    if(i==1){
    cv::ellipse(img_dstm1, cv::Point(kpts1[matches[i].queryIdx].pt.x-img_prmw[0].cols / 4,kpts1[matches[i].queryIdx].pt.y-img_prmw[0].rows / 4),cv::Size(3, 3), 0.0, 0.0, 360.0, cv::Scalar(250, 250, 0), 2);//予測の円（黄色）
    cv::ellipse(img_dstm2, cv::Point(kpts2[matches[i].trainIdx].pt.x-img_prmw[1].cols / 4,kpts2[matches[i].trainIdx].pt.y-img_prmw[1].rows / 4),cv::Size(3, 3), 0.0, 0.0, 360.0, cv::Scalar(250, 250, 0), 2);//予測の円（黄色)
    cv::ellipse(img_dstm1, cv::Point(kpts1[matches[i].queryIdx].pt.x-img_prmw[0].cols / 4,kpts1[matches[i].queryIdx].pt.y-img_prmw[0].rows / 4),cv::Size(15, 15), 0.0, 0.0, 360.0, cv::Scalar(250, 250, 0), 2);//予測の円（黄色）
    cv::ellipse(img_dstm2, cv::Point(kpts2[matches[i].trainIdx].pt.x-img_prmw[1].cols / 4,kpts2[matches[i].trainIdx].pt.y-img_prmw[1].rows / 4),cv::Size(15, 15), 0.0, 0.0, 360.0, cv::Scalar(250, 250, 0), 2);//予測の円（黄色)
    }
    else{
    cv::ellipse(img_dstm1, cv::Point(kpts1[matches[i].queryIdx].pt.x-img_prmw[0].cols / 4,kpts1[matches[i].queryIdx].pt.y-img_prmw[0].rows / 4),cv::Size(3, 3), 0.0, 0.0, 360.0, cv::Scalar(0, 250, 250), 2);//予測の円（黄色）
    cv::ellipse(img_dstm2, cv::Point(kpts2[matches[i].trainIdx].pt.x-img_prmw[1].cols / 4,kpts2[matches[i].trainIdx].pt.y-img_prmw[1].rows / 4),cv::Size(3, 3), 0.0, 0.0, 360.0, cv::Scalar(0, 250, 250), 2);//予測の円（黄色
    cv::ellipse(img_dstm1, cv::Point(kpts1[matches[i].queryIdx].pt.x-img_prmw[0].cols / 4,kpts1[matches[i].queryIdx].pt.y-img_prmw[0].rows / 4),cv::Size(15, 15), 0.0, 0.0, 360.0, cv::Scalar(0, 250, 250), 2);//予測の円（黄色）
    cv::ellipse(img_dstm2, cv::Point(kpts2[matches[i].trainIdx].pt.x-img_prmw[1].cols / 4,kpts2[matches[i].trainIdx].pt.y-img_prmw[1].rows / 4),cv::Size(15, 15), 0.0, 0.0, 360.0, cv::Scalar(0, 250, 250), 2);//予測の円（黄色
    }

    cv::line(img_match,cv::Point(0,100), cv::Point(100,100), cv::Scalar(0,255,255), 4, cv::LINE_AA);
    cv::imshow("matchs", img_match);//ORBマッチングの結果表示
    cv::imshow("dstm1", img_dstm1);//ORBマッチングの結果表示
    cv::imshow("dstm2", img_dstm2);//ORBマッチングの結果表示
	}
	
	// (3) マッチング結果から，F行列を推定する
	cv::Mat F = cv::findFundamentalMat(points_src, points_dst);
	std::cout << "F=" << F << std::endl;

  // (4) カメラの内部パラメータが既知の場合はE行列を計算し，外部パラメータを推定する
	// カメラ内部パラメータ読み込み
	cv::Mat V;
	cv::FileStorage fs;
  fs.open("realsense_para.xml", cv::FileStorage::READ);
	fs["intrinsic"]>>V;
	std::cout << "V=" << V << std::endl;
	
	// E行列の計算
	cv::Mat E = cv::findEssentialMat(points_src, points_dst, V);
	
	// 外部パラメータ（回転，並進ベクトル）の計算
	cv::Mat R, t;
	cv::recoverPose(E, points_src, points_dst, V, R, t);

	std::cout << "E=" << E << std::endl;
	std::cout << "R=" << R << std::endl;
	std::cout << "t=" << t << std::endl;

	//行列の要素抜き出し（数値)
  std::cout << "R(0,0)=" << R.at<double>(0, 0) << std::endl << std::endl;

   //配列の列の追加方法(斉次化してないバージョン)
  //cv::Mat_<double> Rt_ = cv::Mat_<double>(3, 4);        
  //Rt_ << R.at<double>(0, 0), R.at<double>(0, 1), R.at<double>(0, 2), t.at<double>(0, 0), R.at<double>(1, 0), R.at<double>(1, 1), R.at<double>(1, 2),t.at<double>(1, 0),R.at<double>(2, 0),R.at<double>(2, 1),R.at<double>(2, 2),t.at<double>(2, 0);
  //cv::Mat Rt = Rt_;
  //std::cout << "Rt=" << Rt << std::endl << std::endl;

  //配列の列の追加方法(斉次化バージョン)
  cv::Mat_<double> Rt_ = cv::Mat_<double>(4, 4);        
  Rt_ << R.at<double>(0, 0), R.at<double>(0, 1), R.at<double>(0, 2), t.at<double>(0, 0), R.at<double>(1, 0), R.at<double>(1, 1), R.at<double>(1, 2),t.at<double>(1, 0),R.at<double>(2, 0),R.at<double>(2, 1),R.at<double>(2, 2),t.at<double>(2, 0),0,0,0,1;
  cv::Mat Rt = Rt_;
  std::cout << "Rt=" << Rt << std::endl << std::endl; 


    std::cout <<"for文終了"<< std::endl;

    /*cv::namedWindow(win_src, cv::WINDOW_AUTOSIZE);
    cv::namedWindow(win_edge, cv::WINDOW_AUTOSIZE);
    cv::namedWindow(win_dst, cv::WINDOW_AUTOSIZE);
    cv::namedWindow(win_dst2, cv::WINDOW_AUTOSIZE);
    cv::namedWindow(win_depth, cv::WINDOW_AUTOSIZE);
    cv::namedWindow(win_line, cv::WINDOW_AUTOSIZE);

    //cv::imshow(win_src, img_src);
    //cv::imshow(win_edge, img_edge);
    cv::imshow(win_dst, img_dst);
    cv::imshow(win_dst2, img_dst2);
    cv::imshow(win_depth, img_depth4);
    cv::imshow(win_line, img_line);*/

    RGBimage3 = bridgeRGBImage->image.clone();//一つ前の画像をメモリー保存
    kaisu=kaisu+1;
    sec=sec+1;

  	cv::waitKey(1);
   //ros::spinにジャンプする
}

//メイン関数
int main(int argc,char **argv){

	ros::init(argc,argv,"opencv_main");//rosを初期化
	ros::NodeHandle nhSub;//ノードハンドル
	
	//subscriber関連
	message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nhSub, "/robot1/camera/color/image_raw", 1);
	message_filters::Subscriber<sensor_msgs::Image> depth_sub(nhSub, "/robot1/camera/depth/image_rect_raw", 1);

  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,sensor_msgs::Image> MySyncPolicy;	
	message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10),rgb_sub, depth_sub);
	sync.registerCallback(boost::bind(&callback,_1, _2));

  ros::NodeHandle n;
  marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);
  marker_pub_W = n.advertise<visualization_msgs::Marker>("Would_marker", 10);
  // image_pub = n.advertise<sensor_msgs::Image>("maskImageData", 10);//realsenseの画像データをパブリッシュ


	ros::spin();//トピック更新待機
			
	return 0;
}