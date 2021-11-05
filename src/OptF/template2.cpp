//RealsenseのRGB画像とDepth画像では画角が違うため画像情報が一致しない
//そのためDepthの画像を調節してRGB画像を一致させる必要がある
//ここではDepth画像をRGB画像に一致するように切り取り、拡大することでRGB画層とDeoth画像の一致を行う

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
#include <librealsense2/rs.hpp>//Realsenseの画角調整用


ros::Subscriber sub;//データをsubcribeする奴
std::string win_src = "src";
std::string win_edge = "edge";
std::string win_dst = "dst";
std::string win_dst2 = "dst2";
std::string win_depth = "depth";
std::string win_line = "line";
std::string win_template1 = "template1";
std::string win_minmax1 = "minmax1";
std::string win_minmax2 = "minmax2";
std::string win_minmax3 = "minmax3";
std::string win_curr = "curr";
std::string win_prev = "prev";
std::string win_1 = "1";
std::string win_2 = "2";
std::string win_temp_match="temp_match";
std::string win_match1="match1";


using namespace std;
using namespace cv;

// 抽出する画像の輝度値の範囲を指定
#define B_MAX 255
#define B_MIN 150
#define G_MAX 255
#define G_MIN 180
#define R_MAX 255
#define R_MIN 180
int best = 30;
int kaisu= 1;
cv::Mat RGBimage3;//ここで定義する必要がある
cv::Mat img_minmax1,img_dst,img_minmax2,img_minmax3,img_minmax4;
cv::Mat img_match_minmax1;

cv::Mat frame,image_curr, image_prev,img_1,img_2;

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
        ROS_ERROR("Could not convert from '%s' to 'BGR8'.",rgb_msg->encoding.c_str());
        return ;}

    try{//MAT形式変換
       bridgedepthImage=cv_bridge::toCvCopy(depth_msg, sensor_msgs::image_encodings::TYPE_32FC1);//MAT形式に変える
       ROS_INFO("callBack");}//printと秒数表示
    
	//エラー処理
    catch(cv_bridge::Exception& e) {//エラー処理(失敗)成功ならスキップ
        std::cout<<"depth_image_callback Error \n";
        ROS_ERROR("Could not convert from '%s' to '32FC1'.",depth_msg->encoding.c_str());
        return ;}
        
    std::cout << "kaisu=" << kaisu << std::endl;
    RGBimage = bridgeRGBImage->image.clone();//image変数に変換した画像データを代入
    depthimage = bridgedepthImage->image.clone();//image変数に変換した画像データを代入
    image_curr = bridgeRGBImage->image.clone();

  cv::Mat img_src = bridgeRGBImage->image.clone();
  cv::Mat img_depth = depthimage;


  /*//-------------------------------------------------------------------------------------------------------------------------  
  //画像クロップ(中距離でほぼ一致)
  cv::Rect roi(cv::Point(110, 95), cv::Size(640/1.6, 480/1.6));//このサイズでDepth画像を切り取るとほぼカメラ画像と一致する
  cv::Mat img_dst = img_depth(roi); // 切り出し画像
  cv::Mat img_dst2;//修正版Depth画像
  resize(img_dst, img_dst2,cv::Size(), 1.6, 1.6);//クロップした画像を拡大
  //------------------------------------------------------------------------------------------------------------------------*/
if(kaisu==1){
  img_1 = bridgeRGBImage->image.clone();
  //img_2 = img_1;
  img_1.copyTo(img_2);
  cv::rectangle(img_2, cv::Point(100, 100),cv::Point(200, 200), cv::Scalar(255, 255, 255), 5);
  cv::rectangle(img_2, cv::Point(110, 110),cv::Point(210, 210), cv::Scalar(255, 255, 0), 5);
  //cv::rectangle(img_2, cv::Point(100, 100),cv::Point(200, 200), cv::Scalar(255, 0, 0), 5);
  //cv::rectangle(img_2, cv::Point(300, 300),cv::Point(400, 400), cv::Scalar(255, 0, 0), 5);


}

if(kaisu!=1){
    cv::Rect roi1(cv::Point(100, 100), cv::Size(100, 100));
    cv::Mat img_template1 = img_1(roi1); // 切り出し画像

    cv::Rect roi2(cv::Point(100, 100), cv::Size(100, 100));
    cv::Mat img_template2 = img_1(roi2); // 切り出し画像

    cv::Rect roi3(cv::Point(100, 100), cv::Size(100, 100));
    cv::Mat img_template3 = img_1(roi3); // 切り出し画像

    cv::Rect roi4(cv::Point(110, 110), cv::Size(100, 100));
    cv::Mat img_template4 = img_1(roi4); // 切り出し画像

    

  /*if (!img_src.data || !img_template.data) {
    std::cout << "error" << std::endl;
    return -1;
  }*/

  img_src.copyTo(img_dst);

  // テンプレートマッチング
  cv::matchTemplate(image_curr, img_template1, img_minmax1, cv::TM_CCORR_NORMED);//正規化相互関数
  cv::matchTemplate(image_curr, img_template2, img_minmax2, cv::TM_CCOEFF_NORMED);//正規化相関係数
  cv::matchTemplate(image_curr, img_template3, img_minmax3, cv::TM_SQDIFF_NORMED);//正規化差分2乗和
  cv::matchTemplate(image_curr, img_template4, img_minmax4, cv::TM_CCORR_NORMED);//正規化相互関数

  // 最大位置に矩形描画
  cv::Point min_pt1, max_pt1,min_pt2, max_pt2,min_pt3, max_pt3,min_pt4, max_pt4;
  double min_val1, max_val1,min_val2, max_val2,min_val3, max_val3,min_val4, max_val4;

  cv::minMaxLoc(img_minmax1, &min_val1, &max_val1, &min_pt1, &max_pt1);//配列の最大と最小を求める関数
  if(max_val1>0.9){
  std::cout << "min_val1(白)=" << min_val1 << std::endl;//一致度が上がると値が大きくなる
  std::cout << "max_val1(白)=" << max_val1 << std::endl;
  cv::rectangle(img_dst, cv::Rect(max_pt1.x, max_pt1.y, img_template1.cols, img_template1.rows), cv::Scalar(255, 255, 255), 10);}//白

  else{
    std::cout << "min_val1(紫)=" << min_val1 << std::endl;//一致度が上がると値が大きくなる
    std::cout << "max_val1(紫)=" << max_val1 << std::endl;
    //Scv::rectangle(img_dst, cv::Rect(max_pt1.x, max_pt1.y, img_template1.cols, img_template1.rows), cv::Scalar(255, 0, 255), 10);
    //cv::rectangle(img_dst, cv::Rect(min_pt1.x, min_pt1.y, img_template1.cols, img_template1.rows), cv::Scalar(0, 0, 255), 10);
  }

  cv::minMaxLoc(img_minmax2, &min_val2, &max_val2, &min_pt2, &max_pt2);
  std::cout << "min_val2(青)=" << min_val2 << std::endl;
  std::cout << "max_val2(青)=" << max_val2 << std::endl;
  //cv::rectangle(img_dst, cv::Rect(max_pt2.x, max_pt2.y, img_template2.cols, img_template2.rows), cv::Scalar(255, 0, 0),5);//青

  cv::minMaxLoc(img_minmax3, &min_val3, &max_val3, &min_pt3, &max_pt3);
  std::cout << "min_val3(緑)=" << min_val3 << std::endl;
  std::cout << "max_val3(緑)=" << max_val3 << std::endl;
  //cv::rectangle(img_dst, cv::Rect(min_pt3.x, min_pt3.y, img_template3.cols, img_template3.rows), cv::Scalar(0, 255, 0), 3);//緑

   cv::minMaxLoc(img_minmax4, &min_val4, &max_val4, &min_pt4, &max_pt4);//配列の最大と最小を求める関数
  std::cout << "min_val4(水色)=" << min_val4 << std::endl;//一致度が上がると値が大きくなる
  std::cout << "max_val4(水色)=" << max_val4 << std::endl;
 //cv::rectangle(img_dst, cv::Rect(max_pt4.x, max_pt4.y, img_template4.cols, img_template4.rows), cv::Scalar(255, 255, 0), 5);
  //水色

  //２重マッチング法----------------------------------------------------------------------------------------------
  cv::Mat img_template1_match;
  img_template1.copyTo(img_template1_match);//結果表示用コピー作成

  cv::Rect roi_match1(cv::Point(max_pt1.x, max_pt1.y), cv::Size(img_template1.cols, img_template1.rows));
  cv::Mat img_match1 = image_curr(roi_match1); // 切り出し画像

  cv::Point match_min_pt1, match_max_pt1;
  double match_min_val1, match_max_val1;

  cv::matchTemplate(img_template1,img_match1, img_match_minmax1, cv::TM_CCORR_NORMED);//正規化相互関数
  cv::minMaxLoc(img_match_minmax1, &match_min_val1, &match_max_val1, &match_min_pt1, &match_max_pt1);//配列の最大と最小を求める関数
  std::cout << "match_min_val1(赤)=" << match_min_val1 << std::endl;//一致度が上がると値が大きくなる
  std::cout << "match_max_val1(赤)=" << match_max_val1 << std::endl;
  if(match_max_val1>0.95){
  cv::rectangle(img_template1_match, cv::Rect(match_max_pt1.x, match_max_pt1.y, img_template1.cols, img_template1.rows), cv::Scalar(0, 0, 255), 10);}



  // ウインドウ生成
  cv::namedWindow(win_src, cv::WINDOW_AUTOSIZE);
  cv::namedWindow(win_template1, cv::WINDOW_AUTOSIZE);
  cv::namedWindow(win_minmax1, cv::WINDOW_AUTOSIZE);
  cv::namedWindow(win_minmax2, cv::WINDOW_AUTOSIZE);
  cv::namedWindow(win_minmax3, cv::WINDOW_AUTOSIZE);
  cv::namedWindow(win_dst, cv::WINDOW_AUTOSIZE);
  cv::namedWindow(win_curr, cv::WINDOW_AUTOSIZE);
  cv::namedWindow(win_prev, cv::WINDOW_AUTOSIZE);
  cv::namedWindow(win_1, cv::WINDOW_AUTOSIZE);
  cv::namedWindow(win_2, cv::WINDOW_AUTOSIZE);
  cv::namedWindow(win_temp_match, cv::WINDOW_AUTOSIZE);
  cv::namedWindow(win_match1, cv::WINDOW_AUTOSIZE);

  // 表示
  cv::imshow(win_src, img_src);
  cv::imshow(win_template1, img_template1);
  cv::imshow(win_minmax1, img_minmax1);
  cv::imshow(win_minmax2, img_minmax2);
  cv::imshow(win_minmax3, img_minmax3);
  cv::imshow(win_dst, img_dst);
  cv::imshow(win_curr, image_curr);
  cv::imshow(win_prev, image_prev);
  cv::imshow(win_1, img_1);
  cv::imshow(win_2, img_2);
  cv::imshow(win_temp_match, img_template1_match);
  cv::imshow(win_match1, img_match1);
}


   kaisu=kaisu+1;
   if(kaisu==200){kaisu=2;}
  cv::swap(image_curr, image_prev);// image_curr を image_prev に移す（交換する）
  cv::waitKey(1);
   //ros::spinにジャンプする
   
}

//メイン関数
int main(int argc,char **argv){

	ros::init(argc,argv,"opencv_main");//rosを初期化
	ros::NodeHandle nhSub;//ノードハンドル
	
	//subscriber関連
	//message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nhSub, "/robot1/camera/color/image_raw", 1);
	//message_filters::Subscriber<sensor_msgs::Image> depth_sub(nhSub, "/robot1/camera/depth/image_rect_raw", 1);
  
  message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nhSub, "/camera/color/image_raw", 1);//センサーメッセージを使うときは対応したヘッダーが必要
	message_filters::Subscriber<sensor_msgs::Image> depth_sub(nhSub, "/camera/aligned_depth_to_color/image_raw", 1);

  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,sensor_msgs::Image> MySyncPolicy;	
	message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10),rgb_sub, depth_sub);
	sync.registerCallback(boost::bind(&callback,_1, _2));

 
	ros::spin();//トピック更新待機
			
	return 0;
}

