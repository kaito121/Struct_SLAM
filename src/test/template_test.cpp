//20211006 差分二乗和(SSD)を使用したテンプレートマッチングテストプログラム
//差分二乗和を使用することで誤マッチング時、マッチングを消すことができる
//またSSDは値が小さくなるほど一致度が高くなる

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
std::string win_dst = "dst";
std::string win_template1 = "template1";
std::string win_minmax1 = "minmax1";
std::string win_1 = "1";
std::string win_2 = "2";


using namespace std;
using namespace cv;

int best = 30;
int kaisu= 0;
cv::Mat img_minmax1,img_dst;
cv::Mat img_template1,img_template2;
cv::Mat img_1,img_2;

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

  cv::Mat img_src = bridgeRGBImage->image.clone();
  cv::Mat img_depth = depthimage;

//テンプレート画像準備
if(kaisu==0){
  img_1 = bridgeRGBImage->image.clone();
  img_1.copyTo(img_2);
  cv::rectangle(img_2, cv::Point(100, 100),cv::Point(200, 200), cv::Scalar(255, 255, 255), 5);
}

else{
  //テンプレート画像作成
  cv::Rect roi1(cv::Point(100, 100), cv::Size(100, 100));
  img_template1 = img_1(roi1); // 切り出し画像
    
  img_src.copyTo(img_dst);

  // テンプレートマッチング
  cv::matchTemplate(img_src, img_template1, img_minmax1, cv::TM_SQDIFF);//差分二乗和

  // 最小位置に矩形描画
  cv::Point min_pt1, max_pt1;
  double min_val1, max_val1;

  cv::minMaxLoc(img_minmax1, &min_val1, &max_val1, &min_pt1, &max_pt1);//配列の最大と最小を求める関数
  std::cout << "min_val1(白)=" << min_val1 << std::endl;//一致度が上がると値が小さくなる
  std::cout << "max_val1(白)=" << max_val1 << std::endl;
  //最小値がしきい値以下なら表示
  if(min_val1<max_val1*0.01){
    cv::rectangle(img_dst, cv::Rect(min_pt1.x, min_pt1.y, img_template1.cols, img_template1.rows), cv::Scalar(255, 255, 255), 10);//白
    std::cout << "test0" << std::endl;

    img_template1.copyTo(img_template2);
    std::cout << "test1" << std::endl;

    cv::matchTemplate(img_template1, img_template2, img_minmax1, cv::TM_SQDIFF);//差分二乗和
    std::cout << "test2" << std::endl;


  }
}

  // ウインドウ生成
  cv::namedWindow(win_src, cv::WINDOW_AUTOSIZE);
  cv::namedWindow(win_template1, cv::WINDOW_AUTOSIZE);
  cv::namedWindow(win_minmax1, cv::WINDOW_AUTOSIZE);
  cv::namedWindow(win_dst, cv::WINDOW_AUTOSIZE);
  cv::namedWindow(win_1, cv::WINDOW_AUTOSIZE);
  cv::namedWindow(win_2, cv::WINDOW_AUTOSIZE);

  // 表示
  cv::imshow(win_src, img_src);
  cv::imshow(win_1, img_1);
  cv::imshow(win_2, img_2);
  if(kaisu!=0){
    cv::imshow(win_template1, img_template1);
    cv::imshow(win_minmax1, img_minmax1);
    cv::imshow(win_dst, img_dst);
  }


   kaisu=kaisu+1;
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

