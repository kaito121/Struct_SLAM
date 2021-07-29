//Depth調整バージョン(D435_test.cppを参照)
//rosのヘッダ
//20210601
//カメラをつかってリアルタイムで物体の追跡が可能プログラム
//しかし追跡はできるがマッチングはしていないので、配列はバラバラに格納されている
//そのためオプティカルフローなどをする場合はこれにマッチングのプログラムを導入する必要がある
#include <ros/ros.h>
#include <sensor_msgs/Image.h>//センサーデータ形式ヘッダ
#include <cv_bridge/cv_bridge.h>//画像変換のヘッダ
#include <sensor_msgs/image_encodings.h>//エンコードのためのヘッダ
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <math.h>
#include <highgui.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/ximgproc/fast_line_detector.hpp>//FLD


ros::Subscriber sub;//データをsubcribeする奴
std::string win_src = "src";//カメラ画像
std::string win_depth = "depth";//深度画像（修正前）
std::string win_depth2 = "depth2";//深度画像（修正前）+FLD線
std::string win_depth3 = "depth3";//深度画像（修正後）
std::string win_depth4 = "depth4";//深度画像（修正後）+FLD線
std::string win_edge = "edge";
std::string win_dst = "dst";//カメラ画像+FLDの線表示
std::string win_line = "line";//FLDの線を表示
std::string win_line2 = "line2";//FLD+確率Houghの線を表示
std::string win_prev = "prev";
std::string win_curr = "curr";

using namespace std;
using namespace cv;

// 抽出する画像の輝度値の範囲を指定
#define B_MAX 255
#define B_MIN 150
#define G_MAX 255
#define G_MIN 180
#define R_MAX 255
#define R_MIN 180
// reset == TRUE のとき特徴点検出を行う
// 最初のフレームで必ず特徴点検出を行うように、初期値を TRUE にする
bool reset = true;
cv::Mat frame,image_curr, image_prev,img_dst;
vector<cv::Point2f> points_prev, points_curr;

//コールバック関数
void callback(const sensor_msgs::Image::ConstPtr& rgb_msg,const sensor_msgs::Image::ConstPtr& depth_msg)
{
	//変数宣言
	cv_bridge::CvImagePtr bridgeImage;//クラス::型//cv_brigeは画像変換するとこ
    cv_bridge::CvImagePtr bridgedepthImage;//クラス::型//cv_brigeは画像変換するとこ
    cv::Mat RGBimage;//opencvの画像
    cv::Mat depthimage;//opencvの画像
	cv::Mat image;//opencvの画像
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

    image = bridgeImage->image.clone();//image変数に変換した画像データを代入
    depthimage = bridgedepthImage->image.clone();//image変数に変換した画像データを代入
	cv::imshow(win_src, image);
	int req;

	
	// reset == TRUE のとき特徴点検出を行う
	// 最初のフレームで必ず特徴点検出を行うように、初期値を TRUE にする
	//bool reset = true;
 
	// image_curr:  現在の入力画像、    image_prev:  直前の入力画像
	// points_curr: 現在の特徴点リスト、points_prev: 直前の特徴点リスト
	//cv::Mat frame,image_curr, image_prev,img_dst;
	//vector<cv::Point2f> points_prev, points_curr;
		
	image.copyTo(frame);//ここ間違えていたので注意
	image.copyTo(img_dst);//ここ間違えていたので注意

	cv::cvtColor(image, image_curr, cv::COLOR_BGR2GRAY);//グレースケール化
	
 
		if (reset == true) {//最初のフレーム
		std::cout <<"初回検出プログラム"<< std::endl;//特徴点の座標
			// 特徴点検出(グレースケール画像から特徴点検出)
			cv::goodFeaturesToTrack(image_curr, points_curr, 500, 0.01, 10, cv::Mat(), 3, 3, 0, 0.04);
			cv::cornerSubPix(image_curr, points_curr, cv::Size(10, 10), cv::Size(-1, -1), cv::TermCriteria(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, 20, 0.03));
			points_prev = points_curr;//データのコピー
			reset = false;//if文切り替え
		} 
		else {
			std::cout <<"二回目オプティカルフロー"<< std::endl;//特徴点の座標
			// 特徴点追跡(二回目のフレーム)
			vector<uchar> status;//特徴点の数
			vector<float> err;
 
			cv::calcOpticalFlowPyrLK(image_prev, image_curr, points_prev, points_curr, status, err);//オプティカルフロー

			std::cout <<"status.size()="<<status.size()<< std::endl;//特徴点の個数
 
			// 追跡できなかった特徴点をリストから削除する
			int i, k;
			req=1;
			for (i = k = 0; i < status.size(); i++)
			{
				std::cout <<"status["<<i<<"]="<<status[i]<< std::endl;//0以外の時は変なマークがでる
				if (status[i] == 0) {
					continue;
				}
				points_prev[k]   = points_prev[i];
				points_curr[k++] = points_curr[i];
			}
			points_curr.resize(k);
			points_prev.resize(k);

		}
 
		// 特徴点を丸で描く
		for (int i = 0; i < points_curr.size(); i++) {
			cv::Scalar c(0, 255, 0);//緑
			if (cv::norm(points_prev[i] - points_curr[i]) > 0.5) {
				c = cv::Scalar(0, 100, 255);//オレンジ
			}
			if(i==0){cv::circle(img_dst, points_curr[i], 8, Scalar(0,255,255), -1, cv::LINE_AA);}
			else{cv::circle(img_dst, points_curr[i], 3, c, -1, cv::LINE_AA);}
			
			std::cout <<"points_curr["<<i<<"]="<<points_curr[i]<< std::endl;//特徴点の座標
			std::cout <<"points_prev["<<i<<"]="<<points_prev[i]<< std::endl;//特徴点の座標
		}

		cv::imshow("win_dst", img_dst);
		cv::imshow("win_curr", image_curr);//今の画像

		if (req == 1) {//初回は直前の画像がないため考慮
		cv::imshow("win_prev", image_prev);//一つ前の画像像
		std::cout <<"でてるよ"<< std::endl;//特徴点の座標
		}
 
		int key = cv::waitKey(30);
		if (key == 'r') {
			// Rキーが押されたら特徴点を再検出
			//reset = true;
		}
		
		// image_curr を image_prev に移す（交換する）
		cv::swap(image_curr, image_prev);
		// points_curr を points_prev に移す（交換する）
		cv::swap(points_curr, points_prev);
 
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


	ros::spin();//トピック更新待機
			
	return 0;
}