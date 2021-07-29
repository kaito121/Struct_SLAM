//hsvで色でフィルタを作りあるしきい値以上の色のみをマスクで取り出す
//その後取り出した画像に対し膨張圧縮処理を施しノイズ除去
//マスク処理してある１面積のみにした画像に対し重心を求め、その重心の移動値からカルマンフィルタを使い位置を推定する
//つまりこのプログラムは１つのマークに対してのみしか処理が行われていない
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
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
 
using namespace std;
 
int main(void)
{
  std::string filepath = "/home/fuji/catkin_ws/src/Struct_SLAM/src/OptF/04-06.wmv";
  //std::string filepath = "/home/fuji/catkin_ws/src/Struct_SLAM/src/OptF/train103.wmv";
  //std::string filepath = "/home/fuji/catkin_ws/src/Struct_SLAM/src/OptF/smp1.mp4";
	// 動画ファイルを取り込むためのオブジェクトを宣言する
  ROS_INFO("callback_functionが呼ばれたよ2");
	cv::VideoCapture video;
	// 動画ファイルを開く
	video.open(filepath);
	if (video.isOpened() == false) {
     ROS_INFO("callback");
		// 動画ファイルが開けなかったときは終了する
		return 0;
	}

  ROS_INFO("callback_functionが呼ばれたよ3");
	cv::VideoWriter writer; // 動画ファイルを書き出すためのオブジェクトを宣言する
  int    width, height, fourcc; // 作成する動画ファイルの設定
	double fps;
  fourcc = cv::VideoWriter::fourcc('w', 'm', 'v', 'v'); // ビデオフォーマットの指定( ISO MPEG-4 / .wmv)
  //fourcc = cv::VideoWriter::fourcc('m', 'p', '4', 'v'); // ビデオフォーマットの指定( ISO MPEG-4 / .mp4)
	width  = (int)video.get(cv::CAP_PROP_FRAME_WIDTH);	// フレーム横幅を取得
	height = (int)video.get(cv::CAP_PROP_FRAME_HEIGHT);	// フレーム縦幅を取得
	fps    = video.get(cv::CAP_PROP_FPS);				// フレームレートを取得



	/*cv::VideoCapture cap;
	cap.open("/home/fuji/catkin_ws/src/Struct_SLAM/src/OptF/04-06.wmv");
 
	if (cap.isOpened() == false) {
		return 0;
	}*/
 
	// reset == TRUE のとき特徴点検出を行う
	// 最初のフレームで必ず特徴点検出を行うように、初期値を TRUE にする
  ROS_INFO("callback_functionが呼ばれたよ4");
	bool reset = true;
 
	// image_curr:  現在の入力画像、    image_prev:  直前の入力画像
	// points_curr: 現在の特徴点リスト、points_prev: 直前の特徴点リスト
	cv::Mat frame, image, image_curr, image_prev,img_src;
	vector<cv::Point2f> points_prev, points_curr;
  ROS_INFO("callback_functionが呼ばれたよ5");
 
	//for (;;) {
    while (1){
    ROS_INFO("callback_functionが呼ばれたよ6");
    video >> img_src; // videoからimageへ1フレームを取り込む
		if (img_src.empty() == true) {break;} // 画像が読み込めなかったとき、無限ループを抜ける
    ROS_INFO("callback_functionが呼ばれたよ7");
		cv::imshow("showing", img_src); // ウィンドウに動画を表示する
    ROS_INFO("callback_functionが呼ばれたよ8");
		
    img_src.copyTo(frame);
    ROS_INFO("callback_functionが呼ばれたよ8.1");
		cv::cvtColor(img_src, image_curr, cv::COLOR_BGR2GRAY);
    ROS_INFO("callback_functionが呼ばれたよ9");
 
		if (reset == true) {
			// 特徴点検出
			cv::goodFeaturesToTrack(image_curr, points_curr, 500, 0.01, 10, cv::Mat(), 3, 3, 0, 0.04);
			cv::cornerSubPix(image_curr, points_curr, cv::Size(10, 10), cv::Size(-1, -1), cv::TermCriteria(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, 20, 0.03));
			points_prev = points_curr;
			reset = false;
		} 
    else {
			// 特徴点追跡
			vector<uchar> status;
			vector<float> err;
 
			cv::calcOpticalFlowPyrLK(image_prev, image_curr, points_prev, points_curr, status, err);

      ROS_INFO("callback_functionが呼ばれたよ11");
 
			// 追跡できなかった特徴点をリストから削除する
			int i, k;
			for (i = k = 0; i < status.size(); i++)
			{
				if (status[i] == 0) {
					continue;
				}
				points_prev[k]   = points_prev[i];
				points_curr[k++] = points_curr[i];
			}
			points_curr.resize(k);
			points_prev.resize(k);
		}
    ROS_INFO("callback_functionが呼ばれたよ12");
 
		// 特徴点を丸で描く
		for (int i = 0; i < points_curr.size(); i++) {
			cv::Scalar c(0, 255, 0);
			if (cv::norm(points_prev[i] - points_curr[i]) > 0.5) {
				c = cv::Scalar(0, 100, 255);
			}
			cv::circle(img_src, points_curr[i], 3, c, -1, cv::LINE_AA);
		}
		cv::imshow("特徴点追跡", img_src);
    ROS_INFO("callback_functionが呼ばれたよ13");
 
		int key = cv::waitKey(30);
		if (key == 'r') {
			// Rキーが押されたら特徴点を再検出
			reset = true;
		}
    ROS_INFO("callback_functionが呼ばれたよ14");
		
		// image_curr を image_prev に移す（交換する）
		cv::swap(image_curr, image_prev);
		// points_curr を points_prev に移す（交換する）
		cv::swap(points_curr, points_prev);
    if (cv::waitKey(100) == 'q') break;
	
  }
	return 0;
}