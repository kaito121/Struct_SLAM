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

std::string win_src = "src";
std::string win_dst = "dst";
std::string win_lbl = "lbl";
std::string win_hsv = "hsv";
std::string win_bin1 = "bin1";
std::string win_bin3 = "bin3";
std::string win_max_index = "max_index";
std::string win_dst2 = "dst2";
//std::string win = "main";

//int h_upper = 115, h_lower = 60;
//int s_upper = 255, s_lower = 50;
//int v_upper = 200, v_lower = 20;

//int h_upper = 66, h_lower = 60;
//int s_upper = 255, s_lower = 50;
//int v_upper = 255, v_lower = 20;

int h_upper = 81, h_lower =75;
int s_upper = 245, s_lower = 50;
int v_upper = 255, v_lower = 20;
cv::Mat img_dst2;
int kaisu=0;

int main()
{
  //std::string filepath = "/home/fuji/catkin_ws/src/Struct_SLAM/src/OptF/04-06.wmv";
  //std::string filepath = "/home/fuji/catkin_ws/src/Struct_SLAM/src/OptF/train103.wmv";
  std::string filepath = "/home/fuji/catkin_ws/src/Struct_SLAM/src/OptF/test_klman.mp4";
	// 動画ファイルを取り込むためのオブジェクトを宣言する
	cv::VideoCapture video;
	// 動画ファイルを開く
	video.open(filepath);
	if (video.isOpened() == false) {
     ROS_INFO("callback");
		// 動画ファイルが開けなかったときは終了する
		return 0;
	}
	cv::VideoWriter writer; // 動画ファイルを書き出すためのオブジェクトを宣言する
  int    width, height, fourcc; // 作成する動画ファイルの設定
	double fps;
  //fourcc = cv::VideoWriter::fourcc('w', 'm', 'v', 'v'); // ビデオフォーマットの指定( ISO MPEG-4 / .wmv)
  fourcc = cv::VideoWriter::fourcc('m', 'p', '4', 'v'); // ビデオフォーマットの指定( ISO MPEG-4 / .mp4)
	width  = (int)video.get(cv::CAP_PROP_FRAME_WIDTH);	// フレーム横幅を取得
	height = (int)video.get(cv::CAP_PROP_FRAME_HEIGHT);	// フレーム縦幅を取得
	fps    = video.get(cv::CAP_PROP_FPS);				// フレームレートを取得

	//writer.open("CloneVideo.mp4", fourcc, fps, cv::Size(width, height));

  /*cv::VideoCapture cap("04-06.wmv");
  
  cvcv::Mat img_src;::namedWindow(win);
  ROS_INFO("callback_functionが呼ばれたよ");*/

  cv::KalmanFilter KF(4, 2);
  KF.statePre.at<float>(0) = 0;//予測の状態方程式(x'(k)): x(k)=A*x(k-1)+B*u(k)
  KF.statePre.at<float>(1) = 0;
  KF.statePre.at<float>(2) = 0;
  KF.statePre.at<float>(3) = 0;

  // 運動モデル(システムの時間遷移に関する線形モデル式)
  KF.transitionMatrix = (cv::Mat_<float>(4, 4) << // 等速直線運動（速度利用あり）
    1, 0, 1, 0,
    0, 1, 0, 1,
    0, 0, 1, 0,
    0, 0, 0, 1);

  setIdentity(KF.measurementMatrix);//観測データの配列(H)
  setIdentity(KF.processNoiseCov, cv::Scalar::all(1e-1)); //誤差共分散の雑音(Q)
  setIdentity(KF.measurementNoiseCov, cv::Scalar::all(1e-1));//観測誤差の標準偏差（Rt)(出力方程式の誤差共分散)
  setIdentity(KF.errorCovPost, cv::Scalar::all(1e-1));//誤差共分散の更新式(P(k)): P(k)=(I-K(k)*H)*P'(k)

  cv::Mat img_hsv, img_gray, img_gray_th, img_bin1,img_bin2 ,img_bin3,img_lbl, img_dst, img_rgb_th;
  cv::Mat element8 = (cv::Mat_<uchar>(3, 3) << 1, 1, 1, 1, 1, 1, 1, 1, 1); // 8近傍
  cv::Mat img_src;


	while (1) {
		video >> img_src; // videoからimageへ1フレームを取り込む
		if (img_src.empty() == true) break; // 画像が読み込めなかったとき、無限ループを抜ける
		cv::imshow("showing", img_src); // ウィンドウに動画を表示する
    img_src.copyTo(img_dst2);
    //cv::circle(img_src, cv::Point(kaisu,100), 30, cv::Scalar(30, 90, 40), -1);//予測の点（黄色）
    cv::rectangle(img_src, cv::Rect(kaisu, 100, 50, 50), cv::Scalar(30, 90, 40), 10);//白


  /*while (1) {
    std::vector<cv::Mat> vec_hsv(3);
    cap >> img_src;
    ROS_INFO("callback_functionが呼ばれたよ1");*/

    std::vector<cv::Mat> vec_hsv(3);

    // 追跡対象の抽出
    cv::cvtColor(img_src, img_gray, cv::COLOR_BGR2GRAY);
    cv::cvtColor(img_src, img_hsv, cv::COLOR_BGR2HSV_FULL);//hsv画像化
    cv::split(img_hsv, vec_hsv);//画像をHSVの色で分割
    ROS_INFO("callback_functionが呼ばれたよ2");
    //std::cout <<"vec_hsv="<<vec_hsv<<std::endl;
    cv::imshow(win_hsv, img_hsv);

    // HSV閾値処理（あるしきい値以上の色のみを二値化で抽出）
    cv::inRange(img_hsv, cv::Scalar(h_lower, s_lower, v_lower), cv::Scalar(h_upper, s_upper, v_upper), img_bin1);
    ROS_INFO("callback_functionが呼ばれたよ3");
    cv::imshow(win_bin1, img_bin1);

    // ノイズ処理（抽出した画像に対しノイズ処理）
    //cv::dilate(img_bin1, img_bin3, element8, cv::Point(-1, -1), 5); // 膨張オンリー

    cv::erode(img_bin1, img_bin2, element8, cv::Point(-1, -1), 5); // 収縮
    cv::dilate(img_bin2, img_bin3, element8, cv::Point(-1, -1), 5); // 膨張
    ROS_INFO("callback_functionが呼ばれたよ4");
    cv::imshow(win_bin3, img_bin3);

    // 面積最大ラベルの選択
    cv::Mat stats, centroids;
    ROS_INFO("callback_functionが呼ばれたよ5");
    int labelnum = cv::connectedComponentsWithStats(img_bin3, img_lbl, stats, centroids);//オブジェクト検出
    std::cout <<"labelnum="<<labelnum<<std::endl;
    ROS_INFO("callback_functionが呼ばれたよ6");
    if (labelnum == 1) continue;
    long int max_area = 0, max_index = 0;
    for (int i = 1; i < labelnum; i++) {
      int area = stats.at<int>(i, cv::CC_STAT_AREA);
      std::cout <<"area="<<area<<std::endl;
      if (area > max_area) {
        max_area = area;
        max_index = i;
      }
    }
    std::cout <<"max_index="<<max_index<<std::endl;
    cv::compare(img_lbl, max_index, img_dst, cv::CMP_EQ);

    // 面積最大ラベルの重心(面積最大を計算しているので一つしか出ない）
    cv::Moments m = cv::moments(img_dst, true);//抽出画像の重心を求める
    cv::Point pos(m.m10 / m.m00, m.m01 / m.m00);//重心計算(x,y)
    std::cout <<"m.m10="<<m.m10<<std::endl;
    std::cout <<"m.m00="<<m.m00<<std::endl;
    std::cout <<"m.m01="<<m.m01<<std::endl;
    std::cout <<"m.m00="<<m.m00<<std::endl;

    // カルマンフィルタ
    // 観測
    cv::Mat measurement(2, 1, CV_32F);
    measurement.at<float>(0) = pos.x;//重心位置x座標
    measurement.at<float>(1) = pos.y;//重心位置y座標
    std::cout <<"観測座標_pos.x="<<pos.x<<std::endl;
    std::cout <<"観測座標_pos.y="<<pos.y<<std::endl;

    // 修正（カルマンフィルタ関数）ここで測定結果から予測された状態の更新を行っている
    cv::Mat correction = KF.correct(measurement);//観測値をカルマンフィルタに代入

    // 予測（カルマンフィルタ関数）
    cv::Mat prediction = KF.predict();//予測状態の計算

    // 結果の描画
    // 重心位置
    cv::circle(img_src, pos, 5, cv::Scalar(0, 0, 255), -1);//赤点


    // 予測位置
    cv::circle(img_src, cv::Point(prediction.at<float>(0), prediction.at<float>(1)), 5, cv::Scalar(0, 255, 255), -1);//予測の点（黄色）
    cv::ellipse(img_src, cv::Point(prediction.at<float>(0), prediction.at<float>(1)),
    cv::Size(abs(prediction.at<float>(2)), abs(prediction.at<float>(3))),
    0.0, 0.0, 360.0, cv::Scalar(0, 255, 255), 3);//予測の円（黄色）
    std::cout <<"予測座標x="<<prediction.at<float>(0)<<std::endl;
    std::cout <<"予測座標y="<<prediction.at<float>(1)<<std::endl;

    // 画面表示
    //cv::imshow(win, img_src);
    cv::imshow(win_src, img_src);
    cv::imshow(win_dst, img_dst);
    cv::imshow(win_dst2, img_dst2);
    cv::imshow(win_lbl, img_lbl);
    kaisu=kaisu+20;
    if (cv::waitKey(100) == 'q') break;
  }

  return 0;
}
