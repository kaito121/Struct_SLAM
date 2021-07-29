//20210527  
//画像座標の変化具合を知るためにオプティカルフローを使う
#include <ros/ros.h>
/*#include <stdlib.h>
#include <opencv2/opencv.hpp>
#include <opencv/highgui.h>
#include <string.h>
#include <stdio.h>
#include <math.h>*/

#include <opencv2/opencv.hpp>
#include <opencv2/superres/optical_flow.hpp>

std::string win_src = "src";
std::string win_result = "result";
std::string win_magnitude = "magnitude";


int main(int argc, char* argv[])
{
	// オプティカルフローの計算手法の決定
	//cv::Ptr<cv::superres::DenseOpticalFlowExt> opticalFlow = cv::superres::createOptFlow_Farneback();     // Franeback
	cv::Ptr<cv::superres::DenseOpticalFlowExt> opticalFlow = cv::superres::createOptFlow_DualTVL1();		   // TVL1
	
	// Optical Flowを計算する前後2フレームを保存するMat
	cv::Mat1b prev_img, curr_img;

	bool first_frame = true;
	std::string filename_base = "/home/fuji/catkin_ws/src/Struct_SLAM/src/OptF/input/20210527_210527_%d.jpg";
	std::string writename_base = "/home/fuji/catkin_ws/src/Struct_SLAM/src/OptF/result/out20210527_210527_%d.jpg";

	for (int i = 1; i < 7000; i++) {

		int start = clock();

		// ファイル名の取得
		std::string filename = cv::format(filename_base.c_str(), i);
		std::cout << "input image name : " << filename << std::endl;

		// 最初の1フレーム目の処理
		if (first_frame == true) {

			// 画像の取得
			prev_img = cv::imread(filename, 0);
			if (prev_img.empty() == true) {
				std::cerr << "【Error】cannot find input image : " << filename << std::endl;
			}
			first_frame = false;
		}

		// 2フレーム目以降の処理
		else {

			// 画像の取得
			curr_img = cv::imread(filename, 0);
			if (curr_img.empty() == true) {
				std::cerr << "【Error】cannot find input image : " << filename << std::endl;
			}

			// オプティカルフローの計算
			cv::Mat flow_x, flow_y;
			opticalFlow->calc(prev_img, curr_img, flow_x, flow_y);

			// オプティカルフローの2次元ベクトルの角度と大きさを算出
			cv::Mat magnitude, angle;
			cv::cartToPolar(flow_x, flow_y, magnitude, angle, true);

			// 大きさを正規化
			normalize(magnitude, magnitude, 0, 255, cv::NORM_MINMAX);

			// 表示
			/*cv::Mat display_img[2];
			cv::Mat result;
			display_img[0] = curr_img;
			magnitude.convertTo(magnitude, CV_8UC1);
			display_img[1] = magnitude;
			vconcat(display_img, 2, result); */    // 縦方向に結合

      cv::namedWindow(win_src, cv::WINDOW_AUTOSIZE);
      cv::namedWindow(win_magnitude, cv::WINDOW_AUTOSIZE);
      //cv::namedWindow(win_result, cv::WINDOW_AUTOSIZE);

      cv::imshow(win_src, prev_img);
      //cv::imshow(win_result, result);
      cv::imshow(win_magnitude, magnitude);

			cv::waitKey(1);

			// 保存
			//std::string writename = cv::format(writename_base.c_str(), i);
			//cv::imwrite(writename, result);

			// 前のフレームを保存
			prev_img = curr_img;
		}

		int end = clock();
		std::cout << "Processing time : " << end - start << "[ms]" << std::endl;
	}
	return 0;
}