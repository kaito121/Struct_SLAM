#include <ros/ros.h>
#include <opencv2/opencv.hpp>

std::string win_src1 = "src1";
std::string win_src2 = "src2";
std::string win_dst = "dst";
 
int main(void)
{
	cv::Mat src1, src2, dst;
 
	src1 = cv::imread("/home/fuji/catkin_ws/src/Struct_SLAM/src/OptF/input/20210527_210527_4.jpg");
	src2 = cv::imread("/home/fuji/catkin_ws/src/Struct_SLAM/src/OptF/input/20210527_210527_5.jpg");
 
	// 特徴点検出アルゴリズムの選択
	// # ORB::create(検出特徴点数, scale factor, ...)
	cv::Ptr<cv::ORB>  orb = cv::ORB::create(100);
 
	// 検出したキーポイント（特徴点）を格納する配列
	std::vector<cv::KeyPoint> key1, key2;
 
	// キーポイントの検出
	orb->detect(src1, key1);
	orb->detect(src2, key2);
 
	// 特徴量記述の計算
	cv::Mat des1, des2;
	orb->compute(src1, key1, des1);
	orb->compute(src2, key2, des2);
 
	// 特徴点マッチングアルゴリズムの選択
	cv::Ptr<cv::DescriptorMatcher> hamming = cv::DescriptorMatcher::create("BruteForce-Hamming");
 
	// 特徴点マッチング
	// * 特徴量記述des1とdes2のマッチングを行い、結果をmatchへ書き込む
	std::vector<cv::DMatch> match;
	hamming->match(des1, des2, match);
 
	// 特徴点マッチングの結果を画像化する
	cv::drawMatches(src1, key1, src2, key2, match, dst);
 
	cv::imshow("画像1", src1);
	cv::imshow("画像2", src2);
	cv::imshow("マッチング結果", dst);

	cv::namedWindow(win_src1, cv::WINDOW_AUTOSIZE);
	cv::namedWindow(win_src2, cv::WINDOW_AUTOSIZE);
    cv::namedWindow(win_dst, cv::WINDOW_AUTOSIZE);

   
    cv::imshow(win_src1, src1);
	cv::imshow(win_src2, src2);
    cv::imshow(win_dst, dst);
  
	cv::waitKey();
 
	return 0;
}

