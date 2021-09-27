#define _CRT_SECURE_NO_WARNINGS
#include <ros/ros.h>
#include <iostream>
#include <string>
#include <vector>
#include <opencv2/opencv.hpp>
#include <opencv/highgui.h>
#include <cv_bridge/cv_bridge.h>//画像変換のヘッダ

#include <nav_msgs/Path.h>//経路情報を記録する
#include <time.h>//処理の時間を出力する
#include <sys/time.h>

std::string win_src = "src";
std::string win_dst = "dst";

using namespace std;
using namespace cv;


//メイン関数
int main(int argc, char** argv)
{	
	ros::init(argc,argv,"marker2");//rosを初期化
	ros::NodeHandle nhSub;//ノードハンドル
	
	//RANSAC_y_Ax_B();

	int trueCount=200;
	int falseCount=200;
	int XMax=400;
	int cornerCount=800;
	int inlierCount=400;
	int outlierCount=cornerCount-inlierCount;

	vector<cv::Point2f> corners[800];//すべての点群

	//正解の値を生成  //create inliers
	for ( int n=0;n<400;n++)
	{
		double a=1.2;
		double b=20;		

		int randomnoise=(rand()%10)-(rand()%10);    //-10<= randomnoise <=10;

		corners[n].x=n;
		corners[n].y=a * n + b + randomnoise;
	}

	//不正解の値をランダムで生成  //create outliers	
	for ( int n=inlierCount;n<cornerCount;n++)
	{
		double r=rand()%XMax;
		corners[n].x=r;
		r=rand()%XMax;
		corners[n].y=r;
	}
		RANSAC_y_Ax_B(corners,cornerCount);
		

    return 0;
}
			
	return 0;
}