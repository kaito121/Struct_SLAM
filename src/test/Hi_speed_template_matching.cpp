//特徴点検出のテストプログラム
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
#include <sensor_msgs/CameraInfo.h>//camera_infoを獲得するためのヘッダー
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <time.h>//処理の時間を出力する
#include <sys/time.h>

std::string win_src = "src";
std::string win_dst = "dst";

using namespace std;
using namespace cv;


cv::Mat img_dst,img_gray,img_akaze,img_orb,img_Harris;//画像定義
cv::Mat img_src,img_tmp;
vector<cv::Point2f> points_prev, points_curr;//特徴点定義
vector<cv::Point3f> camera_point_p,camera_point_c;//特徴点定義
sensor_msgs::CameraInfo camera_info;//CameraInfo受け取り用
int kaisu;


ros::Time ros_begin;//プログラム時間
ros::WallTime wall_begin = ros::WallTime::now();//プログラム開始時の時間
ros::WallDuration wall_prev;//一つ前の時間
ros::WallDuration wall_systemtime;//サンプリング時間
//ros::Time ros_begin = ros::Time::now();
struct timeval startTime, endTime;  // 構造体宣言
float realsec;//サンプリング時間（C++)

Mat pyr(Mat, int);   // 画像ピラミッド生成
void sad(Mat, Mat);  // 高速テンプレートマッチング


//コールバック関数
void callback(const sensor_msgs::Image::ConstPtr& rgb_msg,const sensor_msgs::Image::ConstPtr& depth_msg, const sensor_msgs::CameraInfo::ConstPtr& cam_info)
{
  ROS_INFO("callback_functionが呼ばれたよ");

  //サンプリング時間取得(C言語の方法)(こっちのほうが正確らしい)
  gettimeofday(&startTime, NULL);// 開始時刻取得
  if(kaisu!=0){
    time_t diffsec = difftime(startTime.tv_sec, endTime.tv_sec);    // 秒数の差分を計算
    suseconds_t diffsub = startTime.tv_usec - endTime.tv_usec;      // マイクロ秒部分の差分を計算
    realsec = diffsec+diffsub*1e-6;                          // 実時間を計算
    printf("処理の時間=%f\n", realsec);
  }

//サンプリング時間取得(ROS)
  ros::WallTime wall_now = ros::WallTime::now();
  ros::WallDuration wall_duration = wall_now - wall_begin;
  ROS_INFO("WALL:%u.%09u", wall_duration.sec, wall_duration.nsec);
  wall_systemtime = wall_duration - wall_prev;
  //ROS_INFO("systemtime:%u.%09u", wall_systemtime.sec, wall_systemtime.nsec);
std::cout << "wall_systemtime=" <<wall_systemtime<< std::endl;//サンプリング時間

    //変数宣言
  cv_bridge::CvImagePtr bridgeImage;//クラス::型//cv_brigeは画像変換するとこ
  cv_bridge::CvImagePtr bridgedepthImage;//クラス::型//cv_brigeは画像変換するとこ
  cv::Mat RGBimage,depthimage,image;
//
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
//  
  	camera_info=*cam_info;//CameraInfo受け取り
   
  	image = bridgeImage->image.clone();//image変数に変換した画像データを代入
  	depthimage = bridgedepthImage->image.clone();//image変数に変換した画像データを代入

	img_src = image.clone();// 入力画像
	cv::Rect roi2(cv::Point(100,100), cv::Size(100, 100));//特徴点を中心とした16☓16pixelの画像を切り取る
    img_tmp = image(roi2);// テンプレート画像
	

	cv::rectangle(img_src, cv::Point(100,100), cv::Point(200,200), cv::Scalar(255, 255, 255), 1, cv::LINE_AA);//四角形を描写(青)
	sad(image, img_tmp);

  
    // 画面表示
    cv::imshow(win_src, img_src);
    cv::imshow(win_dst, image);
    //cv::imshow(win_dst, img_dst);
	kaisu++;
	wall_prev=wall_duration;

    cv::waitKey(1);//ros::spinにジャンプする
}

Mat pyr(Mat src, int n)  //（入力画像，出力画像の大きさ[1/(2^n)]）
{
	Mat dst = src;

	for (int i = 0; i < n; i++)	pyrDown(dst, dst);  // pyrDownは元画像の半分の画像を生成する標準関数

	return dst;
}

void sad(Mat src, Mat tmp)
{
	int sad_min = INT_MAX;
	int sad_x = 0;
	int sad_y = 0;
    std::cout << "0sad_min" << sad_min << std::endl;


	// 第一段階
	Mat src1_4 = pyr(src, 2);  // ４分の１の入力画像
	Mat tmp1_4 = pyr(tmp, 2);  // ４分の１のテンプレート画像
	for (int y = 0; y < src1_4.rows - tmp1_4.rows; y++){
		for (int x = 0; x < src1_4.cols - tmp1_4.cols; x++){
			int sad = 0;
			for (int j = 0; j < tmp1_4.rows; j++){
				for (int i = 0; i < tmp1_4.cols; i++){
					sad += abs((int)src1_4.at<uchar>(y + j, x + i) - (int)tmp1_4.at<uchar>(j, i));
					if (sad > sad_min) {
        				//std::cout << "残差逐次検定法(y,x),(j,i)=("<<y<<","<<x<<"),("<<j<<","<<i<<")"<< std::endl;
        				std::cout << "0sad" << sad << std::endl;
        				std::cout << "0sad_min" << sad_min << std::endl;
						goto SSDA_loop1;
					}
				}
			}
			sad_min = sad;
			sad_x = x;
			sad_y = y;
        	std::cout << "1sad_min=" << sad_min << std::endl;
        	std::cout << "1sad_x=" << sad_x << std::endl;
        	std::cout << "1sad_y=" << sad_y << std::endl;

			SSDA_loop1:;
			if(sad > sad_min){
        		std::cout << "残差逐次検定法"<< std::endl;
				std::cout << "sad" << sad << std::endl;
        		std::cout << "sad_min" << sad_min << std::endl;
			}
		}
	}

	// 第二段階
	Mat src1_2 = pyr(src, 1);  // ２分の１の入力画像
	Mat tmp1_2 = pyr(tmp, 1);  // ２分の１のテンプレート画像
	sad_min = INT_MAX;
	int yyy = sad_y * 2;  // 画像が２倍の大きさになるので２倍
	int xxx = sad_x * 2;
	// テンプレート画像サイズの周囲±１ピクセルの範囲を探索
	for (int y = yyy - 1; y <= yyy + 1 && y < src1_2.rows - tmp1_2.rows; y++){
		for (int x = xxx - 1; x <= xxx + 1 && x < src1_2.cols - tmp1_2.cols; x++){
			int sad = 0;
			for (int j = 0; j < tmp1_2.rows; j++){
				for (int i = 0; i < tmp1_2.cols; i++){
					sad += abs((int)src1_2.at<uchar>(y + j, x + i) - (int)tmp1_2.at<uchar>(j, i));
					if (sad > sad_min) goto SSDA_loop2;
				}
			}
			sad_min = sad;
			sad_x = x;
			sad_y = y;
			std::cout << "2sad_min=" << sad_min << std::endl;
        	std::cout << "2sad_x=" << sad_x << std::endl;
        	std::cout << "2sad_y=" << sad_y << std::endl;
		SSDA_loop2:;
		}
	}

	// 第三段階
	Mat src1_1 = src.clone();  // 元画像と同じ大きさの入力画像
	Mat tmp1_1 = tmp.clone();  // 元画像と同じ大きさのテンプレート画像
	sad_min = INT_MAX;
	int yy = sad_y * 2;  // 画像が２倍の大きさになるので２倍
	int xx = sad_x * 2;
	// テンプレート画像サイズの周囲±１ピクセルの範囲を探索
	for (int y = yy - 1; y <= yy + 1 && y < src1_1.rows - tmp1_1.rows; y++){
		for (int x = xx - 1; x <= xx + 1 && x < src1_1.cols - tmp1_1.cols; x++){
			int sad = 0;
			for (int j = 0; j < tmp1_1.rows; j++){
				for (int i = 0; i < tmp1_1.cols; i++){
					sad += abs((int)src1_1.at<uchar>(y + j, x + i) - (int)tmp1_1.at<uchar>(j, i));
					if (sad > sad_min) goto SSDA_loop3;
				}
			}
			sad_min = sad;
			sad_x = x;
			sad_y = y;
			std::cout << "3sad_min=" << sad_min << std::endl;
        	std::cout << "3sad_x=" << sad_x << std::endl;
        	std::cout << "3sad_y=" << sad_y << std::endl;
		SSDA_loop3:;
		}
	}

	// テンプレートマッチング位置を四角で囲む
	Mat rect = src.clone();
	rectangle(rect, Point(sad_x, sad_y), Point(sad_x + tmp.cols, sad_y + tmp.rows), Scalar(0, 0, 200), 3, 4);
	namedWindow("Rectangle", CV_WINDOW_AUTOSIZE);  // ウィンドウ表示の準備
	imshow("Rectangle", rect);  // ウィンドウで表示
	imwrite("build.jpg", rect);  // 画像として出力

	cout << "３段階SAD法" << endl;
	// マッチした座標（左上）とそのときのsad値を表示
	cout << "(" << sad_x << ", " << sad_y << ") " << sad_min << endl;
}



//メイン関数
int main(int argc,char **argv){

	ros::init(argc,argv,"marker2");//rosを初期化
	ros::NodeHandle nhSub;//ノードハンドル
  if(kaisu!=0){ros_begin = ros::Time::now();}
	//subscriber関連
  //Realsensesの時(roslaunch realsense2_camera rs_camera.launch align_depth:=true)(Depth修正版なのでこっちを使うこと)
	message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nhSub, "/camera/color/image_raw", 1);//センサーメッセージを使うときは対応したヘッダーが必要
	message_filters::Subscriber<sensor_msgs::Image> depth_sub(nhSub, "/camera/aligned_depth_to_color/image_raw", 1);
	message_filters::Subscriber<sensor_msgs::CameraInfo> info_sub(nhSub, "/camera/color/camera_info", 1);

  	typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,sensor_msgs::Image,sensor_msgs::CameraInfo> MySyncPolicy;	
	message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10),rgb_sub, depth_sub, info_sub);
	sync.registerCallback(boost::bind(&callback,_1, _2, _3));
  
	ros::spin();//トピック更新待機
			
	return 0;
}



