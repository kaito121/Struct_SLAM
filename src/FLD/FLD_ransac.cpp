//y=ax+bのaとbをRANSACを使って求める。
//入力：測定した点(x,y)の配列

/*
ＲＡＮＳＡＣのアルゴリズム
１．総データ個数がU個あるデータから、ランダムでn個のデータを取り出します。
２．取り出したn個のデータから、パラメータを求めます。
  （取り出すデータの個数ですが、これは求めるべきパラメータの数に比例するようです)
３．求めたパラメータを、総データ点数から取り出したn個のデータを除いたものに式を当てはめ、
観測されたデータと２．で求めたパラメータの誤差を計算します。
４．誤差が許容範囲内であれば、パラメータに対して投票を行います。
５．１～４を繰り返し、投票数が一番多かったパラメータをひとまず採用します。
（これを仮パラメータとします）

６．仮パラメータを使ってすべてのデータに再度式を適用し、誤差が許容範囲内のものを抽出します。（この時点で雑音データがほぼ取り除かれます）
７．抽出したデータを元に、再度パラメータを求めます。
８．求まったパラメータがこのデータのもっともらしいパラメータとなります。

*/
#define _CRT_SECURE_NO_WARNINGS
#include <ros/ros.h>
#include <iostream>
#include <string>
#include <vector>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/ximgproc/fast_line_detector.hpp>//FLD

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
std::string win_point = "point";
std::string win_fld = "FLD";

using namespace std;
using namespace cv;

#define UNKNOWNS 4  //未知数の数

cv::Mat img_src,img_point,img_gray,img_fld;//画像定義
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

int XMax,cornerCount,inlierCount,outlierCount;
float corners[800][2];//すべての点群
float inliers[800][2];//正解の点群リスト


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
   
  	img_src = bridgeImage->image.clone();//image変数に変換した画像データを代入
  	depthimage = bridgedepthImage->image.clone();//image変数に変換した画像データを代入
	
	//ラインだけの画像を作るために単色で塗りつぶした画像を用意する
	img_point = img_src.clone();
    img_point = cv::Scalar(255,255,255);
	img_fld = img_src.clone();
    img_fld = cv::Scalar(255,255,255);


	cv::cvtColor(img_src, img_gray, cv::COLOR_RGB2GRAY);
	//FLD変換
    std::vector<cv::Vec4f> lines;
    cv::Ptr<cv::ximgproc::FastLineDetector> fld =  cv::ximgproc::createFastLineDetector();//特徴線クラスオブジェクトを作成
    fld->detect( img_gray, lines);//特徴線検索

    //FLDの線描写
    for(int i = 0; i < lines.size(); i++){
       cv::line(img_fld,cv::Point(lines[i][0],lines[i][1]),cv::Point(lines[i][2],lines[i][3]),cv::Scalar(0,0,255), 1.5, cv::LINE_AA);
	   cv::circle(img_fld, cv::Point(lines[i][0],lines[i][1]), 3, Scalar(0, 255, 0), -1, cv::LINE_AA);
	   cv::circle(img_fld, cv::Point(lines[i][2],lines[i][3]), 3, Scalar(0, 255, 0), -1, cv::LINE_AA);
    }
	int countn=0;
    for(int i = 0; i < lines.size(); i++){
		corners[countn][0]=lines[i][0];
		corners[countn+1][0]=lines[i][2];
		corners[countn][1]=lines[i][1];
		corners[countn+1][1]=lines[i][3];
		countn=countn+2;
	}

    std::cout <<"lines.size()="<<lines.size()<<std::endl;


	cornerCount=countn-1;

	//RANCACアルゴリズム-------------------------------------------------------------------------------------------------------------------------

	/*XMax=400;
	cornerCount=800;
	inlierCount=400;
	outlierCount=cornerCount-inlierCount;	

	//正解の値を生成  //create inliers
	for ( int n=0;n<400;n++)
	{
		double a=1.2;
		double b=20;		

		int randomnoise=(rand()%10)-(rand()%10);    //-10<= randomnoise <=10;

		corners[n][0]=n;
		corners[n][1]=a * n + b + randomnoise;
		cv::circle(img_point, cv::Point(corners[n][0],corners[n][1]), 4, Scalar(255, 0, 0), -1, cv::LINE_AA);

	}

	//不正解の値をランダムで生成  //create outliers	
	for ( int n=inlierCount;n<cornerCount;n++)
	{
		double r=rand()%XMax*1.5;
		corners[n][0]=r*1.1;
		r=rand()%XMax;
		corners[n][1]=r*1.2;
		cv::circle(img_point, cv::Point(corners[n][0],corners[n][1]), 4, Scalar(255, 0, 0), -1, cv::LINE_AA);
	}*/

	//RANSAC_y_Ax_B

	float samplePointA[2],samplePointB[2];
	double a[UNKNOWNS][UNKNOWNS+1];
	double answers[UNKNOWNS];
	int threshold=10; //正解とみなす値の範囲
	double probA=0;
	double probB=0;
	int	score=0;
	int rand1,rand2;	
	int localScore;

	for (int i=0;i<cornerCount;i++)
	{
		rand1=rand()%cornerCount;
		rand2=rand()%cornerCount;
		samplePointA[0]=corners[rand1][0];
		samplePointA[1]=corners[rand1][1];
		samplePointB[0]=corners[rand2][0];
		samplePointB[1]=corners[rand2][1];

				//pointA.x *a + 1*b=pointA.y
		a[0][0]=samplePointA[0];//x1
		a[0][1]=1;							
		a[0][2]=samplePointA[1];//y1

		a[1][0]=samplePointB[0];//x2
		a[1][1]=1;
		a[1][2]=samplePointB[1];//y2

		// 2変数の連立1次方程式(aとbを求める)
	
		// | x1   1 | | a |   | y1 |
		// |        | |   | = |    |
		// | x2   1 | | b |   | y2 |

		cv::Mat lhand = (cv::Mat_<float>(2, 2) << a[0][0], 1, a[1][0], 1);
		cv::Mat rhand = (cv::Mat_<float>(2, 1) << a[0][2], a[1][2]);
		cv::Mat ans;
	
		// 解を求める
		cv::solve(lhand, rhand, ans);
		//cout << "a=" << ans.at<float>(0, 0) << endl;
		//cout << "b=" << ans.at<float>(1, 0) << endl << endl;

		localScore=0;
		float samplePoint[2];
		for(int j=0;j<cornerCount;j++){
			double calcY;
			samplePoint[0]=corners[j][0];
			samplePoint[1]=corners[j][1];
			calcY=ans.at<float>(0, 0)*samplePoint[0]+ans.at<float>(1, 0);
			if(samplePoint[1]!=0){
				if(abs(calcY-samplePoint[1])<threshold){		
					localScore++;
				}
			}
		}
		if(localScore>score){
			//仮パラメータをアップデート
			score=localScore;
			probA=ans.at<float>(0, 0);
			probB=ans.at<float>(1, 0);
		}
	}
	cout << "probA=" <<probA<<",probB="<<probB<< endl << endl;

	//仮説の検証		(Verification Stage)
	//正解の点のリストを作成する
	int	 inlierIndex=0;
	for( int n=0;n<score;n++){
		//initialize;
		inliers[n][0]=0;
		inliers[n][1]=0;
	}
	for(int j=0;j<cornerCount;j++){
		double calcY=probA*corners[j][0]+probB;
			if(abs(calcY-corners[j][1]) < threshold){
				//正解の値のみ、リストに追加
				inliers[inlierIndex][0]=(int)corners[j][0];
				inliers[inlierIndex][1]=(int)corners[j][1];
				inlierIndex++;
			}
	}
	//直線近似
	//今回はｘの合計÷ｙの合計とした
	
	//a=vx/vy
	//b=pointA.y-pointA.x *a
	//
	double va=0;
	double vb=0;
	int counts=0;

	double vx=0;
	double vy=0;

	for(int i=0;i<inlierIndex-1;i++){
		a[0][0]=inliers[i][0];//x1
		a[0][1]=1;					
		a[0][2]=inliers[i][1];//y1
		a[1][0]=inliers[i+1][0];//x2
		a[1][1]=1;
		a[1][2]=inliers[i+1][1];//y2

		// 2変数の連立1次方程式(aとbを求める)
	
		// | x1   1 | | a |   | y1 |
		// |        | |   | = |    |
		// | x2   1 | | b |   | y2 |
		cv::Mat lhand = (cv::Mat_<float>(2, 2) << a[0][0], 1, a[1][0], 1);
		cv::Mat rhand = (cv::Mat_<float>(2, 1) << a[0][2], a[1][2]);
		cv::Mat ans;
	
		// 解を求める
		cv::solve(lhand, rhand, ans);
		//cout << "a=" << ans.at<float>(0, 0) << endl;
		//cout << "b=" << ans.at<float>(1, 0) << endl << endl;
		va=va+ ans.at<float>(0, 0);
		vb=vb+ ans.at<float>(1, 0);
		counts++;
	}
	double RANSAC_A=va/counts;
	double RANSAC_B=vb/counts;
	cout << "RANSAC_A=" << RANSAC_A << endl;
	cout << "RANSAC_B=" << RANSAC_B << endl << endl;

	//サンプルの点を描画　//Draw  all sample points.
	for( int i=0;i<cornerCount;i++){
		cv::circle(img_point, cv::Point(corners[i][0],corners[i][1]), 4, Scalar(150, 150, 150), -1, cv::LINE_AA);
	}
	//サンプルの点を描画　//Draw  all sample points.
	for( int i=0;i<inlierIndex;i++){
		cv::circle(img_point, cv::Point(inliers[i][0],inliers[i][1]), 4, Scalar(255, 255, 0), -1, cv::LINE_AA);
	}

	//RANSACで求めた直線を引く
	//param from Hypothesis(仮説からのパラメタ)
    cv::line(img_point,cv::Point(0,probB),cv::Point(400,400*probA+probB),cv::Scalar(0,255,0), 1, cv::LINE_AA);
	//param from Verification
    cv::line(img_point,cv::Point(0,RANSAC_B),cv::Point(400,400*RANSAC_A+RANSAC_B),cv::Scalar(0,0,255), 1, cv::LINE_AA);






  
    // 画面表示
    cv::imshow(win_src, img_src);
    cv::imshow(win_point, img_point);
    cv::imshow(win_fld, img_fld);
	kaisu++;

    cv::waitKey(1);//ros::spinにジャンプする
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