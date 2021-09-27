#define _CRT_SECURE_NO_WARNINGS
#include <ros/ros.h>
#include <iostream>
#include <string>
#include <vector>
#include <opencv2/opencv.hpp>
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
#include <Eigen/Dense>
#include<fstream>//ファイル出力


std::string win_src = "src";
std::string win_fld = "fld";
std::string win_point = "point";

using namespace std;
using namespace cv;
using Eigen::MatrixXd;

cv::Mat image, img, gray,img_fld;
cv::Mat frame;
vector< vector<int> > points;

cv::Mat prevRes;
cv::Mat res, aug, error;
cv::Mat_<float>Atemp, btemp;
//ofstream out1, out2;
float epsilon;

//store slope (m) and y-intercept (c) of each lines
//(各直線の傾き(m)とY字型切片(c)を格納する。)
float m,c;

//store minimum length for lines to be considered while estimating vanishing point
//消失点を推定する際に考慮すべき線の最小長さを保存する。
int minlength;

//temporary vector for intermediate storage
//(中間貯蔵のための一時的なベクトル)
vector<int> temp;

//store (x1, y1) and (x2, y2) endpoints for each line segment
//(各線分の端点(x1, y1)と(x2, y2)を格納する)
vector<cv::Vec4i> lines_std;

//video capture object from OpenCV
cv::VideoCapture cap;

//to store intermediate errors(中間的なエラーを格納する)
double temperr;

sensor_msgs::CameraInfo camera_info;//CameraInfo受け取り用
int kaisu;


ros::Time ros_begin;//プログラム時間
ros::WallTime wall_begin = ros::WallTime::now();//プログラム開始時の時間
ros::WallDuration wall_prev;//一つ前の時間
ros::WallDuration wall_systemtime;//サンプリング時間
//ros::Time ros_begin = ros::Time::now();
struct timeval startTime, endTime;  // 構造体宣言
float realsec;//サンプリング時間（C++)

ofstream outputfile1("/home/fuji/catkin_ws/src/Struct_SLAM/src/L2_sp3/X(0,0).txt");//txtファイルに記録
ofstream outputfile2("/home/fuji/catkin_ws/src/Struct_SLAM/src/L2_sp3/X(1,0).txt");//txtファイルに記録

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

    image.copyTo(img_fld);

    // define minimum length requirement for any line(あらゆるラインの最小長さの要件を定義する)
		minlength = image.cols * image.cols * 0.001 ;

		cv::cvtColor(image,image , cv::COLOR_BGR2GRAY);

    ////equalize histogram(ヒストグラムの均等化)
		//cv::equalizeHist(image, image);

		//initialize the line segment matrix in format y = m*x + c	
    //(y = m*x + c という形式で線分行列を初期化する。)

    //FLD変換
    cv::Ptr<cv::ximgproc::FastLineDetector> fld =  cv::ximgproc::createFastLineDetector();//特徴線クラスオブジェクトを作成
    
    //initialize
    lines_std.clear();
    
    fld->detect( image, lines_std);//特徴線検索

    // Show found lines
    cv::Mat drawnLines (image);
    double lines_std2[lines_std.size()][4],lines_std_lc[lines_std.size()],lines_std_thetal[lines_std.size()];
    MatrixXd N(2,lines_std.size());//Nは2行L列の行列(Eigen定義)
    MatrixXd PC(2,lines_std.size());//推定交点(Eigen定義)
    int lines_std_count=0;



    for(int i=0; i<lines_std.size(); i++)
	{
	    //ignore if almost vertical(ほぼ垂直の場合は無視)
	    if ( abs(lines_std[i][0]-lines_std[i][2]) < 10 || abs(lines_std[i][1]-lines_std[i][3]) < 10) //check if almost vertical
				continue;
		//ignore shorter lines (x1-x2)^2 + (y2-y1)^2 < minlength(短い行を無視する (x1-x2)^2 + (y2-y1)^2 < minlength)
		if( ((lines_std[i][0]-lines_std[i][2])*(lines_std[i][0]-lines_std[i][2]) +(lines_std[i][1]-lines_std[i][3])*(lines_std[i][1]-lines_std[i][3])) < minlength)
			continue;

        //store valid lines' endpoints for calculations(有効なラインの端点を計算のために保存)
		for(int j=0; j<4; j++)
		{
			temp.push_back(lines_std[i][j]);
		}

		points.push_back(temp);
		temp.clear();
        cv::line(img_fld,cv::Point(lines_std[i][0],lines_std[i][1]),cv::Point(lines_std[i][2],lines_std[i][3]),cv::Scalar(0,0,255), 2, cv::LINE_AA); 

        //座標から一次関数を引く関数
        lines_std_thetal[i]=(M_PI/2)-(M_PI-atan2((lines_std[i][2]-lines_std[i][0]),(lines_std[i][3]-lines_std[i][1])));
        lines_std_lc[i]=(lines_std[i][2]-lines_std[i][0])*(lines_std[i][2]-lines_std[i][0])+(lines_std[i][3]-lines_std[i][1])*(lines_std[i][3]-lines_std[i][1]);
        lines_std2[i][0]=lines_std[i][0]+(cos(-lines_std_thetal[i])*sqrt(lines_std_lc[i]))*1000;//X1座標
        lines_std2[i][1]=lines_std[i][1]+(sin(-lines_std_thetal[i])*sqrt(lines_std_lc[i]))*1000;//Y1座標
        lines_std2[i][2]=lines_std[i][0]+(cos(-lines_std_thetal[i])*sqrt(lines_std_lc[i]))*-1000;//X2座標
        lines_std2[i][3]=lines_std[i][1]+(sin(-lines_std_thetal[i])*sqrt(lines_std_lc[i]))*-1000;//Y2座標

        cv::line(img_fld,cv::Point(lines_std2[i][0],lines_std2[i][1]),cv::Point(lines_std2[i][2],lines_std2[i][3]),cv::Scalar(0,255,0), 1, cv::LINE_AA);
        //直線の作成(消失点と任意の点を端点とした線を作成する)
        N(0,lines_std_count)=lines_std[i][0];//直線の端点の座標x
        N(1,lines_std_count)=lines_std[i][1];//直線の端点の座標y
        std::cout <<"直線の端点の座標x_N(0,"<<lines_std_count<<")="<< N(0,lines_std_count) << std::endl;
        std::cout <<"直線の端点の座標y_N(1,"<<lines_std_count<<")="<< N(1,lines_std_count) << std::endl;
        PC(0,lines_std_count)=lines_std[i][2];//消失点真値(x座標)_観測データ=真値±ノイズ
        PC(1,lines_std_count)=lines_std[i][3];//消失点真値(y座標)
        std::cout <<"消失点真値(x座標)_PC(0,"<<lines_std_count<<")="<< PC(0,lines_std_count) << std::endl;
        std::cout <<"消失点真値(y座標)_PC(1,"<<lines_std_count<<")="<< PC(1,lines_std_count) << std::endl;

        lines_std_count=lines_std_count+1;//ナナメの線の数をカウント


	}

    //法線ベクトルを求める
    //法線ベクトルは直線の90度なので90度回転させる
    //ただし90度回転させただけなので、そのベクトルを単位ベクトル化することで法線ベクトルを作る(M2 4月研究参照)
    MatrixXd R(2,2);//(Eigen定義)
    R(0,0)=cos(M_PI/2);
    R(0,1)=-sin(M_PI/2);
    R(1,0)=sin(M_PI/2);
    R(1,1)=cos(M_PI/2);

    //std::cout <<"回転行列R=\n"<< R << std::endl;

    MatrixXd n(2,lines_std_count);
    n=R*(PC-N);//直線を90度回転させたベクトルn
  
    //std::cout <<"直線を90度回転させたベクトルn=\n"<< n << std::endl;

    //法線ベクトルの大きさを１にする
    MatrixXd na(2,lines_std_count);
    MatrixXd na1(lines_std_count,0);
    na=n.transpose()*n;//na=n^T*n（ルートの中身を計算）
    na1=na.diagonal();//naを対角化することで要素の二乗の和を求める(例:a1^2+a2^2);
    //std::cout <<"na=\n"<< na << std::endl;
    //std::cout <<"naの対角行列=\n"<< na.diagonal() << std::endl;
    MatrixXd n1(lines_std_count,1);
    MatrixXd n2(2,lines_std_count);
    MatrixXd n22(lines_std_count,1);
    MatrixXd p0(lines_std_count,lines_std_count);
    MatrixXd n3(2,2);
    MatrixXd n30(lines_std_count,lines_std_count);
    MatrixXd n4(2,1);
    MatrixXd n40(2,1);
    MatrixXd X(2,1);
    MatrixXd X0(2,1);
    
    for(int t=0;lines_std_count>t;t++){
        n1(t,0)=sqrt(na1(t,0));//ベクトルnの大きさ（二乗の和に対しルートをかける）
        n2(0,t)=n(0,t)/n1(t,0);//法線ベクトル
        n2(1,t)=n(1,t)/n1(t,0);//法線ベクトル
        //cv::line(img_line3,cv::Point(PC(0,t),PC(1,t)),cv::Point((PC(0,t)-n2(0,t)*100),(PC(1,t)-n2(1,t)*100)),cv::Scalar(0,0,255), 4, cv::LINE_AA);//法線ベクトル(描写時に100倍してる）(赤の線)
    }

    //std::cout <<"n1=\n"<< n1 << std::endl;
    //std::cout <<"法線ベクトルn2=\n"<< n2 << std::endl;

    //最小二乗法の計算
    p0=n2.transpose()*N;//P0=nT*Nを計算//ここ要チェック------------------------------------------------
    n3=n2*n2.transpose();//逆行列の内部を計算

    //最小二乗法の計算
    n4=n2*p0.diagonal();//nにP0の対角行列をかけている 
    X=n3.inverse()*n4;//逆行列と法線ベクトルの転置を掛け算
    
    //std::cout <<"p0=\n"<< p0 << std::endl;
    //std::cout <<"n3=\n"<< n3 << std::endl;
    //std::cout <<"n3の逆行列=\n"<< n3.inverse() << std::endl;
    //std::cout <<"n4=\n"<< n4 << std::endl;
    //std::cout <<"p0.diagonal()=\n"<< p0.diagonal() << std::endl;
    std::cout <<"X=\n"<< X << std::endl;//推定交点

    cv::circle(img_fld,Point(X(0,0),X(1,0)),8,Scalar(0,0,255),-1);

    outputfile1<<X(0,0)<<"\n";
    outputfile2<<X(1,0)<<"\n";





  
    // 画面表示
    cv::imshow(win_src, image);
	cv::imshow(win_fld, img_fld);// Show found lines

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