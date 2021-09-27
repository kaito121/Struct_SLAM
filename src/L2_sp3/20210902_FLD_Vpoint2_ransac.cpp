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


std::string win_src = "src";
std::string win_fld = "fld";
std::string win_point = "point";

using namespace std;
using namespace cv;
using Eigen::MatrixXd;
#define UNKNOWNS 2  //未知数の数


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
    double kansuA[lines_std.size()],kansuB[lines_std.size()];//RANSAC用一次関数作成
    float inliers[lines_std.size()][2];//正解の点群リスト


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
        //N(0,lines_std_count)=lines_std[i][0];//直線の端点の座標x
        //N(1,lines_std_count)=lines_std[i][1];//直線の端点の座標y
        //std::cout <<"直線の端点の座標x_N(0,"<<lines_std_count<<")="<< N(0,lines_std_count) << std::endl;
        //std::cout <<"直線の端点の座標y_N(1,"<<lines_std_count<<")="<< N(1,lines_std_count) << std::endl;
        //PC(0,lines_std_count)=lines_std[i][2];//消失点真値(x座標)_観測データ=真値±ノイズ
        //PC(1,lines_std_count)=lines_std[i][3];//消失点真値(y座標)
        //std::cout <<"消失点真値(x座標)_PC(0,"<<lines_std_count<<")="<< PC(0,lines_std_count) << std::endl;
        //std::cout <<"消失点真値(y座標)_PC(1,"<<lines_std_count<<")="<< PC(1,lines_std_count) << std::endl;

        //RANSAC用一次関数作成
        kansuA[lines_std_count]=(lines_std[i][1]-lines_std[i][3])/(lines_std[i][0]-lines_std[i][2]);
        kansuB[lines_std_count]=lines_std[i][1]-(kansuA[lines_std_count]*lines_std[i][0]);

        lines_std_count=lines_std_count+1;//ナナメの線の数をカウント

	}
  //RANSACを利用した消失点推定
  //RANSAC_y_Ax_B

	float samplePointA[2],samplePointB[2];
	double a[UNKNOWNS][UNKNOWNS+1];
	double answers[UNKNOWNS];
	int threshold=10; //正解とみなす値の範囲
	double probA=0;
	double probB=0;
	int	score=0;
	int rand1=0,rand2=0;	
	int localScore;
	cout << "lines_std_count=" << lines_std_count << endl;


	for (int i=0;i<lines_std_count;i++)
	{
    rand1=rand()%lines_std_count;
		rand2=rand()%lines_std_count;
    while (rand1 == rand2){
      rand1=rand()%lines_std_count;
		  rand2=rand()%lines_std_count;
    }

		cout << "rand1=" << rand1 << endl;
		cout << "rand2=" << rand2 << endl;

		//samplePointA[0]=kansuA[rand1];//A1
		//samplePointA[1]=kansuB[rand1];//B1
		//samplePointB[0]=kansuA[rand2];//A2
		//samplePointB[1]=kansuB[rand2];//B2

		//pointA.x *a + 1*b=pointA.y
		a[0][0]=kansuA[rand1];//A1
		a[0][1]=1;							
		a[0][2]=kansuB[rand1];//B1

		a[1][0]=kansuA[rand2];//A2
		a[1][1]=1;
		a[1][2]=kansuB[rand2];//B2
		cout << "a[0][0]=" << -a[0][0] << endl;
		cout << "a[0][2]=" << a[0][2] << endl;
		cout << "a[1][0]=" << -a[1][0] << endl;
		cout << "a[1][2]=" << a[1][2] << endl;


		// 2変数の連立1次方程式(xとyを求める)
	
		// | -A1   1 | | x |   | B1 |
		// |         | |   | = |    |
		// | -A2   1 | | y |   | B2 |

		cv::Mat lhand = (cv::Mat_<float>(2, 2) << -a[0][0], 1, -a[1][0], 1);
		cv::Mat rhand = (cv::Mat_<float>(2, 1) << a[0][2], a[1][2]);
		cv::Mat ans;
	
		// 解を求める
		cv::solve(lhand, rhand, ans);
		cout << "x=" << ans.at<float>(0, 0) << endl;
		cout << "y=" << ans.at<float>(1, 0) << endl << endl;

		localScore=0;
		float samplePoint[2];
		for(int j=0;j<lines_std_count;j++){
			double calcY;
			samplePoint[0]=kansuA[j];//A
			samplePoint[1]=kansuB[j];//B
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
	for(int j=0;j<lines_std_count;j++){
		double calcY=probA*kansuA[j]+probB;
			if(abs(calcY-kansuB[j]) < threshold){
				//正解の値のみ、リストに追加
				inliers[inlierIndex][0]=kansuA[j];
				inliers[inlierIndex][1]=kansuB[j];
				inlierIndex++;
			}
	}
	//直線近似
	//今回はｘの合計÷ｙの合計とした
  //
	
	//a=vx/vy
	//b=pointA.y-pointA.x *a
	//
	double va=0;
	double vb=0;
	int counts=0;

	double vx=0;
	double vy=0;

	for(int i=0;i<inlierIndex-1;i++){
		a[0][0]=inliers[i][0];//A1
		a[0][1]=1;					
		a[0][2]=inliers[i][1];//B1
		a[1][0]=inliers[i+1][0];//A2
		a[1][1]=1;
		a[1][2]=inliers[i+1][1];//B2

		// 2変数の連立1次方程式(xとyを求める)
	
		// | -A1   1 | | x |   | B1 |
		// |         | |   | = |    |
		// | -A2   1 | | y |   | B2 |
		cv::Mat lhand = (cv::Mat_<float>(2, 2) << -a[0][0], 1, -a[1][0], 1);
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
	cv::circle(img_fld, cv::Point(RANSAC_A,RANSAC_B), 4, Scalar(0,0,255), -1, cv::LINE_AA);


	//サンプルの点を描画　//Draw  all sample points.
	//for( int i=0;i<lines_std_count;i++){
	//	cv::circle(img_fld, cv::Point(kansuA[i],kansuB[i]), 4, Scalar(150, 150, 150), -1, cv::LINE_AA);
	//}
	////サンプルの点を描画　//Draw  all sample points.
	//for( int i=0;i<inlierIndex;i++){
	//	cv::circle(img_fld, cv::Point(inliers[i][0],inliers[i][1]), 4, Scalar(255, 255, 0), -1, cv::LINE_AA);
	//}
    //RANSACで求めた消失点
		//cv::circle(img_fld, cv::Point(probA,probB), 4, Scalar(0,255,0), -1, cv::LINE_AA);




   /*//最小二乗法を利用した消失点推定
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

    cv::circle(img_fld,Point(X(0,0),X(1,0)),8,Scalar(0,0,255),-1);*/




  
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