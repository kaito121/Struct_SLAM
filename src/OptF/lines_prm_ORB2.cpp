//Depth調整バージョン(D435_test.cppを参照)
//rosのヘッダ
//20210601
//カメラをつかってリアルタイムで物体の追跡が可能プログラム
//しかし追跡はできるがマッチングはしていないので、配列はバラバラに格納されている
//そのためオプティカルフローなどをする場合はこれにマッチングのプログラムを導入する必要がある
//20210602
//Ver1では特徴点の更新プログラムが入っておらず画面から消えたら特徴点が消えてしまい追跡ができない
//そこで常に更新と追跡を行うように変更したい
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
// image_curr:  現在の入力画像、    image_prev:  直前の入力画像
// points_curr: 現在の特徴点リスト、points_prev: 直前の特徴点リスト
cv::Mat frame,image_curr, image_prev,img_dst;
vector<cv::Point2f> points_prev, points_curr,points_prev_not, points_curr_not;

cv::Mat_<double>World0,WorldC;//世界座標系設定
World0 = cv::Mat_<double>::zeros(3, 1);  // 世界座標系原点(零行列)
WorldC = cv::Mat_<double>::eye(3, 1);	  // 世界座標系でのカメラの座標(単位行列)

//カメラの内部パラメータ情報
cv::Mat_<double>A;//内部パラメータ（詳しくはM2 6月研究ノート)
A = (cv::Mat_<ushort>(3, 3) << 1.93/326.15115000485503, 0, 326.15115000485503, 0, 1.93/244.02271379203739, 244.02271379203739,0,0,1);

cv::Mat_<double>d0;//カメラ座標系と正規化画像座標系の変換に使用する行列
d0 = (cv::Mat_<ushort>(3, 4) << 1,0,0,0,0,1,0,0,0,0,1,0);



//コールバック関数
void callback(const sensor_msgs::Image::ConstPtr& rgb_msg,const sensor_msgs::Image::ConstPtr& depth_msg)
{
	//変数宣言
	cv_bridge::CvImagePtr bridgeImage;//クラス::型//cv_brigeは画像変換するとこ
  cv_bridge::CvImagePtr bridgedepthImage;//クラス::型//cv_brigeは画像変換するとこ
  cv::Mat RGBimage,depthimage,image;//opencvの画像
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

	image.copyTo(frame);//ここ間違えていたので注意
	image.copyTo(img_dst);//ここ間違えていたので注意

	cv::cvtColor(image, image_curr, cv::COLOR_BGR2GRAY);//グレースケール化
	
	if (reset == true) {
		std::cout <<"初回検出プログラム"<< std::endl;//最初のフレーム
		// 特徴点検出(グレースケール画像から特徴点検出)
		cv::goodFeaturesToTrack(image_curr, points_curr, 500, 0.01, 10, cv::Mat(), 3, 3, 0, 0.04);
		cv::cornerSubPix(image_curr, points_curr, cv::Size(10, 10), cv::Size(-1, -1), cv::TermCriteria(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, 20, 0.03));
		points_prev = points_curr;//データのコピー
		for (int i = 0; i < points_curr.size(); i++) {
			cv::circle(img_dst, points_curr[i], 8, Scalar(255,0,0), -1, cv::LINE_AA);
			}
		reset = false;//if文切り替え
	} 
	else {
		std::cout <<"二回目オプティカルフロー"<< std::endl;// 特徴点追跡(二回目のフレーム)
		vector<uchar> status;//特徴点の数
		vector<float> err;

		cv::calcOpticalFlowPyrLK(image_prev, image_curr, points_prev, points_curr, status, err);//オプティカルフロー
		std::cout <<"status.size()="<<status.size()<< std::endl;//特徴点の個数

		// 追跡できなかった特徴点をリストから削除する
		int i, k,n;
		req=1;
		for (i = k = n =0; i < status.size(); i++)
		{
			std::cout <<"status["<<i<<"]="<<status[i]<< std::endl;//0以外の時は変なマークがでる
			//ここで検索出来なかったものを消し探索の範囲を狭めている(statusが0の時)
			//statusが0以外の時値を更新する
			if (status[i] != 0) {	
			points_prev[k]   = points_prev[i];
			points_curr[k++] = points_curr[i];
			std::cout <<"status["<<i<<"]がゼロ以外の時"<< std::endl;//0以外の時は変なマークがでる
			}
		}
		std::cout <<"k="<<k<< std::endl;
		points_curr.resize(k);//ここでkの個数でリサイズされている
		points_prev.resize(k);
		//特徴点が100個以下になったら再び特徴点を検出する
		if(status.size()<150){reset = true;}
	}

	// 特徴点を丸で描く
	for (int i = 0; i < points_curr.size(); i++) {
		cv::Scalar c(0, 255, 0);//緑
		if (cv::norm(points_prev[i] - points_curr[i]) > 0.5) {
			c = cv::Scalar(0, 100, 255);//今の特徴点と一つ前の特徴点との差が0.5以上の時オレンジ
		}
		if(i==0){cv::circle(img_dst, points_curr[i], 8, Scalar(0,255,255), -1, cv::LINE_AA);}
		if(i==1){cv::circle(img_dst, points_curr[i], 8, Scalar(0,250,0), -1, cv::LINE_AA);}
		if(i==2){cv::circle(img_dst, points_curr[i], 8, Scalar(0,200,255), -1, cv::LINE_AA);}
		if(i==3){cv::circle(img_dst, points_curr[i], 8, Scalar(0,150,255), -1, cv::LINE_AA);}
		if(i==4){cv::circle(img_dst, points_curr[i], 8, Scalar(100,125,0), -1, cv::LINE_AA);}
		if(i==5){cv::circle(img_dst, points_curr[i], 8, Scalar(0,100,255), -1, cv::LINE_AA);}
		if(i==6){cv::circle(img_dst, points_curr[i], 8, Scalar(0,75,255), -1, cv::LINE_AA);}
		if(i==7){cv::circle(img_dst, points_curr[i], 8, Scalar(100,50,255), -1, cv::LINE_AA);}
		if(i==8){cv::circle(img_dst, points_curr[i], 8, Scalar(50,25,255), -1, cv::LINE_AA);}
		if(i==9){cv::circle(img_dst, points_curr[i], 8, Scalar(0,0,255), -1, cv::LINE_AA);}
		if(i==10){cv::circle(img_dst, points_curr[i], 8, Scalar(100,255,200), -1, cv::LINE_AA);}
		else{
			cv::circle(img_dst, points_curr[i], 3, c, -1, cv::LINE_AA);//今の座標情報
			cv::circle(img_dst, points_prev[i], 3, Scalar(255,255,255), -1, cv::LINE_AA);//一つ前の画像の座標
			cv::line(img_dst,cv::Point(points_curr[i]),cv::Point(points_prev[i]),cv::Scalar(0,255,255), 1, cv::LINE_AA);
			}
		
		std::cout <<"points_curr["<<i<<"]="<<points_curr[i]<< std::endl;//特徴点の座標
		std::cout <<"points_prev["<<i<<"]="<<points_prev[i]<< std::endl;//特徴点の座標

    std::cout <<"画像座標["<<i<<"]="<<points_curr[i]<< std::endl;//特徴点の座標
		std::cout <<"画像座標["<<i-1<<"]="<<points_prev[i]<< std::endl;//特徴点の座標
 
	}

		cv::imshow("win_dst", img_dst);
		cv::imshow("win_curr", image_curr);//今の画像

		if (req == 1) {//初回は直前の画像がないため考慮
		cv::imshow("win_prev", image_prev);//一つ前の画像像
		std::cout <<"でてるよ"<< std::endl;//特徴点の座標
		}

  // 行列表示
  std::cout << "世界座標原点:World0=\n" << World0 << std::endl; 
  std::cout << "カメラの座標(世界座標系):WorldC=\n" << WorldC << std::endl; 

  //---------------------------------------------------------------------------------------------

  // (3) マッチング結果から，F行列を推定する
	cv::Mat F = cv::findFundamentalMat(points_curr, points_prev);
	std::cout << "F=" << F << std::endl;

  // (4) カメラの内部パラメータが既知の場合はE行列を計算し，外部パラメータを推定する
	// カメラ内部パラメータ読み込み
	cv::Mat V;
	cv::FileStorage fs;
  fs.open("realsense_para.xml", cv::FileStorage::READ);
	fs["intrinsic"]>>V;
	std::cout << "V=" << V << std::endl;
	
	// E行列の計算
	cv::Mat E = cv::findEssentialMat(points_curr, points_prev, V);
	
	// 外部パラメータ（回転，並進ベクトル）の計算
	cv::Mat R, t;
	cv::recoverPose(E, points_curr, points_prev, V, R, t);

	std::cout << "E=" << E << std::endl;
	std::cout << "R=" << R << std::endl;
	std::cout << "t=" << t << std::endl;

	//行列の要素抜き出し（数値)
  std::cout << "R(0,0)=" << R.at<double>(0, 0) << std::endl << std::endl;

   //配列の列の追加方法(斉次化してないバージョン)
  //cv::Mat_<double> Rt_ = cv::Mat_<double>(3, 4);        
  //Rt_ << R.at<double>(0, 0), R.at<double>(0, 1), R.at<double>(0, 2), t.at<double>(0, 0), R.at<double>(1, 0), R.at<double>(1, 1), R.at<double>(1, 2),t.at<double>(1, 0),R.at<double>(2, 0),R.at<double>(2, 1),R.at<double>(2, 2),t.at<double>(2, 0);
  //cv::Mat Rt = Rt_;
  //std::cout << "Rt=" << Rt << std::endl << std::endl;

  //配列の列の追加方法(斉次化バージョン)
  cv::Mat_<double> Rt_ = cv::Mat_<double>(4, 4);        
  Rt_ << R.at<double>(0, 0), R.at<double>(0, 1), R.at<double>(0, 2), t.at<double>(0, 0), R.at<double>(1, 0), R.at<double>(1, 1), R.at<double>(1, 2),t.at<double>(1, 0),R.at<double>(2, 0),R.at<double>(2, 1),R.at<double>(2, 2),t.at<double>(2, 0),0,0,0,1;
  cv::Mat Rt = Rt_;
  std::cout << "Rt=" << Rt << std::endl << std::endl; 

   /*//座標変換
  //Eigenの定義
    C1z_E(0,0)=0,C1z_E(1,0)=0,C1z_E(2,0)=A[j][i][0][2]-1;//-1してるのは距離情報を追加する際元データにある１を消すため
    C2z_E(0,0)=0,C2z_E(1,0)=0,C2z_E(2,0)=A[j][i][1][2]-1;
    cv::Mat C1z_M = (cv::Mat_<double>(3,1) << 0, 0, A[j][i][0][2]-1);
    cv::Mat C2z_M = (cv::Mat_<double>(3,1) << 0, 0, A[j][i][1][2]-1);
    std::cout << "C1z_M=" << C1z_M << std::endl << std::endl;
    std::cout << "C1z_M=" << C2z_M << std::endl << std::endl;
    
    //座標変換
    //画像座標系の斉次化
    cv::Mat_<double> u1_ = cv::Mat_<double>(3, 1);        
    u1_ << A[j][i][0][0], A[j][i][0][1], 1;//画像座標系（u1,v1,1)※斉次化
    cv::Mat u1 = u1_;
    cv::Mat u1_1 = u1_;//Eigen変換用（変換用を別に用意しないとエラーが発生する）
    std::cout << "u1=" << u1 << std::endl << std::endl;
    cv::cv2eigen(u1_1,u1_E);//cvMatからEigenに座標変換
    std::cout << "u1_E=" << u1_E << std::endl;
    cv::Mat_<double> u2_ = cv::Mat_<double>(3, 1);        
    u2_ << A[j][i][1][0], A[j][i][1][1], 1;//画像座標系（u2,v2,1)※斉次化
    cv::Mat u2 = u2_;
    std::cout << "u2=" << u2 << std::endl << std::endl;
    // 逆行列を求める(内部パラメータ)
	  cv::Mat Vinv = V.inv();
	  std::cout << "Vinv=\n" << Vinv << std::endl;
    // 逆行列を求める(外部パラメータ)
	  cv::Mat Rtinv = Rt.inv();
	  std::cout << "Rtinv=\n" << Rtinv << std::endl;
    //座標変換画像座標→カメラ座標
    cv::Mat C1 = Vinv * u1;
    cv::Mat C2 = Vinv * u2;
    std::cout << "Second=" << sec << std::endl;
    std::cout << "C1["<<j<<"]["<<i<<"]=\n" << C1 << std::endl;
    std::cout << "C2["<<j<<"]["<<i<<"]=\n" << C2 << std::endl;
    cv::Mat C1_1 = C1;//Eigen変換用（変換用を別に用意しないとエラーが発生する）
    cv::Mat C2_1 = C2;//Eigen変換用（変換用を別に用意しないとエラーが発生する）
    cv::cv2eigen(C1_1,C1_E);//cvMatからEigenに座標変換
    cv::cv2eigen(C2_1,C2_E);//cvMatからEigenに座標変換
    std::cout << "C1_E=\n" << C1_E << std::endl;
    std::cout << "C2_E=\n" << C2_E << std::endl;
    std::cout << "C1z_E=\n" << C1z_E << std::endl;//距離データを追加する()
    std::cout << "C2z_E=\n" << C2z_E << std::endl;
    std::cout << "C1_E+C1z_E=\n" << C1_E+C1z_E << std::endl;
    std::cout << "C2_E+C2z_E=\n" << C2_E+C2z_E << std::endl;
    std::cout << "C2_E(0,0)=\n" << C2_E(0,0) << std::endl;
    int nrow=3,ncol=0;
    double C1_d[nrow][ncol];
    //Map<RowMatrixXi>(&C1_d[0][0],nrow,ncol)=C1_E

    //C1_d[0]=C1_E(0,0);
    //C1_d[1]=C1_E(0,1);
    //C1_d[2]=C1_E(0,1);
    //std::cout << "C1_d1=\n" << C1_d[0][0] << std::endl;
    double value=20.3;
    
    C1_D[j][i][0][0]=C1_E(0,0),C1_D[j][i][1][0]=C1_E(1,0),C1_D[j][i][2][0]=C1_E(2,0);
    C2_D[j][i][0][0]=C2_E(0,0),C2_D[j][i][1][0]=C2_E(1,0),C2_D[j][i][2][0]=C2_E(2,0);
    C1 = C1+C1z_M;
    C2 = C2+C2z_M;
    //std::cout << "C2_D[0][0]=\n" << C2_D[0][0] << std::endl;
    
    //sprintf_s(C1_c,"C1[%d][%d]=(%f,%f,%f)",j,i,C1_D[0][0],C1_D[1][0],C1_D[2][0]);
    //sprintf_s(C2_c,"C2[%d][%d]=(%f,%f,%f)",j,i,C2_D[0][0],C2_D[1][0],C2_D[2][0]);
    //sprintf(C2_c,"C2[%d][%d]=(%f,%f,%f)\n",j,i,C2_E(0,0),C2_E(0,1),C2_E(0,2));
    //sprintf(value_c,"value=%f,unti=%f",value,value);
    
    //putText(img_line, C1_c,Point(A[j][i][0][0],A[j][i][0][1]),0,0.5,Scalar(0,0,0),1,CV_AA);
    //putText(img_line, C2_c,Point(A[j][i][1][0],A[j][i][1][1]),0,0.5,Scalar(0,0,0),1,CV_AA);
    //putText(img_line, value_c,Point(20,100),0,0.5,Scalar(0,0,255),1,CV_AA);
    putText(img_line,"Second="+std::to_string(sec),Point(10,10),0,0.5,Scalar(0,0,0),1);
    putText(img_line,"C1[j="+std::to_string(j),Point(A[j][i][0][0],A[j][i][0][1]),0,0.5,Scalar(0,0,0),1);
    putText(img_line,"C1[i="+std::to_string(i),Point(A[j][i][0][0],A[j][i][0][1]+30),0,0.5,Scalar(0,0,0),1);
    putText(img_line,"C2[j="+std::to_string(j),Point(A[j][i][1][0],A[j][i][1][1]),0,0.5,Scalar(0,0,0),1);
    putText(img_line,"C2[i="+std::to_string(i),Point(A[j][i][1][0],A[j][i][1][1]+30),0,0.5,Scalar(0,0,0),1);
    
    //カメラ座標系を斉次化
    C1.resize(4);//配列の行要素を増やす（でも０しか入れられない）
    C2.resize(4);
    cv::Mat n0 = (cv::Mat_<double>(4,1)<<0,0,0,1);
    C1 = C1+n0;//拡張した行列に1を足すことで斉次化
    C2 = C2+n0;
    std::cout << "C1_2=\n" << C1 << std::endl;
    std::cout << "C2_2=\n" << C2 << std::endl;
    //座標変換カメラ座標→世界座標系
    cv::Mat W1 = Rtinv * C1;
    cv::Mat W2 = Rtinv * C2;
    std::cout << "W1=\n" << W1 << std::endl;
    std::cout << "W2=\n" << W2 << std::endl;*/


		int key = cv::waitKey(30);
		if (key == 'r') {reset = true;}// Rキーが押されたら特徴点を再検出
		
		cv::swap(image_curr, image_prev);// image_curr を image_prev に移す（交換する）
		cv::swap(points_curr, points_prev);// points_curr を points_prev に移す（交換する）
 
	cv::waitKey(1);//ros::spinにジャンプする
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