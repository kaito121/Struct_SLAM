//Depth調整バージョン(D435_test.cppを参照)
//rosのヘッダ
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
#include <random>
#include <functional>


ros::Subscriber sub;//データをsubcribeする奴
std::string win_src = "src";//カメラ画像
std::string win_dst = "dst";//カメラ画像


using namespace std;
using namespace cv;
float Ts=0.1,d=0.2;
double kaisu;

cv::Mat F,H,u,W,V,Q,R,xDes,xAct,xEst,P,z,I,y,K,S;

cv::Mat img_dst;
 

//コールバック関数
void callback(const sensor_msgs::Image::ConstPtr& rgb_msg,const sensor_msgs::Image::ConstPtr& depth_msg)
{
	//変数宣言
	cv_bridge::CvImagePtr bridgeImage;//クラス::型//cv_brigeは画像変換するとこ
    cv_bridge::CvImagePtr bridgedepthImage;//クラス::型//cv_brigeは画像変換するとこ
    cv::Mat RGBimage;//opencvの画像
    cv::Mat depthimage;//opencvの画像
	cv::Mat image;//opencvの画像
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
    if(kaisu==0){
    img_dst = image.clone();//オプティカルフローの描写画面の用意
    img_dst = cv::Scalar(255,255,255);
  }

    std::random_device rnd;     // 非決定的な乱数生成器を生成
    std::mt19937 mt(rnd());     //  メルセンヌ・ツイスタの32ビット版、引数は初期シード値
    std::uniform_int_distribution<> rand50(0, 100);        // [0, 99] 範囲の一様乱数
    std::uniform_int_distribution<> rand100(0, 200);        // [0, 99] 範囲の一様乱数
    

    //std::normal_distribution<> normW(0.04, 0.04);// 平均0.04, 分散値0.04の正規分布
    //std::normal_distribution<> normV(0.09, 0.04);// 平均0.09, 分散値0.04の正規分布

    std::normal_distribution<> normW(0.1, 0.5);// 平均0.04, 分散値0.04の正規分布
    std::normal_distribution<> normV(0.2, 0.5);// 平均0.09, 分散値0.04の正規分布


//カルマンフィルタ初期設定---------------------------------------------------------------------------------------------------------
 if(kaisu==0){


  F = cv::Mat_<float>::eye(2, 2);//状態行列
  H = cv::Mat_<float>::eye(2, 2);//出力ベクトル
  u = (cv::Mat_<float>(2, 1) <<0.2,0.1);//入力

  W = (cv::Mat_<float>(2, 1) <<0.04,0.04);//入力ノイズ
  V = (cv::Mat_<float>(2, 1) <<0.09,0.09);//出力ノイズ

  Q = (cv::Mat_<float>(2,2) << //共分散Q
    0.04, 0,
    0, 0.04);

  R = (cv::Mat_<float>(2,2) << //共分散R
    0.09, 0, 
    0, 0.09);

  xDes = cv::Mat_<float>::zeros(2, 1);//目標指令状態
  xAct = cv::Mat_<float>::zeros(2, 1);//雑音の影響を考慮した実際の状態
  xEst = cv::Mat_<float>::zeros(2, 1);//推定状態

  z = cv::Mat_<float>::zeros(2, 1);//出力方程式


  P = cv::Mat_<float>::eye(2, 2);//誤差分散行列
  I = cv::Mat_<float>::eye(2, 2);//単位行列

  y = cv::Mat_<float>::zeros(2, 2);
  S = cv::Mat_<float>::zeros(2, 2);
  K = cv::Mat_<float>::zeros(2, 2);//カルマンゲイン



 }
std::cout <<"11111111111111111"<< std::endl;
 cv::Mat normWT = (cv::Mat_<float>(2,1) << normW(mt), normW(mt)); 
 cv::Mat normVT = (cv::Mat_<float>(2,1) << normV(mt), normV(mt)); 
std::cout <<"2222222222222222222222"<< std::endl;

    

//--------------------------------------------------------------------------------------------------------------------------

xDes =  xDes + u +normWT*0;
xAct =  xAct + u +normWT;

z= xAct + normVT;

std::cout <<"目標の状態xDes="<<xDes<< std::endl;
std::cout <<"実際の状態xAct="<<xAct<< std::endl;
std::cout <<"観測値z="<<z<< std::endl;



//予測ステップ-----------------------------------------------------------------------------------------------------

xEst = F * xEst + u;//一つ前のxEstから現在のロボットの状態xEstを推定
P = F*P*F.t()+Q;

std::cout <<"推定の状態xEst="<<xEst<< std::endl;
std::cout <<"誤差共分散P="<<P<< std::endl;


//更新ステップ-------------------------------------------------------------------------------------

y = z - H * xEst;
S = R +H * P * H.t();
K = P * H.t() / S;
xEst = xEst + K * y;
P = (I - K * H) * P;

std::cout <<"カルマンゲインK="<<K<< std::endl;
std::cout <<"推定の状態(更新)xEst="<<xEst<< std::endl;

cv::circle(img_dst, cv::Point(xEst.at<float>(0), xEst.at<float>(1)), 3, cv::Scalar(0, 255, 0), -1);//推定状態（緑
cv::circle(img_dst, cv::Point(xDes.at<float>(0), xDes.at<float>(1)), 3, cv::Scalar(255, 0, 0), -1);//目標指令状態（青
cv::circle(img_dst, cv::Point(xAct.at<float>(0), xAct.at<float>(1)), 3, cv::Scalar(0, 0, 255), -1);//雑音の影響を考慮した実際の状態（赤















    


    cv::namedWindow(win_src, cv::WINDOW_AUTOSIZE);
    cv::namedWindow(win_dst, cv::WINDOW_AUTOSIZE);

    cv::imshow(win_src, image);
    cv::imshow(win_dst, img_dst);

	cv::waitKey(1);

    kaisu=kaisu+1;
   //ros::spinにジャンプする
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
