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
float Ts=1,d=2;
float kaisu;

cv::Mat F,H,W,V,Q,R,P,I,y,K,S,Ft,Ht,xEST,xACT,xDES;

float u[2],LM[10],uR=0,uL=0,vAct,vDes,omegaAct,omegaDes;
float xDes[3],xAct[3],xEst[3],zEst[4],z[4];



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
    

    std::normal_distribution<> normW1(0, 0.04);// 平均0.04, 分散値0.04の正規分布
    std::normal_distribution<> normW2(0, 0.04);// 平均0.04, 分散値0.04の正規分布
    std::normal_distribution<> normV1(0, 0.04);// 平均0.09, 分散値0.04の正規分布
    std::normal_distribution<> normV2(0, 0.04);// 平均0.09, 分散値0.04の正規分布



//カルマンフィルタ初期設定---------------------------------------------------------------------------------------------------------
 if(kaisu==0){

  u[0]=2.05;//入力1
  u[1]=1.95;


  LM[0]=50;//ランドマーク1の位置
  LM[1]=100;

  LM[2]=150;//ランドマーク2の位置
  LM[3]=300;

  W = (cv::Mat_<float>(2, 1) <<0.04,0.04);//入力ノイズ
  V = (cv::Mat_<float>(2, 1) <<0.09,0.09);//出力ノイズ

  Q = (cv::Mat_<float>(3,3) << //共分散Q
    0.04, 0,0,
    0, 0.04,0,
    0, 0, 0.01);

  R = (cv::Mat_<float>(4,4) << //共分散R
    0.16, 0, 0, 0, 
    0, 0.04 ,0 ,0,
    0, 0, 0.04, 0,
    0, 0, 0, 0.04);

  xDES = cv::Mat_<float>::zeros(3, 1);//目標指令状態
  xACT = cv::Mat_<float>::zeros(3, 1);//雑音の影響を考慮した実際の状態
  xEST = cv::Mat_<float>::zeros(3, 1);//推定状態

  xDes[0]=0;
  xDes[1]=0;
  xDes[2]=0;
  xAct[0]=0;
  xAct[1]=0;
  xAct[2]=0;
  xEst[0]=0;
  xEst[1]=0;
  xEst[2]=0;
  zEst[0]=0;
  zEst[1]=0;
  zEst[2]=0;
  zEst[3]=0;
  z[0]=0;//出力方程式
  z[1]=0;
  z[2]=0;//出力方程式
  z[3]=0;


  P = cv::Mat_<float>::eye(3, 3);//誤差分散行列
  I = cv::Mat_<float>::eye(3, 3);//単位行列

  y = cv::Mat_<float>::zeros(1, 4);//観測残渣y
  S = cv::Mat_<float>::zeros(4, 4);
  K = cv::Mat_<float>::zeros(3, 4);//カルマンゲイン

  Ft = cv::Mat_<float>::zeros(3, 3);//Ftヤコビアン行列
  Ht = cv::Mat_<float>::zeros(4, 3);//Ftヤコビアン行列
  //H = cv::Mat_<float>::zeros(4, 3);//Ftヤコビアン行列
std::cout <<"初期設定"<< std::endl;

 }

//--------------------------------------------------------------------------------------------------------------------------
std::cout <<"状態方程式"<< std::endl;

//状態方程式
uR = u[0] + normW1(mt);//右のタイヤ(雑音計算）
uL = u[1] + normW1(mt);//左のタイヤ(雑音計算)

vAct = (uR+uL)/2;//雑音あり
vDes = (u[0]+u[1])/2;//雑音なし

omegaAct = (uR-uL)/(2*d);//雑音あり
omegaDes = (u[0]-u[1])/(2*d);//雑音なし

//雑音の影響を考慮した実際の状態
//xAct[0] = xAct[0] + vAct * Ts *cos(xAct[2]+omegaAct*Ts/2);//ロボットの状態x
//xAct[1] = xAct[1] + vAct * Ts *sin(xAct[2]+omegaAct*Ts/2);//ロボットの状態y
//xAct[2] = xAct[2] + omegaAct * Ts;//ロボットの状態θ

xAct[0] = xAct[0] + vAct * Ts *cos(xAct[2]+Ts/2);//ロボットの状態x
xAct[1] = xAct[1] + vAct * Ts *sin(xAct[2]+Ts/2);//ロボットの状態y
xAct[2] = xAct[2] + Ts;//ロボットの状態θ


xACT= (cv::Mat_<float>(3,1) <<
xAct[0],
xAct[1],
xAct[2]);

//目標指令状態
xDes[0] = xDes[0] + vDes * Ts *cos(xDes[2]+omegaDes*Ts/2);
xDes[1] = xDes[1] + vDes * Ts *sin(xDes[2]+omegaDes*Ts/2);
xDes[2] = xDes[2] + omegaDes * Ts;

xDES= (cv::Mat_<float>(3,1) <<
xDes[0],
xDes[1],
xDes[2]);

//出力方程式(ロボットの現在の位置から見たLMまでの距離と角度)
z[0]=sqrt((LM[0]-xAct[0])*(LM[0]-xAct[0]) + (LM[1]-xAct[1])*(LM[1]-xAct[1])) + normV1(mt);
z[1]=atan2(LM[1]-xAct[1],LM[0]-xAct[0]) - xAct[2] + normV2(mt);

z[2]=sqrt((LM[2]-xAct[0])*(LM[2]-xAct[0]) + (LM[3]-xAct[1])*(LM[3]-xAct[1])) + normV1(mt);
z[3]=atan2(LM[3]-xAct[1],LM[2]-xAct[0]) - xAct[2] + normV2(mt);

//予測ステップ-----------------------------------------------------------------------------------------------------
std::cout <<"予測ステップ"<< std::endl;

Ft= (cv::Mat_<float>(3,3) << //共分散Q
    1, 0, -vDes * Ts *sin(xEst[2]+omegaDes*Ts/2),
    0, 1, vDes * Ts *cos(xEst[2]+omegaDes*Ts/2),
    0, 0, 1);

//推定状態の予測
xEst[0] = xEst[0] + vDes * Ts *cos(xEst[2]+omegaDes*Ts/2);//推定状態(一つ前のxEstから現在のロボットの状態xEstを推定)
xEst[1] = xEst[1] + vDes * Ts *sin(xEst[2]+omegaDes*Ts/2);
xEst[2] = xEst[2] + omegaDes * Ts;

xEST= (cv::Mat_<float>(3,1) <<
xEst[0],
xEst[1],
xEst[2]);

P = Ft*P*Ft.t()+Q;//誤差共分散の予測

//更新ステップ-------------------------------------------------------------------------------------
std::cout <<"更新ステップ"<< std::endl;

Ht= (cv::Mat_<float>(4,3) << //共分散Q
   -(LM[0]-xEst[0])/(sqrt(((LM[0]-xEst[0])*(LM[0]-xEst[0]))+((LM[1]-xEst[1])*(LM[1]-xEst[1])))), 
   -(LM[1]-xEst[1])/(sqrt(((LM[0]-xEst[0])*(LM[0]-xEst[0]))+((LM[1]-xEst[1])*(LM[1]-xEst[1])))), 0,
   -(LM[1]-xEst[1])/(((LM[1]-xEst[1])*(LM[1]-xEst[1]))+((LM[0]-xEst[0])*(LM[0]-xEst[0]))), 
   -(LM[0]-xEst[0])/(((LM[1]-xEst[1])*(LM[1]-xEst[1]))+((LM[0]-xEst[0])*(LM[0]-xEst[0]))), -1,

   -(LM[2]-xEst[0])/(sqrt(((LM[2]-xEst[0])*(LM[2]-xEst[0]))+((LM[3]-xEst[1])*(LM[3]-xEst[1])))), 
   -(LM[3]-xEst[1])/(sqrt(((LM[2]-xEst[0])*(LM[2]-xEst[0]))+((LM[3]-xEst[1])*(LM[3]-xEst[1])))), 0,
   -(LM[3]-xEst[1])/(((LM[3]-xEst[1])*(LM[3]-xEst[1]))+((LM[2]-xEst[0])*(LM[2]-xEst[0]))), 
   -(LM[2]-xEst[0])/(((LM[3]-xEst[1])*(LM[3]-xEst[1]))+((LM[2]-xEst[0])*(LM[2]-xEst[0]))), -1);


//出力方程式
zEst[0]=sqrt((LM[0]-xEst[0])*(LM[0]-xEst[0]) + (LM[1]-xEst[1])*(LM[1]-xEst[1]));
zEst[1]=atan2(LM[1]-xEst[1],LM[0]-xEst[0]) - xEst[2];

zEst[2]=sqrt((LM[2]-xEst[0])*(LM[2]-xEst[0]) + (LM[3]-xEst[1])*(LM[3]-xEst[1]));
zEst[3]=atan2(LM[3]-xEst[1],LM[2]-xEst[0]) - xEst[2];


//観測残渣yの計算
//y= (cv::Mat_<float>(1,2) <<
//z[0] - zEst[0],
//z[1] - zEst[1]);

y= (cv::Mat_<float>(1,4) <<
z[0] - zEst[0],
z[1] - zEst[1],
z[2] - zEst[2],
z[3] - zEst[3]);

std::cout <<"観測残渣の共分散S"<< std::endl;


S = Ht * P * Ht.t()+R;//観測残渣の共分散S
std::cout <<"カルマンゲインの算出"<< std::endl;


K = P * Ht.t() * S.inv();//カルマンゲインの算出
std::cout <<"予測推定状態の更新"<< std::endl;


xEST = xEST + K * y.t();//予測推定状態の更新
std::cout <<"誤差共分散Pの更新"<< std::endl;

P = (I - K * Ht) * P;//誤差共分散Pの更新

std::cout <<"カルマンゲインK=\n"<<K<< std::endl;
std::cout <<"推定の状態(更新)xEst=\n"<<xEST<< std::endl;
std::cout <<"目標の状態xDES=\n"<<xDES<< std::endl;
std::cout <<"実際の状態xACT=\n"<<xACT<< std::endl;
std::cout <<"出力方程式z[0]="<<z[0]<< std::endl;//ロボットからランドマークまでの距離
std::cout <<"出力方程式z[1]="<<z[1]<< std::endl;//ロボットから見たランドマークまでの角度

//cv::line(img_dst,cv::Point(xEST.at<float>(0)*5,xEST.at<float>(1)*5),cv::Point(z[0]*cos(z[1]),z[0]*sin(z[1])),cv::Scalar(0,0,0), 0.5, cv::LINE_AA);
//cv::line(img_dst,cv::Point(xEST.at<float>(0)*5,xEST.at<float>(1)*5),cv::Point(z[2]*cos(z[3]),z[2]*sin(z[3])),cv::Scalar(100,0,255), 0.5, cv::LINE_AA);
//cv::line(img_dst,cv::Point(xDES.at<float>(0)*5,xDES.at<float>(1)*5),cv::Point(zEst[0]*cos(zEst[1]),zEst[0]*sin(zEst[1])),cv::Scalar(0,0,0), 0.5, cv::LINE_AA);
//cv::line(img_dst,cv::Point(xDES.at<float>(0)*5,xDES.at<float>(1)*5),cv::Point(zEst[2]*cos(zEst[3]),zEst[2]*sin(zEst[3])),cv::Scalar(100,0,255), 0.5, cv::LINE_AA);

cv::circle(img_dst, cv::Point(xEST.at<float>(0)*5, xEST.at<float>(1)*5), 3, cv::Scalar(0, 255, 0), -1);//推定状態（緑
cv::circle(img_dst, cv::Point(xDES.at<float>(0)*5, xDES.at<float>(1)*5), 3, cv::Scalar(255, 0, 0), -1);//目標指令状態（青
cv::circle(img_dst, cv::Point(xACT.at<float>(0)*5, xACT.at<float>(1)*5), 3, cv::Scalar(0, 0, 255), -1);//雑音の影響を考慮した実際の状態（赤

cv::circle(img_dst, cv::Point(LM[0], LM[1]), 8, cv::Scalar(255, 255, 0), -1);//ランドマーク1の位置
cv::circle(img_dst, cv::Point(LM[2], LM[3]), 8, cv::Scalar(255, 0, 255), -1);//ランドマーク2の位置



//cv::line(img_dst,cv::Point(xACT.at<float>(0)*5,xACT.at<float>(1)*5),cv::Point(zEst[0]*cos(zEst[1]),zEst[0]*sin(zEst[1])),cv::Scalar(0,0,255), 1, cv::LINE_AA);




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