//最小二乗法を用いた消失点推定シミュレーション
//rosのヘッダ
//消失点(100,100)
#include <ros/ros.h>
#include <sensor_msgs/Image.h>//センサーデータ形式ヘッダ
#include <cv_bridge/cv_bridge.h>//画像変換のヘッダ
#include <sensor_msgs/image_encodings.h>//エンコードのためのヘッダ
#include <math.h>
#include <highgui.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/ximgproc/fast_line_detector.hpp>
#include "ros/ros.h"
#include <iostream>
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include <random>
#include<fstream>


using Eigen::MatrixXd;
using namespace std;
using namespace cv;
using namespace Eigen;

int kaisu=0;

ros::Subscriber sub;//データをsubcribeする奴
std::string win_src = "src";
std::string win_edge = "edge";
std::string win_dst = "dst";
std::string win_fld = "fld";
std::string win_line = "line";
std::string win_line2 = "line2";
std::string win_line3 = "line3";

// 抽出する画像の輝度値の範囲を指定
#define B_MAX 255
#define B_MIN 150
#define G_MAX 255
#define G_MIN 180
#define R_MAX 255
#define R_MIN 180

ofstream outputfile1("/home/fuji/catkin_ws/src/Struct_SLAM/src/L2_sp3/20210831/PC_200_N_200_X(0,0).txt");//txtファイルに記録
ofstream outputfile2("/home/fuji/catkin_ws/src/Struct_SLAM/src/L2_sp3/20210831/PC_200_N_200_X(1,0).txt");//txtファイルに記録

//コールバック関数
void callback_function(const sensor_msgs::Image::ConstPtr& msg)//画像トピックが更新されたらよばれるやつ//センサーデータがmsgに入る
{
	//変数宣言
	cv_bridge::CvImagePtr bridgeImage;//クラス::型//cv_brigeは画像変換するとこ
	cv::Mat image;//opencvの画像
	ROS_INFO("callback_functionが呼ばれたよ");
	
    try{//MAT形式変換
       bridgeImage=cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);//MAT形式に変える
       ROS_INFO("callBack");//printと秒数表示
    }
	//エラー処理
    catch(cv_bridge::Exception& e) {//エラー処理(失敗)成功ならスキップ
        std::cout<<"depth_image_callback Error \n";
        ROS_ERROR("Could not convert from '%s' to 'BGR8'.",
        msg->encoding.c_str());
        return ;
    }

    image = bridgeImage->image.clone();//image変数に変換した画像データを代入

//aaaaaaaaaaaaaaaaaaaaaaaaaaa
//ここに処理項目
	cv::Mat img_src = image;
  cv::Mat img_gray,img_gray2,img_edge,img_dst,img_FLD,img_line,img_line2,img_line3;
  double theta[1000],theta0,theta90;

  //ラインだけの画像を作るために単色で塗りつぶした画像を用意する
  img_line = img_src.clone();
  img_line = cv::Scalar(255,255,255);
  img_line2 = img_src.clone();
  img_line2 = cv::Scalar(255,255,255);
    
  //cv::line(img_line,cv::Point(0,50),cv::Point(640,50),cv::Scalar(0,0,0), 4, cv::LINE_AA);//X線(黒線)
  //cv::line(img_line,cv::Point(50,0),cv::Point(50,600),cv::Scalar(0,0,0), 4, cv::LINE_AA);//Y線(黒線)
  //cv::circle(img_line,Point(50,50),5,Scalar(255,0,0),-1);//(青点)
    
    
  int r=300;//半径
  double q,coss;
  q=M_PI/6;
  coss=cos(180/M_PI);
  std::cout <<"cos="<< coss << std::endl;
  //Kはfor文の範囲要素
  //for(K=-2;K>=2;K=K+0.01)
  double K=-2;
  //int L=(2+2)/0.1;
  int L=6;//LはKの個数(直線の個数)
  std::cout <<"直線の個数L="<< L << std::endl;

  //可変列数の行列作成
  int j=0,t=0;
  MatrixXd N(2,L);//Nは2行L列の行列
  MatrixXd PC(2,L);

  std::random_device rnd;     // 非決定的な乱数生成器を生成
  std::mt19937 mt(rnd());     //  メルセンヌ・ツイスタの32ビット版、引数は初期シード値
  std::uniform_int_distribution<> rand50(0, 100);        // [0, 99] 範囲の一様乱数
  std::uniform_int_distribution<> rand100(0, 200);        // [0, 99] 範囲の一様乱数
  std::normal_distribution<> norm(50.0, 50.0);       // 平均50, 分散値10の正規分布
  std::normal_distribution<> norm350(350.0, 50.0);       // 平均350, 分散値50の正規分布
  std::normal_distribution<> norm300(300.0, 50.0);       // 平均300, 分散値50の正規分布
  std::normal_distribution<> norm1(1.0, 50.0);       // 平均1, 分散値50の正規分布
  //std::normal_distribution<> norm300(300.0, 10.0);       // 平均300, 標準偏差10の正規分布
   // for (int i = 0; i < 20; ++i) {
   //     std::cout << "ランダム数rand100(mt)="<<rand100(mt)<<std::endl;
   // }

    //直線の作成(消失点と任意の点を端点とした線を作成する)
   for(j=0;L>j;j++){
    std::cout <<"j="<< j << std::endl;
      //N(0,j)=r*cos(q*K)+rand100(mt);//直線の端点の座標x
      //N(1,j)=r*sin(q*K)+rand100(mt);//直線の端点の座標y
      //N(0,j)=r*cos(q*K)+norm(mt);//直線の端点の座標x
      //N(1,j)=r*sin(q*K)+norm(mt);//直線の端点の座標y
      N(0,j)=r*cos(q*K)+norm1(mt);//直線の端点の座標x
      N(1,j)=r*sin(q*K)+norm1(mt);//直線の端点の座標y
      std::cout <<"直線の端点の座標x_N(0,"<<j<<")="<< N(0,j) << std::endl;
      std::cout <<"直線の端点の座標y_N(1,"<<j<<")="<< N(1,j) << std::endl;
      K=K+1;
      //PC(0,j)=350+rand100(mt);//消失点真値(x座標)
      //PC(1,j)=300+rand100(mt);//消失点真値(y座標)
      //PC(0,j)=350+norm(mt);//消失点真値(x座標)
      //PC(1,j)=300+norm(mt);//消失点真値(y座標)
      PC(0,j)=norm300(mt);//消失点真値(x座標)_観測データ=真値±ノイズ
      PC(1,j)=norm300(mt);//消失点真値(y座標)
      std::cout <<"消失点真値(x座標)_PC(0,"<<j<<")="<< PC(0,j) << std::endl;
      std::cout <<"消失点真値(y座標)_PC(1,"<<j<<")="<< PC(1,j) << std::endl;
      cv::line(img_line,cv::Point(PC(0,j),PC(1,j)),cv::Point(N(0,j),N(1,j)),cv::Scalar(150,50,0), 4, cv::LINE_AA);//直線の描写(青線)
      cv::circle(img_line,Point(PC(0,0),PC(1,0)),5,Scalar(255,255,0),-1);//消失点真値の描写(水色の点)
    }
    //任意の直線を描写
    //N(0,0)=130,N(1,0)=100,PC(0,0)=100,PC(1,0)=130;
    //N(0,1)=100,N(1,1)=130,PC(0,1)=70,PC(1,1)=100;
    //N(0,2)=70,N(1,2)=100,PC(0,2)=100,PC(1,2)=70;
    //N(0,3)=100,N(1,3)=70,PC(0,3)=130,PC(1,3)=100;
    int fx=100,fy=200;


    //N(0,0)=0+fx,N(1,0)=-100+fy,PC(0,0)=300+fx,PC(1,0)=200+fy;
    //N(0,1)=0+fx,N(1,1)=100+fy,PC(0,1)=300+fx,PC(1,1)=-200+fy;
    //N(0,2)=0+fx,N(1,2)=100+fy,PC(0,2)=300+fx,PC(1,2)=400+fy;
    //N(0,3)=0+fx,N(1,3)=300+fy,PC(0,3)=300+fx,PC(1,3)=0+fy;
    //for(j=0;L>j;j++){cv::line(img_line,cv::Point(PC(0,j),PC(1,j)),cv::Point(N(0,j),N(1,j)),cv::Scalar(150,50,0), 4, cv::LINE_AA);}//直線の描写(青線)
    

    //一次関数を描写するための関数
    double thetal[10],l[10][4],lc[10];
    for(j=0;L>j;j++){
    //座標から一次関数を引く関数
    thetal[j]=(M_PI/2)-(M_PI-atan2((PC(0,j)-N(0,j)),(PC(1,j)-N(1,j))));
    lc[j]=((PC(0,j)-N(0,j))*(PC(0,j)-N(0,j)))+((PC(1,j)-N(1,j))*(PC(1,j)-N(1,j)));
    l[j][0]=N(0,j)+(cos(-thetal[j])*sqrt(lc[j]))*10;//X1座標
    l[j][1]=N(1,j)+(sin(-thetal[j])*sqrt(lc[j]))*10;//Y1座標
    l[j][2]=N(0,j)+(cos(-thetal[j])*sqrt(lc[j]))*-10;//X2座標
    l[j][3]=N(1,j)+(sin(-thetal[j])*sqrt(lc[j]))*-10;//Y2座標
    //std::cout <<"直線の角度1θ="<< thetal[j] << std::endl;
    //std::cout <<"直線座標1=("<< l[j][0] <<","<<l[j][1]<<")"<< std::endl;
    //std::cout <<"直線座標2=("<< l[j][2] <<","<<l[j][3]<<")"<< std::endl;

    cv::line(img_line,cv::Point(l[j][0],l[j][1]),cv::Point(l[j][2],l[j][3]),cv::Scalar(150,50,0), 1, cv::LINE_AA);
    cv::line(img_line,cv::Point(PC(0,j),PC(1,j)),cv::Point(N(0,j),N(1,j)),cv::Scalar(150,50,0), 4, cv::LINE_AA);//直線の描写(青線)
    cv::circle(img_line,Point(N(0,j),N(1,j)),5,Scalar(255,150,0),-1);
    cv::circle(img_line,Point(PC(0,j),PC(1,j)),5,Scalar(255,150,0),-1);
    }

    /*//座標から一次関数を引く関数
    thetal[0]=(M_PI/2)-(M_PI-atan2((PC(0,0)-N(0,0)),(PC(1,0)-N(1,0))));
    lc[0]=((PC(0,0)-N(0,0))*(PC(0,0)-N(0,0)))+((PC(1,0)-N(1,0))*(PC(1,0)-N(1,0)));
    l[0][0]=N(0,0)+(cos(-thetal[0])*sqrt(lc[0]))*10;//X1座標
    l[0][1]=N(1,0)+(sin(-thetal[0])*sqrt(lc[0]))*10;//Y1座標
    l[0][2]=N(0,0)+(cos(-thetal[0])*sqrt(lc[0]))*-10;//X2座標
    l[0][3]=N(1,0)+(sin(-thetal[0])*sqrt(lc[0]))*-10;//Y2座標
    std::cout <<"直線の角度1θ="<< thetal[0] << std::endl;
    std::cout <<"直線座標1=("<< l[0][0] <<","<<l[0][1]<<")"<< std::endl;
    std::cout <<"直線座標2=("<< l[0][2] <<","<<l[0][3]<<")"<< std::endl;

    cv::line(img_line,cv::Point(l[0][0],l[0][1]),cv::Point(l[0][2],l[0][3]),cv::Scalar(0,255,255), 4, cv::LINE_AA);//黄色の線


    std::cout <<"元座標N=("<< N(0,0) <<","<< N(1,0)<<")"<< std::endl;
    std::cout <<"元座標PC=("<< PC(0,0) <<","<< PC(1,0)<<")"<< std::endl;


    //三角形の描写
    cv::line(img_line,cv::Point(100,100),cv::Point(100+100,100+100*sqrt(3)),cv::Scalar(0,0,0), 4, cv::LINE_AA);
    cv::line(img_line,cv::Point(100,100),cv::Point(100+100,100),cv::Scalar(0,0,0), 4, cv::LINE_AA);
    cv::line(img_line,cv::Point(100+100,100),cv::Point(100+100,100+100*sqrt(3)),cv::Scalar(0,0,0), 4, cv::LINE_AA);

    //座標から一次関数を引く関数(実行可能)
    thetal[1]=M_PI-atan2((100+100-100),(100+100*sqrt(3)-100));
    thetal[2]=(M_PI/2)-(M_PI-atan2((100+100-100),(100+100*sqrt(3)-100)));
    lc[1]=((100+100-100)*(100+100-100)+(100+100*sqrt(3)-100)*(100+100*sqrt(3)-100));
    l[1][0]=100+(cos(-thetal[2])*sqrt(lc[1]))*10;//X1座標
    l[1][1]=100+(sin(-thetal[2])*sqrt(lc[1]))*10;//Y1座標
    l[1][2]=100+(cos(-thetal[2])*sqrt(lc[1]))*-10;//X2座標
    l[1][3]=100+(sin(-thetal[2])*sqrt(lc[1]))*-10;//Y2座標
    std::cout <<"直線の角度1θ="<< thetal[1] << std::endl;
    std::cout <<"直線の角度2(PI/2-θ)="<< thetal[2] << std::endl;
    std::cout <<"直線座標1=("<< l[1][0] <<","<<l[1][1]<<")"<< std::endl;
    std::cout <<"直線座標2=("<< l[1][2] <<","<<l[1][3]<<")"<< std::endl;

    cv::line(img_line,cv::Point(l[1][0],l[1][1]),cv::Point(l[1][2],l[1][3]),cv::Scalar(0,255,255), 4, cv::LINE_AA);//黄色の線

    //直線の描写の試し例(画像座標系の回転方向など)----------------------------------------------------------------------------------------------------------
    cv::line(img_line2,cv::Point(0,0),cv::Point(400*cos(M_PI/3),400*sin(M_PI/3)),cv::Scalar(0,0,0), 4, cv::LINE_AA);//x軸から時計回りで60度の線を描写
    cv::line(img_line2,cv::Point(0,0),cv::Point(400*cos(M_PI/6),400*sin(M_PI/6)),cv::Scalar(255,0,0), 4, cv::LINE_AA);//x軸から時計回りで30度の線を描写

    cv::line(img_line2,cv::Point(50,50),cv::Point(50+400*cos(M_PI/6),50+400*sin(M_PI/6)),cv::Scalar(255,0,0), 4, cv::LINE_AA);//x軸から時計回りで30度の線を描写

    thetal[3]=M_PI-atan2(400*cos(M_PI/3),400*sin(M_PI/3));//直線とY軸との角度(4月5月　研究を参考)
    thetal[4]=M_PI-atan2(400*cos(M_PI/6),400*sin(M_PI/6));//直線とY軸との角度
    std::cout <<"直線とY軸との角度3θ(直線とY軸との角度)="<< thetal[3] << std::endl;
    std::cout <<"直線とY軸との角度4θ(直線とY軸との角度)="<< thetal[4] << std::endl;

    thetal[5]=(M_PI/2)-(M_PI-atan2(400*cos(M_PI/3),400*sin(M_PI/3)));//直線とY軸との角度から90度引いた角度(4月5月　研究を参考)=X軸との角度（絶対値をつけたら）
    thetal[6]=(M_PI/2)-(M_PI-atan2(400*cos(M_PI/6),400*sin(M_PI/6)));//直線とY軸との角度から90度引いた角度=X軸との角度（絶対値をつけたら）

    std::cout <<"直線とY軸との角度から90度引いた角度5θ="<< thetal[5] << std::endl;
    std::cout <<"直線とY軸との角度から90度引いた角度6θ="<< thetal[6] << std::endl;

    cv::line(img_line2,cv::Point(0,0),cv::Point(100*cos(abs(thetal[5])),100*sin(abs(thetal[5]))),cv::Scalar(0,0,0), 8, cv::LINE_AA);//x軸から時計回りで60度の線を描写
    cv::line(img_line2,cv::Point(0,0),cv::Point(100*cos(abs(thetal[6])),100*sin(abs(thetal[6]))),cv::Scalar(255,0,0), 8, cv::LINE_AA);//x軸から時計回りで30度の線を描写
    //ここまで----------------------------------------------------------------------------------------------------------------------------------
    */






  //cv::circle(img_line,Point(100,100),5,Scalar(255,255,0),-1);//消失点真値の描写(水色の点)
  //std::cout <<"直線の端点の座標N=\n"<< N << std::endl;
  //std::cout <<"直線の端点の座標(消失点)PC=\n"<< PC << std::endl;
//
  //法線ベクトルを求める
  //法線ベクトルは直線の90度なので90度回転させる
  //ただし90度回転させただけなので、そのベクトルを単位ベクトル化することで法線ベクトルを作る(M2 4月研究参照)
  MatrixXd R(2,2);
  R(0,0)=cos(M_PI/2);
  R(0,1)=-sin(M_PI/2);
  R(1,0)=sin(M_PI/2);
  R(1,1)=cos(M_PI/2);
  //R(0,0)=0;//回転行列の代わりにこれを使う（近似してるだけ）
  //R(0,1)=-1;
  //R(1,0)=1;
  //R(1,1)=0;

  //std::cout <<"回転行列R=\n"<< R << std::endl;

  MatrixXd n(2,L);
  n=R*(PC-N);//直線を90度回転させたベクトルn
  for(t=0;L>t;t++){
  cv::line(img_line,cv::Point(PC(0,t),PC(1,t)),cv::Point(PC(0,t)-n(0,t),PC(1,t)-n(1,t)),cv::Scalar(0,255,0), 4, cv::LINE_AA);//90度回転した直線(PC-nでベクトルから座標変換)(緑の線)
  //cv::line(img_line,cv::Point(PC(0,0),PC(1,0)),cv::Point(PC(0,0)-n(0,t),PC(1,0)-n(1,t)),cv::Scalar(0,255,0), 4, cv::LINE_AA);//90度回転した直線(PC-nでベクトルから座標変換)(緑の線)
  }
  
  //std::cout <<"直線を90度回転させたベクトルn=\n"<< n << std::endl;

  //法線ベクトルの大きさを１にする
  MatrixXd na(2,L);
  MatrixXd na1(L,0);
  na=n.transpose()*n;//na=n^T*n（ルートの中身を計算）
  na1=na.diagonal();//naを対角化することで要素の二乗の和を求める(例:a1^2+a2^2);
  //std::cout <<"na=\n"<< na << std::endl;
  //std::cout <<"naの対角行列=\n"<< na.diagonal() << std::endl;
  MatrixXd n1(L,1);
  MatrixXd n2(2,L);
  MatrixXd n22(L,1);
  MatrixXd p0(L,L);
  MatrixXd n3(2,2);
  MatrixXd n30(L,L);
  MatrixXd n4(2,1);
  MatrixXd n40(2,1);
  MatrixXd X(2,1);
  MatrixXd X0(2,1);
    
  for(t=0;L>t;t++){
    //std::cout <<"t="<< t << std::endl;
    n1(t,0)=sqrt(na1(t,0));//ベクトルnの大きさ（二乗の和に対しルートをかける）
    //n1(t,0)=sqrt(na(0,t));//ベクトルnの大きさ（
    n2(0,t)=n(0,t)/n1(t,0);//法線ベクトル
    n2(1,t)=n(1,t)/n1(t,0);//法線ベクトル
    //std::cout <<"n(0)("<<t<<")=\n"<< n(0,t) << std::endl;
    //std::cout <<"n(1)("<<t<<")=\n"<< n(1,t) << std::endl;
    
    cv::line(img_line,cv::Point(PC(0,t),PC(1,t)),cv::Point((PC(0,t)-n2(0,t)*100),(PC(1,t)-n2(1,t)*100)),cv::Scalar(0,0,255), 4, cv::LINE_AA);//法線ベクトル(描写時に100倍してる）(赤の線)
    }

  std::cout <<"n1=\n"<< n1 << std::endl;
  std::cout <<"法線ベクトルn2=\n"<< n2 << std::endl;

  //最小二乗法の計算
  //p0=n2*N;//P0=nT*Nを計算
  //std::cout <<"for文終了0"<< std::endl;
  p0=n2.transpose()*N;//P0=nT*Nを計算//ここ要チェック------------------------------------------------
  n3=n2*n2.transpose();//逆行列の内部を計算
  //p0=n2.transpose()*N;//P0=nT*Nを計算
  //n3=n2.transpose()*n2;//逆行列の内部を計算
  //std::cout <<"for文終了1"<< std::endl;

  //最小二乗法の計算
  n4=n2*p0.diagonal();//nにP0の対角行列をかけている 
  X=n3.inverse()*n4;//逆行列と法線ベクトルの転置を掛け算
  //X0=n30*n4;//逆行列と法線ベクトルの転置を掛け算
  //std::cout <<"for文終了3"<< std::endl;

  
  /*n4=n3.inverse()*n2.transpose();//逆行列と法線ベクトルの転置を掛け算
  n40=n30*n2.transpose();//逆行列と法線ベクトルの転置を掛け算
  X=n4.transpose()*p0.diagonal();//上記の計算結果にP0の対角行列をかけている※ここで余計に転置をやっている（そうしないと計算できない）
  X0=n40.transpose()*p0.diagonal();//上記の計算結果にP0の対角行列をかけている※ここで余計に転置をやっている（そうしないと計算できない）*/  

  /*for(t=0;L>t;t++){
      n22(t,0)=sqrt(n3(0,t));
    }

  std::cout <<"法線ベクトルの大きさn22=\n"<< n22 << std::endl;*/

    
    
  std::cout <<"p0=\n"<< p0 << std::endl;
  std::cout <<"n3=\n"<< n3 << std::endl;
  std::cout <<"n3の逆行列=\n"<< n3.inverse() << std::endl;
  std::cout <<"n4=\n"<< n4 << std::endl;
  std::cout <<"p0.diagonal()=\n"<< p0.diagonal() << std::endl;
  std::cout <<"X=\n"<< X << std::endl;//推定交点

  outputfile1<<X(0,0)<<"\n";
  outputfile2<<X(1,0)<<"\n";


  cv::circle(img_line,Point(X(0,0),X(1,0)),5,Scalar(0,0,255),-1);
  cv::circle(img_line,Point(X0(0,0),X0(1,0)),5,Scalar(255,0,255),-1);


  cv::namedWindow(win_src, cv::WINDOW_AUTOSIZE);
  cv::namedWindow(win_line, cv::WINDOW_AUTOSIZE);
  cv::namedWindow(win_line2, cv::WINDOW_AUTOSIZE);


  cv::imshow(win_src, img_src);
  cv::imshow(win_line, img_line);
  cv::imshow(win_line2, img_line2);
  kaisu=kaisu+1;

	cv::waitKey(1);
   //ros::spinにジャンプする
 
}

//メイン関数
int main(int argc,char **argv){

	ros::init(argc,argv,"simulation");//rosを初期化
	ros::NodeHandle nh;//ノードハンドル
	
	//subscriber関連
	sub=nh.subscribe("/camera/color/image_raw",1,callback_function);//取得するトピックデータを選択しコールバック関数に登録
	//sub=nh.subscribe("/robot1/camera/color/image_raw",1,callback_function);//取得するトピックデータを選択しコールバック関数に登録

	ros::spin();//トピック更新待機
			
	return 0;
}