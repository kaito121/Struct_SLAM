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
#include <Eigen/Dense>

using Eigen::MatrixXd;
using namespace std;
using namespace Eigen;

ros::Subscriber sub;//データをsubcribeする奴
std::string win_src = "src";
std::string win_edge = "edge";
std::string win_dst = "dst";
std::string win_fld = "fld";
std::string win_line = "line";
std::string win_line2 = "line2";
std::string win_line3 = "line3";

using namespace std;
using namespace cv;

// 抽出する画像の輝度値の範囲を指定
#define B_MAX 255
#define B_MIN 150
#define G_MAX 255
#define G_MIN 180
#define R_MAX 255
#define R_MIN 180

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


//ここに処理項目
	cv::Mat img_src = image;
  cv::Mat img_gray,img_gray2,img_edge,img_dst,img_FLD,img_line,img_line2,img_line3;
  double theta[1000],theta0,theta90;

  //ラインだけの画像を作るために単色で塗りつぶした画像を用意する
  img_line = img_src.clone();
  img_line = cv::Scalar(255,255,255);
    
  cv::line(img_line,cv::Point(0,50),cv::Point(640,50),cv::Scalar(0,0,0), 4, cv::LINE_AA);//X線(黒線)
  cv::line(img_line,cv::Point(50,0),cv::Point(50,600),cv::Scalar(0,0,0), 4, cv::LINE_AA);//Y線(黒線)
  cv::circle(img_line,Point(50,50),5,Scalar(255,0,0),-1);//(青点)
    
    
  int r=300;//半径
  double q,coss;
  q=M_PI/6;
  coss=cos(180/M_PI);
  std::cout <<"cos="<< coss << std::endl;
  //Kはfor文の範囲要素
  //for(K=-2;K>=2;K=K+0.01)
  double K=-2;
  //int L=(2+2)/0.1;
  int L=3;//LはKの個数(直線の個数)
  std::cout <<"直線の個数L="<< L << std::endl;

  //可変列数の行列作成
  int j=0,t=0;
  MatrixXd N(2,L);//Nは2行L列の行列
  MatrixXd PC(2,L);

    //直線の作成(消失点と任意の点を端点とした線を作成する)
    for(j=0;L>j;j++){
    std::cout <<"j="<< j << std::endl;
      N(0,j)=r*cos(q*K);//直線の端点の座標x
      N(1,j)=r*sin(q*K);//直線の端点の座標y
      K=K+1;
      PC(0,j)=300;//消失点真値(x座標)
      PC(1,j)=300;//消失点真値(y座標)
      cv::line(img_line,cv::Point(PC(0,j),PC(1,j)),cv::Point(N(0,j),N(1,j)),cv::Scalar(150,50,0), 4, cv::LINE_AA);//直線の描写(青線)
    }
    cv::circle(img_line,Point(PC(0,0),PC(1,0)),5,Scalar(255,255,0),-1);//消失点真値の描写(水色の点)
  std::cout <<"直線の端点の座標N=\n"<< N << std::endl;
  std::cout <<"直線の端点の座標(消失点)PC=\n"<< PC << std::endl;

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

  std::cout <<"回転行列R=\n"<< R << std::endl;

  MatrixXd n(2,L);
  n=R*(PC-N);//直線を90度回転させたベクトルn
  for(t=0;L>t;t++){
  cv::line(img_line,cv::Point(PC(0,0),PC(1,0)),cv::Point(PC(0,0)-n(0,t),PC(1,0)-n(1,t)),cv::Scalar(0,255,0), 4, cv::LINE_AA);//90度回転した直線(PC-nでベクトルから座標変換)(緑の線)
  }
  
  std::cout <<"直線を90度回転させたベクトルn=\n"<< n << std::endl;

  //法線ベクトルの大きさを１にする
  MatrixXd na(2,L);
  MatrixXd na1(L,0);
  na=n.transpose()*n;//na=n^T*n（ルートの中身を計算）
  na1=na.diagonal();//naを対角化することで要素の二乗の和を求める(例:a1^2+a2^2);
  std::cout <<"na=\n"<< na << std::endl;
  std::cout <<"naの対角行列=\n"<< na.diagonal() << std::endl;
  MatrixXd n1(L,1);
  MatrixXd n2(2,L);
  MatrixXd n22(L,1);
  MatrixXd p0(L,L);
  MatrixXd n3(2,2);
  MatrixXd n30(L,L);
  MatrixXd n4(L,2);
  MatrixXd n40(L,2);
  MatrixXd X(2,1);
  MatrixXd X0(2,1);
    
  for(t=0;L>t;t++){
    std::cout <<"t="<< t << std::endl;
    n1(t,0)=sqrt(na1(t,0));//ベクトルnの大きさ（二乗の和に対しルートをかける）
    //n1(t,0)=sqrt(na(0,t));//ベクトルnの大きさ（
    n2(0,t)=n(0,t)/n1(t,0);//法線ベクトル
    n2(1,t)=n(1,t)/n1(t,0);//法線ベクトル
    std::cout <<"n(0)("<<t<<")=\n"<< n(0,t) << std::endl;
    std::cout <<"n(1)("<<t<<")=\n"<< n(1,t) << std::endl;
    
    cv::line(img_line,cv::Point(PC(0,0),PC(1,0)),cv::Point((PC(0,0)-n2(0,t)*100),(PC(1,0)-n2(1,t)*100)),cv::Scalar(0,0,255), 4, cv::LINE_AA);//法線ベクトル(描写時に100倍してる）(赤の線)
    }

  std::cout <<"n1=\n"<< n1 << std::endl;
  std::cout <<"法線ベクトルn2=\n"<< n2 << std::endl;

  //最小二乗法の計算
  //p0=n2*N;//P0=nT*Nを計算
  std::cout <<"for文終了0"<< std::endl;
  p0=n2.transpose()*N;//P0=nT*Nを計算//ここ要チェック------------------------------------------------
  n3=n2*n2.transpose();//逆行列の内部を計算
  //p0=n2.transpose()*N;//P0=nT*Nを計算
  //n3=n2.transpose()*n2;//逆行列の内部を計算
  std::cout <<"for文終了1"<< std::endl;

  /*/逆行列の計算
   double e[L][L]; //入力用の配列
   double inv_e[L][L]; //ここに逆行列が入る
   double buf; //一時的なデータを蓄える
   int i2,j2,k2; //カウンタ
   
   //単位行列を作る
   for(i2=0;i2<L;i2++){
    for(j2=0;j2<L;j2++){
      e[i2][j2]=n3(i2,j2);
      inv_e[i2][j2]=(i2==j2)?1.0:0.0;
    }
   }
   //掃き出し法
   for(i2=0;i2<L;i2++){
    buf=1/e[i2][i2];
    for(j2=0;j2<L;j2++){
    e[i2][j2]*=buf;
    inv_e[i2][j2]*=buf;
   }
   for(j2=0;j2<L;j2++){
    if(i2!=j2){
     buf=e[j2][i2];
     for(k2=0;k2<L;k2++){
      e[j2][k2]-=e[i2][k2]*buf;
      inv_e[j2][k2]-=inv_e[i2][k2]*buf;
     }
    }
   }
   }
   //逆行列を出力
   for(i2=0;i2<L;i2++){
    for(j2=0;j2<L;j2++){
     n30(i2,j2)=inv_e[i2][j2];
    }
    std::cout <<"for文終了2"<< std::endl;

    printf("\n");
   }//逆行列の計算ここまで*/


  //最小二乗法の計算
  n4=n2*p0.diagonal();//nにP0の対角行列をかけている 
  X=n3.inverse()*n4;//逆行列と法線ベクトルの転置を掛け算
  //X0=n30*n4;//逆行列と法線ベクトルの転置を掛け算
  std::cout <<"for文終了3"<< std::endl;

  
  /*n4=n3.inverse()*n2.transpose();//逆行列と法線ベクトルの転置を掛け算
  n40=n30*n2.transpose();//逆行列と法線ベクトルの転置を掛け算
  X=n4.transpose()*p0.diagonal();//上記の計算結果にP0の対角行列をかけている※ここで余計に転置をやっている（そうしないと計算できない）
  X0=n40.transpose()*p0.diagonal();//上記の計算結果にP0の対角行列をかけている※ここで余計に転置をやっている（そうしないと計算できない）*/  

  for(t=0;L>t;t++){
      n22(t,0)=sqrt(n3(0,t));
    }

  std::cout <<"法線ベクトルの大きさn22=\n"<< n22 << std::endl;

    
    
  std::cout <<"p0=\n"<< p0 << std::endl;
  std::cout <<"n3=\n"<< n3 << std::endl;
  std::cout <<"n3の逆行列=\n"<< n3.inverse() << std::endl;
  //std::cout <<"n3の逆行列n30=\n"<< n30 << std::endl;
  std::cout <<"n4=\n"<< n4 << std::endl;
  //std::cout <<"n4.transpose()=\n"<<n4.transpose()<< std::endl;
  std::cout <<"p0.diagonal()=\n"<< p0.diagonal() << std::endl;
  std::cout <<"X=\n"<< X << std::endl;
  //std::cout <<"X0=\n"<< X0 << std::endl;
  cv::circle(img_line,Point(X(0,0),X(1,0)),5,Scalar(0,0,255),-1);
  cv::circle(img_line,Point(X0(0,0),X0(1,0)),5,Scalar(255,0,255),-1);


  cv::namedWindow(win_src, cv::WINDOW_AUTOSIZE);
  cv::namedWindow(win_line, cv::WINDOW_AUTOSIZE);


  cv::imshow(win_src, img_src);
  cv::imshow(win_line, img_line);

	cv::waitKey(1);
   //ros::spinにジャンプする
 
}

//メイン関数
int main(int argc,char **argv){

	ros::init(argc,argv,"opencv_main");//rosを初期化
	ros::NodeHandle nh;//ノードハンドル
	
	//subscriber関連
	sub=nh.subscribe("/robot1/camera/color/image_raw",1,callback_function);//取得するトピックデータを選択しコールバック関数に登録

	ros::spin();//トピック更新待機
			
	return 0;
}