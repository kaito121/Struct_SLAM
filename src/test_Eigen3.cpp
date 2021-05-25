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

template<class MatT>
 MatT GenInv(const MatT& M, int r)
 {
 if (M.cols() != M.rows()) abort(); // 正方行列でない

 int n = M.cols(); // サイズ

 // 自己随伴行列 (実数行列では対称放列) の固有値のソルバー
 SelfAdjointEigenSolver<MatT> ES(M);

 if (ES.info() != Success) abort(); // 失敗したら終了

 // 一般逆行列用の行列を零行列に初期化
 MatT GenI = MatT::Zero();

 // 一般逆行列の計算
 for (int i = n - r; i < n; i++)
 {
 GenI += ES.eigenvectors().col(i) * ES.eigenvectors().col(i).transpose() / ES.eigenvalues()(i);
 }

 // 戻り
 return GenI;
 }

//コールバック関数
void callback_function(const sensor_msgs::Image::ConstPtr& msg)//画像トピックが更新されたらよばれるやつ//センサーデータがmsgに入る
{std::cout <<"for文終了4"<< std::endl;
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
std::cout <<"for文終了5"<< std::endl;
    image = bridgeImage->image.clone();//image変数に変換した画像データを代入


//ここに処理項目
	cv::Mat img_src = image;
    cv::Mat img_gray,img_gray2,img_edge,img_dst,img_FLD,img_line,img_line2,img_line3;
    double theta[1000],theta0,theta90;

    //ラインだけの画像を作るために単色で塗りつぶした画像を用意する
    img_line = img_src.clone();
    img_line = cv::Scalar(255,255,255);
    

    cv::line(img_line,cv::Point(0,50),cv::Point(640,50),cv::Scalar(0,0,0), 4, cv::LINE_AA);//X線
    cv::line(img_line,cv::Point(50,0),cv::Point(50,600),cv::Scalar(0,0,0), 4, cv::LINE_AA);//Y線
    cv::circle(img_line,Point(50,50),5,Scalar(255,0,0),-1);
    
    
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
    //Nは2行L列の行列

    //可変列数の行列作成
    int j=0,t=0;
    MatrixXd N(2,L);
    MatrixXd PC(2,L);

    //直線の作成
    for(j=0;L>j;j++){
    std::cout <<"j="<< j << std::endl;
      N(0,j)=r*cos(q*K);//直線の端点の座標x
      N(1,j)=r*sin(q*K);//直線の端点の座標y
      K=K+0.1;
      PC(0,j)=150;//消失点真値(x座標)
      PC(1,j)=300;//消失点真値(y座標)
      cv::line(img_line,cv::Point(PC(0,j),PC(1,j)),cv::Point(N(0,j),N(1,j)),cv::Scalar(150,50,0), 4, cv::LINE_AA);//直線
    }
    cv::circle(img_line,Point(PC(0,0),PC(1,0)),5,Scalar(255,255,0),-1);//消失点真値
  std::cout <<"N=\n"<< N << std::endl;
  std::cout <<"PC=\n"<< PC << std::endl;

  //法線ベクトルを求める
  //法線ベクトルは直線の90度なので90度回転させる
  //ただし90度回転させただけなので、そのベクトルを単位ベクトル化することで法線ベクトルを作る(M2 4月研究参照)
  MatrixXd R(2,2);
  //R(0,0)=cos(M_PI/2);
  //R(0,1)=-sin(M_PI/2);
  //R(1,0)=sin(M_PI/2);
  //R(1,1)=cos(M_PI/2);
  R(0,0)=0;
  R(0,1)=-1;
  R(1,0)=1;
  R(1,1)=0;

  std::cout <<"回転行列R=\n"<< R << std::endl;

  MatrixXd n(2,L);
  n=R*(PC-N);//直線を90度回転させたベクトルn
  for(t=0;L>t;t++){
  cv::line(img_line,cv::Point(PC(0,0),PC(1,0)),cv::Point(PC(0,0)-n(0,t),PC(1,0)-n(1,t)),cv::Scalar(0,255,0), 4, cv::LINE_AA);//90度回転した直線(PC-nでベクトルから座標変換)
  }


  std::cout <<"n=\n"<< n << std::endl;

  MatrixXd na(2,L);
  na=n.transpose()*n;//na=n^T*n
  std::cout <<"na=\n"<< na << std::endl;
  std::cout <<"naの対角行列=\n"<< na.diagonal() << std::endl;
  MatrixXd n1(L,1);
  MatrixXd n2(2,L);
  MatrixXd n22(L,1);
  

    
  for(t=0;L>t;t++){
    std::cout <<"t="<< t << std::endl;
    n1(t,0)=sqrt(na(0,t));//ベクトルnの大きさ
    n2(0,t)=n(0,t)/n1(t,0);//法線ベクトル
    n2(1,t)=n(1,t)/n1(t,0);
    cv::line(img_line,cv::Point(PC(0,0),PC(1,0)),cv::Point((PC(0,0)-n2(0,t)*100),(PC(1,0)-n2(1,t)*100)),cv::Scalar(0,0,255), 4, cv::LINE_AA);//法線ベクトル
    }
  std::cout <<"n1=\n"<< n1 << std::endl;
  std::cout <<"法線ベクトルn2=\n"<< n2 << std::endl;

  MatrixXd p0(L,L);
  MatrixXd n3(L,L);
  MatrixXd n4(L,2);
  MatrixXd X(2,1);

 
  p0=n2.transpose()*N;//P0=nT*Nを計算
  n3=n2.transpose()*n2;//逆行列の内部を計算
 

    Vector3d v; // 初期化用ベクトル
    Matrix3d M; // 対象とする行列
    Matrix3d Mi; // 一般逆行列

 // ランク 5 の対称行列作成
    Matrix3d T, T2;
    T = n2; // ランダムな要素の行列を生成
    T2 = (T + T.transpose()) / 2.0; // 対称行列化
    T = T2 * T2.transpose(); // 正値化

    SelfAdjointEigenSolver<Matrix3d> ES(T);
    M = Matrix3d::Zero(); // 零行列に初期化
    for (int i = 1; i < L; i++)
    {
    M += ES.eigenvectors().col(i) * ES.eigenvectors().col(i).transpose() * ES.eigenvalues()(i);
    }
    cout << "Original␣matrix␣M␣=␣\n" << M << endl;

    // ランク 5 の一般逆行列
    Mi = GenInv(M, 2);
    cout << "Generalized␣inverse␣of␣M␣with␣rank␣5␣=␣\n" << Mi << endl;

    n4=Mi*n2.transpose();//逆行列と法線ベクトルの転置を掛け算
    X=n4.transpose()*p0.diagonal();//上記の計算結果にP0の対角行列をかけている※ここで余計に転置をやっている（そうしないと計算できない）

  for(t=0;L>t;t++){
      n22(t,0)=sqrt(n3(0,t));
    }

    std::cout <<"法線ベクトルの大きさn22=\n"<< n22 << std::endl;

    std::cout <<"p0=\n"<< p0 << std::endl;
    std::cout <<"n3=\n"<< n3 << std::endl;
    std::cout <<"n3の逆行列=\n"<< n3.inverse() << std::endl;
    std::cout <<"n4=\n"<< n4 << std::endl;
    std::cout <<"X=\n"<< X << std::endl;
    cv::circle(img_line,Point(X(0,0),X(1,0)),5,Scalar(0,0,255),-1);






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


/*#include "ros/ros.h"
#include <iostream>
#include <Eigen/Core> // 基本演算のみ
#include <Eigen/Eigen> // 固有値
using namespace Eigen;
using namespace std;
typedef Matrix<double,3,1> Vector3d;
typedef Matrix<double,3,3> Matrix3d;

 // n × n 対称行列 M のランク r の一般逆行列
 //template<class MatT>
 int main(int argc,char **argv)
 {
 Vector3d v; // 初期化用ベクトル
 Matrix3d M; // 対象とする行列
 Matrix3d Mi; // 一般逆行列

 // ランク 5 の対称行列作成
 Matrix3d T, T2;
 T = Matrix3d::Random(); // ランダムな要素の行列を生成
 T2 = (T + T.transpose()) / 2.0; // 対称行列化
 T = T2 * T2.transpose(); // 正値化

 SelfAdjointEigenSolver<Matrix3d> ES(T);
 M = Matrix3d::Zero(); // 零行列に初期化
 for (int i = 1; i < 3; i++)
 {
 M += ES.eigenvectors().col(i) * ES.eigenvectors().col(i).transpose() * ES.eigenvalues()(i);
 }
 cout << "Original␣matrix␣M␣=␣\n" << M << endl;

 if (M.cols() != M.rows()) abort(); // 正方行列でない

 int n = M.cols(); // サイズ

 // 自己随伴行列 (実数行列では対称放列) の固有値のソルバー
 SelfAdjointEigenSolver<MatT> ES(M);

 if (ES.info() != Success) abort(); // 失敗したら終了

 // 一般逆行列用の行列を零行列に初期化
 MatT GenI = MatT::Zero();

 // 一般逆行列の計算
 for (int i = n - r; i < n; i++)
 {
 GenI += ES.eigenvectors().col(i) * ES.eigenvectors().col(i).transpose() / ES.eigenvalues()(i);
 }

 // ランク 5 の一般逆行列
 //Mi = GenInv(M, 2);
 cout << "Generalized␣inverse␣of␣M␣with␣rank␣5␣=␣\n" << GenI << endl;
  
return 0;
}

/*#include "ros/ros.h"
#include <iostream>
#include <Eigen/Core> // 基本演算のみ
#include <Eigen/Eigen> // 固有値
using namespace Eigen;
using namespace std;
typedef Matrix<double,3,1> Vector3d;
typedef Matrix<double,3,3> Matrix3d;

 // n × n 対称行列 M のランク r の一般逆行列
 template<class MatT>
 MatT GenInv(const MatT& M, int r)
 {
 if (M.cols() != M.rows()) abort(); // 正方行列でない

 int n = M.cols(); // サイズ

 // 自己随伴行列 (実数行列では対称放列) の固有値のソルバー
 SelfAdjointEigenSolver<MatT> ES(M);

 if (ES.info() != Success) abort(); // 失敗したら終了

 // 一般逆行列用の行列を零行列に初期化
 MatT GenI = MatT::Zero();

 // 一般逆行列の計算
 for (int i = n - r; i < n; i++)
 {
 GenI += ES.eigenvectors().col(i) * ES.eigenvectors().col(i).transpose() / ES.eigenvalues()(i);
 }

 // 戻り
 return GenI;
 }

 int main()
 {
 Vector3d v; // 初期化用ベクトル
 Matrix3d M; // 対象とする行列
 Matrix3d Mi; // 一般逆行列

 // ランク 5 の対称行列作成
 Matrix3d T, T2;
 T = Matrix3d::Random(); // ランダムな要素の行列を生成
 T2 = (T + T.transpose()) / 2.0; // 対称行列化
 T = T2 * T2.transpose(); // 正値化

 SelfAdjointEigenSolver<Matrix3d> ES(T);
 M = Matrix3d::Zero(); // 零行列に初期化
 for (int i = 1; i < 3; i++)
 {
 M += ES.eigenvectors().col(i) * ES.eigenvectors().col(i).transpose() * ES.eigenvalues()(i);
 }
 cout << "Original␣matrix␣M␣=␣\n" << M << endl;

 // ランク 5 の一般逆行列
 Mi = GenInv(M, 2);
 cout << "Generalized␣inverse␣of␣M␣with␣rank␣5␣=␣\n" << Mi << endl;
  



}*/