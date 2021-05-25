#include "ros/ros.h"
#include <iostream>
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include <Eigen/Dense>

using Eigen::MatrixXd;
using namespace std;
using namespace Eigen;

int main(int argc, char **argv){
    ros::init(argc, argv, "hello_node");
    ROS_INFO("hello world");

    int r=2;//半径
    double q,coss;
    q=M_PI/6;
    coss=cos(180/M_PI);
    std::cout <<"cos="<< coss << std::endl;
    //Kはfor文の範囲要素
    //for(K=-2;K>=2;K=K+0.01)
    //LはKの個数
    double K=-2;
    int L=(2+2)/0.01;
    //Nは2行L列の行列

    //可変列数の行列作成
    int j=0;
    MatrixXd N(2,L);
    MatrixXd RC(2,L);

    for(j=0;L>j;j++){
    std::cout <<"j="<< j << std::endl;
      N(0,j)=r*cos(q*K);
      N(1,j)=r*sin(q*K);
      K=K+0.01;
      RC(0,j)=1;
      RC(1,j)=1;
    }
  std::cout <<"N=\n"<< N << std::endl;
  std::cout <<"RC=\n"<< RC << std::endl;

  MatrixXd R(2,2);
  R(0,0)=cos(M_PI/2);
  R(0,1)=-sin(M_PI/2);
  R(1,0)=sin(M_PI/2);
  R(0,0)=cos(M_PI/2);

  std::cout <<"R=\n"<< R << std::endl;

  MatrixXd n(2,L);
  n=R*(RC-N);
  std::cout <<"n=\n"<< n << std::endl;
  MatrixXd na(2,L);
  na=n.transpose()*n;
  std::cout <<"na=\n"<< na << std::endl;
  std::cout <<"naの対角行列=\n"<< na.diagonal() << std::endl;

  double n1;
  //n1=n.transpose()/(sqrt(n.transpose()*n));
  

    /*for(i=0;i>=L;i++){
      N[i]=(r*cos(q*K),r*sin(q*K));

    }*/
    //PCは要素がすべて1の2行L列の行列
    //Rは2行2列の回転行列
    //nは行列計算
    //n1は法線ベクトル



   
  

   return 0;}