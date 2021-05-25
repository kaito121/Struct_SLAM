#include "ros/ros.h"
#include <iostream>
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include "Eigen/Core"
 
#include "Eigen/LU"

using Eigen::MatrixXd;
using namespace std;
using namespace Eigen;

int main()
{
  MatrixXd m0(2,2);
  m0(0,0) = 3;
  m0(1,0) = 2.5;
  m0(0,1) = -1;
  m0(1,1) = m0(1,0) + m0(0,1);

    MatrixXd m(3,3);
  m(0,0) = 1;
  m(0,1) = 2;
  m(0,2) = 3;
  m(1,0) = 4;
  m(1,1) = 5;
  m(1,2) = 6;
  m(2,0) = 7;
  m(2,1) = 8;
  m(2,2) = 9;;

      MatrixXd m1(3,3);
  m1(0,0) = 1;
  m1(0,1) = 1;
  m1(0,2) = 1;
  m1(1,0) = 1;
  m1(1,1) = 1;
  m1(1,2) = 1;
  m1(2,0) = 1;
  m1(2,1) = 1;
  m1(2,2) = 1;

  MatrixXd m2(3,3);

  m2=m+m1;

  MatrixXd m3(3,3);

  m3=m2*m;


  std::cout << m0 << std::endl;
  std::cout << m1 << std::endl;
  std::cout << m << std::endl;
  std::cout << m2 << std::endl;
  std::cout << m3 << std::endl;

  cout << " 転置行列m0:\n" << m0.transpose() << endl;


//可変列数の行列作成
int L=3,j=0;
double K=-2;
  MatrixXd N(2,L);
  MatrixXd RC(2,L);

    for(j=0;L>j;j++){
    std::cout <<"j="<< j << std::endl;
      N(0,j)=1+K;
      N(1,j)=2+K;
      K=K+0.01;
      RC(0,j)=1;
      RC(1,j)=1;

    }

  std::cout <<"N=\n"<< N << std::endl;
  std::cout <<"RC=\n"<< RC << std::endl;
  //std::cout << m3 << std::endl;

    //逆行列
   Matrix3f A;
   A << -1, 4, -2,
        -2, 5, -2,
        -1, 2, 0;

   MatrixXf C(3,4);
   C << -1, 4, -2, 4,
        -2, 5, -2, 5,
        -1, 2, 0, 6;

   MatrixXf C1(3,4);
   C1 << -5, 5, -5, 5,
        -2, 5, 0, 5,
        -1, 1, 0, 6;

   MatrixXf E(4,4);
   E << -1, 3, 2, -1,
        1, -2, 3, 1,
        0, 0, 1, 1,
        0, 0, 4, 5;

  MatrixXf E1(5,5);
   E1 << 1,2,3,4,5,
        6,7,8,9,0,
        1,2,3,4,5,
        6,7,8,9,0,
        1,2,3,4,5;

  std::cout <<"C=\n"<< C << std::endl;
  std::cout <<"C1=\n"<< C1 << std::endl;
  MatrixXf C2(3,4);

  C2=C*C1.transpose();//正規行列以外で掛け算をするときは転置が必要となる
  std::cout <<"C2=\n"<< C2 << std::endl;


  Matrix3f A1;
  A1=A.transpose()*A;
   
   cout << "Here is the matrix A:\n" << A << endl;
   cout << "The determinant of A is " << A.determinant() << endl;
   cout << "The inverse of A is:\n" << A.inverse() << endl;//逆行列
   cout << "The DenseMatrix of A is:\n" << A.diagonal() << endl;//対角行列

   cout << "Here is the matrix A1:\n" << A1 << endl;

    //転置行列
   cout << " Atenchiis:\n" << A.transpose() << endl;
   //Eigen::MatrixXd sorted_X = perm_row.transpose() * X;
  // std::cout << "sorted_X" << std::endl << sorted_X << std::endl;




    //最小二乗法
   MatrixXf B = MatrixXf::Random(3, 2);
   cout << "Here is the matrix B:\n" << B << endl;
   VectorXf b = VectorXf::Random(3);
   cout << "Here is the right hand side b:\n" << b << endl;
   cout << "The least-squares solution is:\n" << B.bdcSvd(ComputeThinU | ComputeThinV).solve(b) << endl;


  //逆行列の計算
   //double e[5][5]={{1,2,3,4,5},{6,7,8,9,0},{1,2,3,4,5},{6,7,8,9,0},{1,2,3,4,5}};
  double e[4][4]={{-1,3,2,-1},{1,-2,3,1},{0,0,1,1},{0,0,4,5}};
   //double e[3][3]={{-2,-1,0},{8,2,-1},{5,1,-1}}; //入力用の配列
   double inv_e[4][4]; //ここに逆行列が入る
   double buf; //一時的なデータを蓄える
   int i2,j2,k2; //カウンタ
   int n2=4;  //配列の次数
   
   //単位行列を作る
   for(i2=0;i2<n2;i2++){
    for(j2=0;j2<n2;j2++){
    inv_e[i2][j2]=(i2==j2)?1.0:0.0;
    }
   }
   //掃き出し法
   for(i2=0;i2<n2;i2++){
    buf=1/e[i2][i2];
    for(j2=0;j2<n2;j2++){
    e[i2][j2]*=buf;
    inv_e[i2][j2]*=buf;
   }
   for(j2=0;j2<n2;j2++){
    if(i2!=j2){
     buf=e[j2][i2];
     for(k2=0;k2<n2;k2++){
      e[j2][k2]-=e[i2][k2]*buf;
      inv_e[j2][k2]-=inv_e[i2][k2]*buf;
     }
    }
   }
   }
   //逆行列を出力
   for(i2=0;i2<n2;i2++){
    for(j2=0;j2<n2;j2++){
     printf(" %f",inv_e[i2][j2]);
    }
    printf("\n");
   }//逆行列の計算ここまで
   std::cout <<"Eの逆行列=\n"<< E.inverse() << std::endl;
   
   /* 出力
   2.000000 2.000000 -1.000000 3.000000
   -4.000000 -5.000000 3.000000 -7.000000
   3.000000 4.000000 -2.000000 5.000000
   -7.000000 -8.000000 5.000000 -11.000000
   */


   //LU分解（連立方程式）

    /* std::cout << "[[[[[solve_linear_system]]]]]" << std::endl;
     MatrixXd A0 = MatrixXd::Random(5,5);
     FullPivLU< MatrixXd > lu(A0);
    
     VectorXd b0 = VectorXd::Random(5);
     VectorXd x0;
    
     std::cout << "Matrix A0" << std::endl << A0 << std::endl;
     std::cout << "Vector b0" << std::endl << b0 << std::endl << std::endl;
      
     x0=lu.solve(b0);
     std::cout << "Solve A0x0=b0:" << std::endl<< 0 << std::endl << std::endl;
     std::cout << "Check solution by A0x0-b0: " << std::endl<< A0*x0-b0 << std::endl << std::endl;*/

  
      //逆行列の例
      /*MatrixXd A01 = MatrixXd::Random(100,100);
      MatrixXd b01 = MatrixXd::Random(100,50);
      MatrixXd x01 = A01.fullPivLu().solve(b01);
      FullPivLU< MatrixXd > lu(A01);
      cout << x01 <<endl;
      cout << "ランク：" << lu.rank() <<endl;
      
      cout << "A01.cols()：" << A01.cols()<<endl;
      cout << "相対誤差：" << (A01*x01 - b01).norm()/ b01.norm() << endl;*/


      Matrix3f A02;
      Vector3f V02;
      A02 << 1,2,3, 4,5,6, 7,8,10;
      V02 << 3, 3, 4;
      cout << "Here is the matrix A02:\n" << A02 << endl;
      cout << "Here is the matrix V02:\n" << V02 << endl;
      cout << "逆行列？:\n" <<  A02.colPivHouseholderQr().solve(V02) << endl;

      //固有ベクトル

       Matrix3d T,T2;
   T << 1, -1, -1,
        1, 3, -5,
        1, 1, -3;
 cout << "1Here is the matrix T:\n" << T << endl;
 T2 = (T + T.transpose()) / 2.0; // 対称行列化
 T = T2 * T2.transpose(); // 正値化
 cout << "2正規化T:\n" << T << endl;
 SelfAdjointEigenSolver<Matrix3d> ES(T);
 cout << "5ES(T)(固有値)=\n" << ES.eigenvalues() << endl;
 cout << "5ES(T)(固有ベクトル)=\n" << ES.eigenvectors() << endl;//Tの値を変えるとESも変動するからTの影響を受けている(固有ベクトル)
 cout << "5ES(T)(最小固有値)=\n" << ES.eigenvalues()(0) << endl;
 cout << "5ES(T)(最小固有値に対する固有ベクトル)=\n" << ES.eigenvectors().col(0) << endl;




      return 0;





      }