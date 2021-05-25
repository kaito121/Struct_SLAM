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

#include <iostream>
#include <Eigen/Core> // 基本演算のみ
#include <Eigen/SVD>

using namespace Eigen;
using namespace std;

typedef Matrix<double,3,1> Vector6d;
 int main()
 {
 /*MatrixXd A(3,2); // 動的サイズ行列．後で初期化．
 Vector3d b;
 Vector2d x;

 // 係数行列のセット
 A = MatrixXd::Random(3,2);
 cout << "matrix␣A␣=␣\n" << A << endl;

 // ベクトル b のセット
 b = Vector6d::Random();
 cout << "vector␣b␣=␣" << b.transpose() << endl;

 // JacobiSVD を使った特異値分解
 JacobiSVD<MatrixXd> svd(A, ComputeThinU|ComputeThinV);

 // 最小 2 乗解
 x = svd.solve(b);

 cout << "A␣least-squares␣solution␣=␣" << x.transpose() << endl;*/

Matrix3d A;
Vector3d x, b;
// 係数行列のセット
A << 1.0, 2.0, 1.0,
2.0, 1.0, 0.0,
-1.0, 1.0, 2.0;
cout << "matrix␣A␣=␣\n" << A << endl;
// ベクトル b のセット
b << -1.0, 2.0, 1.5;
cout << "vector␣b␣=␣" << b.transpose() << endl;
// FullPivLU を使った LU 分解
FullPivLU<Matrix3d> lu(A);
// LU 分解を使った連立方程式の解
x = lu.solve(b);
cout << "solution␣x␣=␣" << x.transpose() << endl;

 }