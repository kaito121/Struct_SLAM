#define _CRT_SECURE_NO_WARNINGS
#define _USE_MATH_DEFINES
#include <ros/ros.h>
#include <iostream>
#include <cv.h>
#include <highgui.h>
#include <opencv2/opencv.hpp>
std::string win_src = "src";//カメラ画像
std::string win_dst = "dst";//カメラ画像

 
using namespace cv;
 
int main (int argc, char **argv)
{
  const int cluster_count = 2; //クラスタの数
 
  // (1)load a specified file as a 3-channel color image
   cv::Mat src_img = cv::imread("/home/fuji/catkin_ws/src/Struct_SLAM/src/OptF/IMG_3257.JPG", 1);
 
  //(2)画像を1列のマトリクスに整形する 
  Mat points;
  src_img.convertTo(points, CV_32FC3);
  points = points.reshape(3, src_img.rows*src_img.cols);

  cv::Mat m100 = (cv::Mat_<double>(3,3) << 1, 2, 3, 4, 5, 6, 7, 8, 9);
 
  //(3)k-meansクラスタリングアルゴリズムを用いて、RGB色空間のピクセルをセグメント化する。
  Mat_<int> clusters(points.size(), CV_32SC1);
  Mat centers;
  kmeans(points, cluster_count, clusters, 
     cv::TermCriteria(CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 10, 1.0), 1, KMEANS_PP_CENTERS, centers);
 
  // (4)クラスタ内のすべてのピクセルを表すセントロイドを作成します。
  Mat dst_img(src_img.size(), src_img.type());
  MatIterator_<Vec3f> itf = centers.begin<Vec3f>();
  MatIterator_<Vec3b> itd = dst_img.begin<Vec3b>(), itd_end = dst_img.end<Vec3b>();
  for(int i=0; itd != itd_end; ++itd, ++i) {
    Vec3f color = itf[clusters(1,i)];
    (*itd)[0] = saturate_cast<uchar>(color[0]);
    (*itd)[1] = saturate_cast<uchar>(color[1]);
    (*itd)[2] = saturate_cast<uchar>(color[2]);
  }
 
  // (5)ソースとデスティネーションの画像を表示し、いずれかのキーを押すと終了する。
    cv::namedWindow(win_src, cv::WINDOW_AUTOSIZE);
    cv::imshow(win_src, src_img);
    cv::namedWindow(win_dst, cv::WINDOW_AUTOSIZE);
    cv::imshow(win_dst, dst_img);

    cv::waitKey(0);//waitKey(0)にすると止められる

 
  return 0;
}