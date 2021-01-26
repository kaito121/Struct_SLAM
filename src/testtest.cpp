#include "ros/ros.h"
#include <iostream>
#include <opencv2/opencv.hpp>

int main(int argc, char **argv){
    ros::init(argc, argv, "hello_node");
    ROS_INFO("hello world");

   /* int i,j,tmp;
    int theta[5]={10,20,30,30,10,20};
    int A[ ];
    t=1;

  //数値を昇順にソート 
  for (i=0; i<5; ++i) {
    for (j=i+1; j<5; ++j) {
      if (theta[i] > theta[j]) {
        tmp =  theta[i];
        theta[i] = theta[j];
        theta[j] = tmp;
      }
    }
  }

  //昇順ソートした数値を出力 
  printf("昇順ソートした数値\n");
  for (i=0; i<5; ++i)
    printf("%d\n", theta[i]);
}*/

  // 2x2 の行列
  cv::Mat m0 = (cv::Mat_<double>(3,1)<<1.0, 2.0, 3.0);
  cv::Mat m1 = (cv::Mat_<double>(2,2)<<1.0, 2.0, 3.0, 4.0);
  cv::Mat m2 = (cv::Mat_<double>(2,2)<<1.1, 2.1, 3.1, 4.1);
  cv::Mat m3 = (cv::Mat_<double>(2,2)<<1.2, 2.2, 3.2, 4.2);
  cv::Mat m4 = (cv::Mat_<cv::Vec2i>(2, 2) << cv::Vec2i(1,1), cv::Vec2i(2, 4), cv::Vec2i(3, 9), cv::Vec2i(4, 16));

  //cv::Mat mtest0 = (cv::Mat_<double>(3,1)<<1.0, 2.0, 3.0);
  //cv::Mat mtest2 = (cv::Mat_<double>(3,3)<<1, 2, 3, 4, 5, 6, 7, 8,9);
  //cv::Mat mtest1 = (cv::Mat_<double>(3,3)<<10, 20, 30, 40, 50, 60, 70, 80, 90);

  // mixChannlesに与えるのは，cv::Matのvectorでも配列でも構わない
  std::vector<cv::Mat> mv;
  mv.push_back(m1);
  mv.push_back(m2);
  mv.push_back(m3);
  
  // 出力用 Mat は，必ず割り当てる必要がある
  cv::Mat m_mixed1(2,2, CV_64FC2);
  cv::Mat m_mixed2(2,2, CV_64FC2);
  //int fromTo[] = {0,0, 1,1, 1,3, 2,2};
  int fromTo[] = {0,0, 1,1, 1,2, 2,3};
  // mixChannlesに与えるのは，cv::Matのvectorでも配列でも構わない
  std::vector<cv::Mat> mixv;
  mixv.push_back(m_mixed1);
  mixv.push_back(m_mixed2);

  m0.resize(4);//配列の行要素を増やす（でも０しか入れられない）
  cv::Mat n0 = (cv::Mat_<double>(4,1)<<0,0,0,1);
  cv::Mat n1 = m0+n0;//拡張した行列に1を足すことで斉次化

//値代入
cv::Mat_<double> mat_ = cv::Mat_<double>(2, 2);        
mat_ << 0.0, 1.0, 1.0, 1.0;
cv::Mat mat = mat_;

 std::cout << "mat=" << mat << std::endl<< std::endl; // mat_ としても出力は同じ
  
  // ミックス
  cv::mixChannels(mv, mixv, fromTo, 4);
  std::cout << "m0=" << m0 << std::endl << std::endl;
  std::cout << "m1=" << m1 << std::endl << std::endl;
  std::cout << "m2=" << m2 << std::endl << std::endl;
  std::cout << "m3=" << m3 << std::endl << std::endl;
  std::cout << "m4=" << m4 << std::endl << std::endl;
  std::cout << "n1=" << n1 << std::endl << std::endl;

  std::cout << "m_mixed1=" << m_mixed1 << std::endl << std::endl;
  std::cout << "m_mixed2=" << m_mixed2 << std::endl << std::endl;

  std::cout << "m_mixed2(0,0)=" << m_mixed2.at<double>(1, 0) << std::endl << std::endl;//行列の要素抜き出し（数値)



//行列の定義
  cv::Mat mtest0 = (cv::Mat_<double>(3,1)<<1.0, 2.0, 3.0);
  cv::Mat mtest1 = (cv::Mat_<double>(3,3)<<10, 20, 30, 40, 50, 60, 70, 80, 90);

//行列表示
  std::cout << "mtest0=" << mtest0 << std::endl << std::endl;
  std::cout << "mtest1=" << mtest1 << std::endl << std::endl;
//行列の要素抜き出し（数値)
  std::cout << "mtest1(1,0)=" << mtest1.at<double>(1, 0) << std::endl << std::endl;

  //配列の列の追加方法
  /*cv::Mat_<double> mtest3_ = cv::Mat_<double>(3, 4);        
  mtest3_ << mtest1.at<double>(0, 0), mtest1.at<double>(0, 1), mtest1.at<double>(0, 2), mtest0.at<double>(0, 0), mtest1.at<double>(1, 0), mtest1.at<double>(1, 1), mtest1.at<double>(1, 2),mtest0.at<double>(1, 0),mtest1.at<double>(2, 0),mtest1.at<double>(2, 1),mtest1.at<double>(2, 2),mtest0.at<double>(2, 0);
  cv::Mat mtest3 = mtest3_;
  std::cout << "mtest3=" << mtest3 << std::endl << std::endl;*/

  cv::Mat_<double> mtest3_ = cv::Mat_<double>(4, 4);        
  mtest3_ << mtest1.at<double>(0, 0), mtest1.at<double>(0, 1), mtest1.at<double>(0, 2), mtest0.at<double>(0, 0), mtest1.at<double>(1, 0), mtest1.at<double>(1, 1), mtest1.at<double>(1, 2),mtest0.at<double>(1, 0),mtest1.at<double>(2, 0),mtest1.at<double>(2, 1),mtest1.at<double>(2, 2),mtest0.at<double>(2, 0),0,0,0,1;
  cv::Mat mtest3 = mtest3_;
  std::cout << "mtest3=" << mtest3 << std::endl << std::endl;


  //斉次化
  mtest3.resize(5);//配列の行要素を増やす（でも０しか入れられない）
  cv::Mat n00 = (cv::Mat_<double>(4,4)<<0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1);
  //cv::Mat mtest4 = mtest3+n00;//拡張した行列に1を足すことで斉次化

  std::cout << "mtest3_2=" << mtest3 << std::endl << std::endl;
  std::cout << "n00=" << n00 << std::endl << std::endl;

  //std::cout << "mtest4=" << mtest4 << std::endl << std::endl;



  for(int i=0;i<=4;i++){
    for(int j=0;j<=3;j++){

    }

  }
  


    return 0;
}
