#include "ros/ros.h"
/*#include <iostream>
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
}

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
  //cv::Mat_<double> mtest3_ = cv::Mat_<double>(3, 4);        
  //mtest3_ << mtest1.at<double>(0, 0), mtest1.at<double>(0, 1), mtest1.at<double>(0, 2), mtest0.at<double>(0, 0), mtest1.at<double>(1, 0), mtest1.at<double>(1, 1), mtest1.at<double>(1, 2),mtest0.at<double>(1, 0),mtest1.at<double>(2, 0),mtest1.at<double>(2, 1),mtest1.at<double>(2, 2),mtest0.at<double>(2, 0);
  //cv::Mat mtest3 = mtest3_;
  //std::cout << "mtest3=" << mtest3 << std::endl << std::endl;

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




// 3x3 の行列
  cv::Mat m100 = (cv::Mat_<double>(3,3) << 1, 2, 3, 4, 5, 6, 7, 8, 9);

  // すべての行（=rawRange(0,3))
  std::cout << m100.rowRange(cv::Range::all()) << std::endl << std::endl;
  // [0,2) 行の範囲
  std::cout << m100.rowRange(cv::Range(0,2)) << std::endl << std::endl;
  // 0行目
  std::cout << m100.row(0) << std::endl << std::endl;
  
  // すべての列（=colRange(0,3))
  std::cout << m100.colRange(cv::Range::all()) << std::endl << std::endl;
  // [0,2) 列の範囲
  std::cout << m100.colRange(cv::Range(0,2)) << std::endl << std::endl;
  // 0列目
  std::cout << m100.col(0) << std::endl << std::endl;

  // 大きさと初期値を指定して行列を宣言する
	cv::Mat_<double> mat3(2, 3, 1.5);
 
	// 行列を表示する
	std::cout << "mat3=\n" << mat3 << std::endl;
 
	// 行列の大きさと型を表示する
	std::cout << "mat3の行数 = " << mat3.rows << std::endl;
	std::cout << "mat3の列数 = " << mat3.cols << std::endl;
	std::cout << "mat3の型 = " << mat3.type() << std::endl;

  	// 零行列、単位行列の設定
	cv::Mat_<double> mat4, mat5, mat6,A,mat3A;
	mat4 = cv::Mat_<double>::zeros(3, 1);  // 零行列
	mat5 = cv::Mat_<double>::eye(3, 1);	  // 単位行列
	mat6 = cv::Mat_<double>::ones(3, 1);   // 全要素が1の行列
  A = (cv::Mat_<double>(3, 3) << 1.93/326.15115000485503, 0, 326.15115000485503, 0, 1.93/244.02271379203739, 244.02271379203739,0,0,1);//内部パラメータ（詳しくはM2 6月研究ノート)

  mat3A=mat3*A(0,0);//要素を取り出し行列全体にかける
  	// 行列を表示する
	std::cout << "mat4=\n" << mat4 << std::endl;
	std::cout << "mat5=\n" << mat5 << std::endl;
	std::cout << "mat6=\n" << mat6 << std::endl;
  std::cout << "A=\n" << A << std::endl;
  std::cout << "mat3A=\n" << mat3A << std::endl;

cv::Mat_<double> testMat1;
  for(int i=0; i<10; i++){
    testMat1 = cv::Mat_<double>::zeros(3, i);  // 零行列
    std::cout << "testMat1=\n" << testMat1<< std::endl;
  }
   cv::Mat kal1 = (cv::Mat_<double>(3,3) << 1, 2, 3, 4, 5, 6, 7, 8, 9);
   cv::Mat kal2 = (cv::Mat_<double>(3,3) << 0, 1, 2, 0, 1, 2, 0, 1, 2);
   cv::Mat kal3 = (cv::Mat_<double>(3,3) << 1, 1, 1, 1, 1, 1, 1, 1, 1);
   cv::Mat kal4 = (cv::Mat_<double>(3,3) << 0, 0, 0, 0, 0, 0, 0, 0, 0);
   cv::Mat kal5 = (cv::Mat_<double>(3,4) << 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5);
   std::cout << "初期値kal1=\n" << kal1 << std::endl;
   std::cout << "初期値kal2=\n" << kal2 << std::endl;
   std::cout << "初期値kal3=\n" << kal3 << std::endl;
   std::cout << "初期値kal4=\n" << kal4 << std::endl;
   std::cout << "初期値kal5=\n" << kal5 << std::endl;

   kal5 = cv::Mat_<float>::zeros(3, 3); //状態の推定値(x'(k))
   
   std::cout << "リサイズkal5=\n" << kal5 << std::endl;

  // mat2の第2行をmat1の第1列に代入【正しい書き方】
	//kal2.col(2).copyTo(kal1.col(0));
	//std::cout << "代入後kal1=\n" << kal1 << std::endl;	// 代入できている
  int k=0;

   for (int i = 0; i < 3; i++){
			      std::cout <<"i="<<i<<",k="<<k<< std::endl;//0以外の時は変なマークがでる
            kal2.col(i).copyTo(kal1.col(k));
            kal2.col(i).copyTo(kal5.col(k));
            kal4.col(i).copyTo(kal3.col(k++));
            std::cout << "代入後kal1=\n" << kal1 << std::endl;	// 代入できている
            std::cout << "代入後kal3=\n" << kal3 << std::endl;	// 代入できている
            std::cout << "代入後kal5=\n" << kal5 << std::endl;	// 代入できている
            std::cout << "k=" << k << std::endl;	// 代入できている
            
          }


      // CV32SC2, 3x3 行列
  // 初期化＋reshapeを用いた変形
  cv::Mat m0123 = (cv::Mat_<int>(3,6) << 1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18);
  cv::Mat m1123 = m0123.reshape(2);

  std::cout << "m0123=\n" << m0123 << std::endl;
  std::cout << "m1123=\n" << m1123 << std::endl;



  
  


    return 0;
}*/

#include <stdio.h>
  
int main(void)
{
 
  int i, j, tmp;
 
  /* 数値を格納する配列 */
  int number[100];
 
  /* 数値の総数を入力 */
  int total;
  printf("入力する数値の総数 = ");
  scanf("%d", &total);
 
  /* 配列に格納する数値を入力 */
  printf("%d個の数値を入力 \n", total);
  for (i=0; i<total; ++i)
    scanf("%d", &number[i]);
 
  /* 数値を昇順にソート */
  for (i=0; i<total; ++i) {
    for (j=i+1; j<total; ++j) {
      if (number[i] > number[j]) {
        tmp =  number[i];
        number[i] = number[j];
        number[j] = tmp;
      }
    }
  }
 
  /* 昇順ソートした数値を出力 */
  printf("昇順ソートした数値\n");
  for (i=0; i<total; ++i)
    printf("%d\n", number[i]);
}

