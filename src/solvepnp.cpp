#include <iostream>
#include <vector>

#include <opencv2/opencv.hpp>

void genRT(cv::Mat& rvect, cv::Mat& tvect);
void getCorrespondingPairs(cv::Mat K, cv::Mat d, cv::Mat rvec, cv::Mat tvec, std::vector<cv::Point3d>& op, std::vector<cv::Point2d>& ip);

int main(int argc, char** argv)
{
  cv::Mat K = (cv::Mat_<double>(3, 3) << 300, 0, 0, 0, 300, 0, 0, 0, 1);
  cv::Mat distortion = (cv::Mat_<double>(4, 1) << 0, 0, 0, 0);
  cv::Mat rvec(3, 1, CV_64FC1);
  cv::Mat tvec(3, 1, CV_64FC1);

  std::vector<cv::Point3d> objectPoints;
  std::vector<cv::Point2d> imagePoints;

  //カメラ位置・姿勢、対応点情報の生成
  genRT(rvec, tvec);
  getCorrespondingPairs(K, distortion, rvec, tvec, objectPoints, imagePoints);

  cv::Mat rvec_est;
  cv::Mat tvec_est;

  //誤対応の排除なし
  cv::solvePnP(objectPoints, imagePoints, K, distortion, rvec_est, tvec_est);
  //RANSACによる誤対応の排除あり
  //cv::solvePnPRansac(objectPoints, imagePoints, K, distortion, rvec_est, tvec_est, false, 100, 3.0, 100);

  cv::Mat rmat;
  cv::Rodrigues(rvec, rmat);
  std::cout << "rmat(ground truth): " << rmat << "\n" << "tvec(ground truth): " << tvec << std::endl;
  cv::Mat rmat_est;
  cv::Rodrigues(rvec_est, rmat_est);
  std::cout << "rmat(estimated): " << rmat_est << "\n" << "tvec(estimated): " << tvec_est << std::endl;

  return 0;
}

//カメラの位置・姿勢を生成
void genRT(cv::Mat& rvect, cv::Mat& tvect)
{
  rvect.at<double>(0) = (double)rand()/RAND_MAX;
  rvect.at<double>(0) = (double)rand()/RAND_MAX;
  rvect.at<double>(0) = (double)rand()/RAND_MAX;

  tvect.at<double>(0) = (double)rand()/RAND_MAX*10;
  tvect.at<double>(1) = (double)rand()/RAND_MAX*10;
  tvect.at<double>(2) = (double)rand()/RAND_MAX*10;
}
//2D-3Dの対応関係を生成
void getCorrespondingPairs(cv::Mat K, cv::Mat d, cv::Mat rvec, cv::Mat tvec, std::vector<cv::Point3d>& op, std::vector<cv::Point2d>& ip)
{
  cv::Mat rmat(3, 3, CV_64FC1);
  cv::Mat ep = cv::Mat::eye(4, 4, CV_64FC1);

  cv::Rodrigues(rvec, rmat);
  for(int i = 0; i < 3; ++i)
    for(int j = 0; j < 3; ++j)
    {
      ep.at<double>(i, j) = rmat.at<double>(i, j);
    }
  ep.at<double>(0, 3) = tvec.at<double>(0);
  ep.at<double>(1, 3) = tvec.at<double>(1);
  ep.at<double>(2, 3) = tvec.at<double>(2);

#define NUMBER_OF_POINTS 30
  for(int i = 0; i < NUMBER_OF_POINTS; ++i)
  {
    cv::Point2d p;
    p.x = (double)rand()/RAND_MAX*640;
    p.y = (double)rand()/RAND_MAX*480;
    ip.push_back( p );
  }
  cv::convertPointsToHomogeneous(ip, op);

  for(int i = 0; i < NUMBER_OF_POINTS; ++i)
  {
    cv::Mat p( op[i] );
    p = K.inv()*p;

    double dist = (double)rand()/RAND_MAX*1000;
    p *= dist;

    cv::Mat p_homo(4, 1, CV_64FC1);
    p_homo.at<double>(0) = p.at<double>(0);
    p_homo.at<double>(1) = p.at<double>(1);
    p_homo.at<double>(2) = p.at<double>(2);
    p_homo.at<double>(3) = 1.0;

    cv::Mat p_w(4, 1, CV_64FC1);
    p_w = ep.inv()*p_homo;

    op[i].x = p_w.at<double>(0);
    op[i].y = p_w.at<double>(1);
    op[i].z = p_w.at<double>(2);
  }
}
