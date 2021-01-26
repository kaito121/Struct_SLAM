//rosのヘッダ
#include <ros/ros.h>
#include <sensor_msgs/Image.h>//センサーデータ形式ヘッダ
#include <cv_bridge/cv_bridge.h>//画像変換のヘッダ
#include <sensor_msgs/image_encodings.h>//エンコードのためのヘッダ
#include <opencv2/opencv.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <opencv/highgui.h>


ros::Subscriber sub;//データをsubcribeする奴
std::string win_src = "src";//nameteigi
std::string win_src2 = "src2";//nameteigi
std::string win_dst2 = "dst2";
std::string win_dst3 = "dst3";
std::string file_src = "test1.png";

using namespace std;
using namespace cv;

// 抽出する画像の輝度値の範囲を指定
#define B_MAX 255
#define B_MIN 150
#define G_MAX 255
#define G_MIN 180
#define R_MAX 255
#define R_MIN 180

int best = 30;


//コールバック関数

void callback(const sensor_msgs::Image::ConstPtr& rgb_msg,const sensor_msgs::Image::ConstPtr& depth_msg)
{
	//変数宣言
	cv_bridge::CvImagePtr bridgeRGBImage;//クラス::型//cv_brigeは画像変換するとこ
    cv_bridge::CvImagePtr bridgedepthImage;//クラス::型//cv_brigeは画像変換するとこ
    cv::Mat RGBimage,RGBimage2;//opencvの画像
    cv::Mat depthimage;//opencvの画像
	ROS_INFO("callback_functionが呼ばれたよ");
	
    try{//MAT形式変換
       bridgeRGBImage=cv_bridge::toCvCopy(rgb_msg, sensor_msgs::image_encodings::BGR8);//MAT形式に変える
       ROS_INFO("callBack");//printと秒数表示
    }
	//エラー処理
    catch(cv_bridge::Exception& e) {//エラー処理(失敗)成功ならスキップ
        std::cout<<"RGB_image_callback Error \n";
        ROS_ERROR("Could not convert from '%s' to 'BGR8'.",
        rgb_msg->encoding.c_str());
        return ;
    }

    try{//MAT形式変換
       bridgedepthImage=cv_bridge::toCvCopy(depth_msg, sensor_msgs::image_encodings::TYPE_32FC1);//MAT形式に変える
       ROS_INFO("callBack");//printと秒数表示
    }
	//エラー処理
    catch(cv_bridge::Exception& e) {//エラー処理(失敗)成功ならスキップ
        std::cout<<"depth_image_callback Error \n";
        ROS_ERROR("Could not convert from '%s' to '32FC1'.",
        depth_msg->encoding.c_str());
        return ;
    }

    RGBimage2 = cv::imread("test1.png",1); // 画像読み込み
    

    RGBimage = bridgeRGBImage->image.clone();//image変数に変換した画像データを代入
    depthimage = bridgedepthImage->image.clone();//image変数に変換した画像データを代入



//ここに処理項目
	cv::namedWindow(win_src, cv::WINDOW_AUTOSIZE);
	cv::namedWindow(win_src2, cv::WINDOW_AUTOSIZE);
  

    cv::Mat img_src[2], img_srcw[2], img_match, img_per, img_reg;
	//std::string filename[2] = { "/image/stb1.jpg", "/image/stb2.jpg" };
	cv::Scalar color[2] = { cv::Scalar(0, 0, 255), cv::Scalar(255, 0, 0) };
	


     // 画像読み込み
	img_src[0] = RGBimage; // 画像読み込み
	img_src[1] = cv::imread("test1.png",1); // 画像読み込み
	cv::rectangle(img_src[0], cv::Point(0, 0), cv::Point(img_src[0].cols, img_src[0].rows), color[0], 2); // 外枠
    cv::rectangle(img_src[1], cv::Point(0, 0), cv::Point(img_src[1].cols, img_src[1].rows), color[1], 2); // 外枠
	img_srcw[0] = cv::Mat::zeros(img_src[0].size() * 2, img_src[0].type());
    img_srcw[1] = cv::Mat::zeros(img_src[1].size() * 2, img_src[1].type());
	cv::Mat roi1 = img_srcw[0](cv::Rect(img_srcw[0].cols / 4, img_srcw[0].rows / 4, img_src[0].cols, img_src[0].rows));
    cv::Mat roi2 = img_srcw[1](cv::Rect(img_srcw[1].cols / 4, img_srcw[1].rows / 4, img_src[1].cols, img_src[1].rows));
	img_src[0].copyTo(roi1); // 縦横倍のMatの中央にコピー
    img_src[1].copyTo(roi2); // 縦横倍のMatの中央にコピー
	
	cv::imshow("img_src[0]", img_srcw[0]);
	cv::imshow("img_src[1]", img_srcw[1]);
	

	/*// (1) 特徴点抽出(AKAZE)
	cv::Ptr<cv::AKAZE> detector = cv::AKAZE::create();
	std::vector<cv::KeyPoint> kpts1, kpts2;
	cv::Mat desc1, desc2;
	detector->detectAndCompute(img_srcw[0], cv::noArray(), kpts1, desc1);
	detector->detectAndCompute(img_srcw[1], cv::noArray(), kpts2, desc2);*/

    // (1) 特徴点抽出(ORB)
	// FeatureDetectorオブジェクトの生成
	cv::Ptr<cv::ORB> detector = cv::ORB::create();
	// DescriptionExtractorオブジェクトの生成
	cv::Ptr<cv::ORB> extractor = cv::ORB::create();
    // DescriptorMatcherオブジェクトの生成
	cv::BFMatcher matcher(cv::NORM_HAMMING);
	// 特徴点情報を格納するための変数
    std::vector<cv::KeyPoint> kpts1, kpts2;
	// 特徴点抽出の実行
    detector->detect(img_srcw[0], kpts1);
    detector->detect(img_srcw[1], kpts2);
	 // 画像の特徴情報を格納するための変数
    cv::Mat desc1, desc2;
	// 特徴記述の計算を実行
    extractor->compute(img_srcw[0], kpts1, desc1);
    extractor->compute(img_srcw[1], kpts2, desc2);
	// 特徴点のマッチング情報を格納する変数
    std::vector<cv::DMatch> matches;
	// 特徴点マッチングの実行
    matcher.match(desc1, desc2, matches);
	
	// 特徴点が少なすぎる場合は停止する
	/*std::cout <<"kept1.size=" <<kpts1.size()<<" "<<"kpts2.size="<< kpts2.size() << std::endl;
	if(kpts1.size() < best || kpts2.size() < best) {
		std::cout << "few keypoints : "
				<< kpts1.size() << " or " << kpts2.size() << "< " << best << std::endl;
		return 0;
	}*/
	
	/*// (2) 得られた特徴点間のマッチング
	cv::BFMatcher matcher(cv::NORM_HAMMING);
	std::vector<cv::DMatch> matches;
	matcher.match(desc1, desc2, matches);*/
	
	std::cout << "best = " << best << std::endl;
	std::cout << "match size = " << matches.size() << std::endl;
	if (matches.size() < best) {
		std::cout << "few matchpoints" << std::endl;
	}
	
	// 上位best個を採用
	std::nth_element(begin(matches), begin(matches) + best - 1, end(matches));
	matches.erase(begin(matches) + best, end(matches));
	std::cout << "matchs size = " << matches.size() << std::endl;
	
	// 特徴点の対応を表示
	cv::drawMatches(img_srcw[0], kpts1, img_srcw[1], kpts2, matches, img_match);
	cv::imshow("matchs", img_match);
	
	// 特徴点をvectorにまとめる
	std::vector<cv::Point2f> points_src, points_dst;
	for (int i = 0; i < matches.size(); i++) {
		points_src.push_back(kpts1[matches[i].queryIdx].pt);
		points_dst.push_back(kpts2[matches[i].trainIdx].pt);
	}
	
	// (3) マッチング結果から，F行列を推定する
	cv::Mat F = cv::findFundamentalMat(points_src, points_dst);
	std::cout << "F=" << F << std::endl;
	
	// (4) カメラの内部パラメータが既知の場合はE行列を計算し，外部パラメータを推定する
	// カメラ内部パラメータ読み込み
	cv::Mat A;
	//cv::FileStorage fs("realsense_para.xml", cv::FileStorage::READ);
	cv::FileStorage fs;
    fs.open("realsense_para.xml", cv::FileStorage::READ);
	
	fs["intrinsic"]>>A;
	
	//fs.release();
	
	std::cout << "A=" << A << std::endl;

	//fs.release();
	
	// E行列の計算
	cv::Mat E = cv::findEssentialMat(points_src, points_dst, A);
	
	// 外部パラメータ（回転，並進ベクトル）の計算
	cv::Mat R, t ,Rt;
	cv::recoverPose(E, points_src, points_dst, A, R, t);

	std::cout << "E=" << E << std::endl;
	std::cout << "R=" << R << std::endl;
	std::cout << "t=" << t << std::endl;

	//行列の要素抜き出し（数値)
  std::cout << "R(0,0)=" << R.at<double>(0, 0) << std::endl << std::endl;

   //配列の列の追加方法
  cv::Mat_<double> mtest3_ = cv::Mat_<double>(3, 4);        
  mtest3_ << R.at<double>(0, 0), R.at<double>(0, 1), R.at<double>(0, 2), t.at<double>(0, 0), R.at<double>(1, 0), R.at<double>(1, 1), R.at<double>(1, 2),t.at<double>(1, 0),R.at<double>(2, 0),R.at<double>(2, 1),R.at<double>(2, 2),t.at<double>(2, 0);
  cv::Mat mtest3 = mtest3_;
  std::cout << "mtest3=" << mtest3 << std::endl << std::endl;
  

	//処理結果表示もここで行うこともできる（別関数でもあり
    cv::imshow(win_src, RGBimage);
    cv::imshow(win_src2, RGBimage2);
	cv::imwrite(file_src, RGBimage);
	cv::waitKey(1);



   //ros::spinにジャンプする
}

//メイン関数


int main(int argc,char **argv){
	ros::init(argc,argv,"opencv_main1");
    	
	ros::NodeHandle nhSub;
	message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nhSub, "/robot1/camera/color/image_raw", 1);
	message_filters::Subscriber<sensor_msgs::Image> depth_sub(nhSub, "/robot1/camera/depth/image_rect_raw", 1);

	typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,sensor_msgs::Image> MySyncPolicy;	
	message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10),rgb_sub, depth_sub);
	sync.registerCallback(boost::bind(&callback,_1, _2));
    	
	ros::spin();
	
	return 0;
}
