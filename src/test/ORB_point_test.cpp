//特徴点検出のテストプログラム
#include <ros/ros.h>
#include <sensor_msgs/Image.h>//センサーデータ形式ヘッダ
#include <cv_bridge/cv_bridge.h>//画像変換のヘッダ
#include <sensor_msgs/image_encodings.h>//エンコードのためのヘッダ
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <math.h>
#include <highgui.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/ximgproc/fast_line_detector.hpp>//FLD


ros::Subscriber sub;//データをsubcribeする奴
std::string win_src = "src";//カメラ画像
std::string win_depth = "depth";//深度画像（修正前）
std::string win_depth2 = "depth2";//深度画像（修正前）+FLD線
std::string win_depth3 = "depth3";//深度画像（修正後）
std::string win_depth4 = "depth4";//深度画像（修正後）+FLD線
std::string win_edge = "edge";
std::string win_dst = "dst";//カメラ画像+FLDの線表示
std::string win_line = "line";//FLDの線を表示
std::string win_line2 = "line2";//FLD+確率Houghの線を表示
std::string win_prev = "prev";
std::string win_curr = "curr";

using namespace std;
using namespace cv;

// 抽出する画像の輝度値の範囲を指定
#define B_MAX 255
#define B_MIN 150
#define G_MAX 255
#define G_MIN 180
#define R_MAX 255
#define R_MIN 180
// reset == TRUE のとき特徴点検出を行う
// 最初のフレームで必ず特徴点検出を行うように、初期値を TRUE にする
bool reset = true;
int kaisu =0;
int best = 100;
// image_curr:  現在の入力画像、    image_prev:  直前の入力画像
// points_curr: 現在の特徴点リスト、points_prev: 直前の特徴点リスト
cv::Mat frame,image_curr, image_prev,img_dst,img_dst2,img_match;
std::vector<cv::Point2f> points_prev, points_curr;
std::vector<cv::Point2f> points_src, points_dst;//対応点の保存


//コールバック関数
void callback(const sensor_msgs::Image::ConstPtr& rgb_msg,const sensor_msgs::Image::ConstPtr& depth_msg)
{
	//変数宣言
	cv_bridge::CvImagePtr bridgeImage;//クラス::型//cv_brigeは画像変換するとこ
	cv_bridge::CvImagePtr bridgedepthImage;//クラス::型//cv_brigeは画像変換するとこ
	cv::Mat RGBimage,depthimage,image;//opencvの画像
	ROS_INFO("callback_functionが呼ばれたよ");

	try{//MAT形式変換
	   bridgeImage=cv_bridge::toCvCopy(rgb_msg, sensor_msgs::image_encodings::BGR8);//MAT形式に変える
	   ROS_INFO("callBack");//printと秒数表示
	}
	//エラー処理
	catch(cv_bridge::Exception& e) {//エラー処理(失敗)成功ならスキップ
	    std::cout<<"depth_image_callback Error \n";
	    ROS_ERROR("Could not convert from '%s' to 'BGR8'.",rgb_msg->encoding.c_str());
	    return ;
	}
	 try{//MAT形式変換
	   bridgedepthImage=cv_bridge::toCvCopy(depth_msg, sensor_msgs::image_encodings::TYPE_32FC1);//MAT形式に変える
	   ROS_INFO("callBack");}//printと秒数表示

	//エラー処理
	catch(cv_bridge::Exception& e) {//エラー処理(失敗)成功ならスキップ
	    std::cout<<"depth_image_callback Error \n";
	    ROS_ERROR("Could not convert from '%s' to '32FC1'.",depth_msg->encoding.c_str());
	    return ;}	

	image = bridgeImage->image.clone();//image変数に変換した画像データを代入
    depthimage = bridgedepthImage->image.clone();//image変数に変換した画像データを代入
	cv::imshow(win_src, image);
	int req;
		
	image.copyTo(frame);//ここ間違えていたので注意
	image.copyTo(image_curr);//ここ間違えていたので注意
	image_curr.copyTo(img_dst);//ここ間違えていたので注意
	if(kaisu!=0){
		image_prev.copyTo(img_dst2);//ここ間違えていたので注意
	}

	if(kaisu!=0){
		// (1) 特徴点抽出(ORB)
		
		// # ORB::create(検出特徴点数, scale factor, ...)
		//cv::ORB::create	(	
		//		int 	nfeatures = 500,
		//		float 	scaleFactor = 1.2f,
		//		int 	nlevels = 8,
		//		int 	edgeThreshold = 31,
		//		int 	firstLevel = 0,
		//		int 	WTA_K = 2,
		//		int 	scoreType = ORB::HARRIS_SCORE,
		//		int 	patchSize = 31,
		//		int 	fastThreshold = 20 
		//)

		cv::Ptr<cv::ORB> detector = cv::ORB::create(500, 1.2f, 2);// FeatureDetectorオブジェクトの生成
		cv::Ptr<cv::ORB> extractor = cv::ORB::create(500, 1.2f, 2);// DescriptionExtractorオブジェクトの生成
		cv::BFMatcher matcher(cv::NORM_HAMMING);// DescriptorMatcherオブジェクトの生成
    	std::vector<cv::KeyPoint> kpts1, kpts2;// 特徴点情報を格納するための変数
    	detector->detect(image_curr, kpts1);// 特徴点抽出の実行
    	detector->detect(image_prev, kpts2);
    	cv::Mat desc1, desc2;// 画像の特徴情報を格納するための変数
    	extractor->compute(image_curr, kpts1, desc1);// 特徴記述の計算を実行
    	extractor->compute(image_prev, kpts2, desc2);

    	std::vector<cv::DMatch> matches;// 特徴点のマッチング情報を格納する変数
    	matcher.match(desc1, desc2, matches);// 特徴点マッチングの実行
		ROS_INFO("maxtutinngu");//printと秒数表示
	
		std::cout << "best = " << best << std::endl;
		std::cout << "match size = " << matches.size() << std::endl;
		if (matches.size() < best) {
			std::cout << "few matchpoints" << std::endl;
		}
	
		// 上位best個を採用
		std::nth_element(begin(matches), begin(matches) + best - 1, end(matches));
		matches.erase(begin(matches) + best, end(matches));
		std::cout << "matchs size = " << matches.size() << std::endl;
		
		for(size_t i = 0, end = matches.size(); i != end; ++i) {
    		//std::cout <<"matches("<<i<<").distance="<< matches.at(i).distance << "\n";//特徴量記述子の距離
    		//std::cout <<"matches("<<i<<").trainIdx="<< matches.at(i).trainIdx << "\n";//train 画像の特徴量記述子のインデックス
    		//std::cout <<"matches("<<i<<").queryIdx="<< matches.at(i).queryIdx << "\n";//query 画像の特徴量記述子のインデックス
			//trainIdxとqueryIdxはdetectAndComputeで出力されたキーポイントのインデックス
			cv::circle(img_dst, kpts1.at(matches.at(i).queryIdx).pt, 3, Scalar(0,0,255), -1, cv::LINE_AA);
      		cv::circle(img_dst, kpts2.at(matches.at(i).trainIdx).pt, 3, Scalar(255,255,255), -1, cv::LINE_AA);
	    	cv::line(img_dst,kpts1.at(matches.at(i).queryIdx).pt,kpts2.at(matches.at(i).trainIdx).pt,cv::Scalar(255,0,0), 1, cv::LINE_AA);//線を描写する
		}
		cv::circle(img_dst, kpts1.at(matches.at(0).queryIdx).pt, 6, Scalar(0,255,0), -1, cv::LINE_AA);
      	cv::circle(img_dst, kpts2.at(matches.at(0).trainIdx).pt, 6, Scalar(255,0,0), -1, cv::LINE_AA);

		//keypointオブジェクトは以下の属性を持っています。
		//・pt：キーポイントの座標
		//・size：キーポイント周辺の重要領域の直径
		//・angle：計算されたキーポイントの方向（計算できない場合は -1 ）
		//・response：最も強いキーポイントが選択されたときの応答
		//・octave：キーポイントが抽出されるオクターブ（ピラミッドの階層）
		//・class_id：オブジェクトクラス

		//マッチング対応点
		std::cout << "query: " << kpts1.at(matches.at(0).queryIdx).pt << std::endl;
		std::cout << "train: " << kpts2.at(matches.at(0).trainIdx).pt << std::endl;
      	//cv::circle(img_dst, kpts1.at(matches.at(0).queryIdx).pt, 6, Scalar(0,0,255), -1, cv::LINE_AA);
      	//cv::circle(img_dst2, kpts2.at(matches.at(0).trainIdx).pt, 6, Scalar(0,0,255), -1, cv::LINE_AA);

		//記述子の個数
		std::cout << "kpts1.size() = " << kpts1.size() << std::endl;
		std::cout << "kpts2.size() = " << kpts2.size() << std::endl;

		//通常のfor分で取り出す場合
		//for(size_t i = 0, end = kpts1.size(); i != end; ++i) {
    	//	std::cout <<"kpts1("<<i<<")="<< kpts1.at(i).pt << "\n";
		//}
		//for(size_t i = 0, end = kpts2.size(); i != end; ++i) {
    	//	std::cout <<"kpts2("<<i<<")="<< kpts2.at(i).pt << "\n";
		//}

		// 特徴点の対応を表示
		//cv::drawMatches(image_curr, kpts1, image_prev, kpts2, matches, img_match);
    	std::vector<cv::KeyPoint> kpts0;// 特徴点情報を格納するための変数
    	std::vector<cv::DMatch> matches0;// 特徴点のマッチング情報を格納する変数

		cv::drawMatches(image_curr, kpts0, image_prev, kpts0, matches0, img_match);
		cv::imshow("matchs", img_match);

	
		// 特徴点をvectorにまとめる
		for (int i = 0; i < matches.size(); i++) {
			points_src.push_back(kpts1[matches[i].queryIdx].pt);//image_curr
			points_dst.push_back(kpts2[matches[i].trainIdx].pt);//image_prev
		}
		std::cout << "kpts1.size() = " << kpts1.size() << std::endl;
		std::cout << "kpts2.size() = " << kpts2.size() << std::endl;
		std::cout << "points_src.size() = " << points_src.size() << std::endl;
		std::cout << "points_dst.size() = " << points_dst.size() << std::endl;


	}

	cv::imshow("win_dst", img_dst);
	cv::imshow("win_curr", image_curr);//今の画像

	if (kaisu != 0) {//初回は直前の画像がないため考慮
		cv::imshow("win_prev", image_prev);//一つ前の画像像
		std::cout <<"でてるよ"<< std::endl;//特徴点の座標
		cv::imshow("win_dst2", img_dst2);//一つ前の画像像
	}
 
	int key = cv::waitKey(30);
	if (key == 'r') {reset = true;}// Rキーが押されたら特徴点を再検出
		
	cv::swap(image_curr, image_prev);// image_curr を image_prev に移す（交換する）
	kaisu++;
	cv::waitKey(1);//ros::spinにジャンプする
}

//メイン関数
int main(int argc,char **argv){

	ros::init(argc,argv,"opencv_main");//rosを初期化
	ros::NodeHandle nhSub;//ノードハンドル
	
	//subscriber関連
	//message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nhSub, "/robot1/camera/color/image_raw", 1);
	//message_filters::Subscriber<sensor_msgs::Image> depth_sub(nhSub, "/robot1/camera/depth/image_rect_raw", 1);

	message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nhSub, "/camera/color/image_raw", 1);//センサーメッセージを使うときは対応したヘッダーが必要
	message_filters::Subscriber<sensor_msgs::Image> depth_sub(nhSub, "/camera/aligned_depth_to_color/image_raw", 1);

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,sensor_msgs::Image> MySyncPolicy;	
	message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10),rgb_sub, depth_sub);
	sync.registerCallback(boost::bind(&callback,_1, _2));

	ros::spin();//トピック更新待機
			
	return 0;
}