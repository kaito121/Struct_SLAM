//特徴点検出のテストプログラム
#include <ros/ros.h>
#include <sensor_msgs/Image.h>//センサーデータ形式ヘッダ
#include <cv_bridge/cv_bridge.h>//画像変換のヘッダ
#include <sensor_msgs/image_encodings.h>//エンコードのためのヘッダ
#include <sensor_msgs/CameraInfo.h>//camera_infoを獲得するためのヘッダー
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <math.h>
#include <highgui.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/ximgproc/fast_line_detector.hpp>//FLD
#include <geometry_msgs/PoseStamped.h>//tf
#include <tf/transform_broadcaster.h>//tf
#include <nav_msgs/Path.h>//経路情報を記録する


ros::Subscriber sub;//データをsubcribeする奴
ros::Publisher pub_plan;//カメラ経路送信用

std::string source_frame = "map";//mapフレーム
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

// reset == TRUE のとき特徴点検出を行う
// 最初のフレームで必ず特徴点検出を行うように、初期値を TRUE にする
bool reset = true;
int kaisu =0,PT_kosuu=0;
int best = 100;
// image_curr:  現在の入力画像、    image_prev:  直前の入力画像
// points_curr: 現在の特徴点リスト、points_prev: 直前の特徴点リスト
cv::Mat frame,image_curr, image_prev,img_dst,img_dst2,img_match,depthimage_curr,depthimage_prev;
std::vector<cv::Point2f> points_prev, points_curr;
std::vector<cv::Point2f> points_src, points_dst;//対応点の保存
vector<cv::Point3f> camera_point_p,camera_point_c;//特徴点定義
float camera_prmX=0,camera_prmY=0,camera_prmZ=0;//カメラの移動運動パラメータ

cv::Mat_<float>ALL_At;//外部パラメータ推定
cv::Mat_<float>ALL_Et;//外部パラメータ推定
cv::Mat_<float> ALL_Tt=cv::Mat_<float>(6, 1);//教科書運動パラメータ
cv::Mat_<float> ALL_Ft;//外部パラメータ回転行列(マーカー+特徴点)

sensor_msgs::CameraInfo camera_info;//CameraInfo受け取り用
geometry_msgs::Pose camera_base_pose;

nav_msgs::Path path;//カメラ経路表示設定
geometry_msgs::PoseStamped pose;

//コールバック関数
void callback(const sensor_msgs::Image::ConstPtr& rgb_msg,const sensor_msgs::Image::ConstPtr& depth_msg, const sensor_msgs::CameraInfo::ConstPtr& cam_info)
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
  	camera_info=*cam_info;//CameraInfo受け取り
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
		std::cout << "query(x): " << kpts1.at(matches.at(0).queryIdx).pt.x << std::endl;
		std::cout << "query(y): " << kpts1.at(matches.at(0).queryIdx).pt.y << std::endl;
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
		cv::drawMatches(image_curr, kpts1, image_prev, kpts2, matches, img_match);
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
		double depth_point_curr[500],depth_point_prev[500];
    	camera_point_c.resize(matches.size());//要素数初期設定
    	camera_point_p.resize(matches.size());//要素数初期設定

		int depth_point_ok=0;
		//カメラ座標系変換
		for (int i = 0; i < matches.size(); i++) {
			//画像→カメラ座標変換----------------------------------------------------------------------
      		depth_point_curr[i] = depthimage.at<float>(cv::Point(kpts1.at(matches.at(i).queryIdx).pt));
      		depth_point_prev[i] = depthimage_prev.at<float>(cv::Point(kpts2.at(matches.at(i).queryIdx).pt));
			std::cout << "depth_point_curr["<<i<<"] = " << depth_point_curr[i] << std::endl;
			std::cout << "depth_point_prev["<<i<<"] = " << depth_point_prev[i] << std::endl;

			//Depthが取得できない特徴点を削除する+Depthの外れ値を除く
      		if(depth_point_curr[i]>0.001&&depth_point_curr[i]<10000){
      			if(depth_point_prev[i]>0.001&&depth_point_prev[i]<10000){
      				camera_point_c[depth_point_ok].x = depth_point_curr[i] * ((kpts1.at(matches.at(i).queryIdx).pt.x - camera_info.K[2]) / camera_info.K[0])/1000;//メートル表示変換
      				camera_point_c[depth_point_ok].y = depth_point_curr[i] * ((kpts1.at(matches.at(i).queryIdx).pt.y - camera_info.K[5]) / camera_info.K[4])/1000;
      				camera_point_c[depth_point_ok].z = depth_point_curr[i]/1000;
      				std::cout << "特徴点のカメラ座標:camera_point_c[depth_point_ok="<<depth_point_ok<<"]={"<< camera_point_c[depth_point_ok].x <<","<<camera_point_c[depth_point_ok].y<<","<<camera_point_c[depth_point_ok].z<<"}"<< std::endl;

      				camera_point_p[depth_point_ok].x = depth_point_prev[i] * ((kpts2.at(matches.at(i).queryIdx).pt.x - camera_info.K[2]) / camera_info.K[0])/1000;//メートル表示変換
      				camera_point_p[depth_point_ok].y = depth_point_prev[i] * ((kpts2.at(matches.at(i).queryIdx).pt.y - camera_info.K[5]) / camera_info.K[4])/1000;
      				camera_point_p[depth_point_ok].z = depth_point_prev[i]/1000;
      				std::cout << "特徴点のカメラ座標:camera_point_p[depth_point_ok="<<depth_point_ok<<"]={"<< camera_point_p[depth_point_ok].x <<","<<camera_point_p[depth_point_ok].y<<","<<camera_point_p[depth_point_ok].z<<"}"<< std::endl;
      		  		depth_point_ok=depth_point_ok+1;//Depth取得可能の個数をカウント
				}
			}
		}
		PT_kosuu=0;
		std::cout << "depth_point_ok= " <<depth_point_ok<<  std::endl;
		ALL_At=cv::Mat_<float>(depth_point_ok*3, 6);
    	ALL_Et=cv::Mat_<float>(depth_point_ok*3, 1);

		//最小二乗法を用いた外部パラメータの算出(日高手法 特徴点)-----------------------------------------------------------------------------
    	for(int i=0;i<depth_point_ok;i++){
    	  ALL_At(PT_kosuu,0)=-1,  ALL_At(PT_kosuu,1)=0,   ALL_At(PT_kosuu,2)=0,   ALL_At(PT_kosuu,3)=0,   ALL_At(PT_kosuu,4)=-camera_point_p[i].z,    ALL_At(PT_kosuu,5)=0;
    	  ALL_At(PT_kosuu+1,0)=0, ALL_At(PT_kosuu+1,1)=-1,ALL_At(PT_kosuu+1,2)=0, ALL_At(PT_kosuu+1,3)=0, ALL_At(PT_kosuu+1,4)=0,                     ALL_At(PT_kosuu+1,5)=0;
    	  ALL_At(PT_kosuu+2,0)=0, ALL_At(PT_kosuu+2,1)=0, ALL_At(PT_kosuu+2,2)=-1,ALL_At(PT_kosuu+2,3)=0, ALL_At(PT_kosuu+2,4)=camera_point_p[i].x,   ALL_At(PT_kosuu+2,5)=0;

    	  ALL_Et(PT_kosuu,0)=camera_point_c[i].x-camera_point_p[i].x;
    	  ALL_Et(PT_kosuu+1,0)=camera_point_c[i].y-camera_point_p[i].y;
    	  ALL_Et(PT_kosuu+2,0)=camera_point_c[i].z-camera_point_p[i].z;

    	  PT_kosuu=PT_kosuu+3;
    	}
		//すべてのマーカー座標を利用して最小二乗法外部パラメータ推定(初回)
    	ALL_Tt=ALL_At.inv(cv::DECOMP_SVD)*ALL_Et;
    	std::cout <<"ALL_Tt=\n"<<ALL_Tt<< std::endl;//推定外部パラメータ

		ALL_Ft = cv::Mat_<float>(3, 3);//回転行列(日高手法)
      	ALL_Ft(0,0)=1,             ALL_Ft(0,1)=-ALL_Tt(5,0),  ALL_Ft(0,2)=-ALL_Tt(4,0);
      	ALL_Ft(1,0)=-ALL_Tt(5,0),  ALL_Ft(1,1)=1,             ALL_Ft(1,2)=ALL_Tt(3,0),
      	ALL_Ft(2,0)=ALL_Tt(4,0),   ALL_Ft(2,1)=-ALL_Tt(3,0),  ALL_Ft(2,2)=1;

		camera_prmX = camera_prmX-ALL_Tt(5,0)*camera_prmY-ALL_Tt(4,0)*camera_prmZ+ALL_Tt(2,0);
    	camera_prmY = -ALL_Tt(5,0)*camera_prmX+camera_prmY+ALL_Tt(3,0)*camera_prmZ+ALL_Tt(0,0);
    	camera_prmZ = ALL_Tt(4,0)*camera_prmX-ALL_Tt(5,0)*camera_prmY+camera_prmZ+ALL_Tt(1,0);
	
 		//tf(map camera_base間のlink)-----------------------------------------------------------------------------------------
  		//ここがカメラの姿勢部分

    	std::string MaptoCamera_Base_frame = "MaptoCamera_Base_link";
    	//微小区間回転行列
    	double RollX=ALL_Ft(2,1),PitchY=ALL_Ft(0,2),YawZ=ALL_Ft(1,0);

    	//カメラ位置
    	camera_base_pose.position.x = camera_prmZ;//赤(tf:X,画像:Z)
    	camera_base_pose.position.y = camera_prmX;//緑(tf:Y,画像:X)
    	camera_base_pose.position.z = 0;//青(tf:Z,画像:Y)
    	camera_base_pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(YawZ,RollX,PitchY);

    	static tf::TransformBroadcaster br_camera_base_pose;
    	tf::Transform camera_base_transform;
    	poseMsgToTF(camera_base_pose, camera_base_transform);
    	br_camera_base_pose.sendTransform(tf::StampedTransform(camera_base_transform, ros::Time::now(), source_frame, MaptoCamera_Base_frame));
    	std::cout <<"camera_base_pose.position.x="<<camera_base_pose.position.x<< std::endl;
    	std::cout <<"camera_base_pose.position.y="<<camera_base_pose.position.y<< std::endl;
    	std::cout <<"camera_base_pose.position.z="<<camera_base_pose.position.z<< std::endl;
    	std::cout <<"camera_base_pose.orientation.x="<<camera_base_pose.orientation.x<< std::endl;
    	std::cout <<"camera_base_pose.orientation.y="<<camera_base_pose.orientation.y<< std::endl;
    	std::cout <<"camera_base_pose.orientation.z="<<camera_base_pose.orientation.z<< std::endl;
    	std::cout <<"camera_base_pose.orientation.w="<<camera_base_pose.orientation.w<< std::endl;

  		//tf(camera_base camera_link間のlink)-----------------------------------------------------------------------------------------
    	geometry_msgs::Pose Camera_BasetoCamera_Link_pose;

    	//std::string Camera_BasetoCamera_Link_frame = "Camera_BasetoCamera_Link_link";
    	Camera_BasetoCamera_Link_pose.position.x = 0;
    	Camera_BasetoCamera_Link_pose.position.y = 0;
    	Camera_BasetoCamera_Link_pose.position.z = 0.67;//ここに実際のカメラの高さを入れる
    	Camera_BasetoCamera_Link_pose.orientation.w = 1.0;
    	static tf::TransformBroadcaster br_Camera_BasetoCamera_Link_pose;

    	tf::Transform Camera_BasetoCamera_transform;
    	poseMsgToTF(Camera_BasetoCamera_Link_pose, Camera_BasetoCamera_transform);
    	br_Camera_BasetoCamera_Link_pose.sendTransform(tf::StampedTransform(Camera_BasetoCamera_transform, ros::Time::now(), MaptoCamera_Base_frame, "camera_link"));
	}	

  	if(kaisu==0){
  	  camera_base_pose.position.x = 0;
  	  camera_base_pose.position.y = 0;
  	  camera_base_pose.position.z = 0;
  	  camera_base_pose.orientation.x=0;
  	  camera_base_pose.orientation.y=0;
  	  camera_base_pose.orientation.z=0;
  	  camera_base_pose.orientation.w=0;
  	}
  	//経路描写-------------------------------------------------------------
    pose.header.stamp = ros::Time::now();
    pose.header.frame_id = source_frame;
    pose.pose.position = camera_base_pose.position;
    pose.pose.orientation = camera_base_pose.orientation;
    path.header.stamp = ros::Time::now();
    path.header.frame_id = source_frame;
    path.poses.push_back(pose);
    pub_plan.publish(path);



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
	cv::swap(depthimage, depthimage_prev);// image_curr を image_prev に移す（交換する）
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
	message_filters::Subscriber<sensor_msgs::CameraInfo> info_sub(nhSub, "/camera/color/camera_info", 1);

  	typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,sensor_msgs::Image,sensor_msgs::CameraInfo> MySyncPolicy;	
	message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10),rgb_sub, depth_sub, info_sub);
	sync.registerCallback(boost::bind(&callback,_1, _2, _3));

	ros::NodeHandle nhPub;
  	pub_plan = nhPub.advertise<nav_msgs::Path>("/get_multi_path",1000);

	ros::spin();//トピック更新待機
			
	return 0;
}