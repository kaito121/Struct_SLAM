#include <ros/ros.h>
#include <struct_slam/imageMatching.h>

int main(int argc,char **argv){
	ros::init(argc,argv,"struct_slam_im");
	
    imageMatchingClass imc; //image matching 
    ros::spin();
	// while(ros::ok())
	// {	
	// 	//subscribe
	// 	ROS_INFO("subscribeImageData");
	// 	imc.subscribeImageData();
	// 	ROS_INFO("subscribeMaskImageData");
	// 	imc.subscribeMaskImageData();
	// 	// ROS_INFO("if(imc.isBridgeImagePre() && imc.isMaskImagePre())=%d,%d",
	// 	// 	imc.isBridgeImagePre(), imc.isMaskImagePre());
	// 	if(imc.isBridgeImageCur()){
	// 		ROS_INFO("cvtGray");
	// 		imc.cvtGray();
	// 		//特徴点抽出
	// 		ROS_INFO("getFeaturePoints");
	// 		imc.getFeaturePoints();
	// 	}
	// 	//データが２つ以上ある時にマッチング処理をする
	// 	if(imc.isBridgeImagePre() && imc.isMaskImagePre() && imc.isFpointPre()){
	// 		//特徴点を追加すべきか
	// 		ROS_INFO("if(imc.dicideAddPoints");
	// 		if(imc.dicideAddPoints()){
	// 			//追加
	// 			ROS_INFO("addPreFeaturePoints");
	// 			imc.addPreFeaturePoints();
	// 		}
	// 		if(imc.checkPointSize()){
	// 			//特徴点マッチング
	// 			ROS_INFO("featureMatching");
	// 			imc.featureMatching();
	// 			ROS_INFO("checkMatchingError");
	// 			//マッチングミスをデータから除去
	// 			imc.checkMatchingError();
	// 			ROS_INFO("creatMatchingData");
	// 			// publishデータ生成
	// 			imc.creatMatchingData();
	// 			ROS_INFO("publishMatchingData");
	// 			//publish
	// 			imc.publishMatchingData();
	// 			imc.debug();
	// 		}
	// 	}
	// 	ROS_INFO("resetData");
	// 	//データ更新
	// 	imc.resetData();
	// }
    return 0;
}