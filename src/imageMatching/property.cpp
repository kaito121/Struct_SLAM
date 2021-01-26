#include<struct_slam/ImageMatching.h>

void imageMatchingClass::setLaunchParam(){
    
    ros::NodeHandle n("~");
    n.getParam("imageMatching/rqt_reconfigure",rqt_reconfigure);
    //デバッグタイプ
    n.getParam("imageMatching/debugType",debugType);
    //マップパラメータ
    n.getParam("localMap/width/float",mapW);
    n.getParam("localMap/height/float",mapH);
    n.getParam("localMap/resolution",mapR);
	mapWi=(int)(mapW/mapR);//[pixel]
	mapHi=(int)(mapH/mapR);//[pixel]
    //追跡閾値
    n.getParam("imageMatching/trackThreshold", trackThreshold);
    //特徴点抽出
	//画像分割数
	//分割のアスペクト比が均等に近く, 割り切れる値を選出
    n.getParam("imageMatching/featurePoints/divisionW", nw);
    n.getParam("imageMatching/featurePoints/divisionH", nh);
	//max検出数（分割領域１つあたり）
    n.getParam("imageMatching/featurePoints/maxDetect", maxDetectPoint);
	//max取得数（分割領域１つあたり）
    n.getParam("imageMatching/featurePoints/maxPoint", maxPoint);
	//LK法のwindow size
    n.getParam("imageMatching/featurePoints/windowSize", ws);
}
void imageMatchingClass::configCallback(struct_slam::imageMatchingConfig &config, uint32_t level) {
	ROS_INFO("Reconfigure Request: %d %d %d %d %d %d %d", 
		config.debugType,
		config.trackThreshold, config.divisionW, config.divisionH,
        config.maxDetect, config.maxPoint, config.windowSize
		// config.str_param.c_str(), 
		// config.bool_param?"True":"False", 
		// config.size
		);
    debugType = config.debugType;
    trackThreshold = config.trackThreshold;
    nw = config.divisionW;
    nh = config.divisionH;
    maxDetectPoint = config.maxDetect;
    maxPoint = config.maxPoint;
    ws = config.windowSize;
}
