#include<struct_slam/classification.h>

void classificationClass::setLaunchParam(){
    
    ros::NodeHandle n("~");
    n.getParam("classification/rqt_reconfigure",rqt_reconfigure);
    //カメラパラメータ
    n.getParam("camera/minDeg",minCamDeg);
    n.getParam("camera/maxDeg",maxCamDeg);
    //マップパラメータ
    n.getParam("localMap/width/float",mapWidth);
    n.getParam("localMap/height/float",mapHeight);
    n.getParam("localMap/resolution",mapRes);
	mapWidthInt=(int)(mapWidth/mapRes);//[pixel]
	mapHeightInt=(int)(mapHeight/mapRes);//[pixel]
    //窓パラメータ dynamic_reconfigure
    n.getParam("window/DivisionDegree",winDivDeg);
    winDivNum = (int)( (maxCamDeg - minCamDeg) / winDivDeg ) + 1;//後でチェック
    
    n.getParam("window/height",heightWin);
    n.getParam("window/width",widthWin);
    n.getParam("window/minPts",minPts);
	n.getParam("window/baseDistance",baseDistance);	
	//窓定義
	// std::vector<int> winIndex;//基準点（コア点）から参照値
	int heightWinInt = (int32_t)((float)heightWin/mapRes)*2 + 1;// heightWin / 解像度 *2 + 1
	int widthWinInt = (int32_t)((float)widthWin/mapRes)*2 + 1;// widthWin / 解像度 *2 + 1

	winIndex.resize(heightWinInt * widthWinInt);
	int count = 0;//カウント用
	for(int h = -heightWinInt/2; h <= heightWinInt/2; h++ ){
		for(int w = -widthWinInt/2; w <= widthWinInt/2; w++){
			if(h==0 && w==0 ){
				continue;
			}
			winIndex[count++] = h * mapWidthInt + w;
		}
	}
	//追加
	//launch ファイル読み出し
	// setWindowParam();
	//窓定義2(上のコードを消すまで使用変数の語尾に2を追加)
	// std::vector< std::vector<int> > winIndex2;//基準点（コア点）から参照座標
	winIndex2.resize(winDivNum);//分割個数でリサイズ
	//各窓ごとに定義
	for(int k=0; k<winIndex2.size(); k++){
		int deg = minCamDeg + winDivDeg/2 + winDivDeg * k;
		//探索範囲の算出
		//窓の傾き角度(センサ正面を0度, 反時計回りを正)
		float theta = (float)(deg)/180.0*M_PI;
		float thetaAbs = std::abs(theta);
		//探索サイズ
		float searchRangeH = widthWin* cos(thetaAbs) + heightWin*sin(thetaAbs);
		float searchRangeW = widthWin* sin(thetaAbs) + heightWin*cos(thetaAbs);
		//セル数に変換
		int searchRangeHInt = (int32_t)((float)searchRangeH/mapRes)*2 + 1;// heightWin / 解像度 *2 + 1
		int searchRangeWInt = (int32_t)((float)searchRangeW/mapRes)*2 + 1;// widthWin / 解像度 *2 + 1
		//リサイズ用カウンタ
		int count2 = 0;
		winIndex2[k].resize(searchRangeHInt*searchRangeWInt);
		for(int h = -searchRangeHInt/2; h <= searchRangeHInt/2; h++ ){
			for(int w = -searchRangeWInt/2; w <= searchRangeWInt/2; w++){
				//h,wは, すでに探索セル(探索窓中心座標)からの座標差を示している
				//回転行列
				float dw = w*cos(theta) + h*sin(theta);
				float dh = -w*sin(theta) + h*cos(theta);
				//座標が窓内に存在するか
				if(std::abs(dw) < widthWinInt/2.0 && std::abs(dh) < heightWinInt/2.0){
					//探索インデックスに追加
					winIndex2[k][count2++] = h * mapWidthInt + w;
				}
			}
		}
		winIndex2[k].resize(count2);
	}
}

void classificationClass::configCallback(struct_slam::classificationConfig &config, uint32_t level) {
	ROS_INFO("Reconfigure Request: %d %f %f %d", 
		config.windowDivisionDegree, config.windowHeight,
		config.windowWidth,config.windowMinPts
		// config.str_param.c_str(), 
		// config.bool_param?"True":"False", 
		// config.size
		);

    //窓パラメータ-->dynamic_reconfigure
    winDivDeg = config.windowDivisionDegree;
    winDivNum = (int)( (maxCamDeg - minCamDeg) / winDivDeg ) + 1;//後でチェック
    heightWin = config.windowHeight;
    widthWin = config.windowWidth;
	minPts = config.windowMinPts;
	baseDistance = config.baseDistance;
	//窓定義
	// std::vector<int> winIndex;//基準点（コア点）から参照値
	int heightWinInt = (int32_t)((float)heightWin/mapRes)*2 + 1;// heightWin / 解像度 *2 + 1
	int widthWinInt = (int32_t)((float)widthWin/mapRes)*2 + 1;// widthWin / 解像度 *2 + 1

	winIndex.resize(heightWinInt * widthWinInt);
	int count = 0;//カウント用
	for(int h = -heightWinInt/2; h <= heightWinInt/2; h++ ){
		for(int w = -widthWinInt/2; w <= widthWinInt/2; w++){
			if(h==0 && w==0 ){
				continue;
			}
			winIndex[count++] = h * mapWidthInt + w;
		}
	}
	//追加
	//launch ファイル読み出し
	// setWindowParam();
	//窓定義2(上のコードを消すまで使用変数の語尾に2を追加)
	// std::vector< std::vector<int> > winIndex2;//基準点（コア点）から参照座標
	winIndex2.resize(winDivNum);//分割個数でリサイズ
	// ROS_INFO("originI--H,W:%d,%d",heightWinInt,widthWinInt);
	//各窓ごとに定義
	for(int k=0; k<winIndex2.size(); k++){
		int deg = minCamDeg + winDivDeg/2 + winDivDeg * k;
		//探索範囲の算出
		//窓の傾き角度(センサ正面を0度, 反時計回りを正)
		float theta = (float)(deg)/180.0*M_PI;
		// float theta = (float)(-deg)/180.0*M_PI;
		float thetaAbs = std::abs(theta);
		//探索サイズ
		float searchRangeW = widthWin* cos(thetaAbs) + heightWin*sin(thetaAbs);
		float searchRangeH = widthWin* sin(thetaAbs) + heightWin*cos(thetaAbs);
		//セル数に変換
		int searchRangeHInt = (int32_t)((float)searchRangeH/mapRes)*2 + 1;// heightWin / 解像度 *2 + 1
		int searchRangeWInt = (int32_t)((float)searchRangeW/mapRes)*2 + 1;// widthWin / 解像度 *2 + 1
		//リサイズ用カウンタ
		int count2 = 0;
		winIndex2[k].resize(searchRangeHInt*searchRangeWInt);
		for(int h = -searchRangeHInt/2; h <= searchRangeHInt/2; h++ ){
			for(int w = -searchRangeWInt/2; w <= searchRangeWInt/2; w++){
				//h,wは, すでに探索セル(探索窓中心座標)からの座標差を示している
				//回転行列
				float dw = w*cos(theta) + h*sin(theta);
				float dh = -w*sin(theta) + h*cos(theta);
				//座標が窓内に存在するか
				if(std::abs((int)dw) <= widthWinInt/2.0 && std::abs((int)dh) <= heightWinInt/2.0){
					//探索インデックスに追加
					winIndex2[k][count2++] = h * mapWidthInt + w;
				}
			}
		}
		winIndex2[k].resize(count2);
	}
	showSearchWindows(config.debugX, config.debugY);
}

bool classificationClass::isCameraMapData(){
    if(smdCamera.header.seq > 0){
        return true;
    }
    return false;
}
bool classificationClass::isClassificationData(){
    if((int)cd.data.size() > 0){
        return true;
    }
    return false;
}