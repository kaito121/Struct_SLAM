#include<struct_slam/classification.h>


void classificationClass::showSearchWindows(){
    //
    cv::Mat view= cv::Mat::zeros(mapWidthInt, mapHeightInt,CV_8UC3);
    //マップ対角線の半分の半分距離
    float halfLen = std::sqrt(mapWidthInt*mapWidthInt + mapHeightInt*mapHeightInt)/4;
    std::vector<cv::Point2i> sp;
    sp.resize(winIndex2.size());
    //描画対象9つの座標を設定
    for(int k=0; k< winIndex2.size(); k++){
        //y軸を0度
        int deg = minCamDeg + winDivDeg/2 + winDivDeg * k;
		float theta = (float)(deg)/180.0*M_PI;
        //中心からの相対座標->画像座標
        sp[k].x = (int)(halfLen * sin(theta)) + mapWidthInt/2;
        sp[k].y = -(int)(halfLen * cos(theta)) + mapHeightInt/2;
    }
    //9つの座標に対する, 探索窓を描画
    for(int k=0; k< winIndex2.size(); k++){
        //角度算出 と x軸=0度 -> y軸=0度に回転
        int angle = ((int)( atan2(mapHeightInt/2 - sp[k].y,sp[k].x-mapWidthInt/2)/M_PI *180) -90 );
        //使用窓番号選択
        int num = selectWindow(angle);
        //窓内を探索
        for(int m=0; m < winIndex2[num].size(); m++){
            int kk = sp[k].x + sp[k].y * mapWidthInt;
            kk += winIndex2[num][m];
            int w = kk % mapWidthInt;
            int h = kk / mapWidthInt;
            if(w < 0 || w >= mapWidthInt
                || h < 0 || h >= mapHeightInt ){
                    //マップ範囲外検索
                    ROS_INFO("Error");
                    continue;
            }
            //Matに描画
            uint8_t b = 255/9 * k;
            uint8_t g = 200;
            uint8_t r = 200;
            view.at<cv::Vec3b>(h, w)[0] = b;
            view.at<cv::Vec3b>(h, w)[1] = g;
            view.at<cv::Vec3b>(h, w)[2] = r;
        }
    }
    //表示データのPublish
    //cvBridgeデータ作成
	cv_bridge::CvImagePtr viewCvData(new cv_bridge::CvImage);
	viewCvData->encoding=sensor_msgs::image_encodings::BGR8;
    //データコピー
	viewCvData->image=view.clone();
    //Publish
	pubDeb.publish(viewCvData->toImageMsg());
}

void classificationClass::showSearchWindows(float x, float y){

    int angle = -((int)( atan2(y,x)/M_PI *180) )+90 ;
    if(angle<minCamDeg || angle > maxCamDeg){
        ROS_INFO("angle Error: %d",angle);
        return ;
    }
    //使用窓番号選択
    int num = selectWindow(angle);
    if( (int)winIndex2.size() <= num){
        return ;
    }
    //
    cv::Mat view= cv::Mat::zeros(mapWidthInt, mapHeightInt,CV_8UC3);
    //マップ対角線の半分の半分距離
    float halfLen = std::sqrt(mapWidthInt*mapWidthInt + mapHeightInt*mapHeightInt)/4;
    cv::Point2i sp;
    //描画対象9つの座標を設定
    //y軸を0度
    int deg = minCamDeg + winDivDeg/2 + winDivDeg * num;
    float theta = (float)(deg)/180.0*M_PI;
    //中心からの相対座標->画像座標
    sp.x = (int)((x + mapWidth/2) / mapRes);
    sp.y = (int)((mapHeight/2 - y) / mapRes);
    //9つの座標に対する, 探索窓を描画
    //窓内を探索
    for(int m=0; m < winIndex2[num].size(); m++){
        // ROS_INFO("m:%d",m);
        int kk = sp.x + sp.y * mapWidthInt;
        kk += winIndex2[num][m];
        int w = kk % mapWidthInt;
        int h = kk / mapWidthInt;
        if(w < 0 || w >= mapWidthInt
            || h < 0 || h >= mapHeightInt ){
                //マップ範囲外検索
                ROS_INFO("Error");
                continue;
        }
        //Matに描画
        uint8_t b = 255;
        uint8_t g = 200;
        uint8_t r = 200;
        view.at<cv::Vec3b>(h, w)[0] = b;
        view.at<cv::Vec3b>(h, w)[1] = g;
        view.at<cv::Vec3b>(h, w)[2] = r;
    }
    // ROS_INFO("sss");
    //表示データのPublish
    //cvBridgeデータ作成
	cv_bridge::CvImagePtr viewCvData(new cv_bridge::CvImage);
	viewCvData->encoding=sensor_msgs::image_encodings::BGR8;
    //データコピー
	viewCvData->image=view.clone();
    //Publish
	pubDeb.publish(viewCvData->toImageMsg());
}
void classificationClass::showCluster(){
    //障害物カラーレパートリー
	// float colors[12][3] ={{255,0,0},{0,255,0},{0,0,255},{255,255,0},{0,255,255},{255,0,255},{127,255,0},{0,127,255},{127,0,255},{255,127,0},{0,255,127},{255,0,127}};//色リスト
	// float colors[12][3] ={{0,255,255},{0,255,0},{0,0,255},{255,255,0},{0,255,255},{255,0,255},{127,255,0},{0,127,255},{127,0,255},{255,127,0},{0,255,127},{255,0,127}};//色リスト
	//表示用ポイントクラウド
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr viewCloud(new  pcl::PointCloud<pcl::PointXYZRGB>);
    //初期化
    //データ数の計算
    int pointNum = 0;//総データ数
    std::cout<<"cd.data.size():"<<cd.data.size()<<std::endl;
    for(int i = 0; i < cd.data.size(); i++){
        pointNum += (int)cd.data[i].pt.size();
    }
    //
    //表示高さ幅設定(点１つでは見えにくいため表示数を増やす)
    //表示範囲, 表示個数
    //z+zUnder <= z <= z+zUpper
    //(zUpper - zUnder)/zDelta
    float zUpper =0.2;
    float zUnder = -0.2;
    float zDelta =0.05;
    int zLoop = (int)((zUpper - zUnder)/zDelta) + 1;//ループ回数(z軸表示幅)
    //データ数再計算
    pointNum = pointNum * zLoop;
	viewCloud->points.clear();
	viewCloud->points.resize(pointNum);
    
    //要素追加用の仮変数
	pcl::PointXYZRGB cloudTemp;
    //追加済み要素数カウント
    int count = 0;
    //pointCloudデータ作成
    viewCloud->width = count;
    viewCloud->height = 1;
    //各クラスタごとの処理
    for(int i = 0; i < cd.data.size(); i++){
        //非表示処理
        //--データ数が閾値以下の時
        // if(cd.data[i].dens.data < 10){
        //     continue;
        // }
        //カラー設定
        cloudTemp.r=colors[i%12][0];
        cloudTemp.g=colors[i%12][1];
        cloudTemp.b=colors[i%12][2];
        //各データごとの処理
        for(int k = 0; k < cd.data[i].pt.size(); k++){  
            cloudTemp.x=cd.data[i].pt[k].y;//y軸              
            cloudTemp.y=cd.data[i].pt[k].x;//逆向きのx軸
            //表示幅分点を追加
            for(int n=0; n<= (int)((zUpper - zUnder)/zDelta); n++){
                cloudTemp.z=cd.data[i].pt[k].z + zUnder + n*zDelta;//z軸 + 表示範囲            
                //ポイントクラウドに追加
                viewCloud->points[count++] = cloudTemp;
                viewCloud->width = count;
            }
        }
        
    }
	viewCloud->points.resize(count);
	std::cout<<"viewCloud->points.size():"<<viewCloud->points.size()<<"\n";
    //データがないとき
	if(viewCloud->width <= 0)
	{
        ROS_INFO("No point cloud data!");
		return ;
	}
	sensor_msgs::PointCloud2 viewMsgs;
	pcl::toROSMsg (*viewCloud, viewMsgs);
	viewMsgs.header.frame_id="/zed_camera_center";
	pubDebPcl.publish(viewMsgs);
}

void classificationClass::plotObstaclePoints(){
    //障害物カラーレパートリー
	// float colors[12][3] ={{255,0,0},{0,255,0},{0,0,255},{255,255,0},{0,255,255},{255,0,255},{127,255,0},{0,127,255},{127,0,255},{255,127,0},{0,255,127},{255,0,127}};//色リスト
	// float colors[12][3] ={{255,255,127},{0,255,255},{255,0,255},{127,255,0},{0,127,255},{127,0,255},{255,127,0},{0,255,127},{255,0,127},{255,0,0},{0,255,0},{0,0,255}};//色リスト
    //表示用ポイントクラウド
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr viewCloud(new  pcl::PointCloud<pcl::PointXYZRGB>);
    //初期化
    //データ数の計算
    std::cout<<"cd.data.size():"<<cd.data.size()<<std::endl;
    //データ数再計算
	viewCloud->points.clear();
	viewCloud->points.resize(cd.data.size());
    
    //要素追加用の仮変数
	pcl::PointXYZRGB cloudTemp;
    //追加済み要素数カウント
    int count = 0;
    //pointCloudデータ作成
    viewCloud->width = count;
    viewCloud->height = 1;
    //各クラスタごとの処理
    for(int i = 0; i < cd.data.size(); i++){
        //カラー設定
        cloudTemp.r=colors[i%12][0];
        cloudTemp.g=colors[i%12][1];
        cloudTemp.b=colors[i%12][2];
        cloudTemp.x=cd.data[i].gc.y;//y軸              
        cloudTemp.y=cd.data[i].gc.x;//逆向きのx軸
        cloudTemp.z=cd.data[i].gc.z;//z軸       
        //ポイントクラウドに追加
        viewCloud->points[count++] = cloudTemp;
        viewCloud->width = count;
    }
	viewCloud->points.resize(count);
	std::cout<<"viewGravityPointCloud->points.size():"<<viewCloud->points.size()<<"\n";
    //データがないとき
	if(viewCloud->width <= 0)
	{
        ROS_INFO("No point cloud data!");
		return ;
	}
	sensor_msgs::PointCloud2 viewMsgs;
	pcl::toROSMsg (*viewCloud, viewMsgs);
	viewMsgs.header.frame_id="/zed_camera_center";
	pubDebGp.publish(viewMsgs);
}