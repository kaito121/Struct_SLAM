#include<struct_slam/imageMatching.h>

//subscribe
// void imageMatchingClass::subscribeImageData(){//データ受信
// 	queue1.callOne(ros::WallDuration(1));
// }
// void imageMatchingClass::subscribeMaskImageData(){//データ受信
// 	queue2.callOne(ros::WallDuration(1));
// }
void imageMatchingClass::image_callback(const sensor_msgs::ImageConstPtr& msg)//カメラデータを取り込んで
{
    try{
        bridgeImageCur=cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::BGR8);
        ROS_INFO("callBack");
        imgCurOnce=true;
    }
    catch(cv_bridge::Exception& e) {//エラー処理
        std::cout<<"depth_image_callback Error \n";
        ROS_ERROR("Could not convert from '%s' to 'BGR8'.",
        msg->encoding.c_str());
        return ;
    } 
    //manageに移動
    manage();
}
void imageMatchingClass::maskImage_callback(const struct_slam::MaskImageData::ConstPtr& msg)
{//midCurにデータをぶち込んでる
    midCur.header = msg->header;
    //--画像データ
    midCur.width.data = msg->width.data;
    midCur.height.data = msg->height.data;
    //--マップデータ
    midCur.mapWidth.data = msg->mapWidth.data;
    midCur.mapHeight.data = msg->mapHeight.data;
    midCur.mapRes.data = msg->mapRes.data;
    midCur.mapWidthInt.data = msg->mapWidthInt.data;
    midCur.mapHeightInt.data = msg->mapHeightInt.data;
    midCur.index = msg->index;
    midCur.pt = msg->pt;
    //manageに移動
    manage();
}
void imageMatchingClass::manage(){
    
    if(isBridgeImageCur() && isMaskImageCur()){
        ROS_INFO("cvtGray");
        cvtGray();
        //特徴点抽出
        ROS_INFO("getFeaturePoints");
        getFeaturePoints();
    }
    //データが２つ以上ある時にマッチング処理をする
    if(isBridgeImagePre() && isMaskImagePre() && isFpointPre()){
        //特徴点を追加すべきか
        ROS_INFO("if(dicideAddPoints");
        if(dicideAddPoints()){
            //追加
            ROS_INFO("addPreFeaturePoints");
            addPreFeaturePoints();
        }
        if(checkPointSize()){
            //特徴点マッチング
            ROS_INFO("featureMatching");
            featureMatching();
            ROS_INFO("checkMatchingError");
            //マッチングミスをデータから除去
            checkMatchingError();
            ROS_INFO("creatMatchingData");
            // publishデータ生成
            creatMatchingData();
            ROS_INFO("publishMatchingData");
            //publish
            publishMatchingData();
            //デバッグ処理
            debug();
        }
    }
    ROS_INFO("resetData");
    //データ更新
    resetData();
}
void imageMatchingClass::cvtGray(){
    cv::cvtColor(bridgeImageCur->image,grayImgCur,CV_BGR2GRAY);
}
bool imageMatchingClass::isBridgeImageCur(){
    if(!imgCurOnce){//
        return false;
    }
    else{
        return true;
    }
}
bool imageMatchingClass::isBridgeImagePre(){
    if(!imgPreOnce){//
        return false;
    }
    else{
        return true;
    }
}
bool imageMatchingClass::isMaskImageCur(){
    if(midCur.pt.empty()){//データの有無を確認
        return false;
    }
    else{
        return true;
    }
}
bool imageMatchingClass::isMaskImagePre(){
    if(midPre.pt.empty()){//データの有無を確認
        return false;
    }
    else{
        return true;
    }
}
bool imageMatchingClass::isFpointPre(){
    if((int)featurePointsTemp.size() > 0){//データの有無を確認
        return true;
    }
    else{
        return false;
    }
}    
void imageMatchingClass::resetData(){
    // *bridgeImagePre = *bridgeImageCur;
    //念のためしっかりコピー
    ROS_INFO("!imgCurOnce:%d", !imgCurOnce);
    if(!imgCurOnce){
        return ;
    }
    bridgeImagePre = bridgeImageCur;
    // bridgeImagePre->header = bridgeImageCur->header;
    // bridgeImagePre->encoding = bridgeImageCur->encoding;
    // bridgeImagePre->image = bridgeImageCur->image.clone();
    imgCurOnce=false;
    imgPreOnce=true;
    if(midCur.header.seq <= 0){
        return ;
    }
    // maskImageData
    midPre = midCur;
    //グレースケール画像
    grayImgPre = grayImgCur.clone();
    //特徴点
    if((int)featurePointsCur.size()>0){
        featurePointsPre = featurePointsCur;
    }
    else{
        featurePointsPre = featurePointsTemp;
    }
    // ROS_INFO("reset featurePointsPre.size, featurePointsTemp.size=%d,%d", (int)featurePointsPre.size(), (int)featurePointsTemp.size());
    featurePointsTemp.clear();
    featurePointsCur.clear();
    // ROS_INFO("reset featurePointsPre.size, featurePointsTemp.size=%d,%d", (int)featurePointsPre.size(), (int)featurePointsTemp.size());
    //追跡回数
    tranckNumPre = tranckNumCur;
    tranckNumCur.clear();

}
// ■初期化
// void imageMatchingClass::getFeaturePointNum(){

// }
void imageMatchingClass::getFeaturePoints(){
    //キーポイント
    std::vector<cv::KeyPoint> keypoints;
    //特徴点抽出器
    auto detector = cv::FastFeatureDetector::create(maxDetectPoint,false);
    //画像を分割して均等に特徴点を抽出
    int clipPixH=(int)grayImgCur.rows / nh;
    int clipPixW=(int)grayImgCur.cols / nw;
    //特徴点リサイズ
    featurePointsTemp.resize(maxPoint*nh*nw);
    int fpSize=0;//特徴点サイズカウンタ
    // ROS_INFO("for");
    for(int h=0;h<nh;h++){
        for(int w=0;w<nw;w++){
            //画像分割
            clipImg=grayImgCur(cv::Rect(w*clipPixW,h*clipPixH,clipPixW,clipPixH));
            //キーポイントを抽出
            detector->detect(clipImg, keypoints);
            // ROS_INFO("keypoints.size():%d",(int)keypoints.size());
            //レスポンスが強い順（降順）にソート
            sort(keypoints.begin(),keypoints.end(),
                [](const cv::KeyPoint& x, const cv::KeyPoint& y) {return  x.response > y.response;});
            //keypointの中から特徴点を抽出
            //各エリアごとの特徴点取得数をカウント
            // ROS_INFO("featurePointsTemp.size(),fpSize : %d, %d",(int)featurePointsTemp.size(),fpSize);
            int count=0;
            for(std::vector<cv::KeyPoint>::iterator itk = keypoints.begin();
                itk != keypoints.end() && count <= maxPoint; ++itk){
                    cv::Point2i pt;
                    pt.x=w*clipPixW+(int)itk->pt.x;
                    pt.y=h*clipPixH+(int)itk->pt.y;
                    if(midCur.index[pt.y*midCur.width.data + pt.x].data >= 0){
                        featurePointsTemp[fpSize++] = pt;
                        count++;
                    }
            }
        }
    }
    // ROS_INFO("resize");
    //特徴点リサイズ
    featurePointsTemp.resize(fpSize);
    // //追跡数リサイズ, 0で初期化
    // tranckNumCur.resize(fpSize,0);
    //追跡回数を初期化0で挿入
    std::vector<int> zeroInser(featurePointsTemp.size(), 0);
    tranckNumCur.reserve(featurePointsTemp.size());
    std::copy(zeroInser.begin(), zeroInser.end(), std::back_inserter(tranckNumCur));
}
bool imageMatchingClass::dicideAddPoints(){
    if(featurePointsTemp.size() > featurePointsPre.size()){//} || 1000 > (int)featurePointsPre.size()){
        return true;
    }
    else{
        return false;
    }
}
void imageMatchingClass::addPreFeaturePoints(){
    //キーポイント
    std::vector<cv::KeyPoint> keypoints;
    //特徴点抽出器
    auto detector = cv::FastFeatureDetector::create(maxDetectPoint,false);
    //画像を分割して均等に特徴点を抽出
    int clipPixH=(int)grayImgPre.rows / nh;
    int clipPixW=(int)grayImgPre.cols / nw;
    //特徴点リサイズ
    std::vector<cv::Point2f> featureTemp;
    featureTemp.resize(maxPoint*nh*nw);
    int fpSize=0;//特徴点サイズカウンタ
    for(int h=0;h<nh;h++){
        for(int w=0;w<nw;w++){
            //画像分割
            clipImg=grayImgPre(cv::Rect(w*clipPixW,h*clipPixH,clipPixW,clipPixH));
            //キーポイントを抽出
            detector->detect(clipImg, keypoints);
            //レスポンスが強い順（降順）にソート
            sort(keypoints.begin(),keypoints.end(),
                [](const cv::KeyPoint& x, const cv::KeyPoint& y) {return  x.response > y.response;});
            //keypointの中から特徴点を抽出
            //各エリアごとの特徴点取得数をカウント
            // ROS_INFO("keypoints.size():%d",(int)keypoints.size());
            int count=0;
            for(std::vector<cv::KeyPoint>::iterator itk = keypoints.begin();
                itk != keypoints.end() && count <= maxPoint; ++itk){
                    cv::Point2i pt;
                    pt.x=w*clipPixW+(int)itk->pt.x;
                    pt.y=h*clipPixH+(int)itk->pt.y;
                    if(midPre.index[pt.y*midPre.width.data + pt.x].data >= 0){
                        featureTemp[fpSize++] = pt;
                        count++;
                    }
            }
        }
    }
    //特徴点リサイズ
    featureTemp.resize(fpSize);
    //insert
    // featurePointsPre.insert(featurePointsPre.end(),featureTemp.size());
    featurePointsPre.reserve(featurePointsPre.size() + featureTemp.size());
    std::copy(featureTemp.begin(), featureTemp.end(), std::back_inserter(featurePointsPre));
    //追跡回数を初期化0で挿入
    std::vector<int> zeroInser(featurePointsPre.size(), 0);
    tranckNumPre.reserve(tranckNumPre.size() + featureTemp.size());
    std::copy(zeroInser.begin(), zeroInser.end(), std::back_inserter(tranckNumPre));
    // tranckNumPre.resize(featurePointsPre.size(),0);
}
bool imageMatchingClass::checkPointSize(){
    if((int)featurePointsPre.size() > 0 && (int)featurePointsTemp.size() > 0){
        return true;
    }
    else{
        return false;
    }
}
void imageMatchingClass::featureMatching(){//画像マッチング
    ROS_INFO("featurePointsPre.size, featurePointsTemp.size=%d,%d", (int)featurePointsPre.size(), (int)featurePointsTemp.size());
    cv::calcOpticalFlowPyrLK(grayImgPre,grayImgCur, featurePointsPre, featurePointsTemp, sts, ers,
        cv::Size(ws,ws),3,
        cvTermCriteria (CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 30, 0.05), 1);
    // cv::calcOpticalFlowPyrLK(grayImgCur,grayImgPre, featurePointsTemp, featurePointsPre, sts, ers,
    //     cv::Size(ws,ws),3,
    //     cvTermCriteria (CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 30, 0.05), 1);
}
void imageMatchingClass::checkMatchingError(){
    //エラーチェックして再格納
    int count=0;//格納数カウント
    featurePointsCur.resize(featurePointsTemp.size());
    tranckNumCur.resize(featurePointsTemp.size());
    // ROS_INFO("midCur.index.size() :%d",(int)midCur.index.size());
    // ROS_INFO("midCur.width.data*midCur.height.datamidCur.height.data :%d",midCur.height.data*midCur.width.data);
    // ROS_INFO("featurePointsPre.size() :%d",(int)featurePointsPre.size());
    ROS_INFO("size:(fpc, fpp, fpt),(%d,%d,%d)",(int)featurePointsCur.size(), (int)featurePointsPre.size(), (int)featurePointsTemp.size());
    // ROS_INFO("tranckNumPre.size() :%d",(int)tranckNumPre.size());
    // ROS_INFO("tranckNumCur.size() :%d",(int)tranckNumCur.size());
    int countSts=0;
    for(int k=0;k<featurePointsTemp.size();k++){
        // ROS_INFO("(int)featurePointsTemp[k].y*midCur.width.data + (int)featurePointsTemp[k].x:%d",(int)featurePointsTemp[k].y*midCur.width.data + (int)featurePointsTemp[k].x);
        if(featurePointsTemp[k].x < 0 || featurePointsTemp[k].y < 0 ){
            continue;
        }
        if(sts[k]){
            countSts++;
            // ROS_INFO("tranckNumCur.size() :%d",(int)tranckNumCur.size());
            // ROS_INFO("sts is ok: [%d]",k);
            if(midCur.index[(int)featurePointsTemp[k].y*midCur.width.data + (int)featurePointsTemp[k].x].data>=0 ){
                // ROS_INFO("in map: [%d]",k);
                featurePointsPre[count] = featurePointsPre[k];
                featurePointsCur[count] = featurePointsTemp[k];
                tranckNumCur[count] = tranckNumPre[k] + 1;
                count++;
            }
        }
    }
    // ROS_INFO("sts: [%d]",countSts);
    // ROS_INFO("in map: [%d]",count);
    //リサイズ
    featurePointsPre.resize(count);
    featurePointsCur.resize(count);
}
void imageMatchingClass::creatMatchingData(){//matchDataにデータぶち込んでる
    //送信データの設定
    matchData.header = midCur.header;
    matchData.width = midCur.mapWidth;
    matchData.height = midCur.mapHeight;
    matchData.res = midCur.mapRes;
    matchData.widthInt = midCur.mapWidthInt;
    matchData.heightInt = midCur.mapHeightInt;
    //--cpはlaunchファイルで設定すべき
    matchData.cp.x=0;
    matchData.cp.y=0;
    matchData.cp.z=0;
    //リサイズ
    matchData.index.resize(matchData.widthInt.data*matchData.heightInt.data);
    matchData.data.resize((int)featurePointsCur.size());
    int dataSize=0;//データサイズカウント

    //インデックスの中身を初期化
    for(int k=0;k<matchData.index.size();k++){
        matchData.index[k].data = -1;
    }
    //
    for(int k=0;k<featurePointsCur.size();k++){
        //時刻tのデータ
        // ROS_INFO("(int)featurePointsCur.size(),cur,pre:%d,%d", (int)featurePointsCur.size(), (int)featurePointsPre.size());
        int cW = (int)featurePointsCur[k].x;
        int cH = (int)featurePointsCur[k].y;
        // ROS_INFO("cH*midCur.width.data + cW:,%d", cH*midCur.width.data + cW);
        // ROS_INFO("mid(x,y):,%d,%d,%d,%d", cW, cH,(int)midCur.width.data,(int)midCur.height.data);
        int cIndex = midCur.index[cH*midCur.width.data + cW].data;
        // ROS_INFO("cIndex:%d", cIndex);

        geometry_msgs::Point cPt =  midCur.pt[cIndex];
        //時刻t-Delta_tのデータ
        // ROS_INFO("featurePointsPre[%d](x,y):,%f,%f", k,featurePointsPre[k].x, featurePointsPre[k].y);
        int pW = (int)featurePointsPre[k].x;
        int pH = (int)featurePointsPre[k].y;
        // ROS_INFO("pH,pW:,%d,%d", pH, pW);
        int pIndex = midPre.index[pH*midPre.width.data + pW].data;
        geometry_msgs::Point pPt =  midPre.pt[pIndex];
        // ROS_INFO("cPt.x,cPt.y,pPt.x,pPt.y:%f,%f,%f,%f",cPt.x,cPt.y,pPt.x,pPt.y);
        // ROS_INFO("midPre(%d,%d)",midPre.width.data,midPre.height.data);
        if(std::abs(pPt.x) >= mapW/2.0 || std::abs(pPt.y) >= mapH/2.0){
            // ROS_INFO("Continue");
            continue;
        }
        //追跡回数
        if(tranckNumCur[k] < trackThreshold){
            // ROS_INFO("Continue");
            continue;
        }
        //マップ位置
        int xi,yi,zi;//■ ziを追加した
        if(convertToGrid(cPt.x,cPt.y,xi,zi)){//データ変換　■ここに問題が？
            //移動量: cur - pre
            float difX=cPt.x - pPt.x;
            float difY=cPt.y - pPt.y;
            // ROS_INFO("zi,xi,w,h:%d,%d,%d,%d",xi,zi,matchData.widthInt.data,matchData.heightInt.data);
            //インデックスデータ
            matchData.index[zi*matchData.widthInt.data+xi].data = dataSize; //■わからないので、smdからmatchDataに変更した
            //マップ上の移動量
            // ROS_INFO("cPt.x,cPt.y,pPt.x,pPt.y:%f,%f,%f,%f",cPt.x,cPt.y,pPt.x,pPt.y);
            // matchData.data[dataSize].x.data = (int)(difX/midCur.mapRes.data);
            matchData.data[dataSize].x.data = -(int)(difX/midCur.mapRes.data);
            matchData.data[dataSize].y.data = -(int)(difY/midCur.mapRes.data);
            //実測の移動角度
            // ROS_INFO("difX,difY:%f,%f",difX,difY);
            matchData.data[dataSize].theta.data = std::atan2(difY,difX);//-PIから+PI ■引数２つ必要　勝手に変更した difY/difX から difY,difX
            //インクリメント
            dataSize++;
        }
    }
    //リサイズ
    // ROS_INFO("dataSize:%d",dataSize);
    matchData.data.resize(dataSize);
}

bool imageMatchingClass::convertToGrid(const float& x,const float& y,int& xg,int& yg){
	//マップ上の中心座標(ふつうは　センサ位置＝マップ中心座標　のため　cx=cy=0)
	float cx=0;
	float cy=0;
    //マップ原点座標を画像原点座標(左上)に移動
	// float map_x = mapW/2 + (x - cx);
	float map_x = mapW/2 - (x - cx);
	float map_y = mapH/2 + ( -(y - cy) );
    //マップ外のデータの際はreturn false
	if(map_x<0 || map_x>mapW)
		return false;
	if(map_y<0 || map_y>mapH)
		return false;
    //ピクセルデータに変換
	xg = (int)(map_x/mapR);
	yg = (int)(map_y/mapR);
    //変換成功
	return true;
}
void imageMatchingClass::publishMatchingData(){//データ送信
    pubMatch.publish(matchData);//matchDataを出力してる
}