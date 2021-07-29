#include<struct_slam/classification.h>

//subscribe
void classificationClass::subscribeSensorDataCamera(){//Cameraデータ受信
	queue1.callOne(ros::WallDuration(1));
}
void classificationClass::cameraMap_callback(const struct_slam::opt_point::ConstPtr& msg)
{
    //データをコピー
    smdCamera = *msg;//pot_pointのデータの型を入れている
	//move manage method
	manage();
}
//void classificationClass::subscribeSensorDataLRF(){//LRFデータ受信
//	queue2.callOne(ros::WallDuration(1));
//}
/*void classificationClass::laserMap_callback(const struct_slam::opt_point::ConstPtr& msg)
{
    //データをコピー
    smdLRF.header = msg->header;
    smdLRF.width = msg->width;
    smdLRF.height = msg->height;
    smdLRF.res = msg->res;
    smdLRF.widthInt = msg->widthInt;
    smdLRF.heightInt = msg->heightInt;
    smdLRF.cp = msg->cp;
    smdLRF.index = msg->index;
    smdLRF.size = msg->size;
    smdLRF.pt = msg->pt;
	manage();
}*/
void classificationClass::manage(){
	//
	// classificationDBSCAN();
	ROS_INFO("newClassificationDBSCAN");
	newClassificationDBSCAN();
	ROS_INFO("isClassificationData");
	if(!isClassificationData()){
		ROS_INFO("No Classification data");
	}
	publishClassificationData();
	showCluster();
	plotObstaclePoints();
	clearMessages();
}
/*void classificationClass::classificationDBSCAN(){//カメラ

	// 送信データ作成
	// clustedData cd;
	cd.header = smdCamera.header;
	//map設定データ
	cd.width = smdCamera.width;
	cd.height = smdCamera.height;
	cd.res = smdCamera.res;
	cd.widthInt = smdCamera.widthInt;
	cd.heightInt = smdCamera.heightInt;
	cd.cp = smdCamera.cp;
	cd.size.data = 0;
	//障害物データ数確保
	cd.data.resize(smdCamera.index.size());

	//mapIndexデータ
	// mapIndex = -1 -> searched or not include obstacle point
	std::vector<int> mapIndex;
	// mapIndex.insert(mapIndex.end(), smdCamera.index.begin(), smdCamera.index.end());
	mapIndex.resize(smdCamera.index.size());
	for(int k=0; k < mapIndex.size(); k++){
		mapIndex[k] = smdCamera.index[k].data;
	}
	// DBSCAN開始
	for(int k = 0; k < mapIndex.size(); k++){//k : 0 -> widthInt * heightInt -1 
		//マップセルにデータがない場合：スキップ
		if(mapIndex[k] < 0){
			continue;
		}
		// データがある場合
		//探索済み点群を反映したIndex : taskIndexのfor文中での重複探査を防止
		std::vector<int> searchedIndex;
		searchedIndex = mapIndex;
		// クラスタデータに追加
		// クラスタサイズ拡張
		cd.size.data += 1;
		int clusterNum = cd.size.data - 1;
		// リサイズ
		cd.data[clusterNum].index.resize((int)mapIndex.size() - k);//残り最大サイズ
		cd.data[clusterNum].pt.resize((int)mapIndex.size() - k);//残り最大サイズ
		// データ代入(初期化)
		cd.data[clusterNum].size.data = 0;
		cd.data[clusterNum].dens.data = 0;
		cd.data[clusterNum].gc.x = 0;
		cd.data[clusterNum].gc.y = 0;
		cd.data[clusterNum].gc.z = 0;
		
		//タスクデータ作成
		std::vector<int> taskIndex;//候補点群の格納
		int taskSize = 0;//taskIndexのサイズ
		taskIndex.resize((int)mapIndex.size() - k);//残り最大サイズ
		taskIndex[taskSize++] = k;

		//タスク追加データを重複探査防止 
		searchedIndex[k] = -1;

		for(int n=0; n < taskSize; n++){
			int count = 0;//密度（点の数）
			std::vector<int> tempIndex;//Window探索中のみ使用するIndex
			tempIndex.resize(winIndex.size());
			int tempSize = 0;
			//窓内を探索
			for(int m=0; m < winIndex.size(); m++){
				int pos = taskIndex[n] + winIndex[m];
				int posW = taskIndex[n] % smdCamera.widthInt.data;
				int moveW = winIndex[m] % smdCamera.widthInt.data;
				//ここの処理, 後で確認
				int difW = posW + moveW;
				if(pos < 0 || pos >= (int)mapIndex.size()
					|| difW < 0 || difW > smdCamera.widthInt.data ){
						//マップ範囲外検索
						continue;
				}
				//セルにデータがない場合：スキップ
				if(mapIndex[pos] < 0){
					continue;
				}
				//点の数（密度）を追加
				count += smdCamera.size[ mapIndex[pos] ].data;

				//探査済みの場合：スキップ
				if(searchedIndex[pos]< 0){
					continue;
				}
				//次のタスク候補点（コア点候補）として追加
				tempIndex[tempSize++] = pos;
			}
			//リサイズ
			tempIndex.resize(tempSize);

			//点の数（密度）を評価
			// (smdCamera.height/2+smdCamera.cp) : センサからマップの上端までの距離
			// taskIndex[n] / smdCamera.widthInt * smdCamera.res : マップ上端から障害物セルまでの距離
			float y=(smdCamera.height.data/2.0+smdCamera.cp.y) - taskIndex[n] / smdCamera.widthInt.data * smdCamera.res.data;
			float x= (smdCamera.width.data/2.0+smdCamera.cp.x) - taskIndex[n] % smdCamera.widthInt.data * smdCamera.res.data;
			float d=sqrt(pow(x,2.0)+ pow(y,2.0));
			//奥行dに対する評価式
			int minPts_d;

			if(d < baseDistance){
				// minPts_d=minPts;
				minPts_d=(int)( minPts + pow(d-baseDistance,2.0) * minPts );
			}
			else{
				minPts_d=(int)( minPts - pow(d-baseDistance,2.0) * minPts );
			}
			if(minPts_d < 1){
				minPts_d = 1;
			}
			// int minPts=10;
			//評価式よりカウントが小さい時: スキップ
			if(count < minPts_d){
				continue;
			}
			//コア点をクラスタに追加
			// データ代入
			//mapIndex[k] <<< k=taskIndex[n] 
			cd.data[clusterNum].size.data += 1;
			cd.data[clusterNum].dens.data += smdCamera.size[ mapIndex[taskIndex[n]] ].data;
			cd.data[clusterNum].gc.x += smdCamera.pt_curr[ mapIndex[taskIndex[n]] ].x*smdCamera.size[ mapIndex[taskIndex[n]] ].data ;
			cd.data[clusterNum].gc.y += smdCamera.pt_curr[ mapIndex[taskIndex[n]] ].y*smdCamera.size[ mapIndex[taskIndex[n]] ].data ;
			cd.data[clusterNum].gc.z += smdCamera.pt_curr[ mapIndex[taskIndex[n]] ].z*smdCamera.size[ mapIndex[taskIndex[n]] ].data ;
			cd.data[clusterNum].index[cd.data[clusterNum].size.data - 1].data = taskIndex[n];//マップセルに対するインデックス, 1データ番号->マップセル
			cd.data[clusterNum].pt[cd.data[clusterNum].size.data - 1] = smdCamera.pt_curr[ mapIndex[taskIndex[n]] ];
			//重複探査防止
			// tempIndex（タスク候補点をタスクに追加）
			for(int m=0; m < tempIndex.size(); m++){
				//追加
				taskIndex[taskSize++] = tempIndex[m];
				//重複探査防止
				searchedIndex[tempIndex[m]] = -1;
			}
		}
		if(cd.data[clusterNum].size.data==0){
			mapIndex[cd.data[clusterNum].index[0].data] = -1;
			cd.size.data -=1;
			continue;
		}
		//リサイズ
		cd.data[clusterNum].index.resize(cd.data[clusterNum].size.data);
		//追加したデータをマップから削除
		for(int n=0; n < cd.data[clusterNum].index.size(); n++){
			//mapIndex[データ番号[n]->マップ位置] = データ無し
			mapIndex[cd.data[clusterNum].index[n].data] = -1;
		}
			
		//タスクデータ探査終了
		//平均点算出
		cd.data[clusterNum].gc.x /= cd.data[clusterNum].dens.data;
		cd.data[clusterNum].gc.y /= cd.data[clusterNum].dens.data;
		cd.data[clusterNum].gc.z /= cd.data[clusterNum].dens.data;
		//リサイズ
		cd.data[clusterNum].pt.resize(cd.data[clusterNum].size.data);
	}
	// DBSCAN終了
	//データリサイズ
	cd.data.resize(cd.size.data);
}*/

void classificationClass::newClassificationDBSCAN(){//カメラ

	// 送信データ作成
	// clustedData cd;
	cd.header = smdCamera.header;
	//map設定データ
	cd.width = smdCamera.width;
	cd.height = smdCamera.height;
	cd.res = smdCamera.res;
	cd.widthInt = smdCamera.widthInt;
	cd.heightInt = smdCamera.heightInt;
	cd.cp = smdCamera.cp;
	cd.size.data = 0;
	//障害物データ数確保
	cd.data.resize(smdCamera.index.size());

	//mapIndexデータ
	// mapIndex = -1 -> searched or not include obstacle point
	std::vector<int> mapIndex;
	// mapIndex.insert(mapIndex.end(), smdCamera.index.begin(), smdCamera.index.end());
	mapIndex.resize(smdCamera.index.size());
	ROS_INFO("smdCamera.index.size(): %d",(int)smdCamera.index.size());
	
	for(int k=0; k < mapIndex.size(); k++){
		mapIndex[k] = smdCamera.index[k].data;
	}
	ROS_INFO("mapIndex.size(): %d",(int)mapIndex.size());
	// DBSCAN開始
	ROS_INFO("DBSCAN start");
	for(int k = 0; k < mapIndex.size(); k++){//k : 0 -> widthInt * heightInt -1 
		//マップセルにデータがない場合：スキップ
		if(mapIndex[k] < 0){
			ROS_INFO("continue");
			
			continue;
		}
		// データがある場合
		//探索済み点群を反映したIndex : taskIndexのfor文中での重複探査を防止
		std::vector<int> searchedIndex;
		searchedIndex = mapIndex;
		// クラスタデータに追加
		// クラスタサイズ拡張
		cd.size.data += 1;
		int clusterNum = cd.size.data - 1;
		// リサイズ
		cd.data[clusterNum].index.resize((int)mapIndex.size() - k);//残り最大サイズ
		cd.data[clusterNum].pt.resize((int)mapIndex.size() - k);//残り最大サイズ
		// データ代入(初期化)
		cd.data[clusterNum].size.data = 0;
		cd.data[clusterNum].dens.data = 0;
		cd.data[clusterNum].gc.x = 0;
		cd.data[clusterNum].gc.y = 0;
		cd.data[clusterNum].gc.z = 0;
		
		//タスクデータ作成
		std::vector<int> taskIndex;//候補点群の格納
		int taskSize = 0;//taskIndexのサイズ
		taskIndex.resize((int)mapIndex.size() - k);//残り最大サイズ
		taskIndex[taskSize++] = k;
		ROS_INFO("taskIndex.size(): %d",(int)taskIndex.size());
		//タスク追加データを重複探査防止 
		searchedIndex[k] = -1;

		// ROS_INFO("for(int n=0; n < taskSize; n++");
		for(int n=0; n < taskSize; n++){
			int count = 0;//密度（点の数）
			int robotX = (taskIndex[n] % smdCamera.widthInt.data) - smdCamera.widthInt.data/2;
			int robotY = -( (taskIndex[n] / smdCamera.widthInt.data) - smdCamera.heightInt.data/2 );
			float tempX = robotX * smdCamera.res.data;
			float tempY = robotY * smdCamera.res.data;
			// ROS_INFO("x,y:%f,%f",tempX,tempY);
			int angle;
			if(tempX==0){
				angle = 0;
			}
			else
			{
				angle = -((int)( atan2(tempY,tempX)/M_PI *180) )+90 ;
			}
			if(angle<minCamDeg || angle > maxCamDeg){
				ROS_INFO("angle Error: %d",angle);
				continue;
			}
			//使用窓番号選択
			int winNum = selectWindow(angle);
			// ROS_INFO("winNum:%d",winNum);
			if( (int)winIndex2.size() <= winNum){
				ROS_INFO("over winIndex2.size()");
				continue ;
			}
			// ROS_INFO("angle: %d, winIndex2[%d].size():%d",angle, winNum,(int)winIndex2[winNum].size());
			std::vector<int> tempIndex;//Window探索中のみ使用するIndex
			tempIndex.resize(winIndex2[winNum].size());
			int tempSize = 0;
			//窓内を探索---------------------------------------------------------------------------------------------------
			// std::cout<<"task("<<taskIndex[n] % smdCamera.widthInt.data<<","<<taskIndex[n] / smdCamera.widthInt.data<<")-->searching-->\n";
			for(int m=0; m < winIndex2[winNum].size(); m++){
				int pos = taskIndex[n] + winIndex2[winNum][m];
				int posW = taskIndex[n] % smdCamera.widthInt.data;
				int moveW = winIndex2[winNum][m] % smdCamera.widthInt.data;
				//デバッグ用
				// ROS_INFO("%d.size(%d) pos, posW, moveW: (%d, %d, %d)",
				// winNum, (int)(winIndex2[winNum].size()),pos, posW, moveW);
				int taskX= taskIndex[n] % smdCamera.widthInt.data;
				int taskY= taskIndex[n] / smdCamera.widthInt.data;
				int moveX= winIndex2[winNum][m] % smdCamera.widthInt.data;
				int moveY= winIndex2[winNum][m] / smdCamera.widthInt.data;
				if(smdCamera.widthInt.data - moveX < moveX){
					moveX = moveX - smdCamera.widthInt.data;
					moveY = moveY + 1;
				}
				else if(smdCamera.widthInt.data + moveX < -moveX){
					moveX = moveX + smdCamera.widthInt.data;
					moveY = moveY - 1;
				}
				// ROS_INFO("task(%d,%d), move(%d, %d)", taskX, taskY, moveX, moveY);
				int difX = taskX + moveX;
				int difY = taskY + moveY;
				//ここの処理, 後で確認
				int difW = posW + moveW;
				// if(pos < 0 || pos >= (int)mapIndex.size()
				// 	|| difW < 0 || difW > smdCamera.widthInt.data ){
				if(difX < 0 || difX >= smdCamera.widthInt.data
					|| difY < 0 || difY >= smdCamera.heightInt.data ){
						//マップ範囲外検索
						// ROS_INFO("outrange map");
						continue;
				}
				//セルにデータがない場合：スキップ
				if(mapIndex[pos] < 0){
					// ROS_INFO("no data cell");
					continue;
				}
				// std::cout<<"get-->pos("<<pos % smdCamera.widthInt.data<<","<<pos / smdCamera.widthInt.data<<")\n";
				//点の数（密度）を追加
				// std::cout<<"-->Index[pos]:"<<mapIndex[pos]<<std::endl;
				// std::cout<<"-->dens:"<<smdCamera.size[ mapIndex[pos] ]<<std::endl;
				// std::cout<<"-->pos:"<<smdCamera.pt_curr[ mapIndex[pos] ]<<std::endl;
				count += smdCamera.size[ mapIndex[pos] ].data;

				//探査済みの場合：スキップ
				if(searchedIndex[pos]< 0){
					// ROS_INFO("already searched");
					continue;
				}
				//次のタスク候補点（コア点候補）として追加
				tempIndex[tempSize++] = pos;
			}
			// std::cout<<"--> end search \n";
			//リサイズ
			tempIndex.resize(tempSize);

			//点の数（密度）を評価
			// (smdCamera.height/2+smdCamera.cp) : センサからマップの上端までの距離
			// taskIndex[n] / smdCamera.widthInt * smdCamera.res : マップ上端から障害物セルまでの距離
			float y=(smdCamera.height.data/2.0+smdCamera.cp.y) - taskIndex[n] / smdCamera.widthInt.data * smdCamera.res.data;
			//奥行yに対する評価式らしい
			// int minPts=(int)( -35*y*y+1200 );
			// int minPts=10;
			//評価式よりカウントが小さい時: スキップ
			// ROS_INFO("%d < %d",count,minPts);
			if(count < minPts){
				continue;
			}
			//コア点をクラスタに追加
			// データ代入
			//mapIndex[k] <<< k=taskIndex[n] 
			cd.data[clusterNum].size.data += 1;
			cd.data[clusterNum].dens.data += smdCamera.size[ mapIndex[taskIndex[n]] ].data;
			cd.data[clusterNum].gc.x += smdCamera.pt_curr[ mapIndex[taskIndex[n]] ].x*smdCamera.size[ mapIndex[taskIndex[n]] ].data ;
			cd.data[clusterNum].gc.y += smdCamera.pt_curr[ mapIndex[taskIndex[n]] ].y*smdCamera.size[ mapIndex[taskIndex[n]] ].data ;
			cd.data[clusterNum].gc.z += smdCamera.pt_curr[ mapIndex[taskIndex[n]] ].z*smdCamera.size[ mapIndex[taskIndex[n]] ].data ;
			cd.data[clusterNum].index[cd.data[clusterNum].size.data - 1].data = taskIndex[n];//マップセルに対するインデックス, 1データ番号->マップセル
			cd.data[clusterNum].pt[cd.data[clusterNum].size.data - 1] = smdCamera.pt_curr[ mapIndex[taskIndex[n]] ];
			//重複探査防止
			// tempIndex（タスク候補点をタスクに追加）
			// ROS_INFO("tempIndex.size(): %d",(int)tempIndex.size());
			for(int m=0; m < tempIndex.size(); m++){
				//追加
				taskIndex[taskSize++] = tempIndex[m];
				//重複探査防止
				searchedIndex[tempIndex[m]] = -1;
			}
		}
		if(cd.data[clusterNum].size.data==0){
			mapIndex[cd.data[clusterNum].index[0].data] = -1;
			cd.size.data -=1;
			continue;
		}
		//リサイズ
		cd.data[clusterNum].index.resize(cd.data[clusterNum].size.data);
		//追加したデータをマップから削除
		for(int n=0; n < cd.data[clusterNum].index.size(); n++){
			//mapIndex[データ番号[n]->マップ位置] = データ無し
			mapIndex[cd.data[clusterNum].index[n].data] = -1;
		}
			
		//タスクデータ探査終了
		//平均点算出
		cd.data[clusterNum].gc.x /= cd.data[clusterNum].dens.data;
		cd.data[clusterNum].gc.y /= cd.data[clusterNum].dens.data;
		cd.data[clusterNum].gc.z /= cd.data[clusterNum].dens.data;
		//リサイズ
		cd.data[clusterNum].pt.resize(cd.data[clusterNum].size.data);
	}
	// DBSCAN終了
	//データリサイズ
	cd.data.resize(cd.size.data);
}
//追加
//使用する窓を選択
//入力:角度(deg)
//反時計回り, y軸を0度と置く
//出力:使用する窓番号
int classificationClass::selectWindow(int& angle){
	int selectNum = winDivNum / 2;//カメラ正面方向の窓番号
	if(std::abs(angle) > winDivDeg/2){//正面方向ではない
		//angle / winDivDeg
		//第一象限ではマイナス
		//第二象限ではプラス
		//selectNumは正面の番号
		selectNum = selectNum + (angle / winDivDeg);
	}
	return selectNum;
}

void classificationClass::publishClassificationData(){//データ送信
    pub.publish(cd);
}
void classificationClass::clearMessages(){
    smdCamera.index.clear();
    smdCamera.pt_curr.clear();
    smdCamera.size.clear();
    //smdLRF.index.clear();
    //smdLRF.pt_curr.clear();
	cd.data.clear();
}