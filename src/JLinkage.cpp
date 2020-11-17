#include "JLinkage.h"

#ifndef JL_NO_THREADS
// Mutex for shared variables over threadsスレッド上の共有変数のためのミューテックス
	boost::mutex mMutexMCurrentInvalidatingDistance;
#endif

// Costructorコストレータ
JLinkage::JLinkage(
	float (*nDistanceFunction)(const std::vector<float>  &nModel, const std::vector<float>  &nDataPt), 
	/*The Distance function距離関数*/
	float nInliersThreshold, 
	// Scale of the algorithmアルゴリズムのスケール
	unsigned int nModelBufferSize, 
	// Model buffer size, set to 0 to allow growable size
	//モデルバッファサイズ、0に設定してサイズを拡張可能にします。
	bool nCopyPtsCoords, 
	// Copy the coords value of the new points or use directly the pointer? (The second choice is faster but you need to deallocate memory outside the class)
	//新しいポイントの coords 値をコピーするか、それとも直接ポインタを使用するか (2 番目の選択の方が速いですが、クラスの外にメモリを確保する必要があります)
	unsigned int nPtDimension, 
	//Used by Kd-Tree Kd-Treeで使用しています。
	int nKNeighboards,  
	//K-neighboards for each points to be used for the clusterization (neg value means use all the dataset)
	//Kd-TreeK-neighboardsでクラスタ化に使用する各ポイントに使用される（負の値は全てのデータセットを使用することを意味する
	bool (*nClusterClusterDiscardTest)(const sClLnk *, const sClLnk *), 
	// Function handler to perform a cluster-cluster test. Should return true if the cluster can be merged by some additional condition, false otherwise.
	//クラスタクラスタテストを実行するための関数ハンドラです。クラスタが何らかの追加条件で結合できる場合はtrueを返し、そうでない場合はfalseを返します。
	void (*nInitializeAdditionalData)(sClLnk *), 
	// Function handler -  initialization routine when additional data are created
	//関数ハンドラ - 追加データ作成時の初期化ルーチン
	void (*nClusterMergeAdditionalOperation)(const sClLnk *), 
	// Function handler - called when two clusters are merged
	//関数ハンドラ - 2つのクラスタが結合されたときに呼び出される
	void (*nDestroyAdditionalData)(sClLnk *) 
	// Function handler -  called when additional data are destroyed
	// 関数ハンドラ - 追加データが破棄されたときに呼び出される
):

mDataPoints(), mDataClusters(), mModels(), mDistancesToBeUpdated(), mKDTree(nPtDimension){
	// Max models sizes, set to 0 to allow list
	//最大モデルサイズ、0に設定してリストを表示する
	mModelBufferSize = nModelBufferSize;
	mCurrentModelPointer = 0;
	mCopyPtsCoords = nCopyPtsCoords;
	mDistanceFunction = nDistanceFunction;
	mInliersThreshold = nInliersThreshold;
	if(mModelBufferSize != 0)
		mModels.resize(mModelBufferSize, NULL);
	
	mDistancesToBeUpdatedSize = 0;
	mKNeighboards = nKNeighboards;
	if(mKNeighboards <= 0)
		mUseKDTree = false;
	else
		mUseKDTree = true;
	mClusterClusterDiscardTest = nClusterClusterDiscardTest;
	mClusterMergeAdditionalOperation = nClusterMergeAdditionalOperation;
	mInitializeAdditionalData = nInitializeAdditionalData;
	mDestroyAdditionalData = nDestroyAdditionalData;
	mDataPointsSize = 0;
}


JLinkage::~JLinkage(){

	// Delete all the pointsポイントを全て削除する
	for (std::list<sPtLnk *>::iterator tIter = mDataPoints.begin(); tIter != mDataPoints.end(); ++tIter){
		if(mCopyPtsCoords)
			delete (*tIter)->mCoord;
		delete (*tIter);
	}

	// First delete all the pw jaccard distance information ... 
	//まず、PWジャカードの距離情報を全て削除する・・・。
	for (std::list<sClLnk *>::iterator tIter = mDataClusters.begin(); tIter != mDataClusters.end(); ++tIter){
			for (std::list<sDist *>::iterator tIterDist = (*tIter)->mPairwiseJaccardDistance.begin(); tIterDist != (*tIter)->mPairwiseJaccardDistance.begin(); ++tIterDist){
			(*tIterDist)->mCluster1->mPairwiseJaccardDistance.remove((*tIterDist));
			(*tIterDist)->mCluster2->mPairwiseJaccardDistance.remove((*tIterDist));
			delete (*tIterDist);
		}
	}

	// ... Then Delete all the clusters
	// ... その後、すべてのクラスタを削除します
	for (std::list<sClLnk *>::iterator tIter = mDataClusters.begin(); tIter != mDataClusters.end(); ++tIter){
		if(mDestroyAdditionalData != NULL)
			mDestroyAdditionalData(*tIter);
		delete (*tIter);
	}

	// And finally delete all the models
	// そして最後にすべてのモデルを削除します
	for (std::vector<std::vector<float> *>::iterator tIter = mModels.begin(); tIter != mModels.end(); ++tIter){
		if( (*tIter) == NULL)
			continue;

		if(mCopyPtsCoords)
			delete (*tIter);
	}
}

// Add a new point to the class from an 3d-array. Works only if mCopyPtsCoords is set to true
// 3次元配列から新しい点をクラスに追加します。mCopyPtsCoordsがtrueに設定されている場合のみ動作します。
sPtLnk *JLinkage::AddPoint3d(double nCoordinate[]){
	if(!mCopyPtsCoords)
		return NULL;
	
	std::vector<float> *nCoordinatesVec = new std::vector<float>(3);
	(*nCoordinatesVec)[0] = (float)nCoordinate[0];
	(*nCoordinatesVec)[1] = (float)nCoordinate[1];
	(*nCoordinatesVec)[2] = (float)nCoordinate[2];
	mCopyPtsCoords = false;
	sPtLnk *toReturn= AddPoint(nCoordinatesVec);
	mCopyPtsCoords = true;
	return toReturn;
}

// Add a new point to the class
// クラスに新しいポイントを追加します
sPtLnk *JLinkage::AddPoint(std::vector<float>  *nCoordinates){


	// Create a new point and a new cluster as well
	// 新しいポイントと新しいクラスタを作成します。
	sPtLnk *nPt = new sPtLnk();
	sClLnk *nPtCl = new sClLnk();
	nPt->mBelongingCluster = nPtCl;
	nPtCl->mBelongingPts.push_back(nPt);

	if(mModelBufferSize>0){
		nPt->mPreferenceSet.resize(mModelBufferSize);
		nPtCl->mPreferenceSet.resize(mModelBufferSize);
	}
	else{
		nPt->mPreferenceSet.resize((unsigned int)mModels.size());
		nPtCl->mPreferenceSet.resize((unsigned int)mModels.size());
	}


	if(mCopyPtsCoords){
		nPt->mCoord = new std::vector<float>(*nCoordinates);
	}
	else{
		// Directly use the vector pointer
		// ベクトルポインタを直接利用します
		nPt->mCoord = nCoordinates;
	}

	// Create the Preference Set -> check the distance of every points from the loaded models.
	// プリファレンスセットを作成する→読み込んだモデルからの各ポイントの距離を確認します。
	nPt->mPreferenceSet.resize((unsigned int)mModels.size());
	nPtCl->mPreferenceSet.resize((unsigned int)mModels.size());
	
	// Calcolate the PS of the new point
	// 新しいポイントのPSをカルコレートする
	unsigned int cModelCount = 0;
	for(std::vector<std::vector<float> *>::iterator tIter = mModels.begin(); tIter != mModels.end(); ++tIter){
		if( (*tIter) == NULL)
			continue;

		if(mDistanceFunction((**tIter), *(nPt->mCoord)) < mInliersThreshold){
			nPt->mPreferenceSet[cModelCount] = true;
			nPtCl->mPreferenceSet[cModelCount] = true;
		}
		cModelCount++;
		
		
	}

	if(mUseKDTree){
		// Add the point to the kd-tree
		// kd-treeにポイントを追加します．
		sPtLnkPointer nPtPointer; nPtPointer.mPtLnk = nPt; nPtPointer.mPtLnk->mAlreadyFound = false;
		mKDTree.insert(nPtPointer);
		//mKDTree.optimise();
		nPt->mToBeUpdateKDTree=true;
	}
	// Create the new distances(to be updated next)
	// 新しい距離を作成します(次回更新予定)
	if(!mUseKDTree){
		// Update for all the points
		// 全ポイント分の更新
		for(std::list<sClLnk *>::iterator tIter = mDataClusters.begin(); tIter != mDataClusters.end(); ++tIter){
			
			sDist *tDist = new sDist;
			tDist->mCluster1 = nPtCl;
			tDist->mCluster2 = *tIter;
			
			nPtCl->mPairwiseJaccardDistance.push_back(tDist);
			(*tIter)->mPairwiseJaccardDistance.push_back(tDist);
			
			tDist->mToBeUpdated = true;
			mDistancesToBeUpdated.push_back(tDist);
			mDistancesToBeUpdatedSize++;
		}
	}
	// else Update the distance list before doing J-Linkage clusterization.
	// else J-Linkageクラスタ化を行う前に距離リストを更新します
	
	// Add the point and the new cluster to the list
	// ポイントと新しいクラスタをリストに追加します
	mDataPoints.push_back(nPt);
	mDataClusters.push_back(nPtCl);
	nPt->mAddedIdx = mDataPointsSize;
	mDataPointsSize++;

	if(mInitializeAdditionalData != NULL)
		mInitializeAdditionalData(nPtCl);

	return nPt;

}

// Remove a point from the class - Also delete the pointer information
// クラスからポイントを削除する - ポインタ情報も削除する
void JLinkage::RemovePoint(sPtLnk *nPt){

	sClLnk *nPtCl = nPt->mBelongingCluster;
		
	nPtCl->mBelongingPts.remove(nPt);

	if(nPtCl->mBelongingPts.size() == 0){ 
	// No points in the cluster - Remove the cluster
	// クラスタ内にポイントがありません - クラスタを削除します

		// First delete all the pw jaccard distance information ... 
		// 最初にpwジャカードの距離情報を全て削除する ... 
		for (std::list<sDist *>::iterator tIterDist = nPtCl->mPairwiseJaccardDistance.begin(); tIterDist != nPtCl->mPairwiseJaccardDistance.end(); ++tIterDist){
			
			if((*tIterDist)->mCluster1 != nPtCl)
				(*tIterDist)->mCluster1->mPairwiseJaccardDistance.remove((*tIterDist));
			else
				(*tIterDist)->mCluster2->mPairwiseJaccardDistance.remove((*tIterDist));
			if((*tIterDist)->mToBeUpdated){
				mDistancesToBeUpdated.remove((*tIterDist));
				mDistancesToBeUpdatedSize--;
			}

			delete (*tIterDist);
		}

		nPtCl->mPairwiseJaccardDistance.clear();

		// ... Then Delete all the cluster
		// ... その後、すべてのクラスタを削除します。
		mDataClusters.remove(nPtCl);
		delete nPtCl;	
	}
	else{
		// Update cluster information
		// クラスタ情報の更新
		nPtCl->mPreferenceSet = (*(nPtCl->mBelongingPts.begin()))->mPreferenceSet;

		for (std::list<sPtLnk *>::iterator tIter = nPtCl->mBelongingPts.begin(); tIter != nPtCl->mBelongingPts.end(); ++tIter){
			nPtCl->mPreferenceSet &= (*tIter)->mPreferenceSet;
		}	

	}


	if(mDestroyAdditionalData != NULL)
		mDestroyAdditionalData(nPtCl);
	
	if(mUseKDTree){
		// erase the point from the kd-tree
		// kd-tree から点を削除します．
		sPtLnkPointer nPtPointer; nPtPointer.mPtLnk = nPt; nPtPointer.mPtLnk->mAlreadyFound = false;
		mKDTree.erase(nPtPointer);
		mKDTree.optimise();
		nPt->mToBeUpdateKDTree = false;
	}
	
	mDataPoints.remove(nPt);

	if(mCopyPtsCoords)
		delete nPt->mCoord;

	delete nPt;
}

// Add a new model to the buffer - update the ps of all points and clusters
// バッファに新しいモデルを追加します - すべての点とクラスタのpsを更新します
unsigned int JLinkage::AddModel(std::vector<float>  *nModel){

	// If a model already exist in the position, remove it
	// モデルが既にその位置に存在する場合は、それを削除します
	if(mModelBufferSize != 0 && mCurrentModelPointer < mModels.size()){
		RemoveModel(mCurrentModelPointer);
	}

	// Add the model to the buffer list
	// バッファリストにモデルを追加します．
	if(mModelBufferSize == 0)
		if(mCopyPtsCoords)
			mModels.push_back(new std::vector<float>(*nModel));
		else
			mModels.push_back(nModel);
	else
		if(mCopyPtsCoords)
			mModels[mCurrentModelPointer] = new std::vector<float>(*nModel);
		else
			mModels[mCurrentModelPointer] = nModel;

	// Update the preference set of all the points
	// すべてのポイントの環境設定を更新します
	for(std::list<sPtLnk *>::iterator tIter = mDataPoints.begin(); tIter != mDataPoints.end(); tIter++){
		if(mModelBufferSize == 0)
			(*tIter)->mPreferenceSet.resize(mCurrentModelPointer+1);
		if(mDistanceFunction((*nModel), *((*tIter)->mCoord)) < mInliersThreshold){
			(*tIter)->mPreferenceSet[mCurrentModelPointer] = true;
		}
	}

	// Update the preference set of all the clusters
	// すべてのクラスタの環境設定を更新します
	for(std::list<sClLnk *>::iterator tIterCl = mDataClusters.begin(); tIterCl != mDataClusters.end(); tIterCl++){

		if(mModelBufferSize == 0)
			(*tIterCl)->mPreferenceSet.resize(mCurrentModelPointer+1);

		bool allTrue = true;

		for(std::list<sPtLnk *>::iterator tIterPt = (*tIterCl)->mBelongingPts.begin(); tIterPt != (*tIterCl)->mBelongingPts.end(); tIterPt++){
			if(!(*tIterPt)->mPreferenceSet[mCurrentModelPointer]){
				allTrue = false;
				break;
			}
		}

		// If the model set all to true we need a distance update
		// モデルがすべてtrueに設定されている場合、距離を更新する必要があります
		if(allTrue){
			(*tIterCl)->mPreferenceSet[mCurrentModelPointer] = true;
			for(std::list<sDist *>::iterator tIterDist = (*tIterCl)->mPairwiseJaccardDistance.begin(); tIterDist != (*tIterCl)->mPairwiseJaccardDistance.end(); tIterDist++){
				if(!(*tIterDist)->mToBeUpdated){
					mDistancesToBeUpdated.push_back((*tIterDist));
					mDistancesToBeUpdatedSize++;
					(*tIterDist)->mToBeUpdated = true;
				}
				// Else already added to the update list
				// その他、すでに更新リストに追加されているもの
			}
		}
		else{
			(*tIterCl)->mPreferenceSet[mCurrentModelPointer] = false;
		}
	}



	unsigned int toReturn = mCurrentModelPointer;

	// Update the adder pointer
	// 加算器ポインタを更新します．
	if(mModelBufferSize == 0)
		++mCurrentModelPointer;
	else
		mCurrentModelPointer = (++mCurrentModelPointer)%mModelBufferSize;


	return toReturn;
}

const std::vector<float>  *JLinkage::GetModel(unsigned int nModel){


	assert (nModel < mModels.size());

	if (nModel >= mModels.size())
		return NULL;

	return mModels[nModel];
}

// Remove a model from the buffer// バッファからモデルを削除します．
// Helper function called only by addModel// addModel からのみ呼び出されるヘルパー関数
void JLinkage::RemoveModel(unsigned int nModelN){

	assert(nModelN < mModels.size());

	// Check if already removed// 既に削除されているかどうかを確認します。
	if(mModels[nModelN] == NULL){
		return;
	}

	if(mCopyPtsCoords)
		delete mModels[nModelN];


	mModels[nModelN] = NULL;

	// Update all the pts preference set// すべての pts 環境設定を更新します
	for(std::list<sPtLnk *>::iterator tIter = mDataPoints.begin(); tIter != mDataPoints.end(); tIter++){
		(*tIter)->mPreferenceSet[nModelN] = false;
	}

	// And all the cluster preferences sets// そして、すべてのクラスタ環境設定セット
	// Update the preference set of all the clusters// すべてのクラスタの環境設定を更新します
	for(std::list<sClLnk *>::iterator tIterCl = mDataClusters.begin(); tIterCl != mDataClusters.end(); tIterCl++){

		// If the preferencet of the cluster for the model was true, we need a distance update
		// モデルに対するクラスタの優先度が真であった場合、距離の更新が必要となります
		if((*tIterCl)->mPreferenceSet[nModelN]){
			(*tIterCl)->mPreferenceSet[mCurrentModelPointer] = false;
			for(std::list<sDist *>::iterator tIterDist = (*tIterCl)->mPairwiseJaccardDistance.begin(); tIterDist != (*tIterCl)->mPairwiseJaccardDistance.end(); tIterDist++){
				if(!(*tIterDist)->mToBeUpdated){
					mDistancesToBeUpdated.push_back((*tIterDist));
					mDistancesToBeUpdatedSize++;
					(*tIterDist)->mToBeUpdated = true;
				}
				// Else already added to the update list
			}
		}
	}


}

const std::list<sClLnk *> JLinkage::DoJLClusterization(void (*OnProgress)(float)){
	
	
	float tEps = (float)1.0f/(1.0f+(float)mModels.size()); 
	// Just a distance to see if the jaccard distance is equal to one// ジャカードの距離が1に等しいかどうかを確認するだけの距離
	float tOneEpsDistance = 1.0f - tEps; 
	// Just a distance to see if the jaccard distance is equal to one// ジャカードの距離が1に等しいかどうかを確認するだけの距離

	
	// Update neighboardhood information// 近隣の掲示板情報を更新する
	if(mUseKDTree){
		mKDTree.optimise();
		
		/* old FIND WITHING RANGE
		static std::vector<sPtLnkPointer> nearPtsIdx;
	
		static bool firsttime = true;
		if(firsttime){
			nearPtsIdx.reserve(500);
			firsttime = false;
		}
		*/
		for(std::list<sPtLnk *>::iterator tIterPt = mDataPoints.begin(); tIterPt != mDataPoints.end(); ++tIterPt){
			if(!(*tIterPt)->mToBeUpdateKDTree)
				continue;
			// Threadable build Distance list for kdtree // kdtree用のスレッド可能なビルド距離リスト 


			/* old FIND WITHING RANGE
			nearPtsIdx.clear();
			sPtLnkPointer nPtPointer; nPtPointer.mPtLnk = *tIterPt;
			mKDTree.find_within_range(nPtPointer, mNNeighboardsUpdateDistance, std::back_inserter(nearPtsIdx));
			*/
			sPtLnkPointer nPtPointer; nPtPointer.mPtLnk = *tIterPt; nPtPointer.mPtLnk->mAlreadyFound = false;
			std::list<sPtLnkPointer> kneigh = FindKNearest(nPtPointer,std::numeric_limits<float>::max(),mKNeighboards, (int)mDataPointsSize);

			// Find K-Nearest// 最寄のKを探す
			for(std::list<sPtLnkPointer>::iterator tIter = kneigh.begin(); tIter != kneigh.end(); ++tIter){
			
			//for(std::vector<sPtLnkPointer>::iterator tIter = nearPtsIdx.begin(); tIter != nearPtsIdx.end(); ++tIter){
				if((*tIter).mPtLnk->mBelongingCluster == (*tIterPt)->mBelongingCluster)
					continue;				
				
				bool alreadyPresent = false;
				for(std::list<sDist *>::iterator distIter = (*tIter).mPtLnk->mBelongingCluster->mPairwiseJaccardDistance.begin();
					distIter != (*tIter).mPtLnk->mBelongingCluster->mPairwiseJaccardDistance.end();
					distIter++){
					
					if((*distIter)->mCluster1 == (*tIterPt)->mBelongingCluster || (*distIter)->mCluster2 == (*tIterPt)->mBelongingCluster)
						alreadyPresent = true;
				}

				if(alreadyPresent)
					continue;

				sDist *tDist = new sDist;
				tDist->mCluster1 = (*tIterPt)->mBelongingCluster;
				tDist->mCluster2 = (*tIter).mPtLnk->mBelongingCluster;
				
				(*tIterPt)->mBelongingCluster->mPairwiseJaccardDistance.push_back(tDist);
				(*tIter).mPtLnk->mBelongingCluster->mPairwiseJaccardDistance.push_back(tDist);
				
				tDist->mToBeUpdated = true;
				mDistancesToBeUpdated.push_back(tDist);
				mDistancesToBeUpdatedSize++;
			}
			(*tIterPt)->mToBeUpdateKDTree = false;			
		}
		
	}
	
	if(OnProgress != NULL)
		OnProgress(0.01f);
	
	// First step: update all the pw distances that needs an update// 最初のステップ: 更新が必要なすべての pw 距離を更新します。
	// Please Note: If a distance don't need to be updated it means that it would be certainly equal to 1 from the previous JLClusterization
	// 注意: 距離を更新する必要がない場合は、前回の JLClusterization からの距離が 1 になることを意味します。
	// --> Store also a list with ALL the pairwise unique jaccard distance
	// --> ペアごとに一意なジャカード距離のリストも格納します
	std::vector<sDist *> mHeapDistanceList(mDistancesToBeUpdatedSize);	
	
	unsigned int counter = 0;
	while(mDistancesToBeUpdatedSize > 0){
		sDist *tDist = mDistancesToBeUpdated.back();
		mDistancesToBeUpdated.pop_back();
		mDistancesToBeUpdatedSize--;
		tDist->mPairwiseJaccardDistance = 
			PSJaccardDist(	tDist->mCluster1->mPreferenceSet,
							tDist->mCluster2->mPreferenceSet,
							&(tDist->mPairwiseUnion),
							&(tDist->mPairwiseIntersection) );
		tDist->mToBeUpdated = false;
		if(tDist->mPairwiseJaccardDistance < tOneEpsDistance){
			mHeapDistanceList[counter] = tDist;
			++counter;
		}
	}

	if(OnProgress != NULL)
		OnProgress(0.02f);

	mHeapDistanceList.resize(counter);

	// A distance that will invalidate the heap, needing a heap resort// ヒープを無効にする距離で，ヒープリゾートが必要です．
	float mCurrentInvalidatingDistance = 1.0f;

	// Make the heap// ヒープを作成する
	std::sort(mHeapDistanceList.begin(), mHeapDistanceList.end(), sDist());
	
	unsigned int currentSortIdx = 0;
	unsigned int initialDistance = (unsigned int)mHeapDistanceList.size();
		
	while(mHeapDistanceList.size() > 0){
		
		
		sDist *sCurrentMinDist = NULL;
		if(currentSortIdx < mHeapDistanceList.size())
			sCurrentMinDist = mHeapDistanceList[currentSortIdx];
		// TODO speed up previous line!// TODO 前の行を高速化します!
		
	    if(sCurrentMinDist == NULL || sCurrentMinDist->mPairwiseJaccardDistance > tOneEpsDistance && mCurrentInvalidatingDistance > tOneEpsDistance  ){ 
			// All the distance will be equals to one - clusterization is finished
			// すべての距離が1に等しくなります - クラスタ化は終了しました
			// We've finished since all distances have been processed or are equal to 1.0f
			// すべての距離が処理されたか、1.0f と等しくなったので終了しました。
			mHeapDistanceList.clear();
		}
		else if(sCurrentMinDist->mCluster1 == NULL || sCurrentMinDist->mCluster2 == NULL ){
			// Eliminate the non-valid distance(belong to an eliminated clusters
			// 非有効距離を消去する(消去されたクラスタに属する)
			for(std::vector<sDist *>::iterator tIterDist = mHeapDistanceList.begin() + currentSortIdx + 1 ; tIterDist != mHeapDistanceList.end(); tIterDist++){
				assert((*tIterDist) != sCurrentMinDist);
			}
			delete sCurrentMinDist;
			++currentSortIdx;
		}
		else if(sCurrentMinDist->mPairwiseJaccardDistance > (mCurrentInvalidatingDistance-tEps)){ // We need an heap resort// ヒープリゾートが必要
			
			mHeapDistanceList.erase(mHeapDistanceList.begin(), mHeapDistanceList.begin()+(currentSortIdx));
			
			if(mHeapDistanceList.size() > 1){

				// First eliminate all the distance equals to one from the heap
				// 最初にヒープから1に等しい距離をすべて消去する
				// Push all these distances to the end of the vector...
				// これらの距離をすべてベクトルの端に押し込む...
				unsigned int idxElementProcessing = 0;
				unsigned int idxLastUsefullElement = (unsigned int)mHeapDistanceList.size() - 1;
				
				while(idxElementProcessing <= idxLastUsefullElement && idxLastUsefullElement > 0 ){
					
					if(mHeapDistanceList[idxElementProcessing]->mPairwiseJaccardDistance > tOneEpsDistance || mHeapDistanceList[idxElementProcessing]->mCluster1 == NULL || mHeapDistanceList[idxElementProcessing]->mCluster2 == NULL){
						// In this case we need to move the distance to the end
						// この場合は最後まで距離を移動させる必要があります

						// Swap elements// 要素を入れ替える
						sDist *temp = mHeapDistanceList[idxElementProcessing];
						mHeapDistanceList[idxElementProcessing] = mHeapDistanceList[idxLastUsefullElement];
						mHeapDistanceList[idxLastUsefullElement] = temp;

						// If the distance belongs to one eliminated cluster, delete it
						// 距離が除去されたクラスタに属している場合は，そのクラスタを削除します．
						if(mHeapDistanceList[idxLastUsefullElement]->mCluster1 == NULL || mHeapDistanceList[idxLastUsefullElement]->mCluster2 == NULL)
							delete mHeapDistanceList[idxLastUsefullElement];

						idxLastUsefullElement--;
					}
					else{
						idxElementProcessing++;
					}
				}
				
				if(idxLastUsefullElement > 0){
					// ... and then erase them// ... そしてそれらを消去する
					if(idxLastUsefullElement < mHeapDistanceList.size()-1)
						mHeapDistanceList.erase(mHeapDistanceList.begin()+idxLastUsefullElement+1, mHeapDistanceList.end());
					// Re-Set the heap// ヒープを再設定します。
					std::sort(mHeapDistanceList.begin(), mHeapDistanceList.end(), sDist());
				}
				else{
					// Ok we finished// 終わった
					mHeapDistanceList.clear();
				}
								
			}
			
			mCurrentInvalidatingDistance = 2.0f;
			currentSortIdx = 0;
		}
		else{
			// The distance is less than the invalidating distance, merge the two cluster and update the other distances accordingly
			// 距離が無効化距離よりも小さい場合は、2つのクラスタをマージし、それに応じて他の距離を更新します。
			// if kd-tree is used, merge the distances of the clusters for adding the new ones of the new created cluster
			// kd-treeを使用している場合は、新たに作成されたクラスタの新しいクラスタを追加するためにクラスタの距離をマージします。
			std::list<sClLnk *> distancesToBeAdded;

			sCurrentMinDist->mCluster2->mPairwiseJaccardDistance.remove(sCurrentMinDist);
			sCurrentMinDist->mCluster1->mPairwiseJaccardDistance.remove(sCurrentMinDist);

			if(mUseKDTree){
				for(std::list<sDist *>::iterator distIter1 = sCurrentMinDist->mCluster2->mPairwiseJaccardDistance.begin(); distIter1 != sCurrentMinDist->mCluster2->mPairwiseJaccardDistance.end(); ++distIter1){
					// Threadable Check distance to merge KD-Tree
					// KD-Treeをマージするためのスレッド可能なチェック距離
					bool add = true;
					// Check if the distance already exists
					// 距離が既に存在するかどうかをチェック
					for(std::list<sDist *>::iterator distIter2 = sCurrentMinDist->mCluster1->mPairwiseJaccardDistance.begin(); distIter2 != sCurrentMinDist->mCluster1->mPairwiseJaccardDistance.end(); ++distIter2){
						if((*distIter1)->mCluster1 == (*distIter2)->mCluster1 || (*distIter1)->mCluster2 == (*distIter2)->mCluster2 
						   || (*distIter1)->mCluster2 == (*distIter2)->mCluster1 || (*distIter1)->mCluster1 == (*distIter2)->mCluster2){
							add = false; // Point already present// 既に存在しているポイント
							break;
						}
					}
					if(add){
						if((*distIter1)->mCluster1 != sCurrentMinDist->mCluster2)
							distancesToBeAdded.push_back((*distIter1)->mCluster1);
						else if((*distIter1)->mCluster2 != sCurrentMinDist->mCluster2)
							distancesToBeAdded.push_back((*distIter1)->mCluster2);
					}
					
				}
			}
			
			
			// Update the cluster pointer of all the points of the deleted cluster
			// 削除されたクラスタの全ポイントのクラスタポインタを更新します．
			for(std::list<sPtLnk *>::iterator ptIter = sCurrentMinDist->mCluster2->mBelongingPts.begin(); ptIter != sCurrentMinDist->mCluster2->mBelongingPts.end(); ptIter++)
				(*ptIter)->mBelongingCluster = sCurrentMinDist->mCluster1;

						
			// Merge the two clusters into cluster 1// 2つのクラスタをクラスタ1に統合します。
			sCurrentMinDist->mCluster1->mPreferenceSet &= sCurrentMinDist->mCluster2->mPreferenceSet;
			sCurrentMinDist->mCluster1->mBelongingPts.merge(sCurrentMinDist->mCluster2->mBelongingPts);			

			
			// Delete cluster 2// クラスタ2を削除
			mDataClusters.remove(sCurrentMinDist->mCluster2);

			if(mUseKDTree){
				for(std::list<sClLnk *>::iterator clIter = distancesToBeAdded.begin(); clIter != distancesToBeAdded.end(); ++clIter){
					// Threadable Add kd-tree distances// スレッド可能 kd-treeの距離を追加
					sDist *tDist = new sDist;
					tDist->mCluster1 = sCurrentMinDist->mCluster1;
					tDist->mCluster2 = *clIter;
					
					sCurrentMinDist->mCluster1->mPairwiseJaccardDistance.push_back(tDist);
					(*clIter)->mPairwiseJaccardDistance.push_back(tDist);
					mHeapDistanceList.push_back(tDist);
				}
			}
			
				
			
			// Update all the distances of the old cluster -- delete
			// 古いクラスタのすべての距離を更新します -- delete
			for(std::list<sDist *>::iterator tIterDist = sCurrentMinDist->mCluster2->mPairwiseJaccardDistance.begin(); tIterDist != sCurrentMinDist->mCluster2->mPairwiseJaccardDistance.end(); tIterDist++){
				// Threadable: delete distance for the old cluster
				// Threadable: 古いクラスタの距離を削除します。
				if(sCurrentMinDist != (*tIterDist)){
					if((*tIterDist)->mCluster1 != sCurrentMinDist->mCluster2)
						(*tIterDist)->mCluster1->mPairwiseJaccardDistance.remove( (*tIterDist));
					else if((*tIterDist)->mCluster2 != sCurrentMinDist->mCluster2)
						(*tIterDist)->mCluster2->mPairwiseJaccardDistance.remove( (*tIterDist));
					(*tIterDist)->mCluster1 = NULL;
					(*tIterDist)->mCluster2 = NULL;
				}
			}

			// Do additional user-defined update steps if specified
			if(mClusterMergeAdditionalOperation != NULL)
				mClusterMergeAdditionalOperation(sCurrentMinDist->mCluster1);

#ifndef JL_NO_THREADS
				boost::thread_group *tg = new boost::thread_group();
				// Update all the distances of the new cluster
				// 指定された場合は、ユーザー定義の追加更新ステップを実行します。
				for(std::list<sDist *>::iterator tIterDist = sCurrentMinDist->mCluster1->mPairwiseJaccardDistance.begin(); tIterDist != sCurrentMinDist->mCluster1->mPairwiseJaccardDistance.end(); tIterDist++){
					// Update distances// 距離を更新する
					tg->create_thread(boost::bind(&(UpdateDistance),*tIterDist, &mCurrentInvalidatingDistance, mClusterClusterDiscardTest));
					if(tg->size() >= MAXTHREADS){
						tg->join_all();
						delete tg;
						tg = new boost::thread_group();
					}
				}
			tg->join_all();
			delete tg;
#else
				for(std::list<sDist *>::iterator tIterDist = sCurrentMinDist->mCluster1->mPairwiseJaccardDistance.begin(); tIterDist != sCurrentMinDist->mCluster1->mPairwiseJaccardDistance.end(); tIterDist++){
					// Update distances// 距離を更新する
					JLinkage::UpdateDistance(*tIterDist, &mCurrentInvalidatingDistance, mClusterClusterDiscardTest);
				}

#endif

			if(OnProgress != NULL)
				OnProgress(1.0f - ((float) (mHeapDistanceList.size() - currentSortIdx) / (float)initialDistance));
			
			// Delete old cluster// 古いクラスタを削除します。
			if(sCurrentMinDist->mCluster2 && mDestroyAdditionalData)
				mDestroyAdditionalData(sCurrentMinDist->mCluster2);
		
			delete sCurrentMinDist->mCluster2;
			delete sCurrentMinDist;

			++currentSortIdx;

		}



	}

	if(OnProgress != NULL)
		OnProgress(1.0f);
	
	// return the list of clusters// クラスタのリストを返します
	return mDataClusters;
}

void JLinkage::UpdateDistance(sDist *tIterDist, float *nCurrentInvalidatingDistance, bool (*nClusterClusterDiscardTest)(const sClLnk *, const sClLnk *)){
			tIterDist->mPairwiseJaccardDistance = 
				PSJaccardDist(	tIterDist->mCluster1->mPreferenceSet,
								tIterDist->mCluster2->mPreferenceSet,
								&(tIterDist->mPairwiseUnion),
								&(tIterDist->mPairwiseIntersection) );

		// If pw distance is < 1, Do also additional test if a function is specified
		// pw距離が1未満の場合、関数が指定されている場合は追加テストも行う
		if(tIterDist->mPairwiseJaccardDistance < 1.0f 
			&& (nClusterClusterDiscardTest != NULL && !nClusterClusterDiscardTest(tIterDist->mCluster1, tIterDist->mCluster2)))
			tIterDist->mPairwiseJaccardDistance = 1.0f;

#ifndef JL_NO_THREADS
		boost::mutex::scoped_lock
			lock(mMutexMCurrentInvalidatingDistance);
#endif
		// Store the minimum updated distance
		// 更新された最小距離を格納します
		if(tIterDist->mPairwiseJaccardDistance < *nCurrentInvalidatingDistance)
			*nCurrentInvalidatingDistance = tIterDist->mPairwiseJaccardDistance;
}

// Find K-nearest neighboard from the kd-tree// kd-treeからK-nearest neighboardを探す
std::list<sPtLnkPointer> JLinkage::FindKNearest(sPtLnkPointer __V, float const __Max_R, int k, int nMaxSize){
		int msSizePRAF = 0;
		std::list<sPtLnkPointer> msAlreadyFoundPRAF;
		__V.mPtLnk->mAlreadyFound = true;
		while(msSizePRAF < k && msSizePRAF < nMaxSize ){
			std::pair<KDTree::KDTree<sPtLnkPointer>::iterator,float> nif = mKDTree.find_nearest_if(__V,__Max_R,sPredicateAlreadyFoundJL());
			msAlreadyFoundPRAF.push_back(*nif.first);
			msAlreadyFoundPRAF.back().mPtLnk->mAlreadyFound = true;
			msSizePRAF++;
		}
		// Reset values// 値をリセットします。
		for(std::list<sPtLnkPointer>::iterator it = msAlreadyFoundPRAF.begin(); it != msAlreadyFoundPRAF.end(); ++it)
			(*it).mPtLnk->mAlreadyFound = false;
		__V.mPtLnk->mAlreadyFound = false;

	  return msAlreadyFoundPRAF;
}

// Manually specify a cluster of points. Idx are relative to pts not clusters.
// ポイントのクラスタを手動で指定します。Idxはクラスタではなく、ポイントに対する相対的なものです。
// Merge only singleton points (not belonging to other cluster), and the preference set contain at least a model.
// (他のクラスタに属していない)シングルトンポイントのみをマージし，プリファレンスセットには少なくともモデルが含まれます
// If this does not happens the manual merge is discarded and the function return false
// これが起こらなかった場合は、手動マージは破棄され、関数は false を返します。
bool JLinkage::ManualClusterMerge(std::vector<unsigned int> nPtIdxOfCluster){

	// Sort to be more efficient when traversing the pts list
	// ptsリストをトラバースする際に効率的にソートします。
	std::sort(nPtIdxOfCluster.begin(), nPtIdxOfCluster.end());

	// First check conditions// ファーストチェックの条件
	if(nPtIdxOfCluster.size() == 0 || nPtIdxOfCluster[nPtIdxOfCluster.size() - 1] >= GetPointsN())
		return false;

	bvect newPS;
	std::list<sPtLnk *> ptsToMerge;
	sClLnk *nNewCluster;
	unsigned int curptIdx = 0;

	for(std::list<sPtLnk *>::iterator tIter = mDataPoints.begin(); tIter != mDataPoints.end(); tIter++){
		if((*tIter)->mAddedIdx == nPtIdxOfCluster[curptIdx]){
			if(curptIdx == 0){
				newPS = (*tIter)->mPreferenceSet;
				nNewCluster = (*tIter)->mBelongingCluster;
			}
			else{							
				newPS &= (*tIter)->mPreferenceSet;
			}
			// Cluster non singleton// シングルトンではないクラスタ
			if((*tIter)->mBelongingCluster->mBelongingPts.size() > 1)
				return false;

			ptsToMerge.push_back(*tIter);
			curptIdx++;
			if(curptIdx >= nPtIdxOfCluster.size())
				break;
		}
	}

	// PS not compatible// PSとの互換性がありません

	if(newPS.count() == 0)
		return false;


	nNewCluster->mBelongingPts = ptsToMerge;
	nNewCluster->mPreferenceSet = newPS;

	if(mUseKDTree)
		for(std::list<sPtLnk *>::iterator tIter = nNewCluster->mBelongingPts.begin(); tIter != nNewCluster->mBelongingPts.end(); tIter++)		
			(*tIter)->mToBeUpdateKDTree = true;
	else
		for(std::list<sDist *>::iterator tIterD = nNewCluster->mPairwiseJaccardDistance.begin(); tIterD != nNewCluster->mPairwiseJaccardDistance.end(); ++tIterD)
			(*tIterD)->mToBeUpdated = true;

	ptsToMerge.pop_front();

	// now make every point belonging to the new cluster
	// すべての点を新しいクラスタに属するようにします
	for(std::list<sPtLnk *>::iterator tIter = ptsToMerge.begin(); tIter != ptsToMerge.end(); tIter++){
		
		mDataClusters.remove((*tIter)->mBelongingCluster);		
				
		// Update all the distances of the old cluster -- delete// 古いクラスタのすべての距離を更新します -- delete
		for(std::list<sDist *>::iterator tIterDist = (*tIter)->mBelongingCluster->mPairwiseJaccardDistance.begin(); tIterDist != (*tIter)->mBelongingCluster->mPairwiseJaccardDistance.end(); tIterDist++){
			if((*tIterDist)->mCluster1 != (*tIter)->mBelongingCluster)
				(*tIterDist)->mCluster1->mPairwiseJaccardDistance.remove( (*tIterDist));
			else if((*tIterDist)->mCluster2 != (*tIter)->mBelongingCluster)
				(*tIterDist)->mCluster2->mPairwiseJaccardDistance.remove( (*tIterDist));
			delete *tIterDist;
		}

		if(mDestroyAdditionalData)
			mDestroyAdditionalData((*tIter)->mBelongingCluster);

		// Delete old cluster	// 古いクラスタを削除します。		
		delete (*tIter)->mBelongingCluster;
		
		(*tIter)->mBelongingCluster = nNewCluster;

	}

	return true;
}
