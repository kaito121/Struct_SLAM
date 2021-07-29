//////////////////////////////////////  
//   
// OpenCVで遊ぼう！  
// http://playwithopencv.blogspot.com/  
//  
//CameraCalib.cpp  
//チェスボードに垂直な立方体を描画します。  
//  
//処理の概要  
//１．複数枚のチェスボードの画像を取得し、カメラの内部パラメータを求めます。  
//２．求めた内部パラメータを元に、立方体を描画します。  
//  
//  
//  
#include "stdafx.h"  
#include "cv.h"  
#include "highgui.h"  
  
using namespace System;  
  
  
#define IMAGE_NUM  (10)         /* 記憶させる画像数 */  
#define PAT_ROW    (7)          /* パターンの行数 */  
#define PAT_COL    (10)         /* パターンの列数 */  
#define PAT_SIZE   (PAT_ROW*PAT_COL)  
#define ALL_POINTS (IMAGE_NUM*PAT_SIZE)  
#define CHESS_SIZE (20)       /* パターン1マスの1辺サイズ[mm] */  
  
  
int main(array<System::String ^> ^args)  
{  
 //   Console::WriteLine(L"Hello World");  
//return 0;  
  
  int i, j, k;  
  int corner_count, found;  
  int p_count[IMAGE_NUM];  
  char presskey;  
  
  IplImage *src_img;  
  Cv::Size pattern_size = cv::Size (PAT_COL, PAT_ROW);  
  CvPoint3D32f objects[ALL_POINTS];  
  CvPoint2D32f *corners = (CvPoint2D32f *) cv::Alloc (sizeof (CvPoint2D32f) * ALL_POINTS);  
  CvMat object_points;  
  CvMat image_points;  
  CvMat point_counts;  
  CvMat *intrinsic = cv::CreateMat (3, 3, CV_32FC1);   
  CvMat *distortion = cv::CreateMat (1, 4, CV_32FC1);  
  
  CvMat *rotation = cv::CreateMat (1, 3, CV_32FC1);  
  CvMat *translation = cv::CreateMat (1 , 3, CV_32FC1);  
  //立方体生成用  
  CvMat *srcPoints3D = cv::CreateMat (8, 1, CV_32FC3);//元の3次元座標  
  CvMat *dstPoints2D = cv::CreateMat (8, 1, CV_32FC3);//画面に投影したときの2次元座標  
  
  
  //立方体の座標を決める。  
  int cube_size=5;  //1辺の長さをチェスボードのマス5個分に設定  
  
 for (i=0;i<8;i++)  
 {   
  switch (i)  
  {  
   case 0: srcPoints3D->data.fl[0]     =0;  
        srcPoints3D->data.fl[1]     =0;  
     srcPoints3D->data.fl[2]     =0;  
     break;  
   case 1: srcPoints3D->data.fl[0+i*3] =(float)cube_size*CHESS_SIZE;  
     srcPoints3D->data.fl[1+i*3] =0;  
     srcPoints3D->data.fl[2+i*3] =0;  
     break;  
   case 2: srcPoints3D->data.fl[0+i*3] =(float)cube_size*CHESS_SIZE;  
     srcPoints3D->data.fl[1+i*3] =(float)cube_size*CHESS_SIZE;  
     srcPoints3D->data.fl[2+i*3] =0;  
     break;  
   case 3: srcPoints3D->data.fl[0+i*3] =0;  
     srcPoints3D->data.fl[1+i*3] =(float)cube_size*CHESS_SIZE;  
     srcPoints3D->data.fl[2+i*3] =0;  
     break;  
     
   //後ろ  
   case 4: srcPoints3D->data.fl[0+i*3] =0;  
     srcPoints3D->data.fl[1+i*3] =0;  
     srcPoints3D->data.fl[2+i*3] =(float)cube_size*CHESS_SIZE;  
     break;  
   case 5: srcPoints3D->data.fl[0+i*3] =(float)cube_size*CHESS_SIZE;  
     srcPoints3D->data.fl[1+i*3] =0;  
     srcPoints3D->data.fl[2+i*3] =(float)cube_size*CHESS_SIZE;  
     break;  
   case 6: srcPoints3D->data.fl[0+i*3] =(float)cube_size*CHESS_SIZE;;  
     srcPoints3D->data.fl[1+i*3] =(float)cube_size*CHESS_SIZE;  
     srcPoints3D->data.fl[2+i*3] =(float)cube_size*CHESS_SIZE;  
     break;  
   case 7: srcPoints3D->data.fl[0+i*3] =0;  
     srcPoints3D->data.fl[1+i*3] =(float)cube_size*CHESS_SIZE;   
     srcPoints3D->data.fl[2+i*3] =(float)cube_size*CHESS_SIZE;  
     break;  
   default :srcPoints3D->data.fl[0] =0;  
     srcPoints3D->data.fl[1] =0;   
     srcPoints3D->data.fl[2] =0;  
     break;  
  }  
 }   
  
   //カメラからの取り込み処理  
 IplImage* cap_img;  
 CvCapture *capture;    //開放したり、変更したりするとエラーが起きる  
 capture=cv::CreateCameraCapture(0);  
 cap_img=cv::QueryFrame(capture);  
  
 src_img=cv::CreateImage(cv::GetSize(cap_img),IPL_DEPTH_8U,3);  
 src_img=cv::CloneImage(cap_img);  
    
 IplImage *src_gray = cv::CreateImage (cv::GetSize (src_img), IPL_DEPTH_8U, 1);  
  
 //処理時間計測用  
 double process_time;  
 process_time=(double)cv::GetTickCount();  
  
   
  
 //チェスボードのコーナー検出  
 int found_num = 0;  
 cv::NamedWindow ("Calibration", CV_WINDOW_AUTOSIZE);  
   
 while(found_num<IMAGE_NUM)  
 {  
  cap_img=cv::QueryFrame(capture);  
  src_img=cv::CloneImage(cap_img);  
    
  
  found = cv::FindChessboardCorners (src_img, pattern_size, &corners[found_num * PAT_SIZE], &corner_count);  
    if (found)   
  {  
        
   process_time = (double)cv::GetTickCount() - process_time;  
   printf( "コーナーを見つけました  処理時間 time = %gms\n", process_time/(cv::GetTickFrequency()*1000.));   
      process_time = (double)cv::GetTickCount();  
  
   cv::CvtColor (src_img, src_gray, CV_BGR2GRAY);  
   cv::FindCornerSubPix (src_gray, &corners[found_num * PAT_SIZE], corner_count, cv::Size (3, 3), cv::Size (-1, -1), cv::TermCriteria (CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 20, 0.03));  
   cv::DrawChessboardCorners (src_img, pattern_size, &corners[found_num * PAT_SIZE], corner_count, found);  
   p_count[found_num] = corner_count;  
   found_num++;  
  }  
  else  
  {  
   //何もしない  
   process_time = (double)cv::GetTickCount() - process_time;  
   printf( "コーナーをが見つかりませんでした  処理時間 time = %gms\n", process_time/(cv::GetTickFrequency()*1000.));   
      process_time = (double)cv::GetTickCount();  
  }  
  
  cv::ShowImage ("Calibration", src_img);  
  presskey=cv::WaitKey (200);  
    
  if(presskey==27)  
  {  
   break;  
  }  
  
 }  
    
 if (found_num != IMAGE_NUM)    return -1;  
 cv::InitMatHeader (&image_points, ALL_POINTS, 1, CV_32FC2, corners);  
 cv::InitMatHeader (&point_counts, IMAGE_NUM, 1, CV_32SC1, p_count);  
    
   
 //チェスボードの物理空間での座標設定  
 //各コーナーが物理空間上ではどの座標になるかを指定する。  
 //例：チェスボードのマスのサイズが20mmの場合  
 //コーナー      実際の座標(mm)  
 //   X   Y     X    Y    
 //   0   0   = 0    0  
 //   0   1   = 0    20  
 //   1   0   = 20   0  
 for (i = 0; i < IMAGE_NUM; i++)  
   {  
     for (j = 0; j < PAT_ROW; j++)   
  {        
   for (k = 0; k < PAT_COL; k++)  
   {      
    objects[i * PAT_SIZE + j * PAT_COL + k].x =(float) j * CHESS_SIZE;  
    objects[i * PAT_SIZE + j * PAT_COL + k].y =(float) k * CHESS_SIZE;  
    objects[i * PAT_SIZE + j * PAT_COL + k].z = 0.0;  
   }  
  }  
 }  
  
 cv::InitMatHeader (&object_points, ALL_POINTS, 3, CV_32FC1, objects);  
 cv::Save("object_points.txt",&object_points);  
 cv::Save("image_poits.txt",&image_points);  
 cv::Save("point_counts.txt",&point_counts);  
  
 //内部パラメータを求める。  
 //毎回変わるrotation とtranslationは今回は求めなくてよい  
 cv::CalibrateCamera2 (&object_points, &image_points, &point_counts, cvGetSize(src_img), intrinsic, distortion,NULL,NULL);  
  
   
 cv::Free(&corners);  
   
  
 //*****************************************************  
 //ここまでで内部パラメータの算出が完了したので、  
 //算出したパラメータを基に、これ以降撮影する画像に立方体を描画する。  
 //  
  
  
 //チェスボード1枚分  
 CvPoint3D32f objects_forLoop[PAT_SIZE];  
 CvPoint2D32f *corners_forLoop =(CvPoint2D32f *) cv::Alloc (sizeof (CvPoint2D32f) * PAT_SIZE);  
   
    for (i = 0; i< PAT_ROW; i++)   
 {        
  for (j = 0; j < PAT_COL; j++)  
  {      
   objects_forLoop[i * PAT_COL + j].x =(float) i * CHESS_SIZE;  
   objects_forLoop[i * PAT_COL + j].y =(float) j * CHESS_SIZE;  
   objects_forLoop[i * PAT_COL + j].z = 0.0;  
  }  
 }  
   
  
 CvPoint startpoint;  
 CvPoint endpoint;  
 while(1)  
 {  
  cap_img=cv::QueryFrame(capture);  
  src_img=cv::CloneImage(cap_img);  
    
  found = cv::FindChessboardCorners (src_img, pattern_size, &corners_forLoop[0], &corner_count);  
    if (found)   
  {  
        
   process_time = (double)cv::GetTickCount() - process_time;  
   printf( "コーナーを見つけました  処理時間 time = %gms\n", process_time/(cv::GetTickFrequency()*1000.));   
      process_time = (double)cv::GetTickCount();  
   cv::CvtColor (src_img, src_gray, CV_BGR2GRAY);  
   cv::FindCornerSubPix (src_gray, &corners_forLoop[0], corner_count, cv::Size (3, 3), cv::Size (-1, -1), cv::TermCriteria (CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 20, 0.03));  
  // cvDrawChessboardCorners (src_img, pattern_size, &corners_forLoop[0], corner_count, found);  
   
  
   //チェスボードの現実空間での座標を設定。  
   cv::InitMatHeader (&image_points, PAT_SIZE, 1, CV_32FC2, corners_forLoop);  
   cv::InitMatHeader (&object_points, PAT_SIZE, 3, CV_32FC1, objects_forLoop);  
  
     
   //調整用の画像で求めたカメラの内部定数(intrinsticとdistortion)から、rotationとtranslationを求める   
   cv::FindExtrinsicCameraParams2(&object_points,&image_points,intrinsic,distortion,rotation,translation);  
  
   //求めたものを使用して、現実空間上の座標が画面上だとどの位置に来るかを計算  
   cv::ProjectPoints2(srcPoints3D,rotation,translation,intrinsic,distortion,dstPoints2D);   
  
//   cvSave("srcPoints3D.txt",srcPoints3D);  
//   cvSave("dstPoints2D.txt",dstPoints2D);  
//   cvSave("intrinsic.txt",intrinsic);  
//   cvSave("distortion.txt",distortion);  
//   cvSave("rotation.txt",rotation);  
//   cvSave("translation.txt",translation);//-1.32703932e-003, -1.32703932e-003, -1.32703932e-003   
  
   //立方体を描画  
   //点0から3が立方体の前面、点4から7が立方体の奥面。  
   //まずは前面と奥面を書く  
   for(i=0;i<2;i++)  
   {  
    for(j=0;j<4;j++)  
    {  
     if(j==3)  
     {  
      startpoint=cvPoint((int)dstPoints2D->data.fl[0+(i*4+j)*3],(int)dstPoints2D->data.fl[1+(i*4+j)*3]);  
      endpoint=  cvPoint((int)dstPoints2D->data.fl[0+(i*4+0)*3],(int)dstPoints2D->data.fl[1+(i*4+0)*3]);  
     }  
     else  
     {  
      startpoint=cv::Point((int)dstPoints2D->data.fl[0+(i*4+j)*3], (int)dstPoints2D->data.fl[1+(i*4+j)*3]);  
      endpoint=  cv::Point((int)dstPoints2D->data.fl[0+(i*4+j+1)*3],(int)dstPoints2D->data.fl[1+(i*4+j+1)*3]);  
     }  
       c::vLine(src_img,startpoint,endpoint,cv::Scalar(0,255,0),2,8,0);  
    }   
    
   }  
     
   //2つの面をつなぐ線を描く  
   //0-4 1-5 2-6 3-7  
   for(i=0;i<4;i++)  
   {  
    startpoint=cv::Point((int)dstPoints2D->data.fl[0+(i  )*3],(int)dstPoints2D->data.fl[1+(i   )*3]);  
    endpoint=  cv::Point((int)dstPoints2D->data.fl[0+(i+4)*3],(int)dstPoints2D->data.fl[1+(i+4 )*3]);  
    cv::Line(src_img,startpoint,endpoint,cv::Scalar(0,255,0),2,8,0);  
   }  
  
  }  
  else  
  {  
   //何もしない  
   process_time = (double)c::vGetTickCount() - process_time;  
   printf( "コーナーをが見つかりませんでした  処理時間 time = %gms\n", process_time/(cv::GetTickFrequency()*1000.));   
      process_time = (double)cv::GetTickCount();  
  }  
  
  
  cv::ShowImage ("Calibration", src_img);  
  presskey=cv::WaitKey (20);    
  if(presskey==27)  
  {  
   break;  
  }  
 }  
 cv::ReleaseImage(&src_img);   
 cv::ReleaseImage(&src_gray);  
   
 cv::ReleaseMat(&distortion);  
 cv::ReleaseMat(&intrinsic);  
 cv::ReleaseMat(&rotation);  
 cv::ReleaseMat(&translation);  
 cv::ReleaseMat(&srcPoints3D);  
 cv::ReleaseMat(&dstPoints2D);  
    
 cv::DestroyWindow ("Calibration");  
  
   return 0;  
}  
