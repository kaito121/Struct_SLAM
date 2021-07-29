//20210527  
//画像座標の変化具合を知るためにオプティカルフローを使う
#include <ros/ros.h>
#include <cv.h>
#include <highgui.h>

int
main (int argc, char **argv)
{
  char *status;
  int i, corner_count = 150;
  CvPoint2D32f *corners1, *corners2;
  CvTermCriteria criteria;
  IplImage *src_img1, *src_img2, *dst_img;
  IplImage *eig_img, *temp_img;
  IplImage *prev_pyramid, *curr_pyramid;

  	// Optical Flowを計算する前後2フレームを保存するMat
	cv::Mat1b prev_img, curr_img;

	bool first_frame = true;
	std::string filename_base = "/home/fuji/catkin_ws/src/Struct_SLAM/src/OptF/input/20210527_210527_%d.jpg";
	std::string writename_base = "/home/fuji/catkin_ws/src/Struct_SLAM/src/OptF/result/out20210527_210527_%d.jpg";

	for (int i = 1; i < 7000; i++) {

		int start = clock();

		// ファイル名の取得
		std::string filename = cv::format(filename_base.c_str(), i);
		std::cout << "input image name : " << filename << std::endl;

		// 最初の1フレーム目の処理
		if (first_frame == true) {

			// 画像の取得
			prev_img = cv::imread(filename, 0);
			if (prev_img.empty() == true) {
				std::cerr << "【Error】cannot find input image : " << filename << std::endl;
			}
			first_frame = false;
		}

		// 2フレーム目以降の処理
		else {

			// 画像の取得
			curr_img = cv::imread(filename, 0);
			if (curr_img.empty() == true) {
				std::cerr << "【Error】cannot find input image : " << filename << std::endl;
			}

  if (argc != 3 ||
      (src_img1 = cvLoadImage (argv[1], CV_LOAD_IMAGE_GRAYSCALE)) == 0 ||
      (src_img2 = cvLoadImage (argv[2], CV_LOAD_IMAGE_GRAYSCALE)) == 0)
    return -1;

  dst_img = cvLoadImage (argv[2], CV_LOAD_IMAGE_COLOR);

  // (1)必要な構造体の確保
  eig_img = cvCreateImage (cvGetSize (src_img1), IPL_DEPTH_32F, 1);
  temp_img = cvCreateImage (cvGetSize (src_img1), IPL_DEPTH_32F, 1);
  corners1 = (CvPoint2D32f *) cvAlloc (corner_count * sizeof (CvPoint2D32f));
  corners2 = (CvPoint2D32f *) cvAlloc (corner_count * sizeof (CvPoint2D32f));
  prev_pyramid = cvCreateImage (cvSize (src_img1->width + 8, src_img1->height / 3), IPL_DEPTH_8U, 1);
  curr_pyramid = cvCreateImage (cvSize (src_img1->width + 8, src_img1->height / 3), IPL_DEPTH_8U, 1);
  status = (char *) cvAlloc (corner_count);
  criteria = cvTermCriteria (CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 64, 0.01);

  // (2)疎な特徴点を検出
  cvGoodFeaturesToTrack (src_img1, eig_img, temp_img, corners1, &corner_count, 0.001, 5, NULL);

  // (3)オプティカルフローを計算
  cvCalcOpticalFlowPyrLK (src_img1, src_img2, prev_pyramid, curr_pyramid,
                          corners1, corners2, corner_count, cvSize (10, 10), 4, status, NULL, criteria, 0);

  // (4)計算されたフローを描画
  for (i = 0; i < corner_count; i++) {
    if (status[i])
      cvLine (dst_img, cvPointFrom32f (corners1[i]), cvPointFrom32f (corners2[i]), CV_RGB (255, 0, 0), 1, CV_AA, 0);
  }
  cvNamedWindow ("Image", 1);
  cvShowImage ("Image", dst_img);
  cvWaitKey (0);


  cvDestroyWindow ("Image");
  cvReleaseImage (&src_img1);
  cvReleaseImage (&src_img2);
  cvReleaseImage (&dst_img);
  cvReleaseImage (&eig_img);
  cvReleaseImage (&temp_img);
  cvReleaseImage (&prev_pyramid);
  cvReleaseImage (&curr_pyramid);

  return 0;
}
