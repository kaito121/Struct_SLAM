#include<ros/ros.h>
#include<cv.h>
#include<cxcore.h>
#include<highgui.h>
#include<opencv2/opencv.hpp>

public ref class Pixel
{
public:
	Pixel(int x, int y, double angle, double magnitude) 
            : x(x), y(y), angle(angle), magnitude(magnitude) {}
	int x;
	int y;
	double angle;
	double magnitude;
	bool isUsed;
};

void LSD::Run()
{
	// USB cameraで撮影(OpenCV)
    string file_src = "image/59232.jpg";//元画像
    Mat color_imag = imread(file_src, 1);

	//auto color_image = camera_->GrabImage();

	// ぼかす
	cv::GaussianBlur(color_image, color_image, cv::Size(7, 7), 0.6/0.8);

	k

	// 1. Image Scaling
	// Scale factor
	const double s = 0.8; 
	cv::resize(color_image, color_image, cv::Size(), s, s, cv::INTER_CUBIC);

	cv::Mat_<uchar> image;
	cv::cvtColor(color_image, image, CV_BGR2GRAY);

	// 2. Gradient Computation
	data->Clear();
	cv::Mat_<cv::Vec3b> magnitude(image.size());
	ComputeGradient(image, magnitude);

        // System::Drawing::Bitmap^に変換
	camera_image_ = GetBitmap(color_image, 0);
	magnitude_image_ = GetBitmap(magnitude, 1);
}

void LSD::ComputeGradient(cv::Mat_<uchar>& image, cv::Mat_<cv::Vec3b>& m)
{
	for (int y = 0; y < image.rows - 1; y++)
	{
		for (int x = 0; x < image.cols - 1; x++)
		{
			double gx = (image(y, x + 1) + image(y + 1, x + 1) - image(y, x) - image(y + 1, x)) / 2.0;
			double gy = (image(y + 1, x) + image(y + 1, x + 1) - image(y, x) - image(y, x + 1)) / 2.0;

			double angle = atan2(gx, -gy);
			double magnitude = sqrt(gx*gx + gy*gy);

			data->Add(gcnew Pixel(x, y, angle, magnitude));
			pixels_[y, x] = gcnew Pixel(x, y, angle, magnitude);

			m(y, x)[0] = magnitude;
			m(y, x)[1] = magnitude;
			m(y, x)[2] = magnitude;
		}
	}
}