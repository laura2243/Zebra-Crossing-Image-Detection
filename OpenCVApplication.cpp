#include <iostream>
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
// OpenCVApplication.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include "common.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>


void testOpenImage()
{
	char fname[MAX_PATH];
	while (openFileDlg(fname))
	{
		Mat src;
		src = imread(fname);
		imshow("opened image", src);
		waitKey();
	}
}

void testOpenImagesFld()
{
	char folderName[MAX_PATH];
	if (openFolderDlg(folderName) == 0)
		return;
	char fname[MAX_PATH];
	FileGetter fg(folderName, "bmp");
	while (fg.getNextAbsFile(fname))
	{
		Mat src;
		src = imread(fname);
		imshow(fg.getFoundFileName(), src);
		if (waitKey() == 27) //ESC pressed
			break;
	}
}

void testColor2Gray()
{
	char fname[MAX_PATH];
	while (openFileDlg(fname))
	{
		Mat_<Vec3b> src = imread(fname, IMREAD_COLOR);

		int height = src.rows;
		int width = src.cols;

		Mat_<uchar> dst(height, width);

		for (int i = 0; i < height; i++)
		{
			for (int j = 0; j < width; j++)
			{
				Vec3b v3 = src(i, j);
				uchar b = v3[0];
				uchar g = v3[1];
				uchar r = v3[2];
				dst(i, j) = (r + g + b) / 3;
			}
		}

		imshow("original image", src);
		imshow("gray image", dst);
		waitKey();
	}
}
void resizeImage() {
	char fname[MAX_PATH];
	while (openFileDlg(fname)) {
		Mat src, dst, gauss, small_img;
		src = imread(fname, IMREAD_COLOR);
		int down_width = src.cols - 0.9 * src.cols;
		int down_height = src.rows - 0.9 * src.rows;
		Mat resized_down;
		//resize down
		resize(src, resized_down, Size(down_width, down_height), INTER_LINEAR);
		imshow("gauss image", resized_down);
	}
}
void testCanny()
{
	char fname[MAX_PATH];
	while (openFileDlg(fname))
	{
		cv::namedWindow("canny", cv::WINDOW_AUTOSIZE);
		cv::namedWindow("input image", cv::WINDOW_AUTOSIZE);
		cv::namedWindow("gauss image", cv::WINDOW_AUTOSIZE);


		Mat src, dst, gauss, small_img;
		src = imread(fname, IMREAD_COLOR);

		int down_width = src.cols - 0.9 * src.cols;
		int down_height = src.rows - 0.9 * src.rows;
		Mat resized_down;




		//resize down
		resize(src, resized_down, Size(down_width, down_height), INTER_LINEAR);


		double k = 0.4;
		int pH = 50;
		int pL = (int)k * pH;
		GaussianBlur(resized_down,  // input image
			gauss,					// output image
			Size(5, 5),				// smoothing window width and height in pixels
			1.4,					// igma value, determines how much the image will be blurred
			1.4);
		// show image after gauss algorithm
		imshow("gauss image", gauss);



		Canny(gauss, dst, 100, 3 * pL, 3);



		imshow("input image", resized_down);
		moveWindow("input image", 300, 200);

		imshow("canny", dst);
		moveWindow("canny", 300, 140);
		//imwrite("cannyImg2.jpg", dst);
		waitKey();
	}
}
int ErrorMsg(const std::string& msg) {
	std::cerr << "\n !!! Error !!!\n " << msg << "\n";
	return -1;
}



const std::string window = "Canny Edge Detection"; //Window Name
int lowThreshold = 0;
const int maxThreshold = 100;
const int ratio = 3;
const int kernel_size = 3;

cv::Mat input, output; //input and output images;
cv::Mat detected_edges; //Edge Detected image;



static void CannyEdgeDetection(int, void*)
{
	int down_width = input.cols - 0.9 * input.cols;
	int down_height = input.rows - 0.9 * input.rows;
	Mat resized_down;
	//resize down
	resize(input, resized_down, Size(down_width, down_height), INTER_LINEAR);
	cv::imshow("input", resized_down);

	cv::GaussianBlur(resized_down, detected_edges, cv::Size(5, 5), 0); // Applying Gaussian Blur with kernel 5x5
	cv::Canny(detected_edges, detected_edges, lowThreshold, lowThreshold * ratio, kernel_size);

	output = cv::Scalar::all(0);

	resized_down.copyTo(output, detected_edges);

	cv::imshow(window, output);
}


void exemple() {
	char fname[MAX_PATH];
	while (openFileDlg(fname))
	{
		input = imread(fname, IMREAD_GRAYSCALE);


		output.create(input.size(), input.type());
		cv::namedWindow(window, cv::WINDOW_AUTOSIZE);

		cv::createTrackbar("Threshold: ", window, &lowThreshold, maxThreshold, CannyEdgeDetection);

		CannyEdgeDetection(0, 0);

		cv::waitKey(0);

	}


}

int main()
{
	int op;
	do
	{
		system("cls");
		destroyAllWindows();
		printf("Menu:\n");
		printf(" 1 - Basic image opening...\n");
		printf(" 2 - Open BMP images from folder\n");
		printf(" 3 - Color to Gray\n");
		printf(" 4 - Canny\n");
		printf(" 5 - Resize image\n");
		printf(" 6 - eXEMPLE\n");
		printf(" 0 - Exit\n\n");
		printf("Option: ");
		scanf("%d", &op);
		switch (op)
		{
		case 1:
			testOpenImage();
			break;
		case 2:
			testOpenImagesFld();
			break;
		case 3:
			testColor2Gray();
			break;
		case 4:
			testCanny();
			break;
		case 5:
			resizeImage();
			break;
		case 6:
			exemple();
			break;
		}



	} while (op != 0);
	return 0;
}

