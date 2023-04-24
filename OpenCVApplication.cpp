// OpenCVApplication.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include "common.h"

#include <iostream>
#include <opencv2/opencv.hpp>
#include <math.h>
#include <time.h>
#include <vector>

using namespace std;
using namespace cv;

bool inside(Mat img, int x, int y) {
    if (x >= 0 && x < img.rows && y >= 0 && y < img.cols)
        return true;
    return false;
}


Mat Canny(Mat src) {
    Mat dst;
    Mat gauss, small_img;
//        int down_width = src.cols - 0.9 * src.cols;
//        int down_height = src.rows - 0.9 * src.rows;
//        Mat resized_down;
    //resize down
    //resize(src, resized_down, Size(down_width, down_height), INTER_LINEAR);

    double k = 0.4;
    int pH = 50;
    int pL = (int) k * pH;
    GaussianBlur(src,
                 gauss,
                 Size(5, 5),
                 1.4,
                 1.4);

    Canny(gauss, dst, 100, 53, 3);

    return dst;

}


int main() {
    Mat input;
    input = imread("ransac/canny6.jpg", 1);

    if (input.data == nullptr) {
        cerr << "Failed to load image" << endl;
    }
    Mat gray;

    cvtColor(input, gray, COLOR_BGR2GRAY);

    Mat canny = Canny(gray);

    vector<Vec4i> linesP; // will hold the results of the detection
    HoughLinesP(canny, linesP, 0.5, CV_PI / 180, 40, 20, 20); // runs the actual detection
    // Draw the lines
    for (size_t i = 0; i < linesP.size(); i++)
    {
        Vec4i l = linesP[i];
        line(input, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0, 0, 255), 2);
    }

    imshow("Input", input);
    imshow("Output", canny);
    moveWindow("Output", 300, 200);
    waitKey(0);

    return 0;
}