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
    Mat idealPhoto;
    input = imread("ransac/canny6.jpg", 1);
    idealPhoto = imread("ransac/cannyR.jpg", 1);

    if (input.data == nullptr) {
        cerr << "Failed to load image" << endl;
    }
    Mat gray;
    Mat grayIdeal;

    cvtColor(input, gray, COLOR_BGR2GRAY);
    cvtColor(idealPhoto, grayIdeal, COLOR_BGR2GRAY);

    Mat canny = Canny(gray);
    Mat cannyIdeal = Canny(grayIdeal);

    vector<Vec4i> linesP; // will hold the results of the detection
    vector<Vec4i> linesIdeal;
    HoughLinesP(canny, linesP, 0.5, CV_PI / 180, 40, 50, 20); // runs the actual detection
    HoughLinesP(cannyIdeal, linesIdeal, 0.5, CV_PI / 180, 40, 50, 20);
    // Draw the lines
    for (size_t i = 0; i < linesP.size(); i++) {
        Vec4i l = linesP[i];
        line(input, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0, 0, 255), 2);
    }

    for (size_t i = 0; i < linesIdeal.size(); i++) {
        Vec4i l = linesIdeal[i];
        std::cout << "Point1: ";
        std::cout << l[0] << " " << l[1] << std::endl;
        std::cout << "Point2: ";
        std::cout << l[2] << " " << l[3] << std::endl;
    }

    std::cout << "Imagine urata" << endl << endl;
    for (size_t i = 0; i < linesP.size(); i++) {
        Vec4i l = linesP[i];
        std::cout << "Point1: ";
        std::cout << l[0] << " " << l[1] << std::endl;
        std::cout << "Point2: ";
        std::cout << l[2] << " " << l[3] << std::endl;
    }


    Point2f srcTri[4];
//    srcTri[0] = Point2f(0.f, 0.f);
//    srcTri[1] = Point2f(input.cols - 1.f, 0.f);
//    srcTri[2] = Point2f(0.f, input.rows - 1.f);
//
//    Point2f dstTri[3];
//    dstTri[0] = Point2f(0.f, input.rows * 0.33f);
//    dstTri[1] = Point2f(input.cols * 0.85f, input.rows * 0.25f);
//    dstTri[2] = Point2f(input.cols * 0.15f, input.rows * 0.7f);

    srcTri[0] = Point2f(29,35);
    srcTri[1] = Point2f(29,229);
    srcTri[2] = Point2f(65,32);
    srcTri[3] = Point2f(243,218);

    Point2f dstTri[4];
    dstTri[0] = Point2f(46,230);
    dstTri[1] = Point2f(228,220);
    dstTri[2] = Point2f(46,167);
    dstTri[3] = Point2f(input.cols,input.rows);

//    Mat warp_dst = Mat::zeros(input.rows, input.cols, input.type());
//    Mat warp_mat = getPerspectiveTransform(srcTri, dstTri);
//    warpPerspective(input, warp_dst, warp_mat, warp_dst.size());

    Mat warp_dst2 = Mat::zeros(idealPhoto.rows, idealPhoto.cols, input.type());
    Mat warp_mat2 = getAffineTransform(srcTri, dstTri);
    warpAffine(idealPhoto, warp_dst2, warp_mat2, warp_dst2.size());

//    imshow("Warp", warp_dst);
    imshow("Warp affine", warp_dst2);

    imshow("Input", input);
    imshow("Output", canny);
    moveWindow("Output", 300, 200);
    waitKey(0);

    return 0;
}