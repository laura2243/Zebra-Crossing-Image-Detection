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

bool intersection(Point2f o1, Point2f p1, Point2f o2, Point2f p2,
                  Point2f &r) {
    Point2f x = o2 - o1;
    Point2f d1 = p1 - o1;
    Point2f d2 = p2 - o2;

    float cross = d1.x * d2.y - d1.y * d2.x;
    if (abs(cross) < 1e-8)
        return false;

    double t1 = (x.x * d2.y - x.y * d2.x) / cross;
    r = o1 + d1 * t1;
    return true;
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
//    for (size_t i = 0; i < linesP.size(); i++) {
//        Vec4i l = linesP[i];
//        line(input, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0, 0, 255), 2);
//    }


    vector<Point2f> intersections;
    vector<Vec4i> linesAfterIntersection;
    for (size_t i = 0; i < linesP.size() - 1; i++) {
        Vec4i li = linesP[i];
        for (size_t j = i + 1; j < linesP.size(); j++) {
            Vec4i lj = linesP[j];


            Point2f inter;
            if (intersection(Point(li[0], li[1]), Point(li[2], li[3]), Point(lj[0], lj[1]), Point(lj[2], lj[3]),
                             inter)) {
                intersections.push_back(inter);

                if (inter.x > 1700 && inter.x < 2200) {

                    linesAfterIntersection.push_back(li);
                    linesAfterIntersection.push_back(lj);
                }

            }

        }
    }

    Point2f minx, maxx, miny, maxy = Point2f(linesAfterIntersection[0][0], linesAfterIntersection[0][1]);

    for (int i = 0; i < linesAfterIntersection.size(); i++) {
        if (linesAfterIntersection[i][0] < minx.x) {
            minx = Point2f(linesAfterIntersection[i][0], linesAfterIntersection[i][1]);
        }
        if (linesAfterIntersection[i][1] < miny.y) {
            miny = Point2f(linesAfterIntersection[i][0], linesAfterIntersection[i][1]);
        }
        if (linesAfterIntersection[i][1] > maxy.y) {
            maxy = Point2f(linesAfterIntersection[i][0], linesAfterIntersection[i][1]);
        }
        if (linesAfterIntersection[i][0] > maxx.x) {
            maxx = Point2f(linesAfterIntersection[i][0], linesAfterIntersection[i][1]);
        }

        if (linesAfterIntersection[i][2] < minx.x) {
            minx = Point2f(linesAfterIntersection[i][2], linesAfterIntersection[i][3]);
        }
        if (linesAfterIntersection[i][3] < miny.y) {
            miny = Point2f(linesAfterIntersection[i][2], linesAfterIntersection[i][3]);
        }
        if (linesAfterIntersection[i][1] > maxy.y) {
            maxy = Point2f(linesAfterIntersection[i][2], linesAfterIntersection[i][3]);
        }
        if (linesAfterIntersection[i][2] > maxx.x) {
            maxx = Point2f(linesAfterIntersection[i][2], linesAfterIntersection[i][3]);
        }
    }


    vector<Point> contur;
    contur.push_back(minx);
    contur.push_back(miny);
    contur.push_back(maxx);
    contur.push_back(maxy);


//    Mat imgRectangle=zeros;

    line(input, minx, miny, Scalar(255, 0, 0), 2);
    line(input, maxx, miny, Scalar(255, 0, 0), 2);
    line(input, minx, maxy, Scalar(255, 0, 0), 2);
    line(input, maxx, maxy, Scalar(255, 0, 0), 2);

//    imshow("imgFinal", imgRectangle);

    for (size_t i = 0; i < linesAfterIntersection.size(); i++) {
        Vec4i l = linesAfterIntersection[i];
        line(input, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0, 0, 255), 2);
    }

    for (size_t i = 0; i < intersections.size(); i++) {
        Point p = intersections[i];
        std::cout << "Point: ";
        std::cout << p.x << " " << p.y << std::endl;
    }


    imshow("Input", input);
    imshow("Output", canny);
    moveWindow("Output", 300, 200);
    waitKey(0);

    return 0;
}