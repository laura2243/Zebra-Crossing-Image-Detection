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


bool isInCircle(Point p1, Point p2, double r) {

    double d = sqrt(pow((p1.x - p2.x), 2) + pow((p1.y - p2.y), 2));
    return d < r;

}

vector<pair<Point2f, int>> calculScore(vector<Point2f> points, int r) {
    int cnt;
    int cntMax = 0;
    vector<pair<Point2f, int>> maxP;
    vector<pair<Point2f, int>> maxPinside;
    for (int i = 0; i < points.size(); i++) {
        maxPinside.clear();
        cnt = 0;
        for (int j = 0; j < points.size(); j++) {
            if (isInCircle(points[i], points[j], r)) {
                cnt++;
                pair<Point2f, int> p;
                p.first = points[j];
                p.second = j;
                maxPinside.push_back(p);
            }
        }

        if (cnt > cntMax) {
            for (int k = 0; k < maxPinside.size(); k++) {
                maxP.push_back(maxPinside[k]);
            }

            cntMax = cnt;
        }

    }
    return maxP;
}


double angleBetweenLines(Point2f line1_start, Point2f line1_end, Point2f line2_start, Point2f line2_end) {
   // slopes
    double slope1 = (line1_end.y - line1_start.y) / (line1_end.x - line1_start.x);
    double slope2 = (line2_end.y - line2_start.y) / (line2_end.x - line2_start.x);

    // Calculate the dot product of the two lines
    double dotProduct = slope1 * slope2 + 1e-10;

    // Calculate the angle between the two lines
    double angle = acos(dotProduct) * 180 / CV_PI;

    return angle;
}


int main() {
    Mat input;
    Mat idealPhoto;
    input = imread("ransac/canny9.jpg", 1);
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

    Mat m = Mat::zeros(input.rows, input.cols, input.type());
    //Draw the lines
    for (size_t i = 0; i < linesP.size(); i++) {
        Vec4i l = linesP[i];
        line(m, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(255, 255, 255), 1);
    }
    imshow("Before vanishing point", m);

    vector<Point2f> intersections;
    vector<Vec4i> linesAfterIntersection;
    vector<Vec4i> linesIntersection;
    vector<Vec4i> paralel;
    for (size_t i = 0; i < linesP.size() - 1; i++) {
        Vec4i li = linesP[i];
        for (size_t j = i + 1; j < linesP.size(); j++) {
            Vec4i lj = linesP[j];

            Point2f inter;

            if (intersection(Point(li[0], li[1]), Point(li[2], li[3]), Point(lj[0], lj[1]), Point(lj[2], lj[3]),
                             inter)) {
                intersections.push_back(inter);

                linesIntersection.push_back(li);
                linesIntersection.push_back(lj);

            } else {
                paralel.push_back(li);
                paralel.push_back(lj);
            }

        }
    }

    vector<pair<Point2f, int>> finalPoints;
    finalPoints = calculScore(intersections, 100);

    Mat f = Mat::zeros(input.rows, input.cols, CV_8UC1);
    for (int i = 0; i < finalPoints.size(); i++) {
        Vec4i l = linesIntersection[2 * finalPoints[i].second];
        line(f, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(255, 0, 0), 1);
        l = linesIntersection[2 * finalPoints[i].second + 1];
        line(f, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(255, 0, 0), 1);

    }
    for (int i = 0; i < paralel.size(); i++) {
        Vec4i l = paralel[i];
        line(f, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(255, 0, 0), 1);
    }

    imshow("lines", f);

    int minx = linesIntersection[0][0];
    int maxx = linesIntersection[0][0];
    int miny = linesIntersection[0][1];
    int maxy = linesIntersection[0][1];

    for (int i = 0; i < linesIntersection.size(); i++) {
        for (int j = 0; j < 4; j++) {
            if (j % 2 == 0) { //x
                if (linesIntersection[i][j] < minx) {
                    minx = linesIntersection[i][j];
                }
                if (linesIntersection[i][j] > maxx) {
                    maxx = linesIntersection[i][j];
                }
            } else {
                if (linesIntersection[i][j] < miny) {
                    miny = linesIntersection[i][j];
                }
                if (linesIntersection[i][j] > maxy) {
                    maxy = linesIntersection[i][j];
                }
            }
        }
    }



    for (size_t i = 0; i < linesIntersection.size(); i++) {
        Vec4i l = linesIntersection[i];
        line(input, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0, 0, 255), 2);
    }

    // rectangle
    line(input, Point(minx, miny), Point(maxx, miny), Scalar(255, 0, 0), 2);
    line(input, Point(minx, maxy), Point(maxx, maxy), Scalar(255, 0, 0), 2);
    line(input, Point(minx, miny), Point(minx, maxy), Scalar(255, 0, 0), 2);
    line(input, Point(maxx, miny), Point(maxx, maxy), Scalar(255, 0, 0), 2);



    imshow("Input", input);
    imshow("Output", canny);
    moveWindow("Output", 300, 200);
    waitKey(0);

    return 0;
}