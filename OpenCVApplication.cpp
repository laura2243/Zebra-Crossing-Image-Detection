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


float rmse(Mat src, Mat dst) {

    int square = 0;
    float mean = 0.0;

    for (int i = 0; i < src.rows; i++) {
        for (int j = 0; j < src.cols; j++) {
            square += pow((src.at<uchar>(i, j) - dst.at<uchar>(i, j)), 2);
        }
    }

    mean = square / (src.rows * src.cols);
    return sqrt(mean);
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
    int pL = (int)k * pH;
    GaussianBlur(src,
                 gauss,
                 Size(5, 5),
                 1.4,
                 1.4);

    Canny(gauss, dst, 100, 53, 3);

    return dst;

}

bool intersection(Point2f o1, Point2f p1, Point2f o2, Point2f p2,
                  Point2f& r) {
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

bool angle(vector<Point2f> points) {
    double slope = (points[1].x - points[0].x) / (points[1].y - points[0].y);

    double ang = atan(slope);

    return ang;
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


int main() {
    Mat input;
    Mat idealPhoto;
    input = imread("ransac/canny1.jpg", 1);
    idealPhoto = imread("ransac/cannyR.jpg", 1);
    Mat output = Mat::zeros(input.rows, input.cols, CV_8UC1);

    if (input.data == nullptr) {
        cerr << "Failed to load image" << endl;
    }
    Mat gray = Mat::zeros(input.rows, input.cols, CV_8UC1);
    Mat grayIdeal;

    cvtColor(input, gray, COLOR_BGR2GRAY);
    cvtColor(idealPhoto, grayIdeal, COLOR_BGR2GRAY);

    Mat canny = Canny(gray);
    Mat cannyIdeal = Canny(grayIdeal);

    vector<Vec4i> linesP; // will hold the results of the detection
    vector<Vec4i> linesIdeal;
    HoughLinesP(canny, linesP, 0.5, CV_PI / 180, 40, 50, 10); // runs the actual detection
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

            }
            else {
                paralel.push_back(li);
                paralel.push_back(lj);
            }

        }
    }

    for (size_t i = 0; i < linesP.size(); i++) {
        Vec4i l = linesP[i];
        line(output, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(255, 255, 255), 2);
    }

    vector<pair<Point2f, int>> finalPoints;
    finalPoints = calculScore(intersections, 100);

    vector<Vec4i> goodlines;

    Mat f = Mat::zeros(input.rows, input.cols, CV_8UC1);
    for (int i = 0; i < finalPoints.size(); i++) {
        Vec4i l = linesIntersection[2 * finalPoints[i].second];
        line(f, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(255, 0, 0), 1);
        if (std::find(goodlines.begin(), goodlines.end(), l) == goodlines.end())
            goodlines.push_back(l);
        l = linesIntersection[2 * finalPoints[i].second + 1];
        line(f, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(255, 0, 0), 1);
        if (std::find(goodlines.begin(), goodlines.end(), l) == goodlines.end())
            goodlines.push_back(l);

    }
    for (int i = 0; i < paralel.size(); i++) {
        Vec4i l = paralel[i];
        line(f, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(255, 0, 0), 1);
        if (std::find(goodlines.begin(), goodlines.end(), l) == goodlines.end())
            goodlines.push_back(l);
    }

    imshow("lines", f);

    vector<Point2f> ang;
    ang.push_back(Point2f(goodlines[0][0], goodlines[0][1]));
    ang.push_back(Point2f(goodlines[0][2], goodlines[0][3]));
    if (angle(ang) < 45) {
        std::sort(goodlines.begin(), goodlines.end(),
                  [](const cv::Vec4i& a, const cv::Vec4i& b) {
                      return a[1] < b[1];
                  });
    }
    else {
        std::sort(goodlines.begin(), goodlines.end(),
                  [](const cv::Vec4i& a, const cv::Vec4i& b) {
                      return a[0] < b[0];
                  });
    }

    Point2f srcTri[4];

    srcTri[0] = Point(goodlines[0][0]-100, goodlines[0][1]+50);
    srcTri[1] = Point(goodlines[0][2], goodlines[0][3]);
    srcTri[2] = Point(goodlines[goodlines.size()-1][0], goodlines[goodlines.size()-1][1]);
    srcTri[3] = Point(goodlines[goodlines.size()-1][2]+100, goodlines[goodlines.size()-1][3]-50);


    Point2f dstTri[4];
    int minn = min(linesP.size(), linesIdeal.size());
    dstTri[0] = Point(0, idealPhoto.rows);
    dstTri[1] = Point(0, 0);

    float minRMSE = FLT_MAX;
    Mat imgFinal;

    Mat warp_mat2;
    Mat warp_mat2BEST;
    Mat warp_dst2 = Mat::zeros(input.rows, input.cols, input.type());


    for (int i = 0; i < linesIdeal.size(); i++) {
        dstTri[2] = Point(linesIdeal[i][0], linesIdeal[i][1]);
        dstTri[3] = Point(linesIdeal[i][2], linesIdeal[i][3]);
        Mat tst = idealPhoto(Range(0, idealPhoto.rows), Range(0, linesIdeal[i][2]));
        warp_mat2 = getPerspectiveTransform(dstTri, srcTri);
        warpPerspective(tst, warp_dst2, warp_mat2, warp_dst2.size());
        Mat dst;

        float score = rmse(output, warp_dst2);
        cout << score << endl;

        if (score < minRMSE) {
            minRMSE = score;
            warp_dst2.copyTo(imgFinal);
            warp_mat2.copyTo(warp_mat2BEST);
        }

    }

    imshow("njrbif", warp_dst2);


    Mat newImage(idealPhoto.rows, idealPhoto.cols, idealPhoto.type(), Scalar(255, 255, 255));
    Mat mt = Mat::zeros(input.rows, input.cols, input.type());
    Mat final(input.rows, input.cols, input.type(), Scalar(255, 0, 0));


    warpPerspective(newImage, mt, warp_mat2, mt.size());

    for (int i = 0; i < final.rows; i++) {
        for (int j = 0; j < final.cols*3; j++) {
            if (mt.at<uchar>(i, j) == 0)
                final.at<uchar>(i, j) = input.at<uchar>(i, j);
            else
                final.at<uchar>(i, j) = warp_dst2.at<uchar>(i, j);

        }
    }

    imshow("mkdenide", mt);
    imshow("cece", final);



    //int minx = linesIntersection[0][0];
    //int maxx = linesIntersection[0][0];
    //int miny = linesIntersection[0][1];
    //int maxy = linesIntersection[0][1];

    //for (int i = 0; i < linesIntersection.size(); i++) {
    //    for (int j = 0; j < 4; j++) {
    //        if (j % 2 == 0) { //x
    //            if (linesIntersection[i][j] < minx) {
    //                minx = linesIntersection[i][j];
    //            }
    //            if (linesIntersection[i][j] > maxx) {
    //                maxx = linesIntersection[i][j];
    //            }
    //        }
    //        else {
    //            if (linesIntersection[i][j] < miny) {
    //                miny = linesIntersection[i][j];
    //            }
    //            if (linesIntersection[i][j] > maxy) {
    //                maxy = linesIntersection[i][j];
    //            }
    //        }
    //    }
    //}



    //for (size_t i = 0; i < linesIntersection.size(); i++) {
    //    Vec4i l = linesIntersection[i];
    //    line(input, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0, 0, 255), 2);
    //}

    //// rectangle
    //line(input, Point(minx, miny), Point(maxx, miny), Scalar(255, 0, 0), 2);
    //line(input, Point(minx, maxy), Point(maxx, maxy), Scalar(255, 0, 0), 2);
    //line(input, Point(minx, miny), Point(minx, maxy), Scalar(255, 0, 0), 2);
    //line(input, Point(maxx, miny), Point(maxx, maxy), Scalar(255, 0, 0), 2);



    imshow("Input", input);
    imshow("Output", canny);
    moveWindow("Output", 300, 200);
    waitKey(0);

    return 0;
}