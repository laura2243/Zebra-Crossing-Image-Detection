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


double getLength(Point a, Point b) {
    double sum = sqrt(pow((b.x - a.x), 2) + pow((a.y - b.y), 2));
    return sum;
}


bool inside(Mat img, int x, int y) {
    if (x >= 0 && x < img.rows && y >= 0 && y < img.cols)
        return true;
    return false;
}


void FitLineRANSAC(
        Mat canny,
        vector<Point> points_,
        double threshold_,
        int maximum_iteration_number_,
        Mat image_, void *) {
    srand(time(NULL));
    int iterationNumber = 0;

    vector<int> inliers;
    inliers.reserve(points_.size());

    constexpr int kSampleSize = 2;
    std::vector<int> sample(kSampleSize);
    bool shouldDraw = image_.data != nullptr;
    while (iterationNumber++ < maximum_iteration_number_) {
        int found = 0;
        // 1. Select a minimal sample, i.e., in this case, 2 random points.

        for (size_t sampleIdx = 0; sampleIdx < kSampleSize; ++sampleIdx) {
            do {
                sample[sampleIdx] =
                        round((points_.size() - 1) * static_cast<double>(rand()) / static_cast<double>(RAND_MAX));
                if (sampleIdx == 0)
                    break;
                if (sampleIdx == 1 &&
                    sample[0] != sample[1] &&
                    getLength(points_[sample[0]], points_[sample[1]]) > 150)
                    break;
            } while (true);
        }


        // 2. Fit a line to the points.
        const Point2d &p1 = points_[sample[0]]; // First point selected
        const Point2d &p2 = points_[sample[1]]; // Second point select
        Point2d v = p2 - p1; // Direction of the line
        v = v / cv::norm(v); // Division by the length of the vector to make it unit length
        Point2d n; // Normal of the line (perpendicular to the line)
        // Rotate v by 90° to get n
        n.x = -v.y;
        n.y = v.x;
        // distance(line, point) = | a * x + b * y + c | / sqrt(a * a + b * b)
        // if ||n||_2 = 1 (unit length) then sqrt(a * a + b * b) = 1 and I don't have to do the division that is in the previous line
        long double a = n.x;
        long double b = n.y;
        long double c = -(a * p1.x + b * p1.y);

        // 3. Count the number of inliers, i.e., the points closer than the threshold.
        inliers.clear();
        for (size_t pointIdx = 0; pointIdx < points_.size(); ++pointIdx) {
            const Point2d &point = points_[pointIdx];
            const long double distance =
                    abs(a * point.x + b * point.y + c);

            if (distance < threshold_) {
                inliers.emplace_back(pointIdx);
            }
        }


        float sumX = 0, sumX2 = 0, sumY = 0, sumXY = 0, a1, b1;
        int len = inliers.size();

        if (inliers.size() > 100 && shouldDraw) {
            for (size_t pointIdx = inliers.size() - 1; pointIdx > 0; --pointIdx) {

                sumX = sumX + points_[inliers[pointIdx]].x;
                sumX2 = sumX2 + points_[inliers[pointIdx]].x * points_[inliers[pointIdx]].x;
                sumY = sumY + points_[inliers[pointIdx]].y;
                sumXY = sumXY + points_[inliers[pointIdx]].x * points_[inliers[pointIdx]].y;

                points_.erase(points_.begin() + inliers[pointIdx]);
            }
            inliers.clear();
            inliers.resize(0);

            //y = a+bx
            b1 = (len * sumXY - sumX * sumY) / ((len * sumX2 - sumX * sumX) * 1.0);
            a1 = (sumY - b1 * sumX) / (len * 1.0);


            Point pmax = Point(0, -1.0 * a1 / b1);
            Point pmin = Point(a1 + b1 * image_.cols, image_.cols);



            std::cout << pmax.y << endl;
            //x = (y-a)/b
//            pmax = Point (-a1/b1,0);

            cv::line(image_,
                     pmin,
                     pmax,
                     cv::Scalar(255, 0, 0),
                     3);
        }
    }


}

vector<Point> points;
Mat input;
int lowThreshold = 0;
int maxThreshold = 100;
int r = 3;

//static void on_track(int, void *) {
//    FitLineRANSAC(points, lowThreshold * r, 100000, input, 0);
//}

void DrawPoints(vector<Point> &points, Mat image) {
    for (int i = 0; i < points.size(); ++i) {
        circle(image, points[i], 1, Scalar(0, 255, 0));
    }
}

void f(Mat img, vector<Point> &points) {
    for (int i = 0; i < img.rows; i++) {
        for (int j = 0; j < img.cols; j++) {
            uchar pix = img.at<uchar>(i, j);
            if (pix > 127) {
                points.push_back(Point(j, i));
            }
        }
    }
}


const std::string window = "test";

int main() {
    input = imread("ransac/cannyR.jpg", 1);

    if (input.data == nullptr) {
        cerr << "Failed to load image" << endl;
    }
    Mat gray;
    Mat edge_gaus;

    int kernel_size = 5;
    Mat blur_gray;
    cvtColor(input, gray, COLOR_BGR2GRAY);
    GaussianBlur(gray, blur_gray, Size(5, 5), 1.4, 1.4);
    Canny(blur_gray, edge_gaus, 100, 120, 3);
    //imshow("Gaus", edge_gaus);
//
//    Mat image = Mat::zeros(edge_gaus.rows, edge_gaus.cols, CV_8UC3);


    findNonZero(edge_gaus, points);
//    DrawPoints(points, input);

//    cv::namedWindow(window, cv::WINDOW_AUTOSIZE);

//    cv::createTrackbar("Threshold: ", window, &lowThreshold, maxThreshold, on_track);

//    on_track(0,0);
    FitLineRANSAC(edge_gaus, points, 1, 100000, input, 0);

//
    imshow("Edge detected image", edge_gaus);
    imshow("Output", input);
    moveWindow("Output", 300, 200);
    waitKey(0);

    return 0;
}