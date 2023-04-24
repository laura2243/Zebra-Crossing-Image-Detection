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


float calculateSDX(vector<Point> points, vector<int> inliers) {
    float sum = 0.0, mean, SD = 0.0;
    int i;
    for (i = 0; i < inliers.size(); ++i) {
        sum += points[inliers[i]].x;
    }
    mean = sum / inliers.size();
    for (i = 0; i < inliers.size(); ++i) {
        SD += pow(points[inliers[i]].x - mean, 2);
    }
    return sqrt(SD / inliers.size());
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
            float standardDeviation = calculateSDX(points_, inliers);
//            std::cout<<standardDeviation<<std::endl;
            for (size_t pointIdx = inliers.size() - 1; pointIdx > 0; --pointIdx) {

                if (std::round(standardDeviation) == 0) {

                    sumX = sumX + points_[inliers[pointIdx]].y;
                    sumX2 = sumX2 + points_[inliers[pointIdx]].y * points_[inliers[pointIdx]].y;
                    sumY = sumY + points_[inliers[pointIdx]].x;
                    sumXY = sumXY + points_[inliers[pointIdx]].x * points_[inliers[pointIdx]].y;

                } else {
                    sumX = sumX + points_[inliers[pointIdx]].x;
                    sumX2 = sumX2 + points_[inliers[pointIdx]].x * points_[inliers[pointIdx]].x;
                    sumY = sumY + points_[inliers[pointIdx]].y;
                    sumXY = sumXY + points_[inliers[pointIdx]].x * points_[inliers[pointIdx]].y;
                }

                points_.erase(points_.begin() + inliers[pointIdx]);
            }
            inliers.clear();
            inliers.resize(0);


            b1 = (len * sumXY - sumX * sumY) / ((len * sumX2 - sumX * sumX) * 1.0);
            a1 = (sumY - b1 * sumX) / (len * 1.0);

            Point pmax;
            Point pmin;
            if (std::round(standardDeviation) == 0) {
                //x = a+by
                pmax = Point(a1, 0);
                pmin = Point(a1 + b1 * image_.rows, image_.rows);

            } else {
                //y = a+bx
                pmax = Point(0, a1);
                pmin = Point(image_.rows, a1 + b1 * image_.rows);
            }

            cv::line(image_,
                     pmin,
                     pmax,
                     cv::Scalar(255, 0, 0),
                     2);
        }
    }


}

vector<Point> points;
Mat input;
int r = 3;


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
    GaussianBlur(src,  // input image
                 gauss,                    // output image
                 Size(5, 5),                // smoothing window width and height in pixels
                 1.4,                    // igma value, determines how much the image will be blurred
                 1.4);

    Canny(gauss, dst, 100, 53, 3);

    return dst;

}


int main() {
    input = imread("ransac/canny6.jpg", 1);

    if (input.data == nullptr) {
        cerr << "Failed to load image" << endl;
    }
    Mat gray;

    Mat blur_gray;
    cvtColor(input, gray, COLOR_BGR2GRAY);

    Mat canny = Canny(gray);


    findNonZero(canny, points);

    FitLineRANSAC(canny, points, 1, 100000, input, 0);

    imshow("Input", input);
    imshow("Output", canny);
    moveWindow("Output", 300, 200);
    waitKey(0);

    return 0;
}