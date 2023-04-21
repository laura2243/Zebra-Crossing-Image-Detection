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
    vector<pair<Point, Point>> lines;
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


        if (inliers.size() > 100 && shouldDraw) {
            lines.push_back(make_pair(p1, p2));
            for (size_t pointIdx = inliers.size() - 1; pointIdx > 0; --pointIdx) {
                points_.erase(points_.begin() + inliers[pointIdx]);
            }
            inliers.clear();
            inliers.resize(0);


//            cv::line(image_,
//                     p1,
//                     p2,
//                     cv::Scalar(255, 0, 0),
//                     3);
        }
    }

    for (int i = 0; i < lines.size(); i++) {
        float slope =  (lines[i].first.y - lines[i].second.y) / (1.0 * (lines[i].first.x - lines[i].second.x));
        Point2d v = lines[i].second - lines[i].first; // Direction of the line
        v = v / cv::norm(v); // Division by the length of the vector to make it unit length
        Point2d n; // Normal of the line (perpendicular to the line)
        // Rotate v by 90° to get n
        n.x = -v.y;
        n.y = v.x;
        // distance(line, point) = | a * x + b * y + c | / sqrt(a * a + b * b)
        // if ||n||_2 = 1 (unit length) then sqrt(a * a + b * b) = 1 and I don't have to do the division that is in the previous line
        long double a = n.x;
        long double b = n.y;
        long double c = -(a * lines[i].first.x + b * lines[i].first.y);

        int xmax = max(lines[i].first.x, lines[i].second.x);
        int xmin = min(lines[i].first.x, lines[i].second.x);
        int ymin = min(lines[i].first.y, lines[i].second.y);
        int ymax = max(lines[i].first.y, lines[i].second.y);

        Point minPoint = lines[i].first;
        Point maxPoint = lines[i].second;

        int dx[] = {0, 0, -1, 1, 0, 0, 0, -2, 2, 0, 0, -3, 3};
        int dy[] = {1, -1, 0, 0, 0, 2, -2, 0, 0, 3, -3, 0, 0};


        if (lines[i].first.x - lines[i].second.x == 0) {
            while (true) {

                ymax++;
                Point newPoint = Point(lines[i].first.x, ymax);
                int ok = 0;
                for (int k = 0; k < 9; k++) {
                    int y = newPoint.x + dx[k];
                    int x = newPoint.y + dy[k];
                    if (!inside(image_, x,y)) {
                        break;
                    } else {
                        if (canny.at<uchar>(x, y) > 128) {
                            ok = 1;
                            break;
                        }
                    }
                }

                if (ok == 0)
                    break;


                maxPoint = newPoint;
            }

            while (true) {
                ymin--;
                int ok = 0;
                Point newPoint = Point(lines[i].first.x, ymin);
                for (int k = 0; k < 9; k++) {
                    int y = newPoint.x + dx[k];
                    int x = newPoint.y + dy[k];
                    if (!inside(image_, x,y)) {
                        break;
                    } else {
                        if (canny.at<uchar>(x, y) > 128) {
                            ok = 1;
                            break;
                        }
                    }
                }

                if (ok == 0)
                    break;
                minPoint = newPoint;
            }

        } else {
            while (true) {
                xmax++;
                Point newPoint = Point(xmax, slope * xmax + c);
                int ok = 0;
                for (int k = 0; k < 9; k++) {
                    int y = newPoint.x + dx[k];
                    int x = newPoint.y + dy[k];
                    if (!inside(image_, x,y)) {
                        break;
                    } else {
                        if (canny.at<uchar>(x, y) > 128) {
                            ok = 1;
                            break;
                        }
                    }
                }

                if (ok == 0)
                    break;
                maxPoint = newPoint;
            }

            while (true) {
                xmin--;
                Point newPoint = Point(xmin, slope * xmin + c);
                int ok = 0;
                for (int k = 0; k < 9; k++) {
                    int y = newPoint.x + dx[k];
                    int x = newPoint.y + dy[k];
                    if (!inside(image_, x,y)) {
                        break;
                    } else {
                        if (canny.at<uchar>(x, y) > 128) {
                            ok = 1;
                            break;
                        }
                    }
                }

                if (ok == 0)
                    break;
                minPoint = newPoint;
            }
        }
        cv::line(image_,
                 minPoint,
                 maxPoint,
                 cv::Scalar(0, 0, 255),
                 2);
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