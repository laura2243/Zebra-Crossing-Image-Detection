// OpenCVApplication.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include "common.h"

#include <iostream>
#include <opencv2/opencv.hpp>
#include <math.h>
#include <time.h>
#include <vector>

#include "opencv2/features2d.hpp"

using namespace cv;
using namespace std;


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

    Mat output= Mat::zeros(input.rows,input.cols,CV_8UC1);
    Mat outputIdeal= Mat::zeros(idealPhoto.rows,idealPhoto.cols,CV_8UC1);

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
        line(output, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(255, 255, 255), 2);
    }

    for (size_t i = 0; i < linesIdeal.size(); i++) {
        Vec4i l = linesIdeal[i];
        line(outputIdeal, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(255, 255, 255), 2);
    }


//    Mat img_src = imread("ransac/canny6.jpg", 1);
//    Mat img_tgt = imread("ransac/cannyR.jpg", 1);

    // Create a SIFT object
    Ptr<SIFT> sift = SIFT::create();

    // Detect keypoints and extract feature descriptors
    vector<KeyPoint> keypoints_src, keypoints_tgt;
    Mat descriptors_src, descriptors_tgt;
    sift->detectAndCompute(gray, noArray(), keypoints_src, descriptors_src);
    sift->detectAndCompute(grayIdeal, noArray(), keypoints_tgt, descriptors_tgt);

    // Create a brute force matcher
    BFMatcher bf(NORM_L2, true);

    // Match the keypoints between the two images

    Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create(DescriptorMatcher::FLANNBASED);
    std::vector< std::vector<DMatch> > matches;
    matcher->knnMatch( descriptors_src, descriptors_tgt, matches, 2 );

//    vector<vector<DMatch>> matches;
//    bf.knnMatch(descriptors_src, descriptors_tgt, matches, 2);

    // Apply ratio test to filter out poor matches
    vector<DMatch> good_matches;
    for (size_t i = 0; i < matches.size(); i++) {
        if (matches[i][0].distance < 1 * matches[i][1].distance) {
            good_matches.push_back(matches[i][0]);
        }
    }

    // Draw the matched keypoints
    Mat img_matches;
    drawMatches(gray, keypoints_src, grayIdeal, keypoints_tgt, good_matches, img_matches, Scalar::all(-1), Scalar::all(-1), vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);

    // Set RANSAC parameters
    double ransac_reproj_threshold = 4.0;
    int ransac_max_iters = 2000;

    // Convert the keypoints to numpy arrays
    vector<Point2f> src_pts, tgt_pts;
    for (size_t i = 0; i < good_matches.size(); i++) {
        src_pts.push_back(keypoints_src[good_matches[i].queryIdx].pt);
        tgt_pts.push_back(keypoints_tgt[good_matches[i].trainIdx].pt);
    }

    // Estimate the homography using RANSAC
    Mat homography;
    vector<char> inliers;
    homography = findHomography(src_pts, tgt_pts, RANSAC, ransac_reproj_threshold, inliers, ransac_max_iters);

    // Draw the inlier matches
    Mat img_inliers;
    drawMatches(gray, keypoints_src, grayIdeal, keypoints_tgt, good_matches, img_inliers, Scalar::all(-1), Scalar::all(-1), inliers, DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);

    // Warp the source image using the homography
    Mat img_warped;
    warpPerspective(gray, img_warped, homography, grayIdeal.size());

    // Create a mask for the projected image
    Mat mask = Mat::zeros(grayIdeal.size(), CV_8UC1);

    // Draw the projected image onto the mask
    vector<Point> pts;
    for (size_t i = 0; i < keypoints_tgt.size(); i++) {
        pts.push_back(keypoints_tgt[i].pt);
    }
    fillConvexPoly(mask, pts, Scalar(255));

    // Apply the mask to the projected image
    Mat img_result;
    img_warped.copyTo(img_result, mask);

    // Display the results
    imshow("Matches", img_matches);
    imshow("Inliers", img_inliers);
    imshow("Projected Image", img_result);


//    for (size_t i = 0; i < linesIdeal.size(); i++) {
//        Vec4i l = linesIdeal[i];
//        std::cout << "Point1: ";
//        std::cout << l[0] << " " << l[1] << std::endl;
//        std::cout << "Point2: ";
//        std::cout << l[2] << " " << l[3] << std::endl;
//    }
//
//    std::cout << "Imagine urata" << endl << endl;
//    for (size_t i = 0; i < linesP.size(); i++) {
//        Vec4i l = linesP[i];
//        std::cout << "Point1: ";
//        std::cout << l[0] << " " << l[1] << std::endl;
//        std::cout << "Point2: ";
//        std::cout << l[2] << " " << l[3] << std::endl;
//    }


//    Point2f srcTri[4];
//    srcTri[0] = Point2f(0.f, 0.f);
//    srcTri[1] = Point2f(input.cols - 1.f, 0.f);
//    srcTri[2] = Point2f(0.f, input.rows - 1.f);
//
//    Point2f dstTri[3];
//    dstTri[0] = Point2f(0.f, input.rows * 0.33f);
//    dstTri[1] = Point2f(input.cols * 0.85f, input.rows * 0.25f);
//    dstTri[2] = Point2f(input.cols * 0.15f, input.rows * 0.7f);

//    srcTri[0] = Point2f(29,35);
//    srcTri[1] = Point2f(29,229);
//    srcTri[2] = Point2f(65,32);
//    srcTri[3] = Point2f(243,218);
//
//    Point2f dstTri[4];
//    dstTri[0] = Point2f(46,230);
//    dstTri[1] = Point2f(228,220);
//    dstTri[2] = Point2f(46,167);
//    dstTri[3] = Point2f(input.cols,input.rows);

//    Mat warp_dst = Mat::zeros(input.rows, input.cols, input.type());
//    Mat warp_mat = getPerspectiveTransform(srcTri, dstTri);
//    warpPerspective(input, warp_dst, warp_mat, warp_dst.size());

//    Mat warp_dst2 = Mat::zeros(idealPhoto.rows, idealPhoto.cols, input.type());
//    Mat warp_mat2 = getAffineTransform(srcTri, dstTri);
//    warpAffine(idealPhoto, warp_dst2, warp_mat2, warp_dst2.size());

//    imshow("Warp", warp_dst);
//    imshow("Warp affine", warp_dst2);

    imshow("Input", input);
    imshow("out", output);
    imshow("outIdeal", outputIdeal);
    imshow("Output", canny);
    moveWindow("Output", 300, 200);
    waitKey(0);

    return 0;
}