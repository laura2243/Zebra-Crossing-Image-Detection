// OpenCVApplication.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include "common.h"


void testOpenImage() {
    char fname[MAX_PATH];
    while (openFileDlg(fname)) {
        Mat src;
        src = imread(fname);
        imshow("opened image", src);
        waitKey();
    }
}

void testOpenImagesFld() {
    char folderName[MAX_PATH];
    if (openFolderDlg(folderName) == 0)
        return;
    char fname[MAX_PATH];
    FileGetter fg(folderName, "bmp");
    while (fg.getNextAbsFile(fname)) {
        Mat src;
        src = imread(fname);
        imshow(fg.getFoundFileName(), src);
        if (waitKey() == 27) //ESC pressed
            break;
    }
}

void testColor2Gray() {
    char fname[MAX_PATH];
    while (openFileDlg(fname)) {
        Mat_<Vec3b> src = imread(fname, IMREAD_COLOR);

        int height = src.rows;
        int width = src.cols;

        Mat_<uchar> dst(height, width);

        for (int i = 0; i < height; i++) {
            for (int j = 0; j < width; j++) {
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

void scaling(Mat src, double fx, double fy) {
    int height = src.rows;
    int width = src.cols;

    Size newSize = Size(round(fx * width), round(fy * height));
    Mat dst = Mat::zeros(src.rows, src.cols, src.type());
    resize(src, dst, newSize, fx, fy, INTER_CUBIC);

    imshow("original image", src);
    imshow("Scaled image", dst);
    waitKey();
}

void translation(Mat src, float tx, float ty) {
    int height = src.rows;
    int width = src.cols;
    Mat dst = Mat::zeros(src.rows, src.cols, src.type());

    // matricea de translatie - matrice de 2x3
    float vals[6] = {1, 0, tx, 0, 1, ty};
    Mat matrix(2, 3, CV_32FC1, vals);

    warpAffine(src, dst, matrix, Size(width, height));

    imshow("original image", src);
    imshow("Translated image", dst);
    waitKey();
}

void rotation(Mat src, Point2f center, double angle, double scale) {
    Mat dst = Mat::zeros(src.rows, src.cols, src.type());

    //OpenCV provides scaled rotation with adjustable center of rotation
    //so that we can rotate at any location you prefer.
    Mat matrix = getRotationMatrix2D(center, angle, scale);

    warpAffine(src, dst, matrix, Size(dst.cols, dst.rows));

    imshow("original image", src);
    imshow("Rotated image", dst);
    waitKey();
}

void affineTransform(Mat src, Point2f center, double angle, double scale) {
    Point2f srcTri[3];
    srcTri[0] = Point2f(0.f, 0.f);
    srcTri[1] = Point2f(src.cols - 1.f, 0.f);
    srcTri[2] = Point2f(0.f, src.rows - 1.f);

    Point2f dstTri[3];
    dstTri[0] = Point2f(0.f, src.rows * 0.33f);
    dstTri[1] = Point2f(src.cols * 0.85f, src.rows * 0.25f);
    dstTri[2] = Point2f(src.cols * 0.15f, src.rows * 0.7f);

    Mat warp_dst = Mat::zeros(src.rows, src.cols, src.type());
    Mat warp_mat = getAffineTransform(srcTri, dstTri);
    warpAffine(src, warp_dst, warp_mat, warp_dst.size());

    Mat warp_rotate_dst;
    Mat rot_mat = getRotationMatrix2D(center, angle, scale);
    warpAffine(warp_dst, warp_rotate_dst, rot_mat, warp_dst.size());

    imshow("Source image", src);
    imshow("Warp", warp_dst);
    imshow("Warp + Rotate", warp_rotate_dst);
    waitKey();
}


int main() {
    int op;
    do {
        system("cls");
        destroyAllWindows();
        printf("Menu:\n");
        printf(" 1 - Basic image opening...\n");
        printf(" 2 - Open BMP images from folder\n");
        printf(" 3 - Color to Gray\n");
        printf(" 4 - Scaling\n");
        printf(" 5 - Translation\n");
        printf(" 6 - Rotation\n");
        printf(" 7 - Affine Transformation\n");
        printf(" 0 - Exit\n\n");
        printf("Option: ");
        scanf("%d", &op);
        switch (op) {
            case 1:
                testOpenImage();
                break;
            case 2:
                testOpenImagesFld();
                break;
            case 3:
                testColor2Gray();
                break;
            case 4: {
                char fname[MAX_PATH];
                while (openFileDlg(fname)) {
                    Mat src;
                    src = imread(fname);

                    double fx = 2;
                    double fy = 2;
                    scaling(src, fx, fy);
                    break;
                }
            }
            case 5: {
                char fname[MAX_PATH];
                while (openFileDlg(fname)) {
                    Mat src;
                    src = imread(fname);

                    float tx = 100;
                    float ty = 50;
                    translation(src, tx, ty);
                    break;
                }
            }
            case 6: {
                char fname[MAX_PATH];
                while (openFileDlg(fname)) {
                    Mat src;
                    src = imread(fname);

                    Point2f center = Point2f((src.cols - 1) / 2.0, (src.rows - 1) / 2.0);
                    double angle = 90;
                    double scale = 2;
                    rotation(src, center, angle, scale);
                    break;
                }
            }
            case 7: {
                char fname[MAX_PATH];
                while (openFileDlg(fname)) {
                    Mat src;
                    src = imread(fname);

                    Point2f center = Point2f((src.cols - 1) / 2.0, (src.rows - 1) / 2.0);
                    double angle = -50;
                    double scale = 0.6;
                    affineTransform(src, center, angle, scale);
                    break;
                }
            }

        }
    } while (op != 0);
    return 0;
}
