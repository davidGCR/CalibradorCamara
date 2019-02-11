//
//  main.cpp
//  
//
//  Created by David Choqueluque Roman on 2/10/19.
//

#include <stdio.h>
//
// file : HW5.cpp
// ---------------------------
//
#include <stdlib.h>
#include <math.h>
#include "cv.h"
#include "cxcore.h"
#include "opencv2/highgui.h"
#include "utility.h"
#include "corner.h"
#include "homography.h"
#include "calibration.h"
void MarkCornerPoints(IplImage *image, Points *cornerMap);
int main(int argc, char **argv) {
    // declaration
    IplImage *inImage = 0;
    IplImage *colorImage = 0;
    PairPoints corspMap;
    Points cornerMap;
    CameraParam cp;
    HomographySet h;
    PairPointSet imagePt;
    char inImageName[80];
    char imageNamePrefix[80];
    CvMat *H = cvCreateMat(3, 3, CV_32FC1); // homography
    CvMat *K = cvCreateMat(3, 3, CV_32FC1); // intrinsic matrix
    int i, j;
    int numOfImages, imageNumber;
    int numPointsRow = NUM_H_LINES;
    int numPointsColumn = NUM_V_LINES;
    12
    int pointOrder;
    if(argc == 3){
        strcpy(imageNamePrefix, argv[1]);
        numOfImages = atoi(argv[2]);
    }else{
        printf("\n");
        printf(" Usage: hw5 imageNamePrefix [number of images]\n");
        printf("\n");
    }
    // set struct parameters
    h.len = numOfImages;
    h.numPointsRow = numPointsRow;
    h.numPointsColumn = numPointsColumn;
    corspMap.len = numPointsRow * numPointsColumn;
    cornerMap.len = numPointsRow * numPointsColumn;
    imagePt.imageLen = numOfImages;
    imagePt.pointLen = numPointsRow * numPointsColumn;
    cp.len = numOfImages;
    for(imageNumber = 0; imageNumber < numOfImages; imageNumber++){
        sprintf(inImageName, "../image/%s%d.pgm", imageNamePrefix, imageNumber);
        inImage = cvLoadImage(inImageName, -1);
        if(!inImage){
            printf("Could not load image file: %s\n", inImageName);
            exit(0);
        }
        ////////////////////////////////////////////////////////////
        // 1. Corner Detection //
        // find indexed corner using Hough xform //
        ////////////////////////////////////////////////////////////
        IndexedCornerDetect(inImage, &cornerMap);
        // display and store the result image
        colorImage = cvCreateImage(cvGetSize(inImage), IPL_DEPTH_8U, 3);
        cvCvtColor(inImage, colorImage, CV_GRAY2BGR);
        MarkCornerPoints(colorImage, &cornerMap);
        cvNamedWindow("output image", CV_WINDOW_AUTOSIZE);
        cvShowImage("output image", colorImage);
        cvWaitKey(0);
        cvDestroyWindow("output image");
        WriteImage(colorImage, "indexedcorner.jpg");
        ////////////////////////////////////////////////////////////
        // 2. Homography Estimation //
        // calculate the homoraphy using DLT(SVD) //
        13
        ////////////////////////////////////////////////////////////
        // create the correspondence map for calculating H
        for(i = 0; i < numPointsRow; i++){
            for(j = 0; j < numPointsColumn; j++){
                pointOrder = i*numPointsColumn + j;
                // 1 square length = unit 1
                // so, if actual 1 square length = a
                // we need scale unit with a, after calibration.
                corspMap.pt1I[pointOrder] = i;
                corspMap.pt1J[pointOrder] = j;
                corspMap.pt2I[pointOrder] = cornerMap.ptI[pointOrder];
                corspMap.pt2J[pointOrder] = cornerMap.ptJ[pointOrder];
                // store input corner points for radial distortion calculation
                imagePt.ptmI[pointOrder][imageNumber] = i;
                imagePt.ptmJ[pointOrder][imageNumber] = j;
                imagePt.ptiI[pointOrder][imageNumber] = cornerMap.ptI[pointOrder];
                imagePt.ptiJ[pointOrder][imageNumber] = cornerMap.ptJ[pointOrder];
            }
        }
        // estimate correspondences
        HomograhyEstimation(&corspMap, H);
        // store the estimated homography
        float H33 = cvmGet(H, 2, 2);
        // normalize homography : however, this step is not required
        for(int n = 0; n < 3; n++){
            for(int m = 0; m < 3; m++){
                h.H[n][m][imageNumber] = cvmGet(H, n, m) / H33;
            }
        }
    }
    ////////////////////////////////////////////////////////////
    // 3. Camera Calibration //
    // calculate intrinsic and extrinsic parameters //
    ////////////////////////////////////////////////////////////
    CameraCalibration(&h, &cp, &imagePt);
    ////////////////////////////////////////////////////////////
    // 5. Results //
    ////////////////////////////////////////////////////////////
    IplImage *outImage = cvCreateImage(cvGetSize(inImage), IPL_DEPTH_8U, 1);
    for(imageNumber = 0; imageNumber < numOfImages; imageNumber++){
        sprintf(inImageName, "../image/%s%d.pgm", imageNamePrefix, imageNumber);
        inImage = cvLoadImage(inImageName, -1);
        14
        if(!inImage){
            printf("Could not load image file: %s\n", inImageName);
            exit(0);
        }
        //H = KA, where A = [r1 r2 t];
        // get intrinsic matrix K
        float kArr[9] = {cp.alphaX, cp.skew, cp.x0,
            0.f , cp.alphaY, cp.y0,
            0.f , 0.f , 1};
        Array2CvMat(kArr, K, 3, 3);
        // get undistorted images
        float distCoeffs[4] = {cp.k1, cp.k2, 0.f, 0.f};
        cvUnDistortOnce(inImage, outImage, kArr, distCoeffs, 1);
        CvMat *A = cvCreateMat(3, 3, CV_32FC1);
        for(i = 0; i < 3; i++){
            cvmSet(A, i, 0, cp.r1[i][imageNumber]);
            cvmSet(A, i, 1, cp.r2[i][imageNumber]);
            cvmSet(A, i, 2, cp.t[i][imageNumber]);
        }
        cvMatMul(K, A, H);
        float h[9];
        CvMat2Array(H, h, 3, 3);
        int height = inImage->height;
        int width = inImage->width;
        for(i = 0; i < numPointsRow; i++){
            for(j = 0; j < numPointsColumn; j++){
                pointOrder = i*numPointsColumn + j;
                float x1 = h[0] * j + h[1] * i + h[2];
                float x2 = h[3] * j + h[4] * i + h[5];
                float x3 = h[6] * j + h[7] * i + h[8];
                float vi = min(height - 1, max(0, x2 / x3));
                float ui = min(width - 1, max(0, x1 / x3));
                // radial distortion
                float tpU = (ui - cp.x0);
                float tpV = (vi - cp.y0);
                float r = pow(tpU / cp.alphaX , 2) + pow(tpV / cp.alphaY, 2);
                cornerMap.ptJ[pointOrder] = cvRound(ui + tpU *
                                                    (cp.k1 * r + cp.k2 * pow(r, 2)));
                cornerMap.ptI[pointOrder] = cvRound(vi + tpV *
                                                    (cp.k1 * r + cp.k2 * pow(r, 2)));
                15
                cvCircle(inImage, cvPoint(cornerMap.ptJ[pointOrder],
                                          cornerMap.ptI[pointOrder]), 2, cvScalar(0, 255, 0), 3);
            }
        }
        WriteImage(inImage, "result.jpg");
        WriteImage(outImage, "distortionCorrResult.jpg");
        cvNamedWindow("output image", CV_WINDOW_AUTOSIZE);
        cvShowImage("output image", inImage);
        cvWaitKey(0);
        cvDestroyWindow("output image");
        cvNamedWindow("output image", CV_WINDOW_AUTOSIZE);
        cvShowImage("output image", outImage);
        cvWaitKey(0);
        cvDestroyWindow("output image");
        cvReleaseMat(&A);
    }
    // release the images a1nd matrix
    cvReleaseImage(&inImage);
    cvReleaseImage(&colorImage);
    cvReleaseMat(&H);
    cvReleaseMat(&K);
    return 0;
}
//
// function : MarkCornerPoints
// usage : MarkCornerPoints(image, cornerMap);
// -------------------------------------------------
// This function draws marks in input image corresponding
// to the corner map.
//
void MarkCornerPoints(IplImage *image, Points *cornerMap) {
    int i, j, k;
    char label[10];
    CvFont font;
    double hScale = .5;
    16
    double vScale = .5;
    int lineWidth = 2;
    cvInitFont(&font, CV_FONT_HERSHEY_SIMPLEX|CV_FONT_ITALIC,
               hScale, vScale, 0, lineWidth);
    for(k = 0; k < cornerMap->len; k++){
        i = cornerMap->ptI[k];
        j = cornerMap->ptJ[k];
        cvCircle(image, cvPoint(j, i), 3, cvScalar(0, 255, 0), 3);
        sprintf(label, "%d", k);
        cvPutText (image, label, cvPoint(j, i), &font, cvScalar(0, 0, 255));
    }
}
