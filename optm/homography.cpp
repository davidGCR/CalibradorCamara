//
//  homography.cpp
//  
//
//  Created by David Choqueluque Roman on 2/10/19.
//

#include "homography.hpp"

//
// file : homography.cpp
//------------------------------------------------
// this file contains functions for homography
// estimation
//
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include "cv.h"
#include "cxcore.h"
#include "highgui.h"
#include "utility.h"
#include "homography.h"
void HomograhyEstimation(PairPoints *corspMap, CvMat *H) {
    int i;
    int numOfCorresp = corspMap->len;;
    float domainPositions[numOfCorresp][2], rangePositions[numOfCorresp][2];
    for(i = 0; i < numOfCorresp; i++){
        // I -> y, J -> x
        domainPositions[i][1] = corspMap->pt1I[i];
        domainPositions[i][0] = corspMap->pt1J[i];
        rangePositions[i][1] = corspMap->pt2I[i];
        rangePositions[i][0] = corspMap->pt2J[i];
    }
    // compute the homography
    ComputeHomography(domainPositions, rangePositions, numOfCorresp, H);
}
void RansacHomograhyEstimation(PairPoints *corspMap, PairPoints *inlierMap,
                               CvMat *H) {
    int numOfCorresp = 4;
    float domainPositions[numOfCorresp][2], rangePositions[numOfCorresp][2];
    int i, sampleCount = 0;
    int pos;
    float p, e, outlierProb;
    int numOfInliers, totalNumOfPoints, maxNumOfInliers = 1;
    PairPoints tempInlierMap;
    CvMat *Htmp;
    Htmp = cvCreateMat(3, 3, CV_32FC1);
    p = 0.99;
    e = 0.5;
    float N = 1000;
    while(N > sampleCount){
        // pick 4 corresponding points
        for(i = 0; i < numOfCorresp; i++){
            pos = rand() % corspMap->len; // select random positions
            // I -> y, J -> x
            domainPositions[i][1] = corspMap->pt1I[pos];
            domainPositions[i][0] = corspMap->pt1J[pos];
            rangePositions[i][1] = corspMap->pt2I[pos];
            rangePositions[i][0] = corspMap->pt2J[pos];
        }
        // check whether samples aree good or not.4
        // if the seleted samples are good, then do homography estimation
        // else reselect samples.
        if(IsGoodSample(domainPositions, numOfCorresp) &&
           IsGoodSample(rangePositions, numOfCorresp)){
            // compute the homography
            ComputeHomography(domainPositions, rangePositions, numOfCorresp, Htmp);
            // calculate the distance for each correspondences
            // compute the number of inliers
            InitializePairPoints(&tempInlierMap);
            CalculateDistance(Htmp, corspMap, &tempInlierMap);
            // choose H with the largest number of inliears
            numOfInliers = tempInlierMap.len;
            if(numOfInliers > maxNumOfInliers){
                maxNumOfInliers = numOfInliers;
                CopyPairPoints(inlierMap, &tempInlierMap);
                cvCopy(Htmp, H, 0);
            }
            // adaptive algorithm for determining the number of RANSAC samples
            // textbook algorithm 4.6
            totalNumOfPoints = corspMap->len;
            outlierProb = 1 - ((float)maxNumOfInliers / (float)totalNumOfPoints);
            e = (e < outlierProb ? e : outlierProb);
            N = log(1 - p) / log(1 - pow((1 - e), numOfCorresp));
            sampleCount += 1;
        }
    }
}
//
// function : ComputeHomography
// usage : ComputeHomography(domainPositions, rangePositions,
// numOfCorresp, H);
// -----------------------------------------------------------
// This function calculate the homography, H, using the set of
// given pairs of correspodences.
// Before computing the homography, data normalization will be
// performed. Then, it solve Ah = 0 using SVD to get H.
//
void ComputeHomography(float domainPositions[][2], float rangePositions[][2],
                       int numOfCorresp, CvMat *H)
{
    int column = 9, row;
    float x1, y1, w1, x2, y2, w2;
    int i, ii, jj;
    float h[9];
    if(numOfCorresp == 4){
        row = 3; // eq 4.1 : make a thin matrix to solve SVD in opencv
    }else if(numOfCorresp > 4){
        row = 2; //eq 4.3
    }else{
        printf("Need more correpondence points! for computing H.\n");
        exit(0);
    }
    float a[row * column];
    CvMat *A, *Htmp;
    CvMat *T1, *T2;
    CvMat *D, *V, *U;
    CvMat *invT2, *temp;
    // normalization
    T1 = cvCreateMat(3, 3, CV_32FC1);
    T2 = cvCreateMat(3, 3, CV_32FC1);
    DataNormalization(numOfCorresp, domainPositions, T1);
    DataNormalization(numOfCorresp, rangePositions, T2);
    // set A
    A = cvCreateMat(numOfCorresp * row, column, CV_32FC1);
    for(i = 0; i < numOfCorresp; i++){
        x1 = domainPositions[i][0];
        y1 = domainPositions[i][1];
        w1 = 1;
        x2 = rangePositions[i][0];
        y2 = rangePositions[i][1];
        w2 = 1;
        // set Ai
        // [0, 0, 0, -w2*x1, -w2*y1, -w2*w1, y2*x1, y2*y1, y2*w1]
        // [w2*x1, w2*y1, w2*w1, 0, 0, 0, -x2*x1, -x2*y1, -x2*w1]
        a[0] = 0; a[1] = 0; a[2] = 0;
        a[3] = -w2*x1; a[4] = -w2*y1; a[5] = -w2*w1;
        a[6] = y2*x1; a[7] = y2*y1; a[8] = y2*w1;
        a[9] = w2*x1; a[10] = w2*y1; a[11] = w2*w1;
        a[12] = 0; a[13] = 0; a[14] = 0;
        a[15] = -x2*x1; a[16] = -x2*y1; a[17] = -x2*w1;
        if(row == 3){
            a[18] = -y2*x1; a[19] = -y2*y1; a[20] = -y2*w1;
            a[21] = x2*x1; a[22] = x2*y1; a[23] = x2*w1;
            a[24] = 0; a[25] = 0; a[26] = 0;
        }
        // assemble Ai into a matrix A
        for(ii = 0; ii < row; ii++){
            for(jj = 0; jj < column; jj++){
                cvmSet(A, ii + i*row, jj, a[ii*column + jj]);
            }
        }
    }
    // calculate H
    Htmp = cvCreateMat(3, 3, CV_32FC1);
    D = cvCreateMat(numOfCorresp*row, column, CV_32FC1);
    U = cvCreateMat(numOfCorresp*row, numOfCorresp*row, CV_32FC1);
    V = cvCreateMat(column, column, CV_32FC1);
    cvSVD(A, D, U, V, CV_SVD_U_T|CV_SVD_V_T); // : opencv setting
    // A = U^T D V in openCV : A = U’ D’ V’^T in text
    // take last column of V’ : last row of V
    for(i = 0; i < column; i++){
        h[i] = cvmGet(V, column-1, i);
    }
    Array2CvMat(h, Htmp, 3, 3);
    // denormalization : H = invT2 * Htmp * T1 <- Htmp = T2 * H * invT1
    invT2 = cvCreateMat(3, 3, CV_32FC1);
    temp = cvCreateMat(3, 3, CV_32FC1);
    cvInvert(T2, invT2);
    cvMatMul(invT2, Htmp, temp);
    cvMatMul(temp,T1, H);
    // release matrices
    cvReleaseMat(&T1); cvReleaseMat(&T2);
    cvReleaseMat(&A); cvReleaseMat(&Htmp);
    cvReleaseMat(&D); cvReleaseMat(&U); cvReleaseMat(&V);
    cvReleaseMat(&T1); cvReleaseMat(&temp);
}
//
// function : DataNormalization
// usage : DataNormalization(numOfx, x, T);
// ------------------------------------------------------
// Thsi function normalizes x and returns the similarity
// transform, T.
// The centroid of x will be transformed into (0,0).
// The average distance of normalized x will be sqrt(2).
//
void DataNormalization(int numOfPositions, float positions[][2], CvMat *T) {
    int i;
    float sumI = 0, sumJ = 0, meanI = 0, meanJ = 0;
    float squareDist = 0, sumDist = 0, meanDist = 0;
    float scale = 0;
    float x, y, xx, yy, ww;
    // calculate the centroid
    for(i = 0; i < numOfPositions; i++){
        sumI += positions[i][0];
        sumJ += positions[i][1];
    }
    meanI = sumI / numOfPositions;
    meanJ = sumJ / numOfPositions;
    // calculate the mean distance
    for(i = 0; i < numOfPositions; i++){
        squareDist = pow(positions[i][0] - meanI, 2)
        + pow(positions[i][1] - meanJ, 2);
        sumDist += pow(squareDist, 0.5);
    }
    meanDist = sumDist / numOfPositions;
    // set the similarity transform
    scale = pow(1, 0.5) / meanDist;
    float t[9] = {scale, 0, -scale * meanI,
        0, scale, -scale * meanJ,
        0, 0, 1};
    Array2CvMat(t, T, 3, 3);
    // data normalization
    for(i = 0; i < numOfPositions; i++){
        x = positions[i][0];
        y = positions[i][1];
        xx = t[0] * x + t[1] * y + t[2];
        yy = t[3] * x + t[4] * y + t[5];
        ww = t[6] * x + t[7] * y + t[8];
        xx = xx / ww;
        yy = yy / ww;
        positions[i][0] = xx;
        positions[i][1] = yy;
    }
}

//
// function : CalculateDistance
// usage : CalculateDistance(H, corspMap, inlierMap);
// ---------------------------------------------------
// This function calculates distance of data using
// symmetric transfer error. Then, compute inliears
// that consist with H.
//
void CalculateDistance(CvMat *H, PairPoints *corspMap, PairPoints *inlierMap) {
    int i;
    int x1, y1, x2, y2;
    float x1Trans, y1Trans, w1Trans, x2Trans, y2Trans, w2Trans;
    float dist2x1AndInvHx2, dist2x2AndHx1, dist2Trans;
    float tSquare = T_SQUARE;
    CvMat *invH = cvCreateMat(3, 3, CV_32FC1);
    cvInvert(H, invH);
    // use d^2_transfer as distance measure
    for(i = 0; i < corspMap->len; i++){
        // I -> y, J -> x
        x1 = corspMap->pt1J[i];
        y1 = corspMap->pt1I[i];
        x2 = corspMap->pt2J[i];
        y2 = corspMap->pt2I[i];
        // calculate x_trans = H * x
        x2Trans = cvmGet(H, 0, 0) * x1 + cvmGet(H, 0, 1) * y1
        + cvmGet(H, 0, 2);
        y2Trans = cvmGet(H, 1, 0) * x1 + cvmGet(H, 1, 1) * y1
        + cvmGet(H, 1, 2);
        w2Trans = cvmGet(H, 2, 0) * x1 + cvmGet(H, 2, 1) * y1
        + cvmGet(H, 2, 2);
        x2Trans = x2Trans / w2Trans;
        y2Trans = y2Trans / w2Trans;
        // calculate x’_trans = H^(-1) * x’
        x1Trans = cvmGet(invH, 0, 0) * x2 + cvmGet(invH, 0, 1) * y2
        + cvmGet(invH, 0, 2);
        y1Trans = cvmGet(invH, 1, 0) * x2 + cvmGet(invH, 1, 1) * y2
        + cvmGet(invH, 1, 2);
        w1Trans = cvmGet(invH, 2, 0) * x2 + cvmGet(invH, 2, 1) * y2
        + cvmGet(invH, 2, 2);
        x1Trans = x1Trans / w1Trans;
        y1Trans = y1Trans / w1Trans;
        // calculate the square distance (symmetric transfer error)
        dist2x1AndInvHx2 = pow(x1 - x1Trans, 2) + pow(y1 - y1Trans, 2);
        dist2x2AndHx1 = pow(x2 - x2Trans, 2) + pow(y2 - y2Trans, 2);
        dist2Trans = dist2x1AndInvHx2 + dist2x2AndHx1;
        if(dist2Trans < tSquare){
            UpdatePairPoints(inlierMap, y1, x1, y2, x2);
        }
    }
    // release matrices
    cvReleaseMat(&invH);
}
//
// function : IsGoodSample
// usage : r = IsGoodSample(points, numOfPoints)
// -------------------------------------------------
// This function checks colinearity of all given points.
//
bool IsGoodSample(float points[][2], int numOfPoints) {
    bool r;
    int i, j, k;
    CvMat *A = cvCreateMat(3, 1, CV_32FC1);
    CvMat *B = cvCreateMat(3, 1, CV_32FC1);
    CvMat *C = cvCreateMat(3, 1, CV_32FC1);
    i = 0;
    j = i + 1;
    k = j + 1;
    r = false;
    // check colinearity recursively
    while(true){
        // set point vectors
        cvmSet(A, 0, 0, points[i][0]);
        cvmSet(A, 1, 0, points[i][1]);
        cvmSet(A, 2, 0, 1);
        cvmSet(B, 0, 0, points[j][0]);
        cvmSet(B, 1, 0, points[j][1]);
        cvmSet(B, 2, 0, 1);
        cvmSet(C, 0, 0, points[k][0]);
        cvmSet(C, 1, 0, points[k][1]);
        cvmSet(C, 2, 0, 1);
        // check linearity
        r = IsColinear(A, B, C) || r;
        // update point index
        if(k < numOfPoints - 1){
            k += 1;
        }else{
            if(j < numOfPoints - 2){
                j += 1;
                k = j + 1;
            }else{
                if(i < numOfPoints - 3){
                    i += 1;
                    j = i + 1;
                    k = j + 1;
                }else{
                    break;
                }
            }
        }
    }
    return(!r);
}
//
// function : IsColinear
// usage : r = IsColinear(A, B, C);
// --------------------------------------
// This function checks the colinearity of
// the given 3 points A, B, and C.
// If these are colinear, it returns false.
//
bool IsColinear(CvMat *A, CvMat *B, CvMat *C) {
    float d;
    CvMat *lineAB;
    lineAB = cvCreateMat(3, 1, CV_32FC1);
    cvCrossProduct(A, B, lineAB);
    d = cvDotProduct(lineAB, C);
    // release matrices
    cvReleaseMat(&lineAB);
    if((d < EPS) && (d > -EPS)){
        return(true);
    }else{
        return(false);
    }
}


