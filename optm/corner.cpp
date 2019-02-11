//
//  corner.cpp
//  testOpencv
//
//  Created by David Choqueluque Roman on 2/10/19.
//  Copyright © 2019 David Choqueluque Roman. All rights reserved.
//

#include "corner.hpp"
//
// file : corner.cpp
//-----------------------------
// this file contains functions for corner
// detection
//
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include "cv.h"
#include "cxcore.h"
#include "highgui.h"
#include "utility.h"
#include "corner.h"
void HarrisCornerDectect(IplImage *inImage,
                         IplImage *cornerImage, float threshold)
{
    IplImage *tempImage = 0;
    IplImage *fxfxImage = 0, *fxfyImage = 0, *fyfyImage = 0;
    IplImage *eigenvalueImage = 0;
    float *eigenData = 0;
    float *fxfxData = 0, *fxfyData = 0, *fyfyData = 0;
    float fxfx, fxfy, fyfy;
    float slambda, lambda1, lambda2, lambda3;
    int i, j, k;
    int height, width, channels, step;
    int eigStep;
    CvPoint anchor;
    float dx[9] = {-1, 0, 1, -1, 0, 1, -1, 0, 1}; // derivative masks
    float dy[9] = {1, 1, 1, 0, 0, 0, -1, -1, -1}; // derivative masks
    float sMask[9] = {1, 1, 1, 1, 1, 1, 1, 1, 1}; // window sum mask
    CvMat *Dx = cvCreateMat(3, 3, CV_32FC1); // derivative mask
    CvMat *Dy = cvCreateMat(3, 3, CV_32FC1); // derivative mask
    CvMat *window = cvCreateMat(3, 3, CV_32FC1); // window sum mask
    CvMat *G = cvCreateMat(2, 2, CV_32FC1); // Harris Matrix
    CvMat *q = cvCreateMat(2, 2, CV_32FC1); // eigen vector of G
    CvMat *lambda = cvCreateMat(2, 1, CV_32FC1); // eigenvalue of G
    // assign predefined values to matrices
    Array2CvMat(dx, Dx, 3, 3);
    Array2CvMat(dy, Dy, 3, 3);
    Array2CvMat(sMask, window, 3, 3);
    height = inImage->height;
    width = inImage->width;
    channels = inImage->nChannels;
    step = inImage->widthStep;
    eigStep = cornerImage->widthStep;
    // create the processing image
    tempImage = cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, channels);
    fxfxImage = cvCreateImage(cvSize(width, height), IPL_DEPTH_32F, channels);
    fxfyImage = cvCreateImage(cvSize(width, height), IPL_DEPTH_32F, channels);
    fyfyImage = cvCreateImage(cvSize(width, height), IPL_DEPTH_32F, channels);
    eigenvalueImage = cvCreateImage(cvSize(width, height), IPL_DEPTH_32F, 1);
    
    fxfxData = (float *)fxfxImage->imageData;
    fxfyData = (float *)fxfyImage->imageData;
    fyfyData = (float *)fyfyImage->imageData;
    eigenData = (float *)eigenvalueImage->imageData;
    // LPF filtering to reduce noise
    cvSmooth(inImage, tempImage, CV_GAUSSIAN, 3, 0, 0);
    // get fxfx, fxfy, fyfy images
    anchor = cvPoint(0, 0);
    GetffImage(tempImage, fxfxImage, Dx, Dx, anchor, WINDOW_OPTION, PARAM1, PARAM2);
    GetffImage(tempImage, fyfyImage, Dy, Dy, anchor, WINDOW_OPTION, PARAM1, PARAM2);
    GetffImage(tempImage, fxfyImage, Dx, Dy, anchor, WINDOW_OPTION, PARAM1, PARAM2);
    cvReleaseImage(&tempImage);
    // every fxfx, fxfy, and fyfy image pixel is summed by window
    // to construct the matrix
    // [window sum (fxfx) window sum (fxfy)]
    // [window sum (fxfy) window sum (fyfy)]
    // find small eigenvalues for each pixel
    for(i = 0; i < height; i++){
        for(j = 0; j < width; j++){
            for(k = 0; k < channels; k++){
                fxfx = fxfxData[i*step + j*channels + k];
                fxfy = fxfyData[i*step + j*channels + k];
                fyfy = fyfyData[i*step + j*channels + k];
                // set matrix G = [window sum (fxfx) window sum (fxfy)]
                // [window sum (fxfy) window sum (fyfy)]
                cvmSet(G, 0, 0, fxfx);
                cvmSet(G, 0, 1, fxfy);
                cvmSet(G, 1, 0, fxfy);
                cvmSet(G, 1, 1, fyfy);
                // eigen value decomapStepmposition
                cvEigenVV(G, q, lambda);
                // lambda = eigenvalues of G (descending order)
                // q = corresponding orthogonal eigenvectors (rows)
                if(channels == 3){
                    if(k == 0)
                        lambda1 = cvmGet(lambda, 1, 0); // lambda for B
                    else if(k == 1)
                        lambda2 = cvmGet(lambda, 1, 0); // lambda for G
                    else
                        lambda3 = cvmGet(lambda, 1, 0); // lambda for R
                }else{ // channels == 1
                    lambda1 = cvmGet(lambda, 1, 0);
                    20
                }
            }
            if(channels == 3){
                slambda = pow(pow(lambda1, 2) + pow(lambda1, 2) + pow(lambda1, 2), .5);
            }else{
                slambda = lambda1;
            }
            // store the small eigen values that are normalized by threshold
            eigenData[i*eigStep + j] = slambda / threshold;
        }
    }
    // fine local maximum corner points
    FindLocalMaxPoint(eigenvalueImage, cornerImage, RANGE_NEIGHBOR);
    // release images
    cvReleaseImage(&tempImage);
    cvReleaseImage(&fxfxImage);
    cvReleaseImage(&fxfyImage);
    cvReleaseImage(&fyfyImage);
    cvReleaseImage(&eigenvalueImage);
    // release matrices
    cvReleaseMat(&Dx); cvReleaseMat(&Dy);
    cvReleaseMat(&window);
    cvReleaseMat(&G); cvReleaseMat(&q); cvReleaseMat(&lambda);
}
void GetffImage(IplImage *inImage, IplImage *outImage,
                CvMat *kernel1st, CvMat *kernel2nd, CvPoint anchor,
                int windowOption, int param1, int param2)
{
    IplImage *f1Image = 0, *f2Image = 0, *tempImage = 0;
    float *f1Data = 0, *f2Data = 0;
    float *outData;
    int height, width, step, channels;
    int i, j, k;
    // create the output image
    height = inImage->height;
    width = inImage->width;
    channels = inImage->nChannels;
    step = inImage->widthStep;
    f1Image = cvCreateImage(cvSize(width, height), IPL_DEPTH_32F, channels);
    21
    f2Image = cvCreateImage(cvSize(width, height), IPL_DEPTH_32F, channels);
    tempImage = cvCreateImage(cvSize(width, height), IPL_DEPTH_32F, channels);
    f1Data = (float *)f1Image->imageData;
    f2Data = (float *)f2Image->imageData;
    outData = (float *)outImage->imageData;
    // copy input image to float precision image
    CvImageCopyUchar2Float(inImage, tempImage);
    cvFilter2D(tempImage, f1Image, kernel1st, anchor);
    cvFilter2D(tempImage, f2Image, kernel2nd, anchor);
    for(i = 0; i < height; i++){
        for(j = 0; j < width; j++){
            for(k = 0; k < channels; k++){
                outData[i*step + j*channels + k]
                = f1Data[i*step + j*channels + k]
                * f2Data[i*step + j*channels + k];
            }
        }
    }
    // window sum of fxfx, fxfy, or fyfy
    cvCopyImage (outImage, tempImage);
    cvSmooth(tempImage, outImage, windowOption, param1, param2);
    cvReleaseImage(&tempImage);
    cvReleaseImage(&f1Image);
    cvReleaseImage(&f2Image);
}
void FindLocalMaxPoint(IplImage *inImage, IplImage *map, int range) {
    int r, sum, numOfNeighbor;
    int i, j, ii, jj, posI, posJ;
    float current;
    float *inData = 0;
    uchar *mapData = 0;
    int height = inImage->height;
    int width = inImage->width;
    int step = map->widthStep;
    r = range / 2;
    numOfNeighbor = (2*r + 1) * (2*r + 1);
    inData = (float *)inImage->imageData;
    22
    mapData = (uchar *)map->imageData;
    for(i = 0; i < height; i++){
        for(j = 0; j < width; j++){
            // mark the corner on image
            // write the corner position
            current = inData[i*step + j];
            if(current < 1){ // lambda < threshold
                mapData[i*step + j] = false;
            }else{
                // check neighbors
                sum = 0;
                for(ii = -r; ii <= r; ii++){
                    for(jj = -r; jj <= r; jj++){
                        posI = min(max((i+ii), 0), height - 1);
                        posJ = min(max((j+jj), 0), width - 1);
                        sum += (current >= inData[posI*step + posJ]);
                    }
                }
                // if current pixel is maximum in its neighbors
                // sum == numOfNeighbor
                if(sum == numOfNeighbor)
                    mapData[i*step + j] = true;
                else
                    mapData[i*step + j] = false;
            }
        }
    }
}
void IndexedCornerDetect(IplImage *inImage, Points *cornerMap) {
    int largeNum = 50;
    int height, width;
    int i, j, k;
    int thresholdV = LINE_DEVIATION_THRESHOLD;
    int thresholdH = LINE_DEVIATION_THRESHOLD;
    IplImage *edgeImage = 0, *colorImage = 0;
    CvMemStorage *storage = cvCreateMemStorage(0);
    CvSeq *lines = 0;
    PairPoints hLinePoints, vLinePoints;
    PairPoints newHLinePoints, newVLinePoints;
    Points tempCornerMap;
    23
    height = inImage->height;
    width = inImage->width;
    edgeImage = cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, 1);
    colorImage = cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, 3);
    // Canny edge detection
    cvCanny(inImage, edgeImage, LOW_THRESHOLD, HIGH_THRESHOLD, APERTURE_SIZE);
    k = 0;
    while(true){
        // Hough Transform
        lines = cvHoughLines2(edgeImage, storage, CV_HOUGH_STANDARD,
                              1, CV_PI/180, 40, 0, 0 );
        // get 2 points for every line
        Get2LinePoints(lines, &hLinePoints, &vLinePoints, width, height);
        // sort lines in ascendant order
        SortingLines(&hLinePoints);
        SortingLines(&vLinePoints);
        // group lines and represent a line group with mean value of lines
        GroupingLines(&hLinePoints, &newHLinePoints, thresholdH);
        GroupingLines(&vLinePoints, &newVLinePoints, thresholdV);
        if(newHLinePoints.len == NUM_H_LINES
           && newVLinePoints.len == NUM_V_LINES) break;
        if(newHLinePoints.len != NUM_H_LINES)
            thresholdH += newHLinePoints.len - NUM_H_LINES;
        if(newVLinePoints.len != NUM_V_LINES)
            thresholdV += newVLinePoints.len - NUM_V_LINES;
        if(k == largeNum){
            printf("error in line fitting : image is not suitable!!\n");
            exit(0);
        }
        k++;
    }
    // estimate indexed corners with line crossings
    EstimateIndexedCorner(&newHLinePoints, &newVLinePoints, &tempCornerMap);
    // refine indexed corners(corner map) with true corners
    // true corners are obtained using Harris corner detection
    RefineIndexedCorner(inImage, &tempCornerMap, cornerMap);
    24
    /////////////////////////////////////////////////////////////////////////
    // draw and store images
    cvCvtColor(inImage, colorImage, CV_GRAY2BGR);
    for(i = 0; i < newHLinePoints.len; i++){
        CvPoint pt1, pt2;
        pt1.x = newHLinePoints.pt1J[i];
        pt1.y = newHLinePoints.pt1I[i];
        pt2.x = newHLinePoints.pt2J[i];
        pt2.y = newHLinePoints.pt2I[i];
        cvLine(colorImage, pt1, pt2, CV_RGB(0,0,255), 2);
    }
    for(i = 0; i < newVLinePoints.len; i++){
        CvPoint pt1, pt2;
        pt1.x = newVLinePoints.pt1J[i];
        pt1.y = newVLinePoints.pt1I[i];
        pt2.x = newVLinePoints.pt2J[i];
        pt2.y = newVLinePoints.pt2I[i];
        cvLine(colorImage, pt1, pt2, CV_RGB(0,0,255), 2);
    }
    WriteImage(colorImage, "linefitting.jpg");
    WriteImage(edgeImage, "edge.jpg");
    for(k = 0; k < tempCornerMap.len; k++){
        i = tempCornerMap.ptI[k];
        j = tempCornerMap.ptJ[k];
        cvCircle(colorImage, cvPoint(j, i), 3, cvScalar(0, 255, 0), 3);
    }
    WriteImage(colorImage, "estimatedcorner.jpg");
    cvNamedWindow("output image", CV_WINDOW_AUTOSIZE);
    cvShowImage("output image", colorImage);
    cvWaitKey(0);
    cvDestroyWindow("output image");
    /////////////////////////////////////////////////////////////////////////
    cvReleaseMemStorage(&storage);
    cvReleaseImage(&edgeImage);
    cvReleaseImage(&colorImage);
}
//
// function : Get2LinePoints
// usage : Get2LinePoints(lines, hLinePoints, vLinePoints, width, height);
// ---------------------------------------------------------------------------
25
// this function calculates 2 points corresponding to every line.
// also stores the points seperately in vertical line points or horizontal lines.
// it is assumed that vertical lines pass through 2 horizontal image borders and
// horizontal lines pass through 2 vertical image borders.
//
void Get2LinePoints(CvSeq *lines, PairPoints *hLinePoints,
                    PairPoints *vLinePoints, int imageWidth, int imageHeight) {
    int i;
    hLinePoints->len = 0;
    vLinePoints->len = 0;
    // assign 2 points for every line
    // this block is from the course website link
    for(i = 0; i < lines->total; i++ ){
        float* line = (float*)cvGetSeqElem(lines,i);
        float rho = line[0];
        float theta = line[1];
        CvPoint pt1, pt2;
        double a = cos(theta), b = sin(theta), c = tan(theta);
        if( fabs(b) < 0.001 ){
            pt1.x = pt2.x = cvRound(rho);
            pt1.y = 0;
            pt2.y = imageHeight;
        }else if( fabs(a) < 0.001 ){
            pt1.y = pt2.y = cvRound(rho);
            pt1.x = 0;
            pt2.x = imageWidth;
        }else{
            pt1.x = 0;
            pt1.y = cvRound(rho/b);
            if(pt1.y < 0){
                pt1.x = cvRound(rho/a);
                pt1.y = 0;
            }
            if(pt1.y > imageHeight){
                pt1.x = cvRound((pt1.y - imageHeight)*c);
                pt1.y = imageHeight;
            }
            pt2.x = imageWidth;
            pt2.y = cvRound(rho/b - imageWidth/c);
            if(pt2.y < 0){
                pt2.x = cvRound(rho/a);
                pt2.y = 0;
            }
            if(pt2.y > imageHeight){
                26
                pt2.x = cvRound(-1.0 * ((imageHeight - rho/b) * c));
                pt2.y = imageHeight;
            }
        }
        // classify vertical lines and horizontal lines
        // assume that vertical lines pass through 2 horizontal image borders
        // horizontal lines pass through 2 vertical image borders
        if(pt1.x == 0 && pt2.x == imageWidth && pt1.y != pt2.y){
            hLinePoints->pt1J[hLinePoints->len] = pt1.x;
            hLinePoints->pt1I[hLinePoints->len] = pt1.y;
            hLinePoints->pt2J[hLinePoints->len] = pt2.x;
            hLinePoints->pt2I[hLinePoints->len] = pt2.y;
            hLinePoints->len++;
        }else if(pt2.x == 0 && pt1.x == imageWidth && pt1.y != pt2.y){
            hLinePoints->pt1J[hLinePoints->len] = pt2.x;
            hLinePoints->pt1I[hLinePoints->len] = pt2.y;
            hLinePoints->pt2J[hLinePoints->len] = pt1.x;
            hLinePoints->pt2I[hLinePoints->len] = pt1.y;
            hLinePoints->len++;
        }else if(pt1.y == 0 && pt2.y == imageHeight && pt1.x != pt2.x){
            vLinePoints->pt1J[vLinePoints->len] = pt1.x;
            vLinePoints->pt1I[vLinePoints->len] = pt1.y;
            vLinePoints->pt2J[vLinePoints->len] = pt2.x;
            vLinePoints->pt2I[vLinePoints->len] = pt2.y;
            vLinePoints->len++;
        }else if(pt2.y == 0 && pt1.y == imageHeight && pt1.x != pt2.x){
            vLinePoints->pt1J[vLinePoints->len] = pt2.x;
            vLinePoints->pt1I[vLinePoints->len] = pt2.y;
            vLinePoints->pt2J[vLinePoints->len] = pt1.x;
            vLinePoints->pt2I[vLinePoints->len] = pt1.y;
            vLinePoints->len++;
        }
    }
}
//
// function : SortingLines;
// usage : SortingLines(linePoints);
// -------------------------------------------------
// this function sorts lines in ascendant order
//
void SortingLines(PairPoints *linePoints) {
    int mean1, mean2;
    int temp1, temp2, temp3, te
    int i, j;
    int len = linePoints->len;
    for(i = len - 1; i > 0; i--){
        for(j = 0; j < i; j++){
            mean1 = (linePoints->pt1I[j+1] + linePoints->pt2I[j+1]
                     + linePoints->pt1J[j+1] + linePoints->pt2J[j+1]) / 4;
            mean2 = (linePoints->pt1I[j] + linePoints->pt2I[j]
                     + linePoints->pt1J[j] + linePoints->pt2J[j]) / 4;
            if(mean2 > mean1){
                // swap
                temp1 = linePoints->pt1I[j+1];
                temp2 = linePoints->pt1J[j+1];
                temp3 = linePoints->pt2I[j+1];
                temp4 = linePoints->pt2J[j+1];
                linePoints->pt1I[j+1] = linePoints->pt1I[j];
                linePoints->pt1J[j+1] = linePoints->pt1J[j];
                linePoints->pt2I[j+1] = linePoints->pt2I[j];
                linePoints->pt2J[j+1] = linePoints->pt2J[j];
                linePoints->pt1I[j] = temp1;
                linePoints->pt1J[j] = temp2;
                linePoints->pt2I[j] = temp3;
                linePoints->pt2J[j] = temp4;
            }
        }
    }
}
//
// function : GroupingLines
// usage : GroupingLines(linePoints, newLinePoints);
// ----------------------------------------------------
// this function calculates new line point pairs.
// input line point pairs should be in ascendant order.
//
void GroupingLines(PairPoints *linePoints, PairPoints *newLinePoints, int
                   threshold) {
    int i = 0, j;
    int len = newLinePoints->len = 0;
    while(true){
        int tempPt1I = linePoints->pt1I[i];
        int tempPt1J = linePoints->pt1J[i];
        int tempPt2I = linePoints->pt2I[i];
        int tempPt2J = linePoints->pt2J[i];
        int numOfGroup = 1;
        28
        j = i+1;
        if(j >= linePoints->len) break;
        while(true){
            if(j >= linePoints->len) break;
            float diff1 = fabs(linePoints->pt1I[i] - linePoints->pt1I[j]);
            float diff2 = fabs(linePoints->pt1J[i] - linePoints->pt1J[j]);
            float diff3 = fabs(linePoints->pt2I[i] - linePoints->pt2I[j]);
            float diff4 = fabs(linePoints->pt2J[i] - linePoints->pt2J[j]);
            if(diff1 + diff2 + diff3 + diff4 < threshold){
                tempPt1I += linePoints->pt1I[j];
                tempPt1J += linePoints->pt1J[j];
                tempPt2I += linePoints->pt2I[j];
                tempPt2J += linePoints->pt2J[j];
                numOfGroup++;
                j++;
            }else{
                break;
            }
        }
        newLinePoints->pt1I[len] = cvRound(tempPt1I / numOfGroup);
        newLinePoints->pt1J[len] = cvRound(tempPt1J / numOfGroup);
        newLinePoints->pt2I[len] = cvRound(tempPt2I / numOfGroup);
        newLinePoints->pt2J[len] = cvRound(tempPt2J / numOfGroup);
        i = j;
        len++;
    }
    newLinePoints->len = len;
}
//
// function : EstimateIndexedCorner
// usage : EstimateIndexedCorner(hLinePoints, vLinePoints, cornerMap);
//------------------------------------------------------------------------
// this function estimates indexed corners with crossing of ordered
// horizontal lines and ordered vertical lines.
// the cornerMap array is stroed in order.
//
void EstimateIndexedCorner(PairPoints *hLinePoints, PairPoints *vLinePoints,
                           Points *cornerMap)
{
    int i, j;
    int k = 0;
    29
    for(i = 0; i < hLinePoints->len; i++){
        float hPt1[3] = {hLinePoints->pt1J[i], hLinePoints->pt1I[i], 1};
        float hPt2[3] = {hLinePoints->pt2J[i], hLinePoints->pt2I[i], 1};
        float hLine[3] = {0, 0, 0}; // initialize
        // get a horiziontal line that passes through hPt1 and hPt2
        PassLineOf2Points(hPt1, hPt2, hLine);
        for(j = 0; j < vLinePoints->len; j++){
            float vPt1[3] = {vLinePoints->pt1J[j], vLinePoints->pt1I[j], 1};
            float vPt2[3] = {vLinePoints->pt2J[j], vLinePoints->pt2I[j], 1};
            float vLine[3] = {0, 0, 0}; // initialize
            // get a vertical line that passes through vPt1 and vPt2
            PassLineOf2Points(vPt1, vPt2, vLine);
            float pt[3] = {0, 0, 0}; // initialize
            // get a cross point of a horizontal and a vertical lines
            CrossPointOf2Lines(vLine, hLine, pt);
            // store the cross point in the corner map
            cornerMap->ptI[k] = cvRound(pt[1]/pt[2]);
            cornerMap->ptJ[k] = cvRound(pt[0]/pt[2]);
            k++;
        }
    }
    cornerMap->len = k;
}
//
// function : RefineIndexedCorner
// usage: RefineIndexedCorner(inImage, tempCornerMap, cornerMap);
//---------------------------------------------------------------------
// this function refines indexed corners(corner map) with true corners.
// true corners are obtained using Harris corner detection.
//
void RefineIndexedCorner(IplImage *inImage, Points *tempCornerMap,
                         Points *cornerMap)
{
    int i, j, k;
    int initialDistance = 10000;
    float dist, minDist;
    int blockCenterI, blockCenterJ;
    int r = SEARCH_RANGE / 2;
    int height = inImage->height;
    int width = inImage->width;
    30
    IplImage *cornerImage = cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, 1);
    uchar *cornerImageData = 0;
    int step = cornerImage->widthStep;
    cornerImageData = (uchar *)cornerImage->imageData;
    HarrisCornerDectect(inImage, cornerImage, HARRIS_THRESHOLD);
    for(k = 0; k < tempCornerMap->len; k++){
        blockCenterI = tempCornerMap->ptI[k];
        blockCenterJ = tempCornerMap->ptJ[k];
        // find true corner that has the minimum distance between
        // the estimated corner and the true corner
        minDist = initialDistance;
        for(i = blockCenterI - r; i <= blockCenterI + r; i++){
            for(j = blockCenterJ - r; j <= blockCenterJ + r; j++){
                i = max(min(height - 1, i), 0);
                j = max(min(width - 1, j), 0);
                if(cornerImageData[i*step + j] == true){
                    dist = pow(blockCenterI - i, 2) + pow(blockCenterJ - j, 2);
                    if(dist < minDist){
                        cornerMap->ptI[k] = i;
                        cornerMap->ptJ[k] = j;
                        minDist = dist;
                    }
                }
            }
        }
        if(minDist == initialDistance){
            printf("error in refine indexed corner: need larger search area\n");
            exit(0);
        }
    }
    // new number of corners should be the same as
    // number of initially estimated corners
    cornerMap->len = tempCornerMap->len;
}
