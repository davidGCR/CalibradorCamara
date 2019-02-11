//
//  utility.cpp
//  
//
//  Created by David Choqueluque Roman on 2/10/19.
//

#include "utility.hpp"
//
// file : utility.cpp
//-----------------------
// this file contains utility functions to
// deal with general processes.
//
#include <stdlib.h>
#include <stdio.h>
#include <math.h>

#include "cv.h"
#include "cxcore.h"
#include "highgui.h"
#include "utility.h"
void Array2CvMat(float *arr, CvMat *cvArr, int row, int column) {
    int i, j;
    for(i = 0; i < row; i++){
        for(j = 0; j < column; j++){
            cvmSet(cvArr, i, j, arr[i*column + j]);
        }
    }
}
void CvMat2Array(CvMat *cvArr, float *arr, int row, int column) {
    int i, j;
    for(i = 0; i < row; i++){
        for(j = 0; j < column; j++){
            arr[i*column + j] = cvmGet(cvArr, i, j);
        }
    }
}
void CvImageCopyFloat2Uchar(IplImage *src, IplImage *dst) {
    int i, j, k;
    float pixel;
    int height = src->height;
    int width = src->width;
    int channels = src->nChannels;
    int step = dst->widthStep;
    uchar *dstData = (uchar *)dst->imageData;
    float *srcData = (float *)src->imageData;
    // copy float precision image to uchar precision image
    for(i = 0; i < height; i++){
        for(j = 0; j < width; j++){
            for(k = 0; k < channels; k++){
                pixel = srcData[i*step + j*channels + k];
                pixel = (pixel > 255 ? 255 : pixel);
                pixel = (pixel < 0 ? 0 : pixel);
                dstData[i*step + j*channels + k] = (uchar)pixel;
            }
            
        }
    }
}
void CvImageCopyUchar2Float(IplImage *src, IplImage *dst) {
    int i, j, k;
    int height = src->height;
    int width = src->width;
    int channels = src->nChannels;
    int step = src->widthStep;
    float *dstData = (float *)dst->imageData;
    uchar *srcData = (uchar *)src->imageData;
    // copy uchar precision image to float precision image
    for(i = 0; i < height; i++){
        for(j = 0; j < width; j++){
            for(k = 0; k < channels; k++){
                dstData[i*step + j*channels + k]
                = (float)srcData[i*step + j*channels + k];
            }
        }
    }
}
void InitializeImage(IplImage *image) {
    int i, j, k;
    int height = image->height;
    int width = image->width;
    int channels = image->nChannels;
    int step = image->widthStep;
    uchar *imageData = (uchar *)image->imageData;
    for(i = 0; i < height; i++){
        for(j = 0; j < width; j++){
            for(k = 0; k < channels; k++){
                imageData[i*step + j*channels + k]
                = 0;
            }
        }
    }
}
void CombineTwoImages(IplImage *image1, IplImage *image2,
                      IplImage *outImage)

{
    int i, j, k;
    uchar *outImageData = 0, *image1Data = 0, *image2Data = 0;
    int height = image1->height;
    int width = image1->width;
    int step = image1->widthStep;
    int channels = image1->nChannels;
    int outWidth = outImage->width;
    int outHeight = outImage->height;
    int outStep = outImage->widthStep;
    if(outWidth == width * 2 && outHeight == height){
    }else if(outWidth == width && outHeight == height * 2){
    }else{
        printf("image combining error\n");
        exit(0);
    }
    outImageData = (uchar *)outImage->imageData;
    image1Data = (uchar *)image1->imageData;
    image2Data = (uchar *)image2->imageData;
    for(i = 0; i < outHeight; i++){
        for(j = 0; j < outWidth; j++){
            for(k = 0; k < channels; k++){
                if(i < height && j < width){
                    outImageData[i*outStep + j*channels + k]
                    = image1Data[i*step + j*channels + k];
                }else if((i >= height && j < width)){
                    outImageData[i*outStep + j*channels + k]
                    = image2Data[(i-height)*step + j*channels + k];
                }else if((i < height && j >= width)){
                    outImageData[i*outStep + j*channels + k]
                    = image2Data[i*step + (j-width)*channels + k];
                }else{
                    printf("there is no i > height & j > width \n");
                    exit(0);
                }
            }
        }
    }
}
void WriteImage(IplImage *image, char *imageName) {
    if(!cvSaveImage(imageName, image)){
        
        printf("Could not save: %s\n", imageName);
    }
}
//
// function : MakeImageBlock
// usage : MakeImageBlock(block, image, centPosI, centPosJ);
// ------------------------------------------------------------
// This function copies a block region of the image into a block
// for example, if block size is 3 by 3 and the position of the block
// is i, j on the image. the resultant block will be 3 by 3 and
// the block will be copied by image(i-1, j-1) ... image(i+1, j+1).
//
void MakeImageBlock(IplImage *block, IplImage *image, int centPosI, int
                    centPosJ) {
    uchar *blockData = 0, *imageData = 0;
    int blockHeight, blockWidth, imageHeight, imageWidth;
    int blockStep, channels, imageStep;
    int i, j, k, posI, posJ;
    blockHeight = block->height;
    blockWidth = block->width;
    imageHeight = image->height;
    imageWidth = image->width;
    channels = block->nChannels;
    blockStep = block->widthStep;
    imageStep = image->widthStep;
    blockData = (uchar *)block->imageData;
    imageData = (uchar *)image->imageData;
    for(i = 0; i < blockHeight; i++){
        for(j = 0; j < blockWidth; j++){
            for(k = 0; k < channels; k++){
                posI = centPosI + i - blockHeight / 2;
                posJ = centPosJ + j - blockWidth / 2;
                posI = min(max(posI, 0), imageHeight - 1);
                posJ = min(max(posJ, 0), imageWidth - 1);
                blockData[i*blockStep + j*channels + k]
                = imageData[posI*imageStep + posJ*channels + k];
            }
        }
    }
}
//
// function : TransformImage
// usage : TransformImage(inImage, outImage, H);
// ---------------------------------------------
// This function transforms input image using H
//
void TransformImage(IplImage *inImage, IplImage *outImage, CvMat *H) {
    uchar *inData;
    uchar *outData;
    int height, width, step, channels;
    int i, j, k;
    // get the input image data
    height = inImage->height;
    width = inImage->width;
    step = inImage->widthStep;
    channels = inImage->nChannels;
    inData = (uchar *)inImage->imageData;
    outData = (uchar *)outImage->imageData;
    // apply the transform to get the target image
    // -----------------------------------------------------
    // out(x’) = in(x) : has 2 implementation forms
    // case 1 : out(Hx) = in(x)
    // case 2 : out(x’) = inv(H)x’
    CvMat *invH = cvCreateMat(3, 3, CV_32FC1);
    cvInvert(H, invH);
    float h[9];
    if(IMPLEMENTATION == 1){ // case 1 : out(Hx) = in(x)
        CvMat2Array(H, h, 3, 3);
    }else{ // case 2 : x = inv(H)x’
        CvMat2Array(invH, h, 3, 3);
    }
    int ii, jj;
    float x1, x2, x3;
    for(i = 0; i < height; i++){ // case 1 : i, j : x, ii, jj : x’, x’ = Hx
        for(j = 0; j < width; j++){ // case 2 : i, j : x’, ii, jj : x, x = invHx’
            for(k = 0; k < channels; k++){ // x : domain, x’ : range
                x1 = h[0] * j + h[1] * i + h[2];
                x2 = h[3] * j + h[4] * i + h[5];
                x3 = h[6] * j + h[7] * i + h[8];
                ii = min(height - 1, max(0, (int)(x2 / x3)));
                jj = min(width - 1, max(0, (int)(x1 / x3)));
                if(IMPLEMENTATION == 1){ // case 1 : out(Hx) = in(x)
                    outData[ii*step + jj*channels + k]
                    = inData[i*step + j*channels + k];
                    
                }else{ // case 2 : out(x’) = in(inv(H)x’)
                    if(ii == 0 || ii == height -1 || jj == 0 || jj == width - 1){
                        outData[i*step + j*channels + k] = 0;
                    }else{
                        outData[i*step + j*channels + k]
                        = inData[ii*step + jj*channels + k];
                    }
                }
            }
        }
    }
}
//
// function : CrossPointOf2Lines
// usage : CrossPointOf2Lines(line1, line2, pt);
//-------------------------------------------------------------
// this function calculates the cross point of line1 and line2
//
void CrossPointOf2Lines(float *line1, float *line2, float *pt) {
    CvMat *cvLine1 = cvCreateMat(3, 1, CV_32FC1);
    CvMat *cvLine2 = cvCreateMat(3, 1, CV_32FC1);
    CvMat *cvPt = cvCreateMat(3, 1, CV_32FC1);
    Array2CvMat(line1, cvLine1, 3, 1);
    Array2CvMat(line2, cvLine2, 3, 1);
    cvCrossProduct(cvLine1, cvLine2, cvPt);
    CvMat2Array(cvPt, pt, 3, 1);
}
//
// function : PassLineOf2Points
// usage : PassLineOf2Points(pt1, pt2, line);
//-----------------------------------------------------------------
// this function calculates the line that pass through pt1 and pt2
//
void PassLineOf2Points(float *pt1, float *pt2, float *line) {
    // this function is dual of CrossPointsOf2Lines
    CrossPointOf2Lines(pt1, pt2, line);
}
void InitializePairPoints(PairPoints *corspMap) {
    for(int i = 0; i < MAX_POINT_SIZE; i++){
        66
        corspMap->pt1I[i] = 0;
        corspMap->pt1J[i] = 0;
        corspMap->pt2I[i] = 0;
        corspMap->pt2J[i] = 0;
    }
    corspMap->len = 0;
}
void CopyPairPoints(PairPoints *dst, PairPoints *src) {
    for(int i = 0; i < MAX_POINT_SIZE; i++){
        dst->pt1I[i] = src->pt1I[i];
        dst->pt1J[i] = src->pt1J[i];
        dst->pt2I[i] = src->pt2I[i];
        dst->pt2J[i] = src->pt2J[i];
    }
    dst->len = src->len;
}
void UpdatePairPoints(PairPoints *corspMap, int domainPosI, int domainPosJ,
                      int rangePosI, int rangePosJ)
{
    int len;
    len = corspMap->len;
    if(corspMap->len >= MAX_POINT_SIZE){
        printf("UpdateCorrespMap called on a full corspMap\n");
        printf("Next positions of correspondences will be overwritten\n");
        printf("in the current correspondece \n");
        len = MAX_POINT_SIZE - 1;
    }
    corspMap->pt2I[len] = rangePosI;
    corspMap->pt2J[len] = rangePosJ;
    corspMap->pt1I[len] = domainPosI;
    corspMap->pt1J[len] = domainPosJ;
    corspMap->len = len + 1;
}
//
// function : LeastSquare
// usage : LeastSquare(A, y, xLS);
//-------------------------------------------------
// this function returns the least squre solution of
// Ax = y ; xLs = inv(A’A)A’y
//
void LeastSquare(CvMat *A, CvMat*y, CvMat *xLS)
{
    int n, m; // A is n by m
    CvSize size = cvGetSize(A);
    m = size.width;
    n = size.height;
    CvMat *B = cvCreateMat(m, m, CV_64FC1);
    CvMat *invB = cvCreateMat(m, m, CV_64FC1);
    CvMat *transA = cvCreateMat(m, n, CV_64FC1);
    CvMat *C = cvCreateMat(m, n, CV_64FC1);
    cvMulTransposed (A, B, 1);
    cvInvert(B, invB, CV_SVD);
    cvTranspose(A, transA);
    cvMatMul(invB, transA, C);
    cvMatMul(C, y, xLS);
    // release matrices
    cvReleaseMat(&B);
    cvReleaseMat(&invB);
    cvReleaseMat(&transA);
    cvReleaseMat(&C);
}
