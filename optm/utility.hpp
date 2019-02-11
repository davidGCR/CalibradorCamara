//
//  utility.hpp
//  
//
//  Created by David Choqueluque Roman on 2/10/19.
//

#ifndef utility_hpp
#define utility_hpp

#include <stdio.h>
//
// file : utility.h
//-----------------------
// this file contains utility functions to
// deal with general processes.
//
#define OFF 0
#define ON 1
#define EPS 0.5
#define IMPLEMENTATION 2
#define min(a, b) ((a <= b) ? a : b)
#define max(a, b) ((a>= \b) ? a : b)
#define MAX_POINT_SIZE 500
typedef struct{
    int len;
    int pt1I[MAX_POINT_SIZE];
    int pt1J[MAX_POINT_SIZE];
    int pt2I[MAX_POINT_SIZE];
    int pt2J[MAX_POINT_SIZE];
}PairPoints;
typedef struct{
    int len;
    int ptI[MAX_POINT_SIZE];
    int ptJ[MAX_POINT_SIZE];
}Points;
void Array2CvMat(float *arr, CvMat *cvArr, int row, int column);
void CvMat2Array(CvMat *cvArr, float *arr, int row, int column);
void CvImageCopyFloat2Uchar(IplImage *src, IplImage *dst);
void CvImageCopyUchar2Float(IplImage *src, IplImage *dst);
void InitializeImage(IplImage *image);
void CombineTwoImages(IplImage *image1, IplImage *image2,
                      IplImage *outImage);
void WriteImage(IplImage *image, char *imageName);
void MakeImageBlock(IplImage *block, IplImage *image, int centPosI, int
                    centPosJ);
void TransformImage(IplImage *inImage, IplImage *outImage, CvMat *H);
void CrossPointOf2Lines(float *line1, float *line2, float *pt);
void PassLineOf2Points(float *pt1, float *pt2, float *line);
void InitializePairPoints(PairPoints *corspMap);
void CopyPairPoints(PairPoints *dst, PairPoints *src);
void UpdatePairPoints(PairPoints *corspMap, int domainPosI, int domainPosJ,
                      int rangePosI, int rangePosJ);
void LeastSquare(CvMat *A, CvMat*y, CvMat *xLS);

#endif /* utility_hpp */
