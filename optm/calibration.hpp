//
//  calibration.hpp
//  
//
//  Created by David Choqueluque Roman on 2/10/19.
//

#ifndef calibration_hpp
#define calibration_hpp

#include <stdio.h>
//
// file : calibration.h
//------------------------------------------------
// this file contains functions for calibration
//
#define MAX_NUM_IMAGE 6
#define MAX_POINT_SIZE 500
#define NUM_OF_ITERATION 1000
#define LAMBDA 0.01
typedef struct{
    int len;
    // intrinsic parameters
    float alphaX;
    float alphaY;
    float x0;
    float y0;
    float skew;
    // extrinsic parameters
    float r1[3][MAX_NUM_IMAGE];
    float r2[3][MAX_NUM_IMAGE];
    float r3[3][MAX_NUM_IMAGE];
    float t[3][MAX_NUM_IMAGE];
    // radial distortion
    float k1;
    float k2;
}CameraParam;
typedef struct{
    int len;
    int numPointsRow;
    int numPointsColumn;
    float H[3][3][MAX_NUM_IMAGE];
}HomographySet;
typedef struct{
    int pointLen;
    int imageLen;
    int ptmI[MAX_POINT_SIZE][MAX_NUM_IMAGE];
    int ptmJ[MAX_POINT_SIZE][MAX_NUM_IMAGE];
    int ptiI[MAX_POINT_SIZE][MAX_NUM_IMAGE];
    int ptiJ[MAX_POINT_SIZE][MAX_NUM_IMAGE];
}PairPointSet;

void CameraCalibration(HomographySet *h, CameraParam *cp, PairPointSet
                       *imagePt);
void CalculateIntrinsicMatrix(HomographySet *h, CvMat *K, CameraParam *cp);
void CalculateExtrinsicParameters(HomographySet *h, CvMat *K, CameraParam *cp);
void vijSetting(float *v, int i, int j, CvMat *H);
void CalculateRadialDistotion(HomographySet *h, CvMat *K, CameraParam *cp,
                              PairPointSet *imagePt);
void RefineCameraParameters(CameraParam *cp, PairPointSet *imagePt);
double CalculateError(CameraParam *cp, PairPointSet *imagePt, CvMat *d);
void Rodrigues2R(float wx, float wy, float wz, CvMat *R);
void WriteParameters(CameraParam *cp, char *name);

#endif /* calibration_hpp */
