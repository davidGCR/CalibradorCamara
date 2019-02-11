//
//  homography.hpp
//  
//
//  Created by David Choqueluque Roman on 2/10/19.
//

#ifndef homography_hpp
#define homography_hpp

#include <stdio.h>
//
// file : homography.h
//------------------------------------------------
// this file contains functions for homography
// estimation
//
#define T_SQUARE 36 // 6 * sigma^2 ; as sigma^2 = 6
void HomograhyEstimation(PairPoints *corspMap, CvMat *H);
void RansacHomograhyEstimation(PairPoints *corspMap, PairPoints *inlierMap,
                               CvMat *H);
void ComputeHomography(float domainPosiitons[][2], float rangePosiitons[][2],
                       int numOfCorresp, CvMat *H);
void DataNormalization(int numOfPositions, float positions[][2], CvMat *T);
void CalculateDistance(CvMat *H, PairPoints *corspMap, PairPoints *inlierMap);
bool IsGoodSample(float points[][2], int numOfPoints);
bool IsColinear(CvMat *A, CvMat *B, CvMat *C);
#endif /* homography_hpp */
