//
//  corner.hpp
//  testOpencv
//
//  Created by David Choqueluque Roman on 2/10/19.
//  Copyright © 2019 David Choqueluque Roman. All rights reserved.
//

#ifndef corner_hpp
#define corner_hpp

#include <stdio.h>
//
// file : corner.h
//-----------------------------
// this file contains functions for corner
// detection algorithms
//
/////////////////////////////////////////////
// indexed corner detection parameters //
/////////////////////////////////////////////
// canny edgy detection parameters
#define APERTURE_SIZE 3
#define LOW_THRESHOLD 50
#define HIGH_THRESHOLD 150
#define LINE_DEVIATION_THRESHOLD 50
#define SEARCH_RANGE 32
// search range for true corners
#define NUM_H_LINES 10
#define NUM_V_LINES 8
void IndexedCornerDetect(IplImage *inImage, Points *cornerMap);
void Get2LinePoints(CvSeq *lines, PairPoints *hLinePoints, PairPoints
                    *vLinePoints,
                    int imageWidth, int imageHeight);

void SortingLines(PairPoints *linePoints);
void GroupingLines(PairPoints *linePoints, PairPoints *newLinePoints, int
                   threshold);
void EstimateIndexedCorner(PairPoints *hLinePoints, PairPoints *vLinePoints,
                           Points *cornerMap);
void RefineIndexedCorner(IplImage *inImage, Points *tempCornerMap,
                         Points *cornerMap);
/////////////////////////////////////////////
// Harris corner detection parameters //
/////////////////////////////////////////////
// parameters for window sum (sum fxfx, ...)
#define WINDOW_OPTION CV_BLUR
//#define WINDOW_OPTION CV_GAUSSIAN
#define PARAM1 3
#define PARAM2 3
#define RANGE_NEIGHBOR 6
#define HARRIS_THRESHOLD 500
void HarrisCornerDectect(IplImage *inImage, IplImage *cornerMap,
                         float threshold);
void GetffImage(IplImage *inImage, IplImage *outImage,
                CvMat *kernel1st, CvMat *kernel2nd, CvPoint anchor,
                int windowOption, int param1, int param2);
void FindLocalMaxPoint(IplImage *inImage, IplImage *map, int range);
#endif /* corner_hpp */
