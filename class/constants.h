//
//  constants.h
//  testOpencv
//
//  Created by David Choqueluque Roman on 1/15/19.
//  Copyright © 2019 David Choqueluque Roman. All rights reserved.
//

#ifndef constants_H
#define constants_H

#include <iostream>
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"

using namespace std;
using namespace cv;

const int  MIN_POINTS_ELL_FT = 4; //minimo numero de puntos para ellipse fitting
const int  IDX_SON = 2; //indice del hijo en jerarquia
const int  IDX_FATHER = 3; //indice del padre en jerarquia
const float  DST_2_ELLIPS = 5;
const int NUM_NEAR_ELLIPS = 2;

const int PATTERN_NUM_COLS = 5;
const int PATTERN_NUM_ROWS = 4;

const int ROWS_CTRL_PTS2 = 5;
const int COL_CTRL_PTS2 = 4;
const int REAL_NUM_CTRL_PTS = PATTERN_NUM_ROWS*PATTERN_NUM_COLS;
const float TRACK_THRESHOLD = 10;
const int NUM_FRAMES_FOR_CALIBRATION = 45;
//const string PATH_DATA = "/home/david/Escritorio/calib-data/";
// const string PATH_DATA ="/Users/davidchoqueluqueroman/Desktop/CURSOS-MASTER/IMAGENES/testOpencv/data/";
// const string PATH_RESULTS ="/Users/davidchoqueluqueroman/Desktop/CURSOS-MASTER/IMAGENES/testOpencv/results/";

const string PATH_DATA =".../data/";
const string PATH_RESULTS ="../results/";

// const string PATH_DATA = "data/";
const string PATH_DATA_FRAMES = PATH_DATA+"frames/";

const Scalar red(0, 0, 255);
const Scalar yellow(0, 255, 255);
const Scalar blue(255, 0, 0);
const Scalar green(0, 255, 0);
const Scalar white(255, 255, 255);
const Scalar rose(255, 0, 255);
const Scalar celeste(255, 255 , 0);
const Scalar black(0, 0 , 0);

enum TYPE_REFINEMENT
{
    NORMAL = 1,
    AVERAGE = 2,
    BARICENTER = 3
};

#endif
