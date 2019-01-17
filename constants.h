//
//  constants.h
//  testOpencv
//
//  Created by David Choqueluque Roman on 1/15/19.
//  Copyright © 2019 David Choqueluque Roman. All rights reserved.
//
#include <iostream>
using namespace std;

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
// const string PATH_DATA = "/home/david/Escritorio/calib-data/";
const string PATH_DATA = "data/";
const string PATH_DATA_FRAMES = PATH_DATA+"frames/";