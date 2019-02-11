//
//  jacobianCalculation.cpp
//  
//
//  Created by David Choqueluque Roman on 2/10/19.
//


#include "jacobianCalculation.hpp"

//Function Jx1, ..., Jy13 are obtained from Matlab. Here we do not present these functions
//because these are too long.

//
// file : jacobianCalculation.cpp
//---------------------------------------------------
// this file contains a function that calculates
// Jacobian form that is required in LM algorithm
// for camera calibration
//
#include <math.h>
#include "jacobianCalculation.h"
//
// function : CalculateJacobian
//------------------------------------------
// this function returns the Jacobian
// the Jacobian equations are obtained by
// Matlab symbolic function "jacobian"
//
//------------------------------------------
// X : model point x
// Y : model point y
//
// intrinsic parameters
//--------------------------
// au : alphaX
// av : alphaY
// u0 : x0
// v0 : y0
// sk : skew
//
// extrinsic parameters
//--------------------------
// wx, wy, wz : rotation
// tx, ty, tz : translation
//
// radial distortion
//-------------------------
// k1, k2
//
void CalculateJocobian(double *Jx, double *Jy, int imageNum, int len,
                       double X, double Y,
                       double au, double av, double u0, double v0, double sk,
                       double wx, double wy, double wz,double tx, double ty, double tz, double k1, double k2)
{
    // Jx & Jy size : 7 + numOfImage * 6
    //(7 intrinsic camera parameters including k1 & k2,
    // 6 extrinsic parameters depending on images)
    int i;
    int JSize = len; // 7 + numOfImage * 6
    Jx[0] = Jx1(X, Y, au, av, u0, v0, sk, wx, wy, wz, tx, ty, tz, k1, k2);
    Jx[1] = Jx2(X, Y, au, av, u0, v0, sk, wx, wy, wz, tx, ty, tz, k1, k2);
    Jx[2] = -1.0; //Jx3 = -1.0;
    Jx[3] = 0.0; //Jx4 = 0.0;
    Jx[4] = Jx5(X, Y, au, av, u0, v0, sk, wx, wy, wz, tx, ty, tz, k1, k2);
    Jy[0] = Jy1(X, Y, au, av, u0, v0, sk, wx, wy, wz, tx, ty, tz, k1, k2);
    Jy[1] = Jy2(X, Y, au, av, u0, v0, sk, wx, wy, wz, tx, ty, tz, k1, k2);
    Jy[2] = Jy3(X, Y, au, av, u0, v0, sk, wx, wy, wz, tx, ty, tz, k1, k2);
    Jy[3] = Jy4(X, Y, au, av, u0, v0, sk, wx, wy, wz, tx, ty, tz, k1, k2);
    Jy[4] = Jy5(X, Y, au, av, u0, v0, sk, wx, wy, wz, tx, ty, tz, k1, k2);
    for(i = 5; i < JSize; i++){
        Jx[i] = 0;
        Jy[i] = 0;
    }
    Jx[5 + imageNum * 6] = Jx6(X, Y, au, av, u0, v0, sk,
                               wx, wy, wz, tx, ty, tz, k1, k2);
    Jx[6 + imageNum * 6] = Jx7(X, Y, au, av, u0, v0, sk,
                               wx, wy, wz, tx, ty, tz, k1, k2);
    Jx[7 + imageNum * 6] = Jx8(X, Y, au, av, u0, v0, sk,
                               wx, wy, wz, tx, ty, tz, k1, k2);
    Jx[8 + imageNum * 6] = Jx9(X, Y, au, av, u0, v0, sk,
                               wx, wy, wz, tx, ty, tz, k1, k2);
    Jx[9 + imageNum * 6] = Jx10(X, Y, au, av, u0, v0, sk,
                                wx, wy, wz, tx, ty, tz, k1, k2);
    Jx[10 + imageNum * 6] = Jx11(X, Y, au, av, u0, v0, sk,
                                 wx, wy, wz, tx, ty, tz, k1, k2);
    Jy[5 + imageNum * 6] = Jy6(X, Y, au, av, u0, v0, sk,
                               wx, wy, wz, tx, ty, tz, k1, k2);
    Jy[6 + imageNum * 6] = Jy7(X, Y, au, av, u0, v0, sk,
                               wx, wy, wz, tx, ty, tz, k1, k2);
    Jy[7 + imageNum * 6] = Jy8(X, Y, au, av, u0, v0, sk,
                               wx, wy, wz, tx, ty, tz, k1, k2);
    Jy[8 + imageNum * 6] = Jy9(X, Y, au, av, u0, v0, sk,
                               wx, wy, wz, tx, ty, tz, k1, k2);
    Jy[9 + imageNum * 6] = Jy10(X, Y, au, av, u0, v0, sk,
                                wx, wy, wz, tx, ty, tz, k1, k2);
    Jy[10 + imageNum * 6] = Jy11(X, Y, au, av, u0, v0, sk,
                                 wx, wy, wz, tx, ty, tz, k1, k2);
    Jx[JSize - 2] = Jx12(X, Y, au, av, u0, v0, sk,
                         wx, wy, wz, tx, ty, tz, k1, k2); // for k1
    Jx[JSize - 1] = Jx13(X, Y, au, av, u0, v0, sk,
                         wx, wy, wz, tx, ty, tz, k1, k2); // for k2
    Jy[JSize - 2] = Jy12(X, Y, au, av, u0, v0, sk,
                         wx, wy, wz, tx, ty, tz, k1, k2); // for k1
    Jy[JSize - 1] = Jy13(X, Y, au, av, u0, v0, sk,
                         wx, wy, wz, tx, ty, tz, k1, k2); // for k2
}
