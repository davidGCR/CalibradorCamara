//
//  jacobian.h
//  
//
//  Created by David Choqueluque Roman on 2/10/19.
//

#ifndef jacobian_h
#define jacobian_h

#include <cmath>
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "constants.h"

//#include <cv.h>
#include <iostream>
using namespace cv;
using namespace std;


//< Function setJall >
//Construct Jacobian matrix for the image w.r.t parameter vector P and ObjectPoints (for the entire
//                                                                                   images)
void setJall(CvMat *P,CvMat *J, int Nfiles, CvMat *ObjectPoints){
    // P : 5+6*Nfiles
    int i,j;
    double u0, v0, au, av, sk;
    double tx, ty, tz, wx, wy, wz;
    double X, Y;
    int x_y_total = 2*REAL_NUM_CTRL_PTS;
    // dummy variables
    double
    MapleGenVar1,MapleGenVar2,MapleGenVar3,MapleGenVar4,MapleGenVar5,MapleGenVar6,MapleGenVar7,MapleGenVar8,MapleGenVar9,
    MapleGenVar10;
    au=cvmGet(P,0,0);
    av=cvmGet(P,1,0);
    u0=cvmGet(P,2,0);
    v0=cvmGet(P,3,0);
    sk=cvmGet(P,4,0);
    for(i=0;i<x_y_total*Nfiles;i++) for(j=0;j<5+6*Nfiles;j++) cvmSet(J,i,j,0);
    for(j=0;j<Nfiles;j++)
    {
        wx=cvmGet(P,5+6*j,0);
        wy=cvmGet(P,6+6*j,0);
        wz=cvmGet(P,7+6*j,0);
        tx=cvmGet(P,8+6*j,0);
        ty=cvmGet(P,9+6*j,0);
        tz=cvmGet(P,10+6*j,0);
        for(i=0;i<REAL_NUM_CTRL_PTS;i++)
        {
            X=cvmGet(ObjectPoints,0,i);
            Y=cvmGet(ObjectPoints,1,i);
            // Setup Jacobian matrix
            // intrinsic parameters
            cvmSet(J,x_y_total*j+2*i , 0+j*6 , -((1.0+(1.0-cos(sqrt(wx*wx+wy*wy+wz*wz)))/(wx*wx+wy*wy+wz*wz)*(-
                                                                                                        wz*wz-wy*wy))*X+(-sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(wx*wx+wy*wy+wz*wz)*wz+(1.0-
                                                                                                                                                                                   cos(sqrt(wx*wx+wy*wy+wz*wz)))/(wx*wx+wy*wy+wz*wz)*wy*wx)*Y+tx)/((-
                                                                                                                                                                                                                                                    sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(wx*wx+wy*wy+wz*wz)*wy+(1.0-
                                                                                                                                                                                                                                                                                                             cos(sqrt(wx*wx+wy*wy+wz*wz)))/(wx*wx+wy*wy+wz*wz)*wz*wx)*X+(sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(wx*wx+wy*wy+wz*wz)*wx+(1.0-cos(sqrt(wx*wx+wy*wy+wz*wz)))/(wx*wx+wy*wy+wz*wz)*wz*wy)*Y+tz));
            cvmSet(J,x_y_total*j+2*i+1 , 0+j*6 , 0 );
            cvmSet(J,x_y_total*j+2*i , 1+j*6 , 0 );
            cvmSet(J,x_y_total*j+2*i+1 , 1+j*6 , -((sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(wx*wx+wy*wy+wz*wz)*wz+(1.0-
                                                                                                       cos(sqrt(wx*wx+wy*wy+wz*wz)))/(wx*wx+wy*wy+wz*wz)*wy*wx)*X+(1.0+(1.0-
                                                                                                                                                                        cos(sqrt(wx*wx+wy*wy+wz*wz)))/(wx*wx+wy*wy+wz*wz)*(-wz*wz-wx*wx))*Y+ty)/((-
                                                                                                                                                                                                                                                  sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(wx*wx+wy*wy+wz*wz)*wy+(1.0-
                                                                                                                                                                                                                                                                                                           cos(sqrt(wx*wx+wy*wy+wz*wz)))/(wx*wx+wy*wy+wz*wz)*wz*wx)*X+(sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(wx*wx+wy*wy+wz*wz)*wx+(1.0-cos(sqrt(wx*wx+wy*wy+wz*wz)))/(wx*wx+wy*wy+wz*wz)*wz*wy)*Y+tz));
            cvmSet(J,x_y_total*j+2*i , 2+j*6 , -1);
            cvmSet(J,x_y_total*j+2*i+1 , 2+j*6 , 0);
            cvmSet(J,x_y_total*j+2*i , 3+j*6 , 0);
            cvmSet(J,x_y_total*j+2*i+1 , 3+j*6 , -1);
            cvmSet(J,x_y_total*j+2*i , 4+j*6 , -((sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(wx*wx+wy*wy+wz*wz)*wz+(1.0-
                                                                                                     cos(sqrt(wx*wx+wy*wy+wz*wz)))/(wx*wx+wy*wy+wz*wz)*wy*wx)*X+(1.0+(1.0-
                                                                                                                                                                      cos(sqrt(wx*wx+wy*wy+wz*wz)))/(wx*wx+wy*wy+wz*wz)*(-wz*wz-wx*wx))*Y+ty)/((-
                                                                                                                                                                                                                                                sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(wx*wx+wy*wy+wz*wz)*wy+(1.0-
                                                                                                                                                                                                                                                                                                         cos(sqrt(wx*wx+wy*wy+wz*wz)))/(wx*wx+wy*wy+wz*wz)*wz*wx)*X+(sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(wx*wx+wy*wy+wz*wz)*wx+(1.0-cos(sqrt(wx*wx+wy*wy+wz*wz)))/(wx*wx+wy*wy+wz*wz)*wz*wy)*Y+tz));
            cvmSet(J,x_y_total*j+2*i+1 , 4+j*6 , 0);
            // extrinsic parameters
            MapleGenVar2 = -1.0;
            MapleGenVar7 = au*(sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(pow(wx*wx+wy*wy+wz*wz,3.0))*wx*(-
                                                                                                 wz*wz-wy*wy)-2.0*(1.0-cos(sqrt(wx*wx+wy*wy+wz*wz)))/pow(wx*wx+wy*wy+wz*wz,2.0)*(-wz*wz-wy*wy)*wx);
            MapleGenVar9 = sk*(cos(sqrt(wx*wx+wy*wy+wz*wz))/(wx*wx+wy*wy+wz*wz)*wx*wz-sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(pow(wx*wx+wy*wy+wz*wz,3.0))*wz*wx+sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(pow(wx*wx+wy*wy
                                                                                                                                                                                               +wz*wz,3.0))*wx*wx*wy-2.0*(1.0-cos(sqrt(wx*wx+wy*wy+wz*wz)))/pow(wx*wx+wy*wy+wz*wz,2.0)*wy*wx*wx+(1.0-
                                                                                                                                                                                                                                                                                                 cos(sqrt(wx*wx+wy*wy+wz*wz)))/(wx*wx+wy*wy+wz*wz)*wy);
            MapleGenVar10 = u0*(-
                                cos(sqrt(wx*wx+wy*wy+wz*wz))/(wx*wx+wy*wy+wz*wz)*wx*wy+sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(pow(wx*wx+wy*wy+wz*wz,3.0
                                                                                                                             ))*wy*wx+sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(pow(wx*wx+wy*wy+wz*wz,3.0))*wx*wx*wz-2.0*(1.0-
                                                                                                                                                                                                                  cos(sqrt(wx*wx+wy*wy+wz*wz)))/pow(wx*wx+wy*wy+wz*wz,2.0)*wz*wx*wx+(1.0-
                                                                                                                                                                                                                                                                                     cos(sqrt(wx*wx+wy*wy+wz*wz)))/(wx*wx+wy*wy+wz*wz)*wz);
            MapleGenVar8 = MapleGenVar9+MapleGenVar10;
            MapleGenVar6 = MapleGenVar7+MapleGenVar8;
            MapleGenVar7 = X;
            MapleGenVar5 = MapleGenVar6*MapleGenVar7;
            MapleGenVar8 = au*(-
                               cos(sqrt(wx*wx+wy*wy+wz*wz))/(wx*wx+wy*wy+wz*wz)*wx*wz+sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(pow(wx*wx+wy*wy+wz*wz,3.0)
                                                                                                                        )*wz*wx+sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(pow(wx*wx+wy*wy+wz*wz,3.0))*wx*wx*wy-2.0*(1.0-
                                                                                                                                                                                                            cos(sqrt(wx*wx+wy*wy+wz*wz)))/pow(wx*wx+wy*wy+wz*wz,2.0)*wy*wx*wx+(1.0-
                                                                                                                                                                                                                                                                               cos(sqrt(wx*wx+wy*wy+wz*wz)))/(wx*wx+wy*wy+wz*wz)*wy);
            MapleGenVar9 = sk*(sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(pow(wx*wx+wy*wy+wz*wz,3.0))*wx*(-
                                                                                                 wz*wz-wx*wx)-2.0*(1.0-cos(sqrt(wx*wx+wy*wy+wz*wz)))/pow(wx*wx+wy*wy+wz*wz,2.0)*(-wz*wz-wx*wx)*wx-2.0*(1.0-
                                                                                                                                                                                                       cos(sqrt(wx*wx+wy*wy+wz*wz)))/(wx*wx+wy*wy+wz*wz)*wx)+u0*(cos(sqrt(wx*wx+wy*wy+wz*wz))/(wx*wx+wy*wy+wz*wz)*wx*wx-sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(pow(wx*wx+wy*wy+wz*wz,3.0))*wx*wx+sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(wx*wx+wy*wy+wz
                                                                                                                                                                                                                                                                                                                                                                                                                             *wz)+sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(pow(wx*wx+wy*wy+wz*wz,3.0))*wx*wz*wy-2.0*(1.0-
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              cos(sqrt(wx*wx+wy*wy+wz*wz)))/pow(wx*wx+wy*wy+wz*wz,2.0)*wz*wy*wx);
            MapleGenVar7 = MapleGenVar8+MapleGenVar9;
            MapleGenVar8 = Y;
            MapleGenVar6 = MapleGenVar7*MapleGenVar8;
            MapleGenVar4 = MapleGenVar5+MapleGenVar6;
            MapleGenVar5 = 1/((-sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(wx*wx+wy*wy+wz*wz)*wy+(1.0-
                                                                                         cos(sqrt(wx*wx+wy*wy+wz*wz)))/(wx*wx+wy*wy+wz*wz)*wz*wx)*X+(sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(wx*wx+wy*wy+wz*wz)*wx+(1.0-cos(sqrt(wx*wx+wy*wy+wz*wz)))/(wx*wx+wy*wy+wz*wz)*wz*wy)*Y+tz);
            MapleGenVar3 = MapleGenVar4*MapleGenVar5;
            MapleGenVar1 = MapleGenVar2*MapleGenVar3;
            MapleGenVar5 = (au*(1.0+(1.0-cos(sqrt(wx*wx+wy*wy+wz*wz)))/(wx*wx+wy*wy+wz*wz)*(-wz*wz-wy*wy))+sk*(sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(wx*wx+wy*wy+wz*wz)*wz+(1.0-
                                                                                                                                                                       cos(sqrt(wx*wx+wy*wy+wz*wz)))/(wx*wx+wy*wy+wz*wz)*wy*wx)+u0*(-
                                                                                                                                                                                                                                    sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(wx*wx+wy*wy+wz*wz)*wy+(1.0-
                                                                                                                                                                                                                                                                                             cos(sqrt(wx*wx+wy*wy+wz*wz)))/(wx*wx+wy*wy+wz*wz)*wz*wx))*X;
            MapleGenVar6 = (au*(-sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(wx*wx+wy*wy+wz*wz)*wz+(1.0-
                                                                                          cos(sqrt(wx*wx+wy*wy+wz*wz)))/(wx*wx+wy*wy+wz*wz)*wy*wx)+sk*(1.0+(1.0-
                                                                                                                                                            cos(sqrt(wx*wx+wy*wy+wz*wz)))/(wx*wx+wy*wy+wz*wz)*(-wz*wz-wx*wx))+u0*(sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(wx*wx+wy*wy+wz*wz)*wx+(1.0-
                                                                                                                                                                                                                                                                                          cos(sqrt(wx*wx+wy*wy+wz*wz)))/(wx*wx+wy*wy+wz*wz)*wz*wy))*Y;
            MapleGenVar4 = MapleGenVar5+MapleGenVar6;
            MapleGenVar3 = MapleGenVar4+au*tx+sk*ty+u0*tz;
            MapleGenVar5 = 1/(pow((-sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(wx*wx+wy*wy+wz*wz)*wy+(1.0-
                                                                                             cos(sqrt(wx*wx+wy*wy+wz*wz)))/(wx*wx+wy*wy+wz*wz)*wz*wx)*X+(sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(wx*wx+wy*wy+wz*wz)*wx+(1.0-cos(sqrt(wx*wx+wy*wy+wz*wz)))/(wx*wx+wy*wy+wz*wz)*wz*wy)*Y+tz,2.0));
            MapleGenVar7 = (-
                            cos(sqrt(wx*wx+wy*wy+wz*wz))/(wx*wx+wy*wy+wz*wz)*wx*wy+sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(pow(wx*wx+wy*wy+wz*wz,3.0
                                                                                                                         ))*wy*wx+sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(pow(wx*wx+wy*wy+wz*wz,3.0))*wx*wx*wz-2.0*(1.0-
                                                                                                                                                                                                              cos(sqrt(wx*wx+wy*wy+wz*wz)))/pow(wx*wx+wy*wy+wz*wz,2.0)*wz*wx*wx+(1.0-
                                                                                                                                                                                                                                                                                 cos(sqrt(wx*wx+wy*wy+wz*wz)))/(wx*wx+wy*wy+wz*wz)*wz)*X;
            MapleGenVar8 = (cos(sqrt(wx*wx+wy*wy+wz*wz))/(wx*wx+wy*wy+wz*wz)*wx*wx-sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(pow(wx*wx+wy*wy+wz*wz,3.0))*wx*wx+sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(wx*wx+wy*wy+wz
                                                                                                                                                                                        *wz)+sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(pow(wx*wx+wy*wy+wz*wz,3.0))*wx*wz*wy-2.0*(1.0-
                                                                                                                                                                                                                                                                         cos(sqrt(wx*wx+wy*wy+wz*wz)))/pow(wx*wx+wy*wy+wz*wz,2.0)*wz*wy*wx)*Y;
            MapleGenVar6 = MapleGenVar7+MapleGenVar8;
            MapleGenVar4 = MapleGenVar5*MapleGenVar6;
            MapleGenVar2 = MapleGenVar3*MapleGenVar4;
            cvmSet(J,x_y_total*j+2*i , 5+j*6 , MapleGenVar1+MapleGenVar2);
            MapleGenVar2 = -1.0;
            MapleGenVar7 = av*(cos(sqrt(wx*wx+wy*wy+wz*wz))/(wx*wx+wy*wy+wz*wz)*wx*wz-sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(pow(wx*wx+wy*wy+wz*wz,3.0))*wz*wx+sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(pow(wx*wx+wy*wy
                                                                                                                                                                                               +wz*wz,3.0))*wx*wx*wy-2.0*(1.0-cos(sqrt(wx*wx+wy*wy+wz*wz)))/pow(wx*wx+wy*wy+wz*wz,2.0)*wy*wx*wx+(1.0-
                                                                                                                                                                                                                                                                                                 cos(sqrt(wx*wx+wy*wy+wz*wz)))/(wx*wx+wy*wy+wz*wz)*wy);
            MapleGenVar8 = v0*(-
                               cos(sqrt(wx*wx+wy*wy+wz*wz))/(wx*wx+wy*wy+wz*wz)*wx*wy+sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(pow(wx*wx+wy*wy+wz*wz,3.0
                                                                                                                            ))*wy*wx+sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(pow(wx*wx+wy*wy+wz*wz,3.0))*wx*wx*wz-2.0*(1.0-
                                                                                                                                                                                                                 cos(sqrt(wx*wx+wy*wy+wz*wz)))/pow(wx*wx+wy*wy+wz*wz,2.0)*wz*wx*wx+(1.0-
                                                                                                                                                                                                                                                                                    cos(sqrt(wx*wx+wy*wy+wz*wz)))/(wx*wx+wy*wy+wz*wz)*wz);
            MapleGenVar6 = MapleGenVar7+MapleGenVar8;
            MapleGenVar7 = X;
            MapleGenVar5 = MapleGenVar6*MapleGenVar7;
            MapleGenVar6 = (av*(sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(pow(wx*wx+wy*wy+wz*wz,3.0))*wx*(-
                                                                                                  wz*wz-wx*wx)-2.0*(1.0-cos(sqrt(wx*wx+wy*wy+wz*wz)))/pow(wx*wx+wy*wy+wz*wz,2.0)*(-wz*wz-wx*wx)*wx-2.0*(1.0-
                                                                                                                                                                                                        cos(sqrt(wx*wx+wy*wy+wz*wz)))/(wx*wx+wy*wy+wz*wz)*wx)+v0*(cos(sqrt(wx*wx+wy*wy+wz*wz))/(wx*wx+wy*wy+wz*wz)*wx*wx-sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(pow(wx*wx+wy*wy+wz*wz,3.0))*wx*wx+sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(wx*wx+wy*wy+wz
                                                                                                                                                                                                                                                                                                                                                                                                                              *wz)+sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(pow(wx*wx+wy*wy+wz*wz,3.0))*wx*wz*wy-2.0*(1.0-
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               cos(sqrt(wx*wx+wy*wy+wz*wz)))/pow(wx*wx+wy*wy+wz*wz,2.0)*wz*wy*wx))*Y;
            MapleGenVar4 = MapleGenVar5+MapleGenVar6;
            MapleGenVar5 = 1/((-sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(wx*wx+wy*wy+wz*wz)*wy+(1.0-
                                                                                         cos(sqrt(wx*wx+wy*wy+wz*wz)))/(wx*wx+wy*wy+wz*wz)*wz*wx)*X+(sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(wx*wx+wy*wy+wz*wz)*wx+(1.0-cos(sqrt(wx*wx+wy*wy+wz*wz)))/(wx*wx+wy*wy+wz*wz)*wz*wy)*Y+tz);
            MapleGenVar3 = MapleGenVar4*MapleGenVar5;
            MapleGenVar1 = MapleGenVar2*MapleGenVar3;
            MapleGenVar3 = (av*(sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(wx*wx+wy*wy+wz*wz)*wz+(1.0-
                                                                                         cos(sqrt(wx*wx+wy*wy+wz*wz)))/(wx*wx+wy*wy+wz*wz)*wy*wx)+v0*(-
                                                                                                                                                      sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(wx*wx+wy*wy+wz*wz)*wy+(1.0-
                                                                                                                                                                                                               cos(sqrt(wx*wx+wy*wy+wz*wz)))/(wx*wx+wy*wy+wz*wz)*wz*wx))*X+(av*(1.0+(1.0-
                                                                                                                                                                                                                                                                                     cos(sqrt(wx*wx+wy*wy+wz*wz)))/(wx*wx+wy*wy+wz*wz)*(-wz*wz-wx*wx))+v0*(sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(wx*wx+wy*wy+wz*wz)*wx+(1.0-
                                                                                                                                                                                                                                                                                                                                                                                                                   cos(sqrt(wx*wx+wy*wy+wz*wz)))/(wx*wx+wy*wy+wz*wz)*wz*wy))*Y+av*ty+v0*tz;
            MapleGenVar5 = 1/(pow((-sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(wx*wx+wy*wy+wz*wz)*wy+(1.0-
                                                                                             cos(sqrt(wx*wx+wy*wy+wz*wz)))/(wx*wx+wy*wy+wz*wz)*wz*wx)*X+(sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(wx*wx+wy*wy+wz*wz)*wx+(1.0-cos(sqrt(wx*wx+wy*wy+wz*wz)))/(wx*wx+wy*wy+wz*wz)*wz*wy)*Y+tz,2.0));
            MapleGenVar7 = (-
                            cos(sqrt(wx*wx+wy*wy+wz*wz))/(wx*wx+wy*wy+wz*wz)*wx*wy+sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(pow(wx*wx+wy*wy+wz*wz,3.0
                                                                                                                         ))*wy*wx+sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(pow(wx*wx+wy*wy+wz*wz,3.0))*wx*wx*wz-2.0*(1.0-
                                                                                                                                                                                                              cos(sqrt(wx*wx+wy*wy+wz*wz)))/pow(wx*wx+wy*wy+wz*wz,2.0)*wz*wx*wx+(1.0-
                                                                                                                                                                                                                                                                                 cos(sqrt(wx*wx+wy*wy+wz*wz)))/(wx*wx+wy*wy+wz*wz)*wz)*X;
            MapleGenVar8 = (cos(sqrt(wx*wx+wy*wy+wz*wz))/(wx*wx+wy*wy+wz*wz)*wx*wx-sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(pow(wx*wx+wy*wy+wz*wz,3.0))*wx*wx+sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(wx*wx+wy*wy+wz
                                                                                                                                                                                        *wz)+sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(pow(wx*wx+wy*wy+wz*wz,3.0))*wx*wz*wy-2.0*(1.0-
                                                                                                                                                                                                                                                                         cos(sqrt(wx*wx+wy*wy+wz*wz)))/pow(wx*wx+wy*wy+wz*wz,2.0)*wz*wy*wx)*Y;
            MapleGenVar6 = MapleGenVar7+MapleGenVar8;
            MapleGenVar4 = MapleGenVar5*MapleGenVar6;
            MapleGenVar2 = MapleGenVar3*MapleGenVar4;
            cvmSet(J,x_y_total*j+2*i+1 , 5+j*6 , MapleGenVar1+MapleGenVar2);
            
            MapleGenVar2 = -1.0;
            MapleGenVar7 = au*(sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(pow(wx*wx+wy*wy+wz*wz,3.0))*wy*(-
                                                                                                 wz*wz-wy*wy)-2.0*(1.0-cos(sqrt(wx*wx+wy*wy+wz*wz)))/pow(wx*wx+wy*wy+wz*wz,2.0)*(-wz*wz-wy*wy)*wy-2.0*(1.0-
                                                                                                                                                                                                       cos(sqrt(wx*wx+wy*wy+wz*wz)))/(wx*wx+wy*wy+wz*wz)*wy);
            MapleGenVar9 = sk*(cos(sqrt(wx*wx+wy*wy+wz*wz))/(wx*wx+wy*wy+wz*wz)*wy*wz-sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(pow(wx*wx+wy*wy+wz*wz,3.0))*wz*wy+sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(pow(wx*wx+wy*wy
                                                                                                                                                                                               +wz*wz,3.0))*wy*wy*wx-2.0*(1.0-cos(sqrt(wx*wx+wy*wy+wz*wz)))/pow(wx*wx+wy*wy+wz*wz,2.0)*wy*wy*wx+(1.0-
                                                                                                                                                                                                                                                                                                 cos(sqrt(wx*wx+wy*wy+wz*wz)))/(wx*wx+wy*wy+wz*wz)*wx);
            MapleGenVar10 = u0*(-
                                cos(sqrt(wx*wx+wy*wy+wz*wz))/(wx*wx+wy*wy+wz*wz)*wy*wy+sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(pow(wx*wx+wy*wy+wz*wz,3.0
                                                                                                                             ))*wy*wy-sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(wx*wx+wy*wy+wz*wz)+sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(pow(wx*wx+wy*wy+wz*wz,3.0))*wx
                                *wz*wy-2.0*(1.0-cos(sqrt(wx*wx+wy*wy+wz*wz)))/pow(wx*wx+wy*wy+wz*wz,2.0)*wz*wy*wx);
            MapleGenVar8 = MapleGenVar9+MapleGenVar10;
            MapleGenVar6 = MapleGenVar7+MapleGenVar8;
            MapleGenVar7 = X;
            MapleGenVar5 = MapleGenVar6*MapleGenVar7;
            MapleGenVar8 = au*(-
                               cos(sqrt(wx*wx+wy*wy+wz*wz))/(wx*wx+wy*wy+wz*wz)*wy*wz+sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(pow(wx*wx+wy*wy+wz*wz,3.0)
                                                                                                                        )*wz*wy+sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(pow(wx*wx+wy*wy+wz*wz,3.0))*wy*wy*wx-2.0*(1.0-
                                                                                                                                                                                                            cos(sqrt(wx*wx+wy*wy+wz*wz)))/pow(wx*wx+wy*wy+wz*wz,2.0)*wy*wy*wx+(1.0-
                                                                                                                                                                                                                                                                               cos(sqrt(wx*wx+wy*wy+wz*wz)))/(wx*wx+wy*wy+wz*wz)*wx);
            MapleGenVar9 = sk*(sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(pow(wx*wx+wy*wy+wz*wz,3.0))*wy*(-
                                                                                                 wz*wz-wx*wx)-2.0*(1.0-cos(sqrt(wx*wx+wy*wy+wz*wz)))/pow(wx*wx+wy*wy+wz*wz,2.0)*(-wz*wz-wx*wx)*wy)+u0*(cos(sqrt(wx*wx+wy*wy+wz*wz))/(wx*wx+wy*wy+wz*wz)*wx*wy-sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(pow(wx*wx+wy*wy+wz*wz,3.0))*wy*wx+sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(pow(wx*wx+wy*wy+wz*wz,3.0))*wy*wy*wz-2.0*(1.0-cos(sqrt(wx*wx+wy*wy+wz*wz)))/pow(wx*wx+wy*wy+wz*wz,2.0)*wz*wy*wy+(1.0-
                                                                                                                                                                                                                                                                                                                                                                                                                                                                         cos(sqrt(wx*wx+wy*wy+wz*wz)))/(wx*wx+wy*wy+wz*wz)*wz);
            MapleGenVar7 = MapleGenVar8+MapleGenVar9;
            MapleGenVar8 = Y;
            MapleGenVar6 = MapleGenVar7*MapleGenVar8;
            MapleGenVar4 = MapleGenVar5+MapleGenVar6;
            MapleGenVar5 = 1/((-sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(wx*wx+wy*wy+wz*wz)*wy+(1.0-
                                                                                         cos(sqrt(wx*wx+wy*wy+wz*wz)))/(wx*wx+wy*wy+wz*wz)*wz*wx)*X+(sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(wx*wx+wy*wy+wz*wz)*wx+(1.0-cos(sqrt(wx*wx+wy*wy+wz*wz)))/(wx*wx+wy*wy+wz*wz)*wz*wy)*Y+tz);
            MapleGenVar3 = MapleGenVar4*MapleGenVar5;
            MapleGenVar1 = MapleGenVar2*MapleGenVar3;
            MapleGenVar5 = (au*(1.0+(1.0-cos(sqrt(wx*wx+wy*wy+wz*wz)))/(wx*wx+wy*wy+wz*wz)*(-wz*wz-wy*wy))+sk*(sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(wx*wx+wy*wy+wz*wz)*wz+(1.0-
                                                                                                                                                                       cos(sqrt(wx*wx+wy*wy+wz*wz)))/(wx*wx+wy*wy+wz*wz)*wy*wx)+u0*(-
                                                                                                                                                                                                                                    sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(wx*wx+wy*wy+wz*wz)*wy+(1.0-
                                                                                                                                                                                                                                                                                             cos(sqrt(wx*wx+wy*wy+wz*wz)))/(wx*wx+wy*wy+wz*wz)*wz*wx))*X;
            MapleGenVar6 = (au*(-sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(wx*wx+wy*wy+wz*wz)*wz+(1.0-
                                                                                          cos(sqrt(wx*wx+wy*wy+wz*wz)))/(wx*wx+wy*wy+wz*wz)*wy*wx)+sk*(1.0+(1.0-
                                                                                                                                                            cos(sqrt(wx*wx+wy*wy+wz*wz)))/(wx*wx+wy*wy+wz*wz)*(-wz*wz-wx*wx))+u0*(sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(wx*wx+wy*wy+wz*wz)*wx+(1.0-
                                                                                                                                                                                                                                                                                          cos(sqrt(wx*wx+wy*wy+wz*wz)))/(wx*wx+wy*wy+wz*wz)*wz*wy))*Y;
            MapleGenVar4 = MapleGenVar5+MapleGenVar6;
            MapleGenVar3 = MapleGenVar4+au*tx+sk*ty+u0*tz;
            MapleGenVar5 = 1/(pow((-sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(wx*wx+wy*wy+wz*wz)*wy+(1.0-
                                                                                             cos(sqrt(wx*wx+wy*wy+wz*wz)))/(wx*wx+wy*wy+wz*wz)*wz*wx)*X+(sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(wx*wx+wy*wy+wz*wz)*wx+(1.0-cos(sqrt(wx*wx+wy*wy+wz*wz)))/(wx*wx+wy*wy+wz*wz)*wz*wy)*Y+tz,2.0));
            MapleGenVar7 = (-
                            cos(sqrt(wx*wx+wy*wy+wz*wz))/(wx*wx+wy*wy+wz*wz)*wy*wy+sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(pow(wx*wx+wy*wy+wz*wz,3.0
                                                                                                                         ))*wy*wy-sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(wx*wx+wy*wy+wz*wz)+sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(pow(wx*wx+wy*wy+wz*wz,3.0))*wx
                            *wz*wy-2.0*(1.0-cos(sqrt(wx*wx+wy*wy+wz*wz)))/pow(wx*wx+wy*wy+wz*wz,2.0)*wz*wy*wx)*X;
            MapleGenVar8 = (cos(sqrt(wx*wx+wy*wy+wz*wz))/(wx*wx+wy*wy+wz*wz)*wx*wy-sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(pow(wx*wx+wy*wy+wz*wz,3.0))*wy*wx+sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(pow(wx*wx+wy*wy+wz*wz,3.0))*wy*wy*wz-2.0*(1.0-cos(sqrt(wx*wx+wy*wy+wz*wz)))/pow(wx*wx+wy*wy+wz*wz,2.0)*wz*wy*wy+(1.0-
                                                                                                                                                                                                                                                                                               cos(sqrt(wx*wx+wy*wy+wz*wz)))/(wx*wx+wy*wy+wz*wz)*wz)*Y;
            MapleGenVar6 = MapleGenVar7+MapleGenVar8;
            MapleGenVar4 = MapleGenVar5*MapleGenVar6;
            MapleGenVar2 = MapleGenVar3*MapleGenVar4;
            cvmSet(J,x_y_total*j+2*i , 5+j*6+1 , MapleGenVar1+MapleGenVar2);
            
            MapleGenVar2 = -1.0;
            MapleGenVar7 = av*(cos(sqrt(wx*wx+wy*wy+wz*wz))/(wx*wx+wy*wy+wz*wz)*wy*wz-sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(pow(wx*wx+wy*wy+wz*wz,3.0))*wz*wy+sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(pow(wx*wx+wy*wy
                                                                                                                                                                                               +wz*wz,3.0))*wy*wy*wx-2.0*(1.0-cos(sqrt(wx*wx+wy*wy+wz*wz)))/pow(wx*wx+wy*wy+wz*wz,2.0)*wy*wy*wx+(1.0-
                                                                                                                                                                                                                                                                                                 cos(sqrt(wx*wx+wy*wy+wz*wz)))/(wx*wx+wy*wy+wz*wz)*wx);
            MapleGenVar8 = v0*(-
                               cos(sqrt(wx*wx+wy*wy+wz*wz))/(wx*wx+wy*wy+wz*wz)*wy*wy+sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(pow(wx*wx+wy*wy+wz*wz,3.0
                                                                                                                            ))*wy*wy-sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(wx*wx+wy*wy+wz*wz)+sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(pow(wx*wx+wy*wy+wz*wz,3.0))*wx
                               *wz*wy-2.0*(1.0-cos(sqrt(wx*wx+wy*wy+wz*wz)))/pow(wx*wx+wy*wy+wz*wz,2.0)*wz*wy*wx);
            MapleGenVar6 = MapleGenVar7+MapleGenVar8;
            MapleGenVar7 = X;
            MapleGenVar5 = MapleGenVar6*MapleGenVar7;
            MapleGenVar6 = (av*(sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(pow(wx*wx+wy*wy+wz*wz,3.0))*wy*(-
                                                                                                  wz*wz-wx*wx)-2.0*(1.0-cos(sqrt(wx*wx+wy*wy+wz*wz)))/pow(wx*wx+wy*wy+wz*wz,2.0)*(-wz*wz-wx*wx)*wy)+v0*(cos(sqrt(wx*wx+wy*wy+wz*wz))/(wx*wx+wy*wy+wz*wz)*wx*wy-sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(pow(wx*wx+wy*wy+wz*wz,3.0))*wy*wx+sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(pow(wx*wx+wy*wy+wz*wz,3.0))*wy*wy*wz-2.0*(1.0-cos(sqrt(wx*wx+wy*wy+wz*wz)))/pow(wx*wx+wy*wy+wz*wz,2.0)*wz*wy*wy+(1.0-
                                                                                                                                                                                                                                                                                                                                                                                                                                                                          cos(sqrt(wx*wx+wy*wy+wz*wz)))/(wx*wx+wy*wy+wz*wz)*wz))*Y;
            MapleGenVar4 = MapleGenVar5+MapleGenVar6;
            MapleGenVar5 = 1/((-sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(wx*wx+wy*wy+wz*wz)*wy+(1.0-
                                                                                         cos(sqrt(wx*wx+wy*wy+wz*wz)))/(wx*wx+wy*wy+wz*wz)*wz*wx)*X+(sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(wx*wx+wy*wy+wz*wz)*wx+(1.0-cos(sqrt(wx*wx+wy*wy+wz*wz)))/(wx*wx+wy*wy+wz*wz)*wz*wy)*Y+tz);
            MapleGenVar3 = MapleGenVar4*MapleGenVar5;
            MapleGenVar1 = MapleGenVar2*MapleGenVar3;
            MapleGenVar3 = (av*(sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(wx*wx+wy*wy+wz*wz)*wz+(1.0-
                                                                                         cos(sqrt(wx*wx+wy*wy+wz*wz)))/(wx*wx+wy*wy+wz*wz)*wy*wx)+v0*(-
                                                                                                                                                      sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(wx*wx+wy*wy+wz*wz)*wy+(1.0-
                                                                                                                                                                                                               cos(sqrt(wx*wx+wy*wy+wz*wz)))/(wx*wx+wy*wy+wz*wz)*wz*wx))*X+(av*(1.0+(1.0-
                                                                                                                                                                                                                                                                                     cos(sqrt(wx*wx+wy*wy+wz*wz)))/(wx*wx+wy*wy+wz*wz)*(-wz*wz-wx*wx))+v0*(sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(wx*wx+wy*wy+wz*wz)*wx+(1.0-
                                                                                                                                                                                                                                                                                                                                                                                                                   cos(sqrt(wx*wx+wy*wy+wz*wz)))/(wx*wx+wy*wy+wz*wz)*wz*wy))*Y+av*ty+v0*tz;
            MapleGenVar5 = 1/(pow((-sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(wx*wx+wy*wy+wz*wz)*wy+(1.0-
                                                                                             cos(sqrt(wx*wx+wy*wy+wz*wz)))/(wx*wx+wy*wy+wz*wz)*wz*wx)*X+(sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(wx*wx+wy*wy+wz*wz)*wx+(1.0-cos(sqrt(wx*wx+wy*wy+wz*wz)))/(wx*wx+wy*wy+wz*wz)*wz*wy)*Y+tz,2.0));
            MapleGenVar7 = (-
                            cos(sqrt(wx*wx+wy*wy+wz*wz))/(wx*wx+wy*wy+wz*wz)*wy*wy+sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(pow(wx*wx+wy*wy+wz*wz,3.0
                                                                                                                         ))*wy*wy-sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(wx*wx+wy*wy+wz*wz)+sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(pow(wx*wx+wy*wy+wz*wz,3.0))*wx
                            *wz*wy-2.0*(1.0-cos(sqrt(wx*wx+wy*wy+wz*wz)))/pow(wx*wx+wy*wy+wz*wz,2.0)*wz*wy*wx)*X;
            MapleGenVar8 = (cos(sqrt(wx*wx+wy*wy+wz*wz))/(wx*wx+wy*wy+wz*wz)*wx*wy-sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(pow(wx*wx+wy*wy+wz*wz,3.0))*wy*wx+sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(pow(wx*wx+wy*wy+wz*wz,3.0))*wy*wy*wz-2.0*(1.0-cos(sqrt(wx*wx+wy*wy+wz*wz)))/pow(wx*wx+wy*wy+wz*wz,2.0)*wz*wy*wy+(1.0-
                                                                                                                                                                                                                                                                                               cos(sqrt(wx*wx+wy*wy+wz*wz)))/(wx*wx+wy*wy+wz*wz)*wz)*Y;
            MapleGenVar6 = MapleGenVar7+MapleGenVar8;
            MapleGenVar4 = MapleGenVar5*MapleGenVar6;
            MapleGenVar2 = MapleGenVar3*MapleGenVar4;
            cvmSet(J,x_y_total*j+2*i+1 , 5+j*6+1 , MapleGenVar1+MapleGenVar2);
            
            MapleGenVar2 = -1.0;
            MapleGenVar7 = au*(sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(pow(wx*wx+wy*wy+wz*wz,3.0))*wz*(-
                                                                                                 wz*wz-wy*wy)-2.0*(1.0-cos(sqrt(wx*wx+wy*wy+wz*wz)))/pow(wx*wx+wy*wy+wz*wz,2.0)*(-wz*wz-wy*wy)*wz-2.0*(1.0-
                                                                                                                                                                                                       cos(sqrt(wx*wx+wy*wy+wz*wz)))/(wx*wx+wy*wy+wz*wz)*wz);
            MapleGenVar9 = sk*(cos(sqrt(wx*wx+wy*wy+wz*wz))/(wx*wx+wy*wy+wz*wz)*wz*wz-sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(pow(wx*wx+wy*wy+wz*wz,3.0))*wz*wz+sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(wx*wx+wy*wy+wz
                                                                                                                                                                                           *wz)+sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(pow(wx*wx+wy*wy+wz*wz,3.0))*wx*wz*wy-2.0*(1.0-
                                                                                                                                                                                                                                                                            cos(sqrt(wx*wx+wy*wy+wz*wz)))/pow(wx*wx+wy*wy+wz*wz,2.0)*wz*wy*wx);
            MapleGenVar10 = u0*(-
                                cos(sqrt(wx*wx+wy*wy+wz*wz))/(wx*wx+wy*wy+wz*wz)*wy*wz+sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(pow(wx*wx+wy*wy+wz*wz,3.0)
                                                                                                                         )*wz*wy+sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(pow(wx*wx+wy*wy+wz*wz,3.0))*wz*wz*wx-2.0*(1.0-
                                                                                                                                                                                                             cos(sqrt(wx*wx+wy*wy+wz*wz)))/pow(wx*wx+wy*wy+wz*wz,2.0)*wz*wz*wx+(1.0-
                                                                                                                                                                                                                                                                                cos(sqrt(wx*wx+wy*wy+wz*wz)))/(wx*wx+wy*wy+wz*wz)*wx);
            MapleGenVar8 = MapleGenVar9+MapleGenVar10;
            MapleGenVar6 = MapleGenVar7+MapleGenVar8;
            MapleGenVar7 = X;
            MapleGenVar5 = MapleGenVar6*MapleGenVar7;
            MapleGenVar8 = au*(-
                               cos(sqrt(wx*wx+wy*wy+wz*wz))/(wx*wx+wy*wy+wz*wz)*wz*wz+sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(pow(wx*wx+wy*wy+wz*wz,3.0)
                                                                                                                        )*wz*wz-sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(wx*wx+wy*wy+wz*wz)+sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(pow(wx*wx+wy*wy+wz*wz,3.0))*wx
                               *wz*wy-2.0*(1.0-cos(sqrt(wx*wx+wy*wy+wz*wz)))/pow(wx*wx+wy*wy+wz*wz,2.0)*wz*wy*wx);
            MapleGenVar9 = sk*(sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(pow(wx*wx+wy*wy+wz*wz,3.0))*wz*(-
                                                                                                 wz*wz-wx*wx)-2.0*(1.0-cos(sqrt(wx*wx+wy*wy+wz*wz)))/pow(wx*wx+wy*wy+wz*wz,2.0)*(-wz*wz-wx*wx)*wz-2.0*(1.0-
                                                                                                                                                                                                       cos(sqrt(wx*wx+wy*wy+wz*wz)))/(wx*wx+wy*wy+wz*wz)*wz)+u0*(cos(sqrt(wx*wx+wy*wy+wz*wz))/(wx*wx+wy*wy+wz*wz)*wx*wz-sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(pow(wx*wx+wy*wy+wz*wz,3.0))*wz*wx+sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(pow(wx*wx+wy*wy
                                                                                                                                                                                                                                                                                                                                                                                                                                 +wz*wz,3.0))*wz*wz*wy-2.0*(1.0-cos(sqrt(wx*wx+wy*wy+wz*wz)))/pow(wx*wx+wy*wy+wz*wz,2.0)*wz*wz*wy+(1.0-
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   cos(sqrt(wx*wx+wy*wy+wz*wz)))/(wx*wx+wy*wy+wz*wz)*wy);
            MapleGenVar7 = MapleGenVar8+MapleGenVar9;
            MapleGenVar8 = Y;
            MapleGenVar6 = MapleGenVar7*MapleGenVar8;
            MapleGenVar4 = MapleGenVar5+MapleGenVar6;
            MapleGenVar5 = 1/((-sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(wx*wx+wy*wy+wz*wz)*wy+(1.0-
                                                                                         cos(sqrt(wx*wx+wy*wy+wz*wz)))/(wx*wx+wy*wy+wz*wz)*wz*wx)*X+(sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(wx*wx+wy*wy+wz*wz)*wx+(1.0-cos(sqrt(wx*wx+wy*wy+wz*wz)))/(wx*wx+wy*wy+wz*wz)*wz*wy)*Y+tz);
            MapleGenVar3 = MapleGenVar4*MapleGenVar5;
            MapleGenVar1 = MapleGenVar2*MapleGenVar3;
            MapleGenVar5 = (au*(1.0+(1.0-cos(sqrt(wx*wx+wy*wy+wz*wz)))/(wx*wx+wy*wy+wz*wz)*(-wz*wz-wy*wy))+sk*(sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(wx*wx+wy*wy+wz*wz)*wz+(1.0-
                                                                                                                                                                       cos(sqrt(wx*wx+wy*wy+wz*wz)))/(wx*wx+wy*wy+wz*wz)*wy*wx)+u0*(-
                                                                                                                                                                                                                                    sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(wx*wx+wy*wy+wz*wz)*wy+(1.0-
                                                                                                                                                                                                                                                                                             cos(sqrt(wx*wx+wy*wy+wz*wz)))/(wx*wx+wy*wy+wz*wz)*wz*wx))*X;
            MapleGenVar6 = (au*(-sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(wx*wx+wy*wy+wz*wz)*wz+(1.0-
                                                                                          cos(sqrt(wx*wx+wy*wy+wz*wz)))/(wx*wx+wy*wy+wz*wz)*wy*wx)+sk*(1.0+(1.0-
                                                                                                                                                            cos(sqrt(wx*wx+wy*wy+wz*wz)))/(wx*wx+wy*wy+wz*wz)*(-wz*wz-wx*wx))+u0*(sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(wx*wx+wy*wy+wz*wz)*wx+(1.0-
                                                                                                                                                                                                                                                                                          cos(sqrt(wx*wx+wy*wy+wz*wz)))/(wx*wx+wy*wy+wz*wz)*wz*wy))*Y;
            MapleGenVar4 = MapleGenVar5+MapleGenVar6;
            MapleGenVar3 = MapleGenVar4+au*tx+sk*ty+u0*tz;
            MapleGenVar5 = 1/(pow((-sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(wx*wx+wy*wy+wz*wz)*wy+(1.0-
                                                                                             cos(sqrt(wx*wx+wy*wy+wz*wz)))/(wx*wx+wy*wy+wz*wz)*wz*wx)*X+(sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(wx*wx+wy*wy+wz*wz)*wx+(1.0-cos(sqrt(wx*wx+wy*wy+wz*wz)))/(wx*wx+wy*wy+wz*wz)*wz*wy)*Y+tz,2.0));
            MapleGenVar7 = (-
                            cos(sqrt(wx*wx+wy*wy+wz*wz))/(wx*wx+wy*wy+wz*wz)*wy*wz+sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(pow(wx*wx+wy*wy+wz*wz,3.0)
                                                                                                                     )*wz*wy+sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(pow(wx*wx+wy*wy+wz*wz,3.0))*wz*wz*wx-2.0*(1.0-
                                                                                                                                                                                                         cos(sqrt(wx*wx+wy*wy+wz*wz)))/pow(wx*wx+wy*wy+wz*wz,2.0)*wz*wz*wx+(1.0-
                                                                                                                                                                                                                                                                            cos(sqrt(wx*wx+wy*wy+wz*wz)))/(wx*wx+wy*wy+wz*wz)*wx)*X;
            MapleGenVar8 = (cos(sqrt(wx*wx+wy*wy+wz*wz))/(wx*wx+wy*wy+wz*wz)*wx*wz-sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(pow(wx*wx+wy*wy+wz*wz,3.0))*wz*wx+sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(pow(wx*wx+wy*wy
                                                                                                                                                                                            +wz*wz,3.0))*wz*wz*wy-2.0*(1.0-cos(sqrt(wx*wx+wy*wy+wz*wz)))/pow(wx*wx+wy*wy+wz*wz,2.0)*wz*wz*wy+(1.0-
                                                                                                                                                                                                                                                                                              cos(sqrt(wx*wx+wy*wy+wz*wz)))/(wx*wx+wy*wy+wz*wz)*wy)*Y;
            MapleGenVar6 = MapleGenVar7+MapleGenVar8;
            MapleGenVar4 = MapleGenVar5*MapleGenVar6;
            MapleGenVar2 = MapleGenVar3*MapleGenVar4;
            cvmSet(J,x_y_total*j+2*i , 5+j*6+2 , MapleGenVar1+MapleGenVar2);
            
            MapleGenVar2 = -1.0;
            MapleGenVar7 = av*(cos(sqrt(wx*wx+wy*wy+wz*wz))/(wx*wx+wy*wy+wz*wz)*wz*wz-sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(pow(wx*wx+wy*wy+wz*wz,3.0))*wz*wz+sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(wx*wx+wy*wy+wz
                                                                                                                                                                                           *wz)+sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(pow(wx*wx+wy*wy+wz*wz,3.0))*wx*wz*wy-2.0*(1.0-
                                                                                                                                                                                                                                                                            cos(sqrt(wx*wx+wy*wy+wz*wz)))/pow(wx*wx+wy*wy+wz*wz,2.0)*wz*wy*wx);
            MapleGenVar8 = v0*(-
                               cos(sqrt(wx*wx+wy*wy+wz*wz))/(wx*wx+wy*wy+wz*wz)*wy*wz+sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(pow(wx*wx+wy*wy+wz*wz,3.0)
                                                                                                                        )*wz*wy+sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(pow(wx*wx+wy*wy+wz*wz,3.0))*wz*wz*wx-2.0*(1.0-
                                                                                                                                                                                                            cos(sqrt(wx*wx+wy*wy+wz*wz)))/pow(wx*wx+wy*wy+wz*wz,2.0)*wz*wz*wx+(1.0-
                                                                                                                                                                                                                                                                               cos(sqrt(wx*wx+wy*wy+wz*wz)))/(wx*wx+wy*wy+wz*wz)*wx);
            MapleGenVar6 = MapleGenVar7+MapleGenVar8;
            MapleGenVar7 = X;
            MapleGenVar5 = MapleGenVar6*MapleGenVar7;
            MapleGenVar6 = (av*(sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(pow(wx*wx+wy*wy+wz*wz,3.0))*wz*(-
                                                                                                  wz*wz-wx*wx)-2.0*(1.0-cos(sqrt(wx*wx+wy*wy+wz*wz)))/pow(wx*wx+wy*wy+wz*wz,2.0)*(-wz*wz-wx*wx)*wz-2.0*(1.0-
                                                                                                                                                                                                        cos(sqrt(wx*wx+wy*wy+wz*wz)))/(wx*wx+wy*wy+wz*wz)*wz)+v0*(cos(sqrt(wx*wx+wy*wy+wz*wz))/(wx*wx+wy*wy+wz*wz)*wx*wz-sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(pow(wx*wx+wy*wy+wz*wz,3.0))*wz*wx+sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(pow(wx*wx+wy*wy
                                                                                                                                                                                                                                                                                                                                                                                                                                  +wz*wz,3.0))*wz*wz*wy-2.0*(1.0-cos(sqrt(wx*wx+wy*wy+wz*wz)))/pow(wx*wx+wy*wy+wz*wz,2.0)*wz*wz*wy+(1.0-
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    cos(sqrt(wx*wx+wy*wy+wz*wz)))/(wx*wx+wy*wy+wz*wz)*wy))*Y;
            MapleGenVar4 = MapleGenVar5+MapleGenVar6;
            MapleGenVar5 = 1/((-sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(wx*wx+wy*wy+wz*wz)*wy+(1.0-
                                                                                         cos(sqrt(wx*wx+wy*wy+wz*wz)))/(wx*wx+wy*wy+wz*wz)*wz*wx)*X+(sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(wx*wx+wy*wy+wz*wz)*wx+(1.0-cos(sqrt(wx*wx+wy*wy+wz*wz)))/(wx*wx+wy*wy+wz*wz)*wz*wy)*Y+tz);
            MapleGenVar3 = MapleGenVar4*MapleGenVar5;
            MapleGenVar1 = MapleGenVar2*MapleGenVar3;
            MapleGenVar3 = (av*(sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(wx*wx+wy*wy+wz*wz)*wz+(1.0-
                                                                                         cos(sqrt(wx*wx+wy*wy+wz*wz)))/(wx*wx+wy*wy+wz*wz)*wy*wx)+v0*(-
                                                                                                                                                      sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(wx*wx+wy*wy+wz*wz)*wy+(1.0-
                                                                                                                                                                                                               cos(sqrt(wx*wx+wy*wy+wz*wz)))/(wx*wx+wy*wy+wz*wz)*wz*wx))*X+(av*(1.0+(1.0-
                                                                                                                                                                                                                                                                                     cos(sqrt(wx*wx+wy*wy+wz*wz)))/(wx*wx+wy*wy+wz*wz)*(-wz*wz-wx*wx))+v0*(sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(wx*wx+wy*wy+wz*wz)*wx+(1.0-
                                                                                                                                                                                                                                                                                                                                                                                                                   cos(sqrt(wx*wx+wy*wy+wz*wz)))/(wx*wx+wy*wy+wz*wz)*wz*wy))*Y+av*ty+v0*tz;
            MapleGenVar5 = 1/(pow((-sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(wx*wx+wy*wy+wz*wz)*wy+(1.0-
                                                                                             cos(sqrt(wx*wx+wy*wy+wz*wz)))/(wx*wx+wy*wy+wz*wz)*wz*wx)*X+(sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(wx*wx+wy*wy+wz*wz)*wx+(1.0-cos(sqrt(wx*wx+wy*wy+wz*wz)))/(wx*wx+wy*wy+wz*wz)*wz*wy)*Y+tz,2.0));
            MapleGenVar7 = (-
                            cos(sqrt(wx*wx+wy*wy+wz*wz))/(wx*wx+wy*wy+wz*wz)*wy*wz+sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(pow(wx*wx+wy*wy+wz*wz,3.0)
                                                                                                                     )*wz*wy+sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(pow(wx*wx+wy*wy+wz*wz,3.0))*wz*wz*wx-2.0*(1.0-
                                                                                                                                                                                                         cos(sqrt(wx*wx+wy*wy+wz*wz)))/pow(wx*wx+wy*wy+wz*wz,2.0)*wz*wz*wx+(1.0-
                                                                                                                                                                                                                                                                            cos(sqrt(wx*wx+wy*wy+wz*wz)))/(wx*wx+wy*wy+wz*wz)*wx)*X;
            MapleGenVar8 = (cos(sqrt(wx*wx+wy*wy+wz*wz))/(wx*wx+wy*wy+wz*wz)*wx*wz-sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(pow(wx*wx+wy*wy+wz*wz,3.0))*wz*wx+sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(pow(wx*wx+wy*wy
                                                                                                                                                                                            +wz*wz,3.0))*wz*wz*wy-2.0*(1.0-cos(sqrt(wx*wx+wy*wy+wz*wz)))/pow(wx*wx+wy*wy+wz*wz,2.0)*wz*wz*wy+(1.0-
                                                                                                                                                                                                                                                                                              cos(sqrt(wx*wx+wy*wy+wz*wz)))/(wx*wx+wy*wy+wz*wz)*wy)*Y;
            MapleGenVar6 = MapleGenVar7+MapleGenVar8;
            MapleGenVar4 = MapleGenVar5*MapleGenVar6;
            MapleGenVar2 = MapleGenVar3*MapleGenVar4;
            cvmSet(J,x_y_total*j+2*i+1 , 5+j*6+2 , MapleGenVar1+MapleGenVar2);
            
            cvmSet(J,x_y_total*j+2*i , 5+j*6+3 , -au/((-
                                                 sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(wx*wx+wy*wy+wz*wz)*wy+(1.0-
                                                                                                          cos(sqrt(wx*wx+wy*wy+wz*wz)))/(wx*wx+wy*wy+wz*wz)*wz*wx)*X+(sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(wx*wx+wy*wy+wz*wz)*wx+(1.0-cos(sqrt(wx*wx+wy*wy+wz*wz)))/(wx*wx+wy*wy+wz*wz)*wz*wy)*Y+tz));
            cvmSet(J,x_y_total*j+2*i+1 , 5+j*6+3 , 0);
            cvmSet(J,x_y_total*j+2*i , 5+j*6+4 , -sk/((-
                                                 sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(wx*wx+wy*wy+wz*wz)*wy+(1.0-
                                                                                                          cos(sqrt(wx*wx+wy*wy+wz*wz)))/(wx*wx+wy*wy+wz*wz)*wz*wx)*X+(sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(wx*wx+wy*wy+wz*wz)*wx+(1.0-cos(sqrt(wx*wx+wy*wy+wz*wz)))/(wx*wx+wy*wy+wz*wz)*wz*wy)*Y+tz));
            cvmSet(J,x_y_total*j+2*i+1 , 5+j*6+4 , -av/((-
                                                   sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(wx*wx+wy*wy+wz*wz)*wy+(1.0-
                                                                                                            cos(sqrt(wx*wx+wy*wy+wz*wz)))/(wx*wx+wy*wy+wz*wz)*wz*wx)*X+(sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(wx*wx+wy*wy+wz*wz)*wx+(1.0-cos(sqrt(wx*wx+wy*wy+wz*wz)))/(wx*wx+wy*wy+wz*wz)*wz*wy)*Y+tz));
            
            MapleGenVar1 = -u0/((-sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(wx*wx+wy*wy+wz*wz)*wy+(1.0-
                                                                                           cos(sqrt(wx*wx+wy*wy+wz*wz)))/(wx*wx+wy*wy+wz*wz)*wz*wx)*X+(sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(wx*wx+wy*wy+wz*wz)*wx+(1.0-cos(sqrt(wx*wx+wy*wy+wz*wz)))/(wx*wx+wy*wy+wz*wz)*wz*wy)*Y+tz);
            MapleGenVar5 = (au*(1.0+(1.0-cos(sqrt(wx*wx+wy*wy+wz*wz)))/(wx*wx+wy*wy+wz*wz)*(-wz*wz-wy*wy))+sk*(sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(wx*wx+wy*wy+wz*wz)*wz+(1.0-
                                                                                                                                                                       cos(sqrt(wx*wx+wy*wy+wz*wz)))/(wx*wx+wy*wy+wz*wz)*wy*wx)+u0*(-
                                                                                                                                                                                                                                    sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(wx*wx+wy*wy+wz*wz)*wy+(1.0-
                                                                                                                                                                                                                                                                                             cos(sqrt(wx*wx+wy*wy+wz*wz)))/(wx*wx+wy*wy+wz*wz)*wz*wx))*X;
            MapleGenVar6 = (au*(-sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(wx*wx+wy*wy+wz*wz)*wz+(1.0-
                                                                                          cos(sqrt(wx*wx+wy*wy+wz*wz)))/(wx*wx+wy*wy+wz*wz)*wy*wx)+sk*(1.0+(1.0-
                                                                                                                                                            cos(sqrt(wx*wx+wy*wy+wz*wz)))/(wx*wx+wy*wy+wz*wz)*(-wz*wz-wx*wx))+u0*(sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(wx*wx+wy*wy+wz*wz)*wx+(1.0-
                                                                                                                                                                                                                                                                                          cos(sqrt(wx*wx+wy*wy+wz*wz)))/(wx*wx+wy*wy+wz*wz)*wz*wy))*Y;
            MapleGenVar4 = MapleGenVar5+MapleGenVar6;
            MapleGenVar3 = MapleGenVar4+au*tx+sk*ty+u0*tz;
            MapleGenVar4 = 1/(pow((-sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(wx*wx+wy*wy+wz*wz)*wy+(1.0-
                                                                                             cos(sqrt(wx*wx+wy*wy+wz*wz)))/(wx*wx+wy*wy+wz*wz)*wz*wx)*X+(sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(wx*wx+wy*wy+wz*wz)*wx+(1.0-cos(sqrt(wx*wx+wy*wy+wz*wz)))/(wx*wx+wy*wy+wz*wz)*wz*wy)*Y+tz,2.0));
            MapleGenVar2 = MapleGenVar3*MapleGenVar4;
            cvmSet(J,x_y_total*j+2*i , 5+j*6+5, MapleGenVar1+MapleGenVar2);
            MapleGenVar1 = -v0/((-sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(wx*wx+wy*wy+wz*wz)*wy+(1.0-
                                                                                           cos(sqrt(wx*wx+wy*wy+wz*wz)))/(wx*wx+wy*wy+wz*wz)*wz*wx)*X+(sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(wx*wx+wy*wy+wz*wz)*wx+(1.0-cos(sqrt(wx*wx+wy*wy+wz*wz)))/(wx*wx+wy*wy+wz*wz)*wz*wy)*Y+tz);
            MapleGenVar3 = (av*(sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(wx*wx+wy*wy+wz*wz)*wz+(1.0-
                                                                                         cos(sqrt(wx*wx+wy*wy+wz*wz)))/(wx*wx+wy*wy+wz*wz)*wy*wx)+v0*(-
                                                                                                                                                      sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(wx*wx+wy*wy+wz*wz)*wy+(1.0-
                                                                                                                                                                                                               cos(sqrt(wx*wx+wy*wy+wz*wz)))/(wx*wx+wy*wy+wz*wz)*wz*wx))*X+(av*(1.0+(1.0-
                                                                                                                                                                                                                                                                                     cos(sqrt(wx*wx+wy*wy+wz*wz)))/(wx*wx+wy*wy+wz*wz)*(-wz*wz-wx*wx))+v0*(sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(wx*wx+wy*wy+wz*wz)*wx+(1.0-
                                                                                                                                                                                                                                                                                                                                                                                                                   cos(sqrt(wx*wx+wy*wy+wz*wz)))/(wx*wx+wy*wy+wz*wz)*wz*wy))*Y+av*ty+v0*tz;
            MapleGenVar4 = 1/(pow((-sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(wx*wx+wy*wy+wz*wz)*wy+(1.0-
                                                                                             cos(sqrt(wx*wx+wy*wy+wz*wz)))/(wx*wx+wy*wy+wz*wz)*wz*wx)*X+(sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(wx*wx+wy*wy+wz*wz)*wx+(1.0-cos(sqrt(wx*wx+wy*wy+wz*wz)))/(wx*wx+wy*wy+wz*wz)*wz*wy)*Y+tz,2.0));
            MapleGenVar2 = MapleGenVar3*MapleGenVar4;
            cvmSet(J,x_y_total*j+2*i+1 , 5+j*6+5, MapleGenVar1+MapleGenVar2);
        }
    }
}

//< Function setJ >
//Construct Jacobian matrix for the image w.r.t parameter vector P and ObjectPoints (for a single
//          

void setJ(CvMat *P,CvMat *J, CvMat *ObjectPoints){
    int i;
    double u0, v0, au, av, sk;
    double tx, ty, tz, wx, wy, wz;
    double X, Y;
    // dummy variables
    double
    MapleGenVar1,MapleGenVar2,MapleGenVar3,MapleGenVar4,MapleGenVar5,MapleGenVar6,MapleGenVar7,
    MapleGenVar8,MapleGenVar9,MapleGenVar10;
    au=cvmGet(P,0,0);
    av=cvmGet(P,1,0);
    u0=cvmGet(P,2,0);
    v0=cvmGet(P,3,0);
    sk=cvmGet(P,4,0);
    wx=cvmGet(P,5,0);
    wy=cvmGet(P,6,0);
    wz=cvmGet(P,7,0);
    tx=cvmGet(P,8,0);
    ty=cvmGet(P,9,0);
    tz=cvmGet(P,10,0);
    for(i=0;i<REAL_NUM_CTRL_PTS;i++)
    {
        X=cvmGet(ObjectPoints,0,i);
        Y=cvmGet(ObjectPoints,1,i);
        // Setup Jacobian matrix
        cvmSet(J,2*i , 0 , -((1.0+(1.0-cos(sqrt(wx*wx+wy*wy+wz*wz)))/(wx*wx+wy*wy+wz*wz)*(-wz*wz-wy*wy))*X+(-
                                                                                                            sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(wx*wx+wy*wy+wz*wz)*wz+(1.0-
                                                                                                                                                                     cos(sqrt(wx*wx+wy*wy+wz*wz)))/(wx*wx+wy*wy+wz*wz)*wy*wx)*Y+tx)/((-
                                                                                                                                                                                                                                      sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(wx*wx+wy*wy+wz*wz)*wy+(1.0-
                                                                                                                                                                                                                                                                                               cos(sqrt(wx*wx+wy*wy+wz*wz)))/(wx*wx+wy*wy+wz*wz)*wz*wx)*X+(sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(wx*wx+wy*wy+wz*wz)*wx+(1.0-cos(sqrt(wx*wx+wy*wy+wz*wz)))/(wx*wx+wy*wy+wz*wz)*wz*wy)*Y+tz));
        cvmSet(J,2*i+1 , 0 , 0 );
        cvmSet(J,2*i , 1 , 0 );
        cvmSet(J,2*i+1 , 1 , -((sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(wx*wx+wy*wy+wz*wz)*wz+(1.0-
                                                                                         cos(sqrt(wx*wx+wy*wy+wz*wz)))/(wx*wx+wy*wy+wz*wz)*wy*wx)*X+(1.0+(1.0-
                                                                                                                                                          cos(sqrt(wx*wx+wy*wy+wz*wz)))/(wx*wx+wy*wy+wz*wz)*(-wz*wz-wx*wx))*Y+ty)/((-
                                                                                                                                                                                                                                    sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(wx*wx+wy*wy+wz*wz)*wy+(1.0-
                                                                                                                                                                                                                                                                                             cos(sqrt(wx*wx+wy*wy+wz*wz)))/(wx*wx+wy*wy+wz*wz)*wz*wx)*X+(sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(wx*wx+wy*wy+wz*wz)*wx+(1.0-cos(sqrt(wx*wx+wy*wy+wz*wz)))/(wx*wx+wy*wy+wz*wz)*wz*wy)*Y+tz));
        cvmSet(J,2*i , 2 , -1);
        cvmSet(J,2*i+1 , 2 , 0);
        cvmSet(J,2*i , 3 , 0);
        cvmSet(J,2*i+1 , 3 , -1);
        cvmSet(J,2*i , 4 , -((sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(wx*wx+wy*wy+wz*wz)*wz+(1.0-
                                                                                       cos(sqrt(wx*wx+wy*wy+wz*wz)))/(wx*wx+wy*wy+wz*wz)*wy*wx)*X+(1.0+(1.0-
                                                                                                                                                        cos(sqrt(wx*wx+wy*wy+wz*wz)))/(wx*wx+wy*wy+wz*wz)*(-wz*wz-wx*wx))*Y+ty)/((-
                                                                                                                                                                                                                                  sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(wx*wx+wy*wy+wz*wz)*wy+(1.0-
                                                                                                                                                                                                                                                                                           cos(sqrt(wx*wx+wy*wy+wz*wz)))/(wx*wx+wy*wy+wz*wz)*wz*wx)*X+(sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(wx*wx+wy*wy+wz*wz)*wx+(1.0-cos(sqrt(wx*wx+wy*wy+wz*wz)))/(wx*wx+wy*wy+wz*wz)*wz*wy)*Y+tz));
        cvmSet(J,2*i+1 , 4 , 0);
        MapleGenVar2 = -1.0;
        MapleGenVar7 = au*(sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(pow(wx*wx+wy*wy+wz*wz,3.0))*wx*(-wz*wz-wy*wy)-
                           2.0*(1.0-cos(sqrt(wx*wx+wy*wy+wz*wz)))/pow(wx*wx+wy*wy+wz*wz,2.0)*(-wz*wz-wy*wy)*wx);
        MapleGenVar9 = sk*(cos(sqrt(wx*wx+wy*wy+wz*wz))/(wx*wx+wy*wy+wz*wz)*wx*wz-sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(pow(wx*wx+wy*wy+wz*wz,3.0))*wz*wx+sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(pow(wx*wx+wy*wy
                                                                                                                                                                                            +wz*wz,3.0))*wx*wx*wy-2.0*(1.0-cos(sqrt(wx*wx+wy*wy+wz*wz)))/pow(wx*wx+wy*wy+wz*wz,2.0)*wy*wx*wx+(1.0-
                                                                                                                                                                                                                                                                                              cos(sqrt(wx*wx+wy*wy+wz*wz)))/(wx*wx+wy*wy+wz*wz)*wy);
        MapleGenVar10 = u0*(-
                            cos(sqrt(wx*wx+wy*wy+wz*wz))/(wx*wx+wy*wy+wz*wz)*wx*wy+sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(pow(wx*wx+wy*wy+wz*wz,3.0
                                                                                                                         ))*wy*wx+sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(pow(wx*wx+wy*wy+wz*wz,3.0))*wx*wx*wz-2.0*(1.0-
                                                                                                                                                                                                              cos(sqrt(wx*wx+wy*wy+wz*wz)))/pow(wx*wx+wy*wy+wz*wz,2.0)*wz*wx*wx+(1.0-
                                                                                                                                                                                                                                                                                 cos(sqrt(wx*wx+wy*wy+wz*wz)))/(wx*wx+wy*wy+wz*wz)*wz);
        MapleGenVar8 = MapleGenVar9+MapleGenVar10;
        MapleGenVar6 = MapleGenVar7+MapleGenVar8;
        MapleGenVar7 = X;
        MapleGenVar5 = MapleGenVar6*MapleGenVar7;
        MapleGenVar8 = au*(-
                           cos(sqrt(wx*wx+wy*wy+wz*wz))/(wx*wx+wy*wy+wz*wz)*wx*wz+sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(pow(wx*wx+wy*wy+wz*wz,3.0)
                                                                                                                    )*wz*wx+sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(pow(wx*wx+wy*wy+wz*wz,3.0))*wx*wx*wy-2.0*(1.0-
                                                                                                                                                                                                        cos(sqrt(wx*wx+wy*wy+wz*wz)))/pow(wx*wx+wy*wy+wz*wz,2.0)*wy*wx*wx+(1.0-
                                                                                                                                                                                                                                                                           cos(sqrt(wx*wx+wy*wy+wz*wz)))/(wx*wx+wy*wy+wz*wz)*wy);
        MapleGenVar9 = sk*(sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(pow(wx*wx+wy*wy+wz*wz,3.0))*wx*(-wz*wz-wx*wx)-
                           2.0*(1.0-cos(sqrt(wx*wx+wy*wy+wz*wz)))/pow(wx*wx+wy*wy+wz*wz,2.0)*(-wz*wz-wx*wx)*wx-2.0*(1.0-
                                                                                                                    cos(sqrt(wx*wx+wy*wy+wz*wz)))/(wx*wx+wy*wy+wz*wz)*wx)+u0*(cos(sqrt(wx*wx+wy*wy+wz*wz))/(wx*wx+wy*wy+wz*wz)*wx*wx-sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(pow(wx*wx+wy*wy+wz*wz,3.0))*wx*wx+sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(wx*wx+wy*wy+wz
                                                                                                                                                                                                                                                                                                                                           *wz)+sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(pow(wx*wx+wy*wy+wz*wz,3.0))*wx*wz*wy-2.0*(1.0-
                                                                                                                                                                                                                                                                                                                                                                                                                            cos(sqrt(wx*wx+wy*wy+wz*wz)))/pow(wx*wx+wy*wy+wz*wz,2.0)*wz*wy*wx);
        MapleGenVar7 = MapleGenVar8+MapleGenVar9;
        MapleGenVar8 = Y;
        MapleGenVar6 = MapleGenVar7*MapleGenVar8;
        MapleGenVar4 = MapleGenVar5+MapleGenVar6;
        MapleGenVar5 = 1/((-sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(wx*wx+wy*wy+wz*wz)*wy+(1.0-
                                                                                     cos(sqrt(wx*wx+wy*wy+wz*wz)))/(wx*wx+wy*wy+wz*wz)*wz*wx)*X+(sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(wx*wx+wy*wy+wz*wz)*wx+(1.0-cos(sqrt(wx*wx+wy*wy+wz*wz)))/(wx*wx+wy*wy+wz*wz)*wz*wy)*Y+tz);
        MapleGenVar3 = MapleGenVar4*MapleGenVar5;
        MapleGenVar1 = MapleGenVar2*MapleGenVar3;
        MapleGenVar5 = (au*(1.0+(1.0-cos(sqrt(wx*wx+wy*wy+wz*wz)))/(wx*wx+wy*wy+wz*wz)*(-wz*wz-wy*wy))+sk*(sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(wx*wx+wy*wy+wz*wz)*wz+(1.0-cos(sqrt(wx*wx+wy*wy+wz*wz)))/(wx*wx+wy*wy+wz*wz)*wy*wx)+u0*(-
                                                                                                                                                                                                                                     sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(wx*wx+wy*wy+wz*wz)*wy+(1.0-
                                                                                                                                                                                                                                                                                              cos(sqrt(wx*wx+wy*wy+wz*wz)))/(wx*wx+wy*wy+wz*wz)*wz*wx))*X;
        MapleGenVar6 = (au*(-sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(wx*wx+wy*wy+wz*wz)*wz+(1.0-
                                                                                      cos(sqrt(wx*wx+wy*wy+wz*wz)))/(wx*wx+wy*wy+wz*wz)*wy*wx)+sk*(1.0+(1.0-
                                                                                                                                                        cos(sqrt(wx*wx+wy*wy+wz*wz)))/(wx*wx+wy*wy+wz*wz)*(-wz*wz-wx*wx))+u0*(sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(wx*wx+wy*wy+wz*wz)*wx+(1.0-
                                                                                                                                                                                                                                                                                       cos(sqrt(wx*wx+wy*wy+wz*wz)))/(wx*wx+wy*wy+wz*wz)*wz*wy))*Y;
        MapleGenVar4 = MapleGenVar5+MapleGenVar6;
        MapleGenVar3 = MapleGenVar4+au*tx+sk*ty+u0*tz;
        MapleGenVar5 = 1/(pow((-sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(wx*wx+wy*wy+wz*wz)*wy+(1.0-
                                                                                         cos(sqrt(wx*wx+wy*wy+wz*wz)))/(wx*wx+wy*wy+wz*wz)*wz*wx)*X+(sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(wx*wx+wy*wy+wz*wz)*wx+(1.0-cos(sqrt(wx*wx+wy*wy+wz*wz)))/(wx*wx+wy*wy+wz*wz)*wz*wy)*Y+tz,2.0));
        MapleGenVar7 = (-
                        cos(sqrt(wx*wx+wy*wy+wz*wz))/(wx*wx+wy*wy+wz*wz)*wx*wy+sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(pow(wx*wx+wy*wy+wz*wz,3.0
                                                                                                                     ))*wy*wx+sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(pow(wx*wx+wy*wy+wz*wz,3.0))*wx*wx*wz-2.0*(1.0-
                                                                                                                                                                                                          cos(sqrt(wx*wx+wy*wy+wz*wz)))/pow(wx*wx+wy*wy+wz*wz,2.0)*wz*wx*wx+(1.0-
                                                                                                                                                                                                                                                                             cos(sqrt(wx*wx+wy*wy+wz*wz)))/(wx*wx+wy*wy+wz*wz)*wz)*X;
        MapleGenVar8 = (cos(sqrt(wx*wx+wy*wy+wz*wz))/(wx*wx+wy*wy+wz*wz)*wx*wx-sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(pow(wx*wx+wy*wy+wz*wz,3.0))*wx*wx+sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(wx*wx+wy*wy+wz
                                                                                                                                                                                     *wz)+sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(pow(wx*wx+wy*wy+wz*wz,3.0))*wx*wz*wy-2.0*(1.0-
                                                                                                                                                                                                                                                                      cos(sqrt(wx*wx+wy*wy+wz*wz)))/pow(wx*wx+wy*wy+wz*wz,2.0)*wz*wy*wx)*Y;
        MapleGenVar6 = MapleGenVar7+MapleGenVar8;
        MapleGenVar4 = MapleGenVar5*MapleGenVar6;
        MapleGenVar2 = MapleGenVar3*MapleGenVar4;
        cvmSet(J,2*i , 5 , MapleGenVar1+MapleGenVar2);
        MapleGenVar2 = -1.0;
        MapleGenVar7 = av*(cos(sqrt(wx*wx+wy*wy+wz*wz))/(wx*wx+wy*wy+wz*wz)*wx*wz-sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(pow(wx*wx+wy*wy+wz*wz,3.0))*wz*wx+sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(pow(wx*wx+wy*wy
                                                                                                                                                                                            +wz*wz,3.0))*wx*wx*wy-2.0*(1.0-cos(sqrt(wx*wx+wy*wy+wz*wz)))/pow(wx*wx+wy*wy+wz*wz,2.0)*wy*wx*wx+(1.0-
                                                                                                                                                                                                                                                                                              cos(sqrt(wx*wx+wy*wy+wz*wz)))/(wx*wx+wy*wy+wz*wz)*wy);
        MapleGenVar8 = v0*(-
                           cos(sqrt(wx*wx+wy*wy+wz*wz))/(wx*wx+wy*wy+wz*wz)*wx*wy+sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(pow(wx*wx+wy*wy+wz*wz,3.0
                                                                                                                        ))*wy*wx+sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(pow(wx*wx+wy*wy+wz*wz,3.0))*wx*wx*wz-2.0*(1.0-
                                                                                                                                                                                                             cos(sqrt(wx*wx+wy*wy+wz*wz)))/pow(wx*wx+wy*wy+wz*wz,2.0)*wz*wx*wx+(1.0-
                                                                                                                                                                                                                                                                                cos(sqrt(wx*wx+wy*wy+wz*wz)))/(wx*wx+wy*wy+wz*wz)*wz);
        MapleGenVar6 = MapleGenVar7+MapleGenVar8;
        MapleGenVar7 = X;
        MapleGenVar5 = MapleGenVar6*MapleGenVar7;
        MapleGenVar6 = (av*(sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(pow(wx*wx+wy*wy+wz*wz,3.0))*wx*(-wz*wz-wx*wx)-
                            2.0*(1.0-cos(sqrt(wx*wx+wy*wy+wz*wz)))/pow(wx*wx+wy*wy+wz*wz,2.0)*(-wz*wz-wx*wx)*wx-2.0*(1.0-
                                                                                                                     cos(sqrt(wx*wx+wy*wy+wz*wz)))/(wx*wx+wy*wy+wz*wz)*wx)+v0*(cos(sqrt(wx*wx+wy*wy+wz*wz))/(wx*wx+wy*wy+wz*wz)*wx*wx-sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(pow(wx*wx+wy*wy+wz*wz,3.0))*wx*wx+sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(wx*wx+wy*wy+wz
                                                                                                                                                                                                                                                                                                                                            *wz)+sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(pow(wx*wx+wy*wy+wz*wz,3.0))*wx*wz*wy-2.0*(1.0-
                                                                                                                                                                                                                                                                                                                                                                                                                             cos(sqrt(wx*wx+wy*wy+wz*wz)))/pow(wx*wx+wy*wy+wz*wz,2.0)*wz*wy*wx))*Y;
        MapleGenVar4 = MapleGenVar5+MapleGenVar6;
        MapleGenVar5 = 1/((-sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(wx*wx+wy*wy+wz*wz)*wy+(1.0-
                                                                                     cos(sqrt(wx*wx+wy*wy+wz*wz)))/(wx*wx+wy*wy+wz*wz)*wz*wx)*X+(sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(wx*wx+wy*wy+wz*wz)*wx+(1.0-cos(sqrt(wx*wx+wy*wy+wz*wz)))/(wx*wx+wy*wy+wz*wz)*wz*wy)*Y+tz);
        MapleGenVar3 = MapleGenVar4*MapleGenVar5;
        MapleGenVar1 = MapleGenVar2*MapleGenVar3;
        MapleGenVar3 = (av*(sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(wx*wx+wy*wy+wz*wz)*wz+(1.0-
                                                                                     cos(sqrt(wx*wx+wy*wy+wz*wz)))/(wx*wx+wy*wy+wz*wz)*wy*wx)+v0*(-
                                                                                                                                                  sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(wx*wx+wy*wy+wz*wz)*wy+(1.0-
                                                                                                                                                                                                           cos(sqrt(wx*wx+wy*wy+wz*wz)))/(wx*wx+wy*wy+wz*wz)*wz*wx))*X+(av*(1.0+(1.0-
                                                                                                                                                                                                                                                                                 cos(sqrt(wx*wx+wy*wy+wz*wz)))/(wx*wx+wy*wy+wz*wz)*(-wz*wz-wx*wx))+v0*(sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(wx*wx+wy*wy+wz*wz)*wx+(1.0-
                                                                                                                                                                                                                                                                                                                                                                                                               cos(sqrt(wx*wx+wy*wy+wz*wz)))/(wx*wx+wy*wy+wz*wz)*wz*wy))*Y+av*ty+v0*tz;
        MapleGenVar5 = 1/(pow((-sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(wx*wx+wy*wy+wz*wz)*wy+(1.0-
                                                                                         cos(sqrt(wx*wx+wy*wy+wz*wz)))/(wx*wx+wy*wy+wz*wz)*wz*wx)*X+(sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(wx*wx+wy*wy+wz*wz)*wx+(1.0-cos(sqrt(wx*wx+wy*wy+wz*wz)))/(wx*wx+wy*wy+wz*wz)*wz*wy)*Y+tz,2.0));
        MapleGenVar7 = (-
                        cos(sqrt(wx*wx+wy*wy+wz*wz))/(wx*wx+wy*wy+wz*wz)*wx*wy+sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(pow(wx*wx+wy*wy+wz*wz,3.0
                                                                                                                     ))*wy*wx+sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(pow(wx*wx+wy*wy+wz*wz,3.0))*wx*wx*wz-2.0*(1.0-
                                                                                                                                                                                                          cos(sqrt(wx*wx+wy*wy+wz*wz)))/pow(wx*wx+wy*wy+wz*wz,2.0)*wz*wx*wx+(1.0-
                                                                                                                                                                                                                                                                             cos(sqrt(wx*wx+wy*wy+wz*wz)))/(wx*wx+wy*wy+wz*wz)*wz)*X;
        MapleGenVar8 = (cos(sqrt(wx*wx+wy*wy+wz*wz))/(wx*wx+wy*wy+wz*wz)*wx*wx-sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(pow(wx*wx+wy*wy+wz*wz,3.0))*wx*wx+sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(wx*wx+wy*wy+wz
                                                                                                                                                                                    *wz)+sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(pow(wx*wx+wy*wy+wz*wz,3.0))*wx*wz*wy-2.0*(1.0-
                                                                                                                                                                                                                                                                     cos(sqrt(wx*wx+wy*wy+wz*wz)))/pow(wx*wx+wy*wy+wz*wz,2.0)*wz*wy*wx)*Y;
        MapleGenVar6 = MapleGenVar7+MapleGenVar8;
        MapleGenVar4 = MapleGenVar5*MapleGenVar6;
        MapleGenVar2 = MapleGenVar3*MapleGenVar4;
        cvmSet(J,2*i+1 , 5 , MapleGenVar1+MapleGenVar2);
        
        MapleGenVar2 = -1.0;
        MapleGenVar7 = au*(sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(pow(wx*wx+wy*wy+wz*wz,3.0))*wy*(-wz*wz-wy*wy)-
                           2.0*(1.0-cos(sqrt(wx*wx+wy*wy+wz*wz)))/pow(wx*wx+wy*wy+wz*wz,2.0)*(-wz*wz-wy*wy)*wy-2.0*(1.0-
                                                                                                                    cos(sqrt(wx*wx+wy*wy+wz*wz)))/(wx*wx+wy*wy+wz*wz)*wy);
        MapleGenVar9 = sk*(cos(sqrt(wx*wx+wy*wy+wz*wz))/(wx*wx+wy*wy+wz*wz)*wy*wz-sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(pow(wx*wx+wy*wy+wz*wz,3.0))*wz*wy+sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(pow(wx*wx+wy*wy
                                                                                                                                                                                            +wz*wz,3.0))*wy*wy*wx-2.0*(1.0-cos(sqrt(wx*wx+wy*wy+wz*wz)))/pow(wx*wx+wy*wy+wz*wz,2.0)*wy*wy*wx+(1.0-
                                                                                                                                                                                                                                                                                              cos(sqrt(wx*wx+wy*wy+wz*wz)))/(wx*wx+wy*wy+wz*wz)*wx);
        MapleGenVar10 = u0*(-
                            cos(sqrt(wx*wx+wy*wy+wz*wz))/(wx*wx+wy*wy+wz*wz)*wy*wy+sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(pow(wx*wx+wy*wy+wz*wz,3.0
                                                                                                                         ))*wy*wy-sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(wx*wx+wy*wy+wz*wz)+sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(pow(wx*wx+wy*wy+wz*wz,3.0))*wx
                            *wz*wy-2.0*(1.0-cos(sqrt(wx*wx+wy*wy+wz*wz)))/pow(wx*wx+wy*wy+wz*wz,2.0)*wz*wy*wx);
        MapleGenVar8 = MapleGenVar9+MapleGenVar10;
        MapleGenVar6 = MapleGenVar7+MapleGenVar8;
        MapleGenVar7 = X;
        MapleGenVar5 = MapleGenVar6*MapleGenVar7;
        MapleGenVar8 = au*(-
                           cos(sqrt(wx*wx+wy*wy+wz*wz))/(wx*wx+wy*wy+wz*wz)*wy*wz+sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(pow(wx*wx+wy*wy+wz*wz,3.0)
                                                                                                                    )*wz*wy+sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(pow(wx*wx+wy*wy+wz*wz,3.0))*wy*wy*wx-2.0*(1.0-
                                                                                                                                                                                                        cos(sqrt(wx*wx+wy*wy+wz*wz)))/pow(wx*wx+wy*wy+wz*wz,2.0)*wy*wy*wx+(1.0-
                                                                                                                                                                                                                                                                           cos(sqrt(wx*wx+wy*wy+wz*wz)))/(wx*wx+wy*wy+wz*wz)*wx);
        MapleGenVar9 = sk*(sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(pow(wx*wx+wy*wy+wz*wz,3.0))*wy*(-wz*wz-wx*wx)-
                           2.0*(1.0-cos(sqrt(wx*wx+wy*wy+wz*wz)))/pow(wx*wx+wy*wy+wz*wz,2.0)*(-wz*wz-wx*wx)*wy)+u0*(cos(sqrt(wx*wx+wy*wy+wz*wz))/(wx*wx+wy*wy+wz*wz)*wx*wy-sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(pow(wx*wx+wy*wy+wz*wz,3.0))*wy*wx+sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(pow(wx*wx+wy*wy+wz*wz,3.0))*wy*wy*wz-2.0*(1.0-cos(sqrt(wx*wx+wy*wy+wz*wz)))/pow(wx*wx+wy*wy+wz*wz,2.0)*wz*wy*wy+(1.0-
                                                                                                                                                                                                                                                                                                                                                                                                 cos(sqrt(wx*wx+wy*wy+wz*wz)))/(wx*wx+wy*wy+wz*wz)*wz);
        MapleGenVar7 = MapleGenVar8+MapleGenVar9;
        MapleGenVar8 = Y;
        MapleGenVar6 = MapleGenVar7*MapleGenVar8;
        MapleGenVar4 = MapleGenVar5+MapleGenVar6;
        MapleGenVar5 = 1/((-sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(wx*wx+wy*wy+wz*wz)*wy+(1.0-
                                                                                     cos(sqrt(wx*wx+wy*wy+wz*wz)))/(wx*wx+wy*wy+wz*wz)*wz*wx)*X+(sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(wx*wx+wy*wy+wz*wz)*wx+(1.0-cos(sqrt(wx*wx+wy*wy+wz*wz)))/(wx*wx+wy*wy+wz*wz)*wz*wy)*Y+tz);
        MapleGenVar3 = MapleGenVar4*MapleGenVar5;
        MapleGenVar1 = MapleGenVar2*MapleGenVar3;
        MapleGenVar5 = (au*(1.0+(1.0-cos(sqrt(wx*wx+wy*wy+wz*wz)))/(wx*wx+wy*wy+wz*wz)*(-wz*wz-wy*wy))+sk*(sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(wx*wx+wy*wy+wz*wz)*wz+(1.0-
                                                                                                                                                                    cos(sqrt(wx*wx+wy*wy+wz*wz)))/(wx*wx+wy*wy+wz*wz)*wy*wx)+u0*(-
                                                                                                                                                                                                                                 sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(wx*wx+wy*wy+wz*wz)*wy+(1.0-
                                                                                                                                                                                                                                                                                          cos(sqrt(wx*wx+wy*wy+wz*wz)))/(wx*wx+wy*wy+wz*wz)*wz*wx))*X;
        MapleGenVar6 = (au*(-sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(wx*wx+wy*wy+wz*wz)*wz+(1.0-
                                                                                      cos(sqrt(wx*wx+wy*wy+wz*wz)))/(wx*wx+wy*wy+wz*wz)*wy*wx)+sk*(1.0+(1.0-
                                                                                                                                                        cos(sqrt(wx*wx+wy*wy+wz*wz)))/(wx*wx+wy*wy+wz*wz)*(-wz*wz-wx*wx))+u0*(sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(wx*wx+wy*wy+wz*wz)*wx+(1.0-
                                                                                                                                                                                                                                                                                       cos(sqrt(wx*wx+wy*wy+wz*wz)))/(wx*wx+wy*wy+wz*wz)*wz*wy))*Y;
        MapleGenVar4 = MapleGenVar5+MapleGenVar6;
        MapleGenVar3 = MapleGenVar4+au*tx+sk*ty+u0*tz;
        MapleGenVar5 = 1/(pow((-sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(wx*wx+wy*wy+wz*wz)*wy+(1.0-
                                                                                         cos(sqrt(wx*wx+wy*wy+wz*wz)))/(wx*wx+wy*wy+wz*wz)*wz*wx)*X+(sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(wx*wx+wy*wy+wz*wz)*wx+(1.0-cos(sqrt(wx*wx+wy*wy+wz*wz)))/(wx*wx+wy*wy+wz*wz)*wz*wy)*Y+tz,2.0));
        MapleGenVar7 = (-
                        cos(sqrt(wx*wx+wy*wy+wz*wz))/(wx*wx+wy*wy+wz*wz)*wy*wy+sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(pow(wx*wx+wy*wy+wz*wz,3.0
                                                                                                                     ))*wy*wy-sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(wx*wx+wy*wy+wz*wz)+sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(pow(wx*wx+wy*wy+wz*wz,3.0))*wx
                        *wz*wy-2.0*(1.0-cos(sqrt(wx*wx+wy*wy+wz*wz)))/pow(wx*wx+wy*wy+wz*wz,2.0)*wz*wy*wx)*X;
        MapleGenVar8 = (cos(sqrt(wx*wx+wy*wy+wz*wz))/(wx*wx+wy*wy+wz*wz)*wx*wy-sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(pow(wx*wx+wy*wy+wz*wz,3.0))*wy*wx+sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(pow(wx*wx+wy*wy+wz*wz,3.0))*wy*wy*wz-2.0*(1.0-cos(sqrt(wx*wx+wy*wy+wz*wz)))/pow(wx*wx+wy*wy+wz*wz,2.0)*wz*wy*wy+(1.0-
                                                                                                                                                                                                                                                                                           cos(sqrt(wx*wx+wy*wy+wz*wz)))/(wx*wx+wy*wy+wz*wz)*wz)*Y;
        MapleGenVar6 = MapleGenVar7+MapleGenVar8;
        MapleGenVar4 = MapleGenVar5*MapleGenVar6;
        MapleGenVar2 = MapleGenVar3*MapleGenVar4;
        cvmSet(J,2*i , 6 , MapleGenVar1+MapleGenVar2);
        
        MapleGenVar2 = -1.0;
        MapleGenVar7 = av*(cos(sqrt(wx*wx+wy*wy+wz*wz))/(wx*wx+wy*wy+wz*wz)*wy*wz-sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(pow(wx*wx+wy*wy+wz*wz,3.0))*wz*wy+sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(pow(wx*wx+wy*wy
                                                                                                                                                                                            +wz*wz,3.0))*wy*wy*wx-2.0*(1.0-cos(sqrt(wx*wx+wy*wy+wz*wz)))/pow(wx*wx+wy*wy+wz*wz,2.0)*wy*wy*wx+(1.0-
                                                                                                                                                                                                                                                                                              cos(sqrt(wx*wx+wy*wy+wz*wz)))/(wx*wx+wy*wy+wz*wz)*wx);
        MapleGenVar8 = v0*(-
                           cos(sqrt(wx*wx+wy*wy+wz*wz))/(wx*wx+wy*wy+wz*wz)*wy*wy+sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(pow(wx*wx+wy*wy+wz*wz,3.0
                                                                                                                        ))*wy*wy-
                           sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(wx*wx+wy*wy+wz*wz)+sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(pow(wx*wx+wy*wy+wz*wz,3.0))*wx
                           *wz*wy-2.0*(1.0-cos(sqrt(wx*wx+wy*wy+wz*wz)))/pow(wx*wx+wy*wy+wz*wz,2.0)*wz*wy*wx);
        MapleGenVar6 = MapleGenVar7+MapleGenVar8;
        MapleGenVar7 = X;
        MapleGenVar5 = MapleGenVar6*MapleGenVar7;
        MapleGenVar6 = (av*(sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(pow(wx*wx+wy*wy+wz*wz,3.0))*wy*(-wz*wz-wx*wx)-
                            2.0*(1.0-cos(sqrt(wx*wx+wy*wy+wz*wz)))/pow(wx*wx+wy*wy+wz*wz,2.0)*(-wz*wz-wx*wx)*wy)+v0*(cos(sqrt(wx*wx+wy*wy+wz*wz))/(wx*wx+wy*wy+wz*wz)*wx*wy-sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(pow(wx*wx+wy*wy+wz*wz,3.0))*wy*wx+sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(pow(wx*wx+wy*wy+wz*wz,3.0))*wy*wy*wz-2.0*(1.0-cos(sqrt(wx*wx+wy*wy+wz*wz)))/pow(wx*wx+wy*wy+wz*wz,2.0)*wz*wy*wy+(1.0-
                                                                                                                                                                                                                                                                                                                                                                                        cos(sqrt(wx*wx+wy*wy+wz*wz)))/(wx*wx+wy*wy+wz*wz)*wz))*Y;
        MapleGenVar4 = MapleGenVar5+MapleGenVar6;
        MapleGenVar5 = 1/((-sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(wx*wx+wy*wy+wz*wz)*wy+(1.0-
                                                                                     cos(sqrt(wx*wx+wy*wy+wz*wz)))/(wx*wx+wy*wy+wz*wz)*wz*wx)*X+(sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(wx*wx+wy*wy+wz*wz)*wx+(1.0-cos(sqrt(wx*wx+wy*wy+wz*wz)))/(wx*wx+wy*wy+wz*wz)*wz*wy)*Y+tz);
        MapleGenVar3 = MapleGenVar4*MapleGenVar5;
        MapleGenVar1 = MapleGenVar2*MapleGenVar3;
        MapleGenVar3 = (av*(sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(wx*wx+wy*wy+wz*wz)*wz+(1.0-
                                                                                     cos(sqrt(wx*wx+wy*wy+wz*wz)))/(wx*wx+wy*wy+wz*wz)*wy*wx)+v0*(-
                                                                                                                                                  sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(wx*wx+wy*wy+wz*wz)*wy+(1.0-
                                                                                                                                                                                                           cos(sqrt(wx*wx+wy*wy+wz*wz)))/(wx*wx+wy*wy+wz*wz)*wz*wx))*X+(av*(1.0+(1.0-
                                                                                                                                                                                                                                                                                 cos(sqrt(wx*wx+wy*wy+wz*wz)))/(wx*wx+wy*wy+wz*wz)*(-wz*wz-wx*wx))+v0*(sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(wx*wx+wy*wy+wz*wz)*wx+(1.0-
                                                                                                                                                                                                                                                                                                                                                                                                               cos(sqrt(wx*wx+wy*wy+wz*wz)))/(wx*wx+wy*wy+wz*wz)*wz*wy))*Y+av*ty+v0*tz;
        MapleGenVar5 = 1/(pow((-sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(wx*wx+wy*wy+wz*wz)*wy+(1.0-
                                                                                         cos(sqrt(wx*wx+wy*wy+wz*wz)))/(wx*wx+wy*wy+wz*wz)*wz*wx)*X+(sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(wx*wx+wy*wy+wz*wz)*wx+(1.0-cos(sqrt(wx*wx+wy*wy+wz*wz)))/(wx*wx+wy*wy+wz*wz)*wz*wy)*Y+tz,2.0));
        MapleGenVar7 = (-
                        cos(sqrt(wx*wx+wy*wy+wz*wz))/(wx*wx+wy*wy+wz*wz)*wy*wy+sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(pow(wx*wx+wy*wy+wz*wz,3.0
                                                                                                                     ))*wy*wy-sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(wx*wx+wy*wy+wz*wz)+sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(pow(wx*wx+wy*wy+wz*wz,3.0))*wx
                        *wz*wy-2.0*(1.0-cos(sqrt(wx*wx+wy*wy+wz*wz)))/pow(wx*wx+wy*wy+wz*wz,2.0)*wz*wy*wx)*X;
        MapleGenVar8 = (cos(sqrt(wx*wx+wy*wy+wz*wz))/(wx*wx+wy*wy+wz*wz)*wx*wy-sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(pow(wx*wx+wy*wy+wz*wz,3.0))*wy*wx+sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(pow(wx*wx+wy*wy+wz*wz,3.0))*wy*wy*wz-2.0*(1.0-cos(sqrt(wx*wx+wy*wy+wz*wz)))/pow(wx*wx+wy*wy+wz*wz,2.0)*wz*wy*wy+(1.0-
                                                                                                                                                                                                                                                                                                      cos(sqrt(wx*wx+wy*wy+wz*wz)))/(wx*wx+wy*wy+wz*wz)*wz)*Y;
        MapleGenVar6 = MapleGenVar7+MapleGenVar8;
        MapleGenVar4 = MapleGenVar5*MapleGenVar6;
        MapleGenVar2 = MapleGenVar3*MapleGenVar4;
        cvmSet(J,2*i+1 , 6 , MapleGenVar1+MapleGenVar2);
        
        MapleGenVar2 = -1.0;
        MapleGenVar7 = au*(sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(pow(wx*wx+wy*wy+wz*wz,3.0))*wz*(-wz*wz-wy*wy)-
                           2.0*(1.0-cos(sqrt(wx*wx+wy*wy+wz*wz)))/pow(wx*wx+wy*wy+wz*wz,2.0)*(-wz*wz-wy*wy)*wz-2.0*(1.0-
                                                                                                                    cos(sqrt(wx*wx+wy*wy+wz*wz)))/(wx*wx+wy*wy+wz*wz)*wz);
        MapleGenVar9 = sk*(cos(sqrt(wx*wx+wy*wy+wz*wz))/(wx*wx+wy*wy+wz*wz)*wz*wz-sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(pow(wx*wx+wy*wy+wz*wz,3.0))*wz*wz+sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(wx*wx+wy*wy+wz
                                                                                                                                                                                        *wz)+sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(pow(wx*wx+wy*wy+wz*wz,3.0))*wx*wz*wy-2.0*(1.0-
                                                                                                                                                                                                                                                                         cos(sqrt(wx*wx+wy*wy+wz*wz)))/pow(wx*wx+wy*wy+wz*wz,2.0)*wz*wy*wx);
        MapleGenVar10 = u0*(-
                            cos(sqrt(wx*wx+wy*wy+wz*wz))/(wx*wx+wy*wy+wz*wz)*wy*wz+sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(pow(wx*wx+wy*wy+wz*wz,3.0)
                                                                                                                     )*wz*wy+sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(pow(wx*wx+wy*wy+wz*wz,3.0))*wz*wz*wx-2.0*(1.0-
                                                                                                                                                                                                         cos(sqrt(wx*wx+wy*wy+wz*wz)))/pow(wx*wx+wy*wy+wz*wz,2.0)*wz*wz*wx+(1.0-
                                                                                                                                                                                                                                                                            cos(sqrt(wx*wx+wy*wy+wz*wz)))/(wx*wx+wy*wy+wz*wz)*wx);
        MapleGenVar8 = MapleGenVar9+MapleGenVar10;
        MapleGenVar6 = MapleGenVar7+MapleGenVar8;
        MapleGenVar7 = X;
        MapleGenVar5 = MapleGenVar6*MapleGenVar7;
        MapleGenVar8 = au*(-
                           cos(sqrt(wx*wx+wy*wy+wz*wz))/(wx*wx+wy*wy+wz*wz)*wz*wz+sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(pow(wx*wx+wy*wy+wz*wz,3.0)
                                                                                                                    )*wz*wz-sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(wx*wx+wy*wy+wz*wz)+sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(pow(wx*wx+wy*wy+wz*wz,3.0))*wx
                           *wz*wy-2.0*(1.0-cos(sqrt(wx*wx+wy*wy+wz*wz)))/pow(wx*wx+wy*wy+wz*wz,2.0)*wz*wy*wx);
        MapleGenVar9 = sk*(sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(pow(wx*wx+wy*wy+wz*wz,3.0))*wz*(-wz*wz-wx*wx)-
                           2.0*(1.0-cos(sqrt(wx*wx+wy*wy+wz*wz)))/pow(wx*wx+wy*wy+wz*wz,2.0)*(-wz*wz-wx*wx)*wz-2.0*(1.0-
                                                                                                                    cos(sqrt(wx*wx+wy*wy+wz*wz)))/(wx*wx+wy*wy+wz*wz)*wz)+u0*(cos(sqrt(wx*wx+wy*wy+wz*wz))/(wx*wx+wy*wy+wz*wz)*wx*wz-sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(pow(wx*wx+wy*wy+wz*wz,3.0))*wz*wx+sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(pow(wx*wx+wy*wy
                                                                                                                                                                                                                                                                                                                                               +wz*wz,3.0))*wz*wz*wy-2.0*(1.0-cos(sqrt(wx*wx+wy*wy+wz*wz)))/pow(wx*wx+wy*wy+wz*wz,2.0)*wz*wz*wy+(1.0-
                                                                                                                                                                                                                                                                                                                                                                                                                                                 cos(sqrt(wx*wx+wy*wy+wz*wz)))/(wx*wx+wy*wy+wz*wz)*wy);
        MapleGenVar7 = MapleGenVar8+MapleGenVar9;
        MapleGenVar8 = Y;
        MapleGenVar6 = MapleGenVar7*MapleGenVar8;
        MapleGenVar4 = MapleGenVar5+MapleGenVar6;
        MapleGenVar5 = 1/((-sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(wx*wx+wy*wy+wz*wz)*wy+(1.0-
                                                                                     cos(sqrt(wx*wx+wy*wy+wz*wz)))/(wx*wx+wy*wy+wz*wz)*wz*wx)*X+(sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(wx*wx+wy*wy+wz*wz)*wx+(1.0-cos(sqrt(wx*wx+wy*wy+wz*wz)))/(wx*wx+wy*wy+wz*wz)*wz*wy)*Y+tz);
        MapleGenVar3 = MapleGenVar4*MapleGenVar5;
        MapleGenVar1 = MapleGenVar2*MapleGenVar3;
        MapleGenVar5 = (au*(1.0+(1.0-cos(sqrt(wx*wx+wy*wy+wz*wz)))/(wx*wx+wy*wy+wz*wz)*(-wz*wz-wy*wy))+sk*(sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(wx*wx+wy*wy+wz*wz)*wz+(1.0-
                                                                                                                                                                    cos(sqrt(wx*wx+wy*wy+wz*wz)))/(wx*wx+wy*wy+wz*wz)*wy*wx)+u0*(-
                                                                                                                                                                                                                                 sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(wx*wx+wy*wy+wz*wz)*wy+(1.0-
                                                                                                                                                                                                                                                                                          cos(sqrt(wx*wx+wy*wy+wz*wz)))/(wx*wx+wy*wy+wz*wz)*wz*wx))*X;
        MapleGenVar6 = (au*(-sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(wx*wx+wy*wy+wz*wz)*wz+(1.0-
                                                                                      cos(sqrt(wx*wx+wy*wy+wz*wz)))/(wx*wx+wy*wy+wz*wz)*wy*wx)+sk*(1.0+(1.0-
                                                                                                                                                        cos(sqrt(wx*wx+wy*wy+wz*wz)))/(wx*wx+wy*wy+wz*wz)*(-wz*wz-wx*wx))+u0*(sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(wx*wx+wy*wy+wz*wz)*wx+(1.0-
                                                                                                                                                                                                                                                                                       cos(sqrt(wx*wx+wy*wy+wz*wz)))/(wx*wx+wy*wy+wz*wz)*wz*wy))*Y;
        MapleGenVar4 = MapleGenVar5+MapleGenVar6;
        MapleGenVar3 = MapleGenVar4+au*tx+sk*ty+u0*tz;
        MapleGenVar5 = 1/(pow((-sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(wx*wx+wy*wy+wz*wz)*wy+(1.0-
                                                                                         cos(sqrt(wx*wx+wy*wy+wz*wz)))/(wx*wx+wy*wy+wz*wz)*wz*wx)*X+(sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(wx*wx+wy*wy+wz*wz)*wx+(1.0-cos(sqrt(wx*wx+wy*wy+wz*wz)))/(wx*wx+wy*wy+wz*wz)*wz*wy)*Y+tz,2.0));
        MapleGenVar7 = (-
                        cos(sqrt(wx*wx+wy*wy+wz*wz))/(wx*wx+wy*wy+wz*wz)*wy*wz+sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(pow(wx*wx+wy*wy+wz*wz,3.0)
                                                                                                                 )*wz*wy+sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(pow(wx*wx+wy*wy+wz*wz,3.0))*wz*wz*wx-2.0*(1.0-
                                                                                                                                                                                                     cos(sqrt(wx*wx+wy*wy+wz*wz)))/pow(wx*wx+wy*wy+wz*wz,2.0)*wz*wz*wx+(1.0-
                                                                                                                                                                                                                                                                        cos(sqrt(wx*wx+wy*wy+wz*wz)))/(wx*wx+wy*wy+wz*wz)*wx)*X;
        MapleGenVar8 = (cos(sqrt(wx*wx+wy*wy+wz*wz))/(wx*wx+wy*wy+wz*wz)*wx*wz-sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(pow(wx*wx+wy*wy+wz*wz,3.0))*wz*wx+sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(pow(wx*wx+wy*wy
                                                                                                                                                                                         +wz*wz,3.0))*wz*wz*wy-2.0*(1.0-cos(sqrt(wx*wx+wy*wy+wz*wz)))/pow(wx*wx+wy*wy+wz*wz,2.0)*wz*wz*wy+(1.0-
                                                                                                                                                                                                                                                                                           cos(sqrt(wx*wx+wy*wy+wz*wz)))/(wx*wx+wy*wy+wz*wz)*wy)*Y;
        MapleGenVar6 = MapleGenVar7+MapleGenVar8;
        MapleGenVar4 = MapleGenVar5*MapleGenVar6;
        MapleGenVar2 = MapleGenVar3*MapleGenVar4;
        cvmSet(J,2*i , 7 , MapleGenVar1+MapleGenVar2);
        
        MapleGenVar2 = -1.0;
        MapleGenVar7 = av*(cos(sqrt(wx*wx+wy*wy+wz*wz))/(wx*wx+wy*wy+wz*wz)*wz*wz-sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(pow(wx*wx+wy*wy+wz*wz,3.0))*wz*wz+sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(wx*wx+wy*wy+wz
                                                                                                                                                                                        *wz)+sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(pow(wx*wx+wy*wy+wz*wz,3.0))*wx*wz*wy-2.0*(1.0-
                                                                                                                                                                                                                                                                         cos(sqrt(wx*wx+wy*wy+wz*wz)))/pow(wx*wx+wy*wy+wz*wz,2.0)*wz*wy*wx);
        MapleGenVar8 = v0*(-
                           cos(sqrt(wx*wx+wy*wy+wz*wz))/(wx*wx+wy*wy+wz*wz)*wy*wz+sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(pow(wx*wx+wy*wy+wz*wz,3.0)
                                                                                                                    )*wz*wy+sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(pow(wx*wx+wy*wy+wz*wz,3.0))*wz*wz*wx-2.0*(1.0-
                                                                                                                                                                                                        cos(sqrt(wx*wx+wy*wy+wz*wz)))/pow(wx*wx+wy*wy+wz*wz,2.0)*wz*wz*wx+(1.0-
                                                                                                                                                                                                                                                                           cos(sqrt(wx*wx+wy*wy+wz*wz)))/(wx*wx+wy*wy+wz*wz)*wx);
        MapleGenVar6 = MapleGenVar7+MapleGenVar8;
        MapleGenVar7 = X;
        MapleGenVar5 = MapleGenVar6*MapleGenVar7;
        MapleGenVar6 = (av*(sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(pow(wx*wx+wy*wy+wz*wz,3.0))*wz*(-wz*wz-wx*wx)-
                            2.0*(1.0-cos(sqrt(wx*wx+wy*wy+wz*wz)))/pow(wx*wx+wy*wy+wz*wz,2.0)*(-wz*wz-wx*wx)*wz-2.0*(1.0-
                                                                                                                     cos(sqrt(wx*wx+wy*wy+wz*wz)))/(wx*wx+wy*wy+wz*wz)*wz)+v0*(cos(sqrt(wx*wx+wy*wy+wz*wz))/(wx*wx+wy*wy+wz*wz)*wx*wz-sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(pow(wx*wx+wy*wy+wz*wz,3.0))*wz*wx+sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(pow(wx*wx+wy*wy
                                                                                                                                                                                                                                                                                                                                               +wz*wz,3.0))*wz*wz*wy-2.0*(1.0-cos(sqrt(wx*wx+wy*wy+wz*wz)))/pow(wx*wx+wy*wy+wz*wz,2.0)*wz*wz*wy+(1.0-
                                                                                                                                                                                                                                                                                                                                                                                                                                                 cos(sqrt(wx*wx+wy*wy+wz*wz)))/(wx*wx+wy*wy+wz*wz)*wy))*Y;
        MapleGenVar4 = MapleGenVar5+MapleGenVar6;
        MapleGenVar5 = 1/((-sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(wx*wx+wy*wy+wz*wz)*wy+(1.0-
                                                                                     cos(sqrt(wx*wx+wy*wy+wz*wz)))/(wx*wx+wy*wy+wz*wz)*wz*wx)*X+(sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(wx*wx+wy*wy+wz*wz)*wx+(1.0-cos(sqrt(wx*wx+wy*wy+wz*wz)))/(wx*wx+wy*wy+wz*wz)*wz*wy)*Y+tz);
        MapleGenVar3 = MapleGenVar4*MapleGenVar5;
        MapleGenVar1 = MapleGenVar2*MapleGenVar3;
        MapleGenVar3 = (av*(sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(wx*wx+wy*wy+wz*wz)*wz+(1.0-
                                                                                     cos(sqrt(wx*wx+wy*wy+wz*wz)))/(wx*wx+wy*wy+wz*wz)*wy*wx)+v0*(-
                                                                                                                                                  sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(wx*wx+wy*wy+wz*wz)*wy+(1.0-
                                                                                                                                                                                                           cos(sqrt(wx*wx+wy*wy+wz*wz)))/(wx*wx+wy*wy+wz*wz)*wz*wx))*X+(av*(1.0+(1.0-
                                                                                                                                                                                                                                                                                 cos(sqrt(wx*wx+wy*wy+wz*wz)))/(wx*wx+wy*wy+wz*wz)*(-wz*wz-wx*wx))+v0*(sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(wx*wx+wy*wy+wz*wz)*wx+(1.0-
                                                                                                                                                                                                                                                                                                                                                                                                                cos(sqrt(wx*wx+wy*wy+wz*wz)))/(wx*wx+wy*wy+wz*wz)*wz*wy))*Y+av*ty+v0*tz;
        MapleGenVar5 = 1/(pow((-sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(wx*wx+wy*wy+wz*wz)*wy+(1.0-
                                                                                         cos(sqrt(wx*wx+wy*wy+wz*wz)))/(wx*wx+wy*wy+wz*wz)*wz*wx)*X+(sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(wx*wx+wy*wy+wz*wz)*wx+(1.0-cos(sqrt(wx*wx+wy*wy+wz*wz)))/(wx*wx+wy*wy+wz*wz)*wz*wy)*Y+tz,2.0));
        MapleGenVar7 = (-
                        cos(sqrt(wx*wx+wy*wy+wz*wz))/(wx*wx+wy*wy+wz*wz)*wy*wz+sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(pow(wx*wx+wy*wy+wz*wz,3.0)
                                                                                                                 )*wz*wy+sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(pow(wx*wx+wy*wy+wz*wz,3.0))*wz*wz*wx-2.0*(1.0-
                                                                                                                                                                                                     cos(sqrt(wx*wx+wy*wy+wz*wz)))/pow(wx*wx+wy*wy+wz*wz,2.0)*wz*wz*wx+(1.0-
                                                                                                                                                                                                                                                                        cos(sqrt(wx*wx+wy*wy+wz*wz)))/(wx*wx+wy*wy+wz*wz)*wx)*X;
        MapleGenVar8 = (cos(sqrt(wx*wx+wy*wy+wz*wz))/(wx*wx+wy*wy+wz*wz)*wx*wz-sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(pow(wx*wx+wy*wy+wz*wz,3.0))*wz*wx+sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(pow(wx*wx+wy*wy
                                                                                                                                                                                         +wz*wz,3.0))*wz*wz*wy-2.0*(1.0-cos(sqrt(wx*wx+wy*wy+wz*wz)))/pow(wx*wx+wy*wy+wz*wz,2.0)*wz*wz*wy+(1.0-
                                                                                                                                                                                                                                                                                           cos(sqrt(wx*wx+wy*wy+wz*wz)))/(wx*wx+wy*wy+wz*wz)*wy)*Y;
        MapleGenVar6 = MapleGenVar7+MapleGenVar8;
        MapleGenVar4 = MapleGenVar5*MapleGenVar6;
        MapleGenVar2 = MapleGenVar3*MapleGenVar4;
        cvmSet(J,2*i+1 , 7 , MapleGenVar1+MapleGenVar2);
        
        cvmSet(J,2*i , 8 , -au/((-sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(wx*wx+wy*wy+wz*wz)*wy+(1.0-
                                                                                           cos(sqrt(wx*wx+wy*wy+wz*wz)))/(wx*wx+wy*wy+wz*wz)*wz*wx)*X+(sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(wx*wx+wy*wy+wz*wz)*wx+(1.0-cos(sqrt(wx*wx+wy*wy+wz*wz)))/(wx*wx+wy*wy+wz*wz)*wz*wy)*Y+tz));
        cvmSet(J,2*i+1 , 8 , 0);
        cvmSet(J,2*i , 9 , -sk/((-sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(wx*wx+wy*wy+wz*wz)*wy+(1.0-
                                                                                           cos(sqrt(wx*wx+wy*wy+wz*wz)))/(wx*wx+wy*wy+wz*wz)*wz*wx)*X+(sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(wx*wx+wy*wy+wz*wz)*wx+(1.0-cos(sqrt(wx*wx+wy*wy+wz*wz)))/(wx*wx+wy*wy+wz*wz)*wz*wy)*Y+tz));
        cvmSet(J,2*i+1 , 9 , -av/((-sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(wx*wx+wy*wy+wz*wz)*wy+(1.0-
                                                                                             cos(sqrt(wx*wx+wy*wy+wz*wz)))/(wx*wx+wy*wy+wz*wz)*wz*wx)*X+(sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(wx*wx+wy*wy+wz*wz)*wx+(1.0-cos(sqrt(wx*wx+wy*wy+wz*wz)))/(wx*wx+wy*wy+wz*wz)*wz*wy)*Y+tz));
        
        MapleGenVar1 = -u0/((-sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(wx*wx+wy*wy+wz*wz)*wy+(1.0-
                                                                                       cos(sqrt(wx*wx+wy*wy+wz*wz)))/(wx*wx+wy*wy+wz*wz)*wz*wx)*X+(sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(wx*wx+wy*wy+wz*wz)*wx+(1.0-cos(sqrt(wx*wx+wy*wy+wz*wz)))/(wx*wx+wy*wy+wz*wz)*wz*wy)*Y+tz);
        MapleGenVar5 = (au*(1.0+(1.0-cos(sqrt(wx*wx+wy*wy+wz*wz)))/(wx*wx+wy*wy+wz*wz)*(-wz*wz-wy*wy))+sk*(sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(wx*wx+wy*wy+wz*wz)*wz+(1.0-
                                                                                                                                                                    cos(sqrt(wx*wx+wy*wy+wz*wz)))/(wx*wx+wy*wy+wz*wz)*wy*wx)+u0*(-
                                                                                                                                                                                                                                 sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(wx*wx+wy*wy+wz*wz)*wy+(1.0-
                                                                                                                                                                                                                                                                                          cos(sqrt(wx*wx+wy*wy+wz*wz)))/(wx*wx+wy*wy+wz*wz)*wz*wx))*X;
        MapleGenVar6 = (au*(-sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(wx*wx+wy*wy+wz*wz)*wz+(1.0-
                                                                                      cos(sqrt(wx*wx+wy*wy+wz*wz)))/(wx*wx+wy*wy+wz*wz)*wy*wx)+sk*(1.0+(1.0-
                                                                                                                                                        cos(sqrt(wx*wx+wy*wy+wz*wz)))/(wx*wx+wy*wy+wz*wz)*(-wz*wz-wx*wx))+u0*(sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(wx*wx+wy*wy+wz*wz)*wx+(1.0-
                                                                                                                                                                                                                                                                                       cos(sqrt(wx*wx+wy*wy+wz*wz)))/(wx*wx+wy*wy+wz*wz)*wz*wy))*Y;
        MapleGenVar4 = MapleGenVar5+MapleGenVar6;
        MapleGenVar3 = MapleGenVar4+au*tx+sk*ty+u0*tz;
        MapleGenVar4 = 1/(pow((-sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(wx*wx+wy*wy+wz*wz)*wy+(1.0-
                                                                                         cos(sqrt(wx*wx+wy*wy+wz*wz)))/(wx*wx+wy*wy+wz*wz)*wz*wx)*X+(sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(wx*wx+wy*wy+wz*wz)*wx+(1.0-cos(sqrt(wx*wx+wy*wy+wz*wz)))/(wx*wx+wy*wy+wz*wz)*wz*wy)*Y+tz,2.0));
        MapleGenVar2 = MapleGenVar3*MapleGenVar4;
        cvmSet(J,2*i , 10, MapleGenVar1+MapleGenVar2);
        MapleGenVar1 = -v0/((-sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(wx*wx+wy*wy+wz*wz)*wy+(1.0-
                                                                                       cos(sqrt(wx*wx+wy*wy+wz*wz)))/(wx*wx+wy*wy+wz*wz)*wz*wx)*X+(sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(wx*wx+wy*wy+wz*wz)*wx+(1.0-cos(sqrt(wx*wx+wy*wy+wz*wz)))/(wx*wx+wy*wy+wz*wz)*wz*wy)*Y+tz);
        MapleGenVar3 = (av*(sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(wx*wx+wy*wy+wz*wz)*wz+(1.0-
                                                                                     cos(sqrt(wx*wx+wy*wy+wz*wz)))/(wx*wx+wy*wy+wz*wz)*wy*wx)+v0*(-
                                                                                                                                                  sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(wx*wx+wy*wy+wz*wz)*wy+(1.0-
                                                                                                                                                                                                           cos(sqrt(wx*wx+wy*wy+wz*wz)))/(wx*wx+wy*wy+wz*wz)*wz*wx))*X+(av*(1.0+(1.0-
                                                                                                                                                                                                                                                                                 cos(sqrt(wx*wx+wy*wy+wz*wz)))/(wx*wx+wy*wy+wz*wz)*(-wz*wz-wx*wx))+v0*(sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(wx*wx+wy*wy+wz*wz)*wx+(1.0-
                                                                                                                                                                                                                                                                                                                                                                                                                cos(sqrt(wx*wx+wy*wy+wz*wz)))/(wx*wx+wy*wy+wz*wz)*wz*wy))*Y+av*ty+v0*tz;
        MapleGenVar4 = 1/(pow((-sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(wx*wx+wy*wy+wz*wz)*wy+(1.0-
                                                                                         cos(sqrt(wx*wx+wy*wy+wz*wz)))/(wx*wx+wy*wy+wz*wz)*wz*wx)*X+(sin(sqrt(wx*wx+wy*wy+wz*wz))/sqrt(wx*wx+wy*wy+wz*wz)*wx+(1.0-cos(sqrt(wx*wx+wy*wy+wz*wz)))/(wx*wx+wy*wy+wz*wz)*wz*wy)*Y+tz,2.0));
        MapleGenVar2 = MapleGenVar3*MapleGenVar4;
        cvmSet(J,2*i+1 , 10, MapleGenVar1+MapleGenVar2);
    }
}

#endif /* jacobian_h */
