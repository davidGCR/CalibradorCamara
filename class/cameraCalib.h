//
//  cameraCalib.h
//  
//
//  Created by David Choqueluque Roman on 2/10/19.
//

#ifndef cameraCalib_h
#define cameraCalib_h

#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"

//#include <cv.h>
#include <iostream>
#include "jacobian.h"
#include "constants.h"
using namespace cv;
using namespace std;

// transformation from Rotation matrix to Rodrigues representation
void R2Rodrigues(CvMat *R, CvMat *W);
void Rodrigues2R(CvMat *W, CvMat *R);
// estimate geometric distances
double ErrorsGD(CvMat* P,CvMat* ObjectPoints,CvMat* ImgPoints,int N, CvMat* d,IplImage *img,int Viewflag);
// display
void CheckRtK(CvMat* Rt,CvMat* K,CvMat* ObjectPoints, IplImage *img);
// magnitude of residuals
double Residuals(CvMat* Rt,CvMat* K,CvMat* ObjectPoints,CvMat* ImgPoints,CvMat* d);
double Residuals(CvMat* Rt,CvMat* K,CvMat* ObjectPoints,CvMat* ImgPoints);


//<function R2Rodrigues & Rodrigues2R>
//Convert rotation matrix into Rodrigues representation and its inverse function
void R2Rodrigues(CvMat *R, CvMat *W){
    double norm;
    norm=acos((cvmGet(R,0,0)+cvmGet(R,1,1)+cvmGet(R,2,2)-1)/2);
    cvmSet(W,0,0,norm/(2*sin(norm))*(cvmGet(R,2,1)-cvmGet(R,1,2)));
    cvmSet(W,1,0,norm/(2*sin(norm))*(cvmGet(R,0,2)-cvmGet(R,2,0)));
    cvmSet(W,2,0,norm/(2*sin(norm))*(cvmGet(R,1,0)-cvmGet(R,0,1)));
}
void Rodrigues2R(CvMat *W, CvMat *R){
    double norm;
    CvMat* Wx = cvCreateMat(3,3,CV_64FC1);
    CvMat* Wx2 = cvCreateMat(3,3,CV_64FC1);
    CvMat* I = cvCreateMat(3,3,CV_64FC1);
    CvMat* temp = cvCreateMat(3,3,CV_64FC1);
    cvmSet(I,0,0,1);
    cvmSet(I,1,0,0);
    cvmSet(I,2,0,0);
    cvmSet(I,0,1,0);
    cvmSet(I,1,1,1);
    cvmSet(I,2,1,0);
    cvmSet(I,0,2,0);
    cvmSet(I,1,2,0);
    cvmSet(I,2,2,1);
    // ||W||
    norm=sqrt(cvmGet(W,0,0)*cvmGet(W,0,0)+cvmGet(W,1,0)*cvmGet(W,1,0)+cvmGet(W,2,0)*cvmGet(W,2,0));
    // R=I+sin(||W||)/||W||*Wx+(1-cos(||W||)/(||W||^2)*Wx2
    cvmSet(Wx,0,0,0);
    cvmSet(Wx,1,0,cvmGet(W,2,0));
    cvmSet(Wx,2,0,-cvmGet(W,1,0));
    cvmSet(Wx,0,1,-cvmGet(W,2,0));
    cvmSet(Wx,1,1,0);
    cvmSet(Wx,2,1,cvmGet(W,0,0));
    cvmSet(Wx,0,2,cvmGet(W,1,0));
    cvmSet(Wx,1,2,-cvmGet(W,0,0));
    cvmSet(Wx,2,2,0);
    cvMatMul(Wx,Wx,Wx2);
    //CvScalar CoeffWx,CoeffWx2;
    cvScaleAdd(Wx,cvRealScalar(sin(norm)/norm),I,temp);
    cvScaleAdd(Wx2,cvRealScalar((1-cos(norm))/(norm*norm)),temp,R);
}
//< Function ErrorsGD>
//Return errors w.r.t parameter vector P and display corners on the image
double ErrorsGD(CvMat* P,CvMat* ObjectPoints,CvMat* ImgPoints,int N,CvMat* d,int Viewflag){
    CvMat *R = cvCreateMat(3,3,CV_64FC1);
    CvMat *K = cvCreateMat(3,3,CV_64FC1);
    CvMat *W = cvCreateMat(3,1,CV_64FC1);
    CvMat *Rt = cvCreateMat(3,4,CV_64FC1);
    // set K
    cvmSet(K,0,0,cvmGet(P,0,0));
    cvmSet(K,1,0,0);
    cvmSet(K,2,0,0);
    cvmSet(K,0,1,cvmGet(P,4,0));
    cvmSet(K,1,1,cvmGet(P,1,0));
    cvmSet(K,2,1,0);
    cvmSet(K,0,2,cvmGet(P,2,0));
    cvmSet(K,1,2,cvmGet(P,3,0));
    cvmSet(K,2,2,1);
    // set R
    cvmSet(W,0,0,cvmGet(P,5,0));
    cvmSet(W,1,0,cvmGet(P,6,0));
    cvmSet(W,2,0,cvmGet(P,7,0));
    Rodrigues2R(W, R);
    // set homography Homography=K*[R(1),R(2),t]
    cvmSet(Rt,0,0,cvmGet(R,0,0));
    cvmSet(Rt,1,0,cvmGet(R,1,0));
    cvmSet(Rt,2,0,cvmGet(R,2,0));
    cvmSet(Rt,0,1,cvmGet(R,0,1));
    cvmSet(Rt,1,1,cvmGet(R,1,1));
    cvmSet(Rt,2,1,cvmGet(R,2,1));
    cvmSet(Rt,0,2,cvmGet(R,0,2));
    cvmSet(Rt,1,2,cvmGet(R,1,2));
    cvmSet(Rt,2,2,cvmGet(R,2,2));
    cvmSet(Rt,0,3,cvmGet(P,8,0));
    cvmSet(Rt,1,3,cvmGet(P,9,0));
    cvmSet(Rt,2,3,cvmGet(P,10,0));
//    if(Viewflag==1) CheckRtK(Rt,K,ObjectPoints,img);
    double RMSE=Residuals(Rt,K,ObjectPoints,ImgPoints,d);
    return RMSE;
}


//<Function RefineCamera>
//Input : - Rt : 3X4 matrix (R|t)
//- K : camera matrix containing intrinsic parameters
//- estK : estimated K
//- ObjectPoints : object coordinates of the calibration patterns
//- ImgPoints : image coordinates of the calibration patterns
//- img : image
//Output : refined parameters Rt and estK
void RefineCamera(CvMat *Rt, CvMat *K, CvMat *estK,CvMat *ObjectPoints, CvMat *ImgPoints, ofstream& myfile){
    int x_y_total = 2*REAL_NUM_CTRL_PTS;
    CvMat* J = cvCreateMat(x_y_total,11,CV_64FC1); // Jacobian Matrix
    CvMat* JT = cvCreateMat(11,x_y_total,CV_64FC1); // Transposed Jacobian Matrix
    CvMat* JTd = cvCreateMat(11,1,CV_64FC1);
    CvMat* Hessian = cvCreateMat(11,11,CV_64FC1); // Hessian Matrix
    CvMat* Hessianinv = cvCreateMat(11,11,CV_64FC1);
    CvMat* I = cvCreateMat(11,11,CV_64FC1); // Identity Matrix
    CvMat* Hessian_lm = cvCreateMat(11,11,CV_64FC1); // H_lm Matrix
    CvMat* Hessian_lminv = cvCreateMat(11,11,CV_64FC1); // H_lm Matrix
    
    CvMat* U = cvCreateMat(11,11,CV_64FC1);
    CvMat* D = cvCreateMat(11,11,CV_64FC1);
    CvMat* V = cvCreateMat(11,11,CV_64FC1);
    
    CvMat* P = cvCreateMat(11,1,CV_64FC1); // Parameters
    CvMat* Ptemp = cvCreateMat(11,1,CV_64FC1); // temporary P
    CvMat* dP = cvCreateMat(11,1,CV_64FC1); // delta P
    CvMat* R = cvCreateMat(3,3,CV_64FC1); // Rotation Matrix of the camera
    
    CvMat* Rtemp = cvCreateMat(3,3,CV_64FC1);
   
    CvMat* t = cvCreateMat(3,1,CV_64FC1); // Translation vector of the camera
    CvMat* W = cvCreateMat(3,1,CV_64FC1); // Rodrigues representation of R
    CvMat* Wtemp = cvCreateMat(3,1,CV_64FC1); // Rodrigues representation of R
    CvMat* d = cvCreateMat(x_y_total,1,CV_64FC1);
    CvMat* dtemp = cvCreateMat(x_y_total,1,CV_64FC1);
    CvMat* Homography = cvCreateMat(3,3,CV_64FC1);
    CvMat* HomographyTemp = cvCreateMat(3,3,CV_64FC1);
    double threshold=1e-3;
    double error,error2,delta=100;
    int Iteration=0;
    double lamda=0.01; // create initial damping factor
    int i,j;
    // setup identity matrix
    for(i=0;i<11;i++)
    {
        for(j=0;j<11;j++)
        {
            if(i==j) cvmSet(I,i,j,1);
            else cvmSet(I,i,j,0);
        }
    }
    // Convert R to W
    cvmSet(R,0,0,cvmGet(Rt,0,0));
    cvmSet(R,1,0,cvmGet(Rt,1,0));
    cvmSet(R,2,0,cvmGet(Rt,2,0));
    cvmSet(R,0,1,cvmGet(Rt,0,1));
    cvmSet(R,1,1,cvmGet(Rt,1,1));
    cvmSet(R,2,1,cvmGet(Rt,2,1));
    cvmSet(R,0,2,cvmGet(Rt,0,2));
    cvmSet(R,1,2,cvmGet(Rt,1,2));
    cvmSet(R,2,2,cvmGet(Rt,2,2));
    cvmSet(t,0,0,cvmGet(Rt,0,3));
    cvmSet(t,1,0,cvmGet(Rt,1,3));
    cvmSet(t,2,0,cvmGet(Rt,2,3));
    R2Rodrigues(R, W);
    // Assign Initial Values
    cvmSet(P,0,0,cvmGet(K,0,0)); //au (alpha)
    cvmSet(P,1,0,cvmGet(K,1,1)); //av (beta)
    cvmSet(P,2,0,cvmGet(K,0,2)); //u0
    cvmSet(P,3,0,cvmGet(K,1,2)); //v0
    cvmSet(P,4,0,cvmGet(K,0,1)); //sk (lamda)
    cvmSet(P,5,0,cvmGet(W,0,0)); //wx
    cvmSet(P,6,0,cvmGet(W,1,0)); //wy
    cvmSet(P,7,0,cvmGet(W,2,0)); //wz
    cvmSet(P,8,0,cvmGet(t,0,0)); //tx
    cvmSet(P,9,0,cvmGet(t,1,0)); //ty
    cvmSet(P,10,0,cvmGet(t,2,0)); //tz
    // Display Initial parameter values
//    printf("Initial parameter values\n");
//    for(i=0;i<11;i++) {
//        printf("P(%2d)=%f\n",i,cvmGet(P,i,0));
//    }
    // Initial geometric distance
    error=ErrorsGD(P,ObjectPoints,ImgPoints,REAL_NUM_CTRL_PTS,d,0);
    
    
    
    // Iterate using LM method
    int updateJ=1;
    for(j=0;j<200;j++)
    {
        if(updateJ==1)
        {
            // determine Jacobian matrix J
            setJ(P,J,ObjectPoints);
            // Hessian Matrix
            cvTranspose(J,JT);
            cvMatMul(JT,J,Hessian);
            // apply damping factor
            cvScaleAdd(I,cvScalar(lamda),Hessian,Hessian_lm);
            cvInvert(Hessian_lm,Hessian_lminv);
        }
        // temporary update P Ptemp=P+inv(H_lm)*(J'*d)
        cvMatMul(JT,d,JTd);
        cvMatMul(Hessian_lminv,JTd,dP);
        cvScaleAdd(dP,cvScalar(-1),P,Ptemp);
        delta=0;
        for(i=0;i<11;i++) delta=delta+pow(cvmGet(dP,i,0),2);
        delta=sqrt(delta)/11;
        // check errors by Ptemp
        error2 = ErrorsGD(Ptemp,ObjectPoints,ImgPoints,REAL_NUM_CTRL_PTS,d,0);
        if(error2<error) //decrease damping factor and update
        {
            lamda=lamda/10;
            for(i=0;i<11;i++) cvmSet(P,i,0,cvmGet(Ptemp,i,0));
            updateJ=1;
        }
        else // increase damping factor and try again
        {
            updateJ=0;
            lamda=lamda*10;
        }
        if(delta<threshold) break;
    }
//    cout<<"Error RMS: "<<error2<<endl;
//    save_refinements_parameters_add_row(myfile, j, P, error, delta);
    
//    printf("Number of iteration = %d\n",j);
//    printf("delta = %f\n",delta);
//    // Display Refined parameter values
//    printf("Refined parameter values\n");
//    for(i=0;i<11;i++){
//        printf("P(%2d)=%f\n",i,cvmGet(P,i,0));
//    }
    // Finally estimated K
    cvmSet(estK,0,0,cvmGet(P,0,0));
    cvmSet(estK,1,0,cvmGet(K,1,0));
    cvmSet(estK,2,0,cvmGet(K,2,0));
    cvmSet(estK,0,1,cvmGet(P,4,0));
    cvmSet(estK,1,1,cvmGet(P,1,0));
    cvmSet(estK,2,1,cvmGet(K,2,1));
    cvmSet(estK,0,2,cvmGet(P,2,0));
    cvmSet(estK,1,2,cvmGet(P,3,0));
    cvmSet(estK,2,2,cvmGet(K,2,2));
    
    // Update Rt
    cvmSet(W,0,0,cvmGet(P,5,0));
    cvmSet(W,1,0,cvmGet(P,6,0));
    cvmSet(W,2,0,cvmGet(P,7,0));
    Rodrigues2R(W,R);
    cvmSet(Rt,0,0,cvmGet(R,0,0));
    cvmSet(Rt,1,0,cvmGet(R,1,0));
    cvmSet(Rt,2,0,cvmGet(R,2,0));
    cvmSet(Rt,0,1,cvmGet(R,0,1));
    cvmSet(Rt,1,1,cvmGet(R,1,1));
    cvmSet(Rt,2,1,cvmGet(R,2,1));
    cvmSet(Rt,0,2,cvmGet(R,0,2));
    cvmSet(Rt,1,2,cvmGet(R,1,2));
    cvmSet(Rt,2,2,cvmGet(R,2,2));
    cvmSet(Rt,0,3,cvmGet(P,8,0));
    cvmSet(Rt,1,3,cvmGet(P,9,0));
    cvmSet(Rt,2,3,cvmGet(P,10,0));
}


void RefineCameraAll(CvMat *Rt[], CvMat *K, CvMat *estK,CvMat *ObjectPoints, CvMat *ImgPoints,
                     ofstream& myfile, int M_images, double& error0, double& error){
    // Refine Parameters (using entire images)
    int x_y_total = 2*REAL_NUM_CTRL_PTS;
    
    CvMat* J = cvCreateMat(x_y_total*M_images,5+6*M_images,CV_64FC1); // Jacobian Matrix
    CvMat* JT = cvCreateMat(5+6*M_images,x_y_total*M_images,CV_64FC1); // Transposed Jacobian Matrix
    CvMat* JTd = cvCreateMat(5+6*M_images,1,CV_64FC1);
    CvMat* Hessian = cvCreateMat(5+6*M_images,5+6*M_images,CV_64FC1); // Hessian Matrix
    CvMat* Hessianinv = cvCreateMat(5+6*M_images,5+6*M_images,CV_64FC1);
    CvMat* I = cvCreateMat(5+6*M_images,5+6*M_images,CV_64FC1); // Identity Matrix
    CvMat* Hessian_lm = cvCreateMat(5+6*M_images,5+6*M_images,CV_64FC1); // H_lm Matrix
    CvMat* Hessian_lminv = cvCreateMat(5+6*M_images,5+6*M_images,CV_64FC1); // H_lm Matrix
    CvMat* P = cvCreateMat(5+6*M_images,1,CV_64FC1); // Parameters
    CvMat* Ptemp = cvCreateMat(5+6*M_images,1,CV_64FC1); // temporary P
    CvMat* dP = cvCreateMat(5+6*M_images,1,CV_64FC1); // delta P
    CvMat* Ppartial= cvCreateMat(5+6*M_images,1,CV_64FC1); // Partial P
    CvMat* Rtemp = cvCreateMat(3,3,CV_64FC1);
    CvMat* W = cvCreateMat(3,1,CV_64FC1); // Rodrigues representation of R
    CvMat* Wtemp = cvCreateMat(3,1,CV_64FC1); // Rodrigues representation of R
    CvMat* d = cvCreateMat(x_y_total*M_images,1,CV_64FC1);
    CvMat* dtemp = cvCreateMat(x_y_total*M_images,1,CV_64FC1);
    CvMat* dpartial= cvCreateMat(x_y_total,1,CV_64FC1);
    CvMat* Homography = cvCreateMat(3,3,CV_64FC1);
    CvMat* HomographyTemp = cvCreateMat(3,3,CV_64FC1);
    CvMat* R = cvCreateMat(3,3,CV_64FC1);
    CvMat* t = cvCreateMat(3,1,CV_64FC1); // Translation vector of the camera
    double dtdrefined[M_images], dtd[M_images];
    
    double threshold=1e-6;
    double error2,delta=100;
    double damping=0.01; // create initial damping factor
    int i,j,k;
    // setup identity matrix
    for(i=0;i<5+6*M_images;i++)
    {
        for(j=0;j<5+6*M_images;j++)
        {
            if(i==j) cvmSet(I,i,j,1);
            else cvmSet(I,i,j,0);
        }
    }
    // setup J
    // Assign Initial Values
    cvmSet(P,0,0,cvmGet(K,0,0)); //au (alpha)
    cvmSet(P,1,0,cvmGet(K,1,1)); //av (beta)
    cvmSet(P,2,0,cvmGet(K,0,2)); //u0
    cvmSet(P,3,0,cvmGet(K,1,2)); //v0
    cvmSet(P,4,0,cvmGet(K,0,1)); //sk (lamda)
    for(i=0;i<M_images;i++)
    {
        // Convert R to W
        cvmSet(R,0,0, cvmGet(Rt[i],0,0));
        cvmSet(R,1,0,cvmGet(Rt[i],1,0));
        cvmSet(R,2,0,cvmGet(Rt[i],2,0));
        cvmSet(R,0,1,cvmGet(Rt[i],0,1));
        cvmSet(R,1,1,cvmGet(Rt[i],1,1));
        cvmSet(R,2,1,cvmGet(Rt[i],2,1));
        cvmSet(R,0,2,cvmGet(Rt[i],0,2));
        cvmSet(R,1,2,cvmGet(Rt[i],1,2));
        cvmSet(R,2,2,cvmGet(Rt[i],2,2));
        cvmSet(t,0,0,cvmGet(Rt[i],0,3));
        cvmSet(t,1,0,cvmGet(Rt[i],1,3));
        cvmSet(t,2,0,cvmGet(Rt[i],2,3));
        R2Rodrigues(R, W);
        cvmSet(P,5+6*i ,0,cvmGet(W,0,0)); //wx
        cvmSet(P,6+6*i ,0,cvmGet(W,1,0)); //wy
        cvmSet(P,7+6*i ,0,cvmGet(W,2,0)); //wz
        cvmSet(P,8+6*i ,0,cvmGet(t,0,0)); //tx
        cvmSet(P,9+6*i ,0,cvmGet(t,1,0)); //ty
        cvmSet(P,10+6*i,0,cvmGet(t,2,0)); //tz
    }
    // Initial geometric distance
    error0=0;
    for(i=0;i<M_images;i++)
    {
        cvmSet(Ppartial,0,0,cvmGet(P,0,0));
        cvmSet(Ppartial,1,0,cvmGet(P,1,0));
        cvmSet(Ppartial,2,0,cvmGet(P,2,0));
        cvmSet(Ppartial,3,0,cvmGet(P,3,0));
        cvmSet(Ppartial,4,0,cvmGet(P,4,0));
        cvmSet(Ppartial,5,0,cvmGet(P,5+i*6,0));
        cvmSet(Ppartial,6,0,cvmGet(P,6+i*6,0));
        cvmSet(Ppartial,7,0,cvmGet(P,7+i*6,0));
        cvmSet(Ppartial,8,0,cvmGet(P,8+i*6,0));
        cvmSet(Ppartial,9,0,cvmGet(P,9+i*6,0));
        cvmSet(Ppartial,10,0,cvmGet(P,10+i*6,0));
        error0 = error0 + ErrorsGD(Ppartial,ObjectPoints,ImgPoints,REAL_NUM_CTRL_PTS,dpartial,1);
        for(j=0;j<REAL_NUM_CTRL_PTS;j++)
        {
            cvmSet(d,x_y_total*i+j*2 , 0 , cvmGet(dpartial,j*2 ,0));
            cvmSet(d,x_y_total*i+j*2+1 , 0 , cvmGet(dpartial,j*2+1 ,0));
        }
    }
    printf("Initial error=%f\n",error0);
    // Iterate using LM method
    int updateJ=1;
    error=error0;
    for(i=0;i<200;i++)
    {
        if(updateJ==1)
        {
            // determine Jacobian matrix J
            setJall(P,J,M_images,ObjectPoints);
            // Hessian Matrix
            cvTranspose(J,JT);
            cvMatMul(JT,J,Hessian);
        }
        // apply damping factor
        cvScaleAdd(I,cvScalar(damping),Hessian,Hessian_lm);
        cvInvert(Hessian_lm,Hessian_lminv);
        // temporary update P Ptemp=P-inv(H_lm)*(J'*d)
        cvMatMul(JT,d,JTd);
        cvMatMul(Hessian_lminv,JTd,dP);
        cvScaleAdd(dP,cvScalar(-1),P,Ptemp);
        delta=0;
        for(j=0;j<5+6*M_images;j++) delta=delta+pow(cvmGet(dP,j,0),2);
        delta=sqrt(delta)/(5+6*M_images);
        // check errors by Ptemp
        error2=0;
        for(j=0;j<M_images;j++)
        {
            cvmSet(Ppartial,0,0,cvmGet(Ptemp,0,0));
            cvmSet(Ppartial,1,0,cvmGet(Ptemp,1,0));
            cvmSet(Ppartial,2,0,cvmGet(Ptemp,2,0));
            cvmSet(Ppartial,3,0,cvmGet(Ptemp,3,0));
            cvmSet(Ppartial,4,0,cvmGet(Ptemp,4,0));
            cvmSet(Ppartial,5,0,cvmGet(Ptemp,5+j*6,0));
            cvmSet(Ppartial,6,0,cvmGet(Ptemp,6+j*6,0));
            cvmSet(Ppartial,7,0,cvmGet(Ptemp,7+j*6,0));
            cvmSet(Ppartial,8,0,cvmGet(Ptemp,8+j*6,0));
            cvmSet(Ppartial,9,0,cvmGet(Ptemp,9+j*6,0));
            cvmSet(Ppartial,10,0,cvmGet(Ptemp,10+j*6,0));
//            ErrorsGD(CvMat* P,CvMat* ObjectPoints,CvMat* ImgPoints,int N,CvMat* d,int Viewflag)
            error2 = error2 + ErrorsGD(Ppartial,ObjectPoints,ImgPoints,REAL_NUM_CTRL_PTS,dpartial,1);
            for(k=0;k<REAL_NUM_CTRL_PTS;k++)
            {
                cvmSet(dtemp,x_y_total*j+k*2 , 0 , cvmGet(dpartial,k*2 ,0));
                cvmSet(dtemp,x_y_total*j+k*2+1 , 0 , cvmGet(dpartial,k*2+1 ,0));
            }
        }
        if(error2<error) //decrease damping factor and update
        {
            damping=damping/10;
            for(j=0;j<5+6*M_images;j++){
                cvmSet(P,j,0,cvmGet(Ptemp,j,0));
            }
            for(j=0;j<x_y_total*M_images;j++) {
                cvmSet(d,j,0,cvmGet(dtemp,j,0));
            }
            error=error2;
            updateJ=1;
//            printf("iteration %d ] ErrorsGD=%f\n",i+1,error);
//            printf(" delta=%f\n",delta);
        }
        else // increase damping factor and try again
        {
            updateJ=0;
            i=i-1;
            damping=damping*10;
        }
        if(delta<threshold) break;
    }
    printf("Number of iteration = %d\n",i);
//    printf("delta = %f\n",delta);x
    printf("error=%f\n",error/(M_images*REAL_NUM_CTRL_PTS));
//    printf("damping=%f\n",damping);
    // Finally estimated K
    cvmSet(K,0,0,cvmGet(P,0,0));
    cvmSet(K,0,1,cvmGet(P,4,0));
    cvmSet(K,1,1,cvmGet(P,1,0));
    cvmSet(K,0,2,cvmGet(P,2,0));
    cvmSet(K,1,2,cvmGet(P,3,0));
    // Update Rt
    for(i=0;i<M_images;i++)
    {
        cvmSet(W,0,0,cvmGet(P,5+6*i,0));
        cvmSet(W,1,0,cvmGet(P,6+6*i,0));
        cvmSet(W,2,0,cvmGet(P,7+6*i,0));
        Rodrigues2R(W,R);
        cvmSet(Rt[i],0,0,cvmGet(R,0,0));
        cvmSet(Rt[i],1,0,cvmGet(R,1,0));
        cvmSet(Rt[i],2,0,cvmGet(R,2,0));
        cvmSet(Rt[i],0,1,cvmGet(R,0,1));
        cvmSet(Rt[i],1,1,cvmGet(R,1,1));
        cvmSet(Rt[i],2,1,cvmGet(R,2,1));
        cvmSet(Rt[i],0,2,cvmGet(R,0,2));
        cvmSet(Rt[i],1,2,cvmGet(R,1,2));
        cvmSet(Rt[i],2,2,cvmGet(R,2,2));
        cvmSet(Rt[i],0,3,cvmGet(P,8+6*i,0));
        cvmSet(Rt[i],1,3,cvmGet(P,9+6*i,0));
        cvmSet(Rt[i],2,3,cvmGet(P,10+6*i,0));
    }
    
//    fprintf(report,"\nRefined Instrinsic Parameters\n",tempimgfn);
//    fprintf(report,"| | | %16.8f %16.8f %16.8f |\n",cvmGet(K,0,0),cvmGet(K,0,1),cvmGet(K,0,2));
//    fprintf(report,"| K | = | %16.8f %16.8f %16.8f |\n",cvmGet(K,1,0),cvmGet(K,1,1),cvmGet(K,1,2));
//    fprintf(report,"| | | %16.8f %16.8f %16.8f |\n",cvmGet(K,2,0),cvmGet(K,2,1),cvmGet(K,2,2));
//    imgfn=fopen(imagefilenames,"rt");
    
    for(i=0;i<M_images;i++)
    {
//        fscanf(imgfn,"%s",tempimgfn);
//        // 3. Extrinsic parameters
//        fprintf(report,"\nRefined Extrinsic Parameters (%s) \n",tempimgfn);
//        fprintf(report,"| | | %16.8f %16.8f %16.8f %16.8f|\n",cvmGet(Rt[i],0,0),cvmGet(Rt[i],0,1),cvmGet(Rt[i],0,2),cvmGet(Rt[i],0,3));
//        fprintf(report,"| R | t | = | %16.8f %16.8f %16.8f %16.8f |\n",cvmGet(Rt[i],1,0),cvmGet(Rt[i],1,1),cvmGet(Rt[i],1,2),cvmGet(Rt[i],1,3));
//        fprintf(report,"| | | %16.8f %16.8f %16.8f %16.8f|\n",cvmGet(Rt[i],2,0),cvmGet(Rt[i],2,1),cvmGet(Rt[i],2,2),cvmGet(Rt[i],2,3));
//
        dtdrefined[i] = Residuals(Rt[i],K,ObjectPoints,ImgPoints);
//        cout<<"error refined all: "<<dtdrefined[i]<<endl;
        
//        fprintf(report,"errors(dtd) = %f pixels (%4.2f%%)\n",dtdrefined[i],dtdrefined[i]/dtd[i]*100);
//        fprintf(report,"\n");
    }
    double improvementNumer=0;
    double improvementDenom=0;
    for(i=0;i<M_images;i++)
    {
        improvementNumer=improvementNumer+dtdrefined[i];
        improvementDenom=improvementDenom+dtd[i];
    }
//    fprintf(report,"Improvement = sqrt( sum of refined errors ) / sqrt(sum of initial errors) = %f.\n",sqrt(error/error0));
}
//< Function Residuals>
//Return errors w.r.t the Rt, K , ObjectPoints and ImgPoints
double Residuals(CvMat* Rt,CvMat* K,CvMat* ObjectPoints,CvMat* ImgPoints){
    int i;
    CvMat* Homography=cvCreateMat(3,3,CV_64FC1);
    CvMat* temp=cvCreateMat(3,3,CV_64FC1);
    CvMat* estImgPoints=cvCreateMat(3,REAL_NUM_CTRL_PTS,CV_64FC1);
    cvmSet(temp,0,0,cvmGet(Rt,0,0));
    cvmSet(temp,1,0,cvmGet(Rt,1,0));
    cvmSet(temp,2,0,cvmGet(Rt,2,0));
    cvmSet(temp,0,1,cvmGet(Rt,0,1));
    cvmSet(temp,1,1,cvmGet(Rt,1,1));
    cvmSet(temp,2,1,cvmGet(Rt,2,1));
    cvmSet(temp,0,2,cvmGet(Rt,0,3));
    cvmSet(temp,1,2,cvmGet(Rt,1,3));
    cvmSet(temp,2,2,cvmGet(Rt,2,3));
    cvMatMul(K,temp,Homography);
    cvMatMul(Homography,ObjectPoints,estImgPoints);
    double dtd=0;
    for(i=0;i<REAL_NUM_CTRL_PTS;i++)
    {
        dtd=dtd+pow((cvmGet(estImgPoints,0,i)/cvmGet(estImgPoints,2,i))-cvmGet(ImgPoints,0,i),2);
        dtd=dtd+pow((cvmGet(estImgPoints,1,i)/cvmGet(estImgPoints,2,i))-cvmGet(ImgPoints,1,i),2);
    };
    return dtd;
}
double computeReprojectionErrors(CvMat* Rt[],CvMat* K,CvMat* ObjectPoints,
                                 vector<vector<Point2f>> control_points_images, int n_images){
    
    CvMat* ImgPoints = cvCreateMat(3,REAL_NUM_CTRL_PTS,CV_64FC1); // image coordinates
    CvMat* Homography=cvCreateMat(3,3,CV_64FC1);
    CvMat* temp=cvCreateMat(3,3,CV_64FC1);
    CvMat* estImgPoints=cvCreateMat(3,REAL_NUM_CTRL_PTS,CV_64FC1);
    double error=0, total_error=0;
    
    
    
    for (int i=0; i<n_images; i++) {
        
        for(int j=0;j<REAL_NUM_CTRL_PTS;j++)
        {
            cvmSet(ImgPoints,0,j,control_points_images[i][j].x);
            cvmSet(ImgPoints,1,j,control_points_images[i][j].y);
            cvmSet(ImgPoints,2,j,1);
        }
        cvmSet(temp,0,0,cvmGet(Rt[i],0,0));
        cvmSet(temp,1,0,cvmGet(Rt[i],1,0));
        cvmSet(temp,2,0,cvmGet(Rt[i],2,0));
        cvmSet(temp,0,1,cvmGet(Rt[i],0,1));
        cvmSet(temp,1,1,cvmGet(Rt[i],1,1));
        cvmSet(temp,2,1,cvmGet(Rt[i],2,1));
        cvmSet(temp,0,2,cvmGet(Rt[i],0,3));
        cvmSet(temp,1,2,cvmGet(Rt[i],1,3));
        cvmSet(temp,2,2,cvmGet(Rt[i],2,3));
        cvMatMul(K,temp,Homography); //K*Rt
        cvMatMul(Homography,ObjectPoints,estImgPoints);//projectar:
        //add radial distortion
        
        error = cvNorm(ImgPoints,estImgPoints,CV_L2);
//        cout<<"estimate error: "<<error<<endl;
        total_error += error*error;
    }
    return sqrt(total_error/(n_images*REAL_NUM_CTRL_PTS));
    
}
//< Function Residuals>
//Return errors & d vector w.r.t the Rt, K , ObjectPoints and ImgPoints
double Residuals(CvMat* Rt,CvMat* K,CvMat* ObjectPoints,CvMat* ImgPoints,CvMat* d){
    int i;
    
    CvMat* Homography=cvCreateMat(3,3,CV_64FC1);
    CvMat* temp=cvCreateMat(3,3,CV_64FC1);
    CvMat* estImgPoints=cvCreateMat(3,REAL_NUM_CTRL_PTS,CV_64FC1);
    cvmSet(temp,0,0,cvmGet(Rt,0,0));
    cvmSet(temp,1,0,cvmGet(Rt,1,0));
    cvmSet(temp,2,0,cvmGet(Rt,2,0));
    cvmSet(temp,0,1,cvmGet(Rt,0,1));
    cvmSet(temp,1,1,cvmGet(Rt,1,1));
    cvmSet(temp,2,1,cvmGet(Rt,2,1));
    cvmSet(temp,0,2,cvmGet(Rt,0,3));
    cvmSet(temp,1,2,cvmGet(Rt,1,3));
    cvmSet(temp,2,2,cvmGet(Rt,2,3));
    cvMatMul(K,temp,Homography);
    cvMatMul(Homography,ObjectPoints,estImgPoints);
    for(i=0;i<REAL_NUM_CTRL_PTS;i++)
    {
        // d : input-estimated
        cvmSet(d,i*2,0,cvmGet(ImgPoints,0,i)-cvmGet(estImgPoints,0,i)/cvmGet(estImgPoints,2,i));
        cvmSet(d,i*2+1,0,cvmGet(ImgPoints,1,i)-cvmGet(estImgPoints,1,i)/cvmGet(estImgPoints,2,i));
    }
    //return d'*d
    double dtd=0;
    for(i=0;i<2*REAL_NUM_CTRL_PTS;i++)
        dtd=dtd+pow(cvmGet(d,i,0),2);
    return dtd;
}

#endif /* cameraCalib_h */
