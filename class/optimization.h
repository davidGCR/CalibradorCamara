//
//  optimization.h
//  testOpencv
//
//  Created by David Choqueluque Roman on 2/9/19.
//  Copyright © 2019 David Choqueluque Roman. All rights reserved.
//

#ifndef optimization_h
#define optimization_h


#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"

//#include <cv.h>
#include <iostream>
#include "jacobian.h"
using namespace cv;
using namespace std;

// Homography
void Homography(double **datapoints, int N, CvMat* H);
void Homography(double **datapoints1, double **datapoint2, int N, CvMat* H);
void Homography(CvMat* datapoints1, CvMat* datapoints2, int N, CvMat* H);
// construct Normalization matrix for image coordinates
double NormalizationMatrixImg(int **Corners1,int N1,CvMat *MT);
double NormalizationMatrixImg(double **Corners1,int N1,CvMat *MT);
double NormalizationMatrixImg(CvMat *Corners1,int N1,CvMat *MT);
// draw detected lines(Hough) on the edge image (Canny)
void Edge_HoughCanny(IplImage *img, double **Corners, int viewEdgeFlag, int viewHoughFlag, int viewRefiedHoughFlag);
// calibration with only corner points
void Calib_Corners(char *imagefilenames, char* reportfilename,int Nfiles, int viewCornersFlag);
// set V matrix & b vector
void SetV(CvMat*H, int i, CvMat *V);
// transformation from Rotation matrix to Rodrigues representation
void R2Rodrigues(CvMat *R, CvMat *W);
void Rodrigues2R(CvMat *W, CvMat *R);
// REFINE camera parameters (Levenverg-Marquit Method)
void RefineCamera(CvMat *Rt, CvMat *K,CvMat *estK,CvMat *ObjectPoints, IplImage *img);
// Set Jacobian matrix
void setJall(CvMat *P,CvMat *J, int Nfiles, CvMat *ObjectPoints);
void setJ(CvMat *P,CvMat *J,CvMat *ObjectPoints);
// estimate geometric distances
double ErrorsGD(CvMat* P,CvMat* ObjectPoints,CvMat* ImgPoints,int N, CvMat* d,IplImage *img,int Viewflag);
// display
void CheckRtK(CvMat* Rt,CvMat* K,CvMat* ObjectPoints, IplImage *img);
// magnitude of residuals
double Residuals(CvMat* Rt,CvMat* K,CvMat* ObjectPoints,CvMat* ImgPoints,CvMat* d);
double Residuals(CvMat* Rt,CvMat* K,CvMat* ObjectPoints,CvMat* ImgPoints); 

//<Function Edge_HoughCanny>
//Input : - src : source image
//- Corners : 2 dimensional double array for corners in the image
//- viewHoughFlag : if 1, show lines detected by the Hough transformation
//- viewRefiedHoughFlag : if 1, show refined lines
//Output : - Corners (sorted)


void Calib_Corners(char *imagefilenames, char* reportfilename, int Nfiles, int viewCornersFlag, int Allflag)
{
    int i,j,k;
    double ***Corners;
    FILE *imgfn=fopen(imagefilenames,"rt");
    char tempimgfn[20];
    IplImage* img1,*temp,*temp2;
    Corners=new double **[Nfiles];
    for(i=0;i<Nfiles;i++) Corners[i]=new double *[80];
    for(i=0;i<Nfiles;i++)
    {
        for(j=0;j<100;j++) Corners[i][j]=new double [2];
    }
    // assign matrices
    CvMat* T = cvCreateMat(3,3,CV_64FC1); // Normalizing matrix for image coord.
    CvMat* Tp = cvCreateMat(3,3,CV_64FC1); // Normalizing matrix for object coord.
    CvMat* Tinv = cvCreateMat(3,3,CV_64FC1); // inverse of T
    CvMat* Tpinv = cvCreateMat(3,3,CV_64FC1); // inverse of Tp
    CvMat* ObjectPoints = cvCreateMat(3,80,CV_64FC1); // object coordinates
    CvMat* ImgPoints = cvCreateMat(3,80,CV_64FC1); // image coordinates
    CvMat* nObjectPoints = cvCreateMat(3,80,CV_64FC1); // normalized object coordinates
    CvMat* nImgPoints = cvCreateMat(3,80,CV_64FC1); // normalized image coordinates
    CvMat* Hn = cvCreateMat(3,3,CV_64FC1); // estimated homography by normalized coord.
    CvMat* Hntemp = cvCreateMat(3,3,CV_64FC1); // temporary matrix for calulation only
    CvMat *H[39]; // I don't know how to dynamically assign CvMat
    CvMat* Hinv = cvCreateMat(3,3,CV_64FC1); // inverse of the estimated Homography
    CvMat* V = cvCreateMat(2*Nfiles,6,CV_64FC1); // Vb=0 for camera calibration
    CvMat* b = cvCreateMat(6,1,CV_64FC1); // elements of the absolute conic
    
    for(i=0;i<10;i++)
    {
        for(j=0;j<8;j++)
        {
            cvmSet(ObjectPoints,0,i*8+j,j*21.5/9);
            cvmSet(ObjectPoints,1,i*8+j,i*21.5/9);
            cvmSet(ObjectPoints,2,i*8+j,1);
        }
    }
    
    for(i=0;i<Nfiles;i++)
    {
        // Load images
        fscanf(imgfn,"%s",tempimgfn);
        img1 = cvLoadImage(tempimgfn,0);
        temp = cvLoadImage(tempimgfn,0);
        temp2 = cvLoadImage(tempimgfn,0);
        printf("Processing image %s. \n",tempimgfn);
        // Find & display lines and corner points
//        Edge_HoughCanny(temp,Corners[i],1,1,1); //ONOFF
        // display corners
        CvPoint Draw;
        char text[20];
        CvFont font;
        cvInitFont(&font, CV_FONT_HERSHEY_PLAIN, 0.8, 0.8, 0, 1);
        FILE *pointid;
        pointid=fopen("pointid.txt","rt");
        cvPutText(temp,tempimgfn,cvPoint(20,20),&font,CV_RGB(255,255,255));
        for(j=0;j<80;j++)
        {
            fscanf(pointid,"%s",text);
            Draw.x=(int)(Corners[i][j][0]+.5);
            Draw.y=(int)(Corners[i][j][1]+.5);
            
//            draw_cross(temp,Draw,CV_RGB(255,255,255),3);
            
            cvPutText(temp,text,Draw,&font,CV_RGB(255,255,255));
        }
        if(viewCornersFlag==1)
        {
            cvNamedWindow( "Corner Points", 1 );
            cvShowImage( "Corner Points", temp );
            cvWaitKey(0);
        }
        // Estimate Homography
        // 1. Normalize
        for(j=0;j<80;j++)
        {
            cvmSet(ImgPoints,0,j,Corners[i][j][0]);
            cvmSet(ImgPoints,1,j,Corners[i][j][1]);
            cvmSet(ImgPoints,2,j,1);
        }
        //calculate normalization matrix for Real points
        NormalizationMatrixImg(ObjectPoints,80,T);
        cvMatMul(T,ObjectPoints,nObjectPoints);
        cvInvert(T,Tinv);
        
        //calculate normalization matrix for ImagePoints
        NormalizationMatrixImg(ImgPoints,80,Tp);
        cvMatMul(Tp,ImgPoints,nImgPoints);
        cvInvert(Tp,Tpinv);
        // 2. estimate Hn (normalize)
        Homography(nObjectPoints,nImgPoints,80,Hn);
        //3. estimate H=inv(Tp)*Hn*T (Desnormalize)
        cvMatMul(Tpinv,Hn,Hntemp);
        H[i] = cvCreateMat(3,3,CV_64FC1);
        cvMatMul(Hntemp,T,H[i]);
        // Construct Vb=0
        SetV(H[i], i, V);
    }
    //////////////////////////////////////////////
    // estimate b (elements of the absolute conic)
    CvMat* U = cvCreateMat(Nfiles*2,Nfiles*2,CV_64FC1);
    CvMat* D = cvCreateMat(Nfiles*2,6,CV_64FC1);
    CvMat* V2 = cvCreateMat(6,6,CV_64FC1);
    CvMat* B = cvCreateMat(3,3,CV_64FC1);
    cvSVD(V, D, U, V2, CV_SVD_V_T);
    cvmSet(b,0,0,cvmGet(V2,5,0));
    cvmSet(b,1,0,cvmGet(V2,5,1));
    cvmSet(b,2,0,cvmGet(V2,5,2));
    cvmSet(b,3,0,cvmGet(V2,5,3));
    cvmSet(b,4,0,cvmGet(V2,5,4));
    cvmSet(b,5,0,cvmGet(V2,5,5));
    // setup B
    cvmSet(B,0,0,cvmGet(b,0,0));
    cvmSet(B,0,1,cvmGet(b,1,0));
    cvmSet(B,1,0,cvmGet(b,1,0));
    cvmSet(B,1,1,cvmGet(b,2,0));
    cvmSet(B,0,2,cvmGet(b,3,0));
    cvmSet(B,2,0,cvmGet(b,3,0));
    cvmSet(B,1,2,cvmGet(b,4,0));
    cvmSet(B,2,1,cvmGet(b,4,0));
    cvmSet(B,2,2,cvmGet(b,5,0));
    //////////////////////////////////////////////
    // estimate intrinsic parameters
    CvMat* K = cvCreateMat(3,3,CV_64FC1);
    CvMat* Kinv = cvCreateMat(3,3,CV_64FC1);
    double vo = (cvmGet(B,0,1)*cvmGet(B,0,2)-cvmGet(B,0,0)*cvmGet(B,1,2))/(cvmGet(B,0,0)*cvmGet(B,1,1)-
                                                                           cvmGet(B,0,1)*cvmGet(B,0,1));
    double lamda = cvmGet(B,2,2)-(cvmGet(B,0,2)*cvmGet(B,0,2)+vo*(cvmGet(B,0,1)*cvmGet(B,0,2)-
                                                                  cvmGet(B,0,0)*cvmGet(B,1,2)))/cvmGet(B,0,0);
    double alpha = sqrt(lamda/cvmGet(B,0,0));
    double beta = sqrt(lamda*cvmGet(B,0,0)/(cvmGet(B,0,0)*cvmGet(B,1,1)-cvmGet(B,0,1)*cvmGet(B,0,1)));
    double gamma = -cvmGet(B,0,1)*alpha*alpha*beta/lamda;
    double uo = gamma*vo/beta-cvmGet(B,0,2)*alpha*alpha/lamda;
    cvmSet(K,0,0,alpha);
    cvmSet(K,0,1,gamma);
    cvmSet(K,0,2,uo);
    cvmSet(K,1,0,0);
    cvmSet(K,1,1,beta);
    cvmSet(K,1,2,vo);
    cvmSet(K,2,0,0);
    cvmSet(K,2,1,0);
    cvmSet(K,2,2,1);
    cvInvert(K,Kinv);
    //////////////////////////////////////////////
    // estimate extrinsic parameters
    CvMat* h1 = cvCreateMat(3,1,CV_64FC1);
    CvMat* h2 = cvCreateMat(3,1,CV_64FC1);
    CvMat* h3 = cvCreateMat(3,1,CV_64FC1);
    CvMat* r1 = cvCreateMat(3,1,CV_64FC1);
    CvMat* r2 = cvCreateMat(3,1,CV_64FC1);
    CvMat* r3 = cvCreateMat(3,1,CV_64FC1);
    CvMat* t = cvCreateMat(3,1,CV_64FC1);
    CvMat* Q = cvCreateMat(3,3,CV_64FC1);
    CvMat* R = cvCreateMat(3,3,CV_64FC1);
    CvMat* UU = cvCreateMat(3,3,CV_64FC1);
    CvMat* DD = cvCreateMat(3,3,CV_64FC1);
    CvMat* VV = cvCreateMat(3,3,CV_64FC1);
    CvMat* Vt = cvCreateMat(3,3,CV_64FC1);
    CvMat* Rt[39];
    double lamda1;
    imgfn=fopen(imagefilenames,"rt");
    for(i=0;i<Nfiles;i++)
    {
        fscanf(imgfn,"%s",tempimgfn);
        temp = cvLoadImage(tempimgfn,0);
        Rt[i] = cvCreateMat(3,4,CV_64FC1);
        // setup column vector h1,h2,h3 from H[i]
        cvmSet(h1,0,0,cvmGet(H[i],0,0));
        cvmSet(h1,1,0,cvmGet(H[i],1,0));
        cvmSet(h1,2,0,cvmGet(H[i],2,0));
        cvmSet(h2,0,0,cvmGet(H[i],0,1));
        cvmSet(h2,1,0,cvmGet(H[i],1,1));
        cvmSet(h2,2,0,cvmGet(H[i],2,1));
        cvmSet(h3,0,0,cvmGet(H[i],0,2));
        cvmSet(h3,1,0,cvmGet(H[i],1,2));
        cvmSet(h3,2,0,cvmGet(H[i],2,2));
        // estimate the rotation matrix R and the translation vector t
        cvMatMul(Kinv,h1,r1);
        cvMatMul(Kinv,h2,r2);
        cvMatMul(Kinv,h3,t);
        lamda1=1/sqrt(pow(cvmGet(r1,0,0),2)+pow(cvmGet(r1,1,0),2)+pow(cvmGet(r1,2,0),2));
        cvmSet(r1,0,0,cvmGet(r1,0,0)*lamda1);
        cvmSet(r1,1,0,cvmGet(r1,1,0)*lamda1);
        cvmSet(r1,2,0,cvmGet(r1,2,0)*lamda1);
        cvmSet(r2,0,0,cvmGet(r2,0,0)*lamda1);
        cvmSet(r2,1,0,cvmGet(r2,1,0)*lamda1);
        cvmSet(r2,2,0,cvmGet(r2,2,0)*lamda1);
        cvmSet(t,0,0,cvmGet(t,0,0)*lamda1);
        cvmSet(t,1,0,cvmGet(t,1,0)*lamda1);
        cvmSet(t,2,0,cvmGet(t,2,0)*lamda1);
        cvCrossProduct(r1,r2,r3);
        cvmSet(Q,0,0,cvmGet(r1,0,0));
        cvmSet(Q,1,0,cvmGet(r1,1,0));
        cvmSet(Q,2,0,cvmGet(r1,2,0));
        cvmSet(Q,0,1,cvmGet(r2,0,0));
        cvmSet(Q,1,1,cvmGet(r2,1,0));
        cvmSet(Q,2,1,cvmGet(r2,2,0));
        cvmSet(Q,0,2,cvmGet(r3,0,0));
        cvmSet(Q,1,2,cvmGet(r3,1,0));
        cvmSet(Q,2,2,cvmGet(r3,2,0));
        // Refine Q become orthogonal matrix R
        cvSVD(Q, DD, UU, VV, CV_SVD_V_T);
        cvMatMul(UU,VV,R);
        // Rt matrix
        cvmSet(Rt[i],0,0,cvmGet(R,0,0));
        cvmSet(Rt[i],1,0,cvmGet(R,1,0));
        cvmSet(Rt[i],2,0,cvmGet(R,2,0));
        cvmSet(Rt[i],0,1,cvmGet(R,0,1));
        cvmSet(Rt[i],1,1,cvmGet(R,1,1));
        cvmSet(Rt[i],2,1,cvmGet(R,2,1));
        cvmSet(Rt[i],0,2,cvmGet(R,0,2));
        cvmSet(Rt[i],1,2,cvmGet(R,1,2));
        cvmSet(Rt[i],2,2,cvmGet(R,2,2));
        cvmSet(Rt[i],0,3,cvmGet(t,0,0));
        cvmSet(Rt[i],1,3,cvmGet(t,1,0));
        cvmSet(Rt[i],2,3,cvmGet(t,2,0));
    }
    
    //////////////////////////////////////////////
    // report
    //////////////////////////////////////////////
    // 1. General
    FILE *report=fopen(reportfilename,"wt");
    fprintf(report,"================= Camera Calibration Report ====================\n");
    fprintf(report,"Number of images : %d\n",Nfiles);
    fprintf(report,"Image size : 640 X 480 pixel\n");
    fprintf(report,"Line detection method : the Hough transformation\n");
    fprintf(report,"Point detection method : intersecting lines acquired by the Hough transformation\n\n\n");
    // 2. Absolute Conic & intrinsic parameters
    fprintf(report,"================= Estimated absolute Conic ====================\n");
    for(i=0;i<3;i++)
    {
        fprintf(report,"| %16.8f %16.8f %16.8f |\n",cvmGet(B,i,0),cvmGet(B,i,1),cvmGet(B,i,2));
    }
    fprintf(report,"\n================= Estimated Instrinsic Parameters ====================\n");
    fprintf(report,"vo=%f\n",vo);
    fprintf(report,"lamda=%f\n",lamda);
    fprintf(report,"alpha=%f\n",alpha);
    fprintf(report,"beta=%f\n",beta);
    fprintf(report,"gamma=%f\n",gamma);
    fprintf(report,"uo=%f\n",uo);
    fprintf(report,"| | | %16.8f %16.8f %16.8f |\n",cvmGet(K,0,0),cvmGet(K,0,1),cvmGet(K,0,2));
    fprintf(report,"| K | = | %16.8f %16.8f %16.8f |\n",cvmGet(K,1,0),cvmGet(K,1,1),cvmGet(K,1,2));
    fprintf(report,"| | | %16.8f %16.8f %16.8f |\n",cvmGet(K,2,0),cvmGet(K,2,1),cvmGet(K,2,2));
    fprintf(report,"\n");
    
    // 3. Extrinsic parameters
    fprintf(report,"\n================= Estimated Extrinsic Parameters ====================\n");
    imgfn=fopen(imagefilenames,"rt");
    double dtd[39],dtdrefined[39];
    for(i=0;i<Nfiles;i++)
    {
        // estimate residuals
        
        fscanf(imgfn,"%s",tempimgfn);
        fprintf(report,"%s\n",tempimgfn);
        fprintf(report,"| | | %16.8f %16.8f %16.8f %16.8f |\n",cvmGet(Rt[i],0,0),cvmGet(Rt[i],0,1),cvmGet(Rt[i],0,2),cvmGet(Rt[i],0,3));
        fprintf(report,"| R | t | = | %16.8f %16.8f %16.8f %16.8f |\n",cvmGet(Rt[i],1,0),cvmGet(Rt[i],1,1),cvmGet(Rt[i],1,2),cvmGet(Rt[i],1,3));
        fprintf(report,"| | | %16.8f %16.8f %16.8f %16.8f |\n",cvmGet(Rt[i],2,0),cvmGet(Rt[i],2,1),cvmGet(Rt[i],2,2),cvmGet(Rt[i],2,3));
        dtd[i]= Residuals(Rt[i],K,ObjectPoints,ImgPoints);
        fprintf(report,"errors(dtd) = %f pixels\n",dtd[i]);
        fprintf(report,"\n");
        
    }
    if (Allflag==0) // perform LM algoritm for each image seperately
    {
        /////////////////////////////////////////
        // Refine Parameters (for each camera)
        CvMat* estK = cvCreateMat(3,3,CV_64FC1);
        imgfn=fopen(imagefilenames,"rt");
        for(i=0;i<Nfiles;i++)
        {
            fscanf(imgfn,"%s",tempimgfn);
            for(j=0;j<80;j++)
            {
                cvmSet(ImgPoints,0,j,Corners[i][j][0]);
                cvmSet(ImgPoints,1,j,Corners[i][j][1]);
                cvmSet(ImgPoints,2,j,1);
            }
            
            RefineCamera(Rt[i],K,estK,ObjectPoints,ImgPoints,temp2);
            // display Homography
            // CheckRtK(Rt[i],K,ObjectPoints,temp);
            fprintf(report,"\nRefined Instrinsic Parameters (%s) \n",tempimgfn);
            fprintf(report,"| | | %16.8f %16.8f %16.8f |\n",cvmGet(estK,0,0),cvmGet(estK,0,1),cvmGet(estK,0,2));
            fprintf(report,"| K | = | %16.8f %16.8f %16.8f |\n",cvmGet(estK,1,0),cvmGet(estK,1,1),cvmGet(estK,1,2));
            fprintf(report,"| | | %16.8f %16.8f %16.8f |\n",cvmGet(estK,2,0),cvmGet(estK,2,1),cvmGet(estK,2,2));
            // 3. Extrinsic parameters
            fprintf(report,"\nRefined Extrinsic Parameters (%s) \n",tempimgfn);
            fprintf(report,"| | | %16.8f %16.8f %16.8f %16.8f |\n",cvmGet(Rt[i],0,0),cvmGet(Rt[i],0,1),cvmGet(Rt[i],0,2),cvmGet(Rt[i],0,3));
            fprintf(report,"| R | t | = | %16.8f %16.8f %16.8f %16.8f |\n",cvmGet(Rt[i],1,0),cvmGet(Rt[i],1,1),cvmGet(Rt[i],1,2),cvmGet(Rt[i],1,3));
            fprintf(report,"| | | %16.8f %16.8f %16.8f %16.8f |\n",cvmGet(Rt[i],2,0),cvmGet(Rt[i],2,1),cvmGet(Rt[i],2,2),cvmGet(Rt[i],2,3));
            dtdrefined[i]=Residuals(Rt[i],estK,ObjectPoints,ImgPoints);
            fprintf(report,"errors(dtd) = %f pixels\n",dtdrefined[i]);
            fprintf(report,"\n");
            
        }
        double improvementNumer=0;
        double improvementDenom=0;
        for(i=0;i<Nfiles;i++){
            improvementNumer=improvementNumer+dtdrefined[i];
            improvementDenom=improvementDenom+dtd[i];
            
        }
        fprintf(report,"Improvement = ( sum of initial errors ) / (sum of refined errors)= %f.\n",improvementNumer/improvementDenom);
        
    }else if (Allflag==1) // LM algorithm is applied to entire image set
    {
        ///////////////////////////////////////////
        // Refine Parameters (using entire images)
        CvMat* J = cvCreateMat(160*Nfiles,5+6*Nfiles,CV_64FC1); // Jacobian Matrix
        CvMat* JT = cvCreateMat(5+6*Nfiles,160*Nfiles,CV_64FC1); // Transposed Jacobian Matrix
        CvMat* JTd = cvCreateMat(5+6*Nfiles,1,CV_64FC1);
        CvMat* Hessian = cvCreateMat(5+6*Nfiles,5+6*Nfiles,CV_64FC1); // Hessian Matrix
        CvMat* Hessianinv = cvCreateMat(5+6*Nfiles,5+6*Nfiles,CV_64FC1);
        CvMat* I = cvCreateMat(5+6*Nfiles,5+6*Nfiles,CV_64FC1); // Identity Matrix
        CvMat* Hessian_lm = cvCreateMat(5+6*Nfiles,5+6*Nfiles,CV_64FC1); // H_lm Matrix
        CvMat* Hessian_lminv = cvCreateMat(5+6*Nfiles,5+6*Nfiles,CV_64FC1); // H_lm Matrix
        CvMat* P = cvCreateMat(5+6*Nfiles,1,CV_64FC1); // Parameters
        CvMat* Ptemp = cvCreateMat(5+6*Nfiles,1,CV_64FC1); // temporary P
        CvMat* dP = cvCreateMat(5+6*Nfiles,1,CV_64FC1); // delta P
        CvMat* Ppartial= cvCreateMat(5+6*Nfiles,1,CV_64FC1); // Partial P
        CvMat* Rtemp = cvCreateMat(3,3,CV_64FC1);
        CvMat* W = cvCreateMat(3,1,CV_64FC1); // Rodrigues representation of R
        CvMat* Wtemp = cvCreateMat(3,1,CV_64FC1); // Rodrigues representation of R
        CvMat* d = cvCreateMat(160*Nfiles,1,CV_64FC1);
        CvMat* dtemp = cvCreateMat(160*Nfiles,1,CV_64FC1);
        CvMat* dpartial= cvCreateMat(160,1,CV_64FC1);
        CvMat* Homography = cvCreateMat(3,3,CV_64FC1);
        CvMat* HomographyTemp = cvCreateMat(3,3,CV_64FC1);
        double threshold=1e-6;
        double error,error0,error2,delta=100;
        int Iteration=0;
        double damping=0.01; // create initial damping factor
        // setup identity matrix
        for(i=0;i<5+6*Nfiles;i++)
        {
            for(j=0;j<5+6*Nfiles;j++)
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
        for(i=0;i<Nfiles;i++)
        {
            // Convert R to W
            cvmSet(R,0,0,cvmGet(Rt[i],0,0));
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
        for(i=0;i<Nfiles;i++)
        {
            for(j=0;j<80;j++)
            {
                cvmSet(ImgPoints,0,j,Corners[i][j][0]);
                cvmSet(ImgPoints,1,j,Corners[i][j][1]);
                cvmSet(ImgPoints,2,j,1);
            }
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
            error0=error0+ErrorsGD(Ppartial,ObjectPoints,ImgPoints,80,dpartial);
            for(j=0;j<80;j++)
            {
                cvmSet(d,160*i+j*2 , 0 , cvmGet(dpartial,j*2 ,0));
                cvmSet(d,160*i+j*2+1 , 0 , cvmGet(dpartial,j*2+1 ,0));
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
                setJall(P,J,Nfiles,ObjectPoints);
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
            for(j=0;j<5+6*Nfiles;j++) delta=delta+pow(cvmGet(dP,j,0),2);
            delta=sqrt(delta)/(5+6*Nfiles);
            // check errors by Ptemp
            error2=0;
            for(j=0;j<Nfiles;j++)
            {
                for(k=0;k<80;k++)
                {
                    cvmSet(ImgPoints,0,k,Corners[j][k][0]);
                    cvmSet(ImgPoints,1,k,Corners[j][k][1]);
                    cvmSet(ImgPoints,2,k,1);
                }
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
                error2=error2 + ErrorsGD(Ppartial,ObjectPoints,ImgPoints,80,dpartial);
                for(k=0;k<80;k++)
                {
                    cvmSet(dtemp,160*j+k*2 , 0 , cvmGet(dpartial,k*2 ,0));
                    cvmSet(dtemp,160*j+k*2+1 , 0 , cvmGet(dpartial,k*2+1 ,0));
                }
            }
            if(error2<error) //decrease damping factor and update
            {
                damping=damping/10;
                for(j=0;j<5+6*Nfiles;j++) cvmSet(P,j,0,cvmGet(Ptemp,j,0));
                for(j=0;j<160*Nfiles;j++) cvmSet(d,j,0,cvmGet(dtemp,j,0));
                error=error2;
                updateJ=1;
                printf("iteration %d ] error=%f\n",i+1,error);
                printf(" delta=%f\n",delta);
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
        printf("delta = %f\n",delta);
        printf("error=%f\n",error);
        printf("damping=%f\n",damping);
        // Finally estimated K
        cvmSet(K,0,0,cvmGet(P,0,0));
        cvmSet(K,0,1,cvmGet(P,4,0));
        cvmSet(K,1,1,cvmGet(P,1,0));
        cvmSet(K,0,2,cvmGet(P,2,0));
        cvmSet(K,1,2,cvmGet(P,3,0));
        // Update Rt
        for(i=0;i<Nfiles;i++)
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
        fprintf(report,"\nRefined Instrinsic Parameters\n",tempimgfn);
        fprintf(report,"| | | %16.8f %16.8f %16.8f |\n",cvmGet(K,0,0),cvmGet(K,0,1),cvmGet(K,0,2));
        fprintf(report,"| K | = | %16.8f %16.8f %16.8f |\n",cvmGet(K,1,0),cvmGet(K,1,1),cvmGet(K,1,2));
        fprintf(report,"| | | %16.8f %16.8f %16.8f |\n",cvmGet(K,2,0),cvmGet(K,2,1),cvmGet(K,2,2));
        imgfn=fopen(imagefilenames,"rt");
        for(i=0;i<Nfiles;i++)
        {
            fscanf(imgfn,"%s",tempimgfn);
            // 3. Extrinsic parameters
            fprintf(report,"\nRefined Extrinsic Parameters (%s) \n",tempimgfn);
            fprintf(report,"| | | %16.8f %16.8f %16.8f %16.8f|\n",cvmGet(Rt[i],0,0),cvmGet(Rt[i],0,1),cvmGet(Rt[i],0,2),cvmGet(Rt[i],0,3));
            fprintf(report,"| R | t | = | %16.8f %16.8f %16.8f %16.8f |\n",cvmGet(Rt[i],1,0),cvmGet(Rt[i],1,1),cvmGet(Rt[i],1,2),cvmGet(Rt[i],1,3));
            fprintf(report,"| | | %16.8f %16.8f %16.8f %16.8f|\n",cvmGet(Rt[i],2,0),cvmGet(Rt[i],2,1),cvmGet(Rt[i],2,2),cvmGet(Rt[i],2,3));
            dtdrefined[i]=Residuals(Rt[i],K,ObjectPoints,ImgPoints);
            fprintf(report,"errors(dtd) = %f pixels (%4.2f%%)\n",dtdrefined[i],dtdrefined[i]/dtd[i]*100);
            fprintf(report,"\n");
        }
        double improvementNumer=0;
        double improvementDenom=0;
        for(i=0;i<Nfiles;i++)
        {
            improvementNumer=improvementNumer+dtdrefined[i];
            improvementDenom=improvementDenom+dtd[i];
        }
        fprintf(report,"Improvement = sqrt( sum of refined errors ) / sqrt(sum of initial errors) = %f.\n",sqrt(error/error0));
    }
    fclose(report);
    cvReleaseImage(&img1);
    cvReleaseImage(&temp);
}

//< Function ErrorsGD>
//Return errors w.r.t parameter vector P and display corners on the image
double ErrorsGD(CvMat* P,CvMat* ObjectPoints,CvMat* ImgPoints,int N,CvMat* d,IplImage *img,int Viewflag){
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
    if(Viewflag==1) CheckRtK(Rt,K,ObjectPoints,img);
    double RMSE=Residuals(Rt,K,ObjectPoints,ImgPoints,d);
    return RMSE;
}

double NormalizationMatrixImg(CvMat *Corners1,int N,CvMat *MT){
    int i;
    double scale,Cx,Cy,AvgDist;
    Cx=0;Cy=0;AvgDist=0;
    for(i=0;i<N;i++)
    {
        Cx=Cx+cvmGet(Corners1,0,i);
        Cy=Cy+cvmGet(Corners1,1,i);
    }
    Cx=Cx/N;
    Cy=Cy/N;
    for(i=0;i<N;i++)
        AvgDist = AvgDist+sqrt(pow(cvmGet(Corners1,0,i)-Cx,2)+pow(cvmGet(Corners1,1,i)-Cy,2));
    AvgDist=AvgDist/N;
    scale=sqrt(2)/AvgDist;
    
    cvmSet(MT,0,0,scale);
    cvmSet(MT,0,1,0);
    cvmSet(MT,0,2,-scale*Cx);
    cvmSet(MT,1,0,0);
    cvmSet(MT,1,1,scale);
    cvmSet(MT,1,2,-scale*Cy);
    cvmSet(MT,2,0,0);
    cvmSet(MT,2,1,0);
    cvmSet(MT,2,2,1);
    return scale;
}
//
//<Function Homography>
//- Estimate H , datapoint1 is in the domain and datapoint2 is in the range.
//- N is the number of points
void Homography(CvMat* datapoints1, CvMat* datapoints2, int N, CvMat* H){
    int j;
    CvMat* A = cvCreateMat(N*2,9,CV_64FC1);
    CvMat* U = cvCreateMat(N*2,N*2,CV_64FC1);
    CvMat* D = cvCreateMat(N*2,9,CV_64FC1);
    CvMat* V = cvCreateMat(9,9,CV_64FC1);
    for(j=0;j<N;j++)
    {
        cvmSet(A,2*j,0,0);
        cvmSet(A,2*j,1,0);
        cvmSet(A,2*j,2,0);
        cvmSet(A,2*j,3,-cvmGet(datapoints1,0,j));
        cvmSet(A,2*j,4,-cvmGet(datapoints1,1,j));
        cvmSet(A,2*j,5,-cvmGet(datapoints1,2,j));
        cvmSet(A,2*j,6,cvmGet(datapoints2,1,j)*cvmGet(datapoints1,0,j));
        cvmSet(A,2*j,7,cvmGet(datapoints2,1,j)*cvmGet(datapoints1,1,j));
        cvmSet(A,2*j,8,cvmGet(datapoints2,1,j)*cvmGet(datapoints1,2,j));
        cvmSet(A,2*j+1,0,cvmGet(datapoints1,0,j));
        cvmSet(A,2*j+1,1,cvmGet(datapoints1,1,j));
        cvmSet(A,2*j+1,2,cvmGet(datapoints1,2,j));
        cvmSet(A,2*j+1,3,0);
        cvmSet(A,2*j+1,4,0);
        cvmSet(A,2*j+1,5,0);
        cvmSet(A,2*j+1,6,-cvmGet(datapoints2,0,j)*cvmGet(datapoints1,0,j));
        cvmSet(A,2*j+1,7,-cvmGet(datapoints2,0,j)*cvmGet(datapoints1,1,j));
        cvmSet(A,2*j+1,8,-cvmGet(datapoints2,0,j)*cvmGet(datapoints1,2,j));
    }
    // estimate H
    cvSVD(A, D, U, V, CV_SVD_V_T);
    cvmSet(H,0,0,cvmGet(V,8,0));
    cvmSet(H,0,1,cvmGet(V,8,1));
    cvmSet(H,0,2,cvmGet(V,8,2));
    cvmSet(H,1,0,cvmGet(V,8,3));
    cvmSet(H,1,1,cvmGet(V,8,4));
    cvmSet(H,1,2,cvmGet(V,8,5));
    cvmSet(H,2,0,cvmGet(V,8,6));
    cvmSet(H,2,1,cvmGet(V,8,7));
    cvmSet(H,2,2,cvmGet(V,8,8));
}

//<Function SetV>
//Set up V matrix to estimate the absolute conic B
//H is the homography of the i-th image
void SetV(CvMat*H, int i, CvMat *V){
    double h11,h12,h13,h21,h22,h23,h31,h32,h33;
    h11=cvmGet(H,0,0);
    h12=cvmGet(H,1,0);
    h13=cvmGet(H,2,0);
    h21=cvmGet(H,0,1);
    h22=cvmGet(H,1,1);
    h23=cvmGet(H,2,1);
    h31=cvmGet(H,0,2);
    h32=cvmGet(H,1,2);
    h33=cvmGet(H,2,2);
    cvmSet(V,2*i,0,h11*h21);
    cvmSet(V,2*i,1,h11*h22+h12*h21);
    cvmSet(V,2*i,2,h12*h22);
    cvmSet(V,2*i,3,h13*h21+h11*h23);
    cvmSet(V,2*i,4,h13*h22+h12*h23);
    cvmSet(V,2*i,5,h13*h23);
    cvmSet(V,2*i+1,0,h11*h11-h21*h21);
    cvmSet(V,2*i+1,1,h11*h12+h12*h11-h21*h22-h22*h21);
    cvmSet(V,2*i+1,2,h12*h12-h22*h22);
    cvmSet(V,2*i+1,3,h13*h11+h11*h13-h23*h21-h21*h23);
    cvmSet(V,2*i+1,4,h13*h12+h12*h13-h23*h22-h22*h23);
    cvmSet(V,2*i+1,5,h13*h13-h23*h23);
}
//< Function Residuals>
//Return errors w.r.t the Rt, K , ObjectPoints and ImgPoints
double Residuals(CvMat* Rt,CvMat* K,CvMat* ObjectPoints,CvMat* ImgPoints){
    int i;
    CvMat* Homography=cvCreateMat(3,3,CV_64FC1);
    CvMat* temp=cvCreateMat(3,3,CV_64FC1);
    CvMat* estImgPoints=cvCreateMat(3,80,CV_64FC1);
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
    for(i=0;i<80;i++)
    {
        dtd=dtd+pow((cvmGet(estImgPoints,0,i)/cvmGet(estImgPoints,2,i))-cvmGet(ImgPoints,0,i),2);
        dtd=dtd+pow((cvmGet(estImgPoints,1,i)/cvmGet(estImgPoints,2,i))-cvmGet(ImgPoints,1,i),2);
    };
    return dtd;
}
//< Function Residuals>
//Return errors & d vector w.r.t the Rt, K , ObjectPoints and ImgPoints
double Residuals(CvMat* Rt,CvMat* K,CvMat* ObjectPoints,CvMat* ImgPoints,CvMat* d){
    int i;
    CvMat* Homography=cvCreateMat(3,3,CV_64FC1);
    CvMat* temp=cvCreateMat(3,3,CV_64FC1);
    CvMat* estImgPoints=cvCreateMat(3,80,CV_64FC1);
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
    for(i=0;i<80;i++)
    {
        // d : input-estimated
        cvmSet(d,i*2,0,cvmGet(ImgPoints,0,i)-cvmGet(estImgPoints,0,i)/cvmGet(estImgPoints,2,i));
        cvmSet(d,i*2+1,0,cvmGet(ImgPoints,1,i)-cvmGet(estImgPoints,1,i)/cvmGet(estImgPoints,2,i));
    }
    //return d'*d
    double dtd=0;
    for(i=0;i<160;i++) dtd=dtd+pow(cvmGet(d,i,0),2);
    return dtd;
}
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
//< Function CheckRtK>
//Check a Homography w.r.t Rt , K, and ObjectPoints
void CheckRtK(CvMat* Rt,CvMat* K,CvMat* ObjectPoints, IplImage *img){
    int i;
    CvMat* Homography=cvCreateMat(3,3,CV_64FC1);
    CvMat* temp=cvCreateMat(3,3,CV_64FC1);
    CvMat* estImgPoints=cvCreateMat(3,80,CV_64FC1);
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
    CvPoint Draw;
    for(i=0;i<80;i++)
    {
        Draw.x=cvmGet(estImgPoints,0,i)/cvmGet(estImgPoints,2,i)+.5;
        Draw.y=cvmGet(estImgPoints,1,i)/cvmGet(estImgPoints,2,i)+.5;
//        draw_cross(img,Draw,CV_RGB(255,255,255),3);
    }
    cvNamedWindow( "Check Points", 1 );
    cvShowImage( "Check Points", img );
    cvWaitKey(0);
}

//<Function RefineCamera>
//Input : - Rt : 3X4 matrix (R|t)
//- K : camera matrix containing intrinsic parameters
//- estK : estimated K
//- ObjectPoints : object coordinates of the calibration patterns
//- ImgPoints : image coordinates of the calibration patterns
//- img : image
//Output : refined parameters Rt and estK
void RefineCamera(CvMat *Rt, CvMat *K, CvMat *estK,CvMat *ObjectPoints, CvMat *ImgPoints, IplImage *img){
    CvMat* J = cvCreateMat(160,11,CV_64FC1); // Jacobian Matrix
    CvMat* JT = cvCreateMat(11,160,CV_64FC1); // Transposed Jacobian Matrix
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
    CvMat* d = cvCreateMat(160,1,CV_64FC1);
    CvMat* dtemp = cvCreateMat(160,1,CV_64FC1);
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
    printf("Initial parameter values\n");
    for(i=0;i<11;i++) printf("P(%2d)=%f\n",i,cvmGet(P,i,0));
    // Initial geometric distance
    error=ErrorsGD(P,ObjectPoints,ImgPoints,80,d,img,0);
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
        error2=ErrorsGD(Ptemp,ObjectPoints,ImgPoints,80,d,img,0);
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
    printf("Number of iteration = %d\n",j);
    printf("delta = %f\n",delta);
    // Display Refined parameter values
    printf("Refined parameter values\n");
    for(i=0;i<11;i++) printf("P(%2d)=%f\n",i,cvmGet(P,i,0));
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


    
#endif /* optimization_h */
