//
//  calibration.cpp
//  
//
//  Created by David Choqueluque Roman on 2/10/19.
//

#include "calibration.hpp"
//
// file : calibration.cpp
//------------------------------------------------
// this file contains functions for calibration
//
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include "cv.h"
#include "cxcore.h"
#include "highgui.h"
#include "utility.h"
#include "calibration.h"
#include "jacobianCalculation.h"
void CameraCalibration(HomographySet *h, CameraParam *cp, PairPointSet *imagePt)
{
    CvMat *K = cvCreateMat(3, 3, CV_32FC1); // intrinsic matrix
    // direct calibration method (algebraic minimization)
    CalculateIntrinsicMatrix(h, K, cp);
    CalculateExtrinsicParameters(h, K, cp);
    CalculateRadialDistotion(h, K, cp, imagePt);
    WriteParameters(cp, "InitialCalibration.txt");
    // refinement
    RefineCameraParameters(cp, imagePt);
    WriteParameters(cp, "RefinedCalibration.txt");
}
void CalculateIntrinsicMatrix(HomographySet *h, CvMat *K, CameraParam *cp) {
    int i, n, m;
    int numOfImage = h->len;
    int numColumnVi = 6, numRowVi = 2;
    CvMat *H = cvCreateMat(3, 3, CV_64FC1);
    CvMat *Vi = cvCreateMat(numRowVi, numColumnVi, CV_64FC1);
    CvMat *V = cvCreateMat(numRowVi*numOfImage, numColumnVi, CV_64FC1);
    // set matrix V
    for(i = 0; i < numOfImage; i++){
        for(n = 0; n < 3; n++){
            for(m = 0; m < 3; m++){
                cvmSet(H, n, m, h->H[n][m][i]);
            }
        }
        float v12[numColumnVi], v11[numColumnVi], v22[numColumnVi];
        vijSetting(v12, 1, 2, H);
        vijSetting(v11, 1, 1, H);
        vijSetting(v22, 2, 2, H);
        // Vi = [ v12 ]
        // [ v11 - v22 ]
        for(m = 0; m < numColumnVi; m++){
            cvmSet(Vi, 0, m, v12[m]);
            cvmSet(Vi, 1, m, v11[m] - v22[m]);
        }
        // V = [V1]
        // [V2]
        // [..]
        // [Vn]
        for(n = 0; n < numRowVi; n++){
            for(m = 0; m < numColumnVi; m++){
                cvmSet(V, n + i*numRowVi, m, cvmGet(Vi, n, m));
            }
        }
    }
    // calculate the absolute conic W : Vw
    double w[9];
    CvMat *Dsvd = cvCreateMat(numRowVi*numOfImage, numColumnVi, CV_64FC1);
    CvMat *Usvd = cvCreateMat(numRowVi*numOfImage, numRowVi*numOfImage, CV_64FC1);
    CvMat *Vsvd = cvCreateMat(numColumnVi, numColumnVi, CV_64FC1);
    cvSVD(V, Dsvd, Usvd, Vsvd, CV_SVD_U_T|CV_SVD_V_T);
    // V = Usvd^T Dsvd Vsvd in openCV : V = U’ D’ V’^T in text
    // take last column of V’ : last row of Vsvd
    // w = [w11 , w12 , w22 , w13 , w23 , w33]
    for(i = 0; i < numColumnVi; i++){
        w[i] = cvmGet(Vsvd, numColumnVi-1, i);
    }
    // y0 = (w12 * w13 - w11 * w23) / (w11 * w22 - w12^2)
    // lambda = w33 - (w13^2 + y0 * (w12 * w13 - w11 * w23)) / w11
    // alphaX = (lambda / w11)^.5
    // alphaY = (lambda * w11 / (w11 * w22 - w12^2))^.5
    // skew = -w12 * alphaX^2 * alphaY / lambda
    // x0 = skew * y0 / alphaY - w13 * alphaX^2 / lambda
    double y0 = (w[1]* w[3] - w[0] * w[4]) / (w[0] * w[2] - pow(w[1], 2));
    double lambda = w[5] - (pow(w[3], 2) + y0 * (w[1] * w[3] - w[0] * w[4])) / w[0];
    double alphaX = sqrt(lambda / w[0]);
    double alphaY = sqrt((lambda * w[0]) / (w[0] * w[2] - pow(w[1], 2)));
    double skew = -w[1] * pow(alphaX, 2) * alphaY / lambda;
    double x0 = (skew * y0) / alphaY - (w[3] * pow(alphaX, 2)) / lambda;
    // get intrinsic matrix K
    float kArr[9] = {alphaX, skew, x0,
        0 , alphaY, y0,
        0 , 0 , 1};
    Array2CvMat(kArr, K, 3, 3);
    // store extrinsic parameters
    cp->alphaX = alphaX;
    cp->alphaY = alphaY;
    cp->x0 = x0;
    cp->y0 = y0;
    cp->skew = skew;
    // release matrices
    cvReleaseMat(&H); cvReleaseMat(&Vi); cvReleaseMat(&V);
    cvReleaseMat(&Dsvd); cvReleaseMat(&Usvd); cvReleaseMat(&Vsvd);
}
//
// function : vijSetting
// usage : vijSetting(vij, i, j, H);
//--------------------------------------------------
// this function sets vij vector for constructing V.
//
void vijSetting(float *v, int i, int j, CvMat *H) {
    // following Zhang’s notation
    // hi : ith column vector (n.b. hij is not (H)ij but (H)ji)
    float hi1 = cvmGet(H, 0, i-1);
    float hi2 = cvmGet(H, 1, i-1);
    float hi3 = cvmGet(H, 2, i-1);
    float hj1 = cvmGet(H, 0, j-1);
    float hj2 = cvmGet(H, 1, j-1);
    float hj3 = cvmGet(H, 2, j-1);
    float vij[6] = {hi1*hj1 , hi1*hj2 + hi2*hj1 , hi2*hj2 ,
        hi3*hj1 + hi1*hj3 , hi3*hj2 + hi2*hj3 , hi3*hj3};
    for(int k = 0; k < 6; k++){
        v[k] = vij[k];
    }
}
void CalculateExtrinsicParameters(HomographySet *h, CvMat *K, CameraParam *cp)
{
    int imageNumber, k;
    int numOfImages = h->len;
    CvMat *invK = cvCreateMat(3, 3, CV_32FC1);
    CvMat *h1 = cvCreateMat(3, 1, CV_32FC1);
    CvMat *h2 = cvCreateMat(3, 1, CV_32FC1);
    CvMat *h3 = cvCreateMat(3, 1, CV_32FC1);
    CvMat *r1 = cvCreateMat(3, 1, CV_32FC1);
    CvMat *r2 = cvCreateMat(3, 1, CV_32FC1);
    CvMat *r3 = cvCreateMat(3, 1, CV_32FC1);
    CvMat *t = cvCreateMat(3, 1, CV_32FC1);
    CvMat *D = cvCreateMat(3, 3, CV_64FC1);
    CvMat *U = cvCreateMat(3, 3, CV_64FC1);
    CvMat *V = cvCreateMat(3, 3, CV_64FC1);
    CvMat *Q = cvCreateMat(3, 3, CV_64FC1);
    CvMat *R = cvCreateMat(3, 3, CV_64FC1);
    CvMat *transU = cvCreateMat(3, 3, CV_64FC1);
    cvInvert(K, invK);
    for(imageNumber = 0; imageNumber < numOfImages ; imageNumber++){
        for(k = 0; k < 3; k++){
            cvmSet(h1, k, 0, h->H[k][0][imageNumber]);
            cvmSet(h2, k, 0, h->H[k][1][imageNumber]);
            cvmSet(h3, k, 0, h->H[k][2][imageNumber]);
        }
        // calculate r1 r2 r3 and t
        cvMatMul(invK, h1, r1); // r1 = lambda * invK h1
        cvMatMul(invK, h2, r2); // r2 = lambda * invK h2
        cvMatMul(invK, h3, t); // t = lambda * invK h3
        // cvCrossProduct(r1, r2, r3); //r3 = r1 X r2
        float squareSum = 0;
        // calculate normalization factor lambda
        for(k = 0; k < 3; k++){
            squareSum += pow(cvmGet(r1, k, 0), 2);
        }
        float lambda = 1 / sqrt(squareSum);
        // normalization
        for(k = 0; k < 3; k++){
            cvmSet(t, k, 0, lambda * cvmGet(t, k, 0));
            // followings are not accurate
            cvmSet(r1, k, 0, lambda * cvmGet(r1, k, 0));
            cvmSet(r2, k, 0, lambda * cvmGet(r2, k, 0));
        }
        cvCrossProduct(r1, r2, r3); //r3 = r1 X r2
        ///////////////////////////////////////////////
        // refine rotation matrix //
        // estimate best possible R //
        // min||R - Q||_F, s.t R^TR = I //
        ///////////////////////////////////////////////
        // Q : initially estimated rotation matrix, Q = [r1 r2 r3]
        for(k = 0; k < 3; k++){
            cvmSet(Q, k, 0, cvmGet(r1, k, 0));
            cvmSet(Q, k, 1, cvmGet(r2, k, 0));
            cvmSet(Q, k, 2, cvmGet(r3, k, 0));
        }
        // calculate R
        cvSVD(Q, D, U, V, CV_SVD_U_T|CV_SVD_V_T);
        // Q = U^T D V in openCV : Q = U’ D’ V’^T in text
        // R = U’V’^T : R = U^TV
        cvTranspose(U, transU);
        cvMatMul(transU, V, R);
        // store extrinsic parameters
        for(k = 0; k < 3; k++){
            cp->r1[k][imageNumber] = cvmGet(R, k, 0);
            cp->r2[k][imageNumber] = cvmGet(R, k, 1);
            cp->r3[k][imageNumber] = cvmGet(R, k, 2);
            cp->t[k][imageNumber] = cvmGet(t, k, 0);
        }
        /* in this case, rotation matrix has error
         for(k = 0; k < 3; k++){
         cp->r1[k][imageNumber] = cvmGet(r1, k, 0);
         cp->r2[k][imageNumber] = cvmGet(r2, k, 0);
         cp->r3[k][imageNumber] = cvmGet(r3, k, 0);
         cp->t[k][imageNumber] = cvmGet(t, k, 0);
         }*/
    }
    // release matrices
    cvReleaseMat(&invK);
    cvReleaseMat(&h1); cvReleaseMat(&h2); cvReleaseMat(&h3);
    cvReleaseMat(&r1); cvReleaseMat(&r2); cvReleaseMat(&r3); cvReleaseMat(&t);
    cvReleaseMat(&D); cvReleaseMat(&U); cvReleaseMat(&V);
    cvReleaseMat(&Q); cvReleaseMat(&R); cvReleaseMat(&transU);
}
void CalculateRadialDistotion(HomographySet *h, CvMat *K, CameraParam *cp,
                              PairPointSet *imagePt) {
    int imageNumber, i, m;
    int numOfImages = h->len;
    int numOfPoints = imagePt->pointLen;
    double x1, x2, x3, u, v, X, Y;
    CvMat *A = cvCreateMat(3, 3, CV_32FC1);
    CvMat *H = cvCreateMat(3, 3, CV_32FC1);
    CvMat *Di = cvCreateMat(2, 2, CV_64FC1);
    CvMat *di = cvCreateMat(2, 1, CV_64FC1);
    CvMat *D = cvCreateMat(2 * numOfImages * numOfPoints, 2, CV_64FC1);
    CvMat *d = cvCreateMat(2 * numOfImages * numOfPoints, 1, CV_64FC1);
    CvMat *kLS = cvCreateMat(2, 1, CV_64FC1);
    for(imageNumber = 0; imageNumber < numOfImages; imageNumber++){
        //H = KA, where A = [r1 r2 t];
        for(i = 0; i < 3; i++){
            cvmSet(A, i, 0, cp->r1[i][imageNumber]);
            cvmSet(A, i, 1, cp->r2[i][imageNumber]);
            cvmSet(A, i, 2, cp->t[i][imageNumber]);
        }
        cvMatMul(K, A, H);
        float h[9];
        CvMat2Array(H, h, 3, 3);
        // get ideal points
        for(i = 0; i < imagePt->pointLen; i++){
            X = imagePt->ptmJ[i][imageNumber];
            Y = imagePt->ptmI[i][imageNumber];
            x1 = h[0] * X + h[1] * Y + h[2];
            x2 = h[3] * X + h[4] * Y + h[5];
            x3 = h[6] * X + h[7] * Y + h[8];
            // get ideal points
            u = x1 / x3;
            v = x2 / x3;
            // set Di and di
            double tpU = (u - cp->x0);
            double tpV = (v - cp->y0);
            double r = pow(tpU / cp->alphaX , 2) + pow(tpV / cp->alphaY, 2);
            cvmSet(Di, 0, 0, tpU * r);
            cvmSet(Di, 0, 1, tpU * pow(r, 2));
            cvmSet(Di, 1, 0, tpV * r);
            cvmSet(Di, 1, 1, tpV * pow(r, 2));
            cvmSet(di, 0, 0, imagePt->ptiJ[i][imageNumber] - u);
            cvmSet(di, 1, 0, imagePt->ptiI[i][imageNumber] - v);
            // D = [D1]
            // [D2]
            // [..]
            // [Dn]
            for(m = 0; m < 2; m++){
                cvmSet(D, i*2 + imageNumber*numOfPoints*2, m, cvmGet(Di, 0, m));
                cvmSet(D, i*2 + 1 + imageNumber*numOfPoints*2, m, cvmGet(Di, 1, m));
            }
            // d = [d1]
            // [d2]
            // [..]
            // [dn]
            cvmSet(d, i*2 + imageNumber * numOfPoints*2, 0, cvmGet(di, 0, 0));
            cvmSet(d, i*2 + 1 + imageNumber * numOfPoints*2, 0, cvmGet(di, 1, 0));
        }
    }
    // kLS = inv(D’D)D’d
    // LeastSquare(D, d, kLS);
    cvSolve(D, d, kLS, CV_SVD);
    cp->k1 = cvmGet(kLS, 0, 0);
    cp->k2 = cvmGet(kLS, 1, 0);
    // release matrices
    cvReleaseMat(&A); cvReleaseMat(&H);
    cvReleaseMat(&Di); cvReleaseMat(&di);
    cvReleaseMat(&D); cvReleaseMat(&d);
    cvReleaseMat(&kLS);
}
//
// function : RefineCameraParameters
// usage : RefineCameraParameters(cp, imagePt);
//-----------------------------------------------------------------
// this function refines camera parameters using LM algorithm
// in this function, error(geometric distance) is calcualted using
// whole image set, however, camera parameters are updated image by image
// because Jacobian calculation for whole image set is somewhat not preferable
//
void RefineCameraParameters(CameraParam *cp, PairPointSet *imagePt) {
    int numOfImages = imagePt->imageLen;//numero de imagenes 10
    int numOfData = 2 * imagePt->pointLen * numOfImages; //num. pts ctrl(20)*10 = 2*20*10
    int numOfParams = 7 + numOfImages * 6;
    // Jx & Jy size : 7 + numOfImage * 6
    int numOfIteration = NUM_OF_ITERATION;
    int numOfPoints = imagePt->pointLen;//20
    int iterationNum;
    double e, eNew;
    double lambda = LAMBDA;
    bool update = true;
    int i, m, k;
    double X, Y;
    double theta, a;
    double Jx[numOfParams], Jy[numOfParams];
    CvScalar tr;
    double au;
    double av;
    double u0;
    double v0;
    double sk;
    double k1;
    double k2;
    double wx[numOfImages], wy[numOfImages], wz[numOfImages]; //for rodriguex rep.
    double tx[numOfImages], ty[numOfImages], tz[numOfImages]; //translation vector?
    double wxNew, wyNew, wzNew;
    float stepSize = 1; // default stepSize = 1
    CvMat *R = cvCreateMat(3, 3, CV_64FC1);
    CvMat *J = cvCreateMat(numOfData, numOfParams, CV_64FC1);
    CvMat *Hessian = cvCreateMat(numOfParams, numOfParams, CV_64FC1);
    CvMat *invHessian = cvCreateMat(numOfParams, numOfParams, CV_64FC1);
    CvMat *transJ = cvCreateMat(numOfParams, numOfData, CV_64FC1);
    CvMat *Jerr = cvCreateMat(numOfParams, 1, CV_64FC1);
    CvMat *dp = cvCreateMat(numOfParams, 1, CV_64FC1);
    CvMat *d = cvCreateMat(numOfData, 1, CV_64FC1);
    CvMat *dLM = cvCreateMat(numOfData, 1, CV_64FC1);
    
    e = CalculateError(cp, imagePt, d);
    printf("initial error = %f\n", e);
    for(iterationNum = 0; iterationNum < numOfIteration; iterationNum++){
        if(update == true){
            for(k = 0; k < numOfImages; k++){
                //Rodigrues representation
                for(m = 0; m < 3; m++){
                    cvmSet(R, m, 0, cp->r1[m][k]);
                    cvmSet(R, m, 1, cp->r2[m][k]);
                    cvmSet(R, m, 2, cp->r3[m][k]);
                }
                tr = cvTrace(R); // trace(R) -> tr.val[0]
                theta = acos((tr.val[0] - 1) / 2.0);
                a = (theta / (2 * sin(theta)));
                
                wx[k] = a * (cvmGet(R, 2, 1) - cvmGet(R, 1, 2));
                wy[k] = a * (cvmGet(R, 0, 2) - cvmGet(R, 2, 0));
                wz[k] = a * (cvmGet(R, 1, 0) - cvmGet(R, 0, 1));
                //traslation vector
                tx[k] = cp->t[0][k];
                ty[k] = cp->t[1][k];
                tz[k] = cp->t[2][k];
                
                //intirinsic param...
                au = cp->alphaX;
                av = cp->alphaY;
                u0 = cp->x0;
                v0 = cp->y0;
                sk = cp->skew;
                k1 = cp->k1; //radial distortion
                k2 = cp->k2; //radial distortion
                
                for(i = 0; i < numOfPoints; i++){
                    //Evaluate the Jacobian at the current parameter values
                    // and the values of geometric distance
                    X = imagePt-> ptmJ[i][k];
                    Y = imagePt-> ptmI[i][k];
                    CalculateJocobian(Jx, Jy, k, numOfParams,
                                      X, Y, au, av, u0, v0, sk,
                                      wx[k], wy[k], wz[k], tx[k], ty[k], tz[k],
                                      k1, k2);
                    for(m = 0; m < numOfParams; m++){
                        cvmSet(J, i * 2 + k * numOfPoints * 2, m, Jx[m]);
                        cvmSet(J, i * 2 + 1 + k * numOfPoints * 2, m, Jy[m]);
                    }
                }
            }
            // compute the approximated Hessian matrix
            cvMulTransposed (J, Hessian, 1); // Hessian =J’J
        }
        // update camera parameters (cp)
        // apply the damping factor to the Hessian matrix
        for(m = 0; m < numOfParams; m++){
            double tmp = cvmGet(Hessian, m, m);
            cvmSet(Hessian, m, m, tmp + lambda);
        }
        // compute the updated parameters
        // dp = inv(Hessian)*(J’*d(:));
        cvTranspose(J, transJ);
        cvMatMul(transJ, d, Jerr);
        cvInvert(Hessian, invHessian, CV_SVD_SYM);
        cvMatMul(invHessian, Jerr, dp);
        cp->alphaX = au - cvmGet(dp, 0, 0) * stepSize;
        cp->alphaY = av - cvmGet(dp, 1, 0) * stepSize;
        cp->x0 = u0 - cvmGet(dp, 2, 0) * stepSize;
        cp->y0 = v0 - cvmGet(dp, 3, 0) * stepSize;
        cp->skew = sk - cvmGet(dp, 4, 0) * stepSize;
        cp->k1 = k1 - cvmGet(dp, numOfParams - 2, 0) * stepSize;
        cp->k2 = k2 - cvmGet(dp, numOfParams - 1, 0) * stepSize;
        for(k = 0; k < numOfImages; k++){
            wxNew = wx[k] - cvmGet(dp, 5 + k * 6, 0) * stepSize;
            wyNew = wy[k] - cvmGet(dp, 6 + k * 6, 0) * stepSize;
            wzNew = wz[k] - cvmGet(dp, 7 + k * 6, 0) * stepSize;
            cp->t[0][k] = tx[k] - cvmGet(dp, 8 + k * 6, 0) * stepSize;
            cp->t[1][k] = ty[k] - cvmGet(dp, 9 + k * 6, 0) * stepSize;
            cp->t[2][k] = tz[k] - cvmGet(dp, 10 + k * 6, 0) * stepSize;
            // convert the 3-vector [wx wy wz] of the Rodigrues representation
            // into the 3x3 rotation matrix
            Rodrigues2R(wxNew, wyNew, wzNew, R);
            for(m = 0; m < 3; m++){
                cp->r1[m][k] = cvmGet(R, m, 0);
                cp->r2[m][k] = cvmGet(R, m, 1);
                cp->r3[m][k] = cvmGet(R, m, 2);
            }
        }
        //Evaluate the total geometric distance at the updated parameters
        eNew = CalculateError(cp, imagePt, dLM);
        // if the total geometric distance of the updated parameters is
        // less than the previous one then makes the updated parameters
        // to be the current parameters and decreases
        // the value of the damping factor
        if(eNew < e){
            lambda = lambda / 10;
            e = CalculateError(cp, imagePt, d);
            update = true;
        }else{ // reverse updated data
            update = false;
            lambda = lambda * 20;
            cp->alphaX = au;
            cp->alphaY = av;
            cp->x0 = u0;
            cp->y0 = v0;
            cp->skew = sk;
            cp->k1 = k1;
            cp->k2 = k2;
            for(k = 0; k < numOfImages; k++){
                cp->t[0][k] = tx[k];
                cp->t[1][k] = ty[k];
                cp->t[2][k] = tz[k];
                Rodrigues2R(wx[k], wy[k], wz[k], R);
                for(m = 0; m < 3; m++){
                    cp->r1[m][k] = cvmGet(R, m, 0);
                    cp->r2[m][k] = cvmGet(R, m, 1);
                    cp->r3[m][k] = cvmGet(R, m, 2);
                }
            }
        }
    }
    printf("refined error = %f\n", e);
    cvReleaseMat(&R); cvReleaseMat(&J); cvReleaseMat(&Hessian);
    cvReleaseMat(&dp); cvReleaseMat(&d); cvReleaseMat(&dLM);
    cvReleaseMat(&invHessian); cvReleaseMat(&transJ); cvReleaseMat(&Jerr);
}
double CalculateError(CameraParam *cp, PairPointSet *imagePt, CvMat *d) {
    float au = cp->alphaX;
    float av = cp->alphaY;
    float u0 = cp->x0;
    float v0 = cp->y0;
    float sk = cp->skew;
    float k1 = cp->k1;
    float k2 = cp->k2;
    int k, i;
    int numOfImages = imagePt->imageLen;
    int numOfPoints = imagePt->pointLen;
    double x1, x2, x3, u, v, X, Y, ui, vi;
    double tpU , tpV , r;
    CvMat *A = cvCreateMat(3, 3, CV_64FC1);
    CvMat *K = cvCreateMat(3, 3, CV_64FC1);
    CvMat *H = cvCreateMat(3, 3, CV_64FC1);
    for(k = 0; k < numOfImages; k++){
        //H = KA, where A = [r1 r2 t];
        for(i = 0; i < 3; i++){
            cvmSet(A, i, 0, cp->r1[i][k]);
            cvmSet(A, i, 1, cp->r2[i][k]);
            cvmSet(A, i, 2, cp->t[i][k]);
        }
        float kArr[9] = {au, sk, u0,
            0, av, v0,
            0, 0, 1};
        Array2CvMat(kArr, K, 3, 3);
        cvMatMul(K, A, H);
        float h[9];
        CvMat2Array(H, h, 3, 3);
        // get ideal points
        for(i = 0; i < numOfPoints; i++){
            X = imagePt->ptmJ[i][k];
            Y = imagePt->ptmI[i][k];
            x1 = h[0] * X + h[1] * Y + h[2];
            x2 = h[3] * X + h[4] * Y + h[5];
            x3 = h[6] * X + h[7] * Y + h[8];
            // get ideal points
            ui = x1 / x3;
            vi = x2 / x3;
            // radial distortion
            tpU = (ui - u0);
            tpV = (vi - v0);
            r = pow(tpU / au , 2) + pow(tpV / av, 2);
            u = ui + tpU * (k1 * r + k2 * pow(r, 2));
            v = vi + tpV * (k1 * r + k2 * pow(r, 2));
            cvmSet(d, i*2 + k*numOfPoints*2, 0,
                   imagePt->ptiJ[i][k] - u);
            cvmSet(d, i*2 + 1 + k*numOfPoints*2, 0,
                   imagePt->ptiI[i][k] - v);
        }
    }
    int numOfData = numOfPoints * numOfImages;
    double e = cvDotProduct(d, d) / numOfData;
    cvReleaseMat(&A); cvReleaseMat(&H); cvReleaseMat(&K);
    return(e);
}
void Rodrigues2R(float wx, float wy, float wz, CvMat *R){
    CvMat* W = cvCreateMat(3, 3, CV_64FC1);
    CvMat* Ws = cvCreateMat(3, 3, CV_64FC1);
    CvMat* W2 = cvCreateMat(3, 3, CV_64FC1);
    CvMat* W2s = cvCreateMat(3, 3, CV_64FC1);
    CvMat* Rtmp = cvCreateMat(3, 3, CV_64FC1);
    CvMat* Eye = cvCreateMat(3, 3, CV_64FC1);
    // convert the 3-vector [wx wy wz] of
    // the Rodigrues representation
    // into the 3x3 rotation matrix
    float norm2 = pow(wx, 2) + pow(wy, 2) + pow(wz, 2);
    float norm = sqrt(norm2);
    float omega[9] = {0, -wz, wy,
        wz, 0, -wx,
        -wy, wx, 0};
    Array2CvMat(omega, W, 3, 3);
    float a1 = (sin(norm) / norm);
    float a2 = ((1 - cos(norm)) / norm2);
    cvmScale(W, Ws, a1);
    cvMatMul(W, W, W2);
    cvmScale(W2, W2s, a2);
    float I[9] = {1, 0, 0,
        0, 1, 0,
        0, 0, 1};
    Array2CvMat(I, Eye, 3, 3);
    cvAdd(Eye, Ws, Rtmp); //R = Eye + Ws + W2s;
    cvAdd(Rtmp, W2s, R);
    cvReleaseMat(&W); cvReleaseMat(&W2);
    cvReleaseMat(&Ws); cvReleaseMat(&W2s);
    cvReleaseMat(&Rtmp); cvReleaseMat(&Eye);
}
void WriteParameters(CameraParam *cp, char *name) {
    FILE *file;
    file = fopen(name, "w");
    fprintf(file, "intrinsic parameters\n");
    fprintf(file, "-------------------------\n");
    fprintf(file, "alphaX = %f\n", cp->alphaX);
    fprintf(file, "alphaY = %f\n", cp->alphaY);
    fprintf(file, "x0 = %f\n", cp->x0);
    fprintf(file, "y0 = %f\n", cp->y0);
    fprintf(file, "skew = %f\n", cp->skew);
    fprintf(file, "\n");
    fprintf(file, "radial distortion\n");
    fprintf(file, "-------------------------\n");
    fprintf(file, "k1 = %f\n", cp->k1);
    fprintf(file, "k2 = %f\n", cp->k2);
    fprintf(file, "\n");
    for(int k = 0; k < cp->len; k++){
        fprintf(file, "extrinsic parameters of image %d\n", k);
        fprintf(file, "-----------------------------------------\n");
        fprintf(file, "r1 = (%f, %f, %f)\n",
                cp->r1[0][k], cp->r1[1][k], cp->r1[2][k]);
        fprintf(file, "r2 = (%f, %f, %f)\n",
                cp->r2[0][k], cp->r2[1][k], cp->r2[2][k]);
        fprintf(file, "r3 = (%f, %f, %f)\n",
                cp->r3[0][k], cp->r3[1][k], cp->r3[2][k]);
        fprintf(file, "t = (%f, %f, %f)\n",
                cp->t[0][k], cp->t[1][k], cp->t[2][k]);
        fprintf(file, "\n");
    }
    fclose(file);
}

        
    
