
void first_calibration_homework(string video_file){
    //Camera calibration
    float rms=-1;
    // int NUM_FRAMES_FOR_CALIBRATION = 45;
    int DELAY_TIME = 50;
    vector<vector<Point2f>> imagePoints;
    Mat cameraMatrix;
    Mat distCoeffs = Mat::zeros(8, 1, CV_64F);

    //found pattern points
    vector<P_Ellipse> control_points;

    VideoCapture cap;
    cap.open(video_file);
    if ( !cap.isOpened() ){
        cout << "Cannot open the video file. \n";
        return;
    }


    Mat frame;
    cap.read(frame);
    //total_frames = cap.get(CAP_PROP_FRAME_COUNT);
    int w = frame.rows;
    int h = frame.cols;
    Size imageSize(h, w);
    cout<<"imagesize: "<<imageSize<<endl;


    namedWindow("resultado",CV_WINDOW_AUTOSIZE);
    int total_frames=0;
    int n_fails = 0;
    float frame_time = 0;
    int points_detected = 0;


    while (1) {
        Mat frame, rview;
        cap>>frame;
        if (frame.empty()) {
            cout << "Cannot capture frame. \n";
            break;
        }
        total_frames++;
        frame_time = cap.get(CV_CAP_PROP_POS_MSEC)/1000;

        Mat frame_preprocessed;
        auto t11 = std::chrono::high_resolution_clock::now();
        preprocessing_frame(&frame, &frame_preprocessed);
        //    imshow("Pre-procesada",img_preprocessed);
        Mat img_ellipses = frame.clone();

        //if(total_frames == 3100)//44
        points_detected = find_ellipses(&frame_preprocessed, &img_ellipses,control_points,frame_time,n_fails);
        auto t12 = std::chrono::high_resolution_clock::now();

        duration += std::chrono::duration_cast<std::chrono::milliseconds>(t12 - t11).count();

        imshow("Normal", img_ellipses);

        if(rms==-1){
            rview = img_ellipses.clone();
            if(total_frames% DELAY_TIME == 0 && points_detected == REAL_NUM_CTRL_PTS){
                vector<Point2f> buffer = ellipses2Points(control_points) ;
                imagePoints.push_back(buffer);
                for (int i=0; i<buffer.size(); i++) {
                    cout<<"("<<buffer[i].x<<buffer[i].y<<") - ";
                }

                if(imagePoints.size()==NUM_FRAMES_FOR_CALIBRATION){
                    cout<<"=======================calibrar..."<<endl;
                    rms= calibrate_camera(imageSize, cameraMatrix, distCoeffs, imagePoints);
                    cout << "cameraMatrix " << cameraMatrix << endl;
                    cout << "distCoeffs " << distCoeffs << endl;
                    cout << "rms: " << rms << endl;
                }
            }
        }
        else
        {
            Mat map1, map2;
            initUndistortRectifyMap(cameraMatrix, distCoeffs, Mat(),
                                    getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, imageSize, 1, imageSize, 0),
                                    imageSize, CV_16SC2, map1, map2);

            //for(int i = 0; i < (int)s.imageList.size(); i++ )
            //{
            if(img_ellipses.empty())
                continue;
            remap(img_ellipses, rview, map1, map2, INTER_LINEAR);
            imshow("final", rview);
            char c = (char)waitKey(1);
        }

        //        cvtColor(frame_preprocessed,frame_preprocessed, COLOR_GRAY2RGB);
        //imshow("other",frame_preprocessed);
        //ShowManyImages("resultado", n_fails, total_frames, 4, frame, frame_preprocessed, img_circles, img_ellipses);
        //        ShowManyImages("resultado", n_fails, total_frames, rms, cameraMatrix, 4, frame, frame_preprocessed, img_ellipses, rview);

        if(waitKey(1) == 27)
        {
            break;
        }
    }
    cout<<"# fails: "<<n_fails<<" of: "<<total_frames<<endl;
    cout<<"# frames: "<<total_frames<<" time: "<<duration<<endl;
}

int debug_images_fronto()
{
    
    //string path_data = "/Users/davidchoqueluqueroman/Desktop/CURSOS-MASTER/IMAGENES/testOpencv/data/";
    string frames_file = PATH_DATA_FRAMES;
    // string video_file = PATH_DATA+"cam2/anillos.avi";
    
    //initial frame for get size of frames
    Mat fronto_img, fronto_img_pre, fronto_img_ell;
    
    vector<cv::String> fn;
    glob(PATH_DATA_FRAMES+"3-fronto/*.jpg", fn, false);
    
    vector<Mat> images;
    size_t count = fn.size(); //total number  files in images folder
    
    size_t n = 30;
    
    if (count>0) {
        int fails = 0;
        int points_detected;
        vector<P_Ellipse> out_control_points;
        
        for (size_t i=0; i<count; i++){
            out_control_points.clear();
            fronto_img = imread(fn[i]);
            // cout<<"chanels : "<<fronto_img.channels()<<endl;
            preprocessing_frame2(fronto_img,fronto_img_pre);
            fronto_img_ell = fronto_img.clone();
            points_detected = find_ellipses(&fronto_img_pre, &fronto_img_ell,out_control_points,0,fails);
            cout<<"cp detected : "<<points_detected<<endl;
            
            imshow("fronto",fronto_img);
            imshow("preprocess",fronto_img_pre);
            imshow("ellipses",fronto_img_ell);
            // ShowManyImages("resultado", 2, 3, 0, 0, 4, fronto_img, fronto_img_pre, fronto_img,fronto_img);
            // cout<<"file: "<<fn[i]<<endl;
            if(waitKey(1000) == 27)
            {
                break;
            }
            
        }
        return 1;
    }
    
    cout<<"Houston, we have a problem: Not frames for debug..."<<endl;
    return 0;
    // images.push_back(imread(fn[i]));
}
//#include <stdlib.h>
//#include <cmath>
////#include "cv.h"
////#include "opencv2/cxcore.hpp"
//#include "opencv2/highgui.hpp"
//#include "optm/utility.hpp"
//#include "optm/corner.hpp"
//#include "optm/homography.hpp"
//#include "optm/calibration.hpp"
//#include <iostream>
//
//using namespace cv;
//using namespace std;
//
//void MarkCornerPoints(IplImage *image, Points *cornerMap);
//
//int main(int argc, char **argv) {
//    // declaration
//    IplImage *inImage = 0;
//    IplImage *colorImage = 0;
//    PairPoints corspMap;
//    Points cornerMap;
//    CameraParam cp;
//    HomographySet h;
//    PairPointSet imagePt;
//    char inImageName[80];
//    char imageNamePrefix[80];
//    CvMat *H = cvCreateMat(3, 3, CV_32FC1); // homography
//    CvMat *K = cvCreateMat(3, 3, CV_32FC1); // intrinsic matrix
//    int i, j;
//    int numOfImages, imageNumber;
//    int numPointsRow = NUM_H_LINES;
//    int numPointsColumn = NUM_V_LINES;
//
//    int pointOrder;
//    if(argc == 3){
//        strcpy(imageNamePrefix, argv[1]);
//        numOfImages = atoi(argv[2]);
//    }else{
//        printf("\n");
//        printf(" Usage: hw5 imageNamePrefix [number of images]\n");
//        printf("\n");
//    }
//    // set struct parameters
//    h.len = numOfImages;
//    h.numPointsRow = numPointsRow;
//    h.numPointsColumn = numPointsColumn;
//    corspMap.len = numPointsRow * numPointsColumn;
//    cornerMap.len = numPointsRow * numPointsColumn;
//    imagePt.imageLen = numOfImages;
//    imagePt.pointLen = numPointsRow * numPointsColumn;
//    cp.len = numOfImages;
//    for(imageNumber = 0; imageNumber < numOfImages; imageNumber++){
//        sprintf(inImageName, "../image/%s%d.pgm", imageNamePrefix, imageNumber);
//        inImage = cvLoadImage(inImageName, -1);
//        if(!inImage){
//            printf("Could not load image file: %s\n", inImageName);
//            exit(0);
//        }
//        ////////////////////////////////////////////////////////////
//        // 1. Corner Detection //
//        // find indexed corner using Hough xform //
//        ////////////////////////////////////////////////////////////
//        IndexedCornerDetect(inImage, &cornerMap);
//        // display and store the result image
//        colorImage = cvCreateImage(cvGetSize(inImage), IPL_DEPTH_8U, 3);
//        cvCvtColor(inImage, colorImage, CV_GRAY2BGR);
//        MarkCornerPoints(colorImage, &cornerMap);
//        cvNamedWindow("output image", CV_WINDOW_AUTOSIZE);
//        cvShowImage("output image", colorImage);
//        cvWaitKey(0);
//        cvDestroyWindow("output image");
//        WriteImage(colorImage, "indexedcorner.jpg");
//        ////////////////////////////////////////////////////////////
//        // 2. Homography Estimation //
//        // calculate the homoraphy using DLT(SVD) //
//
//        ////////////////////////////////////////////////////////////
//        // create the correspondence map for calculating H
//        for(i = 0; i < numPointsRow; i++){
//            for(j = 0; j < numPointsColumn; j++){
//                pointOrder = i*numPointsColumn + j;
//                // 1 square length = unit 1
//                // so, if actual 1 square length = a
//                // we need scale unit with a, after calibration.
//                corspMap.pt1I[pointOrder] = i;
//                corspMap.pt1J[pointOrder] = j;
//                corspMap.pt2I[pointOrder] = cornerMap.ptI[pointOrder];
//                corspMap.pt2J[pointOrder] = cornerMap.ptJ[pointOrder];
//                // store input corner points for radial distortion calculation
//                imagePt.ptmI[pointOrder][imageNumber] = i;
//                imagePt.ptmJ[pointOrder][imageNumber] = j;
//                imagePt.ptiI[pointOrder][imageNumber] = cornerMap.ptI[pointOrder];
//                imagePt.ptiJ[pointOrder][imageNumber] = cornerMap.ptJ[pointOrder];
//            }
//        }
//        // estimate correspondences
//        HomograhyEstimation(&corspMap, H);
//        // store the estimated homography
//        float H33 = cvmGet(H, 2, 2);
//        // normalize homography : however, this step is not required
//        for(int n = 0; n < 3; n++){
//            for(int m = 0; m < 3; m++){
//                h.H[n][m][imageNumber] = cvmGet(H, n, m) / H33;
//            }
//        }
//    }
//    ////////////////////////////////////////////////////////////
//    // 3. Camera Calibration //
//    // calculate intrinsic and extrinsic parameters //
//    ////////////////////////////////////////////////////////////
//    CameraCalibration(&h, &cp, &imagePt);
//    ////////////////////////////////////////////////////////////
//    // 5. Results //
//    ////////////////////////////////////////////////////////////
//    IplImage *outImage = cvCreateImage(cvGetSize(inImage), IPL_DEPTH_8U, 1);
//    for(imageNumber = 0; imageNumber < numOfImages; imageNumber++){
//        sprintf(inImageName, "../image/%s%d.pgm", imageNamePrefix, imageNumber);
//        inImage = cvLoadImage(inImageName, -1);
//
//        if(!inImage){
//            printf("Could not load image file: %s\n", inImageName);
//            exit(0);
//        }
//        //H = KA, where A = [r1 r2 t];
//        // get intrinsic matrix K
//        float kArr[9] = {cp.alphaX, cp.skew, cp.x0,
//            0.f , cp.alphaY, cp.y0,
//            0.f , 0.f , 1};
//
//        Array2CvMat(kArr, K, 3, 3);
//        // get undistorted images
//        float distCoeffs[4] = {cp.k1, cp.k2, 0.f, 0.f};
//
//        cvUnDistortOnce(inImage, outImage, kArr, distCoeffs, 1);
//        CvMat *A = cvCreateMat(3, 3, CV_32FC1);
//        for(i = 0; i < 3; i++){
//            cvmSet(A, i, 0, cp.r1[i][imageNumber]);
//            cvmSet(A, i, 1, cp.r2[i][imageNumber]);
//            cvmSet(A, i, 2, cp.t[i][imageNumber]);
//        }
//        cvMatMul(K, A, H);
//        float h[9];
//        CvMat2Array(H, h, 3, 3);
//        int height = inImage->height;
//        int width = inImage->width;
//        for(i = 0; i < numPointsRow; i++){
//            for(j = 0; j < numPointsColumn; j++){
//                pointOrder = i*numPointsColumn + j;
//                float x1 = h[0] * j + h[1] * i + h[2];
//                float x2 = h[3] * j + h[4] * i + h[5];
//                float x3 = h[6] * j + h[7] * i + h[8];
//                float vi = min(height - 1, max(0, x2 / x3));
//                float ui = min(width - 1, max(0, x1 / x3));
//                // radial distortion
//                float tpU = (ui - cp.x0);
//                float tpV = (vi - cp.y0);
//                float r = pow(tpU / cp.alphaX , 2) + pow(tpV / cp.alphaY, 2);
//                cornerMap.ptJ[pointOrder] = cvRound(ui + tpU *
//                                                    (cp.k1 * r + cp.k2 * pow(r, 2)));
//                cornerMap.ptI[pointOrder] = cvRound(vi + tpV *
//                                                    (cp.k1 * r + cp.k2 * pow(r, 2)));
//
//                cvCircle(inImage, cvPoint(cornerMap.ptJ[pointOrder],
//                                          cornerMap.ptI[pointOrder]), 2, cvScalar(0, 255, 0), 3);
//            }
//        }
//        WriteImage(inImage, "result.jpg");
//        WriteImage(outImage, "distortionCorrResult.jpg");
//        cvNamedWindow("output image", CV_WINDOW_AUTOSIZE);
//        cvShowImage("output image", inImage);
//        cvWaitKey(0);
//        cvDestroyWindow("output image");
//        cvNamedWindow("output image", CV_WINDOW_AUTOSIZE);
//        cvShowImage("output image", outImage);
//        cvWaitKey(0);
//        cvDestroyWindow("output image");
//        cvReleaseMat(&A);
//    }
//    // release the images a1nd matrix
//    cvReleaseImage(&inImage);
//    cvReleaseImage(&colorImage);
//    cvReleaseMat(&H);
//    cvReleaseMat(&K);
//    return 0;
//}
////
//// function : MarkCornerPoints
//// usage : MarkCornerPoints(image, cornerMap);
//// -------------------------------------------------
//// This function draws marks in input image corresponding
//// to the corner map.
////
//void MarkCornerPoints(IplImage *image, Points *cornerMap) {
//    int i, j, k;
//    char label[10];
//    CvFont font;
//    double hScale = .5;
//
//    double vScale = .5;
//    int lineWidth = 2;
//    cvInitFont(&font, CV_FONT_HERSHEY_SIMPLEX|CV_FONT_ITALIC,
//               hScale, vScale, 0, lineWidth);
//    for(k = 0; k < cornerMap->len; k++){
//        i = cornerMap->ptI[k];
//        j = cornerMap->ptJ[k];
//        cvCircle(image, cvPoint(j, i), 3, cvScalar(0, 255, 0), 3);
//        sprintf(label, "%d", k);
//        cvPutText (image, label, cvPoint(j, i), &font, cvScalar(0, 0, 255));
//    }
//}
//

 ////
 ////  main.cpp
 ////  testOpencv
 ////
 ////  Created by David Choqueluque Roman on 12/1/18.
 ////  Copyright © 2018 David Choqueluque Roman. All rights reserved.
 ////