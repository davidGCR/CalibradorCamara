//
//  main.cpp
//  testOpencv
//
//  Created by David Choqueluque Roman on 12/1/18.
//  Copyright © 2018 David Choqueluque Roman. All rights reserved.
//

#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"

#include "ellipse.h"
//#include <cv.h>
#include <iostream>
using namespace cv;
using namespace std;

const int  MIN_POINTS_ELL_FT = 4; //minimo numero de puntos para ellipse fitting
const int  IDX_SON = 2; //indice del hijo en jerarquia
const int  IDX_FATHER = 3; //indice del padre en jerarquia
const float  DST_2_ELLIPS = 5;
const int NUM_NEAR_ELLIPS = 2;
const int REAL_NUM_CTRL_PTS = 30;

Mat src, src_gray;
Mat dst, detected_edges;

int lowThreshold = 10;
const int max_lowThreshold = 100;
const int ratio = 3;
const int kernel_size = 3;
const char* window_name = "Edge Map";
Scalar red(0, 0, 255);
Scalar yellow(0, 255, 255);
Scalar blue(255, 0, 0);
Scalar green(0, 255, 0);
Scalar white(255, 255, 255);

void find_ellipses(Mat* img_preprocessing, Mat* img_out, int* n_fails){
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;
    vector<P_Ellipse> p_ellipses;
    vector<P_Ellipse> p_ellipses_f1;
    
    float radio, radio_son;
    
    findContours( (*img_preprocessing), contours, hierarchy, CV_RETR_TREE, CHAIN_APPROX_SIMPLE, Point(0, 0) );
    
//    for (int i=0; i<contours.size(); i++) {
//        cout<<"->"<<contours[i].size()<<endl;
//    }
    
//    cout<<"numero de contornos: "<<contours.size()<<endl;
//    cout<<"numero de jerarquias: "<<hierarchy.size()<<endl;
    
    for (int ct=0; ct<contours.size(); ct++) {
        if(contours[ct].size() > MIN_POINTS_ELL_FT){
            RotatedRect fit_ellipse = fitEllipse(Mat(contours[ct]));
            radio = (fit_ellipse.size.height + fit_ellipse.size.width)/4;
            if(hierarchy[ct][IDX_SON] != -1){
                int idson = hierarchy[ct][IDX_SON];
                if(contours[idson].size()>MIN_POINTS_ELL_FT){
                    RotatedRect fit_ellipse_son = fitEllipse(Mat(contours[idson]));
                    radio_son = (fit_ellipse_son.size.width + fit_ellipse_son.size.height)/4;
                    
                    if(cv::norm(fit_ellipse.center - fit_ellipse_son.center) <radio_son/2){
                        P_Ellipse p_ellipse;
                        p_ellipse.x = (fit_ellipse.center.x + fit_ellipse_son.center.x)/2;
                        p_ellipse.y = (fit_ellipse.center.y + fit_ellipse_son.center.y)/2;
                        p_ellipse.radio = radio;
                        p_ellipses.push_back(p_ellipse);
                        cv::ellipse((*img_out),fit_ellipse,green,2);
                        cv::ellipse((*img_out),fit_ellipse_son,yellow,2);
                        //                    cout<<"->"<<ct<<endl;
                    }
                    
                }
            }
//            cout<<"*"<<ct<<endl;
        }
    }
    int count = 0;
    float distance = 0;
    for(int i=0;i<p_ellipses.size();i++){
        for (int j=0; j<p_ellipses.size(); j++) {
            if(i!=j){
                distance = p_ellipses[i].distance(p_ellipses[j]);
                
                if (distance < DST_2_ELLIPS*p_ellipses[j].radio) {
//                    cout<<"distanceeee: "<<distance<<endl;
                    count++;
//                    cv::line((*img_out), p_ellipses[i].center(), p_ellipses[j].center(), blue,2);
                }
            }
        }
        if(count>=NUM_NEAR_ELLIPS){
            p_ellipses_f1.push_back(p_ellipses[i]);
//            cv::circle((*img_out), p_ellipses[i].center(), p_ellipses[i].radio, green,2);
        }
    }
    
    if (p_ellipses_f1.size()!=REAL_NUM_CTRL_PTS) {
        (*n_fails)++;
    }
//    int menor=0, mayor=0;
//    if(p_ellipses_f1.size()<REAL_NUM_CTRL_PTS){
//        menor++;
//    }else if(p_ellipses_f1.size()>REAL_NUM_CTRL_PTS){
//        mayor++;
//    }
//    cout<<"# menor: "<<menor<<", mayor: "<<mayor<<endl;
    



    //Draw ellipses
//    Mat drawing = Mat::zeros( (*img_preprocessing).size(), CV_8UC3 );
//
//    for( int i = 0; i < contours.size(); i++ )
//    {
//        Scalar color = Scalar(255,255,255);
//        drawContours( drawing, contours, i, color, 2, 8, hierarchy, 0, Point() );
//    }
//
//    imshow("Contornos", drawing);
//    cout<<"numero de contornos: "<<contours.size()<<endl;
//    cout<<"numero de jerarquias: "<<hierarchy.size()<<endl;
//
//    for (int i=0; i<contours.size(); i++) {
//        cout<<"->"<<contours[i].size()<<endl;
//    }
//
    //waitKey(0);
}

void preprocessing_frame(Mat* frame, Mat* frame_output){
    Mat blur;
    Mat frame_gray;
    Mat frame_thresholding;
    cvtColor( *frame,frame_gray, COLOR_BGR2GRAY );
    GaussianBlur( frame_gray, blur, Size(5,5),0 );
    //threshold(src_gray,detected_edges,0,255,THRESH_BINARY+THRESH_OTSU);
    adaptiveThreshold(frame_gray, frame_thresholding, 255, ADAPTIVE_THRESH_GAUSSIAN_C,CV_THRESH_BINARY,185,55);
    (*frame_output) = frame_thresholding;
    cout<<"preprocesada: "<<frame_thresholding.size<<endl;
}

int main()
{
    string path_data = "/Users/davidchoqueluqueroman/Desktop/CURSOS-MASTER/IMAGENES/CalibradorCamara/data/";
//    Mat img = imread(path_data+"padron1.png",CV_LOAD_IMAGE_COLOR);
//    imshow("Original",img);
//
//    Mat img_preprocessed;
//    preprocessing_frame(&img, &img_preprocessed);
////    imshow("Pre-procesada",img_preprocessed);
//
//    Mat img_ellipses = img.clone();
//    find_ellipses(&img_preprocessed, &img_ellipses);
//    imshow("Ellipses",img_ellipses);
//    waitKey(0);
    
    VideoCapture cap;
    cap.open(path_data+"PadronAnillos_02.mp4");
    if ( !cap.isOpened() ){
        cout << "Cannot open the video file. \n";
        return -1;
    }
    namedWindow("resultado",CV_WINDOW_AUTOSIZE);
    int total_frames=0;
    int n_fails = 0;
    while (1) {
        Mat frame;
        cap>>frame;
        if (frame.empty()) {
            cout << "Cannot capture frame. \n";
            break;
        }
        total_frames++;
        Mat frame_preprocessed;
        preprocessing_frame(&frame, &frame_preprocessed);
        //    imshow("Pre-procesada",img_preprocessed);
        Mat img_ellipses = frame.clone();
        find_ellipses(&frame_preprocessed, &img_ellipses,&n_fails);
        imshow("resultado",img_ellipses);
    
        if(waitKey(30) == 27)
        {
            break;
        }
    }
    
    cout<<"# fails: "<<n_fails<<" of "<<total_frames<<endl;
    return 0;
}
