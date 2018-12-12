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
const int ROWS_CTRL_PTS = 5;
const int COL_CTRL_PTS = 4;
const int REAL_NUM_CTRL_PTS = ROWS_CTRL_PTS * COL_CTRL_PTS;

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
Scalar rose(255, 0, 255);
Scalar celeste(255, 255 , 0);
Scalar black(0, 0 , 0);

void track_points(Mat& frame,vector<P_Ellipse>& p_ellipses,
    vector<P_Ellipse>& p_ellipses_f1 );

void find_ellipses(Mat* img_preprocessing, Mat* img_out, int* n_fails, vector<P_Ellipse>& control_points){
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
                        // cv::ellipse((*img_out),fit_ellipse_son,yellow,2);
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
    if (p_ellipses_f1.size()==REAL_NUM_CTRL_PTS) {
        cout<<"# entrooooooo: "<<endl;
        track_points((*img_out),p_ellipses_f1,control_points);
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
float point2point_distance(Point2f p1, Point2f p2) {
    return pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2);
}
float point2rect_distance(P_Ellipse e1, P_Ellipse e2, P_Ellipse ex) {
    
    Point2f p1 = e1.center();
    Point2f p2 = e2.center();
    Point2f x = ex.center();
    
    float l2 = point2point_distance(p1, p2);
    if (l2 == 0.0) return sqrt(point2point_distance(p1, x));
    //float result = abs((p2.y - p1.y) * x.x - (p2.x - p1.x) * x.y + p2.x * p1.y - p2.y * p1.x) / sqrt(pow(p2.y - p1.y, 2) + pow(p2.x - p1.x, 2));
    //return result;
    float t = ((x.x - p1.x) * (p2.x - p1.x) + (x.y - p1.y) * (p2.y - p1.y)) / l2;
    t = max(0.0f, min(1.0f, t));
    float result = point2point_distance(x, Point2f(p1.x + t * (p2.x - p1.x),
                                       p1.y + t * (p2.y - p1.y)));
    return sqrt(result);
}

bool sort_ellipses_by_x(P_Ellipse p1, P_Ellipse p2) {
    return p1.x < p2.x;
}
bool sort_ellipses_by_y(P_Ellipse p1, P_Ellipse p2) {
    return p1.y < p2.y;
}


void track_points(Mat& frame, vector<P_Ellipse>& p_ellipses_f1, vector<P_Ellipse>& control_points)
{
    if (p_ellipses_f1.size() < REAL_NUM_CTRL_PTS && control_points.size() < REAL_NUM_CTRL_PTS) {
        return;
    }
    // vector<Scalar> color_palette(5);
    // color_palette[0] = Scalar(255, 0, 255); //rosado
    // color_palette[1] = Scalar(255, 0, 0); //azul
    // color_palette[2] = Scalar(0, 255, 0); //verde
    // color_palette[3] = Scalar(0, 0 , 255);//rojo
    // color_palette[4] = Scalar(255, 255 , 0); //celeste

    int conincidencias = 0;
    int size_centers = control_points.size();
    float pattern_range = 2;
    float distance;
    float min_distance;
    int replace_point;
    // int line_color = 0;
    // vector<P_Ellipse> temp;
    vector<P_Ellipse> ellipses_in_line; //ellipses de una linea
    vector<P_Ellipse> limit_points;
    int rows = 0;

    cout << "Pattern Traking" << endl;
    cout<<"# vector 1: "<<control_points.size()<<", vector 2: "<<p_ellipses_f1.size()<<endl;

RNG rng(12345);

    if (control_points.size() == 0) {
        cout << "Pattern Traking fase 11111" << endl;
        
        size_centers = p_ellipses_f1.size(); //copiar centros de fase 1

        for (int i = 0; i < size_centers; i++) 
        {
            for (int j = 0; j < size_centers; j++) 
            {
                if (i != j) {
                    // temp.clear();
                    ellipses_in_line.clear();

                    conincidencias = 0; //contador de ellipses 
                    for (int k = 0; k < size_centers; k++) { //para cada par de puntos construir recta(i,j), y medir distancia de punto k a esta recta 
                        min_distance = point2rect_distance (p_ellipses_f1[i], p_ellipses_f1[j], p_ellipses_f1[k]);
                        if (min_distance < pattern_range) {
                            conincidencias++;
                            ellipses_in_line.push_back(p_ellipses_f1[k]);
                        }
                    }

                    if (conincidencias == ROWS_CTRL_PTS) { // se contraron la cantidad de patrones en una fila del patron
                        //ordenar los puntos de una recta con respecto al eje X
                        sort(ellipses_in_line.begin(), ellipses_in_line.end(), sort_ellipses_by_x);
                        
                        
                        // Scalar random_color = Scalar(rng.uniform(0,255), rng.uniform(0, 255), rng.uniform(0, 255));

                        line(frame, ellipses_in_line[0].center(), ellipses_in_line[ ellipses_in_line.size()-1].center(), yellow, 2);
                        cout<<"puntos en recta: "<< ellipses_in_line.size()<<", asumo que son: "<<ROWS_CTRL_PTS<<endl;
                        
                        for(int o=0;o<p_ellipses_f1.size();o++){
                            circle(frame, p_ellipses_f1[o].center(), 2, white, 5);
                        }
                        // for(int o=0;o<ellipses_in_line.size();o++){
                        //     circle(frame, ellipses_in_line[o].center(), 2, rose, 5);
                        // }
                        
                        
                        //medir extremos de la recta: RECTA VERTICAL
                        if (ellipses_in_line[ROWS_CTRL_PTS-1].x - ellipses_in_line[0].x < ellipses_in_line[0].radio) {
                            //ordenar los puntos de una recta con respecto al eje Y
                            sort(ellipses_in_line.begin(), ellipses_in_line.end(), sort_ellipses_by_y);
                            line(frame, ellipses_in_line[0].center(), ellipses_in_line[ ellipses_in_line.size()-1].center(), black, 2);
                            cout<<"puntos en recta2222: "<< ellipses_in_line.size()<<endl;
                            
                            for(int o=0;o<p_ellipses_f1.size();o++){
                                circle(frame, p_ellipses_f1[o].center(), 2, black, 5);
                            }
                            for(int o=0;o<ellipses_in_line.size();o++){
                                circle(frame, ellipses_in_line[o].center(), 2, green, 10);
                            }

                        }

                        bool found = false;
                        for (int l = 0; l < limit_points.size(); l++) {
                            //verificar que la  recta encontrada no fue identificada antes
                            if (limit_points[l].x == ellipses_in_line[0].x && limit_points[l].y == ellipses_in_line[0].y) {
                                found = true;
                                cout<<"found: "<<found<<endl;
                            }
                        }

                        if (!found) {
                            rows++;
                            for (int l = 0; l < ellipses_in_line.size(); l++) {
                                control_points.push_back(ellipses_in_line[l]);
                                putText(frame, to_string(control_points.size() - 1), ellipses_in_line[l].center(), FONT_HERSHEY_COMPLEX_SMALL, 1, red, 2);
                            }
                            //agregar puntos limites de la recta
                            limit_points.push_back(ellipses_in_line[0]);
                            limit_points.push_back(ellipses_in_line[ROWS_CTRL_PTS-1]);
                        }
                        else{
                            cout<<"no entroooooooo..."<<endl;
                        }
                    }
                }
            }
        }
        if (rows != ROWS_CTRL_PTS) {
            //cout << "rows " << rows << endl;
            control_points.clear();
        }
        //cout << "Elements checked " << distance_elements << " prom " << prom_distance / distance_elements << endl;
    } 
    // else {
    //     cout << "ESLE================================================================================= "<<endl;
    //     for (int p = 0; p < control_points.size(); p++) {
    //         replace_point = 0;
    //         min_distance = 100;
    //         // int cercano = p;
    //         for (int n = 0; n < p_ellipses_f1.size(); n++) { //buscar el centro mas cercano
    //             distance = control_points[p].distance(p_ellipses_f1[n]);
    //             if (min_distance > distance) {
    //                 min_distance = distance;
    //                 replace_point = n;
                    
    //             }
    //         }
    //         // cout << "p:"<<p<<", cercano id: "<<replace_point<<endl;
    //         if (min_distance > control_points[p].radio) {
    //             // cout << "break: " <<endl;
    //             min_distance = -1;
    //             break;
    //         }
    //         //  cout << "llegooooooooo: " << p_ellipses[p].center()<<","<<p_ellipses[replace_point].center()<<endl;
    //         line(frame, control_points[p].center(), control_points[replace_point].center(), red, 5);

    //         circle(frame, control_points[p].center(), control_points[p].radio/**1.5*/, yellow, 1);
    //         line(frame, control_points[p].center(), p_ellipses_f1[replace_point].center(),blue, 10);
    //         control_points[p] = p_ellipses_f1[replace_point];
    //         putText(frame, to_string(p), control_points[p].center(), FONT_HERSHEY_COMPLEX_SMALL, 0.7, cvScalar(255, 0, 0), 2);
    //     }
    //     if (min_distance == -1) {
    //         control_points.clear();
    //         return;
    //     }

    // }

    if (control_points.size() == REAL_NUM_CTRL_PTS) {
        //avgColinearDistance(pattern_centers);
        // line(frame, control_points[0].center() , control_points[ROWS_CTRL_PTS-1].center() , rose, 1); //rosado
        // line(frame, p_ellipses[5].center() , p_ellipses[4].center() , color_palette[1], 1); //azul
        // line(frame, p_ellipses[5].center() , p_ellipses[9].center() , color_palette[1], 1);
        // line(frame, p_ellipses[10].center(), p_ellipses[9].center() , color_palette[2], 1);
        // line(frame, p_ellipses[10].center(), p_ellipses[14].center(), color_palette[2], 1);
        // line(frame, p_ellipses[15].center(), p_ellipses[14].center(), color_palette[3], 1);
        // line(frame, p_ellipses[15].center(), p_ellipses[19].center(), color_palette[3], 1);

        // line(frame, p_ellipses[0].center(), p_ellipses[15].center(), celeste, 1);
        // line(frame, p_ellipses[1].center(), p_ellipses[16].center(), celeste, 1);
        // line(frame, p_ellipses[2].center(), p_ellipses[17].center(), celeste, 1);
        // line(frame, p_ellipses[3].center(), p_ellipses[18].center(), celeste, 1);
        // line(frame, p_ellipses[4].center(), p_ellipses[19].center(), celeste, 1);

    }

}

void preprocessing_frame(Mat* frame, Mat* frame_output){
    Mat blur;
    Mat frame_gray;
    Mat frame_thresholding;
    cvtColor( *frame,frame_gray, COLOR_BGR2GRAY );
    GaussianBlur( frame_gray, blur, Size(5,5),0 );
    //threshold(src_gray,detected_edges,0,255,THRESH_BINARY+THRESH_OTSU);
    adaptiveThreshold(frame_gray, frame_thresholding, 255, ADAPTIVE_THRESH_GAUSSIAN_C,CV_THRESH_BINARY,125,20);
    (*frame_output) = frame_thresholding;
    // cout<<"preprocesada: "<<frame_thresholding.size<<endl;
}

int main()
{
    string path_data = "/home/david/Escritorio/calib-data/";
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
    vector<P_Ellipse> control_points;
    
    VideoCapture cap;
    cap.open(path_data+"padron2.avi");
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
        find_ellipses(&frame_preprocessed, &img_ellipses,&n_fails,control_points);
        imshow("resultado",img_ellipses);
    
        if(waitKey(2) == 27)
        {
            break;
        }
    }
    
    cout<<"# fails: "<<n_fails<<" of "<<total_frames<<endl;
    return 0;
}
