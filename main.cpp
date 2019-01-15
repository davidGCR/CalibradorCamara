////
////  main.cpp
////  testOpencv
////
////  Created by David Choqueluque Roman on 12/1/18.
////  Copyright © 2018 David Choqueluque Roman. All rights reserved.
////

#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include <chrono>
#include <stdio.h>
#include <stdarg.h>
#include <string>
#include <time.h>
#include <cmath>
#include <iomanip>
#include "opencv2/calib3d.hpp"

#include "ellipse.h"
#include "Line.h"
#include "constants.h"
//#include <cv.h>
#include <iostream>
using namespace cv;
using namespace std;


Mat src, src_gray;
Mat img_circles;
Mat dst, detected_edges;
auto duration = 0;

int lowThreshold = 10;
const char* window_name = "Edge Map";
Scalar red(0, 0, 255);
Scalar yellow(0, 255, 255);
Scalar blue(255, 0, 0);
Scalar green(0, 255, 0);
Scalar white(255, 255, 255);
Scalar rose(255, 0, 255);
Scalar celeste(255, 255 , 0);
Scalar black(0, 0 , 0);

int track_points(Mat& frame,vector<P_Ellipse>& p_ellipses,
                 vector<P_Ellipse>& pp_control_points, float frame_time );

int find_ellipses(Mat* img_preprocessing, Mat* img_out, vector<P_Ellipse>& control_points, float frame_time = 0,int* n_fails=0){
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;
    vector<P_Ellipse> p_ellipses;
    vector<P_Ellipse> pp_control_points;
    
    float radio, radio_son;
    
    findContours( (*img_preprocessing), contours, hierarchy, CV_RETR_TREE, CHAIN_APPROX_SIMPLE, Point(0, 0) );
    
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
                        p_ellipse.fit_ellipse = fit_ellipse;
                        p_ellipses.push_back(p_ellipse);
                        
                        //cv::ellipse((*img_out),fit_ellipse,green,2);
                        // cv::ellipse((*img_out),fit_ellipse_son,yellow,2);
                        //                    cout<<"->"<<ct<<endl;
                    }
                    
                }
            }
        }
    }
    
    img_circles = img_out->clone();
    
    //cout << "-> " << p_ellipses.size() << endl;
    
    float distance = 0;
    for(int i=0;i<p_ellipses.size();i++){ //filtrar ellipses por distancias
        int count = 0;
        //cout << "points: " << p_ellipses[i].x << " "<<p_ellipses[i].y << " "<< p_ellipses[i].radio<<endl;
        for (int j=0; j<p_ellipses.size(); j++) {
            if(i!=j){
                distance = p_ellipses[i].distance(p_ellipses[j]);
                
                double medirDistancia;
                
                if(p_ellipses[j].radio > p_ellipses[i].radio)
                {
                    medirDistancia = p_ellipses[i].radio;
                }
                else
                {
                    medirDistancia = p_ellipses[j].radio;
                }
                
                if (distance < DST_2_ELLIPS*medirDistancia) {
                    //cout<< "i: " << i << " j: " <<j<<" distance: "<<distance << " radio: " << p_ellipses[j].radio<<endl;
                    count++;
                    //                    cv::line((*img_out), p_ellipses[i].center(), p_ellipses[j].center(), blue,2);
                }
            }
        }
        if(count>=NUM_NEAR_ELLIPS){
            pp_control_points.push_back(p_ellipses[i]);
            
            //cout<<"raius: "<<pp_control_points[i].radio<<endl;
            //            cv::circle((*img_out), p_ellipses[i].center(), p_ellipses[i].radio, green,2);
        }
        else
        {
            p_ellipses.erase(p_ellipses.begin()+i--);
        }
    }
    
    //cout << "-> " << pp_control_points.size() << endl;
    int n_points = 0;
    if (pp_control_points.size()==REAL_NUM_CTRL_PTS) {
        //cout<<"# entro: "<< *n_fails<<endl;
        for(int j=0; j<pp_control_points.size(); j++){
            cv::ellipse((*img_out),pp_control_points[j].fit_ellipse,white,2);
        }
        n_points = track_points((*img_out),pp_control_points,control_points,frame_time);
        //cout << "->"<<n_points << endl;
    }
    else{
        //cout<<"# entro: "<< *n_fails<<endl;
        (*n_fails)++;
    }
    //    int menor=0, mayor=0;
    //    if(p_ellipses_f1.size()<REAL_NUM_CTRL_PTS){
    //        menor++;
    //    }else if(p_ellipses_f1.size()>REAL_NUM_CTRL_PTS){
    //        mayor++;
    //    }
    //    cout<<"# menor: "<<menor<<", mayor: "<<mayor<<endl;
    
    
    return n_points;
    
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
/*
 * e1: punto extremo 1
 * e2: punto extremo 2
 * ex:
 */
float point2rect_distance(P_Ellipse e1, P_Ellipse e2, P_Ellipse ex) {
    
    Point2f p1 = e1.center();
    Point2f p2 = e2.center();
    Point2f x = ex.center();
    
    float l2 = point2point_distance(p1, p2);
    if (l2 == 0.0) return sqrt(point2point_distance(p1, x));
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
bool sort_lines_by_x(Line p1, Line p2) {
    return p1.getLeftPoint().x < p2.getLeftPoint().x;
}

bool verify_vertical_lines_order(vector<P_Ellipse> control_points){
    cout<<"************************* orednaaaaaaaaa *****************************"<<endl;
    float x_left = control_points[0].x;
    for (int i=PATTERN_NUM_COLS; i<control_points.size(); i+=PATTERN_NUM_COLS) {
        if(x_left>control_points[i].x){
            return  true;
        }
        else{
            x_left = control_points[i].x;
        }
    }
    return false;
}
void reorder_vertical_lines(Mat& frame,vector<Line>& lines,vector<P_Ellipse>& control_points){
    Line line;
    for (int i=0; i<control_points.size(); i++) {
        if(!line.isFull()){
            line.add_ellipse(control_points[i]);
        }
        else{
            lines.push_back(line);
            line.clear();
        }
    }
    sort(lines.begin(), lines.end(), sort_lines_by_x);
    control_points.clear();
    for (int i=0; i<lines.size(); i++) {
        for (int j=0; j<PATTERN_NUM_COLS; j++) {
            control_points.push_back(lines[i].getEllipse(j));
            putText(frame, to_string(control_points.size()-1), control_points[i].center(), FONT_HERSHEY_COMPLEX_SMALL, 1, yellow, 2);
        }
    }
    
    
    
    //    P_Ellipse left = control_points[0];
    //    P_Ellipse tmp;
    //
    //    for (int i=ROWS_CTRL_PTS; i<control_points.size(); i+=ROWS_CTRL_PTS) {
    //        if(left.x < control_points[i].x){
    //            left = control_points[i];
    //        }
    //        else{
    //            tmp = left;
    //            control_points[i];
    //        }
    //    }
}

void initialize_track_points(Mat& frame, vector<P_Ellipse>& pp_control_points, vector<P_Ellipse>& control_points, float frame_time){
    int conincidencias = 0;
    int size_centers = (int)control_points.size();
    float pattern_range = 2; //threshold de distancia punto a recta
    float min_distance;
    vector<P_Ellipse> ellipses_in_line; //ellipses de una linea/recta
    vector<P_Ellipse> limit_points;//puntos extremos de una linea/recta
    vector<Line> lines;
    Line line;
    int rows = 0;
    int label=-1;
    Scalar color_text = red;
    P_Ellipse ellipse_rect_left, ellipse_rect_right;
    bool flac_vertical = false;
    float distancia_x = 0;
    
    size_centers = (int)pp_control_points.size(); //numero de ellipsee encontradas en fase 1:preprocesamiento
    
    for (int i = 0; i < size_centers; i++)
    {
        for (int j = 0; j < size_centers; j++)
        {
            if (i != j)
            {
                ellipses_in_line.clear();
                line.clear();
                
                conincidencias = 0; //contador de ellipses
                for (int k = 0; k < size_centers; k++) { //para cada par de puntos construir recta(i,j), y medir distancia de punto k a esta recta
                    min_distance = point2rect_distance (pp_control_points[i], pp_control_points[j], pp_control_points[k]);
                    if (min_distance < pattern_range) {
                        conincidencias++;
                        ellipses_in_line.push_back(pp_control_points[k]);
                    }
                }
                
                if (conincidencias == PATTERN_NUM_COLS) { // se contraron la cantidad de patrones en una fila del patron
                    //ordenar los puntos de una recta con respecto al eje X
                    sort(ellipses_in_line.begin(), ellipses_in_line.end(), sort_ellipses_by_x);
                    
                    
                    // Scalar random_color = Scalar(rng.uniform(0,255), rng.uniform(0, 255), rng.uniform(0, 255));
                    
                    ellipse_rect_left = ellipses_in_line[0]; //extremo izquierdo de recta
                    ellipse_rect_right = ellipses_in_line[ ellipses_in_line.size()-1]; //extremo derecho de recta
                    //                        for(int o=0;o<p_ellipses_f1.size();o++){
                    //                            circle(frame, p_ellipses_f1[o].center(), 2, white, 5);
                    //                        }
                    
                    //medir extremos de la recta: RECTA VERTICAL
                    //                        if (ellipses_in_line[ROWS_CTRL_PTS-1].x - ellipses_in_line[0].x < ellipses_in_line[0].radio)
                    
                    //                    if (ellipse_rect_right.x - ellipse_rect_left.x < ellipse_rect_left.radio)
                    distancia_x = ellipse_rect_right.x - ellipse_rect_left.x;
                    if ( distancia_x < 5*ellipse_rect_left.radio)
                    {
                        flac_vertical = true;
                        float x1 = ellipse_rect_left.x;
                        float x2 = ellipse_rect_right.x;
                        //cout<<"recta vertical!!!!!!!!!!!!!! x1: "<<x1<<", x2: "<<x2<<endl;
                        //ordenar los puntos de una recta con respecto al eje Y
                        sort(ellipses_in_line.begin(), ellipses_in_line.end(), sort_ellipses_by_y);
                        cv::line(frame, ellipses_in_line[0].center(), ellipses_in_line[ ellipses_in_line.size()-1].center(), green, 2);
                        ellipse_rect_left = ellipses_in_line[0]; //extremo izquierdo de recta
                        ellipse_rect_right = ellipses_in_line[ ellipses_in_line.size()-1]; //extremo derecho de recta
                        
                        for(int o=0;o<ellipses_in_line.size();o++){
                            
                            circle(frame, ellipses_in_line[o].center(), 2, rose, 2);
                        }
                    }
                    else{
                        //line(frame, ellipse_rect_left.center(), ellipse_rect_right.center(), yellow, 2);
                    }
                    
                    bool found = false;
                    for (int l = 0; l < limit_points.size(); l++) {
                        //verificar que la  recta encontrada no fue identificada antes
                        if (limit_points[l].x == ellipses_in_line[0].x && limit_points[l].y == ellipses_in_line[0].y) {
                            found = true;
                            //cout<<"found: "<<found<<endl;
                        }
                    }
                    
                    if (!found) {
                        rows++;
                        for (int lbl = 0; lbl < ellipses_in_line.size(); lbl++) {
                            control_points.push_back(ellipses_in_line[lbl]);
                            label = (int)(control_points.size() - 1);
                            putText(frame, to_string(label), ellipses_in_line[lbl].center(), FONT_HERSHEY_COMPLEX_SMALL, 1, color_text, 2);
                            //                                if(control_points.size() - 1 == 15){
                            //                                    //a medida que se va agregando se etiqueta
                            //                                    label = (int)(control_points.size() - 1);
                            //                                    putText(frame, to_string(label), ellipses_in_line[lbl].center(), FONT_HERSHEY_COMPLEX_SMALL, 1, red, 2);
                            //                                }
                        }
                        
                        //agregar puntos limites de la recta
                        limit_points.push_back(ellipse_rect_left);
                        limit_points.push_back(ellipse_rect_right);
                    }
                    //                    string s = "/Users/davidchoqueluqueroman/Desktop/CURSOS-MASTER/IMAGENES/testOpencv/data/captures/cap-"+to_string(frame_time)+".jpg";
                    //                    imwrite(s,frame);
                    
                }
            }
        }
    }
    if (rows != 4) {
        //cout << "rows " << rows << endl;
        control_points.clear();
    }
    //    if(verify_vertical_lines_order(control_points)){
    //        reorder_vertical_lines(frame, lines, control_points);
    //        string s = "/Users/davidchoqueluqueroman/Desktop/CURSOS-MASTER/IMAGENES/testOpencv/data/captures/cap-"+to_string(frame_time)+".jpg";
    //        imwrite(s,frame);
    //    }
    
}
/*
 * frame: imagen preprocesada
 * p_ellipses_f1: ellipses encontradas en preprocesamiento
 * control_points: resultado de track_points
 */
int track_points(Mat& frame, vector<P_Ellipse>& pp_control_points, vector<P_Ellipse>& control_points, float frame_time)
{
    
    int label=-1;
    Scalar color_text = red;
    //cout << "track_points: "<<control_points.size() << endl;
    if (control_points.size() == 0)
    {
        initialize_track_points(frame, pp_control_points, control_points, frame_time);
        
    }
    else{
        
        //cout << "ESLE=================== pp: "<<pp_control_points.size()<<" , cp: "<<control_points.size()<<endl;
        float dist_min;
        float dist = 0;
        int near_point = 0;
        for(int i=0;i<control_points.size();i++){
            dist_min = 100;
            for (int j=0; j<pp_control_points.size(); j++) {
                dist = control_points[i].distance(pp_control_points[j]);
                if(dist < dist_min){
                    dist_min = dist;
                    near_point = j;
                }
            }
            
            cv::line(frame, control_points[i].center(), pp_control_points[near_point].center(), yellow,2);
            //cout << "= ************* dist min: "<<dist_min<<", dist: "<<dist<<endl;
            if(dist_min > TRACK_THRESHOLD){
                //cout << "======lejosssssssssssssssssssssssssss======"<<endl;
                dist_min = -1;
                //                string s = "/Users/davidchoqueluqueroman/Desktop/CURSOS-MASTER/IMAGENES/testOpencv/data/captures/cap-"+to_string(frame_time)+".jpg";
                //                imwrite(s,frame);
                break;
            }
            
            //cout<<"labeleddddddd: "<<i<<endl;
            label = i;
            control_points[i] = pp_control_points[near_point];
            putText(frame, to_string(label), pp_control_points[near_point].center(), FONT_HERSHEY_COMPLEX_SMALL, 1, color_text, 2);
            //control_points=pp_control_points;
        }
        //        string s = "/Users/davidchoqueluqueroman/Desktop/CURSOS-MASTER/IMAGENES/testOpencv/data/captures/cap-"+to_string(frame_time)+".jpg";
        //        imwrite(s,frame);
        if (dist_min == -1) {
            //cout<<"======================== CLEAR ==============================: "<<endl;
            control_points.clear();
            initialize_track_points(frame, pp_control_points, control_points, frame_time);
            return control_points.size();
        }
    }
    
    // if (control_points.size() == REAL_NUM_CTRL_PTS) {
    
    // }
    return control_points.size();
}


void thresholdIntegral(cv::Mat &inputMat, cv::Mat &outputMat)
{
    // accept only char type matrices
    CV_Assert(!inputMat.empty());
    CV_Assert(inputMat.depth() == CV_8U);
    CV_Assert(inputMat.channels() == 1);
    CV_Assert(!outputMat.empty());
    CV_Assert(outputMat.depth() == CV_8U);
    CV_Assert(outputMat.channels() == 1);
    
    // rows -> height -> y
    int nRows = inputMat.rows;
    // cols -> width -> x
    int nCols = inputMat.cols;
    
    // create the integral image
    cv::Mat sumMat;
    cv::integral(inputMat, sumMat);
    
    CV_Assert(sumMat.depth() == CV_32S);
    CV_Assert(sizeof(int) == 4);
    
    int S = MAX(nRows, nCols)/16;
    double T = 0.1;
    
    // perform thresholding
    int s2 = S/2;
    int x1, y1, x2, y2, count, sum;
    
    // CV_Assert(sizeof(int) == 4);
    int *p_y1, *p_y2;
    uchar *p_inputMat, *p_outputMat;
    
    for( int i = 0; i < nRows; ++i)
    {
        y1 = i-s2;
        y2 = i+s2;
        
        if (y1 < 0){
            y1 = 0;
        }
        if (y2 >= nRows) {
            y2 = nRows-1;
        }
        
        p_y1 = sumMat.ptr<int>(y1);
        p_y2 = sumMat.ptr<int>(y2);
        p_inputMat = inputMat.ptr<uchar>(i);
        p_outputMat = outputMat.ptr<uchar>(i);
        
        for ( int j = 0; j < nCols; ++j)
        {
            // set the SxS region
            x1 = j-s2;
            x2 = j+s2;
            
            if (x1 < 0) {
                x1 = 0;
            }
            if (x2 >= nCols) {
                x2 = nCols-1;
            }
            
            count = (x2-x1)*(y2-y1);
            
            // I(x,y)=s(x2,y2)-s(x1,y2)-s(x2,y1)+s(x1,x1)
            sum = p_y2[x2] - p_y1[x2] - p_y2[x1] + p_y1[x1];
            
            if ((int)(p_inputMat[j] * count) < (int)(sum*(1.0-T)))
                p_outputMat[j] = 0;
            else
                p_outputMat[j] = 255;
        }
    }
}

void preprocessing_frame(Mat* frame, Mat* frame_output){
    Mat blur;
    Mat frame_gray;
    Mat frame_thresholding;
    Mat integralImage;
    //namedWindow("New frame", 1 );
    cvtColor( *frame,frame_gray, COLOR_BGR2GRAY );
    GaussianBlur( frame_gray, blur, Size(7,7),0);
    //threshold(src_gray,detected_edges,0,255,THRESH_BINARY+THRESH_OTSU);
    //integral(blur, integralImage);
    //frame_thresholding = Mat::zeros(blur.size(), CV_8UC1);
    blur.copyTo(frame_thresholding);
    thresholdIntegral(blur, frame_thresholding);
    //adaptiveThreshold(frame_thresholding, frame_thresholding, 255, ADAPTIVE_THRESH_GAUSSIAN_C,CV_THRESH_BINARY,125,20);
    
    //imshow("New frame",frame_thresholding);
    (*frame_output) = frame_thresholding;
    // cout<<"preprocesada: "<<frame_thresholding.size<<endl;
}

//void ShowManyImages(string title, int n_fails, int total_frames, float rms, Mat cameraMatrix, int nArgs, ...) {
//    int size;
//    int i;
//    int m, n;
//    int x, y;
//
//    int w, h;
//
//    float scale;
//    int max;
//
//    if(nArgs <= 0) {
//        //printf("Number of arguments too small....\n");
//        return;
//    }
//    else if(nArgs > 14) {
//        //printf("Number of arguments too large, can only handle maximally 12 images at a time ...\n");
//        return;
//    }
//    else if (nArgs == 1) {
//        w = h = 1;
//        size = 400;
//    }
//    else if (nArgs == 2) {
//        w = 2; h = 1;
//        size = 400;
//    }
//    else if (nArgs == 3 || nArgs == 4) {
//        w = 2; h = 2;
//        size = 400;
//    }
//    else if (nArgs == 5 || nArgs == 6) {
//        w = 3; h = 2;
//        size = 200;
//    }
//    else if (nArgs == 7 || nArgs == 8) {
//        w = 4; h = 2;
//        size = 200;
//    }
//    else {
//        w = 4; h = 3;
//        size = 150;
//    }
//
//
//    Mat DispImage = Mat::zeros(Size(350 + size*w, size*h), CV_8UC3);
//
//    va_list args;
//    va_start(args, nArgs);
//
//    for (i = 0, m = 20, n = 120; i < nArgs; i++, m += (20 + size)) {
//        Mat all_img;
//        all_img = va_arg(args, cv::Mat);
//
//        if(all_img.empty()) {
//            printf("Invalid arguments");
//            return;
//        }
//
//        x = all_img.cols;
//        y = all_img.rows;
//
//        max = (x > y)? x: y;
//
//        scale = (float) ( (float) max / size );
//
//        if( i % w == 0 && m!= 20) {
//            m = 20;
//            n+= 20 + size-80;
//        }
//
//
//        Rect ROI(m, n, (int)( x/scale ), (int)( y/scale ));
//        Mat temp; resize(all_img,temp, Size(ROI.width, ROI.height));
//        temp.copyTo(DispImage(ROI));
//    }
//
//    namedWindow( title, 1 );
//    putText(DispImage, "CAMERA CALIBRATION", Point2f(400,50), FONT_HERSHEY_PLAIN, 2,  Scalar(202,109,200), 2 , 8 , false);
//    putText(DispImage, "Original Frame", Point2f(100,90), FONT_HERSHEY_PLAIN, 2,  Scalar(202,109,16), 2 , 8 , false);
//    putText(DispImage, "Preprocessed Frame", Point2f(500,90), FONT_HERSHEY_PLAIN, 2,  Scalar(202,109,16), 2 , 8 , false);
//    putText(DispImage, "Pattern Detected", Point2f(100,450), FONT_HERSHEY_PLAIN, 2,  Scalar(202,109,16), 2 , 8 , false);
//    putText(DispImage, "Order Centers", Point2f(500,450), FONT_HERSHEY_PLAIN, 2,  Scalar(202,109,16), 2 , 8 , false);
//    putText(DispImage, "Time: "+(to_string((float)duration/(float)total_frames)), Point2f(80,790), FONT_HERSHEY_PLAIN, 2,  Scalar(202,109,16), 2 , 8 , false);
//    putText(DispImage, "Fails: "+to_string(n_fails)+" of: "+to_string(total_frames), Point2f(500,790), FONT_HERSHEY_PLAIN, 2,  Scalar(202,109,16), 2 , 8 , false);
//    putText(DispImage, "Calibrate: ", Point2f(870,200), FONT_HERSHEY_PLAIN, 2,  Scalar(202,109,16), 2 , 8 , false);
//
//    if(rms==-1){
//        putText(DispImage, "Rms: "+to_string(0), Point2f(870,300), FONT_HERSHEY_PLAIN, 2,  Scalar(202,109,16), 2 , 8 , false);
//        putText(DispImage, "Fx: "+to_string(0), Point2f(870,350), FONT_HERSHEY_PLAIN, 2,  Scalar(202,109,16), 2 , 8 , false);
//        putText(DispImage, "Fy: "+to_string(0), Point2f(870,400), FONT_HERSHEY_PLAIN, 2,  Scalar(202,109,16), 2 , 8 , false);
//        putText(DispImage, "Cx: "+to_string(0), Point2f(870,450), FONT_HERSHEY_PLAIN, 2,  Scalar(202,109,16), 2 , 8 , false);
//        putText(DispImage, "Cy: "+to_string(0), Point2f(870,500), FONT_HERSHEY_PLAIN, 2,  Scalar(202,109,16), 2 , 8 , false);
//    }
//    else
//    {
//        putText(DispImage, "Rms: "+to_string(rms), Point2f(870,300), FONT_HERSHEY_PLAIN, 2,  Scalar(202,109,16), 2 , 8 , false);
//        putText(DispImage, "Fx: "+to_string(cameraMatrix.at<double>(0,0)), Point2f(870,350), FONT_HERSHEY_PLAIN, 2,  Scalar(202,109,16), 2 , 8 , false);
//        putText(DispImage, "Fy: "+to_string(cameraMatrix.at<double>(0,2)), Point2f(870,400), FONT_HERSHEY_PLAIN, 2,  Scalar(202,109,16), 2 , 8 , false);
//        putText(DispImage, "Cx: "+to_string(cameraMatrix.at<double>(1,1)), Point2f(870,450), FONT_HERSHEY_PLAIN, 2,  Scalar(202,109,16), 2 , 8 , false);
//        putText(DispImage, "Cy: "+to_string(cameraMatrix.at<double>(1,2)), Point2f(870,500), FONT_HERSHEY_PLAIN, 2,  Scalar(202,109,16), 2 , 8 , false);
//    }
//
//    //string a = to_string(countimages);
//    imshow( title, DispImage);
//
//    va_end(args);
//}

float calibrate_camera(Size &imageSize, Mat &cameraMatrix, Mat &distCoeffs, vector<vector<Point2f>>& imagePoints) {
    /*
     * OjectPoints: vector que describe como debe verse el patron
     * imagePoints: vector de vectores con puntos de control(un frame tiene un vector de N puntos de control)
     * imageSize: tamaño de la imagen
     * distCoeffs: coeficientes de distorcion
     * rvecs: vector de rotacion
     * tvecs: vector de tranlacion
     */
    Size boardSize(PATTERN_NUM_COLS, PATTERN_NUM_ROWS);
    float squareSize = 44;
    //    float squareSize = 44.3;
    float aspectRatio = 1;
    vector<Mat> rvecs;
    vector<Mat> tvecs;
    vector<float> reprojErrs;
    vector<vector<Point3f> > objectPoints(1);
    
    distCoeffs = Mat::zeros(8, 1, CV_64F);
    
    objectPoints[0].resize(0);
    for ( int i = 0; i < boardSize.height; i++ ) {
        for ( int j = 0; j < boardSize.width; j++ ) {
            objectPoints[0].push_back(Point3f(  float(j * squareSize),
                                              float(i * squareSize), 0));
        }
    }
    
    objectPoints.resize(imagePoints.size(), objectPoints[0]);
    
    double rms = calibrateCamera(objectPoints,
                                 imagePoints,
                                 imageSize,
                                 cameraMatrix,
                                 distCoeffs,
                                 rvecs,
                                 tvecs,
                                 0/*,
                                   
                                   CV_CALIB_ZERO_TANGENT_DIST*/);
    
    return rms;
}

vector<Point2f> ellipses2Points(vector<P_Ellipse> ellipses){
    vector<Point2f> buffer(REAL_NUM_CTRL_PTS);
    for(int i=0;i<REAL_NUM_CTRL_PTS;i++){
        buffer[i] = ellipses[i].center();
    }
    return buffer;
}

void first_calibration_homework(string video_file){
    //Camera calibration
    float rms=-1;
    int NUM_FRAMES_FOR_CALIBRATION = 45;
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
        points_detected = find_ellipses(&frame_preprocessed, &img_ellipses,control_points,frame_time,&n_fails);
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

/*
 * Iterative Refinement functions
 *
 */
void select_frames_by_time(VideoCapture& cap, vector<Mat>& out_frames_selected, int delay_time,int num_frames_for_calibration){
    //util for save frame
    cout << "Choosing frames by time... \n";
    float frame_time = 0;
    int points_detected = 0;
    int frame_count = 0; //current frame
    int n_fails = 0;
    //found pattern points
    vector<P_Ellipse> control_points;
    
    while (1) {
        Mat frame, rview;
        cap>>frame;
        if (frame.empty()) {
            cout << "Cannot capture frame. \n";
            break;
        }
        //name for save frame
        frame_time = cap.get(CV_CAP_PROP_POS_MSEC)/1000;
        //count frame
        frame_count++;
        
        Mat frame_preprocessed;
        preprocessing_frame(&frame, &frame_preprocessed);
        Mat img_ellipses = frame.clone();
        points_detected = find_ellipses(&frame_preprocessed, &img_ellipses,control_points,frame_time,&n_fails);
        
        if(frame_count% delay_time == 0 && points_detected == REAL_NUM_CTRL_PTS){
            out_frames_selected.push_back(frame);
            if(out_frames_selected.size()==num_frames_for_calibration){
                break;
            }
        }
    }
}


void save_frame(String data_path, string name,Mat& frame){
    string s = data_path +name+".jpg";
    imwrite(s,frame);
}

void create_real_pattern(Size frame_size, vector<Point3f>& out_real_centers){
    cout << "Creating real pattern... \n";
    int h = frame_size.height;
    int w = frame_size.width;
    float desp_w = h*0.5;
    float desp_h = w * 0.5;
    // float squareSize = w / 5.5;
    float squareSize = w / 5.5;

    float margin = 70;


    float radio_ext = 2.9;
    // float squareSize = 2*radio_ext;
    // float desp_w = 4.4-2*radio_ext;

    out_real_centers.push_back(Point3f(  float(margin) ,float( margin), 0));
    out_real_centers.push_back(Point3f(  float(w-margin) ,float( margin), 0));
    out_real_centers.push_back(Point3f(  float(margin) ,float( h-margin), 0));
    out_real_centers.push_back(Point3f(  float(w-margin) ,float( h-margin), 0));
    
    // for ( int i = 1; i < PATTERN_NUM_ROWS; i++ ) {
    //     for ( int j = 1; j < PATTERN_NUM_COLS; j++ ) {
    //         // out_real_centers.push_back(Point3f(  float(j * squareSize + desp_w) ,
    //         //                                    float( w - (i * squareSize + desp_h)), 0));
    //         out_real_centers.push_back(Point3f(  float(initial_point + j*44) ,
    //                                            float(initial_point), 0));
    //     }
    // }
    // cout<<"frame size: "<<frame_size<<endl;
    cout<<"frame size: "<<frame_size<<endl;

    //plot real control points
    Mat real_points_img = Mat::zeros(frame_size, CV_8UC3);
    for(int i=0;i<out_real_centers.size();i++){
        circle(real_points_img, Point2f(out_real_centers[i].x,out_real_centers[i].y), 2, rose, 2);
    }
    // save_frame("/home/david/Escritorio/calib-data/frames/","real pattern2",real_points_img);
}

int find_control_points(Mat& frame,Mat& output,vector<P_Ellipse>& out_control_points){
    
    Mat frame_preprocessed;
    preprocessing_frame(&frame, &frame_preprocessed);
    output = frame.clone();
    //(Mat* img_preprocessing, Mat* img_out, vector<P_Ellipse>& control_points, float frame_time = 0,int* n_fails=0)
    int points_detected = find_ellipses(&frame_preprocessed, &output,out_control_points);
    return points_detected;
}
/*******************************************************************************/

void fronto_parallel_images(vector<Mat>& selected_frames,vector<Mat>& fronto_images,
                        vector<Point3f> real_centers,Size frameSize, 
                        Mat& cameraMatrix, Mat& distCoeffs,vector<P_Ellipse>& out_control_points);

int main()
{
    //    string path_data = "cam2/";
    string path_data = "/home/david/Escritorio/calib-data/";
    //string path_data = "/Users/davidchoqueluqueroman/Desktop/CURSOS-MASTER/IMAGENES/testOpencv/data/";
    string video_file = path_data+"cam2/anillos.avi";
    
    VideoCapture cap;
    cap.open(video_file);
    if ( !cap.isOpened() ){
        cout << "Cannot open the video file. \n";
        return -1;
    }
    //initial frame for get size of frames
    Mat frame;
    cap.read(frame);
    int w = frame.rows;
    int h = frame.cols;
    Size frameSize(h, w);
    
    vector<Mat> selected_frames;
    int delay_time = 50;
    int num_frames_for_calibration = 45;
    select_frames_by_time(cap, selected_frames,delay_time,num_frames_for_calibration);
    
    cout << "Creating ideal image ... "<< endl;
    vector<Point3f> real_centers;
    create_real_pattern(frameSize, real_centers);
    
    //first calibration
    vector<vector<Point2f>> imagePoints;
    Mat cameraMatrix;
    Mat distCoeffs = Mat::zeros(8, 1, CV_64F);
    vector<P_Ellipse> control_points;
    int num_control_points=0;
    double rms=-1;
    Mat output_img_control_points;
    
    for (int i=0; i<selected_frames.size(); i++) {
        control_points.clear();
        num_control_points = find_control_points(selected_frames[i], output_img_control_points,control_points);
        // imshow("img"+to_string(i), output_img_control_points);
        if(num_control_points == REAL_NUM_CTRL_PTS){
            vector<Point2f> buffer = ellipses2Points(control_points) ;
            imagePoints.push_back(buffer);
        }
    }
    if(imagePoints.size()==selected_frames.size()){
        cout << "First calibration... \n";
        rms = calibrate_camera(frameSize, cameraMatrix, distCoeffs, imagePoints);
        cout << "cameraMatrix " << cameraMatrix << endl;
        cout << "distCoeffs " << distCoeffs << endl;
        cout << "rms: " << rms << endl;
    }
    
    //undistort first 
    Mat map1, map2;
    initUndistortRectifyMap(cameraMatrix, distCoeffs, Mat(),getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, 
        frameSize, 1, frameSize, 0),frameSize, CV_16SC2, map1, map2);

    Mat undistorted_image = frame.clone();
    remap(output_img_control_points, undistorted_image, map1, map2, INTER_LINEAR);
    imshow("undisorted image", undistorted_image);
    
    vector<Mat> fronto_images;
    int No_ITER = 1;
    for(int i=0; i<No_ITER;i++){
        fronto_images.clear();
        fronto_parallel_images(selected_frames,fronto_images,real_centers, frameSize, cameraMatrix, distCoeffs,control_points);
        // selected_frames = fronto_images;
        cout<<"selected_frames size: "<<selected_frames.size()<<endl;
        // for (int i=0; i<selected_frames.size(); i++) {
            control_points.clear();
            // num_control_points = find_control_points(fronto_images[0], output_img_control_points,control_points);
            imshow("cbgsdfbh",fronto_images[0] );
        //     if(num_control_points == REAL_NUM_CTRL_PTS){
        //         vector<Point2f> buffer = ellipses2Points(control_points) ;
        //         imagePoints.push_back(buffer);
        //     }
        // }
        // if(imagePoints.size()==selected_frames.size()){
        //     cameraMatrix.release();
        //     distCoeffs.release();
        //     cout << "Refinement... \n";
        //     rms = calibrate_camera(frameSize, cameraMatrix, distCoeffs, imagePoints);
        //     cout << "cameraMatrix " << cameraMatrix << endl;
        //     cout << "distCoeffs " << distCoeffs << endl;
        //     cout << "rms: " << rms << endl;
        // }
    }
    
    
    
    return 0;
}

void fronto_parallel_images(vector<Mat>& selected_frames,vector<Mat>& fronto_images,
                        vector<Point3f> real_centers,Size frameSize, 
                        Mat& cameraMatrix, Mat& distCoeffs,vector<P_Ellipse>& control_points){
   
    vector<vector<Point2f>> imagePoints;
    // Mat cameraMatrix;
    // Mat distCoeffs = Mat::zeros(8, 1, CV_64F);
    // vector<P_Ellipse> control_points;
    int num_control_points=0;
    double rms=-1;
    Mat output_img_control_points;
    Mat frame;
    Mat img_fronto_parallel;
    string path_data = "/home/david/Escritorio/calib-data/frames/";

    for (int i=0; i<selected_frames.size(); i++) {
        // control_points.clear();
        imagePoints.clear();

        frame = selected_frames[i].clone();
        
        
        // num_control_points = find_control_points(frame, output_img_control_points,control_points);
        // // imshow("img"+to_string(i), output_img_control_points);
        // if(num_control_points == REAL_NUM_CTRL_PTS){
        //     vector<Point2f> buffer = ellipses2Points(control_points) ;
        //     imagePoints.push_back(buffer);
        // }

        //undistort
        cout << "Undistort image ... "<< endl;
        Mat map1, map2;
        initUndistortRectifyMap(cameraMatrix, distCoeffs, Mat(),getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, 
        frameSize, 1, frameSize, 0),frameSize, CV_16SC2, map1, map2);

        Mat undistorted_image = frame.clone();
        remap(frame, undistorted_image, map1, map2, INTER_LINEAR);

        // save_frame(path_data,"raw",output_img_control_points);

        // //unproject
        cout << "Unproject image ... "<< endl;
        // vector<Point2f> control_points_2d = ellipses2Points(control_points);
         
        vector<Point2f> control_points_2d;
        control_points_2d.push_back(control_points[15].center());
        control_points_2d.push_back(control_points[19].center());
        control_points_2d.push_back(control_points[0].center());
        control_points_2d.push_back(control_points[4].center());

        Mat homography = findHomography(control_points_2d,real_centers);
        Mat inv_homography = findHomography(real_centers,control_points_2d);

        img_fronto_parallel = frame.clone();
        warpPerspective(undistorted_image, img_fronto_parallel, homography, frameSize);
        fronto_images.push_back(img_fronto_parallel);
        // imshow("fronto paralell",img_fronto_parallel);

        // save_frame(path_data,"fronto"+to_string(i),img_fronto_parallel);
        // break;


    }

    

}

