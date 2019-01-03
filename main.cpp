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
#include <chrono>
#include <stdio.h>
#include <stdarg.h>
#include <string>
#include <time.h>
#include <cmath>

#include "ellipse.h"
#include "Line.h"
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
const float TRACK_THRESHOLD = 10;

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

void track_points(Mat& frame,vector<P_Ellipse>& p_ellipses,
                  vector<P_Ellipse>& pp_control_points, float frame_time );

void find_ellipses(Mat* img_preprocessing, Mat* img_out, int* n_fails, vector<P_Ellipse>& control_points, float frame_time){
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
    
    
    int count = 0;
    float distance = 0;
    for(int i=0;i<p_ellipses.size();i++){ //filtrar ellipses por distancias
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
            pp_control_points.push_back(p_ellipses[i]);
            
            //cout<<"raius: "<<pp_control_points[i].radio<<endl;
            //            cv::circle((*img_out), p_ellipses[i].center(), p_ellipses[i].radio, green,2);
        }
    }
    
    if (pp_control_points.size()==REAL_NUM_CTRL_PTS) {
        //cout<<"# entrooooooo: "<<endl;
        for(int j=0; j<pp_control_points.size(); j++){
            cv::ellipse((*img_out),pp_control_points[j].fit_ellipse,white,2);
        }
        track_points((*img_out),pp_control_points,control_points,frame_time);
    }
    else{
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
    for (int i=ROWS_CTRL_PTS; i<control_points.size(); i+=ROWS_CTRL_PTS) {
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
        for (int j=0; j<ROWS_CTRL_PTS; j++) {
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
                
                if (conincidencias == ROWS_CTRL_PTS) { // se contraron la cantidad de patrones en una fila del patron
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
                        cout<<"recta vertical!!!!!!!!!!!!!! x1: "<<x1<<", x2: "<<x2<<endl;
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
                            //                                if(control_points.size() - 1 == 19){
                            //                                    //a medida que se va agregando se etiqueta
                            //                                    label = (int)(control_points.size() - 1);
                            //                                    putText(frame, to_string(label), ellipses_in_line[lbl].center(), FONT_HERSHEY_COMPLEX_SMALL, 1, white, 2);
                            //                                }
                        }
                        
                        //agregar puntos limites de la recta
                        limit_points.push_back(ellipse_rect_left);
                        limit_points.push_back(ellipse_rect_right);
                    }
                    string s = "/Users/davidchoqueluqueroman/Desktop/CURSOS-MASTER/IMAGENES/testOpencv/data/captures/cap-"+to_string(frame_time)+".jpg";
                    imwrite(s,frame);
                    
                }
            }
        }
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
void track_points(Mat& frame, vector<P_Ellipse>& pp_control_points, vector<P_Ellipse>& control_points, float frame_time)
{
    //    if (p_ellipses_f1.size() < REAL_NUM_CTRL_PTS && control_points.size() < REAL_NUM_CTRL_PTS) {
    //        return;
    //    }
    
    int label=-1;
    Scalar color_text = red;
    
    if (control_points.size() == 0)
    {
        initialize_track_points(frame, pp_control_points, control_points, frame_time);
        
        //cout << "Pattern Traking fase 11111" << endl;
        
        //        if (rows != COL_CTRL_PTS) {
        //            cout << "==================== clear control points: " << rows << endl;
        //            control_points.clear();
        //        }
    }
    else{
        
        cout << "ESLE=================== pp: "<<pp_control_points.size()<<" , cp: "<<control_points.size()<<endl;
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
            cout << "= ************* dist min: "<<dist_min<<", dist: "<<dist<<endl;
            if(dist_min > TRACK_THRESHOLD){
                cout << "======lejosssssssssssssssssssssssssss======"<<endl;
                dist_min = -1;
                //                string s = "/Users/davidchoqueluqueroman/Desktop/CURSOS-MASTER/IMAGENES/testOpencv/data/captures/cap-"+to_string(frame_time)+".jpg";
                //                imwrite(s,frame);
                break;
            }
            
            cout<<"labeleddddddd: "<<i<<endl;
            label = i;
            putText(frame, to_string(label), pp_control_points[near_point].center(), FONT_HERSHEY_COMPLEX_SMALL, 1, color_text, 2);
        }
        //        string s = "/Users/davidchoqueluqueroman/Desktop/CURSOS-MASTER/IMAGENES/testOpencv/data/captures/cap-"+to_string(frame_time)+".jpg";
        //        imwrite(s,frame);
        if (dist_min == -1) {
            cout<<"======================== CLEAR ==============================: "<<endl;
            control_points.clear();
            initialize_track_points(frame, pp_control_points, control_points, frame_time);
            return;
        }
        
        
    }
    
    if (control_points.size() == REAL_NUM_CTRL_PTS) {
        
    }
    
}

void preprocessing_frame(Mat* frame, Mat* frame_output){
    Mat blur;
    Mat frame_gray;
    Mat frame_thresholding;
    //namedWindow("New frame", 1 );
    cvtColor( *frame,frame_gray, COLOR_BGR2GRAY );
    GaussianBlur( frame_gray, blur, Size(5,5),0 );
    //threshold(src_gray,detected_edges,0,255,THRESH_BINARY+THRESH_OTSU);
    adaptiveThreshold(frame_gray, frame_thresholding, 255, ADAPTIVE_THRESH_GAUSSIAN_C,CV_THRESH_BINARY,125,20);
    //imshow("New frame",frame_thresholding);
    (*frame_output) = frame_thresholding;
    // cout<<"preprocesada: "<<frame_thresholding.size<<endl;
}

void ShowManyImages(string title, int n_fails, int total_frames, int nArgs, ...) {
    int size;
    int i;
    int m, n;
    int x, y;
    
    int w, h;
    
    float scale;
    int max;
    
    if(nArgs <= 0) {
        printf("Number of arguments too small....\n");
        return;
    }
    else if(nArgs > 14) {
        printf("Number of arguments too large, can only handle maximally 12 images at a time ...\n");
        return;
    }
    else if (nArgs == 1) {
        w = h = 1;
        size = 400;
    }
    else if (nArgs == 2) {
        w = 2; h = 1;
        size = 400;
    }
    else if (nArgs == 3 || nArgs == 4) {
        w = 2; h = 2;
        size = 400;
    }
    else if (nArgs == 5 || nArgs == 6) {
        w = 3; h = 2;
        size = 200;
    }
    else if (nArgs == 7 || nArgs == 8) {
        w = 4; h = 2;
        size = 200;
    }
    else {
        w = 4; h = 3;
        size = 150;
    }
    
    
    Mat DispImage = Mat::zeros(Size(100 + size*w, size*h), CV_8UC3);
    
    va_list args;
    va_start(args, nArgs);
    
    for (i = 0, m = 20, n = 120; i < nArgs; i++, m += (20 + size)) {
        Mat img;
        //Mat img = va_arg(args, cv::Mat);
        
        if(img.empty()) {
            printf("Invalid arguments");
            return;
        }
        
        x = img.cols;
        y = img.rows;
        
        max = (x > y)? x: y;
        
        scale = (float) ( (float) max / size );
        
        if( i % w == 0 && m!= 20) {
            m = 20;
            n+= 20 + size-80;
        }
        
        
        Rect ROI(m, n, (int)( x/scale ), (int)( y/scale ));
        Mat temp; resize(img,temp, Size(ROI.width, ROI.height));
        temp.copyTo(DispImage(ROI));
    }
    
    namedWindow( title, 1 );
    putText(DispImage, "CAMERA CALIBRATION", Point2f(250,50), FONT_HERSHEY_PLAIN, 2,  Scalar(202,109,200), 2 , 8 , false);
    putText(DispImage, "Original Frame", Point2f(100,90), FONT_HERSHEY_PLAIN, 2,  Scalar(202,109,16), 2 , 8 , false);
    putText(DispImage, "Preprocessed Frame", Point2f(500,90), FONT_HERSHEY_PLAIN, 2,  Scalar(202,109,16), 2 , 8 , false);
    putText(DispImage, "Pattern Detected", Point2f(100,450), FONT_HERSHEY_PLAIN, 2,  Scalar(202,109,16), 2 , 8 , false);
    putText(DispImage, "Order Centers", Point2f(500,450), FONT_HERSHEY_PLAIN, 2,  Scalar(202,109,16), 2 , 8 , false);
    putText(DispImage, "Time: "+(to_string((float)duration/(float)total_frames)), Point2f(80,790), FONT_HERSHEY_PLAIN, 2,  Scalar(202,109,16), 2 , 8 , false);
    putText(DispImage, "Fails: "+to_string(n_fails)+" of: "+to_string(total_frames), Point2f(500,790), FONT_HERSHEY_PLAIN, 2,  Scalar(202,109,16), 2 , 8 , false);
    
    //string a = to_string(countimages);
    imshow( title, DispImage);
    
    va_end(args);
}

int main()
{
    string path_data = "/Users/davidchoqueluqueroman/Desktop/CURSOS-MASTER/IMAGENES/testOpencv/data/";
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
    cap.open(path_data+"padron2.mp4");
    if ( !cap.isOpened() ){
        cout << "Cannot open the video file. \n";
        return -1;
    }
    namedWindow("resultado",CV_WINDOW_AUTOSIZE);
    int total_frames=0;
    int n_fails = 0;
    float frame_time = 0;
    while (1) {
        
        Mat frame;
        cap>>frame;
        if (frame.empty()) {
            cout << "Cannot capture frame. \n";
            break;
        }
        total_frames++;
        
        frame_time = cap.get(CV_CAP_PROP_POS_MSEC)/1000;
        //time_t a = frame_time;
        cout<<"time: "<<frame_time<<endl;
        
        Mat frame_preprocessed;
        auto t11 = std::chrono::high_resolution_clock::now();
        preprocessing_frame(&frame, &frame_preprocessed);
        //    imshow("Pre-procesada",img_preprocessed);
        Mat img_ellipses = frame.clone();
        find_ellipses(&frame_preprocessed, &img_ellipses,&n_fails,control_points,frame_time);
        auto t12 = std::chrono::high_resolution_clock::now();
        duration += std::chrono::duration_cast<std::chrono::milliseconds>(t12 - t11).count();
        imshow("final",img_ellipses);
        
        //cvtColor(frame_preprocessed,frame_preprocessed, COLOR_GRAY2RGB);
        //imshow("other",frame_preprocessed);
        //ShowManyImages("resultado", n_fails, total_frames, 4, frame, frame_preprocessed, img_circles, img_ellipses);
        
        if(waitKey(5) == 27)
        {
            break;
        }
    }
    
    cout<<"# fails: "<<n_fails<<" of: "<<total_frames<<endl;
    cout<<"# frames: "<<total_frames<<" time: "<<duration<<endl;
    return 0;
}

