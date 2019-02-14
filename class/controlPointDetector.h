#include <iostream>
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"

#include "ellipse.h"
#include "Line.h"
#include "constants.h"

using namespace std;
using namespace cv;

int track_points(Mat& frame,vector<P_Ellipse>& p_ellipses,
                 vector<P_Ellipse>& pp_control_points, float frame_time );

int find_ellipses(Mat* img_preprocessing, Mat* img_out, vector<P_Ellipse>& control_points, float frame_time,int& n_fails){
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;
    vector<P_Ellipse> p_ellipses;
    vector<P_Ellipse> pp_control_points;
    
    float radio, radio_son;
    
    findContours( (*img_preprocessing), contours, hierarchy, CV_RETR_TREE, CHAIN_APPROX_SIMPLE, Point(0, 0) );
    
    /// Draw contours
//    for( int i = 0; i< contours.size(); i++ )
//    {
//        drawContours( (*img_out), contours, i, yellow, 2, 8, hierarchy, 0, Point() );
//    }
//
    
    // cout<<"***** Preprocesing: find ellipses..."<<endl;
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

                        cv::ellipse((*img_out),fit_ellipse,celeste,1);
                        cv::ellipse((*img_out),fit_ellipse_son,red,1);
                        
                        // cv::ellipse((*img_out),fit_ellipse_son,yellow,2);
                        //                    cout<<"->"<<ct<<endl;
                    }
                    
                }
            }
        }
//        else{
//            drawContours( (*img_out), contours, ct, rose, 2, 8, hierarchy, 0, Point() );
//        }
    }
    
    // img_circles = img_out->clone();
    
    // cout<<"***** Preprocesing: Remove false positive..."<<endl;
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
            // if(p_ellipses.size()>0){
            p_ellipses.erase(p_ellipses.begin()+i--);
            // }
        }
    }
    int n_points = 0;
    if (pp_control_points.size()==REAL_NUM_CTRL_PTS) {
        //cout<<"# entro: "<< *n_fails<<endl;
        for(int j=0; j<pp_control_points.size(); j++){
            cv::ellipse((*img_out),pp_control_points[j].fit_ellipse,white,2);
        }
        // cout<<"***** Preprocesing: Tracking mode..."<<endl;
        n_points = track_points((*img_out),pp_control_points,control_points,frame_time);
        //cout << "->"<<n_points << endl;
    }
    else{
        // cout<<"###################### pp_control_points not found !!!! "<<endl;
        n_fails++;
    }
    return n_points;
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
// bool sort_lines_by_x(Line p1, Line p2) {
//     return p1.getLeftPoint().x < p2.getLeftPoint().x;
// }

// bool verify_vertical_lines_order(vector<P_Ellipse> control_points){
//     cout<<"************************* orednaaaaaaaaa *****************************"<<endl;
//     float x_left = control_points[0].x;
//     for (int i=PATTERN_NUM_COLS; i<control_points.size(); i+=PATTERN_NUM_COLS) {
//         if(x_left>control_points[i].x){
//             return  true;
//         }
//         else{
//             x_left = control_points[i].x;
//         }
//     }
//     return false;
// }
// void reorder_vertical_lines(Mat& frame,vector<Line>& lines,vector<P_Ellipse>& control_points){
//     Line line;
//     for (int i=0; i<control_points.size(); i++) {
//         if(!line.isFull()){
//             line.add_ellipse(control_points[i]);
//         }
//         else{
//             lines.push_back(line);
//             line.clear();
//         }
//     }
//     sort(lines.begin(), lines.end(), sort_lines_by_x);
//     control_points.clear();
//     for (int i=0; i<lines.size(); i++) {
//         for (int j=0; j<PATTERN_NUM_COLS; j++) {
//             control_points.push_back(lines[i].getEllipse(j));
//             putText(frame, to_string(control_points.size()-1), control_points[i].center(), FONT_HERSHEY_COMPLEX_SMALL, 1, yellow, 2);
//         }
//     }
// }

void initialize_track_points(Mat& frame, vector<P_Ellipse>& pp_control_points, vector<P_Ellipse>& control_points, float frame_time){
    int conincidencias = 0;
    int size_centers = (int)control_points.size();
    float pattern_range = 2; //threshold de distancia punto a recta
    float min_distance;
    vector<P_Ellipse> ellipses_in_line; //ellipses de una linea/recta
    vector<P_Ellipse> limit_points;//puntos extremos de una linea/recta
    // vector<Line> lines;
    // Line line;
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
                // line.clear();
                
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