
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"

#include "constants.h"


//#include <cv.h>
#include <iostream>
using namespace cv;
using namespace std;



void fit_line(vector<Point2f> horizontal_line, vector<Point2f> vertical_line, Point2f& p_intersection){
    //(vx, vy, x0, y0), where (vx, vy) is a normalized vector collinear to the line and (x0, y0) is a point on the line. 
    Vec4f params_line_horizontal;
    Vec4f params_line_vertical;
    fitLine(horizontal_line,params_line_horizontal,CV_DIST_L2,0,0.01, 0.01);
    fitLine(vertical_line,params_line_vertical,CV_DIST_L2,0,0.01, 0.01);
    // cout<<"results fit line size: "<<results<<endl;
    //horizontal line:  [x0 y0] + lb[vx vy ]
    //vertical line:  [x0 y0] + u[vx vy ]
    // the matrix of coefficients
    float vx_h,vy_h,vx_v,vy_v, b1,b2;
    vx_v = -params_line_vertical[0];
    vy_v = -params_line_vertical[1];;
    vx_h = params_line_horizontal[0];
    vy_h = params_line_horizontal[1]; 
    b1 = params_line_vertical[2]-params_line_horizontal[2];
    b2 = params_line_vertical[3]-params_line_horizontal[3];;

    cv::Mat A = (cv::Mat_<float>(2,2) <<
                    vx_v, vx_h,
                    vy_v, vy_h);
    
    //the vector of constants
    cv::Mat B = (cv::Mat_<float>(2,1) <<
                    b1, 
                    b2);
    
    //the vector of variables (results)
    cv::Mat x; //[u, lb]
    cv::solve(A, B, x);
    // printout the result
    float lb = x.at<float>(0,1);
    float u = x.at<float>(0,0);
    // cout << "Result: " << x << "shape: "<<x.size()<<", u: "<<u<<", lb: "<<lb<<endl;
    p_intersection.x = lb*vx_h+params_line_horizontal[2];
    p_intersection.y = lb*vy_h+params_line_horizontal[3];
 
}
/*
** 15 ** 16 ** 17 ** 18 ** 19
** 10 ** 11 ** 12 ** 13 ** 14
** 5  ** 6  ** 7  ** 8  ** 9
** 0  ** 1  ** 2  ** 3  ** 4

** 0 - 1 - 2 - 3 - 4 - 5 - 6 - 7 - 8 - 9 - 10 - 11 - 12 - 13 - 14 - 15 - 16 - 17 - 18 - 19

** idx_array = 5*f + c
** (x,y) p1 = 5*x, p2 = 5*x + 4 
*/
void calculate_intersection(Mat& image, vector<Point2f>& centers,vector<Point2f>& intersection_points){
    int idx_array = -1;
    int h_p1, h_p2, v_p1, v_p2;
    int h_offset = 1;
    int v_offset = PATTERN_NUM_COLS;
    vector<Point2f> horizontal_line;
    vector<Point2f> vertical_line;
    
    int count=0;
    for(int x = 0; x < PATTERN_NUM_ROWS; x++){
        for(int y=0;y<PATTERN_NUM_COLS;y++)
        {
            horizontal_line.clear();
            vertical_line.clear();
            idx_array = 5*x + y;
            h_p1 = 5*x;
            h_p2 = 5*x + 4;
            
            for(int i = h_p1; i <= h_p2; i+=h_offset)
            {
                horizontal_line.push_back(centers[i]);
            }
            // cout<<"horizontal idx: "<<idx_array<<", p1:"<<h_p1<<", p2:"<<h_p2<<endl;
            
            v_p1 = y;
            v_p2 = idx_array+5*(3-x);
            for(int i = v_p1; i <= v_p2; i+=v_offset)
            {
                vertical_line.push_back(centers[i]);
            }
            //plot lines
            cv::line(image,horizontal_line[0],horizontal_line[horizontal_line.size()-1],red,1);
            cv::line(image,vertical_line[0],vertical_line[vertical_line.size()-1],white,1);

            Point2f intersection;
            fit_line(horizontal_line,vertical_line,intersection);
            intersection_points.push_back(intersection);
            cv::circle(image,intersection,3,yellow);
            cv::putText(image,to_string(count),intersection,1,1,yellow);
            count++;
            // cout<<"vertical idx: "<<idx_array<<", v1:"<<v_p1<<", v2:"<<v_p2<<endl;
        }
        // cout<<"======================="<<endl;
    }
    // cout<<"fitting line: "<<intersection<<endl;
}

void baricenter(vector<Point2f> set_points1, vector<Point2f> set_points2, vector<Point2f> set_points3,vector<Point2f>& baricenters){
    for(int i = 0; i < set_points1.size(); i++)
    {
        baricenters[i].x = (set_points1[i].x + set_points2[i].x + set_points3[i].x)/3;
        baricenters[i].y = (set_points1[i].y + set_points2[i].y + set_points3[i].y)/3;
        
    }
}
