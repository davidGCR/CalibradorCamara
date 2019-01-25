#ifndef ELLIPSE_H
#define ELLIPSE_H
#include <cmath>
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"

#include "ellipse.h"
using namespace cv;
class P_Ellipse
{
private:
    
    
    
public:
    P_Ellipse();
    P_Ellipse(float,float,float);
    
    int id_father;
    int id_son;
    float x;
    float y;
    float distance(P_Ellipse);
    float radio;
    Point2f center();
    RotatedRect fit_ellipse;
    
    
    
};


P_Ellipse::P_Ellipse()
{
    
}

P_Ellipse::P_Ellipse(float cx,float cy, float r):x(cx),y(cy),radio(r){}

float P_Ellipse::distance(P_Ellipse e){
    return sqrt(pow(x - e.x, 2) + pow(y - e.y, 2));
}

cv::Point2f P_Ellipse::center(){
    return Point2f(x,y);
}

#endif // ELLIPSE_H

// #include "ellipse.h"



