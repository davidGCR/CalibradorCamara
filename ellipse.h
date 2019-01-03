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

#endif // ELLIPSE_H

