#ifndef Quadrant_h
#define Quadrant_h
#include <cmath>
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
using namespace std;
using namespace cv;

class Quadrant{
private:
    
public:
    float x1;
    float y1;
    float h;
    float w;
    float c_radio;
    Point2f c_center;
    int frames_count;
    
    Quadrant();
    Quadrant(float x1,float y1,float w, float h, int radio);
    bool isInclude(Point2f p);
    void build_circle();
    
};

#endif

Quadrant::Quadrant(){}

Quadrant::Quadrant(float x1,float y1,float w, float h,int radio):x1(x1),y1(y1),w(w),h(h),c_radio(radio){
    c_center = Point2f((x1+x1+w)/2,(y1+y1+w)/2);
    frames_count=0;
}

bool Quadrant::isInclude(Point2f p){
    return (p.x > x1 && p.y > y1 && p.x < (x1+w) && p.y < (y1+h));
}


