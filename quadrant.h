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

    Quadrant();
    Quadrant(float x1,float y1,float w, float h);
    bool isInclude(Point2f p);
    Point2f qcenter();
    
};

#endif 

Quadrant::Quadrant(){}

Quadrant::Quadrant(float x1,float y1,float w, float h):x1(x1),y1(y1),w(w),h(h){}

bool Quadrant::isInclude(Point2f p){
    return (p.x > x1 && p.y > y1 && p.x < (x1+w) && p.y < (y1+h));
}

Point2f Quadrant::qcenter(){
    Point2f center((x1+x1+w)/2,(y1+y1+w)/2);
    return center;
}