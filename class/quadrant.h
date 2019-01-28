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
    int capacity;
    int frame_counter;
    
    Quadrant();
    Quadrant(float x1,float y1,float w, float h, int radio, int capacity);
    bool isInclude(Point2f p);
    void setRadio(float newradio);
    
};

#endif

Quadrant::Quadrant(){}

Quadrant::Quadrant(float x1,float y1,float w, float h,int radio, int capacity):x1(x1),y1(y1),w(w),h(h),c_radio(radio),capacity(capacity){
    c_center = Point2f((x1+x1+w)/2,(y1+y1+w)/2);
    frame_counter = 0;
}

bool Quadrant::isInclude(Point2f p){
    float distance = sqrt(pow((p.x-c_center.x),2)+pow((p.y-c_center.y),2));
    // return (p.x > x1 && p.y > y1 && p.x < (x1+w) && p.y < (y1+h));
    return (distance<c_radio);
}
void Quadrant::setRadio(float newradio){
    c_radio = newradio;
}

