#include "ellipse.h"

P_Ellipse::P_Ellipse()
{

}

P_Ellipse::P_Ellipse(float cx,float cy, float r):x(cx),y(cy),radio(r){}

float P_Ellipse::distance(P_Ellipse e){
    return sqrt(pow(x - e.x, 2) + pow(y - e.y, 2));
}

Point2f P_Ellipse::center(){
    return Point2f(x,y);
}
