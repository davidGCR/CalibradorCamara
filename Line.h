//
//  Line.h
//  testOpencv
//
//  Created by David Choqueluque Roman on 1/3/19.
//  Copyright © 2019 David Choqueluque Roman. All rights reserved.
//

#ifndef Line_h
#define Line_h

#include <cmath>
#include <iostream>
#include <vector>
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "ellipse.h"
using namespace std;
using namespace cv;
class Line
{
private:
    int max_size;
    vector<P_Ellipse> recta;
public:
    Line();
    Line(int max_size);
    Line(vector<P_Ellipse>& ln);
    
    void add_ellipse(P_Ellipse& e);
    P_Ellipse getLeftPoint();
    P_Ellipse getRightPoint();
    int size();
    void clear();
    bool isFull();
    P_Ellipse getEllipse(int index);
};
#endif /* Line_h */
