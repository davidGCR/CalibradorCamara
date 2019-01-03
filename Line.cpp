//
//  Line.cpp
//  testOpencv
//
//  Created by David Choqueluque Roman on 1/3/19.
//  Copyright © 2019 David Choqueluque Roman. All rights reserved.
//

#include "Line.h"
Line::Line(){}
Line::Line(vector<P_Ellipse>& ln):recta(ln){
    
}
Line::Line(int max_size){
    recta = vector<P_Ellipse>(max_size);
}
void Line::add_ellipse(P_Ellipse& e){
    recta.push_back(e);
}
P_Ellipse Line::getLeftPoint(){
    return recta[0];
}
P_Ellipse Line::getRightPoint(){
    return recta[recta.size()-1];
}
int Line::size(){
    return (int)recta.size();
}
void Line::clear(){
    recta.clear();
}
bool Line::isFull(){
    return (recta.size()==max_size);
}
P_Ellipse Line::getEllipse(int index){
    return recta[index];
}
