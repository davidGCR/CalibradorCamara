#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include "constants.h"


#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include <iostream>
using namespace cv;
using namespace std;

void save_refinements_parameters_add_row(ofstream& myfile,int iter, CvMat* K,CvMat* Rt, double rmss){
//    for(int i = 0; i < P->rows; i++)
//    {
//        myfile <<cvmGet(P,i,0)<<",";
//    }
    myfile <<iter<<","<<cvmGet(K,0,0)<<","<<cvmGet(K,1,1)<<","<<cvmGet(K,0,2)<<","<<cvmGet(K,1,2)<<","<<cvmGet(K,0,1)<<","
    <<cvmGet(Rt,0,0)<<","<<cvmGet(Rt,0,1)<<","<<cvmGet(Rt,0,2)<<","
    <<cvmGet(Rt,1,0)<<","<<cvmGet(Rt,1,1)<<","<<cvmGet(Rt,1,2)<<","
    <<cvmGet(Rt,2,0)<<","<<cvmGet(Rt,2,1)<<","<<cvmGet(Rt,2,2)<<","
    <<cvmGet(Rt,0,3)<<","<<cvmGet(Rt,1,3)<<","<<cvmGet(Rt,2,3)<<","<<rmss<<"\n";
}

void set_headers(ofstream& myfile){
    myfile << "it, fx, fy, cx, cy, sk, r00,r01,r02,r10,r11,r12,r20,r21,r22, t1, t2, t3, error\n";
}
void set_headers_refinement(ofstream& myfile){
    myfile << "it, fx, fy, cx, cy, RMS\n";
}
void set_headers_refinement(ofstream& myfile, string more){
    myfile << "it, fx, fy, cx, cy, RMS,"<<more<<"\n";
}

void save_camera_parameters(ofstream& myfile, int it, Mat cameraMatrix, double rms){
    myfile <<it<<","<<cameraMatrix.at<double>(0,0)<<","<<cameraMatrix.at<double>(1,1)<<","<<cameraMatrix.at<double>(0,2)<<","<<cameraMatrix.at<double>(1,2)<<","<<rms<<"\n";
}

void save_camera_parameters(ofstream& myfile, int it, CvMat* K, double rms, double improv){
    myfile <<it<<","<<cvmGet(K,0,0)<<","<<cvmGet(K,1,1)<<","<<cvmGet(K,0,2)<<","<<cvmGet(K,1,2)<<","<<rms<<","<<improv<<"\n";
}

void save_rmss(int no_frames_selected,vector<float> rmss){
    // vector<float> rmss(5);
    // rmss[0] = 12.1;
    // rmss[1] = 2.1;
    // rmss[2] = 0.21;
    // rmss[3] = 0.1323;
    // rmss[4] = 0.123424154;

    std::ofstream myfile;
    myfile.open (PATH_DATA+"results.csv");
    // myfile << "This is the first cell in the first column.\n";
    // myfile << "a,b,c,\n";
    // myfile << "c,s,v,\n";
    // myfile << "1,2,3.456\n";
    // myfile << "semi;colon";
    myfile << "N° frames: "<<no_frames_selected<<"\n";
    myfile << "iteration,RMS\n";

    for(int i = 0; i < rmss.size(); i++)
    {
        myfile <<i<<"," <<rmss[i]<<"\n";
    }
    
    myfile.close();
}

void print_cvMatrix(CvMat* mat, string m_name){
    cout<<"================== "<<m_name<<" ================== \n["<<endl;
    int  width = mat->cols;
    int height = mat->rows;
    cout<<"rows: "<<height<<", cols: "<<width<<endl;
    for(int i=0;i<height;i++){
        for (int j=0; j<width; j++) {
            cout<<cvmGet(mat, i, j)<<", ";
        }
        cout<<endl;
    }
    cout<<"]\n =============================================="<<endl;
}
