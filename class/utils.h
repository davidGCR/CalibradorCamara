#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include "constants.h"

using namespace std;



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
