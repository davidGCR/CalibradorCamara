#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include <iostream>
using namespace cv;
using namespace std;

/// Global variables
Mat src, src_gray;
Mat dst, detected_edges;

int lowThreshold = 10;
const int max_lowThreshold = 100;
const int ratio = 3;
const int kernel_size = 3;
const char* window_name = "Edge Map";

static void CannyThreshold(int, void*)
{
    //![reduce_noise]
    /// Reduce noise with a kernel 3x3
    blur( src_gray, detected_edges, Size(3,3) );
    //![reduce_noise]

    //![canny]
    /// Canny detector
    Canny( detected_edges, detected_edges, lowThreshold, lowThreshold*ratio, kernel_size );
    //![canny]

    /// Using Canny's output as a mask, we display our result
    //![fill]
    dst = Scalar::all(0);
    //![fill]

    //![copyto]
    src.copyTo( dst, detected_edges);
    //![copyto]

    //![display]
    imshow( window_name, dst );
    //![display]
}


/**
 * @function main
 */
int main( int argc, char** argv )
{

  src = imread("data/patron2.png");
        if( src.empty() )
        {
            std::cout << "Could not open or find the image!\n" << std::endl;
        }
        //![create_mat]
        /// Create a matrix of the same type and size as src (for dst)
        dst.create( src.size(), src.type() );
        //![create_mat]

        //![convert_to_gray]
        Mat blur;
        cvtColor( src, src_gray, COLOR_BGR2GRAY );
        GaussianBlur( src_gray, blur, Size(5,5),0 );

        namedWindow( window_name, WINDOW_AUTOSIZE );
        imshow("preprocess image",src_gray);

//        Mat canny_output;
//        vector<vector<Point> > contours;
//        Canny( src_gray, canny_output, thresh, thresh*2, 3 );
////        findContours( canny_output, contours, RETR_TREE, CHAIN_APPROX_SIMPLE );
//        namedWindow( window_name, WINDOW_AUTOSIZE );
//        imshow("canny",canny_output);
//        cout<<"canny contours size : "<<contours.size()<<endl;

        threshold(src_gray,detected_edges,0,255,THRESH_BINARY+THRESH_OTSU);
        vector<vector<Point> > contours2;
        vector<Vec4i> hierarchy;
        findContours( detected_edges, contours2, hierarchy, RETR_LIST, CHAIN_APPROX_SIMPLE, Point(0, 0) );
        cout<<"numero de contornos2: "<<contours2.size()<<endl;
        namedWindow( window_name, WINDOW_AUTOSIZE );
        imshow("Threshold",detected_edges);

        Mat drawing = Mat::zeros( detected_edges.size(), CV_8UC3 );
        RNG rng(12345);

        for( int i = 0; i < 140; i++ )
        {
            Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
//            Scalar color = Scalar(0,255,0);
            drawContours( drawing, contours2, i, color, 2, 8, hierarchy, 0, Point() );
        }
        namedWindow( "Contours", WINDOW_AUTOSIZE );
        imshow( "Contours", drawing );

        waitKey(0);


  return 0;
}