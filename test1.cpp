#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include <cv.h>
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
/*
int main( int argc, char** argv )
{

  src = imread("data/Inner.jpg");
        if( src.empty() )
        {
            std::cout << "Could not open or find the image!\n" << std::endl;
        }
       
        dst.create( src.size(), src.type() );

        Mat blur;
        cvtColor( src, src_gray, COLOR_BGR2GRAY );
        GaussianBlur( src_gray, blur, Size(5,5),0 );

        namedWindow( window_name, WINDOW_AUTOSIZE );
        imshow("preprocess image",src_gray);


        //threshold(src_gray,detected_edges,0,255,THRESH_BINARY+THRESH_OTSU);
        adaptiveThreshold(src_gray, detected_edges, 255, ADAPTIVE_THRESH_GAUSSIAN_C,CV_THRESH_BINARY,105,1);
        vector<vector<Point> > contours2;
        vector<Vec4i> hierarchy;

        findContours( detected_edges, contours2, hierarchy, RETR_LIST, CHAIN_APPROX_SIMPLE, Point(0, 0) );
        cout<<"numero de contornos2: "<<contours2.size()<<endl;
        namedWindow( window_name, WINDOW_AUTOSIZE );
        imshow("Threshold",detected_edges);

        vector<RotatedRect> minRect( contours2.size() );
        vector<RotatedRect> minEllipse( contours2.size() );


        Mat drawing = Mat::zeros( detected_edges.size(), CV_8UC3 );
        RNG rng(12345);

        for( int i = 0; i < contours2.size(); i++ )
        {
          minRect[i] = minAreaRect( Mat(contours2[i]) );
          if(contours2[i].size() > 5 )
          { 
            minEllipse[i] = fitEllipse( Mat(contours2[i]) ); 
          }
        }

        for( int i = 0; i < 41; i++ )
        {

          cout << "Iteración: " << i <<" "<<contours2[i].size() << endl;
            Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
//       
            drawContours( drawing, contours2, i, color, 2, 8, hierarchy, 0, Point() );
            ellipse(drawing, minEllipse[i], color, 2, 8);

            Point2f rect_points[4]; minRect[i].points( rect_points );
            for( int j = 0; j < 4; j++ )
              line( drawing, rect_points[j], rect_points[(j+1)%4], color, 1, 8 );
        }

        namedWindow( "Contours", WINDOW_AUTOSIZE );
        imshow( "Contours", drawing );

        waitKey(0);


  return 0;
}
*/

int main()
{
    VideoCapture cap("data/PadronAnillos_01.avi");
    if ( !cap.isOpened() )
    {
        cout << "Cannot open the video file. \n";
        return -1;
    }

    double fps = cap.get(CV_CAP_PROP_FPS); 
    namedWindow("A_good_name",CV_WINDOW_AUTOSIZE);
    

    while(1)
    {
        Mat frame;
       

        if (!cap.read(frame)) 
        {
            cout<<"\n Cannot read the video file. \n";
            break;
        }

        Mat blur;
        cvtColor( frame, src_gray, COLOR_BGR2GRAY );
        GaussianBlur( src_gray, blur, Size(5,5),0 );
        //threshold(src_gray,detected_edges,0,255,THRESH_BINARY+THRESH_OTSU);
        adaptiveThreshold(src_gray, detected_edges, 255, ADAPTIVE_THRESH_GAUSSIAN_C,CV_THRESH_BINARY,185,55);
        vector<vector<Point> > contours2;
        vector<Vec4i> hierarchy;
        findContours( detected_edges, contours2, hierarchy, RETR_LIST, CHAIN_APPROX_SIMPLE, Point(0, 0) );
        Mat drawing = Mat::zeros( detected_edges.size(), CV_8UC3 );
        RNG rng(12345);

        //cout << contours2.size() << endl;

        for( int i = 0; i < contours2.size(); i++ )
        {
            Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
            drawContours( drawing, contours2, i, color, 2, 8, hierarchy, 0, Point() );
        }

        imshow("A_good_name", drawing);

        if(waitKey(30) == 27) 
        { 
            break; 
        }
    }

    return 0;
}