#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include <iostream>
using namespace cv;
using namespace std;
   
// int main(int argc, char** argv)
// {
// //   Mat img = imread("BandM10.jpg",CV_LOAD_IMAGE_COLOR);
// //   imshow("opencvtest",img);
// //   waitKey(0);
// //   return 0;
//    const char* filename = argc >=2 ? argv[1] : "patron2.png";
//     // Loads an image
//     Mat src = imread( filename, IMREAD_COLOR );
//     // Check if image is loaded fine
//     if(src.empty()){
//         printf(" Error opening image\n");
//         printf(" Program Arguments: [image_name -- default %s] \n", filename);
//         return -1;
//     }
   
//    Mat gray;
//    cvtColor(src, gray, COLOR_BGR2GRAY);

//    medianBlur(gray, gray, 5);


//    vector<Vec3f> circles;
//    HoughCircles(gray, circles, HOUGH_GRADIENT, 1,
//                  gray.rows/16,  // change this value to detect circles with different distances to each other
//                  100, 30, 1, 30 // change the last two parameters
//             // (min_radius & max_radius) to detect larger circles
//     );
   
//    for( size_t i = 0; i < circles.size(); i++ )
//     {
//         Vec3i c = circles[i];
//         Point center = Point(c[0], c[1]);
//         // circle center
//         circle( src, center, 1, Scalar(0,100,100), 3, LINE_AA);
//         // circle outline
//         int radius = c[2];
//         circle( src, center, radius, Scalar(255,0,255), 3, LINE_AA);
//     }
//    imshow("detected circles", src);
//    waitKey();
// }


/// Global variables
//![variables]
Mat src, src_gray;
Mat dst, detected_edges;

int lowThreshold = 10;
const int max_lowThreshold = 100;
const int ratio = 3;
const int kernel_size = 3;
const char* window_name = "Edge Map";
//![variables]

/**
 * @function CannyThreshold
 * @brief Trackbar callback - Canny thresholds input with a ratio 1:3
 */
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
  //![load]
  CommandLineParser parser( argc, argv, "{@input | patron2.png | input image}" );
  src = imread( parser.get<String>( "@input" ), IMREAD_COLOR ); // Load an image

  if( src.empty() )
  {
    std::cout << "Could not open or find the image!\n" << std::endl;
    std::cout << "Usage: " << argv[0] << " <Input image>" << std::endl;
    return -1;
  }
  //![load]

  //![create_mat]
  /// Create a matrix of the same type and size as src (for dst)
  dst.create( src.size(), src.type() );
  //![create_mat]

  //![convert_to_gray]
  cvtColor( src, src_gray, COLOR_BGR2GRAY );
  //![convert_to_gray]

// bilateral_filtered_image = bilateralFilter(src, 5, 175, 175)
// cv2.imshow('Bilateral', bilateral_filtered_image)
// cv2.waitKey(0)


  //![create_window]
  namedWindow( window_name, WINDOW_AUTOSIZE );
  //![create_window]

  //![create_trackbar]
  /// Create a Trackbar for user to enter threshold
  createTrackbar( "Min Threshold:", window_name, &lowThreshold, max_lowThreshold, CannyThreshold );
  //![create_trackbar]

  /// Show the image
  CannyThreshold(1, 0);


   Mat gry,nor;

   GaussianBlur( src, nor, Size(5,5),0 );  
   cvtColor(nor,gry,CV_BGR2GRAY);
   threshold(gry,detected_edges,0,255,THRESH_BINARY+THRESH_OTSU); 

  //extraer contornos
   vector<vector<Point> > contours;
   vector<Vec4i> hierarchy;
   findContours( detected_edges, contours, hierarchy, RETR_LIST, CHAIN_APPROX_SIMPLE, Point(0, 0) );

   cout<<"numero de contornos: "<<contours.size()<<endl;

Mat drawing = Mat::zeros( detected_edges.size(), CV_8UC3 );
RNG rng(12345);

  for( int i = 0; i < 140; i++ )
     {
      //  Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
      Scalar color = Scalar(0,255,0);
       drawContours( drawing, contours, i, color, 2, 8, hierarchy, 0, Point() );
     }
  namedWindow( "Contours", WINDOW_AUTOSIZE );
  imshow( "Contours", drawing );

  /// Wait until user exit program by pressing a key
  waitKey(0);

  return 0;
}