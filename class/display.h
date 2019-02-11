
 #include <iostream>
 #include "opencv2/imgcodecs.hpp"
 #include "opencv2/highgui.hpp"
 #include "opencv2/imgproc.hpp"
 #include <stdarg.h> 
 using namespace std;
 using namespace cv;


 void ShowManyImages(string title, int n_fails, int total_frames, float rms_first, float rms, Mat cameraMatrix, int nArgs, ...) {
    int size;
    int i;
    int m, n;
    int x, y;
    int w, h;

    float scale;
    int max;

    if(nArgs <= 0) {
        //printf("Number of arguments too small....\n");
        return;
    }
    else if(nArgs > 14) {
        //printf("Number of arguments too large, can only handle maximally 12 images at a time ...\n");
        return;
    }
    else if (nArgs == 1) {
        w = h = 1;
        size = 400;
    }
    else if (nArgs == 2) {
        w = 2; h = 1;
        size = 400;
    }
    else if (nArgs == 3 || nArgs == 4) {
        w = 2; h = 2;
        size = 400;
    }
    else if (nArgs == 5 || nArgs == 6) {
        w = 3; h = 2;
        size = 200;
    }
    else if (nArgs == 7 || nArgs == 8) {
        w = 4; h = 2;
        size = 200;
    }
    else {
        w = 4; h = 3;
        size = 150;
    }


    Mat DispImage = Mat::zeros(Size(350 + size*w, size*h), CV_8UC3);

    va_list args;
    va_start(args, nArgs);

    for (i = 0, m = 20, n = 120; i < nArgs; i++, m += (20 + size)) {
        Mat all_img;
        all_img = va_arg(args, cv::Mat);
        if(all_img.empty()) {
            //printf("Invalid arguments");
            return;
        }

        x = all_img.cols;
        y = all_img.rows;

        max = (x > y)? x: y;

        scale = (float) ( (float) max / size );

        if( i % w == 0 && m!= 20) {
            m = 20;
            n+= 20 + size-80;
        }


        Rect ROI(m, n, (int)( x/scale ), (int)( y/scale ));
        Mat temp; resize(all_img,temp, Size(ROI.width, ROI.height));
        temp.copyTo(DispImage(ROI));
    }

    namedWindow( title, 1 );
    putText(DispImage, "CAMERA CALIBRATION", Point2f(400,50), FONT_HERSHEY_PLAIN, 2,  Scalar(202,109,200), 2 , 8 , false);
    putText(DispImage, "Original Frame", Point2f(100,90), FONT_HERSHEY_PLAIN, 2,  Scalar(202,109,16), 2 , 8 , false);
    putText(DispImage, "Fronto Parallel", Point2f(500,90), FONT_HERSHEY_PLAIN, 2,  Scalar(202,109,16), 2 , 8 , false);
    putText(DispImage, "Undistorted First", Point2f(100,450), FONT_HERSHEY_PLAIN, 2,  Scalar(202,109,16), 2 , 8 , false);
    putText(DispImage, "Undistorted Last", Point2f(500,450), FONT_HERSHEY_PLAIN, 2,  Scalar(202,109,16), 2 , 8 , false);
    //putText(DispImage, "Time: "+(to_string((float)duration/(float)total_frames)), Point2f(80,790), FONT_HERSHEY_PLAIN, 2,  Scalar(202,109,16), 2 , 8 , false);
    //putText(DispImage, "Fails: "+to_string(n_fails)+" of: "+to_string(total_frames), Point2f(500,790), FONT_HERSHEY_PLAIN, 2,  Scalar(202,109,16), 2 , 8 , false);
    putText(DispImage, "rms: "+to_string(rms_first), Point2f(80,790), FONT_HERSHEY_PLAIN, 2,  Scalar(202,109,16), 2 , 8 , false);
    putText(DispImage, "rms: "+to_string(rms), Point2f(500,790), FONT_HERSHEY_PLAIN, 2,  Scalar(202,109,16), 2 , 8 , false);
    putText(DispImage, "Calibrate: ", Point2f(870,200), FONT_HERSHEY_PLAIN, 2,  Scalar(202,109,16), 2 , 8 , false);
   if(rms==-1){
        putText(DispImage, "Rms: "+to_string(0), Point2f(870,300), FONT_HERSHEY_PLAIN, 2,  Scalar(202,109,16), 2 , 8 , false);
        putText(DispImage, "Fx: "+to_string(0), Point2f(870,350), FONT_HERSHEY_PLAIN, 2,  Scalar(202,109,16), 2 , 8 , false);
        putText(DispImage, "Fy: "+to_string(0), Point2f(870,400), FONT_HERSHEY_PLAIN, 2,  Scalar(202,109,16), 2 , 8 , false);
        putText(DispImage, "Cx: "+to_string(0), Point2f(870,450), FONT_HERSHEY_PLAIN, 2,  Scalar(202,109,16), 2 , 8 , false);
        putText(DispImage, "Cy: "+to_string(0), Point2f(870,500), FONT_HERSHEY_PLAIN, 2,  Scalar(202,109,16), 2 , 8 , false);
    }
    else
    {
        putText(DispImage, "Rms: "+to_string(rms), Point2f(870,300), FONT_HERSHEY_PLAIN, 2,  Scalar(202,109,16), 2 , 8 , false);
        putText(DispImage, "Fx: "+to_string(cameraMatrix.at<double>(0,0)), Point2f(870,350), FONT_HERSHEY_PLAIN, 2,  Scalar(202,109,16), 2 , 8 , false);
        putText(DispImage, "Fy: "+to_string(cameraMatrix.at<double>(0,2)), Point2f(870,400), FONT_HERSHEY_PLAIN, 2,  Scalar(202,109,16), 2 , 8 , false);
        putText(DispImage, "Cx: "+to_string(cameraMatrix.at<double>(1,1)), Point2f(870,450), FONT_HERSHEY_PLAIN, 2,  Scalar(202,109,16), 2 , 8 , false);
        putText(DispImage, "Cy: "+to_string(cameraMatrix.at<double>(1,2)), Point2f(870,500), FONT_HERSHEY_PLAIN, 2,  Scalar(202,109,16), 2 , 8 , false);
    }
   
    //string a = to_string(countimages);
    imshow( title, DispImage);

    va_end(args);
 }

// void ShowManyImages(string title, int n_fails, int total_frames, float rms_first, float rms, int nArgs, ...) {
//    int size;
//    int i;
//    int m, n;
//    int x, y;

//    int w, h;

//    float scale;
//    int max;

//    if(nArgs <= 0) {
//        //printf("Number of arguments too small....\n");
//        return;
//    }
//    else if(nArgs > 14) {
//        //printf("Number of arguments too large, can only handle maximally 12 images at a time ...\n");
//        return;
//    }
//    else if (nArgs == 1) {
//        w = h = 1;
//        size = 400;
//    }
//    else if (nArgs == 2) {
//        w = 2; h = 1;
//        size = 400;
//    }
//    else if (nArgs == 3 || nArgs == 4) {
//        w = 2; h = 2;
//        size = 400;
//    }
//    else if (nArgs == 5 || nArgs == 6) {
//        w = 3; h = 2;
//        size = 200;
//    }
//    else if (nArgs == 7 || nArgs == 8) {
//        w = 4; h = 2;
//        size = 200;
//    }
//    else {
//        w = 4; h = 3;
//        size = 150;
//    }


//    Mat DispImage = Mat::zeros(Size(350 + size*w, size*h), CV_8UC3);

//    va_list args;
//    va_start(args, nArgs);

//    for (i = 0, m = 20, n = 120; i < nArgs; i++, m += (20 + size)) {
//        Mat all_img;
//        all_img = va_arg(args, cv::Mat);

//        if(all_img.empty()) {
//            //printf("Invalid arguments");
//            return;
//        }

//        x = all_img.cols;
//        y = all_img.rows;

//        max = (x > y)? x: y;

//        scale = (float) ( (float) max / size );

//        if( i % w == 0 && m!= 20) {
//            m = 20;
//            n+= 20 + size-80;
//        }


//        Rect ROI(m, n, (int)( x/scale ), (int)( y/scale ));
//        Mat temp; 
//        resize(all_img,temp, Size(ROI.width, ROI.height));
//        temp.copyTo(DispImage(ROI));
//    }

//    namedWindow( title, 1 );
//    putText(DispImage, "CAMERA CALIBRATION", Point2f(400,50), FONT_HERSHEY_PLAIN, 2,  Scalar(202,109,200), 2 , 8 , false);
//    putText(DispImage, "Fronto img", Point2f(100,90), FONT_HERSHEY_PLAIN, 2,  Scalar(202,109,16), 2 , 8 , false);
//    putText(DispImage, "preprocessing", Point2f(500,90), FONT_HERSHEY_PLAIN, 2,  Scalar(202,109,16), 2 , 8 , false);
//    putText(DispImage, "Undistorted First", Point2f(100,450), FONT_HERSHEY_PLAIN, 2,  Scalar(202,109,16), 2 , 8 , false);
//    putText(DispImage, "Undistorted Last", Point2f(500,450), FONT_HERSHEY_PLAIN, 2,  Scalar(202,109,16), 2 , 8 , false);
//    //putText(DispImage, "Time: "+(to_string((float)duration/(float)total_frames)), Point2f(80,790), FONT_HERSHEY_PLAIN, 2,  Scalar(202,109,16), 2 , 8 , false);
//    //putText(DispImage, "Fails: "+to_string(n_fails)+" of: "+to_string(total_frames), Point2f(500,790), FONT_HERSHEY_PLAIN, 2,  Scalar(202,109,16), 2 , 8 , false);
//    putText(DispImage, "rms: "+to_string(rms_first), Point2f(80,790), FONT_HERSHEY_PLAIN, 2,  Scalar(202,109,16), 2 , 8 , false);
//    putText(DispImage, "rms: "+to_string(rms), Point2f(500,790), FONT_HERSHEY_PLAIN, 2,  Scalar(202,109,16), 2 , 8 , false);
//    putText(DispImage, "Calibrate: ", Point2f(870,200), FONT_HERSHEY_PLAIN, 2,  Scalar(202,109,16), 2 , 8 , false);

   
//    //string a = to_string(countimages);
//    imshow( title, DispImage);
//    if(waitKey(1000) == 27)
//     {
//         return;
//     }

//    va_end(args);
// }