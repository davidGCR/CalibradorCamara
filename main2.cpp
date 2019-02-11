

#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include <chrono>
#include <stdio.h>
#include <stdarg.h>
#include <string>
#include <time.h>
#include <cmath>
#include <iomanip>
#include "opencv2/calib3d.hpp"

#include "class/ellipse.h"
#include "class/Line.h"
// #include "class/constants.h"
#include "class/quadrant.h"
#include "class/display.h"
#include "class/preprocessing.h"
#include "class/controlPointDetector.h"
#include "class/utils.h"
#include "class/iterativeCalibFunctions.h"

#include "class/cameraCalib.h"
#include <stdio.h>



//#include <cv.h>
#include <iostream>
using namespace cv;
using namespace std;

auto duration = 0;

float calibrate_camera(Size &imageSize, Mat &cameraMatrix, Mat &distCoeffs, vector<vector<Point2f>>& imagePoints) {
    /*
     * OjectPoints: vector que describe como debe verse el patron
     * imagePoints: vector de vectores con puntos de control(un frame tiene un vector de N puntos de control)
     * imageSize: tamaño de la imagen
     * distCoeffs: coeficientes de distorcion
     * rvecs: vector de rotacion
     * tvecs: vector de tranlacion
     */
    Size boardSize(PATTERN_NUM_COLS, PATTERN_NUM_ROWS);
    float squareSize = 44;
    //    float squareSize = 44.3;
    float aspectRatio = 1;
    vector<Mat> rvecs;
    vector<Mat> tvecs;
    vector<float> reprojErrs;
    vector<vector<Point3f> > objectPoints(1);

    distCoeffs = Mat::zeros(8, 1, CV_64F);

    objectPoints[0].resize(0);
    for ( int i = 0; i < boardSize.height; i++ ) {
        for ( int j = 0; j < boardSize.width; j++ ) {
            objectPoints[0].push_back(Point3f(  float(j * squareSize),
                                              float(i * squareSize), 0));
        }
    }

    objectPoints.resize(imagePoints.size(), objectPoints[0]);

    double rms = calibrateCamera(objectPoints,
                                 imagePoints,
                                 imageSize,
                                 cameraMatrix,
                                 distCoeffs,
                                 rvecs,
                                 tvecs,
                                 0/*,
                                   CV_CALIB_ZERO_TANGENT_DIST*/);
    cout<<"rotation matrix: "<<rvecs.size()<<endl;
    cout<<"traslation matrix: "<<tvecs.size()<<endl;
    return rms;
}

vector<Point2f> ellipses2Points(vector<P_Ellipse> ellipses){
    vector<Point2f> buffer(REAL_NUM_CTRL_PTS);
    for(int i=0;i<REAL_NUM_CTRL_PTS;i++){
        buffer[i] = ellipses[i].center();
    }

    return buffer;
}



/*
 * Iterative Refinement functions
 *
 */
void select_frames_by_time(VideoCapture& cap, vector<Mat>& out_frames_selected, int delay_time,int num_frames_for_calibration){
    //util for save frame
    cout << "Choosing frames by time... \n";
    float frame_time = 0;
    int points_detected = 0;
    int frame_count = 0; //current frame
    int n_fails = 0;
    //found pattern points
    vector<P_Ellipse> control_points;

    while (1) {
        Mat frame, rview;
        cap>>frame;
        if (frame.empty()) {
            cout << "Cannot capture frame. \n";
            break;
        }
        //name for save frame
        frame_time = cap.get(CV_CAP_PROP_POS_MSEC)/1000;
        //count frame
        frame_count++;

        Mat frame_preprocessed;
        preprocessing_frame(&frame, &frame_preprocessed);
        Mat img_ellipses = frame.clone();
        points_detected = find_ellipses(&frame_preprocessed, &img_ellipses,control_points,frame_time,n_fails);

        if(frame_count% delay_time == 0 && points_detected == REAL_NUM_CTRL_PTS){
            out_frames_selected.push_back(frame);
            if(out_frames_selected.size()==num_frames_for_calibration){
                break;
            }
        }
    }
}

void save_frame(String data_path, string name,Mat& frame){
    string s = data_path +name+".jpg";
    imwrite(s,frame);
}

void plot_quadrants(Mat& frame,Size f_size,vector<Quadrant> quadrants){
    //Quadrants plotting
    frame = Mat::zeros(f_size, CV_8UC3);

    for (int i=0; i<quadrants.size(); i++) {
        circle(frame, quadrants[i].c_center,  quadrants[i].c_radio, rose, 2);
        putText(frame, to_string(quadrants[i].frame_counter), quadrants[i].c_center, FONT_HERSHEY_COMPLEX_SMALL, 1, white, 1);
    }
}

void select_frames(VideoCapture& cap, vector<Mat>& out_frames_selected, Size f_size,
                   int no_frames_desired,int n_quads_rows=3,int num_quads_cols=4, bool DEBUG_MODE = false){

    //util for save frame
    float x_quad_lenght = float(f_size.width)/float(num_quads_cols);
    float y_quad_lenght = float(f_size.height)/float(n_quads_rows);

    /******************************************** Build quadrants *************************************************++*/
    //Quadrants building
    vector<Quadrant> quadrants;
    int space = 20;
    int radio = y_quad_lenght/2 - space;
    int total_quadrants = n_quads_rows * num_quads_cols;
    int no_frames_by_quadrant = no_frames_desired/total_quadrants;

    for(int i=0;i<n_quads_rows;i++){
        for(int j=0;j<num_quads_cols;j++){
            Quadrant quad(j*x_quad_lenght, i*y_quad_lenght, x_quad_lenght, y_quad_lenght,radio,no_frames_by_quadrant);
            quadrants.push_back(quad);
        }
    }
    //expands quadrants corners
    int newradio = x_quad_lenght/2-5;
    quadrants[0].setRadio(newradio);
    quadrants[3].setRadio(newradio);
    quadrants[8].setRadio(newradio);
    quadrants[11].setRadio(newradio);

    //add intermediate quadrants
    int miniradio = 20;
    int capacity = 2;
    quadrants.push_back(Quadrant (x_quad_lenght, y_quad_lenght, 0, 0,miniradio,capacity));
    quadrants.push_back(Quadrant (3*x_quad_lenght, y_quad_lenght, 0, 0,miniradio,capacity));
    quadrants.push_back(Quadrant (x_quad_lenght, 2*y_quad_lenght, 0, 0,miniradio,capacity));
    quadrants.push_back(Quadrant (3*x_quad_lenght, 2*y_quad_lenght, 0, 0,miniradio,capacity));

    // DEBUG_MODE = true;

    Mat img_quadrants,img_quadrants_track;
    if(DEBUG_MODE){
        cout<<"quadrants.size(): "<<quadrants.size()<<endl;
        plot_quadrants(img_quadrants,f_size, quadrants);
        img_quadrants_track = img_quadrants.clone();
    }

    /*********************************************************************************************++*/

    float frame_time = 0;
    int points_detected = 0;

    int n_fails = 0;
    vector<P_Ellipse> control_points;
    Point2f pattern_center;
    int delay_skip = 50;
    int last_quadrant = -1;
    int total_selected_frames=0;



    //    cap>>frame;
    /******************************************** Select Frames(1) *************************************************++*/

    for (int i = 0;total_selected_frames<no_frames_desired;i++) {

        //        cap.set(CAP_PROP_POS_FRAMES,start+i);
        Mat frame;
        cap>>frame;
        if (frame.empty()) {
            cout << "Cannot capture frame. \n";
            break;
        }
        if(i%delay_skip){
            Mat frame_preprocessed;
            preprocessing_frame(&frame, &frame_preprocessed);
            Mat img_ellipses = frame.clone();
            points_detected = find_ellipses(&frame_preprocessed, &img_ellipses,control_points,frame_time,n_fails);

            if(DEBUG_MODE){
                cv::circle(img_quadrants_track, pattern_center, 2, green);
                imshow("real ", img_ellipses);
                imshow("track ", img_quadrants_track);
            }

            //
            if(points_detected == REAL_NUM_CTRL_PTS){
                pattern_center.x = (control_points[7].center().x + control_points[12].center().x)/2;
                pattern_center.y = (control_points[7].center().y + control_points[12].center().y)/2;

                for (int k=0; k<quadrants.size(); k++) {
                    if(quadrants[k].isInclude(pattern_center)){
                        if(last_quadrant != k && quadrants[k].frame_counter < quadrants[k].capacity){
                            out_frames_selected.push_back(frame);
                            quadrants[k].frame_counter++;
                            total_selected_frames++;
                            last_quadrant = k;

                            save_frame(PATH_DATA_FRAMES+"iteration0/selected/","selected-"+to_string(total_selected_frames),frame);

                            if(DEBUG_MODE){
                                plot_quadrants(img_quadrants, f_size,quadrants);
                                imshow("counts", img_quadrants);
                            }

                            break;
                        }
                    }
                }
            }

        }

        if(waitKey(15) == 27)
        {
            break;
        }
    }
    // save_frame(PATH_DATA_FRAMES,"counter-",img_quadrants);
    cout<<"Only selected: "<<out_frames_selected.size()<<endl;
}

void distortPoints(vector<Point2f>& undistortedPoints, vector<Point2f>& distortedPoints,
                   Mat cameraMatrix, Mat distCoef){

    double fx = cameraMatrix.at<double>(0, 0);
    double fy = cameraMatrix.at<double>(1, 1);
    double cx = cameraMatrix.at<double>(0, 2);
    double cy = cameraMatrix.at<double>(1, 2);
    double k1 = distCoef.at<double>(0, 0);
    double k2 = distCoef.at<double>(0, 1);
    double p1 = distCoef.at<double>(0, 2);
    double p2 = distCoef.at<double>(0, 3);
    double k3 = distCoef.at<double>(0, 4);

    double x;
    double y;
    double r2;
    double xDistort;
    double yDistort;
    for (int p = 0; p < undistortedPoints.size(); p++) {
        x = (undistortedPoints[p].x - cx) / fx;
        y = (undistortedPoints[p].y - cy) / fy;
        r2 = x * x + y * y;

        // Radial distorsion
        xDistort = x * (1 + k1 * r2 + k2 * pow(r2, 2) + k3 * pow(r2, 3));
        yDistort = y * (1 + k1 * r2 + k2 * pow(r2, 2) + k3 * pow(r2, 3));

        // Tangential distorsion
        xDistort = xDistort + (2 * p1 * x * y + p2 * (r2 + 2 * x * x));
        yDistort = yDistort + (p1 * (r2 + 2 * y * y) + 2 * p2 * x * y);

        // Back to absolute coordinates.
        xDistort = xDistort * fx + cx;
        yDistort = yDistort * fy + cy;
        distortedPoints[p] = Point2f(xDistort, yDistort);
    }
}

void plot_control_points(Mat& img_in, Mat& img_out, vector<Point2f>& control_points,Scalar color){
    // img_out = img_in.clone();
    for(int i=0;i<control_points.size();i++){
        circle(img_out, control_points[i], 5, color, 0.5);
    }
}
void create_real_pattern(int h, int w, vector<Point3f>& out_real_centers){

    float margin_h = 70;//50
    float margin_w = 90;
    float distance_points = 110;
    out_real_centers.clear();
    out_real_centers.push_back(Point3f(  float(margin_w+distance_points*0) ,float( margin_h), 0));
    out_real_centers.push_back(Point3f(  float(margin_w+distance_points*1) ,float( margin_h), 0));
    out_real_centers.push_back(Point3f(  float(margin_w+distance_points*2) ,float( margin_h), 0));
    out_real_centers.push_back(Point3f(  float(margin_w+distance_points*3) ,float( margin_h), 0));
    out_real_centers.push_back(Point3f(  float(margin_w+distance_points*4) ,float( margin_h), 0));

    out_real_centers.push_back(Point3f(  float(margin_w+distance_points*0) ,float( margin_h+distance_points*1), 0));
    out_real_centers.push_back(Point3f(  float(margin_w+distance_points*1) ,float( margin_h+distance_points*1), 0));
    out_real_centers.push_back(Point3f(  float(margin_w+distance_points*2) ,float( margin_h+distance_points*1), 0));
    out_real_centers.push_back(Point3f(  float(margin_w+distance_points*3) ,float( margin_h+distance_points*1), 0));
    out_real_centers.push_back(Point3f(  float(margin_w+distance_points*4) ,float( margin_h+distance_points*1), 0));

    out_real_centers.push_back(Point3f(  float(margin_w+distance_points*0) ,float( margin_h+distance_points*2), 0));
    out_real_centers.push_back(Point3f(  float(margin_w+distance_points*1) ,float( margin_h+distance_points*2), 0));
    out_real_centers.push_back(Point3f(  float(margin_w+distance_points*2) ,float( margin_h+distance_points*2), 0));
    out_real_centers.push_back(Point3f(  float(margin_w+distance_points*3) ,float( margin_h+distance_points*2), 0));
    out_real_centers.push_back(Point3f(  float(margin_w+distance_points*4) ,float( margin_h+distance_points*2), 0));

    out_real_centers.push_back(Point3f(  float(margin_w+distance_points*0) ,float( margin_h+distance_points*3), 0));
    out_real_centers.push_back(Point3f(  float(margin_w+distance_points*1) ,float( margin_h+distance_points*3), 0));
    out_real_centers.push_back(Point3f(  float(margin_w+distance_points*2) ,float( margin_h+distance_points*3), 0));
    out_real_centers.push_back(Point3f(  float(margin_w+distance_points*3) ,float( margin_h+distance_points*3), 0));
    out_real_centers.push_back(Point3f(  float(margin_w+distance_points*4) ,float( margin_h+distance_points*3), 0));


    Mat real_points_img = Mat::zeros(Size(h,w), CV_8UC3);
    for(int i=0;i<out_real_centers.size();i++){
        circle(real_points_img, Point2f(out_real_centers[i].x,out_real_centers[i].y), 2, rose, 2);
    }
    save_frame(PATH_DATA_FRAMES,"ideal image", real_points_img);
}

int find_control_points(Mat& frame,Mat& prep_output,Mat& output,vector<P_Ellipse>& out_control_points,int iteration=-1,int n_frame=-1, bool PP_MODE=false){

    Mat frame_preprocessed;

    if(!PP_MODE)
        preprocessing_frame(&frame, &frame_preprocessed);
    else
        preprocessing_frame2(frame, frame_preprocessed);

    prep_output = frame_preprocessed;


    // if(n_frame!=-1 && iteration!=-1){
    //     save_frame(PATH_DATA_FRAMES+"preprocesed/","iter-"+to_string(iteration)+"-frm-"+to_string(n_frame),frame_preprocessed);
    // }

    output = frame.clone();
    int  points_detected = -1;
    int fails = 0;
    points_detected = find_ellipses(&frame_preprocessed, &output,out_control_points,0,fails);
    // if(n_frame!=-1 && iteration!=-1){
    //     save_frame(PATH_DATA_FRAMES+"detected/","iter-"+to_string(iteration)+"-frm-"+to_string(n_frame),output);
    // }
    // cout << "find_control_points test: "<<points_detected<< endl;
    return points_detected;
}

void avg_control_points(vector<Point2f>& control_points_undistort, vector<Point2f>& control_points_reproject){
    for (int i=0; i<control_points_reproject.size(); i++) {
        control_points_reproject[i].x = (control_points_undistort[i].x+control_points_reproject[i].x)/2;
        control_points_reproject[i].y = (control_points_undistort[i].y+control_points_reproject[i].y)/2;
    }
}

/*
 * busca puntos de controlen un conjunto de frames, quita la distorcion y crea la imagen frontoparalela.
 * return: lista de frames frontoparalelos
 */
void fronto_parallel_images(vector<Mat>& selected_frames,vector<Mat>& out_fronto_images,Size frameSize,
                            vector<Point3f> real_centers,
                            Mat& cameraMatrix, Mat& distCoeffs,vector<vector<Point2f>>& imagePoints,int iteration){

    // Mat cameraMatrix;
    // Mat distCoeffs = Mat::zeros(8, 1, CV_64F);
    vector<P_Ellipse> control_points_undistort;
    vector<P_Ellipse> original_control_points;
    vector<P_Ellipse> fronto_control_points;

    vector<Point2f> intersection_points;

    int num_control_points=0;
    int num_control_points_fronto=0;
    double rms=-1;
    Mat output_img_control_points;
    Mat output_img_fronto_control_points;
    Mat frame;
    Mat img_fronto_parallel;
    Mat img_prep_proc;
    Mat reprojected_image;
    // string path_data = "/home/david/Escritorio/calib-data/frames/";
    int fails = 0;

    for (int i=0; i<selected_frames.size(); i++) {
        //        cout << "============================IMAGE  "<<i<< endl;
        control_points_undistort.clear();
        original_control_points.clear();
        intersection_points.clear();

        frame = selected_frames[i].clone();
        output_img_control_points = frame.clone();
        // save_frame(PATH_DATA_FRAMES+"1-raw/","raw: iter-"+to_string(iteration)+"-frm-"+to_string(i),frame);

        /**************** undisort*********************/
        // cout << "Undistort image ... "<< endl;
        // Mat map1, map2;
        // initUndistortRectifyMap(cameraMatrix, distCoeffs, Mat(),getOptimalNewCameraMatrix(cameraMatrix, cameraMatrix,
        // frameSize, 1, frameSize, 0),frameSize, CV_16SC2, map1, map2);

        Mat undistorted_image = frame.clone();
        // remap(frame, undistorted_image, map1, map2, INTER_LINEAR);
        undistort(frame, undistorted_image, cameraMatrix, distCoeffs);

        /**************** detect control points in original image *********************/
        int n_ctrl_points_original = find_control_points(frame, img_prep_proc,output_img_control_points,original_control_points,iteration,i,false);
        vector<Point2f> original_control_points2f = ellipses2Points(original_control_points);


        /**************** detect control points in undistorted image *********************/
        int n_ctrl_points_undistorted = find_control_points(undistorted_image, img_prep_proc,output_img_control_points,control_points_undistort,iteration,i,false);
        save_frame(PATH_DATA_FRAMES+"2-undisorted/detected/","undist: iter-"+to_string(iteration)+"-frm-"+to_string(i),output_img_control_points);
        save_frame(PATH_DATA_FRAMES+"2-undisorted/preprocesed/","undist_prep: iter-"+to_string(iteration)+"-frm-"+to_string(i),img_prep_proc);

        if(n_ctrl_points_undistorted != REAL_NUM_CTRL_PTS){
            continue;
        }
        vector<Point2f> control_points_undistort2f = ellipses2Points(control_points_undistort);
        //        plot_control_points(output_img_control_points,output_img_control_points,control_points_centers,yellow);
        /**************** unproject*********************/
        // cout << "Unproject image ... "<< endl;
        // vector<Point2f> control_points_2d = ellipses2Points(control_points);

        vector<Point2f> control_points_2d;

        control_points_2d.push_back(control_points_undistort[15].center());
        control_points_2d.push_back(control_points_undistort[16].center());
        control_points_2d.push_back(control_points_undistort[17].center());
        control_points_2d.push_back(control_points_undistort[18].center());
        control_points_2d.push_back(control_points_undistort[19].center());

        control_points_2d.push_back(control_points_undistort[10].center());
        control_points_2d.push_back(control_points_undistort[11].center());
        control_points_2d.push_back(control_points_undistort[12].center());
        control_points_2d.push_back(control_points_undistort[13].center());
        control_points_2d.push_back(control_points_undistort[14].center());

        control_points_2d.push_back(control_points_undistort[5].center());
        control_points_2d.push_back(control_points_undistort[6].center());
        control_points_2d.push_back(control_points_undistort[7].center());
        control_points_2d.push_back(control_points_undistort[8].center());
        control_points_2d.push_back(control_points_undistort[9].center());

        control_points_2d.push_back(control_points_undistort[0].center());
        control_points_2d.push_back(control_points_undistort[1].center());
        control_points_2d.push_back(control_points_undistort[2].center());
        control_points_2d.push_back(control_points_undistort[3].center());
        control_points_2d.push_back(control_points_undistort[4].center());



        Mat homography = findHomography(control_points_2d,real_centers);
        Mat inv_homography = findHomography(real_centers,control_points_2d);

        img_fronto_parallel = undistorted_image.clone();
        warpPerspective(undistorted_image, img_fronto_parallel, homography, frame.size());
        out_fronto_images.push_back(img_fronto_parallel);

        // undistort(img_fronto_parallel, img_fronto_parallel, cameraMatrix, distCoeffs);

        save_frame(PATH_DATA_FRAMES+"3-fronto/","fronto: iter-"+to_string(iteration)+"-frm-"+to_string(i),img_fronto_parallel);

        /**************** Localize control points in fronto parallel frame *********************/
        // cout << "Localize control points in fronto parallel frame ... "<<i<< endl;
        fronto_control_points.clear();
        output_img_fronto_control_points = img_fronto_parallel.clone();

        num_control_points_fronto = find_control_points(img_fronto_parallel,img_prep_proc, output_img_fronto_control_points,fronto_control_points,iteration,i,true);


        /**************** Reproject control points to camera coordinates *********************/

        if(num_control_points_fronto==REAL_NUM_CTRL_PTS){


            save_frame(PATH_DATA_FRAMES+"3-fronto/detected/","det_fronto: iter-"+to_string(iteration)+"-frm-"+to_string(i),output_img_fronto_control_points);
            vector<Point2f> control_points_fronto_images = ellipses2Points(fronto_control_points);

            //find intersection point for each control point
            Mat img_intersections = img_fronto_parallel.clone();
            calculate_intersection(img_intersections,control_points_fronto_images,intersection_points);
            save_frame(PATH_DATA_FRAMES+"3-fronto/intersections/","intersect_iter-"+to_string(iteration)+"-frm-"+to_string(i),img_intersections);

            /********************************************* Control points in fronto image ****************************************************/
            //REPROJECT: To camera coordinates
            vector<Point2f> reprojected_points;
            perspectiveTransform(control_points_fronto_images, reprojected_points, inv_homography);

            //REPROJECT: Distort reprojected points
            vector<Point2f> reprojected_points_distort(REAL_NUM_CTRL_PTS);
            distortPoints(reprojected_points, reprojected_points_distort, cameraMatrix, distCoeffs);

            /********************************************* Intersections ****************************************************/
            //REPROJECT (intersections): To camera coordinates
            vector<Point2f> reprojected_points_intersections;
            perspectiveTransform(intersection_points, reprojected_points_intersections, inv_homography);

            //REPROJECT (intersections): Distort reprojected intersections points
            vector<Point2f> reprojected_points_intersections_distort(REAL_NUM_CTRL_PTS);
            distortPoints(reprojected_points_intersections, reprojected_points_intersections_distort, cameraMatrix, distCoeffs);

            ////Refinement: Normal
            imagePoints.push_back(reprojected_points_distort);

            ////Refinement: Average
            // avg_control_points(original_control_points2f,reprojected_points_distort);
            // imagePoints.push_back(reprojected_points_distort);

            ////Refinement Baricenter
            // vector<Point2f> baricenters(REAL_NUM_CTRL_PTS);
            // baricenter(original_control_points2f,reprojected_points_distort,reprojected_points_intersections_distort,baricenters);
            // imagePoints.push_back(baricenters);

            reprojected_image = frame.clone();
            plot_control_points(undistorted_image,reprojected_image,original_control_points2f,green);
            plot_control_points(reprojected_image,reprojected_image,reprojected_points_distort,red);
            plot_control_points(reprojected_image,reprojected_image,reprojected_points_intersections_distort,white);

            save_frame(PATH_DATA_FRAMES+"4-reprojected/","rep_iter-"+to_string(iteration)+"-frm-"+to_string(i),reprojected_image);

            // cout << "Found Control points in frame: size:  "<<imagePoints.size()<< endl;
        }
        else{
            // cout << "Not found control points in all fronto parallel images... "<< endl;
        }


        // imshow("fronto paralell",img_fronto_parallel);

        // save_frame(path_data,"fronto"+to_string(i),img_fronto_parallel);
        // break;

    }
}
/*******************************************************************************/



int debug_images_fronto()
{

    //string path_data = "/Users/davidchoqueluqueroman/Desktop/CURSOS-MASTER/IMAGENES/testOpencv/data/";
    string frames_file = PATH_DATA_FRAMES;
    // string video_file = PATH_DATA+"cam2/anillos.avi";

    //initial frame for get size of frames
    Mat fronto_img, fronto_img_pre, fronto_img_ell;

    vector<cv::String> fn;
    glob(PATH_DATA_FRAMES+"3-fronto/*.jpg", fn, false);

    vector<Mat> images;
    size_t count = fn.size(); //total number  files in images folder

    size_t n = 30;

    if (count>0) {
        int fails = 0;
        int points_detected;
        vector<P_Ellipse> out_control_points;

        for (size_t i=0; i<count; i++){
            out_control_points.clear();
            fronto_img = imread(fn[i]);
            // cout<<"chanels : "<<fronto_img.channels()<<endl;
            preprocessing_frame2(fronto_img,fronto_img_pre);
            fronto_img_ell = fronto_img.clone();
            points_detected = find_ellipses(&fronto_img_pre, &fronto_img_ell,out_control_points,0,fails);
            cout<<"cp detected : "<<points_detected<<endl;

            imshow("fronto",fronto_img);
            imshow("preprocess",fronto_img_pre);
            imshow("ellipses",fronto_img_ell);
            // ShowManyImages("resultado", 2, 3, 0, 0, 4, fronto_img, fronto_img_pre, fronto_img,fronto_img);
            // cout<<"file: "<<fn[i]<<endl;
            if(waitKey(1000) == 27)
            {
                break;
            }

        }
        return 1;
    }

    cout<<"Houston, we have a problem: Not frames for debug..."<<endl;
    return 0;
    // images.push_back(imread(fn[i]));
}
// int main(){
//     save_rmss();
//     return 0;
// }

void bulid_V(Mat& homography, CvMat* V, int i);

void my_calibrate_camera( vector<Mat> selected_frames, vector<Point3f> real_centers){
    vector<P_Ellipse> control_points;
    Mat output_img_control_points, img_prep_out;
    
    
    
    int n_images = (int)selected_frames.size();
    
    CvMat *homographies[n_images]; // I don't know how to dynamically assign CvMat
    CvMat* V = cvCreateMat(2*n_images,6,CV_64FC1);
    CvMat* b = cvCreateMat(6,1,CV_64FC1); // elements of the absolute conic
    CvMat* ImgPoints = cvCreateMat(3,REAL_NUM_CTRL_PTS,CV_64FC1); // image coordinates
    CvMat* ObjectPoints = cvCreateMat(3,REAL_NUM_CTRL_PTS,CV_64FC1); // object coordinates
    
    
    for (int i=0; i<selected_frames.size(); i++) {
        int num_control_points = find_control_points(selected_frames[i],img_prep_out ,output_img_control_points,control_points);
        
        for(int j=0;j<REAL_NUM_CTRL_PTS;j++)
        {
            cvmSet(ImgPoints,0,j,control_points[j].x);
            cvmSet(ImgPoints,1,j,control_points[j].y);
            cvmSet(ImgPoints,2,j,1);
        }
        
        vector<Point2f> control_points_2d;
        
        control_points_2d.push_back(control_points[15].center());
        control_points_2d.push_back(control_points[16].center());
        control_points_2d.push_back(control_points[17].center());
        control_points_2d.push_back(control_points[18].center());
        control_points_2d.push_back(control_points[19].center());
        
        control_points_2d.push_back(control_points[10].center());
        control_points_2d.push_back(control_points[11].center());
        control_points_2d.push_back(control_points[12].center());
        control_points_2d.push_back(control_points[13].center());
        control_points_2d.push_back(control_points[14].center());
        
        control_points_2d.push_back(control_points[5].center());
        control_points_2d.push_back(control_points[6].center());
        control_points_2d.push_back(control_points[7].center());
        control_points_2d.push_back(control_points[8].center());
        control_points_2d.push_back(control_points[9].center());
        
        control_points_2d.push_back(control_points[0].center());
        control_points_2d.push_back(control_points[1].center());
        control_points_2d.push_back(control_points[2].center());
        control_points_2d.push_back(control_points[3].center());
        control_points_2d.push_back(control_points[4].center());
        
        Mat homography = findHomography(control_points_2d,real_centers);
        homographies[i] = cvCreateMat(3,3,CV_64FC1);
        
        cvmSet(homographies[i],0,0,homography.at<double>(0,0));
        cvmSet(homographies[i],1,0,homography.at<double>(0,1));
        cvmSet(homographies[i],2,0,homography.at<double>(0,2));
        cvmSet(homographies[i],0,1,homography.at<double>(1,0));
        cvmSet(homographies[i],1,1,homography.at<double>(1,1));
        cvmSet(homographies[i],2,1,homography.at<double>(1,2));
        cvmSet(homographies[i],0,2,homography.at<double>(2,0));
        cvmSet(homographies[i],1,2,homography.at<double>(2,1));
        cvmSet(homographies[i],2,2,homography.at<double>(2,2));
        cout<<"======================================="<<endl;
        cout<<homography<<" sss: "<<homography.size()<<endl;
        cout<<"-- "<<homography.at<float>(1,2)<<endl;
        bulid_V(homography,V,i);
    }
    //////////////////////////////////////////////
    // estimate b (elements of the absolute conic)
    CvMat* U = cvCreateMat(n_images*2,n_images*2,CV_64FC1);
    CvMat* D = cvCreateMat(n_images*2,6,CV_64FC1);
    CvMat* V2 = cvCreateMat(6,6,CV_64FC1);
    CvMat* B = cvCreateMat(3,3,CV_64FC1);
    cvSVD(V, D, U, V2, CV_SVD_V_T);
    
    cvmSet(b,0,0,cvmGet(V2,5,0));
    cvmSet(b,1,0,cvmGet(V2,5,1));
    cvmSet(b,2,0,cvmGet(V2,5,2));
    cvmSet(b,3,0,cvmGet(V2,5,3));
    cvmSet(b,4,0,cvmGet(V2,5,4));
    cvmSet(b,5,0,cvmGet(V2,5,5));
    // setup B
    cvmSet(B,0,0,cvmGet(b,0,0));
    cvmSet(B,0,1,cvmGet(b,1,0));
    cvmSet(B,1,0,cvmGet(b,1,0));
    cvmSet(B,1,1,cvmGet(b,2,0));
    cvmSet(B,0,2,cvmGet(b,3,0));
    cvmSet(B,2,0,cvmGet(b,3,0));
    cvmSet(B,1,2,cvmGet(b,4,0));
    cvmSet(B,2,1,cvmGet(b,4,0));
    cvmSet(B,2,2,cvmGet(b,5,0));
    // estimate intrinsic parameters
    CvMat* K = cvCreateMat(3,3,CV_64FC1);
    CvMat* Kinv = cvCreateMat(3,3,CV_64FC1);
    double vo = (cvmGet(B,0,1)*cvmGet(B,0,2)-cvmGet(B,0,0)*cvmGet(B,1,2))/(cvmGet(B,0,0)*cvmGet(B,1,1)-
                                                                           cvmGet(B,0,1)*cvmGet(B,0,1));
    double lamda = cvmGet(B,2,2)-(cvmGet(B,0,2)*cvmGet(B,0,2)+vo*(cvmGet(B,0,1)*cvmGet(B,0,2)-
                                                                  cvmGet(B,0,0)*cvmGet(B,1,2)))/cvmGet(B,0,0);
    double alpha = sqrt(lamda/cvmGet(B,0,0));
    double beta = sqrt(lamda*cvmGet(B,0,0)/(cvmGet(B,0,0)*cvmGet(B,1,1)-cvmGet(B,0,1)*cvmGet(B,0,1)));
    double gamma = -cvmGet(B,0,1)*alpha*alpha*beta/lamda;
    double uo = gamma*vo/beta-cvmGet(B,0,2)*alpha*alpha/lamda;
    cvmSet(K,0,0,alpha);
    cvmSet(K,0,1,gamma);
    cvmSet(K,0,2,uo);
    cvmSet(K,1,0,0);
    cvmSet(K,1,1,beta);
    cvmSet(K,1,2,vo);
    cvmSet(K,2,0,0);
    cvmSet(K,2,1,0);
    cvmSet(K,2,2,1);
    cvInvert(K,Kinv);
    
    // estimate extrinsic parameters
    CvMat* h1 = cvCreateMat(3,1,CV_64FC1);
    CvMat* h2 = cvCreateMat(3,1,CV_64FC1);
    CvMat* h3 = cvCreateMat(3,1,CV_64FC1);
    CvMat* r1 = cvCreateMat(3,1,CV_64FC1);
    CvMat* r2 = cvCreateMat(3,1,CV_64FC1);
    CvMat* r3 = cvCreateMat(3,1,CV_64FC1);
    CvMat* t = cvCreateMat(3,1,CV_64FC1);
    CvMat* Q = cvCreateMat(3,3,CV_64FC1);
    CvMat* R = cvCreateMat(3,3,CV_64FC1);
    CvMat* UU = cvCreateMat(3,3,CV_64FC1);
    CvMat* DD = cvCreateMat(3,3,CV_64FC1);
    CvMat* VV = cvCreateMat(3,3,CV_64FC1);
    CvMat* Vt = cvCreateMat(3,3,CV_64FC1);
    CvMat* Rt[n_images];
    
    float lamda1;
    for(int i=0;i<n_images;i++)
    {
        
        Rt[i] = cvCreateMat(3,4,CV_64FC1);
        // setup column vector h1,h2,h3 from H[i]
        cvmSet(h1,0,0,cvmGet(homographies[i],0,0));
        cvmSet(h1,1,0,cvmGet(homographies[i],1,0));
        cvmSet(h1,2,0,cvmGet(homographies[i],2,0));
        cvmSet(h2,0,0,cvmGet(homographies[i],0,1));
        cvmSet(h2,1,0,cvmGet(homographies[i],1,1));
        cvmSet(h2,2,0,cvmGet(homographies[i],2,1));
        cvmSet(h3,0,0,cvmGet(homographies[i],0,2));
        cvmSet(h3,1,0,cvmGet(homographies[i],1,2));
        cvmSet(h3,2,0,cvmGet(homographies[i],2,2));
        // estimate the rotation matrix R and the translation vector t
        cvMatMul(Kinv,h1,r1);
        cvMatMul(Kinv,h2,r2);
        cvMatMul(Kinv,h3,t);
        lamda1=1/sqrt(pow(cvmGet(r1,0,0),2)+pow(cvmGet(r1,1,0),2)+pow(cvmGet(r1,2,0),2));
        cvmSet(r1,0,0,cvmGet(r1,0,0)*lamda1);
        cvmSet(r1,1,0,cvmGet(r1,1,0)*lamda1);
        cvmSet(r1,2,0,cvmGet(r1,2,0)*lamda1);
        cvmSet(r2,0,0,cvmGet(r2,0,0)*lamda1);
        cvmSet(r2,1,0,cvmGet(r2,1,0)*lamda1);
        cvmSet(r2,2,0,cvmGet(r2,2,0)*lamda1);
        cvmSet(t,0,0,cvmGet(t,0,0)*lamda1);
        cvmSet(t,1,0,cvmGet(t,1,0)*lamda1);
        cvmSet(t,2,0,cvmGet(t,2,0)*lamda1);
        cvCrossProduct(r1,r2,r3);
        cvmSet(Q,0,0,cvmGet(r1,0,0));
        cvmSet(Q,1,0,cvmGet(r1,1,0));
        cvmSet(Q,2,0,cvmGet(r1,2,0));
        cvmSet(Q,0,1,cvmGet(r2,0,0));
        cvmSet(Q,1,1,cvmGet(r2,1,0));
        cvmSet(Q,2,1,cvmGet(r2,2,0));
        cvmSet(Q,0,2,cvmGet(r3,0,0));
        cvmSet(Q,1,2,cvmGet(r3,1,0));
        cvmSet(Q,2,2,cvmGet(r3,2,0));
        // Refine Q become orthogonal matrix R
        cvSVD(Q, DD, UU, VV, CV_SVD_V_T);
        cvMatMul(UU,VV,R);
        // Rt matrix
        cvmSet(Rt[i],0,0,cvmGet(R,0,0));
        cvmSet(Rt[i],1,0,cvmGet(R,1,0));
        cvmSet(Rt[i],2,0,cvmGet(R,2,0));
        cvmSet(Rt[i],0,1,cvmGet(R,0,1));
        cvmSet(Rt[i],1,1,cvmGet(R,1,1));
        cvmSet(Rt[i],2,1,cvmGet(R,2,1));
        cvmSet(Rt[i],0,2,cvmGet(R,0,2));
        cvmSet(Rt[i],1,2,cvmGet(R,1,2));
        cvmSet(Rt[i],2,2,cvmGet(R,2,2));
        cvmSet(Rt[i],0,3,cvmGet(t,0,0));
        cvmSet(Rt[i],1,3,cvmGet(t,1,0));
        cvmSet(Rt[i],2,3,cvmGet(t,2,0));
    }
    
    const char* reportfilename="/Users/davidchoqueluqueroman/Desktop/CURSOS-MASTER/IMAGENES/testOpencv/results/report.txt";
    FILE *report=fopen(reportfilename,"wt");
    
    
    fprintf(report,"================= Camera Calibration Report ====================\n");
    fprintf(report,"Number of images : %d\n",n_images);
    fprintf(report,"Image size : 640 X 480 pixel\n");
    fprintf(report,"Line detection method : the Hough transformation\n");
    fprintf(report,"Point detection method : intersecting lines acquired by the Hough transformation\n\n\n");
    // 2. Absolute Conic & intrinsic parameters
    fprintf(report,"================= Estimated absolute Conic ====================\n");
    for(int i=0;i<3;i++)
    {
        fprintf(report,"| %16.8f %16.8f %16.8f |\n",cvmGet(B,i,0),cvmGet(B,i,1),cvmGet(B,i,2));
    }
    
    fprintf(report,"\n================= Estimated Instrinsic Parameters ====================\n");
    fprintf(report,"vo=%f\n",vo);
    fprintf(report,"lamda=%f\n",lamda);
    fprintf(report,"alpha=%f\n",alpha);
    fprintf(report,"beta=%f\n",beta);
    fprintf(report,"gamma=%f\n",gamma);
    fprintf(report,"uo=%f\n",uo);
    fprintf(report,"| | | %16.8f %16.8f %16.8f |\n",cvmGet(K,0,0),cvmGet(K,0,1),cvmGet(K,0,2));
    fprintf(report,"| K | = | %16.8f %16.8f %16.8f |\n",cvmGet(K,1,0),cvmGet(K,1,1),cvmGet(K,1,2));
    fprintf(report,"| | | %16.8f %16.8f %16.8f |\n",cvmGet(K,2,0),cvmGet(K,2,1),cvmGet(K,2,2));
    fprintf(report,"\n");
    
    double dtdrefined[n_images], dtd[n_images];
    for(int i=0;i<n_images;i++)
    {
        // estimate residuals
    
        fprintf(report,"| | | %16.8f %16.8f %16.8f %16.8f |\n",cvmGet(Rt[i],0,0),cvmGet(Rt[i],0,1),cvmGet(Rt[i],0,2),cvmGet(Rt[i],0,3));
        fprintf(report,"| R | t | = | %16.8f %16.8f %16.8f %16.8f |\n",cvmGet(Rt[i],1,0),cvmGet(Rt[i],1,1),cvmGet(Rt[i],1,2),cvmGet(Rt[i],1,3));
        fprintf(report,"| | | %16.8f %16.8f %16.8f %16.8f |\n",cvmGet(Rt[i],2,0),cvmGet(Rt[i],2,1),cvmGet(Rt[i],2,2),cvmGet(Rt[i],2,3));
        dtd[i]= Residuals(Rt[i],K,ObjectPoints,ImgPoints);
        fprintf(report,"errors(dtd) = %f pixels\n",dtd[i]);
        fprintf(report,"\n");
        
    }
    if (1) // perform LM algoritm for each image seperately
    {
        /////////////////////////////////////////
        // Refine Parameters (for each camera)
        CvMat* estK = cvCreateMat(3,3,CV_64FC1);
//        imgfn=fopen(imagefilenames,"rt");
       
        
        for(int i=0;i<n_images;i++)
        {
            RefineCamera(Rt[i],K,estK,ObjectPoints,ImgPoints);
        }
        double improvementNumer=0;
        double improvementDenom=0;
        for(int i=0;i<n_images;i++){
            improvementNumer=improvementNumer+dtdrefined[i];
            improvementDenom=improvementDenom+dtd[i];
            dtdrefined[i]=Residuals(Rt[i],estK,ObjectPoints,ImgPoints);
            fprintf(report,"errors(dtd) = %f pixels\n",dtdrefined[i]);
            fprintf(report,"\n");
            
        }
        fprintf(report,"Improvement = ( sum of initial errors ) / (sum of refined errors)= %f.\n",improvementNumer/improvementDenom);
        
    }

    // 3. Extrinsic parameters
//    fprintf(report,"\n================= Estimated Extrinsic Parameters ====================\n");
////    imgfn=fopen(imagefilenames,"rt");
//    double dtd[39],dtdrefined[39];
//    for(int i=0;i<n_images;i++)
//    {
//        // estimate residuals
//
////        fscanf(imgfn,"%s",tempimgfn);
////        fprintf(report,"%s\n",tempimgfn);
//        fprintf(report,"| | | %16.8f %16.8f %16.8f %16.8f |\n",cvmGet(Rt[i],0,0),cvmGet(Rt[i],0,1),cvmGet(Rt[i],0,2),cvmGet(Rt[i],0,3));
//        fprintf(report,"| R | t | = | %16.8f %16.8f %16.8f %16.8f |\n",cvmGet(Rt[i],1,0),cvmGet(Rt[i],1,1),cvmGet(Rt[i],1,2),cvmGet(Rt[i],1,3));
//        fprintf(report,"| | | %16.8f %16.8f %16.8f %16.8f |\n",cvmGet(Rt[i],2,0),cvmGet(Rt[i],2,1),cvmGet(Rt[i],2,2),cvmGet(Rt[i],2,3));
//        dtd[i]= Residuals(Rt[i],K,ObjectPoints,ImgPoints);
//        fprintf(report,"errors(dtd) = %f pixels\n",dtd[i]);
//        fprintf(report,"\n");
//
//    }
    
}
void bulid_V(Mat& homography, CvMat* V, int i){
    float h11,h12,h13,h21,h22,h23,h31,h32,h33;
    h11 = homography.at<double>(0,0);
    h12 = homography.at<double>(0,1);
    h13 = homography.at<double>(0,2);
    h21 = homography.at<double>(1,0);
    h22 = homography.at<double>(1,1);
    h23 = homography.at<double>(1,2);
    h31 = homography.at<double>(2,0);
    h32 = homography.at<double>(2,1);
    h33 = homography.at<double>(2,2);
    
    cvmSet(V,2*i,0,h11*h21);
    cvmSet(V,2*i,1,h11*h22+h12*h21);
    cvmSet(V,2*i,2,h12*h22);
    cvmSet(V,2*i,3,h13*h21+h11*h23);
    cvmSet(V,2*i,4,h13*h22+h12*h23);
    cvmSet(V,2*i,5,h13*h23);
    cvmSet(V,2*i+1,0,h11*h11-h21*h21);
    cvmSet(V,2*i+1,1,h11*h12+h12*h11-h21*h22-h22*h21);
    cvmSet(V,2*i+1,2,h12*h12-h22*h22);
    cvmSet(V,2*i+1,3,h13*h11+h11*h13-h23*h21-h21*h23);
    cvmSet(V,2*i+1,4,h13*h12+h12*h13-h23*h22-h22*h23);
    cvmSet(V,2*i+1,5,h13*h13-h23*h23);
    
    
}
int main()
{

    //string path_data = "/Users/davidchoqueluqueroman/Desktop/CURSOS-MASTER/IMAGENES/testOpencv/data/";
    string video_file = PATH_DATA+"cam1/anillos.mp4";
    //    string video_file = PATH_DATA+"cam2/anillos.avi";


    VideoCapture cap;
    cap.open(video_file);
    if ( !cap.isOpened() ){
        cout << "Cannot open the video file. \n";
        return -1;
    }
    //initial frame for get size of frames
    Mat frame;
    cap.read(frame);
    int h = frame.cols;
    int w = frame.rows;
    Size frameSize(h,w);
    vector<float> rmss;

    cout<<"h: "<<h<<", w: "<<w<<" size: "<<frame.size()<<endl;

    /********************** choose frames *****************************/
    cout << "Selecting frames ... "<< endl;
    vector<Mat> selected_frames;
    // int delay_time = 55;
    //    select_frames_by_time(cap, selected_frames,delay_time,NUM_FRAMES_FOR_CALIBRATION);
    int n_frames = 60;
    select_frames(cap,selected_frames,frameSize,n_frames);
    //VideoCapture& cap, vector<Mat>& out_frames_selected, int w, int h,int n_quads_rows,int num_quads_cols

    cout << "Creating ideal image ... "<< endl;
    vector<Point3f> real_centers;
    create_real_pattern(h,w, real_centers);
    
    my_calibrate_camera(selected_frames, real_centers);
    //    load_object_points(h,w, real_centers);

//    /*************************first calibration**********************************/
//    vector<vector<Point2f>> imagePoints;
//    Mat cameraMatrix, cameraMatrix_first;
//    Mat distCoeffs = Mat::zeros(8, 1, CV_64F);
//    Mat distCoeffs_first = Mat::zeros(8, 1, CV_64F);
//    vector<P_Ellipse> control_points;
//    int num_control_points=0;
//    double rms=-1;
//    double rms_first=-1;
//    Mat output_img_control_points, img_prep_out;
//
//    for (int i=0; i<selected_frames.size(); i++) {
//        control_points.clear();
//        num_control_points = find_control_points(selected_frames[i],img_prep_out ,output_img_control_points,control_points);
//
//        //    imshow("preproces img"+to_string(i), img_prep_out);
//        //    imshow("img"+to_string(i), output_img_control_points);
//        save_frame(PATH_DATA_FRAMES+"iteration0/preprocesed/","iter-"+to_string(i),img_prep_out);
//        save_frame(PATH_DATA_FRAMES+"iteration0/detected/","iter-"+to_string(i),output_img_control_points);
//
//        if(num_control_points == REAL_NUM_CTRL_PTS){
//            vector<Point2f> buffer = ellipses2Points(control_points) ;
//            imagePoints.push_back(buffer);
//        }
//        if(waitKey(30) == 27){
//            break;
//        }
//    }
//
//    if(imagePoints.size()==selected_frames.size()){
//        cout << "First calibration... \n";
//        rms = calibrate_camera(frameSize, cameraMatrix, distCoeffs, imagePoints);
//        cout << "cameraMatrix " << cameraMatrix << endl;
//        cout << "distCoeffs " << distCoeffs << endl;
//        cout << "rms: " << rms << endl;
//        rms_first = rms;
//        rmss.push_back(rms);
//        cameraMatrix_first = cameraMatrix.clone();
//        distCoeffs_first = distCoeffs;
//        //Input : - Rt : 3X4 matrix (R|t)
//        CvMat* Rt = cvCreateMat(3,4,CV_64FC1);
////        cvmSet(Rt,0,0,cvmGet(Rt,0,0));
//
//    }
    //
    /************************ Points Refinement **********************************/
//    vector<Mat> fronto_images;
//    int No_ITER = 25;
//    for(int i=0; i<No_ITER;i++){
//        fronto_images.clear();
//        imagePoints.clear();
//        fronto_parallel_images(selected_frames,fronto_images,frameSize,real_centers, cameraMatrix, distCoeffs,imagePoints,i);
//
//        /************************ Calibrate camera **********************************/
//        // cameraMatrix.release();
//        // distCoeffs.release();
//        cout << "saved frames: " << imagePoints.size() << endl;
//        if(imagePoints.size() > 0){
//            cout << "REFINEMENT ("<<i<<")"<<endl;
//            rms = calibrate_camera(frameSize, cameraMatrix, distCoeffs, imagePoints);
//            rmss.push_back(rms);
//            cout << "cameraMatrix " << cameraMatrix << endl;
//            cout << "distCoeffs " << distCoeffs << endl;
//            cout << "rms: " << rms << endl;
//        }
//    }
//    save_rmss(selected_frames.size(),rmss);

    //    debug_images_fronto();

    // VideoCapture cap2;
    // cap2.open(video_file);

    // namedWindow("Image View", CV_WINDOW_AUTOSIZE);

    // if ( !cap.isOpened() )
    //     cout << "Cannot open the video file. \n";
    // while(1){

    //     Mat frame2;
    //     cap2>>frame2;

    //     if(frame2.empty())
    //         break;

    // //First
    //     Mat undistorted_image_first, map1_first, map2_first;
    //     initUndistortRectifyMap(cameraMatrix_first, distCoeffs_first, Mat(),
    //     getOptimalNewCameraMatrix(cameraMatrix_first, distCoeffs_first, frameSize, 1, frameSize, 0),
    //     frameSize, CV_16SC2, map1_first, map2_first);
    //     remap(frame2, undistorted_image_first, map1_first, map2_first, INTER_LINEAR);

    //  //Last
    //     Mat undistorted_image, map1, map2, img_fronto_parallel;
    //     initUndistortRectifyMap(cameraMatrix, distCoeffs, Mat(),
    //     getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, frameSize, 1, frameSize, 0),
    //     frameSize, CV_16SC2, map1, map2);
    //     remap(frame2, undistorted_image, map1, map2, INTER_LINEAR);


    //     int n_ctrl_points_undistorted = find_control_points(undistorted_image, output_img_control_points,control_points);

    //     if(n_ctrl_points_undistorted == REAL_NUM_CTRL_PTS){
    //         //cout << " ====================================================== "<< endl;
    //         vector<Point2f> control_points2f = ellipses2Points(control_points);
    //         //        plot_control_points(output_img_control_points,output_img_control_points,control_points_centers,yellow);
    //         /**************** unproject*********************/
    //         // cout << "Unproject image ... "<< endl;
    //         // vector<Point2f> control_points_2d = ellipses2Points(control_points);

    //         vector<Point2f> control_points_2d;
    //         control_points_2d.push_back(control_points[15].center());
    //         control_points_2d.push_back(control_points[16].center());
    //         control_points_2d.push_back(control_points[17].center());
    //         control_points_2d.push_back(control_points[18].center());
    //         control_points_2d.push_back(control_points[19].center());

    //         control_points_2d.push_back(control_points[10].center());
    //         control_points_2d.push_back(control_points[11].center());
    //         control_points_2d.push_back(control_points[12].center());
    //         control_points_2d.push_back(control_points[13].center());
    //         control_points_2d.push_back(control_points[14].center());

    //         control_points_2d.push_back(control_points[5].center());
    //         control_points_2d.push_back(control_points[6].center());
    //         control_points_2d.push_back(control_points[7].center());
    //         control_points_2d.push_back(control_points[8].center());
    //         control_points_2d.push_back(control_points[9].center());

    //         control_points_2d.push_back(control_points[0].center());
    //         control_points_2d.push_back(control_points[1].center());
    //         control_points_2d.push_back(control_points[2].center());
    //         control_points_2d.push_back(control_points[3].center());
    //         control_points_2d.push_back(control_points[4].center());

    //         Mat homography = findHomography(control_points_2d,real_centers);
    //         Mat inv_homography = findHomography(real_centers,control_points_2d);

    //         img_fronto_parallel = undistorted_image.clone();
    //         warpPerspective(undistorted_image, img_fronto_parallel, homography, frame.size());

    //     }
    //     else{
    //         cout << "NOT FOUND ... "<< endl;
    //     }

    //     //imshow("Image View", rview);
    //     //equalizeHist(img_fronto_parallel, img_fronto_parallel);
    //     ShowManyImages("resultado", 2, 3, rms_first, rms, cameraMatrix, 4, frame2, img_fronto_parallel, undistorted_image_first,undistorted_image);
    //         // waitKey(2);
    //     if(waitKey(1) == 27)
    //     {
    //         break;
    //     }
    // }

    return 0;
}



