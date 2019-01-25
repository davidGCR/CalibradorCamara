#include <iostream>
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"

using namespace std;
using namespace cv;

void thresholdIntegral(cv::Mat &inputMat, cv::Mat &outputMat)
{
    // accept only char type matrices
    CV_Assert(!inputMat.empty());
    CV_Assert(inputMat.depth() == CV_8U);
    CV_Assert(inputMat.channels() == 1);
    CV_Assert(!outputMat.empty());
    CV_Assert(outputMat.depth() == CV_8U);
    CV_Assert(outputMat.channels() == 1);
    
    // rows -> height -> y
    int nRows = inputMat.rows;
    // cols -> width -> x
    int nCols = inputMat.cols;
    
    // create the integral image
    cv::Mat sumMat;
    cv::integral(inputMat, sumMat);
    
    CV_Assert(sumMat.depth() == CV_32S);
    CV_Assert(sizeof(int) == 4);
    
    int S = MAX(nRows, nCols)/16;
    double T = 0.1;
    
    // perform thresholding
    int s2 = S/2;
    int x1, y1, x2, y2, count, sum;
    
    // CV_Assert(sizeof(int) == 4);
    int *p_y1, *p_y2;
    uchar *p_inputMat, *p_outputMat;
    
    for( int i = 0; i < nRows; ++i)
    {
        y1 = i-s2;
        y2 = i+s2;
        
        if (y1 < 0){
            y1 = 0;
        }
        if (y2 >= nRows) {
            y2 = nRows-1;
        }
        
        p_y1 = sumMat.ptr<int>(y1);
        p_y2 = sumMat.ptr<int>(y2);
        p_inputMat = inputMat.ptr<uchar>(i);
        p_outputMat = outputMat.ptr<uchar>(i);
        
        for ( int j = 0; j < nCols; ++j)
        {
            // set the SxS region
            x1 = j-s2;
            x2 = j+s2;
            
            if (x1 < 0) {
                x1 = 0;
            }
            if (x2 >= nCols) {
                x2 = nCols-1;
            }
            
            count = (x2-x1)*(y2-y1);
            
            // I(x,y)=s(x2,y2)-s(x1,y2)-s(x2,y1)+s(x1,x1)
            sum = p_y2[x2] - p_y1[x2] - p_y2[x1] + p_y1[x1];
            
            if ((int)(p_inputMat[j] * count) < (int)(sum*(1.0-T)))
                p_outputMat[j] = 0;
            else
                p_outputMat[j] = 255;
        }
    }
}

void segmentar(Mat &in, Mat &out, Mat adapThresh, int w, int h ) {

    int **intImg = new int*[w];
    for (int i = 0; i < w; i++) {
        intImg[i] = new int[h];
    }

    int sum = 0;
    for (int i = 0; i < w; i++) {
        sum = 0;
        for (int j = 0; j < h; j++) {
            unsigned char & pixel = in.at<unsigned char >(i, j);
            sum += pixel;
            if (i == 0) {
                intImg[i][j] = sum;
            } else {
                intImg[i][j] = intImg[i - 1][j] + sum;
            }
        }
    }
    int s = w / 8;
    int t = 15;
    int x1, x2, y1, y2;
    int count;
    for (int i = 0; i < w; i++) {
        for (int j = 0; j < h; j++) {
            x1 = i - s / 2;
            x2 = i + s / 2;
            y1 = j - s / 2;
            y2 = j + s / 2;

            if (x1 > 0 && x2 < w && y1 > 0 && y2 < h) {
                count = (x2 - x1) * (y2 - y1);
                sum = intImg[x2][y2] - intImg[x2][y1 - 1] - intImg[x1 - 1][y2] + intImg[x1 - 1][y1 - 1];
                unsigned char & pixel = in.at<unsigned char >(i, j);
                unsigned char & pixel_o = out.at<unsigned char >(i, j);
                if (pixel * count <= sum * (100 - t) / 100) {
                    pixel_o = 0;
                } else {
                    pixel_o = 255;
                }
            }else{
                unsigned char & pixel_o = out.at<unsigned char >(i, j);
                pixel_o = adapThresh.at<unsigned char>(i, j);
            }

        }
    }
    for (int i = 0; i < w; i++) {
        delete intImg[i];
    }
    delete intImg;
}


void preprocessing_frame2(Mat& frame, Mat& frame_output){
    Mat blur;
    Mat frame_gray;
    Mat frame_thresholding;
    Mat integralImage;
    cvtColor( frame,frame_gray, COLOR_BGR2GRAY);
    int w = frame.rows;
	int h = frame.cols;

    // cout<<"chanels gray : "<<frame_gray.channels()<<endl;
    
    cv::adaptiveThreshold(frame_gray, frame_thresholding, 255, ADAPTIVE_THRESH_GAUSSIAN_C, THRESH_BINARY, 41, 12);
    segmentar(frame_gray,frame_gray,frame_thresholding, w, h );
    frame_output = frame_gray;
}

void preprocessing_frame(Mat* frame, Mat* frame_output){
    Mat blur;
    Mat frame_gray;
    Mat frame_thresholding;
    Mat integralImage;
    //namedWindow("New frame", 1 );
    cvtColor( *frame,frame_gray, COLOR_BGR2GRAY );
    GaussianBlur( frame_gray, blur, Size(7,7),0);
    //threshold(src_gray,detected_edges,0,255,THRESH_BINARY+THRESH_OTSU);
    //integral(blur, integralImage);
    //frame_thresholding = Mat::zeros(blur.size(), CV_8UC1);
    blur.copyTo(frame_thresholding);
    thresholdIntegral(blur, frame_thresholding);
    //adaptiveThreshold(frame_thresholding, frame_thresholding, 255, ADAPTIVE_THRESH_GAUSSIAN_C,CV_THRESH_BINARY,125,20);
    
    //imshow("New frame",frame_thresholding);
    (*frame_output) = frame_thresholding;
    // cout<<"preprocesada: "<<frame_thresholding.size<<endl;
}
