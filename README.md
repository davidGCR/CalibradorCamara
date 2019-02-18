# Camera Calibration with Rings Pattern and  OpenCV
A method for camera calibration using rings pattern an OpenCV, based on:
- [1] Zhengyou Zhang,“A Flexible New Technique for Camera Calibration”,2000.
- [2] Ankur Datta, “Accurate Camera Calibration using Iterative Refinement of Control Points”, 2009.
- [3] Prakash, Camera Calibration using Adaptive Segmentation and Ellipse Fitting for Localizing Control Points”, 2012.
- [4] Open Source Computer Vision,“Camera calibration With OpenCV“.
- [5] Burger, “Zhang’s Camera Calibration Algorithm:In-Depth Tutorial and Implementation“.

### Web Page

[[https://davidgcr.github.io/CalibradorCamara/]]

### Video Demo
<iframe width="800" height="500" src="https://www.youtube.com/embed/f9BiisoXgWY" frameborder="0" allow="accelerometer; autoplay; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>



### Students: 
- Diego Javier Quispe 
- David Choqueluque Roman


# Getting Started
Requirements:
- OS: Ubuntu
- Opencv 3.2.0 +
- Compiler gcc, g++

To run the code:

- Clone o download the source code.
- Download the test videos from : https://drive.google.com/open?id=1sflgm5tMdCjtriVWZmNSOreaXoliZ-ot
- Save the videos on "data/" on repository folder.
- Open a terminal in the repository folder.
- Write "make" in the console.
- Run with the command: ./main

# 1. Pipeline
![Alt text](https://github.com/davidGCR/CalibradorCamara/blob/master/results/images/pipeline_calibrador.png?raw=true "Title")

# 2. General View
![Alt text](https://github.com/davidGCR/CalibradorCamara/blob/master/results/images/GeneralPicture.png?raw=true "Title")

# 3. Results
## 3.1 Pre-processing, detection and tracking
![Alt text](https://github.com/davidGCR/CalibradorCamara/blob/master/results/images/pre-detect-track.png?raw=true "Title")

## 3.2 Distortion correction
![Alt text](https://github.com/davidGCR/CalibradorCamara/blob/master/results/images/undistord_remap.png?raw=true "Title")

## 3.3 Time and Accuracy test
Our method improve the time and accuracy in pattern detection.
![Alt text](https://github.com/davidGCR/CalibradorCamara/blob/master/results/images/time-acc.png?raw=true "Title")

## 3.4 Compare with other  OpenCV calibrations
![Alt text](https://github.com/davidGCR/CalibradorCamara/blob/master/results/images/c1-c2-accuracy.png?raw=true "Title")

## 3.5 Control points refinements
![Alt text](https://github.com/davidGCR/CalibradorCamara/blob/master/results/images/refinement.png?raw=true "Title")

## 3.6 Refinement Comparison
![Alt text](https://github.com/davidGCR/CalibradorCamara/blob/master/results/images/refinement-plot-c1.png?raw=true "Title")
![Alt text](https://github.com/davidGCR/CalibradorCamara/blob/master/results/images/refinement-plot-c2.png?raw=true "Title")

## Zhang's Algorithm (Implementation)
![Alt text](https://github.com/davidGCR/CalibradorCamara/blob/master/results/images/zhang-c1.png?raw=true "Title")
![Alt text](https://github.com/davidGCR/CalibradorCamara/blob/master/results/images/zhang-c2.png?raw=true "Title")

