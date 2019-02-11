all:
	g++ -ggdb main.cpp class/cameraCalib.h class/utils.h class/iterativeCalibFunctions.h class/ellipse.h class/Line.h class/constants.h class/quadrant.h class/controlPointDetector.h class/display.h class/preprocessing.h -o main -std=c++11 `pkg-config --cflags --libs opencv`
exec:
	g++ -ggdb camera_calibration.cpp -o chess -std=c++11 `pkg-config --cflags --libs opencv`
	
