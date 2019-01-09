all:
	g++ -ggdb main.cpp ellipse.cpp Line.cpp -o main -std=c++11 `pkg-config --cflags --libs opencv`
exec:
	g++ -ggdb camera_calibration.cpp -o chess -std=c++11 `pkg-config --cflags --libs opencv`
	
	