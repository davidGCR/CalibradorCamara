all:
	g++ -ggdb main.cpp ellipse.cpp -o main -std=c++11 `pkg-config --cflags --libs opencv`
