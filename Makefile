all:
	g++ -ggdb main.cpp ellipse.cpp -o main `pkg-config --cflags --libs opencv`
