all:
	g++ -ggdb main.cpp -o main `pkg-config --cflags --libs opencv`
