all:
	g++ -ggdb test1.cpp -o main `pkg-config --cflags --libs opencv`
