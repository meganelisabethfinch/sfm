CFLAGS = -std=c++17 -I./headers
LDFLAGS = `pkg-config --cflags --libs opencv4`
SRC = ./src/*.cpp