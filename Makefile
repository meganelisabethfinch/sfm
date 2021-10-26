CFLAGS = -std=c++17 -I./headers
LDFLAGS = `pkg-config --cflags --libs opencv4`
SRC = ./src/*.cpp

app:
	g++ $(LDFLAGS) $(CFLAGS) $(SRC) -o ./build/app

run:
	./build/app -i "/Users/meganfinch/Documents/Part II Project/Code/data/lion"