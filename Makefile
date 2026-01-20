TARGET=nrc2.out
all :
	g++-4.8 -m32 -o nrc2.out src/*.cpp -I./include -L./lib -lNexRob -lpthread -lm -ldl -lrt -std=c++11

clean :
	rm $(TARGET) $(objects)
