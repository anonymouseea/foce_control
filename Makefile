TARGET := nrc2.out
CXX := g++
SOURCES := $(wildcard src/*.cpp)

all: $(TARGET)

$(TARGET): $(SOURCES)
	$(CXX) -m32 -no-pie -std=c++11 \
		-o $(TARGET) $(SOURCES) \
		-I./include \
		-L./lib \
		-lNexRob -lpthread -lm -ldl -lrt

clean:
	rm -f $(TARGET)
