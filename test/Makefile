CXX = g++
CFLAGS = -Wall -Wextra -std=c++11 -O3 -g -c -I../include -o $@
LFLAGS = -Wall
PTHREAD = -lpthread
METIS = -L/usr/local/lib -lmetis
CARGO = -L/home/jpan/devel/Cargo/lib -lcargo
#-------------------------------------------------------------------------------
OBJECTS = \
		  build/main.o \
		  build/simulator_test.o
all: $(OBJECTS)
	$(CXX) $(LFLAGS) $(OBJECTS) $(CARGO) $(PTHREAD) $(METIS) -o run_tests
#-------------------------------------------------------------------------------
$(OBJECTS): | build

build/main.o: \
	src/main.cpp
	$(CXX) $(CFLAGS) src/main.cpp

build/simulator_test.o: \
	src/simulator_test.cpp
	$(CXX) $(CFLAGS) src/simulator_test.cpp

build:
	mkdir -p build

clean:
	rm -rf build/ run_tests
