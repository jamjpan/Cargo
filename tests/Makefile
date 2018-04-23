CXX = g++
CFLAGS = -Wall -Wextra -std=c++11 -O3 -g -c
LFLAGS = -Wall
PTHREAD = -lpthread
METIS = -L/usr/local/lib -lmetis
CARGO = -L/home/jpan/devel/Cargo/lib -lcargo
#-------------------------------------------------------------------------------
OBJECTS = main.o SimTest.o
all: $(OBJECTS)
	$(CXX) $(LFLAGS) $(OBJECTS) $(CARGO) $(PTHREAD) $(METIS) -o run_tests
#-------------------------------------------------------------------------------
main.o: main.cpp
	$(CXX) $(CFLAGS) main.cpp

SimTest.o: SimTest.cpp
	$(CXX) $(CFLAGS) SimTest.cpp

clean:
	rm -rf *.o run_tests
