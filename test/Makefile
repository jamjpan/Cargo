CXX = g++
CFLAGS = -Wall -Wextra -std=c++11 -O3 -g -c -I../include
LFLAGS = -Wall
PTHREAD = -lpthread
METIS = -L/usr/local/lib -lmetis
CARGO = -L/home/jpan/devel/Cargo/lib -lcargo
#-------------------------------------------------------------------------------
OBJECTS = main.o simulator_test.o
all: $(OBJECTS)
	$(CXX) $(LFLAGS) $(OBJECTS) $(CARGO) $(PTHREAD) $(METIS) -o run_tests
#-------------------------------------------------------------------------------
main.o: src/main.cpp
	$(CXX) $(CFLAGS) src/main.cpp

simulator_test.o: src/simulator_test.cpp
	$(CXX) $(CFLAGS) src/simulator_test.cpp

clean:
	rm -rf *.o run_tests
