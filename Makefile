CXX = g++
CFLAGS = -Wall -Wextra -std=c++11 -O3 -g -c
LFLAGS = -Wall
PTHREAD = -lpthread
METIS = -L/usr/local/lib -lmetis
#-------------------------------------------------------------------------------
OBJECTS = Sim.o GTree.o common.o
lib/cargo.a: $(OBJECTS)
	ar rcs $@ $^
#-------------------------------------------------------------------------------
Sim.o: src/Sim.cpp src/Sim.h src/common.h src/gtree/GTree.h
	$(CXX) $(CFLAGS) src/Sim.cpp

GTree.o: src/gtree/GTree.cc src/gtree/GTree.h
	$(CXX) $(CFLAGS) src/gtree/GTree.cc

common.o: src/common.cpp src/common.h
	$(CXX) $(CFLAGS) src/common.cpp

clean:
	rm -rf *.o lib/cargo.a
