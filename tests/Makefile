CXX = g++
CFLAGS = -Wall -Wextra -std=c++11 -O3 -g -c
LFLAGS = -Wall
PTHREAD = -lpthread
METIS = -L/usr/local/lib -lmetis
OBJS = main.o Sim.o common.o GTree.o NearestNeighbor.o

test: $(OBJS)
	$(CXX) $(LFLAGS) $(OBJS) $(PTHREAD) $(METIS) -o test

main.o: main.cpp Sim.h
	$(CXX) $(CFLAGS) main.cpp

Sim.o: Sim.cpp Sim.h common.h gtree/GTree.h
	$(CXX) $(CFLAGS) Sim.cpp

common.o: common.cpp common.h
	$(CXX) $(CFLAGS) common.cpp

GTree.o: gtree/GTree.cc gtree/GTree.h
	$(CXX) $(CFLAGS) gtree/GTree.cc

NearestNeighbor.o: NearestNeighbor.cpp
	$(CXX) $(CFLAGS) NearestNeighbor.cpp

clean:
	rm -rf *.o test
