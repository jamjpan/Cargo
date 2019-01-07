CXX = g++
CFLAGS = -Wall -Wextra -std=c++11 -O3 -g -c -I../../include -Iinclude -o $@
LFLAGS = -Wall
PTHREAD = -pthread -lrt
LDL = -ldl
METIS = -L/usr/local/lib -lmetis
CARGO = -L../../lib -lcargo

OBJS = kinetic_tree.o treeTaxiPath.o

all: $(OBJS)
	$(CXX) $(LFLAGS) $(OBJS) $(CARGO) $(PTHREAD) $(METIS) $(LDL) -o kinetic_tree

kinetic_tree.o: ../../lib/libcargo.a kinetic_tree.h kinetic_tree.cpp
	$(CXX) $(CFLAGS) kinetic_tree.cpp

treeTaxiPath.o: include/treeTaxiPath.h src/treeTaxiPath.cpp
	$(CXX) $(CFLAGS) src/treeTaxiPath.cpp

clean:
	rm *.o kinetic_tree

