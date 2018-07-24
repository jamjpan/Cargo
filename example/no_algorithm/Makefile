CXX = g++
CFLAGS = -Wall -Wextra -std=c++11 -O3 -g -c -I../../include -o $@
LFLAGS = -Wall
PTHREAD = -pthread -lrt
LDL = -ldl
METIS = -L/usr/local/lib -lmetis
CARGO = -L../../lib -lcargo

OBJS = no_algorithm.o

all: $(OBJS)
	$(CXX) $(LFLAGS) $(OBJS) $(CARGO) $(PTHREAD) $(METIS) $(LDL) -o no_algorithm

no_algorithm.o: ../../lib/libcargo.a no_algorithm.h no_algorithm.cpp
	$(CXX) $(CFLAGS) no_algorithm.cpp

clean:
	rm *.o no_algorithm
