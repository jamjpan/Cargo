CXX = g++
CFLAGS = -Wall -Wextra -std=c++11 -O0 -g -c -I../../include -o $@
LFLAGS = -Wall
PTHREAD = -pthread -lrt
LDL = -ldl
METIS = -L/usr/local/lib -lmetis
CARGO = -L../../lib -lcargo

OBJS = grasp.o

all: $(OBJS)
	$(CXX) $(LFLAGS) $(OBJS) $(CARGO) $(PTHREAD) $(METIS) $(LDL) -o grasp

grasp.o: ../../lib/libcargo.a grasp.h grasp.cpp
	$(CXX) $(CFLAGS) grasp.cpp

clean:
	rm *.o grasp
