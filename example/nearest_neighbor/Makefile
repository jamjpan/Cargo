CXX = g++
CFLAGS = -Wall -Wextra -std=c++11 -O0 -g -c -I../../include -o $@
LFLAGS = -Wall
PTHREAD = -pthread -lrt
LDL = -ldl
METIS = -L/usr/local/lib -lmetis
CARGO = -L../../lib -lcargo

OBJS = nearest_neighbor.o

all: $(OBJS)
	$(CXX) $(LFLAGS) $(OBJS) $(CARGO) $(PTHREAD) $(METIS) $(LDL) -o nearest_neighbor

nearest_neighbor.o: ../../lib/libcargo.a nearest_neighbor.h nearest_neighbor.cpp
	$(CXX) $(CFLAGS) nearest_neighbor.cpp

clean:
	rm *.o nearest_neighbor
