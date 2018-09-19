CXX = g++
CFLAGS = -Wall -Wextra -std=c++11 -O0 -g -c -I../../include -o $@
LFLAGS = -Wall
PTHREAD = -pthread -lrt
LDL = -ldl
METIS = -L/usr/local/lib -lmetis
CARGO = -L../../lib -lcargo

OBJS = genetic.o

all: $(OBJS)
	$(CXX) $(LFLAGS) $(OBJS) $(CARGO) $(PTHREAD) $(METIS) $(LDL) -o genetic

genetic.o: ../../lib/libcargo.a genetic.h genetic.cpp
	$(CXX) $(CFLAGS) genetic.cpp

clean:
	rm *.o genetic
