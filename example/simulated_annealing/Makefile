CXX = g++
CFLAGS = -Wall -Wextra -std=c++11 -O3 -g -c -I../../include -o $@
LFLAGS = -Wall
PTHREAD = -pthread -lrt
LDL = -ldl
METIS = -L/usr/local/lib -lmetis
CARGO = -L../../lib -lcargo

OBJS = simulated_annealing.o

all: $(OBJS)
	$(CXX) $(LFLAGS) $(OBJS) $(CARGO) $(PTHREAD) $(METIS) $(LDL) -o simulated_annealing

simulated_annealing.o: ../../lib/libcargo.a simulated_annealing.h simulated_annealing.cpp
	$(CXX) $(CFLAGS) simulated_annealing.cpp

clean:
	rm *.o simulated_annealing
