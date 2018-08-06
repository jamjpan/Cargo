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

simuated_annealing.o: ../../lib/libcargo.a simuated_annealing.h simuated_annealing.cpp
	$(CXX) $(CFLAGS) simuated_annealing.cpp

clean:
	rm *.o simuated_annealing
