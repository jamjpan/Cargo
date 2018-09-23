CXX = g++
CFLAGS = -Wall -Wextra -std=c++11 -O0 -g -c -I../../include -o $@
LFLAGS = -Wall
PTHREAD = -pthread -lrt
LDL = -ldl
METIS = -L/usr/local/lib -lmetis
CARGO = -L../../lib -lcargo

OBJS = population_annealing_near.o

all: $(OBJS)
	$(CXX) $(LFLAGS) $(OBJS) $(CARGO) $(PTHREAD) $(METIS) $(LDL) -o population_annealing_near

population_annealing_near.o: ../../lib/libcargo.a population_annealing_near.h population_annealing_near.cpp
	$(CXX) $(CFLAGS) population_annealing_near.cpp

clean:
	rm *.o population_annealing_near
