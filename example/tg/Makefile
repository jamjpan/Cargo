CXX = g++
CFLAGS = -Wall -Wextra -std=c++11 -O3 -g -c -I../../include -Iinclude -o $@
LFLAGS = -Wall -fopenmp
PTHREAD = -pthread -lrt
LDL = -ldl
GLPK = -l:libglpk.a
METIS = -lmetis
CARGO = -L../../lib -lcargo

OBJS = tg.o

all: $(OBJS)
	$(CXX) $(LFLAGS) $(OBJS) $(CARGO) $(PTHREAD) $(METIS) $(GLPK) $(LDL) -fopenmp -o tg

tg.o: ../../lib/libcargo.a tg.h tg.cpp
	$(CXX) $(CFLAGS) -fopenmp tg.cpp

clean:
	rm *.o tg
