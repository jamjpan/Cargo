CXX = g++
CFLAGS = -Wall -Wextra -std=c++11 -O3 -g -c -I../../include -o $@
LFLAGS = -Wall
PTHREAD = -pthread -lrt
LDL = -ldl
METIS = -L/usr/local/lib -lmetis
CARGO = -L../../lib -lcargo

OBJS = greedy_insertion.o

all: $(OBJS)
	$(CXX) $(LFLAGS) $(OBJS) $(CARGO) $(PTHREAD) $(METIS) $(LDL) -o greedy_insertion

greedy_insertion.o: ../../lib/libcargo.a greedy_insertion.h greedy_insertion.cpp
	$(CXX) $(CFLAGS) greedy_insertion.cpp

clean:
	rm *.o greedy_insertion
