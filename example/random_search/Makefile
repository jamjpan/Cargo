CXX = g++
CFLAGS = -Wall -Wextra -std=c++11 -O3 -g -c -I../../include -o $@
LFLAGS = -Wall
PTHREAD = -pthread -lrt
LDL = -ldl
METIS = -L/usr/local/lib -lmetis
CARGO = -L../../lib -lcargo

OBJS = random_search.o

all: $(OBJS)
	$(CXX) $(LFLAGS) $(OBJS) $(CARGO) $(PTHREAD) $(METIS) $(LDL) -o random_search

random_search.o: ../../lib/libcargo.a random_search.h random_search.cpp
	$(CXX) $(CFLAGS) random_search.cpp

clean:
	rm *.o random_search
