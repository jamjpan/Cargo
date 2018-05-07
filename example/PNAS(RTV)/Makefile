CXX = g++
CFLAGS = -Wall -Wextra -std=c++11 -O3 -g -c -I../../include -o $@
LFLAGS = -Wall
PTHREAD = -pthread -lrt
METIS = -L/usr/local/lib -lmetis
CARGO = -L../../lib -lcargo

OBJS = PNAS.o

all: $(OBJS)
	$(CXX) $(LFLAGS) $(OBJS) $(CARGO) $(PTHREAD) $(METIS) -o test

PNAS.o: PNAS.cpp
	$(CXX) $(CFLAGS) PNAS.cpp

clean:
	rm *.o test