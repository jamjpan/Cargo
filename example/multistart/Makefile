CXX = g++
CFLAGS = -Wall -Wextra -std=c++11 -O0 -g -c -I../../include -o $@
LFLAGS = -Wall
PTHREAD = -pthread -lrt
LDL = -ldl
METIS = -L/usr/local/lib -lmetis
CARGO = -L../../lib -lcargo

OBJS = multistart.o

all: $(OBJS)
	$(CXX) $(LFLAGS) $(OBJS) $(CARGO) $(PTHREAD) $(METIS) $(LDL) -o multistart

multistart.o: ../../lib/libcargo.a multistart.h multistart.cpp
	$(CXX) $(CFLAGS) multistart.cpp

clean:
	rm *.o multistart
