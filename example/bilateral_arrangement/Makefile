CXX = g++
CFLAGS = -Wall -Wextra -std=c++11 -O3 -g -c -I../../include -o $@
LFLAGS = -Wall
PTHREAD = -pthread -lrt
LDL = -ldl
METIS = -L/usr/local/lib -lmetis
CARGO = -L../../lib -lcargo

OBJS = bilateral_arrangement.o

all: $(OBJS)
	$(CXX) $(LFLAGS) $(OBJS) $(CARGO) $(PTHREAD) $(METIS) $(LDL) -o bilateral_arrangement

bilateral_arrangement.o: ../../lib/libcargo.a bilateral_arrangement.h bilateral_arrangement.cpp
	$(CXX) $(CFLAGS) bilateral_arrangement.cpp

clean:
	rm *.o bilateral_arrangement
