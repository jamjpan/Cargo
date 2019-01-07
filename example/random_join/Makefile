CXX = g++
CFLAGS = -Wall -Wextra -std=c++11 -O3 -g -c -I../../include -o $@
LFLAGS = -Wall
PTHREAD = -pthread -lrt
LDL = -ldl
METIS = -L/usr/local/lib -lmetis
CARGO = -L../../lib -lcargo

OBJS = random_join.o

all: $(OBJS)
	$(CXX) $(LFLAGS) $(OBJS) $(CARGO) $(PTHREAD) $(METIS) $(LDL) -o random_join

random_join.o: ../../lib/libcargo.a random_join.h random_join.cpp
	$(CXX) $(CFLAGS) random_join.cpp

clean:
	rm *.o random_join
