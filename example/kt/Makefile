CXX = g++
CFLAGS = -Wall -Wextra -std=c++11 -O3 -g -c -I../../include -Iinclude -o $@
LFLAGS = -Wall
PTHREAD = -pthread -lrt
LDL = -ldl
METIS = -L/usr/local/lib -lmetis
CARGO = -L../../lib -lcargo

OBJS = kt.o treeTaxiPath.o

all: $(OBJS)
	$(CXX) $(LFLAGS) $(OBJS) $(CARGO) $(PTHREAD) $(METIS) $(LDL) -o kt

kt.o: ../../lib/libcargo.a kt.h kt.cpp
	$(CXX) $(CFLAGS) kt.cpp

treeTaxiPath.o: include/treeTaxiPath.h src/treeTaxiPath.cpp
	$(CXX) $(CFLAGS) src/treeTaxiPath.cpp

clean:
	rm *.o kt

