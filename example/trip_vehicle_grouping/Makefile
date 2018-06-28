CXX = g++
CFLAGS = -Wall -Wextra -std=c++11 -O3 -g -c -I../../include -Iinclude -o $@
LFLAGS = -Wall
PTHREAD = -pthread -lrt
LDL = -ldl
GLPK = -l:libglpk.a
METIS = -lmetis
CARGO = -L../../lib -lcargo

OBJS = trip_vehicle_grouping.o

all: $(OBJS)
	$(CXX) $(LFLAGS) $(OBJS) $(CARGO) $(PTHREAD) $(METIS) $(GLPK) $(LDL) -fopenmp -o trip_vehicle_grouping

trip_vehicle_grouping.o: ../../lib/libcargo.a trip_vehicle_grouping.h trip_vehicle_grouping.cpp
	$(CXX) $(CFLAGS) -fopenmp trip_vehicle_grouping.cpp

clean:
	rm *.o trip_vehicle_grouping
