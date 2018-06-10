CXX = g++
CFLAGS = -Wall -Wextra -std=c++11 -O3 -g -c -I../../include -o $@
LFLAGS = -Wall
PTHREAD = -pthread -lrt
LDL = -ldl
METIS = -L/usr/local/lib -lmetis
CARGO = -L../../lib -lcargo

OBJS = trip_vehicle_grouping.o

all: $(OBJS)
	$(CXX) $(LFLAGS) $(OBJS) $(CARGO) $(PTHREAD) $(METIS) -o trip_vehicle_grouping

trip_vehicle_grouping.o: trip_vehicle_grouping.cpp
	$(CXX) $(CFLAGS) trip_vehicle_grouping.cpp

clean:
	rm *.o test
