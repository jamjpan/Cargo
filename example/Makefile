INCLUDE_DIR = /home/jpan/devel/Cargo/include
CARGO_DIR = /home/jpan/devel/Cargo/lib
METIS_DIR = /usr/local/lib
# ------------------------------------------------------------------------------
CXX = g++
CFLAGS = -Wall -Wextra -std=c++11 -O3 -g -c -I$(INCLUDE_DIR) -o $@
LFLAGS = -Wall
PTHREAD = -pthread -lrt
LDL = -ldl
GLPK = -l:libglpk.a
METIS = -L$(METIS_DIR) -lmetis
CARGO = -L$(CARGO_DIR) -lcargo
LIBCARGO = $(CARGO_DIR)/libcargo.a
# ------------------------------------------------------------------------------
OBJECTS = \
		bilateral+.o \
		bilateral_arrangement.o \
		grabby.o \
		grasp.o \
		greedy.o \
		kinetic_tree.o \
		treeTaxiPath.o \
		nearest_neighbor.o \
		nearest_road.o \
		simulated_annealing.o \
		trip_vehicle_grouping.o \
		launcher.o

# all: $(OBJECTS)
# 	$(CXX) $(LFLAGS) -o launcher $(OBJECTS) $(CARGO) $(PTHREAD) $(METIS) $(GLPK) $(LDL) -lltdl -lz -lamd -lcolamd -fopenmp

all: $(OBJECTS)
	$(CXX) $(LFLAGS) -o launcher $(OBJECTS) $(CARGO) $(PTHREAD) $(METIS) $(GLPK) $(LDL) -fopenmp

launcher.o: $(LIBCARGO) launcher.cc
	$(CXX) $(CFLAGS) -Ikinetic_tree/include -Itrip_vehicle_grouping/include launcher.cc

bilateral+.o: $(LIBCARGO) bilateral+/bilateral+.h bilateral+/bilateral+.cc
	$(CXX) $(CFLAGS) bilateral+/bilateral+.cc

bilateral_arrangement.o: $(LIBCARGO) bilateral_arrangement/bilateral_arrangement.h bilateral_arrangement/bilateral_arrangement.cc
	$(CXX) $(CFLAGS) bilateral_arrangement/bilateral_arrangement.cc

grabby.o: $(LIBCARGO) grabby/grabby.h grabby/grabby.cc
	$(CXX) $(CFLAGS) grabby/grabby.cc

grasp.o: $(LIBCARGO) grasp/grasp.h grasp/grasp.cc
	$(CXX) $(CFLAGS) grasp/grasp.cc

greedy.o: $(LIBCARGO) greedy/greedy.h greedy/greedy.cc
	$(CXX) $(CFLAGS) greedy/greedy.cc

kinetic_tree.o: $(LIBCARGO) kinetic_tree/kinetic_tree.h kinetic_tree/kinetic_tree.cc kinetic_tree/include/treeTaxiPath.h kinetic_tree/src/treeTaxiPath.cpp
	$(CXX) $(CFLAGS) -Ikinetic_tree/include kinetic_tree/kinetic_tree.cc

treeTaxiPath.o: kinetic_tree/include/treeTaxiPath.h kinetic_tree/src/treeTaxiPath.cpp
	$(CXX) $(CFLAGS) -Ikinetic_tree/include kinetic_tree/src/treeTaxiPath.cpp

nearest_neighbor.o: $(LIBCARGO) nearest_neighbor/nearest_neighbor.h nearest_neighbor/nearest_neighbor.cc
	$(CXX) $(CFLAGS) nearest_neighbor/nearest_neighbor.cc

nearest_road.o: $(LIBCARGO) nearest_road/nearest_road.h nearest_road/nearest_road.cc
	$(CXX) $(CFLAGS) nearest_road/nearest_road.cc

simulated_annealing.o: $(LIBCARGO) simulated_annealing/simulated_annealing.h simulated_annealing/simulated_annealing.cc
	$(CXX) $(CFLAGS) simulated_annealing/simulated_annealing.cc

trip_vehicle_grouping.o: $(LIBCARGO) trip_vehicle_grouping/trip_vehicle_grouping.h trip_vehicle_grouping/trip_vehicle_grouping.cc
	$(CXX) $(CFLAGS) -Itrip_vehicle_grouping/include -fopenmp trip_vehicle_grouping/trip_vehicle_grouping.cc

clean:
	rm *.o launcher

