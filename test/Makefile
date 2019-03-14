INCLUDE = /home/jpan/devel/Cargo/include
METISDIR = /usr/local/lib
CARGODIR = /home/jpan/devel/Cargo/lib
# ------------------------------------------------------------------------------
CXX = g++
CFLAGS = -Wall -Wextra -std=c++11 -O3 -g -c -I$(INCLUDE) -o $@
LFLAGS = -Wall
PTHREAD = -lpthread
LDL = -ldl
METIS = -L$(METISDIR) -lmetis
CARGO = -L$(CARGODIR) -lcargo
#-------------------------------------------------------------------------------
OBJECTS = test-1.o test-2.o main.o
all: $(OBJECTS)
	$(CXX) $(LFLAGS) $(OBJECTS) $(CARGO) $(PTHREAD) $(LDL) $(METIS) -fopenmp -o run
#-------------------------------------------------------------------------------
test-1.o: $(CARGODIR)/libcargo.a src/test-1.cc
	$(CXX) $(CFLAGS) src/test-1.cc

test-2.o: $(CARGODIR)/libcargo.a src/test-2.cc
	$(CXX) $(CFLAGS) src/test-2.cc

main.o: src/main.cc
	$(CXX) $(CFLAGS) src/main.cc

clean:
	rm -rf *.o run

