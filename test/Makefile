CXX = g++
CFLAGS = -Wall -Wextra -std=c++11 -O0 -g -c -I../include -o $@
LFLAGS = -Wall
PTHREAD = -lpthread
LDL = -ldl
METIS = -L/usr/local/lib -lmetis
CARGO = -L../lib -lcargo
#-------------------------------------------------------------------------------
OBJECTS = \
		  build/main.o \
		  build/message_test.o \
		  build/sqlite3_test.o \
		  build/inserter_test.o \
		  build/simulator_test.o \
		  build/router_test.o
#		  build/DA_test.o
all: $(OBJECTS)
	$(CXX) $(LFLAGS) $(OBJECTS) $(CARGO) $(PTHREAD) $(LDL) $(METIS) -o run_tests
#-------------------------------------------------------------------------------
$(OBJECTS): | build

build/main.o: \
	src/main.cpp
	$(CXX) $(CFLAGS) src/main.cpp

build/simulator_test.o: \
	src/simulator_test.cpp
	$(CXX) $(CFLAGS) src/simulator_test.cpp

build/message_test.o: \
	src/message_test.cpp
	$(CXX) $(CFLAGS) src/message_test.cpp

build/sqlite3_test.o: \
	src/sqlite3_test.cpp
	$(CXX) $(CFLAGS) src/sqlite3_test.cpp

build/inserter_test.o: \
	src/inserter_test.cpp
	$(CXX) $(CFLAGS) src/inserter_test.cpp

build/router_test.o: \
	src/router_test.cpp
	$(CXX) $(CFLAGS) src/router_test.cpp

build/DA_test.o: \
	src/DA_test.cpp
	$(CXX) $(CFLAGS) src/DA_test.cpp

build:
	mkdir -p build

clean:
	rm -rf build/ run_tests
