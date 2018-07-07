CXX = g++
CFLAGS = -Wall -Wextra -std=c++11 -O3 -g -c -I../include -o $@
LFLAGS = -Wall
PTHREAD = -lpthread
LDL = -ldl
METIS = -L/usr/local/lib -lmetis
CARGO = -L../lib -lcargo
#-------------------------------------------------------------------------------
OBJECTS = \
		  build/main.o \
		  build/message_test.o \
		  build/cargo_test.o
all: $(OBJECTS)
	$(CXX) $(LFLAGS) $(OBJECTS) $(CARGO) $(PTHREAD) $(LDL) $(METIS) -o run_tests
#-------------------------------------------------------------------------------
$(OBJECTS): | build

build/main.o: \
	src/main.cc
	$(CXX) $(CFLAGS) src/main.cc

build/cargo_test.o: \
	../lib/libcargo.a \
	../include/libcargo.h \
	src/cargo_test.cc
	$(CXX) $(CFLAGS) src/cargo_test.cc

build/message_test.o: \
	../lib/libcargo.a \
	../include/libcargo.h \
	src/message_test.cc
	$(CXX) $(CFLAGS) src/message_test.cc

build:
	mkdir -p build

clean:
	rm -rf build/ run_tests

