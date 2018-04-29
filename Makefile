CC = gcc -O3 -g -c -Iinclude -o $@
CXX = g++
CFLAGS = -Wall -Wextra -std=c++11 -O3 -g -c -Iinclude -o $@
#-------------------------------------------------------------------------------
OBJECTS = \
		  build/Simulator.o \
		  build/gtree.o \
		  build/file.o \
		  build/Solution.o
lib/libcargo.a: $(OBJECTS)
	ar rcs $@ $^
#-------------------------------------------------------------------------------
$(OBJECTS): | build

build/Simulator.o: \
	include/libcargo/Simulator.h \
	include/libcargo/types.h \
	include/libcargo/file.h \
	include/libcargo/options.h \
	include/libcargo/message.h \
	include/gtree/gtree.h \
	src/Simulator.cpp
	$(CXX) $(CFLAGS) src/Simulator.cpp

build/gtree.o: \
	include/gtree/gtree.h \
	src/gtree/gtree.cpp
	$(CXX) $(CFLAGS) src/gtree/gtree.cpp

build/file.o: \
	include/libcargo/file.h \
	src/file.cpp
	$(CXX) $(CFLAGS) src/file.cpp

build/Solution.o: \
	include/libcargo/Solution.h \
	include/libcargo/types.h \
	include/libcargo/Simulator.h
	$(CXX) $(CFLAGS) src/Solution.cpp

build:
	mkdir -p build

clean:
	rm -rf build/ lib/libcargo.a
