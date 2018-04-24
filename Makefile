CXX = g++
CFLAGS = -Wall -Wextra -std=c++11 -O3 -g -c -Iinclude
#-------------------------------------------------------------------------------
OBJECTS = simulator.o gtree.o file.o
lib/libcargo.a: $(OBJECTS)
	ar rcs $@ $^
#-------------------------------------------------------------------------------
simulator.o: \
	include/libcargo/simulator.h \
	include/libcargo/types.h \
	include/libcargo/file.h \
	include/libcargo/options.h \
	include/gtree/gtree.h \
	src/simulator.cpp
	$(CXX) $(CFLAGS) src/simulator.cpp

gtree.o: \
	include/gtree/gtree.h \
	src/gtree/gtree.cpp
	$(CXX) $(CFLAGS) src/gtree/gtree.cpp

file.o: \
	include/libcargo/file.h \
	src/file.cpp
	$(CXX) $(CFLAGS) src/file.cpp

clean:
	rm -rf *.o lib/libcargo.a
