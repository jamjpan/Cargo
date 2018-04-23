CXX = g++
CFLAGS = -Wall -Wextra -std=c++11 -O3 -g -c
#-------------------------------------------------------------------------------
OBJECTS = Simulator.o GTree.o file.o
lib/libcargo.a: $(OBJECTS)
	ar rcs $@ $^
#-------------------------------------------------------------------------------
Simulator.o: \
	src/Simulator.cpp \
	src/Simulator.h \
	src/base/basic_types.h \
	src/base/ridesharing_types.h \
	src/base/file.h \
	src/gtree/GTree.h
	$(CXX) $(CFLAGS) src/Simulator.cpp

GTree.o: src/gtree/GTree.cc src/gtree/GTree.h
	$(CXX) $(CFLAGS) src/gtree/GTree.cc

file.o: src/base/file.cpp src/base/file.h
	$(CXX) $(CFLAGS) src/base/file.cpp

clean:
	rm -rf *.o lib/libcargo.a
