CXX ?= g++
CFLAGS = -Wall -Wextra -std=c++11 -c
LFLAGS = -Wall
METIS = -L/usr/local/lib -lmetis

all_tests: main.o DistanceHelperTest.cpp libezrs.a
	$(CXX) $(LFLAGS) main.o DistanceHelperTest.cpp -L. -lezrs $(METIS) -o all_tests

main.o: main.cpp
	$(CXX) $(CFLAGS) main.cpp

###############################################################################

DistanceHelper.o: ../src/DistanceHelper.cpp ../src/libezrs.h
	$(CXX) $(CFLAGS) ../src/DistanceHelper.cpp

GTree.o: ../src/gtree/GTree.cc ../src/gtree/GTree.h
	$(CXX) $(CFLAGS) ../src/gtree/GTree.cc

libezrs.a: DistanceHelper.o GTree.o
	ar rcs libezrs.a DistanceHelper.o GTree.o

clean:
	rm -f all_tests *.o *.a

rebuild: clean all_tests
