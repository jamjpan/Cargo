CXX ?= g++
CFLAGS = -Wall -Wextra -std=c++11 -c
LFLAGS = -Wall
METIS = -L/usr/local/lib -lmetis

all_tests: main.o test_common.cpp libcargo.a
	$(CXX) $(LFLAGS) main.o -L. -lcargo -o all_tests

main.o: main.cpp
	$(CXX) $(CFLAGS) main.cpp

###############################################################################

DistanceHelper.o: ../src/DistanceHelper.cpp ../src/libcargo.h
	$(CXX) $(CFLAGS) ../src/DistanceHelper.cpp

#libcargo.a: DistanceHelper.o GTree.o
#	ar rcs libcargo.a DistanceHelper.o GTree.o

clean:
	rm -f all_tests *.o *.a

rebuild: clean all_tests
