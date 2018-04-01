CC = g++
CFLAGS = -Wall -Wextra -std=c++11 -O3 -g -c
LFLAGS = -Wall
PTHREAD = -lpthread
METIS = -L/usr/local/lib -lmetis
SRC = ../src
OBJS = main.o Cargo.o Customer.o Loader.o RoadNet.o Scheduel.o Solution.o \
	TimeWindow.o Vehicle.o NearestSolution.o gtree.o

cargo: $(OBJS)
	$(CC) $(LFLAGS) $(OBJS) $(PTHREAD) $(METIS) -o cargo

################################################################################
main.o: main.cpp $(SRC)/Cargo.h $(SRC)/Solution.h ./NearestSolution.h
	$(CC) $(CFLAGS) main.cpp

Cargo.o: $(SRC)/Cargo.cpp $(SRC)/Cargo.h $(SRC)/readerwriterqueue.h $(SRC)/Loader.h
	$(CC) $(CFLAGS) $(SRC)/Cargo.cpp

Customer.o: $(SRC)/Customer.cpp $(SRC)/Customer.h
	$(CC) $(CFLAGS) $(SRC)/Customer.cpp

# grid.o: $(SRC)/grid.cpp $(SRC)/common.h
 	# $(CC) $(CFLAGS) $(SRC)/grid.cpp

gtree.o: $(SRC)/gtree.cpp $(SRC)/gtree.h
	$(CC) $(CFLAGS) $(SRC)/gtree.cpp

# TShare.o: $(SRC)/TShare.cpp $(SRC)/TShare.h $(SRC)/common.h $(SRC)/print.h
	# $(CC) $(CFLAGS) $(SRC)/TShare.cpp

# Noah.o: $(SRC)/Noah.cpp $(SRC)/Noah.h $(SRC)/common.h $(SRC)/print.h
	# $(CC) $(CFLAGS) $(SRC)/Noah.cpp

# naive.o: $(SRC)/naive.cpp $(SRC)/naive.h $(SRC)/common.h $(SRC)/print.h
	# $(CC) $(CFLAGS) $(SRC)/naive.cpp

# Solution.o: $(SRC)/Solution.cpp $(SRC)/RSAlg.h $(SRC)/common.h $(SRC)/print.h
	# $(CC) $(CFLAGS) $(SRC)/Solution.cpp

# common.o: $(SRC)/common.cpp $(SRC)/common.h $(SRC)/print.h
	# $(CC) $(CFLAGS) $(SRC)/common.cpp

# print.o: $(SRC)/print.cpp $(SRC)/print.h $(SRC)/colors.h $(SRC)/common.h
	# $(CC) $(CFLAGS) $(SRC)/print.cpp

# io.o: $(SRC)/io.cpp $(SRC)/io.h $(SRC)/common.h $(SRC)/print.h
	# $(CC) $(CFLAGS) $(SRC)/io.cpp

clean:
	rm -rf *.o *~ cargo test/test
