CC = gcc -O3 -g -c -Iinclude -o $@
CXX = g++
CFLAGS = -Wall -Wextra -std=c++11 -O3 -g -c -I. -o $@
LFLAGS = -Wall
PTHREAD = -pthread -lrt
LDL = -ldl
METIS = -L/usr/local/lib -lmetis
#-------------------------------------------------------------------------------
OBJS = gtree.o \
		  datcvt.o

# lib/libcargo.a: $(OBJECTS)
# 	ar rcs $@ $^
#-------------------------------------------------------------------------------
# $(OBJECTS): | build

all: $(OBJS)
	$(CXX) $(LFLAGS) $(OBJS) $(PTHREAD) $(METIS) $(LDL) -o datcvt

gtree.o: \
	gtree/gtree.h \
	gtree.cc
	$(CXX) $(CFLAGS) gtree.cc

datcvt.o: \
	gtree/gtree.h \
	datcvt.cpp
	$(CXX) $(CFLAGS) datcvt.cpp

clean:
	rm *.o datcvt
