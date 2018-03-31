CC = g++
CFLAGS = -Wall -Wextra -std=c++11 -O3 -g -c
LFLAGS = -Wall
PTHREAD = -lpthread
METIS = -L/usr/local/lib -lmetis
OBJS = main.o \
	   print.o io.o \
	   routeRS.o grid.o gtree.o common.o \
	   TShare.o Rex.o Rap.o RSAlg.o Noah.o naive.o

routeRS: $(OBJS)
	$(CC) $(LFLAGS) $(OBJS) $(PTHREAD) $(METIS) -o routeRS

################################################################################
main.o: main.cpp ../src/Cargo.h ../src/Solution.h
	$(CC) $(CFLAGS) src/main.cc

routeRS.o: src/routeRS.cc src/routeRS.h src/common.h src/readerwriterqueue.h \
	src/print.h src/io.h src/TShare.h src/Rex.h src/Rap.h src/naive.h src/RSAlg.h
	$(CC) $(CFLAGS) src/routeRS.cc

Rex.o: src/Rex.cc src/Rex.h src/common.h src/print.h
	$(CC) $(CFLAGS) src/Rex.cc

Rap.o: src/Rap.cc src/Rap.h src/common.h src/print.h
	$(CC) $(CFLAGS) src/Rap.cc

grid.o: src/grid.cc src/common.h
	$(CC) $(CFLAGS) src/grid.cc

gtree.o: src/gtree.cc src/gtree.h
	$(CC) $(CFLAGS) src/gtree.cc

TShare.o: src/TShare.cc src/TShare.h src/common.h src/print.h
	$(CC) $(CFLAGS) src/TShare.cc

Noah.o: src/Noah.cc src/Noah.h src/common.h src/print.h
	$(CC) $(CFLAGS) src/Noah.cc

naive.o: src/naive.cc src/naive.h src/common.h src/print.h
	$(CC) $(CFLAGS) src/naive.cc

RSAlg.o: src/RSAlg.cc src/RSAlg.h src/common.h src/print.h
	$(CC) $(CFLAGS) src/RSAlg.cc

common.o: src/common.cc src/common.h src/print.h
	$(CC) $(CFLAGS) src/common.cc

print.o: src/print.cc src/print.h src/colors.h src/common.h
	$(CC) $(CFLAGS) src/print.cc

io.o: src/io.cc src/io.h src/common.h src/print.h
	$(CC) $(CFLAGS) src/io.cc

clean:
	rm -rf *.o *~ routeRS test/test

