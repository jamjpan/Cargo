CC = gcc -O3 -g -c -Iinclude -o $@
CXX = g++
CFLAGS = -Wall -Wextra -std=c++11 -O3 -g -c -Iinclude -o $@
#-------------------------------------------------------------------------------
OBJECTS = \
		  build/cargo.o \
		  build/classes.o \
		  build/dbutils.o \
		  build/file.o \
		  build/functions.o \
		  build/gtree.o \
		  build/rsalgorithm.o \
		  build/sqlite3.o
lib/libcargo.a: $(OBJECTS)
	ar rcs $@ $^
#-------------------------------------------------------------------------------
$(OBJECTS): | build

build/cargo.o: \
	include/libcargo/cargo.h \
	include/libcargo/classes.h \
	include/libcargo/dbutils.h \
	include/libcargo/file.h \
	include/libcargo/message.h \
	include/libcargo/options.h \
	include/libcargo/types.h \
	include/gtree/gtree.h \
	src/cargo.cc
	$(CXX) $(CFLAGS) src/cargo.cc

build/classes.o: \
	include/libcargo/classes.h \
	include/libcargo/types.h \
	src/classes.cc
	$(CXX) $(CFLAGS) src/classes.cc

build/dbutils.o: \
	include/libcargo/dbutils.h \
	include/libcargo/cargo.h \
	include/libcargo/classes.h \
	include/libcargo/types.h \
	src/dbutils.cc
	$(CXX) $(CFLAGS) src/dbutils.cc

build/file.o: \
	include/libcargo/file.h \
	include/libcargo/classes.h \
	include/libcargo/types.h \
	src/file.cc
	$(CXX) $(CFLAGS) src/file.cc

build/functions.o: \
	include/libcargo/functions.h \
	include/libcargo/cargo.h \
	include/libcargo/classes.h \
	include/libcargo/types.h \
	src/functions.cc
	$(CXX) $(CFLAGS) src/functions.cc

build/gtree.o: \
	include/gtree/gtree.h \
	src/gtree/gtree.cc
	$(CXX) $(CFLAGS) src/gtree/gtree.cc

build/rsalgorithm.o: \
	include/libcargo/rsalgorithm.h \
	include/libcargo/classes.h \
	include/libcargo/message.h \
	include/libcargo/types.h \
	src/rsalgorithm.cc
	$(CXX) $(CFLAGS) src/rsalgorithm.cc

build/sqlite3.o: \
	include/sqlite3/sqlite3.h \
	src/sqlite3/sqlite3.c
	$(CC) -DSQLITE_ENABLE_RTREE src/sqlite3/sqlite3.c

build:
	mkdir -p build

clean:
	rm -rf build/ lib/libcargo.a

