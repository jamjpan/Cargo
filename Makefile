CC = gcc -O3 -g -c -Iinclude -o $@
CXX = g++
CFLAGS = -Wall -Wextra -std=c++11 -O3 -g -c -Iinclude -o $@
#-------------------------------------------------------------------------------
OBJECTS = \
		include/libcargo.h \
		include/libcargo/classes.h \
		include/libcargo/debug.h \
		include/libcargo/distance.h \
		include/libcargo/gui.h \
		include/libcargo/message.h \
		include/libcargo/types.h \
		build/cargo.o \
		build/dbsql.o \
		build/classes.o \
		build/file.o \
		build/functions.o \
		build/grid.o \
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
	include/libcargo/dbsql.h \
	include/libcargo/debug.h \
	include/libcargo/file.h \
	include/libcargo/message.h \
	include/libcargo/options.h \
	include/libcargo/types.h \
	include/gtree/gtree.h \
	src/cargo.cc
	$(CXX) $(CFLAGS) src/cargo.cc

build/classes.o: \
	include/libcargo/classes.h \
	include/libcargo/functions.h \
	include/libcargo/types.h \
	src/classes.cc
	$(CXX) $(CFLAGS) src/classes.cc

build/dbsql.o: \
	include/libcargo/dbsql.h \
	include/libcargo/types.h \
	include/sqlite3/sqlite3.h \
	src/dbsql.cc
	$(CXX) $(CFLAGS) src/dbsql.cc

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
	include/libcargo/debug.h \
	include/libcargo/distance.h \
	include/libcargo/types.h \
	include/gtree/gtree.h \
	src/functions.cc
	$(CXX) $(CFLAGS) src/functions.cc

build/grid.o: \
	include/libcargo/grid.h \
	include/libcargo/cargo.h \
	include/libcargo/classes.h \
	include/libcargo/distance.h \
	include/libcargo/types.h \
	src/grid.cc
	$(CXX) $(CFLAGS) src/grid.cc

build/gtree.o: \
	include/gtree/gtree.h \
	src/gtree/gtree.cc
	$(CXX) $(CFLAGS) src/gtree/gtree.cc

build/rsalgorithm.o: \
	include/libcargo/rsalgorithm.h \
	include/libcargo/classes.h \
	include/libcargo/dbsql.h \
	include/libcargo/debug.h \
	include/libcargo/message.h \
	include/libcargo/types.h \
	src/rsalgorithm.cc
	$(CXX) $(CFLAGS) src/rsalgorithm.cc

build/sqlite3.o: \
	include/sqlite3/sqlite3.h \
	src/sqlite3/sqlite3.c
	$(CC) -DSQLITE_ENABLE_RTREE src/sqlite3/sqlite3.c

build:
	mkdir -p build lib

clean:
	rm -rf build/ lib/libcargo.a

