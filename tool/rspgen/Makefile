all: rspgen_c.o gtree.o kdtree.o
	g++ -std=c++11 -Wall -Wextra -g -O3 rspgen_c.o gtree.o kdtree.o -o rspgen_c -L/usr/local/lib -lmetis

rspgen_c.o: src/rspgen_c.cc
	g++ -std=c++11 -Wall -Wextra -g -c src/rspgen_c.cc

gtree.o: include/gtree.h src/gtree.cc
	g++ -std=c++11 -Wall -Wextra -g -c src/gtree.cc

kdtree.o: include/kdtree.h src/kdtree.c
	gcc -Wall -Wextra -c src/kdtree.c

clean:
	rm -f *.o rspgen_c

