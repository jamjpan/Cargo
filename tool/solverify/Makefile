all: solverify.o gtree.o
	g++ -std=c++11 -Wall -Wextra -g solverify.o gtree.o -o solverify -L/usr/local/lib -lmetis

solverify.o: src/solverify.cc
	g++ -std=c++11 -Wall -Wextra -g -c src/solverify.cc

gtree.o: include/gtree.h src/gtree.cc
	g++ -std=c++11 -Wall -Wextra -g -c src/gtree.cc

clean:
	rm -f *.o solverify

