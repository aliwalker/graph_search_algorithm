CC = g++
CXXFLAGS = -Wall -c

romania: romania_problem.cpp graph.o
	$(CC) -g -Wall -o romania romania_problem.cpp graph.o

graph.o: graph.cpp graph.h
	$(CC) $(CXXFLAGS) graph.cpp

clean:
	rm -f *.o *.gch romania