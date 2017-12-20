/**
 * file graph.h:                                                 
 * *************                                                             
 * 
 * This file contains an internal graph instance, and associated 
 * functions operates on this graph instance.                    
 **/

#ifndef GRAPH_H
#define GRAPH_H

#include <vector>
#include <set>
#include <queue>
#include <stack>
#include <cstring>
#define SIZE 100                    /* Max number of nodes. */

using namespace std;


/*
 * struct InternalLink:
 * This data structure is used for building a solution path.
 * Note that an instance of InternalLink holds the last node of a path,
 * i.e. internalLink.s equals the last node in the path.
 * Some examples as follow:
 *      InternalLink    *path1 = new InternalLink(new InternalLink(NULL, Arad), Sibiu);  // Build a path from Arad to Sibiu.
 *      InternalLink    *path2 = new InternalLink(
 *                                  new InternalLink( new InternalLink(NULL, Arad), Sibiu), RimnicuVilcea ) // Arad -> Sibiu -> RimnicuVilcea.
*/
struct InternalLink {
    InternalLink *p;                            /* A pointer to parent. */
    int s;                                      /* A number that represents a node. */

    // Constructor.
    InternalLink(InternalLink *p, int s): p(p), s(s) {  }
};

/*
 * Type Path:
 * Used for building a solution path with cost as the first of the pair.
 * first: the cost from start node to node path.second->s.
 * second: a path as described by struct InternalLink.
*/
typedef pair<double, InternalLink*> Path;  

/*
 * Type Link:
 * A type used for an internal graph representation.
 * first: the weight on the edge.
 * second: the number that represents a node.
*/
typedef pair<double, int> Link;

typedef bool(*searchCallBack)(int);             /* Functions to be called before or after expanding a node during search algorithms. */
typedef void(*solution)(InternalLink *);        /* Functions to be called if a solution path is found. */

/*
 * class PathComp:
 * A comparasion type for priority_queue.
*/
class PathComp {
public:
    bool operator() (Path a, Path b) const {
        if (a.first == b.first)    return a.second->s > b.second->s;
        return a.first > b.first;
    }
};


/*
 * function addEdge:
 * add an edge between node a & b.
 * @param w: weight.
*/
void addEdge(int a, int b, double w);

/*
 * function addDirectedEdge:
 * add a directed edge a -> b.
*/
void addDirectedEdge(int a, int b, double w);

/*
 * function bfs:
 * Breath-first search from node a.
 * @Param r: the root node.
 * @Param d: the destination node.
 * @Param soln: call back function if a path is found.
*/
Path bfs(int r, int d, solution soln = NULL);

/*
 * function dfs:
 * Depth-first search from node a. Note that this function is recursive.
 * @Param a: the root node.
 * @Param d: the destination node.
 * @Param prev: the previous constructed path, used in recursive functions.
 * @Param soln: call back function if a path is found.
*/
Path dfs(int a, int d, Path *prev, solution soln = NULL);

/*
 * function ucs:
 * Uniform-cost search from node a.
 * @Param r: the root node.
 * @Param d: the destination node.
 * @Param soln: callback function. If a path is found, call this function. 
*/
Path ucs(int r, int d, solution soln = NULL);


/*
 * function iddfs:
 * Iterative-depenning depth-first search from node a.
 * @Param a: the root node.
 * @Param d: the destination node.
 * @Param soln: call back function if a path is found.
*/
Path iddfs(int a, int d, int maxl, solution soln = NULL);

/*
 * function Dijkstra:
 * Calculate the single-source shortest paths to d.
 * This function is used for heuristic in gs & astar.
 * Note that if you call this function, make sure f is alloc'ed.
 * @Param f: an array that holds the distance to d.
 * @Param d: the single source.
 * @Param n: n nodes.
*/
void Dijkstra(double *f, int d, int n);


/*
 * function gs:
 * greedy search from node s.
 * @Param s: the root node.
 * @Param d: the destination node.
 * @Param n: number of nodes.
 * @Param soln: call back function if a path is found.
*/
Path gs(int s, int d, int n, solution soln = NULL);

/*
 * function astar:
 * A* search algorithm.
 * @Param s: the root node.
 * @Param d: the destination node.
 * @Param n: number of nodes.
 * @Param soln: call back function if a path is found.
*/
Path astar(int s, int d, int n, solution soln = NULL);

/*
 * function printPath:
 * Note that to use this function, one must implement it himself.
 * Used for printing a found path.
 * This function can be used as a solution call back function.
 * Pass the last node in the path to this function.
*/
void printPath(InternalLink *p);

/*
 * function printGraph:
 * print all links of a graph. for example:
 * 0 --> 1
 * 0 --> 3
 * 1 --> 0
 * 1 --> 2
 * 3 --> 0
 * 3 --> 2
 * 
*/
void printGraph(int N);

/*
 * function clearPath:
 * Call this function if a path is no longer used.
 * @Param p: the last node in the path.
*/
void clearPath(InternalLink *p);

/*
 * function clearClosed:
 * before applying any search algorithm, call this function to unmark every node in the graph.
*/
void clearClosed();

#endif