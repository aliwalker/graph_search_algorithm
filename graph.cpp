#ifndef GRAPH_CPP
#define GRAPH_CPP

#include <iostream>
#include "graph.h"
using namespace std;

static bool closed[SIZE];                                 /* Used as closed set. */
static queue<Path> q;                                     /* Used for bfs algorithm. Actually it is an open set. */
static priority_queue<Path, vector<Path>, PathComp> open; /* Used for ucs, where the first = cost, and */

/*
 * Internal Representation of a graph.
*/
vector<Link> G[SIZE];                                     /* G[i] contains vector of pairs with first = weight, and second = the other node. */

/*
 * function addEdge:
 * add an edge between node a & b.
 * @param w: weight.
*/
void addEdge(int a, int b, double w) {
    G[a].push_back(make_pair(w, b));
    G[b].push_back(make_pair(w, a));
    return;
}

/*
 * function addDirectedEdge:
 * add a directed edge a -> b.
*/
void addDirectedEdge(int a, int b, double w) {
    G[a].push_back(make_pair(w, b));
    return;
}


/*
 * function bfs:
 * Breath-first search from node a.
 * @Param r: the root node.
 * @Param d: the destination node.
*/
Path bfs(int r, int d, solution soln) {
    if (r == d) {
        Path p = make_pair(0, new InternalLink(NULL, r));
        if (soln) {
            soln(p.second);
        }
        return p;
    }

    q.push(make_pair(0, new InternalLink(NULL, r)));

    while(!q.empty()) {                                     /* Expand every node in the queue(open set). */
        Path p = q.front();                                 /* Choose a node to expand. */
        q.pop();                                            /* Pop this node off the queue, */

        closed[p.second->s] = true;                         /* Add node to closed set. */

        for (vector<Link>::iterator it = G[p.second->s].begin(); it != G[p.second->s].end(); it++) {    /* Push children into queue(open set). */
            if (!closed[it->second]) {
                if (it->second == d) {                      /* If it is a solution? */
                    Path fp = make_pair(it->first + p.first, new InternalLink(p.second, it->second));
                    if (soln)   soln(fp.second);
                    while(!q.empty()) { p = q.front(); q.pop(); delete p.second; }  /* Free memory. */
                    return fp;
                }
                q.push(make_pair(it->first + p.first, new InternalLink(p.second, it->second)));
            }
        }
    }
    return make_pair(0, (InternalLink*)NULL);               /* Cannot reach the destination. */
}

/*
 * function dfs:
 * Depth-first search from node a.
*/
Path dfs(int a, int d, Path *prev, solution soln) {
    if (closed[a]) return make_pair(0, (InternalLink*)NULL);        /* Cannot reach the destination. */
    if (a == d) {
        // Simply return.
        if (soln)   soln(prev->second);
        return *prev;                                               /* Found path. */
    }

    closed[a] = true;                                               /* Put it to closed set. */

    vector<Link>::iterator it;
    for (it = G[a].begin(); it != G[a].end(); it++) {
        if (!closed[it->second]) {
            double curCost = prev->first + it->first;
            Path curPath =
                make_pair(
                    curCost,                                        /* Current cost. */
                    new InternalLink(prev->second, it->second));    /* Current path. */
            Path nextPath = 
                dfs(
                    it->second,
                    d,
                    &curPath,
                    soln);

            if (nextPath.second && nextPath.second->s == d) {
                return nextPath;
            } else {
                /*
                 * If the recursive call above returns and nextPath is not a solution,
                 * then this path can never be a solution. Free the newly alloc'ed memory.
                */
                delete curPath.second;
            }
        }
    }
    return make_pair(0, (InternalLink*)NULL);                       /* No solution is found. */
}

/*
 * function ucs:
 * Uniform-cost search from node a.
 * @Param r: the root node.
 * @Param d: the destination node.
 * @Param soln: Type of void (*solution)(InternalLink*). a callback function. If a path is found, call this function. 
*/
Path ucs(int r, int d, solution soln) {
    memset(closed, false, sizeof(closed));              /* Init the closed set to be empty. */
    open.push(make_pair(0, new InternalLink(NULL, r))); /* Put the start node in priority queue. */

    while(!open.empty()) {                              /* Expand every node in the open set. */
        Path p = open.top();                            /* Choose the node with smallest cost. */
        open.pop();                                     /* Pop the node to be expanded off the open set. */

        if (p.second->s == d) {                         /* If a solution is found? */
            if (soln)
                soln(p.second);                         /* Call back when found. */
            while(!open.empty()) {                      /* Delete every dynamically alloc'ed InternalLink except the found path. */
                delete open.top().second;
                open.pop();
            }
            return p;                                   /* Return the path */
        }

        closed[p.second->s] = true;                     /* Put the expaned node  */

        for (vector<Link>::iterator it = G[p.second->s].begin(); it != G[p.second->s].end(); it++) {
            double cost = p.first + it->first;
            if (!closed[it->second])  {
                open.push(make_pair(cost, new InternalLink(p.second, it->second)));
            }
        }
    }
}

/*
 * _ndfs:
 * Helper function for iddfs, dfs within depth n.
 * The same as dfs, with limited depth maxl.
 * This function should not be called from outside of this file.
 * @Param cl: current level depth.
*/
static Path _ndfs(int a, int d, int cl, int maxl, Path *prev, solution soln = NULL) {
    if (closed[a] || cl == maxl) return make_pair(0, (InternalLink*)NULL); /* Cannot reach the destination. */
    if (a == d) {
        // Simply return.
        if (soln)   soln(prev->second);
        return *prev;                                               /* Found path. */
    }

    closed[a] = true;                                               /* Put it to closed set. */

    vector<Link>::iterator it;
    for (it = G[a].begin(); it != G[a].end(); it++) {
        if (!closed[it->second]) {
            double curCost = prev->first + it->first;
            Path curPath =
                make_pair(
                    curCost,                                        /* Current cost. */
                    new InternalLink(prev->second, it->second));    /* Current path. */
            Path nextPath = 
                _ndfs(
                    it->second,
                    d,
                    cl + 1,
                    maxl,
                    &curPath,
                    soln);
            if ((nextPath.second) && (nextPath.second->s == d)) {
                return nextPath;
            } else {
                /*
                 * If the recursive call above returns and nextPath is not a solution,
                 * then this path can never be a solution. Free the newly alloc'ed memory.
                */
                delete curPath.second;
            }
        }
    }
    return make_pair(0, (InternalLink*)NULL);                       /* No solution is found. */
}

/*
 * function iddfs:
 * Iterative-depenning depth-first search from node a.
*/
Path iddfs(int a, int d, int maxl, solution soln) {
    int l = 1, cl = 0;

    Path s = make_pair(0, new InternalLink(NULL, a));
    Path p = _ndfs(a, d, cl, l, &s, soln);

    while(l < maxl) {                                   /* Start from depth level 1. */
        if (p.second && p.second->s == d)   return p;
        if (p.second)
            delete p.second;
        l ++;
        cl = 0;
        p = _ndfs(a, d, cl, l, &s, soln);
        clearClosed();;
    }
    delete s.second;
    delete p.second;
    return make_pair(0, (InternalLink*)NULL);
}

/*
 * function Dijkstra:
 * Calculate the single-source shortest paths to d.
 * This function is used for heuristic in gs & astar.
*/
void Dijkstra(double *f, int d, int n) {
    // Dijkstra algorithm
    vector<Link>::iterator it;
    set<int>   u;

    for (int i = 0; i < n; i++) {                              // Put each node into U.
        u.insert(i);
        f[i] = 100000;
    }
    f[d] = 0;
    u.erase(d);

    int a = d;                                                 // Start node.
    while(!u.empty()) {
        for (it = G[a].begin(); it != G[a].end(); it++) {      // Re-evaluate its neighbors
            if (it->first + f[a] < f[it->second])
                f[it->second] = it->first + f[a];
        }

        int min = *(u.begin());
        for (set<int>::iterator it2 = u.begin(); it2 != u.end(); it2++) {  // Find min node.
            if (f[*it2] < f[min])
                min = *it2;
        }
        u.erase(min);
        a = min;
    }
}


/*
 * greedy search.
*/
Path gs(int s, int d, int n, solution soln) {
    double *f = new double[n];                                  /* heuristic. */
    set<int> open, closed;                                      /* For simplicity, use int sets. */
    Path p = make_pair(0, new InternalLink(NULL, s));           /* The path to return. */
    vector<Link>::iterator it;
    int a;

    Dijkstra(f, d, n);
    open.insert(s);

    do {
        // Find the next node to expand.
        a = *(open.begin());
        for (set<int>::iterator it2 = open.begin(); it2 != open.end(); it2 ++) {
            if (f[a] > f[*it2])
                a = (*it2);
        }
        open.erase(a);                                          /* Erase it from the open set */
        closed.insert(a);                                       /* Insert it to the closed set. */
        for (it = G[p.second->s].begin(); it != G[p.second->s].end(); it++) {
            if (it->second == a)    break;
        }

        p = make_pair(p.first + it->first, new InternalLink(p.second, a));

        for (it = G[a].begin(); it != G[a].end(); it++) {
            if (closed.find(it->second) == closed.end()) {
                open.insert(it->second);
                if (it->second == d) {
                    p = make_pair(p.first + it->first, new InternalLink(p.second, it->second));
                    if (soln)   soln(p.second);
                    delete [] f;
                    return p;
                }
            }
        }
    } while(!open.empty());
    delete [] f;
    return make_pair(0, (InternalLink*)NULL);
}

Path astar(int s, int d, int n, solution soln) {
    double *h = new double[n];                                 // heuristic.
    double *g = new double[n];
    Path p = make_pair(0, new InternalLink(NULL, s));           /* The path to return. */
    vector<Link>::iterator it;
    set<int> open, closed;
    int a;

    Dijkstra(h, d, n);
    Dijkstra(g, s, n);
    open.insert(s);

    do {
        // Find the next node to expand.
        a = *(open.begin());
        for (set<int>::iterator it2 = open.begin(); it2 != open.end(); it2 ++) {
            if (h[a] + g[a] > h[*it2] + g[*it2])
                a = (*it2);
        }
        open.erase(a);
        closed.insert(a);
        for (it = G[p.second->s].begin(); it != G[p.second->s].end(); it++) {
            if (it->second == a)    break;
        }

        p = make_pair(p.first + it->first, new InternalLink(p.second, a));

        for (it = G[a].begin(); it != G[a].end(); it++) {
            if (closed.find(it->second) == closed.end()) {
                open.insert(it->second);
                if (it->second == d) {
                    p = make_pair(p.first + it->first, new InternalLink(p.second, it->second));
                    if (soln)   soln(p.second);
                    delete [] g;
                    delete [] h;
                    return p;
                }
            }
        }
    } while(!open.empty());
    delete [] h;
    delete [] g;
    return make_pair(0, (InternalLink*)NULL);
}

void printGraph(int N) {
    vector< pair<double, int> >::iterator it;
    for (int i = 0; i < N; i++) {
        for (it = G[i].begin(); it != G[i].end(); it++) {
            cout << i << " ---> " << it->second << "W: " << it->first << endl;
        }
    }
}

void clearPath(InternalLink *p) {
    if (p != NULL && p->p != NULL)
        clearPath(p->p);
    delete p;
}

void clearClosed() { memset(closed, false, SIZE); }

#endif