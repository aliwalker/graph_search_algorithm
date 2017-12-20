#include "graph.h"
#include <iostream>
using namespace std;

#define Arad            0
#define Zerind          1
#define Sibiu           2
#define Timisoare       3
#define Oradea          4
#define Fagaras         5
#define RimnicuVilcea   6
#define Lugej           7
#define Bucharest       8
#define Pitesti         9
#define Craiova         10
#define Mehadia         11
#define Dobreta         12
#define Giurglu         13
#define Urziceni        14
#define Vaslui          15
#define Mirsove         16
#define lasi            17
#define Neamt           18
#define Eforie          19

/*
 * function print:
 * Used in printPath.
*/
static bool print(int a);

/*
 * function printPath:
 * Used for printing a found path.
*/
void printPath(InternalLink *p);

int main(void) {
    addEdge(Arad, Zerind, 75);
    addEdge(Arad, Sibiu, 140);
    addEdge(Arad, Timisoare, 118);
    addEdge(Zerind, Oradea, 71);
    addEdge(Sibiu, Fagaras, 90);
    addEdge(Sibiu, RimnicuVilcea, 80);
    addEdge(Timisoare, Lugej, 111);
    addEdge(Oradea, Sibiu, 151);
    addEdge(Fagaras, Bucharest, 211);
    addEdge(RimnicuVilcea, Pitesti, 97);
    addEdge(RimnicuVilcea, Craiova, 146);
    addEdge(Lugej, Mehadia, 70);
    addEdge(Bucharest, Giurglu, 90);
    addEdge(Bucharest, Urziceni, 85);
    addEdge(Pitesti, Bucharest, 101);
    addEdge(Craiova, Pitesti, 138);
    addEdge(Mehadia, Dobreta, 75);
    addEdge(Dobreta, Craiova, 120);
    addEdge(Urziceni, Vaslui, 142);
    addEdge(Urziceni, Mirsove, 98);
    addEdge(Mirsove, Eforie, 86);
    addEdge(Vaslui, lasi, 92);
    addEdge(lasi, Neamt, 97);
    
    cout << "*******************************bfs************************************\n";
    clearClosed();;
    Path p = bfs(Arad, Bucharest, printPath);
    cout << "\nTotal cost: " << p.first << endl;
    clearPath(p.second);

    cout << "*******************************dfs************************************\n";
    clearClosed();;
    Path s = make_pair(0, new InternalLink(NULL, Arad));
    p = dfs(Arad, Bucharest, &s, printPath);
    cout << "\nTotal cost: " << p.first << endl;
    clearPath(p.second);

    cout << "******************************iddfs***********************************\n";
    clearClosed();;
    p = iddfs(Arad, Bucharest, 20, printPath);
    cout << "\nTotal cost: " << p.first << endl;
    clearPath(p.second);

    cout << "*******************************ucs************************************\n";
    clearClosed();;
    p = ucs(Arad, Bucharest, printPath);
    cout << "\nTotal cost: " << p.first << endl;
    clearPath(p.second);

    cout << "*******************************gs*************************************\n";
    p = gs(Arad, Bucharest, 20, printPath);
    cout << "\nTotal cost: " << p.first << endl;
    clearPath(p.second);

    cout << "*****************************astar************************************\n";
    p = astar(Arad, Bucharest, 20, printPath);
    cout << "\nTotal cost: " << p.first << endl;
    clearPath(p.second);

    return 0;
}

bool print(int a) {
    switch(a) {
    case Arad           : cout << "Arad          "; return true;
    case Zerind         : cout << "Zerind        "; return true;
    case Sibiu          : cout << "Sibiu         "; return true;
    case Timisoare      : cout << "Timisoare     "; return true;       
    case Oradea         : cout << "Oradea        "; return true;           
    case Fagaras        : cout << "Fagaras       "; return true;           
    case RimnicuVilcea  : cout << "RimnicuVilcea "; return true;
    case Lugej          : cout << "Lugej         "; return true;           
    case Bucharest      : cout << "Bucharest     "; return true;           
    case Pitesti        : cout << "Pitesti       "; return true;           
    case Craiova        : cout << "Craiova       "; return true;           
    case Mehadia        : cout << "Mehadia       "; return true;           
    case Dobreta        : cout << "Dobreta       "; return true;           
    case Giurglu        : cout << "Giurglu       "; return true;           
    case Urziceni       : cout << "Urziceni      "; return true;           
    case Vaslui         : cout << "Vaslui        "; return true;           
    case Mirsove        : cout << "Mirsove       "; return true;           
    case lasi           : cout << "lasi          "; return true;           
    case Neamt          : cout << "Neamt         "; return true;           
    case Eforie         : cout << "Eforie        "; return true;           
    }
}

void printPath(InternalLink *p) {
    if (p->p != NULL)
        printPath(p->p);
    print(p->s);
}