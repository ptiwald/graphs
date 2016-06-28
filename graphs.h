//====================
// include guards
#ifndef __graphs_H_INCLUDED__
#define __graphs_H_INCLUDED__

//====================
//forward declared dependencies
//... you can not forward declare path/path_list -> objects of them are used

//====================
//included dependencies
#include<vector>
#include<string>
#include "paths.h"

//====================
// declare class graph point
class graph_point{
public:
  graph_point(int n);
  void set_name(int n);
  void add_edge(int to, int cost);
  int get_name();
  int get_degree();
  int get_connectedPointName(int n);
  int get_costToPoint(int n);
private:
  int name;
  vector <int> connectedTO;
  vector <int> costTO;  
};

//====================
// declare class graph
class graph{
public:
  graph(int Nnodes, float dens, int costMin, int costMax);
  graph(string filename);
  void add_point(graph_point p);
  void add_point_wo_edges(int name);
  void add_undirected_edge(int from, int to, int cost);
  void add_directed_edge(int from, int to, int cost);
  int get_numOfPts();
  int get_nameOfP(int i);
  int get_degreeOfP(int i);
  int get_numOfEdges_undirectedGraph();
  graph_point get_gpByIndex(int i);
  // Dijkstra's algorithm to find shortest path from start to end
  // it returns the shortest path
  path find_shortest_Path(int start_point, int end_point);
  // Prim's algorithm to find the minimum spanning tree
  // it returns the list of edges building up the tree
  path_list find_min_spanning_tree(int start_point);

//properties of graph
private:
  vector <graph_point> my_graph;
  void checkAdd_path2OpenSet(path_list &open_set, path_list closed_set, path p_new);
};

