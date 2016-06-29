//====================
// include guards
#ifndef __paths_H_INCLUDED__
#define __paths_H_INCLUDED__

//====================
// forward declared dependencies

//====================
// included dependencies
#include<iostream>
#include<vector>
#include<fstream>
#include<string>
#include<iterator>

// define class path_point--------------------------------------------------------
class path_point{
public:
  path_point();
  void set_name(int n);
  void set_costUpToHere(int n);
  int get_name();
  int get_costUpToHere();
private:
  int name;
  int costUpToHere;
};


// define class path-------------------------------------------------------------
class path{
public:
  path();
  path(int graph_point_name);
  int get_finalPathPointName();
  int get_finalPathPointCost();
  int get_PathPointName(int i);
  int get_PathPointCostsUpToHere(int i);
  int get_NumOfPathPoints();
  void print_path();
  void add_graphPoint2Path(int index, int cost); // adds graph point "i" with length "cost" to path
private:
  std::vector <path_point> path_points;

};


//define class list_of_paths------------------------------------------------------
class path_list{
public:
  path_list();
  int get_NumOfPaths();
  void add_path2pathlist(path p);
  void remove_pathFromPathlist(int index);
  path return_lastPath();
  path return_Path(int index);
  void set_Path(path p, int index);
  int ret_cheapestPathIndex();
  path get_cheapestPath();
  int is_path2dest(int dest);
  void print_pathlist();
  int sum_CostsOfPaths();
private:
  std::vector <path> mypath_list;
};

#endif

