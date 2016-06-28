#include<iostream>
#include<cstdlib>
#include<ctime>
#include<fstream>
#include<vector>
#include<string>
#include<iterator>
using namespace std;


// define class path_point--------------------------------------------------------
class path_point{
public:
  path_point(){name=costUpToHere=0;} //default constr.
  void set_name(int n){name=n;}
  void set_costUpToHere(int n){costUpToHere=n;}
  int get_name(){return name;}
  int get_costUpToHere(){return costUpToHere;}
private:
  int name;
  int costUpToHere;
};


// define class path-------------------------------------------------------------
class path{
public:
  path(){path_points.clear();} // default constr.
  path(graph_point gp); // constr. that sets gp the initial point of the path
  int get_finalPathPointName(){return path_points[path_points.size()-1].get_name();}
  int get_finalPathPointCost(){return path_points[path_points.size()-1].get_costUpToHere();}
  int get_PathPointName(int i){return path_points[i].get_name();}
  int get_PathPointCostsUpToHere(int i){return path_points[i].get_costUpToHere();}
  int get_NumOfPathPoints(){return path_points.size();}
  void print_path();
  void add_graphPoint2Path(int index, int cost); // adds graph point "i" with length "cost" to path
private:
  vector <path_point> path_points;

};


//define class list_of_paths------------------------------------------------------
class path_list{
public:
  path_list(){mypath_list.clear();} // default constr.
  int get_NumOfPaths(){return mypath_list.size();}
  void add_path2pathlist(path p){mypath_list.push_back(p);}
  void remove_pathFromPathlist(int index){mypath_list.erase(mypath_list.begin()+index);}
  path return_lastPath(){return mypath_list.back();} // returns last entry in path_list
  path return_Path(int index){return mypath_list.at(index);} // returns path "index"
  void set_Path(path p, int index){mypath_list[index] = p;} // set a given path within path list
  int ret_cheapestPathIndex(); // returns index of cheapest/shortest path
  path get_cheapestPath(){return mypath_list[ret_cheapestPathIndex()];} // returns cheapes/shortest path
  int is_path2dest(int dest); // returns 1 if a path to destination is in path_list, otherwise 0
  void print_pathlist();
  int sum_CostsOfPaths(); // sums the cost of all paths contained in a path_list
private:
  vector <path> mypath_list;
};

