#include "paths.hpp"

//====================
// methods of path_point
path_point::path_point(){name=costUpToHere=0;} //default constr.
void path_point::set_name(int n){name=n;}
void path_point::set_costUpToHere(int n){costUpToHere=n;}
int path_point::get_name(){return name;}
int path_point::get_costUpToHere(){return costUpToHere;}


//====================
// methods of path
path::path(){path_points.clear();} // default constr.
int path::get_finalPathPointName(){return path_points[path_points.size()-1].get_name();}
int path::get_finalPathPointCost(){return path_points[path_points.size()-1].get_costUpToHere();}
int path::get_PathPointName(int i){return path_points[i].get_name();}
int path::get_PathPointCostsUpToHere(int i){return path_points[i].get_costUpToHere();}
int path::get_NumOfPathPoints(){return path_points.size();}

// constr. that sets gp the initial point of the path
path::path(int graph_point_name){
  path_point pp;
  pp.set_name(graph_point_name);
  pp.set_costUpToHere(0);
  path_points.push_back(pp);
}    

void path::print_path(){
  for (int i=0;i<get_NumOfPathPoints();i++){
    if (i != get_NumOfPathPoints()-1) std::cout << path_points[i].get_name() << " -> ";
    if (i == get_NumOfPathPoints()-1) std::cout << path_points[i].get_name() << " : " << get_finalPathPointCost();
  }
  std::cout << std::endl;
}

void path::add_graphPoint2Path(int index, int cost){
  path_point pp;
  pp.set_name(index);
  pp.set_costUpToHere(cost + get_finalPathPointCost());
  path_points.push_back(pp);
}


//====================
//methods of path_list
path_list::path_list(){mypath_list.clear();} // default constr.
int path_list::get_NumOfPaths(){return mypath_list.size();}
void path_list::add_path2pathlist(path p){mypath_list.push_back(p);}
void path_list::remove_pathFromPathlist(int index){mypath_list.erase(mypath_list.begin()+index);}
path path_list::return_lastPath(){return mypath_list.back();} // returns last entry in path_list
path path_list::return_Path(int index){return mypath_list.at(index);} // returns path "index"
void path_list::set_Path(path p, int index){mypath_list[index] = p;} // set a given path within path list
path path_list::get_cheapestPath(){return mypath_list[ret_cheapestPathIndex()];} // returns cheapes/shortest path

int path_list::ret_cheapestPathIndex(){
  int minCost, imin;
  minCost = mypath_list[0].get_finalPathPointCost();
  imin = 0;
  for (int i=1;i<get_NumOfPaths();i++){
    if (minCost > mypath_list[i].get_finalPathPointCost()){
      minCost = mypath_list[i].get_finalPathPointCost();
      imin = i;
    }
  }
  return imin;
}

int path_list::is_path2dest(int dest){
  for (int i=0;i<get_NumOfPaths();i++){
    if (mypath_list[i].get_finalPathPointName() == dest) return 1;
  }
  return 0;
}

void path_list::print_pathlist(){
  for (int i=0;i<get_NumOfPaths();i++)
    mypath_list[i].print_path();
}

int path_list::sum_CostsOfPaths(){
  int sum=0;
  for (int i=0;i<get_NumOfPaths();i++)
    sum += mypath_list[i].get_finalPathPointCost();
  return sum;
}
