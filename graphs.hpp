#include<iostream>
#include<cstdlib>
#include<ctime>
#include<fstream>
#include<vector>
#include<string>
#include<iterator>
using namespace std;


// define class graph point-----------------------------------------------------
class graph_point{
public:
  graph_point() {name = 0; connectedTO.clear(); costTO.clear();} //default constr.
  graph_point(int n) {name = n; connectedTO.clear(); costTO.clear();} //constr. for a node "n" without edges
  void set_name(int n) {name = n;}
  void add_edge(int to, int cost);
  int get_name() {return name;}
  int get_degree() {return connectedTO.size();} //get number of edges
  int get_connectedPointName(int n) {return connectedTO.at(n);} //get connected point n
  int get_costToPoint(int n) {return costTO.at(n);} //get length/cost to connected point n
private:
  int name;
  vector <int> connectedTO;
  vector <int> costTO;  
};


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


// define class graph-----------------------------------------------------------
class graph{
public:
  graph(){my_graph.clear();} // default constructor
  // constr. for creating a random graph with Nnodes points, a density dens,
  // and a randomly chosen edge length between costMin and costMax
  graph(int Nnodes, float dens, int costMin, int costMax);
  graph(string filename); //constructor: reads graph from file
  void add_point(graph_point p){my_graph.push_back(p);} // add point with edges to graph
  void add_point_wo_edges(int name) {graph_point p(name); my_graph.push_back(p);} // add edgeless point to graph
  void add_undirected_edge(int from, int to, int cost);
  void add_directed_edge(int from, int to, int cost);
  int get_numOfPts(){return my_graph.size();}
  int get_nameOfP(int i){return my_graph[i].get_name();}
  int get_degreeOfP(int i){return my_graph[i].get_degree();}
  int get_numOfEdges_undirectedGraph();
  graph_point get_gpByIndex(int i){return my_graph[i];} // returns the graph point "i"
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


//FUNCTIONS--------------------------------------------------------------------------------------

//class graph_point:-------------------------
void graph_point::add_edge(int to, int cost){
  connectedTO.push_back(to);
  costTO.push_back(cost);
}


//class path:-------------------------------
// constr. that sets gp the initial point of the path
path::path(graph_point gp){
  path_point pp;
  pp.set_name(gp.get_name());
  pp.set_costUpToHere(0);
  path_points.push_back(pp);
}    

void path::print_path(){
  for (int i=0;i<get_NumOfPathPoints();i++){
    if (i != get_NumOfPathPoints()-1) cout << path_points[i].get_name() << " -> ";
    if (i == get_NumOfPathPoints()-1) cout << path_points[i].get_name() << " : " << get_finalPathPointCost();
  }
  cout << endl;
}

void path::add_graphPoint2Path(int index, int cost){
  path_point pp;
  pp.set_name(index);
  pp.set_costUpToHere(cost + get_finalPathPointCost());
  path_points.push_back(pp);
}


//class graph:-----------------------------------
// constr. for creating a random graph with Nnodes points, a density dens,
// and a randomly chosen edge length between costMin and costMax
graph::graph(int Nnodes, float dens, int costMin, int costMax){
  
  //create Nnodes empty points ... just their names/indices are set
  for (int i=0; i<Nnodes; i++) {
    add_point_wo_edges(i);
  }
  
  //go through all possible pairs of points and add undirected edges randomly
  srand(clock());
  float prop;
  int random_cost;
  for (int i=0; i<Nnodes-1; i++) {
    for (int j=i+1; j<Nnodes; j++) {
      prop = rand()/static_cast<double>(RAND_MAX);
      random_cost = (rand() % (costMax-costMin+1)) + costMin;
      if ( prop < dens) add_undirected_edge(i,j,random_cost);
    }
  }
}

//constructor that reads graph from file
graph::graph(string filename){
  ifstream graph_file(filename);
  istream_iterator<int> start(graph_file), eos;
  vector<int> values(start,eos);
 
  int Nnodes = values[0];
  for (int i=0;i<Nnodes;i++)
    add_point_wo_edges(i);

  for(int i=1; i<values.size(); i++) {
    int from,to,cost;
    switch (i%3) {
    case 1:
      from = values[i];
      break;
    case 2:
      to = values[i];
      break;
    case 0:
      cost = values[i];
      add_directed_edge(from,to,cost);
      // add_directed_edge is used since in file both edges A->B and B->A appear
      break;
    }
  }
    
  cout << "read graph from file:" << filename << endl;
  cout << "graph has " << get_numOfPts() << " nodes and " << get_numOfEdges_undirectedGraph() << " edges." << endl;
}

int graph::get_numOfEdges_undirectedGraph()
{
  int counter=0;
  for (int i=0; i<get_numOfPts(); ++i)
    counter = counter + my_graph[i].get_degree();
  
  return counter/2;
}

void graph::add_undirected_edge(int from, int to, int cost)
{
  my_graph[from].add_edge(to,cost);
  my_graph[to].add_edge(from,cost);
}

void graph::add_directed_edge(int from, int to, int cost)
{
  my_graph[from].add_edge(to,cost);
}

void graph::checkAdd_path2OpenSet(path_list &open_set, path_list closed_set, path p_new)
{
  //check if end point of path p_new is already in open or in closed set
  int pathInList = 0;
  //check if end point of p_new is already in open set; if yes: check if the p_new is shorter
  for (int path_index=0; path_index<open_set.get_NumOfPaths(); path_index++){
    if ( p_new.get_finalPathPointName() == (open_set.return_Path(path_index)).get_finalPathPointName() ) {
      pathInList = 1;
      if ( p_new.get_finalPathPointCost() < (open_set.return_Path(path_index)).get_finalPathPointCost() ) {
	open_set.set_Path(p_new,path_index);
	break;
      }
      break;
    }
  }
  //check if end point of p_new is already in closed set
  for (int path_index=0; path_index<closed_set.get_NumOfPaths(); path_index++){
    if ( p_new.get_finalPathPointName() == (closed_set.return_Path(path_index)).get_finalPathPointName() )
      pathInList = 1;
  }
  
  //if p_new is neither in open nor in closed set: add it to the open set
  if (pathInList == 0) {
    open_set.add_path2pathlist(p_new);
  }
}


//Prim's algorithm to find minimal spanning tree (MST) in a graph from start_point
//returns the shortest path
path_list graph::find_min_spanning_tree(int start_point) {
  
  path_list closed_set, open_set;
  
  //initialize closed set add the start_point to closed_set:
  { graph_point gp;
    gp = get_gpByIndex(start_point);
    path p(gp); // path p consists of the single point start_point
    closed_set.add_path2pathlist(p);
    //at the end of the algorithm this "path" needs to be removed again from the closed_set!
  }
  
  //loop runs until end_point is reached or no new edge is found (=open set is empty)
  while (1){
        
    //FIRST: add to open set the connections of the last node in the closed set
    { graph_point gp;
      int neighbor_index, cost, pathInList;

      if (closed_set.get_NumOfPaths() == 0) {
	gp = get_gpByIndex(start_point);
      }
      else {
	path p = closed_set.return_lastPath(); // get last path p (=entry) in closed set
	gp = get_gpByIndex(p.get_finalPathPointName()); // get end point gp of this path
      }

      // run through all neighbors of gp
      // for every neighbor: create a new connection p_new
      for (int i=0;i<gp.get_degree();i++){
	path p_new(gp);
	neighbor_index = gp.get_connectedPointName(i);
	cost = gp.get_costToPoint(i);
	p_new.add_graphPoint2Path(neighbor_index,cost);

	//check if endpoint of path p_new is already in open or in closed set
	//if in open set: check if p_new is shorter than the old path: if yes replace
	//if endpoint of p_new neither in open_set nor in closed set: add p_new to open_set
	checkAdd_path2OpenSet(open_set,closed_set,p_new);
      }
    }
    
    //SECOND: abort if open set is empty after expansion: no more new neighbors
    if (open_set.get_NumOfPaths() == 0 && closed_set.get_NumOfPaths() < get_numOfPts() ) {
      cout << "The graph is disconnected!" << endl;
      cout << "The program will stop now!" << endl;
      exit( 2 );
    }
    
    //THIRD: move cheapest/shortest path from open to closed set
    {
      path p;
      p = open_set.get_cheapestPath();
      closed_set.add_path2pathlist(p);
      open_set.remove_pathFromPathlist(open_set.ret_cheapestPathIndex());
    }
    

    //check if the number of paths in the closed set is n-1 - if yes: return closed set
    if (closed_set.get_NumOfPaths() == get_numOfPts()) {
	closed_set.remove_pathFromPathlist(0); // remove the trivial path consisting of single point: start_point
	return closed_set;
    }
    
    
  } //end of while loop
  
}

//Dijkstra's algorithm to find shortest path from start to end_point
//returns the shortest path
path graph::find_shortest_Path(int start_point, int end_point) {
  
  path_list closed_set, open_set;
  
  //initialize closed set
  { path p(get_gpByIndex(start_point));    
    closed_set.add_path2pathlist(p);
  }

  //loop runs until end_point is reached or no new edge is found (=open set is empty)
  while (1){
    
    //check if destination node is in closed set
    if (closed_set.is_path2dest(end_point)) {
      path p;
      p = closed_set.return_lastPath();
      return p;
    }
    
    //if destination is not reached yet:
    //FIRST: add to open set the expansion of the last path (last entry) in the closed set  
    {
      path p, p_new;
      graph_point gp;
      int neighbor_index, cost, pathInList;
      p = closed_set.return_lastPath(); // get last path p (=entry) in closed set
      gp = get_gpByIndex(p.get_finalPathPointName()); // get end point gp of this path

      // run through all neighbors of gp
      // for every neighbor: create a new path p_new by adding the neighbor to path p
      for (int i=0;i<gp.get_degree();i++){
	neighbor_index = gp.get_connectedPointName(i);
	cost = gp.get_costToPoint(i);
	p_new = p;
	p_new.add_graphPoint2Path(neighbor_index,cost);

	checkAdd_path2OpenSet(open_set,closed_set,p_new);
      }	
    }
    
    
    //SECOND: abort if open set is empty after expansion: no more new neighbors
    if (open_set.get_NumOfPaths() == 0) {
      cout << "The point of destination is not connected to the starting point!" << endl;
      cout << "The program will stop now!" << endl;
      exit( 2 );
    }
    
    //THIRD: move cheapest/shortest path from open to closed set
    {
      path p;
      p = open_set.get_cheapestPath();
      closed_set.add_path2pathlist(p);
      open_set.remove_pathFromPathlist(open_set.ret_cheapestPathIndex());
    }
        
  } //end of while loop
  
}


//class path_list:-----------------------------------------
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
