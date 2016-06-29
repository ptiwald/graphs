#include "graphs.hpp"

//====================
//functions of class graph_point

void graph_point::add_edge(int to, int cost){
  connectedTO.push_back(to);
  costTO.push_back(cost);
}

graph_point::graph_point(int n) {name = n; connectedTO.clear(); costTO.clear();} //constr. for a node "n" without edges
void graph_point::set_name(int n) {name = n;}
void graph_point::add_edge(int to, int cost);
int graph_point::get_name() {return name;}
int graph_point::get_degree() {return connectedTO.size();} //get number of edges
int graph_point::get_connectedPointName(int n) {return connectedTO.at(n);} //get connected point n
int graph_point::get_costToPoint(int n) {return costTO.at(n);} //get length/cost to connected point n

//====================
//functions of class graph

graph::graph(){my_graph.clear();} // default constructor
  // constr. for creating a random graph with Nnodes points, a density dens,
  // and a randomly chosen edge length between costMin and costMax
void graph::add_point(graph_point p){my_graph.push_back(p);} // add point with edges to graph
void graph::add_point_wo_edges(int name) {graph_point p(name); my_graph.push_back(p);} // add edgeless point to graph
int graph::get_numOfPts(){return my_graph.size();}
int graph::get_nameOfP(int i){return my_graph[i].get_name();}
int graph::get_degreeOfP(int i){return my_graph[i].get_degree();}
graph_point graph::get_gpByIndex(int i){return my_graph[i];} // returns the graph point "i"

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
    
  std::cout << "read graph from file:" << filename << endl;
  std::cout << "graph has " << get_numOfPts() << " nodes and " << get_numOfEdges_undirectedGraph() << " edges." << endl;
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
    path p(gp.get_name()); // path p consists of the single point start_point
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
      std::cout << "The graph is disconnected!" << endl;
      std::cout << "The program will stop now!" << endl;
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
      std::cout << "The point of destination is not connected to the starting point!" << endl;
      std::cout << "The program will stop now!" << endl;
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
