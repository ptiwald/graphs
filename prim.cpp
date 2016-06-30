#include "graphs.hpp"

int main() {
  const int graph_size=50; //number of nodes
  const int length_low=1; //lower bound for edge length
  const int length_high=10; //upper bound for edge length
  const int start_point=0; //start point name/index
  const int end_point=6;
  const float dens=0.2; //density of graph
  path shortest_path;
  path_list min_spanning_tree;
  float avg_path_length;

  //  graph mygraph(graph_size,dens,length_low,length_high); //initialize graph (randomly)

  std::cout << std::endl;
  graph graph_from_file("test_graph3.txt");
  min_spanning_tree = graph_from_file.find_min_spanning_tree(start_point);
  min_spanning_tree.print_pathlist();
  std::cout << "The MST value for this graph reads " << min_spanning_tree.sum_CostsOfPaths() << std::endl;
  shortest_path = graph_from_file.find_shortest_Path(start_point, end_point);
  std::cout << "The shortest path from node " << start_point << " to node " << end_point << " reads:" << std::endl;
  shortest_path.print_path();
  std::cout << std::endl;
  
  return 0;
}
