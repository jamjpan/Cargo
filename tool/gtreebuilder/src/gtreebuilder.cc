// Copyright(c) 2018 James J. Pan
// Distributed under the MIT License (http://opensource.org/licenses/MIT)
//
#include "GTree.h"

#include <iostream>
#include <string>

void PrintUsage() {
  std::cerr << "Usage: ./gtreebuilder <edge_file>\n"
            << "\n"
            << "<edge_file> format:\n"
            << "first line: [# of nodes] [# of edges]\n"
            << "all other lines: [from] [to] [integer weight]\n"
            << "(nodes are 0-indexed)\n";
}

int main(int argc, char **argv) {
  if (argc < 2) {
    PrintUsage();
    return 1;
  }

  const auto fn = std::string(argv[1]);
  GTree::init();
  GTree::read(fn);
  GTree::Graph graph = GTree::getG();
  std::printf("nodes: %d\tedges: %d\n", graph.n, graph.m);
  GTree::setAdMem(2 * graph.n * log2(graph.n));
  GTree::G_Tree gtree = GTree::get();
  gtree.build(graph);
  GTree::save(gtree); // Saves to "GP_Tree.gtree"
  std::printf("Complete! Saved to \"GP_Tree.gtree\"\n");

  return 0;
}
