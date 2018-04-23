#ifndef CARGO_FILE_H_
#define CARGO_FILE_H_

#include "basic_types.h"
#include "ridesharing_types.h"

#include <string>

namespace cargo {
namespace file {

// Read nodes from the road network into memory. Don't consider the edges here;
// use ReadEdges for that. That way, the gtree and the edge map are consistent
// (the gtree is built using the edge file). Returns the number of nodes read as
// a size type.
//
// TODO: we already have a separate .edges file for building the gtree; might
// as well have a separate .nodes file as well, for the node coordinates.
// Otherwise the .rnet file is redundant... the original purpose of the rnet
// file was to help plotting using gnuplot!!
size_t ReadNodes(const Filepath &, NodeMap &);

// Read edges from the edge file into memory. Returns the number of edges
// read as a size type.
size_t ReadEdges(const Filepath &, EdgeMap &);

// Read a problem instance and parse it into trips. Returns the number of
// trips as a size type.
size_t ReadProblemInstance(const Filepath &, ProblemInstance &);

} // namespace file
} // namespace cargo

#endif // CARGO_FILE_H_
