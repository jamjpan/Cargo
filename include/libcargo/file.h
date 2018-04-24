// MIT License
//
// Copyright (c) 2018 the Cargo authors
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
#ifndef CARGO_INCLUDE_FILE_H_
#define CARGO_INCLUDE_FILE_H_

#include "types.h"

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
size_t ReadNodes(const Filepath &, LU_NODES &);

// Read edges from the edge file into memory. Returns the number of edges
// read as a size type.
size_t ReadEdges(const Filepath &, LU_EDGES &);

// Read a problem instance and parse it into trips. Returns the number of
// trips as a size type.
size_t ReadProblemInstance(const Filepath &, ProblemInstance &);

} // namespace file
} // namespace cargo

#endif // CARGO_INCLUDE_FILE_H_
