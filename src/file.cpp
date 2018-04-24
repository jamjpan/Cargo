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
#include "libcargo/file.h"

#include <fstream>
#include <iostream>
#include <stdexcept>

namespace cargo {
namespace file {

size_t ReadNodes(const Filepath &path, LU_NODES &N) {
    std::cout << "begin reading nodes from \"" << path << "\"\n";
    std::ifstream ifs(path);
    if (!ifs.good())
        throw std::runtime_error("node path not found");
    N.clear();
    NodeId oid, did;
    Latitude oy, dy;
    Longitude ox, dx;
    int _unused;
    while (ifs >> _unused >> oid >> did >> ox >> oy >> dx >> dy) {
        N[oid] = {oid, {ox, oy}};
        N[did] = {did, {dx, dy}};
    }
    ifs.close();
    return N.size();
}

size_t ReadEdges(const Filepath &path, LU_EDGES &M) {
    std::cout << "begin reading edges from \"" << path << "\"\n";
    std::ifstream ifs(path);
    if (!ifs.good())
        throw std::runtime_error("edge path not found");
    M.clear();
    std::string line;
    std::getline(ifs, line); // skip the header line
    NodeId oid, did;
    Distance weight;
    size_t count_edges = 0;
    while (ifs >> oid >> did >> weight) {
        M[oid][did] = weight;
        M[did][oid] = weight;
        count_edges++;
    }
    ifs.close();
    return count_edges;
}

size_t ReadProblemInstance(const Filepath &path, ProblemInstance &P) {
    std::cout << "begin reading problem instance from \"" << path << "\"\n";
    std::ifstream ifs(path);
    if (!ifs.good())
        throw std::runtime_error("problem path not found");
    P.trips.clear();
    std::string _unused;
    size_t m, n;
    size_t count_trips = 0;
    ifs >> P.name >> _unused >> m >> _unused >> n;
    ifs >> _unused; // skip the blank line
    std::getline(ifs, _unused); // skip the header row
    TripId tid;
    NodeId oid, did;
    Demand q;
    SimTime early, late;
    while (ifs >> tid >> oid >> did >> q >> early >> late) {
        P.trips[early].push_back({tid, oid, did, early, late, q});
        count_trips++;
    }
    ifs.close();
    return count_trips;
}

} // namespace file
} // namespace cargo
