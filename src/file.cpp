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
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
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
#include <stdexcept>

namespace cargo {
namespace file {

size_t ReadNodes(const Filepath &path, KeyValueNodes &N) {
    std::ifstream ifs(path);
    if (!ifs.good())
        throw std::runtime_error("node path not found");
    N.clear();
    NodeId oid, did;
    Latitude oy, dy;
    Longitude ox, dx;
    int _; // unused
    while (ifs >> _ >> oid >> did >> ox >> oy >> dx >> dy) {
        N[oid] = {ox, oy};
        N[did] = {dx, dy};
    }
    ifs.close();
    return N.size();
}

size_t ReadNodes(const Filepath &path, KeyValueNodes &N, Longitude &minX,
                 Longitude &maxX, Latitude &minY, Latitude &maxY) {
    std::ifstream ifs(path);
    if (!ifs.good())
        throw std::runtime_error("node path not found");
    N.clear();
    NodeId oid, did;
    Latitude oy, dy;
    Longitude ox, dx;
    int _; // unused
    Longitude minLong = 999, maxLong = -999;
    Latitude minLat = 999, maxLat = -999;
    while (ifs >> _ >> oid >> did >> ox >> oy >> dx >> dy) {
        N[oid] = {ox, oy};
        N[did] = {dx, dy};
        for (int i = 0; i < 2; i++) {
            if (ox < minLong)
                minLong = ox;
            if (ox > maxLong)
                maxLong = ox;
            if (oy < minLat)
                minLat = oy;
            if (oy > maxLat)
                maxLat = oy;
            ox = dx;
            oy = dy;
        }
    }
    ifs.close();
    minX = minLong;
    maxX = maxLong;
    minY = minLat;
    maxY = maxLat;
    return N.size();
}

size_t ReadEdges(const Filepath &path, KeyValueEdges &M) {
    std::ifstream ifs(path);
    if (!ifs.good())
        throw std::runtime_error("edge path not found");
    M.clear();
    std::string _;        // unused
    std::getline(ifs, _); // skip the header line
    NodeId oid, did;
    double weight;
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
    std::ifstream ifs(path);
    if (!ifs.good())
        throw std::runtime_error("problem path not found");
    P.trips.clear();
    std::string _;
    size_t m, n;
    size_t count_trips = 0;
    ifs >> P.name >> P.road_network >> _ >> m >> _ >> n;
    ifs >> _;             // skip the blank line
    std::getline(ifs, _); // skip the header row
    TripId tid;
    NodeId oid, did;
    int q;
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
