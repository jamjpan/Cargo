#include "file.h"

#include <fstream>
#include <iostream>
#include <stdexcept>

namespace cargo {
namespace file {

size_t ReadNodes(const Filepath &path, NodeMap &N) {
    std::cout << "begin reading nodes from \"" << path << "\"\n";
    std::ifstream ifs(path);
    if (!ifs.good())
        throw std::runtime_error("node path not found");
    N.clear();
    NodeId oid, did;
    Latitude oy, dy;
    Longitude ox, dx;
    int unused;
    while (ifs >> unused >> oid >> did >> ox >> oy >> dx >> dy) {
        N[oid] = {oid, {ox, oy}};
        N[did] = {did, {dx, dy}};
    }
    ifs.close();
    return N.size();
}

size_t ReadEdges(const Filepath &path, EdgeMap &M) {
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
    std::string str_unused;
    size_t m, n;
    size_t count_trips = 0;
    ifs >> P.name >> str_unused >> m >> str_unused >> n;
    ifs >> str_unused; // skip the blank line
    std::getline(ifs, str_unused); // skip the header row
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
