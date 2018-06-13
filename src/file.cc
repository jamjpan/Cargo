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
#include <algorithm> /* min(), max() */
#include <fstream>
#include <map>
#include <stdexcept>
#include <unordered_map>

#include "libcargo/file.h"
#include "libcargo/classes.h"
#include "libcargo/types.h"

namespace cargo {

size_t read_nodes(const Filepath& path, KeyValueNodes& N)
{
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

size_t read_nodes(const Filepath& path, KeyValueNodes& N, BoundingBox& bbox)
{
    read_nodes(path, N);
    Longitude min_lng = 180, max_lng = -180;
    Latitude min_lat = 90, max_lat = -90;
    for (const auto &kv : N) {
        max_lng = std::max(max_lng, kv.second.lng);
        max_lat = std::max(max_lat, kv.second.lat);
        min_lng = std::min(min_lng, kv.second.lng);
        min_lat = std::min(min_lat, kv.second.lat);
    }
    bbox = {{min_lng, min_lat}, {max_lng, max_lat}};
    return N.size();
}

size_t read_edges(const Filepath& path, KeyValueEdges& M)
{
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

size_t read_problem(const Filepath& path, ProblemSet &probset)
{
    std::ifstream ifs(path);
    if (!ifs.good())
        throw std::runtime_error("problem path not found");
    std::unordered_map<SimTime, std::vector<Trip>> trips;
    std::string _;       // unused
    size_t m, n;
    size_t count_trips = 0;
    ifs >> probset.name() >> probset.road_network() >> _ >> m >> _ >> n;
    ifs >> _;             // skip the blank line
    std::getline(ifs, _); // skip the header row
    TripId tid;
    OriginId oid;
    DestinationId did;
    Load q;
    EarlyTime early;
    LateTime late;
    while (ifs >> tid >> oid >> did >> q >> early >> late) {
        Trip tr(tid, oid, did, early, late, q);
        trips[early].push_back(tr);
        count_trips++;
    }
    ifs.close();
    probset.set_trips(trips);
    return count_trips;
}

void write_solution(
    const std::string& name, const std::string& rn, const int m, const int n,
    const int base_cost, const DistanceInt final_cost, const Filepath& path,
    const std::unordered_map<SimTime, std::map<VehicleId, NodeId>>& r,
    const std::unordered_map<SimTime, std::map<CustomerId, CustomerStatus>>& s,
    const std::unordered_map<SimTime, std::map<CustomerId, VehicleId>>& a)
{
    std::ofstream ofs(path, std::ios::out);
    ofs << name << '\n'
        << rn << '\n'
        << "VEHICLES " << m << '\n'
        << "CUSTOMERS " << n << '\n'
        << "base cost " << base_cost << '\n'
        << "solution cost " << final_cost << '\n'
        << '\n'
        << "T";

    // Print out vehicle IDs and store
    std::vector<VehicleId> veh_ids_;
    for (const auto& i : r.at(0)) { // <-- not all vehicles!@!@!@!@!
        ofs << "\tVEH" << i.first;
        veh_ids_.push_back(i.first);
    }

    // Print out customer IDs
    for (const auto& i : s.at(0))
        ofs << "\tC" << i.first << "_ST\tC" << i.first << "_AT";

    ofs << '\n';

    SimTime t = 0;
    auto check_continue = [&](SimTime t_) {
        bool in_r = (r.find(t_) != r.end());
        bool in_s = (s.find(t_) != s.end());
        bool in_a = (a.find(t_) != a.end());
        return (in_r && in_s && in_a);
    };

    while (check_continue(t)) {
        ofs << t;
        for (const auto& veh_id : veh_ids_)
            if (r.at(t).find(veh_id) != r.at(t).end())
                ofs << '\t' << r.at(t).at(veh_id);
            else
                ofs << '\t' << 0;
        for (const auto& kv : s.at(t)) {
            ofs << '\t' << (int)kv.second;
            ofs << '\t' << a.at(t).at(kv.first);
        }
        ofs << '\n';
        ++t;
    }
}

} // namespace cargo

