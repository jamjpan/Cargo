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
#include <condition_variable>
#include <fstream>
#include <map>
#include <mutex>
#include <queue>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

#include "libcargo/cargo.h"
#include "libcargo/classes.h"
#include "libcargo/file.h"
#include "libcargo/types.h"

namespace cargo {

std::pair<std::string, std::string> parse_road_path(const std::string& fn) {
  //https://stackoverflow.com/a/8520815
  std::string road = fn;
  std::string path = fn;
  const size_t last_slash_idx = road.find_last_of("\\/");
  if (std::string::npos != last_slash_idx) {
    road.erase(0, last_slash_idx + 1);
    path = fn.substr(0, last_slash_idx + 1);
  }
  const size_t period_idx = road.rfind('.');
  if (std::string::npos != period_idx)
    road.erase(period_idx);

  return std::make_pair(path, road);
}

Speed parse_speed(const std::string& fn) {
  std::string path = fn;
  std::string speed_str;
  const size_t s_pos = path.find("-s");
  const size_t x_pos = path.find("-x");
  if (std::string::npos != s_pos
   && std::string::npos != x_pos) {
    speed_str = path.substr(s_pos+2, x_pos-s_pos-2);
    return std::stoi(speed_str);
  }
  return -1;
}

size_t read_nodes(const Filepath& path, KVNodes& N) {
  std::ifstream ifs(path);
  if (!ifs.good()) throw std::runtime_error("node path not found");
  N.clear();
  NodeId oid, did;
  Lat oy, dy;
  Lon ox, dx;
  int _;  // unused
  while (ifs >> _ >> oid >> did >> ox >> oy >> dx >> dy) {
    N[oid] = {ox, oy};
    N[did] = {dx, dy};
  }
  ifs.close();
  return N.size();
}

size_t read_nodes(const Filepath& path, KVNodes& N, BoundingBox& bbox) {
  read_nodes(path, N);
  Lon min_lng = 180, max_lng = -180;
  Lat min_lat = 90, max_lat = -90;
  for (const auto& kv : N) {
    max_lng = std::max(max_lng, kv.second.lng);
    max_lat = std::max(max_lat, kv.second.lat);
    min_lng = std::min(min_lng, kv.second.lng);
    min_lat = std::min(min_lat, kv.second.lat);
  }
  bbox = {{min_lng, min_lat}, {max_lng, max_lat}};
  return N.size();
}

size_t read_edges(const Filepath& path, KVEdges& M) {
  std::ifstream ifs(path);
  if (!ifs.good()) throw std::runtime_error("edge path not found");
  M.clear();
  std::string _;         // unused
  std::getline(ifs, _);  // skip the header line
  NodeId oid, did;
  double weight;
  size_t count_edges = 0;
  while (ifs >> oid >> did >> weight) {
    M[oid][did] = weight;
    M[did][oid] = weight;
    M[oid][oid] = 0;
    M[did][did] = 0;
    count_edges++;
  }
  ifs.close();
  return count_edges;
}

size_t read_problem(const Filepath& path, ProblemSet& probset) {
  std::ifstream ifs(path);
  if (!ifs.good()) throw std::runtime_error("problem path not found");
  std::unordered_map<SimlTime, vec_t<Trip>> trips;
  std::string _;  // unused
  size_t m, n;
  size_t count_trips = 0;
  ifs >> probset.name() >> probset.road_network() >> _ >> _ >> m >> _ >> n;
  ifs >> _;              // skip the blank line
  std::getline(ifs, _);  // skip the header row
  TripId tid;
  OrigId oid;
  DestId did;
  Load q;
  ErlyTime early;
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

std::queue<std::string> Logger::queue_;
std::condition_variable Logger::condition_;
std::mutex Logger::mutex_;

Logger::Logger(const Filepath& f_out) : file_output_(f_out, std::ios::out) {
  done_ = false;
}

Logger::~Logger() {
  file_output_.flush();
  file_output_.close();
}

void Logger::stop() {
  done_ = true;
  condition_.notify_all();
}

void Logger::run() {
  while (!done_) {
    std::string output = pop();
    if (output.length() != 0) {
      file_output_ << output << "\n";
      file_output_.flush();
    }
  }
}

/// Many of these can be combined (refactor)

void Logger::put_r_message(const vec_t<Wayp>& rte, const Vehicle& vehl) {
  std::string item =
      std::to_string(Cargo::now()) + " R " + std::to_string(vehl.id());
  for (size_t i = vehl.idx_last_visited_node(); i < rte.size(); ++i)
    item.append(" " + std::to_string(rte.at(i).second));
  push(item);
}

void Logger::put_r_message(const vec_t<Wayp>& rte, const VehlId& vid, const RteIdx& idx) {
  std::string item =
      std::to_string(Cargo::now()) + " R " + std::to_string(vid);
  for (size_t i = idx; i < rte.size(); ++i)
    item.append(" " + std::to_string(rte.at(i).second));
  push(item);
}

void Logger::put_v_message(const std::map<VehlId, vec_t<std::pair<NodeId, DistInt>>>& locs) {
  for (const auto& kv : locs) {
    for (const auto& loc : kv.second) {
      std::string item = std::to_string(Cargo::now()) + " V";
      item.append(" " + std::to_string(kv.first) + " "
                   + std::to_string(loc.first) + " " + std::to_string(loc.second));
      push(item);
    }
  }
}

void Logger::put_m_message(const vec_t<CustId>& cadd,
                           const vec_t<CustId>& cdel, const VehlId& vid) {
  std::string item = std::to_string(Cargo::now()) + " M " + std::to_string(vid);
  for (const CustId& id : cdel) item.append(" " + std::to_string(-id));
  for (const CustId& id : cadd) item.append(" " + std::to_string(id));
  push(item);
}

void Logger::put_a_message(const vec_t<VehlId>& arrived) {
  std::string item = std::to_string(Cargo::now()) + " A";
  for (const VehlId& id : arrived) item.append(" " + std::to_string(id));
  push(item);
}

void Logger::put_t_message(const vec_t<CustId>& timeout) {
  std::string item = std::to_string(Cargo::now()) + " T";
  for (const CustId& id : timeout) item.append(" " + std::to_string(id));
  push(item);
}

void Logger::put_p_message(const vec_t<CustId>& picked) {
  std::string item = std::to_string(Cargo::now()) + " P";
  for (const CustId& id : picked) item.append(" " + std::to_string(id));
  push(item);
}

void Logger::put_d_message(const vec_t<CustId>& dropped) {
  std::string item = std::to_string(Cargo::now()) + " D";
  for (const CustId& id : dropped) item.append(" " + std::to_string(id));
  push(item);
}

void Logger::put_l_message(const vec_t<VehlId>& load_change) {
  std::string item = std::to_string(Cargo::now()) + " L";
  for (const VehlId& id : load_change) item.append(" " + std::to_string(id));
  push(item);
}

void Logger::put_q_message(const int& q) {
  std::string s = std::to_string(Cargo::now()) + " Q " + std::to_string(q);
  push(s);
}

void Logger::push(std::string item) {
  std::unique_lock<std::mutex> lock(mutex_);
  queue_.push(item);
  condition_.notify_one();
}

std::string Logger::pop() {
  std::unique_lock<std::mutex> lock(mutex_);
  while (queue_.empty() && !done_) condition_.wait(lock);
  if (done_) return "";
  std::string top = queue_.front();
  queue_.pop();
  return top;
}

}  // namespace cargo

