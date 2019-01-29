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
#ifndef CARGO_INCLUDE_LIBCARGO_FILE_H_
#define CARGO_INCLUDE_LIBCARGO_FILE_H_
#include <condition_variable>
#include <fstream>
#include <map>
#include <mutex>
#include <queue>
#include <string>
#include <vector>

#include "classes.h"
#include "types.h"

namespace cargo {

std::pair<std::string, std::string> parse_road_path(const std::string &);
Speed parse_speed(const std::string &);

/* These functions throw runtime_errors if the file cannot be read. */
size_t read_nodes(const Filepath &, KVNodes &);                // return # nodes
size_t read_nodes(const Filepath &, KVNodes &, BoundingBox &); // output bbox
size_t read_edges(const Filepath &, KVEdges &);                // return # edges
size_t read_problem(const Filepath &, ProblemSet &);           // return # trips

/* Thread-safe Logger for generating solplot.py input
 * The logger is "event-based"; certain events trigger putting different
 * messages into the log file. */
class Logger {
 public:
  Logger(const Filepath &);
  ~Logger();

  void run();
  void stop();

  /* put_r_message: route update
   * put_v_message: vehicle position update
   * put_m_message: match update
   * put_a_message: vehicle arrived
   * put_t_message: customer timeout
   * put_p_message: customer pickup
   * put_d_message: customer dropoff
   * put_l_message: vehicle load change
   * put_q_message: queue size
   * */
  static void put_r_message(const vec_t<Wayp> &, const Vehicle &);
  static void put_r_message(const vec_t<Wayp> &, const VehlId &, const RteIdx &);
  static void put_v_message(const std::map<VehlId, vec_t<std::pair<NodeId, DistInt>>> &);
  static void put_m_message(const vec_t<CustId> &,
                            const vec_t<CustId> &,
                            const VehlId &);
  static void put_a_message(const vec_t<VehlId> &);
  static void put_t_message(const vec_t<CustId> &);
  static void put_p_message(const vec_t<CustId> &);
  static void put_d_message(const vec_t<CustId> &);
  static void put_l_message(const vec_t<VehlId> &);
  static void put_q_message(const int &);
  static void push(std::string);
  std::string pop();

 private:
  static std::queue<std::string> queue_;
  static std::condition_variable condition_;
  static std::mutex mutex_;
  std::ofstream file_output_;
  bool done_;
};


} // namespace cargo

#endif // CARGO_INCLUDE_LIBCARGO_FILE_H_

