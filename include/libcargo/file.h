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

/* -------
 * SUMMARY
 * -------
 * This file describes file system operations and the Logger class. The
 * operations are:
 *   - read_nodes: read *.rnet file
 *   - read_edges: read *.edges file
 *   - read_problem: read *.instance file
 * The Logger class writes statistics of a Cargo simulation to *.dat file.
 * ADD DESCRIPTION OF THE MEMBER FUNCTIONS HERE.
 */

namespace cargo {

/* Filesystem Read -----------------------------------------------------------*/
size_t read_nodes(const Filepath &, KVNodes &);                // return # nodes
size_t read_nodes(const Filepath &, KVNodes &, BoundingBox &); // output bbox
size_t read_edges(const Filepath &, KVEdges &);                // return # edges
size_t read_problem(const Filepath &, ProblemSet &);           // return # trips


/* Logger --------------------------------------------------------------------*/
class Logger {
 public:
  Logger(const Filepath &);
  ~Logger();

  void run();
  void stop();

  /* put_r_message: route update
   * put_v_message: vehicle position update
   * put_m_message: match update
   * put_t_message: customer timeout
   * put_p_message: customer pickup
   * put_d_message: customer dropoff */
  static void put_r_message(const std::vector<Wayp> &, const Vehicle &);
  static void put_v_message(const std::map<VehlId, NodeId> &);
  static void put_m_message(const std::vector<CustId> &,
                            const std::vector<CustId> &,
                            const VehlId &);
  static void put_a_message(const std::vector<VehlId> &);
  static void put_t_message(const std::vector<CustId> &);
  static void put_p_message(const std::vector<CustId> &);
  static void put_d_message(const std::vector<CustId> &);

 private:
  static std::queue<std::string> queue_;
  static std::condition_variable condition_;
  static std::mutex mutex_;
  std::ofstream file_output_;
  bool done_;

  static void push(std::string);
  std::string pop();
};


} // namespace cargo

#endif // CARGO_INCLUDE_LIBCARGO_FILE_H_

