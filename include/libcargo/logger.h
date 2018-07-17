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
#ifndef CARGO_INCLUDE_LIBCARGO_LOGGER_H_
#define CARGO_INCLUDE_LIBCARGO_LOGGER_H_

#include <condition_variable>
#include <fstream>
#include <iostream>
#include <map>
#include <mutex>
#include <queue>
#include "libcargo/classes.h"
#include "libcargo/types.h"

namespace cargo {

// Semaphore using C++11 condition_variable
class Semaphore {
 public:
  Semaphore(int value) : value_(value) {}
  Semaphore(const Semaphore &) = delete;
  void P() {
    std::unique_lock<std::mutex> lock(mutex_);
    while (value_ == 0) condition_.wait(lock);
    --value_;
  }
  void V() {
    std::unique_lock<std::mutex> lock(mutex_);
    ++value_;
    condition_.notify_one();
  }

 private:
  int value_;
  std::mutex mutex_;
  std::condition_variable condition_;
};

// Thread-safe logger class
// Using thread-safe message queue
class Logger {
 public:
  Logger(std::string);
  ~Logger();
  void run();
  void stop() {
    done_ = true;
    condition_.notify_all();
    // condition_.notify_one();
  }
  static void put_message(std::string);
  // route update
  static void put_r_message(const std::vector<Wayp> &, const Vehicle &);
  // vehicle position update
  static void put_v_message(const std::map<VehlId, NodeId> &);
  // match update
  static void put_m_message(const std::vector<CustId> &,
                            const std::vector<CustId> &, const VehlId);
  // vehicle arrived udpate
  static void put_a_message(const std::vector<VehlId> &);

  static void put_c_message(
      const std::map<CustId, std::pair<CustStatus, VehlId>> &);
  // customer timeout
  static void put_t_message(const std::vector<CustId> &);
  // customer pickup
  static void put_p_message(const std::vector<CustId> &);
  // customer dropoff
  static void put_d_message(const std::vector<CustId> &);
  static void push(std::string);
  std::string pop();

 private:
  static std::queue<std::string> queue_;
  static Semaphore semaphore_;
  static std::condition_variable condition_;
  static std::mutex mutex_;
  std::ofstream file_output_;
  bool done_;
};
}  // namespace cargo

#endif
