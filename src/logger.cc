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
#include "libcargo/logger.h"
#include "libcargo/cargo.h"

namespace cargo {
Semaphore Logger::semaphore_(0);
std::queue<std::string> Logger::queue_;
std::condition_variable Logger::condition_;
std::mutex Logger::mutex_;

Logger::Logger(std::string output_path)
    : file_output_(output_path, std::ios::out) {
  done_ = false;
}

Logger::~Logger() {
  done_ = true;
  file_output_.flush();
  file_output_.close();
}

void Logger::run() {
  while (!done_) {
    // semaphore_.P();
    // std::string output = queue_.front();
    file_output_ << pop();
    file_output_ << "\n";
    // queue_.pop();
  }
}

void Logger::put_r_message(const std::vector<Wayp> &route,
                           const Vehicle &vehicle) {
  std::string item =
      std::to_string(Cargo::now()) + " R " + std::to_string(vehicle.id());
  unsigned int index = vehicle.idx_last_visited_node();
  // unsigned int index = 0;
  // const std::vector<Wayp> &points = route.data();
  for (; index < route.size(); ++index) {
    item.append(" ");
    item.append(std::to_string(route[index].second));
  }
  push(item);
}

void Logger::put_v_message(const std::map<VehlId, NodeId> &sol_routes) {
  std::string item = std::to_string(Cargo::now()) + " V";
  for (auto iter = sol_routes.begin(); iter != sol_routes.end(); ++iter) {
    item.append(" ");
    item.append(std::to_string(iter->first));
    item.append(" ");
    item.append(std::to_string(iter->second));
  }
  push(item);
}

void Logger::put_m_message(const std::vector<CustId> &cust_to_add,
                           const std::vector<CustId> &cust_to_del,
                           const VehlId vid) {
  std::string item = std::to_string(Cargo::now()) + " M " + std::to_string(vid);
  for (CustId id : cust_to_del) {
    item.append(" ");
    item.append(std::to_string(-id));
  }
  for (CustId id : cust_to_add) {
    item.append(" ");
    item.append(std::to_string(id));
  }
  push(item);
}

void Logger::put_a_message(const std::vector<VehlId> &arrived) {
  std::string item = std::to_string(Cargo::now()) + " A";
  for (VehlId id : arrived) {
    item.append(" ");
    item.append(std::to_string(id));
  }
  push(item);
}

void Logger::put_c_message(
    const std::map<CustId, std::pair<CustStatus, VehlId>> &sol_status) {}

void Logger::put_t_message(const std::vector<CustId> &timeout) {
  std::string item = std::to_string(Cargo::now()) + " T";
  for (CustId id : timeout) {
    item.append(" ");
    item.append(std::to_string(id));
  }
  push(item);
}

void Logger::put_p_message(const std::vector<CustId> &picked) {
  std::string item = std::to_string(Cargo::now()) + " P";
  for (CustId id : picked) {
    item.append(" ");
    item.append(std::to_string(id));
  }
  push(item);
}

void Logger::put_d_message(const std::vector<CustId> &dropped) {
  std::string item = std::to_string(Cargo::now()) + " D";
  for (CustId id : dropped) {
    item.append(" ");
    item.append(std::to_string(id));
  }
  push(item);
}

void Logger::push(std::string item) {
  std::unique_lock<std::mutex> lock(mutex_);
  queue_.push(item);
  lock.unlock();
  condition_.notify_one();
}

std::string Logger::pop() {
  std::unique_lock<std::mutex> lock(mutex_);
  while (queue_.empty() && !done_) {
    condition_.wait(lock);
  }
  if (done_) return "";
  std::string top = queue_.front();
  queue_.pop();
  return top;
}
}  // namespace cargo