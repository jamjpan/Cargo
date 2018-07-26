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
#ifndef CARGO_INCLUDE_LIBCARGO_MESSAGE_H_
#define CARGO_INCLUDE_LIBCARGO_MESSAGE_H_

// From http://stackoverflow.com/questions/9158150/ddg#9158263
// The following are UBUNTU/LINUX ONLY terminal color codes.
#define RESET       "\033[0m"
#define BLACK       "\033[30m"              /* Black        */
#define RED         "\033[31m"              /* Red          */
#define GREEN       "\033[32m"              /* Green        */
#define YELLOW      "\033[33m"              /* Yellow       */
#define BLUE        "\033[34m"              /* Blue         */
#define MAGENTA     "\033[35m"              /* Magenta      */
#define CYAN        "\033[36m"              /* Cyan         */
#define WHITE       "\033[37m"              /* White        */
#define BOLDBLACK   "\033[1m\033[30m"       /* Bold Black   */
#define BOLDRED     "\033[1m\033[31m"       /* Bold Red     */
#define BOLDGREEN   "\033[1m\033[32m"       /* Bold Green   */
#define BOLDYELLOW  "\033[1m\033[33m"       /* Bold Yellow  */
#define BOLDBLUE    "\033[1m\033[34m"       /* Bold Blue    */
#define BOLDMAGENTA "\033[1m\033[35m"       /* Bold Magenta */
#define BOLDCYAN    "\033[1m\033[36m"       /* Bold Cyan    */
#define BOLDWHITE   "\033[1m\033[37m"       /* Bold White   */

#include <chrono>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <sstream>
#include <string>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>

// Provides colored output.
// Usage:
//     Message myMsg(MessageType::T);
//     myMsg << "the answer is " << 42 << std::endl;
namespace cargo {

enum class MessageType {
  Default,  // black
  Info,     // blue
  Warning,  // magenta
  Error,    // red
  Success,  // green
};

class MessageStreamBuffer : public std::streambuf {
 public:
  MessageStreamBuffer(MessageType t = MessageType::Default) {
    type = t;
  }

  MessageType type;
  bool head = true;
  std::string name;
  std::streambuf* sb;

 private:
  int sync() { // override
    sb->sputn(RESET, sizeof(RESET)-1);
    sb->pubsync();
    head = true;
    type = MessageType::Default;
    return 0;
  }

  /* Message has no buffer; every char "overflows". Append the timestamp and
   * the color code and put into buf. */
  int overflow(int c) {
    if (head) {
      auto now = std::chrono::system_clock::now();
      auto now_c = std::chrono::system_clock::to_time_t(now);
      switch (type) {
        case MessageType::Default:
          sb->sputn(RESET, sizeof(RESET)-1);
          break;
        case MessageType::Info:
          sb->sputn(BLUE, sizeof(BLUE)-1);
          break;
        case MessageType::Warning:
          sb->sputn(MAGENTA, sizeof(MAGENTA)-1);
          break;
        case MessageType::Error:
          sb->sputn(RED, sizeof(RED)-1);
          break;
        case MessageType::Success:
          sb->sputn(GREEN, sizeof(GREEN)-1);
          break;
      }
      auto tm = std::localtime(&now_c);
      std::ostringstream ts;
      ts << std::setfill('0') << "["
         << std::setw(2) << tm->tm_hour << ":"
         << std::setw(2) << tm->tm_min  << ":"
         << std::setw(2) << tm->tm_sec  << "]"
         << "[" << name << "] ";
      const std::string& ts_str = ts.str();
      const char* c_ts = ts_str.c_str();
      sb->sputn(c_ts, ts_str.length());
      head = false;
    }
    sb->sputc(c);
    if (c == int('\n')) head = true;
    return 0;
  }
};

class Message : public std::ostream {
 public:
  Message(std::string n = "noname", bool fifo = false) : name(n) {
    buf = new MessageStreamBuffer;
    if (fifo) {
      std::cout << "Creating " << (n+".feed") << std::endl;
      struct stat _;
      if (stat((n+".feed").c_str(), &_) == 0) {
        std::remove((n+".feed").c_str()); // an old fifo exists
        std::cout << "(removed an old feed)" << std::endl;
      }
      int rc = mkfifo((n+".feed").c_str(), 0666);
      if (rc == -1) {
        std::cout << "Failed to make feed" << std::endl;
        throw;
      }
      std::cout << "Attach a listener " << "(e.g. cat " << n << ".feed | tee log.txt) "
                << " to continue..." << std::endl;
      of.open(n+".feed", std::ios::out);
      if (!of.is_open()) {
        std::cout << "Failed to open feed" << std::endl;
        throw;
      }
      buf->sb = of.rdbuf();
    } else
      buf->sb = std::cout.rdbuf();
    buf->name = n;
    this->std::ios::init(buf);
  }

  ~Message() { delete buf; unlink((name+".feed").c_str()); }

  Message& operator()(MessageType t) {
    buf->type = t;
    return *this;
  }

 private:
  std::string name;
  std::ofstream of;
  MessageStreamBuffer* buf;
};

}  // namespace cargo

#endif // CARGO_INCLUDE_LIBCARGO_MESSAGE_H_

