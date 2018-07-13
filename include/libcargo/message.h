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
#include <iomanip>
#include <iostream>
#include <mutex>
#include <string>

// Provides colored output.
// Usage:
//     Message myMsg(MessageType::T);
//     myMsg << "the answer is " << 42 << std::endl;
namespace cargo {

enum class MessageType {
    Default,    // black
    Info,       // blue
    Warning,    // magenta
    Error,      // red
    Success,    // green
};

class Message : public std::ostream, public std::streambuf {
public:
    Message() : std::ostream(this), name("noname") {}
    Message(std::string n) : std::ostream(this), name(n) {}

    Message& operator()(MessageType t) { this->type = t; return *this; }

private:
    bool head = true;
    MessageType type = MessageType::Default;
    std::string name;
    static std::mutex mtx_;

    int sync()
    {
        std::cout << RESET;
        std::cout.flush();
        head = true;
        type = MessageType::Default;
        return 0;
    }

    int overflow(int c)
    {
        if (head) {
            auto now = std::chrono::system_clock::now();
            auto now_c = std::chrono::system_clock::to_time_t(now);
            switch (type) {
            case MessageType::Default:
                std::cout << RESET;
                break;
            case MessageType::Info:
                std::cout << BLUE;
                break;
            case MessageType::Warning:
                std::cout << MAGENTA;
                break;
            case MessageType::Error:
                std::cout << RED;
                break;
            case MessageType::Success:
                std::cout << GREEN;
                break;
            }
            std::lock_guard<std::mutex> lock(mtx_);
            std::cout << std::put_time(std::localtime(&now_c), "[%F %T]") << "[" << name << "] ";
            head = false;
        }
        std::cout << char(c);
        if (c == int('\n'))
            head = true;
        return 0;
    }
};

} // namespace cargo

#endif // CARGO_INCLUDE_LIBCARGO_MESSAGE_H_

