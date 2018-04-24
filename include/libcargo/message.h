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
#ifndef CARGO_INCLUDE_MESSAGE_H_
#define CARGO_INCLUDE_MESSAGE_H_

//http://stackoverflow.com/questions/9158150/ddg#9158263
//the following are UBUNTU/LINUX ONLY terminal color codes.
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

#include <iostream>

// Provides colored output.
// Usage:
//     Message myMsg(MessageType::T);
//     myMsg << "the answer is " << 42 << std::endl;
//
// Types:
//     INFO     - blue
//     WARNING  - magenta
//     ERROR    - red
//     SUCCESS  - green
namespace cargo {
namespace msg {

enum class MessageType {
    INFO,
    WARNING,
    ERROR,
    SUCCESS,
};

struct Message : std::ostream, std::streambuf {
    Message(MessageType t) : std::ostream(this), type(t) {}

    bool head = true;
    MessageType type;

    int sync() {
        std::cout << RESET;
        std::cout.flush();
        return 0;
    }

    int overflow(int c) {
        if (head) {
            switch(type) {
                case MessageType::INFO:     std::cout << BLUE;      break;
                case MessageType::WARNING:  std::cout << MAGENTA;   break;
                case MessageType::ERROR:    std::cout << RED;       break;
                case MessageType::SUCCESS:  std::cout << GREEN;     break;
            }
            head = false;
        }
        std::cout << char(c);
        return 0;
    }
};

} // namespace msg
} // namespace cargo

#endif // CARGO_INCLUDE_MESSAGE_H_
