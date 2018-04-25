#include "catch.hpp"

#include "libcargo.h"

using namespace cargo;

using  msg::Message;
using  msg::MessageType;

// =============================================================================
// Messages
// =============================================================================
TEST_CASE("Messages print nice colors", "[msg]") {
    Message msg1(MessageType::DEFAULT);
    Message info(MessageType::INFO);
    Message warn(MessageType::WARNING);
    Message err(MessageType::ERROR);
    Message succ(MessageType::SUCCESS);

    msg1 << "this is a default msg" << std::endl;
    info << "this is an info " << "msg " << 42 << std::endl;
    warn << "this is a warn" << "ing " << 4.2 << std::endl;
    err <<  "this is an err" << "or msg " << 4*2 << std::endl;
    succ << "this is succexss" << std::endl;
}

