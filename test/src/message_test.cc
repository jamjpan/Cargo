#include "catch.hpp"
#include "libcargo.h"
// =============================================================================
// Messages
// =============================================================================
using namespace cargo;

TEST_CASE("Messages print nice colors", "[msg]") {
    Message msg1(MessageType::Default);
    Message info(MessageType::Info);
    Message warn(MessageType::Warning);
    Message err(MessageType::Error);
    Message succ(MessageType::Success);

    msg1 << "Running message test...\n";
    msg1 << "this is a default msg" << std::endl;
    info << "this is an info " << "msg " << 42 << std::endl;
    warn << "this is a warn" << "ing " << 4.2 << std::endl;
    err <<  "this is an err" << "or msg " << 4*2 << std::endl;
    succ << "this is succexss" << std::endl;
    msg1 << "Message test complete." << std::endl;
}

