#pragma once

#include "robobeetle/protocol/frame.hpp"

namespace robobeetle::transport {

class Transport {
public:
    virtual ~Transport() = default;

    // The transport owns no Protocol V2 semantics; it only accepts one
    // already-encoded wire frame for transmission.
    virtual bool write(const protocol::Bytes &wire_bytes) = 0;
};

} // namespace robobeetle::transport
