#pragma once

#include "robobeetle/protocol/frame.hpp"

namespace robobeetle::transport {

class Transport {
public:
    virtual ~Transport() = default;

    // The transport owns no Protocol V2 semantics; it only accepts one
    // already-encoded wire frame for transmission. A true result means the
    // complete frame has been accepted into transport TX ownership. A false
    // result means the frame was not accepted at all. Future POSIX partial
    // writes/EAGAIN handling belongs inside the transport implementation,
    // rather than being represented by an ambiguous half-written result.
    virtual bool write(const protocol::Bytes &wire_bytes) = 0;
};

} // namespace robobeetle::transport
