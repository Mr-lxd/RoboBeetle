#pragma once

#include "robobeetle/protocol/frame.hpp"

namespace robobeetle::protocol {

Bytes cobs_encode(const Bytes &input);
bool cobs_decode(const Bytes &input, Bytes &output);

} // namespace robobeetle::protocol
