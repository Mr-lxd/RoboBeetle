#pragma once

#include "robobeetle/protocol/frame.hpp"

#include <string>

namespace robobeetle::protocol {

enum class DecodeError {
    None,
    CobsDecodeFailed,
    TooShort,
    InvalidMagic,
    InvalidVersion,
    InvalidLength,
    CrcMismatch,
};

struct DecodeResult {
    Frame frame;
    DecodeError error{DecodeError::None};
    std::string detail;

    bool ok() const { return error == DecodeError::None; }
};

} // namespace robobeetle::protocol
