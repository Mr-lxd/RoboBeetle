#pragma once

#include "robobeetle/protocol/frame.hpp"
#include <optional>

namespace robobeetle::protocol {

struct ProportionalConfig
{
    std::uint16_t max_scale{1000};
    std::uint16_t turn_gain{1000};
    std::uint16_t pitch_limit_cdeg{1000};
    std::uint16_t slew_per_second{2000};
};

struct ProportionalStart
{
    ProportionalConfig config;
    Byte session_id{};
};

struct ProportionalSetpoint
{
    Byte session_id{};
    std::uint16_t sequence{};
    std::uint16_t throttle{};
    std::int16_t turn{};
    std::int16_t pitch{};
};

inline bool valid_proportional_config(const ProportionalConfig &q)
{
    return q.max_scale >= 100 && q.max_scale <= 1000 && q.turn_gain <= 2000 &&
           q.pitch_limit_cdeg <= 2000 && q.slew_per_second >= 100 && q.slew_per_second <= 10000;
}

inline bool valid_proportional_setpoint(const ProportionalSetpoint &s)
{
    return s.throttle <= 1000 && s.turn >= -1000 && s.turn <= 1000 && s.pitch >= -1000 &&
           s.pitch <= 1000;
}

inline void proportional_put16(Bytes &out, unsigned offset, std::uint16_t value)
{
    out[offset] = static_cast<Byte>(value);
    out[offset + 1] = static_cast<Byte>(value >> 8);
}

inline std::uint16_t proportional_get16(const Bytes &in, unsigned offset)
{
    return static_cast<std::uint16_t>(in[offset] | (in[offset + 1] << 8));
}

inline std::optional<Bytes> encode_proportional_start(const ProportionalStart &s)
{
    if (!valid_proportional_config(s.config))
        return {};
    Bytes out(9);
    proportional_put16(out, 0, s.config.max_scale);
    proportional_put16(out, 2, s.config.turn_gain);
    proportional_put16(out, 4, s.config.pitch_limit_cdeg);
    proportional_put16(out, 6, s.config.slew_per_second);
    out[8] = s.session_id;
    return out;
}

inline std::optional<ProportionalStart> decode_proportional_start(const Bytes &in)
{
    if (in.size() != 9)
        return {};
    ProportionalStart s{{proportional_get16(in, 0), proportional_get16(in, 2),
                         proportional_get16(in, 4), proportional_get16(in, 6)},
                        in[8]};
    if (!valid_proportional_config(s.config))
        return {};
    return s;
}

inline std::optional<Bytes> encode_proportional_setpoint(const ProportionalSetpoint &s)
{
    if (!valid_proportional_setpoint(s))
        return {};
    Bytes out(9);
    out[0] = s.session_id;
    proportional_put16(out, 1, s.sequence);
    proportional_put16(out, 3, s.throttle);
    proportional_put16(out, 5, static_cast<std::uint16_t>(s.turn));
    proportional_put16(out, 7, static_cast<std::uint16_t>(s.pitch));
    return out;
}

inline std::optional<ProportionalSetpoint> decode_proportional_setpoint(const Bytes &in)
{
    if (in.size() != 9)
        return {};
    ProportionalSetpoint s{in[0], proportional_get16(in, 1), proportional_get16(in, 3),
                           static_cast<std::int16_t>(proportional_get16(in, 5)),
                           static_cast<std::int16_t>(proportional_get16(in, 7))};
    if (!valid_proportional_setpoint(s))
        return {};
    return s;
}

} // namespace robobeetle::protocol
