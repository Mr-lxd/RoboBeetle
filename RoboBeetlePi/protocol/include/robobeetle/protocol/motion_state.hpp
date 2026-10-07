#pragma once
#include "robobeetle/protocol/frame.hpp"
#include <array>
#include <optional>

namespace robobeetle::protocol {
inline constexpr std::size_t MotionStateHeaderSize = 16;
inline constexpr std::size_t MotionStateSampleSize = 22;
inline constexpr std::size_t MotionStateMaxPayload = 60;

// gyro 0.1 deg/s; roll/pitch 0.01 deg; phase radians = phase_u16 * 2*pi/65536.
struct MotionStateSample {
    std::uint32_t mcu_ms{0};
    std::array<std::int16_t, 3> gyro_tenth_dps{};
    std::int16_t roll_centidegrees{0}, pitch_centidegrees{0};
    std::uint16_t phase_u16{0};
    Byte active_mode{0}, target_mode{0}, coordination{0};
    Byte backend{0}, state{0};
    bool transition{false}, phase_valid{false}, gyro_valid{false}, angle_valid{false};
    std::uint16_t angle_age_ms{65535}, phase_age_ms{65535};
};
struct MotionStateBatch {
    Byte sample_count{0};
    std::uint16_t batch_seq{0};
    Byte fragment_index{0}, fragment_count{1};
    std::uint32_t mcu_tx_ms{0}, sampler_drop_total{0};
    std::array<MotionStateSample, 2> samples{};
};

namespace motion_wire {
inline void put(Bytes &p, std::size_t o, std::uint64_t v, unsigned n) {
    for (unsigned i = 0; i < n; ++i)
        p[o + i] = static_cast<Byte>(v >> (8 * i));
}
inline std::uint64_t get(const Bytes &p, std::size_t o, unsigned n) {
    std::uint64_t v = 0;
    for (unsigned i = 0; i < n; ++i)
        v |= std::uint64_t(p[o + i]) << (8 * i);
    return v;
}
inline std::int16_t i16(const Bytes &p, std::size_t o) {
    const auto v = get(p, o, 2);
    return static_cast<std::int16_t>(v < 32768 ? std::int32_t(v) : std::int32_t(v) - 65536);
}
} // namespace motion_wire

inline std::optional<Bytes> encode_motion_state_batch(const MotionStateBatch &b) {
    if (b.sample_count > 2 || b.fragment_count == 0 || b.fragment_count > 16 ||
        b.fragment_index >= b.fragment_count)
        return std::nullopt;
    Bytes p(16 + 22 * b.sample_count, 0);
    p[0] = 1;
    p[1] = b.sample_count;
    motion_wire::put(p, 2, b.batch_seq, 2);
    p[4] = b.fragment_index;
    p[5] = b.fragment_count;
    motion_wire::put(p, 8, b.mcu_tx_ms, 4);
    motion_wire::put(p, 12, b.sampler_drop_total, 4);
    for (unsigned i = 0; i < b.sample_count; ++i) {
        const auto &s = b.samples[i];
        const auto o = 16 + 22 * i;
        motion_wire::put(p, o, s.mcu_ms, 4);
        for (unsigned axis = 0; axis < 3; ++axis)
            motion_wire::put(p, o + 4 + 2 * axis,
                             static_cast<std::uint16_t>(s.gyro_tenth_dps[axis]), 2);
        motion_wire::put(p, o + 10, static_cast<std::uint16_t>(s.roll_centidegrees), 2);
        motion_wire::put(p, o + 12, static_cast<std::uint16_t>(s.pitch_centidegrees), 2);
        motion_wire::put(p, o + 14, s.phase_u16, 2);
        p[o + 16] =
            (s.active_mode & 7U) | ((s.target_mode & 7U) << 3) | ((s.coordination & 1U) << 6);
        p[o + 17] = (s.backend & 3U) | ((s.state & 3U) << 2) | (s.transition << 4) |
                    (s.phase_valid << 5) | (s.gyro_valid << 6) | (s.angle_valid << 7);
        motion_wire::put(p, o + 18, s.angle_age_ms, 2);
        motion_wire::put(p, o + 20, s.phase_age_ms, 2);
    }
    return p;
}

inline std::optional<MotionStateBatch> decode_motion_state_batch(const Bytes &p) {
    if (p.size() < 16 || p[0] != 1 || p[1] > 2 || p.size() != 16U + 22U * p[1] ||
        motion_wire::get(p, 6, 2) != 0 || p[5] == 0 || p[5] > 16 || p[4] >= p[5])
        return std::nullopt;
    MotionStateBatch b{};
    b.sample_count = p[1];
    b.batch_seq = motion_wire::get(p, 2, 2);
    b.fragment_index = p[4];
    b.fragment_count = p[5];
    b.mcu_tx_ms = motion_wire::get(p, 8, 4);
    b.sampler_drop_total = motion_wire::get(p, 12, 4);
    for (unsigned i = 0; i < b.sample_count; ++i) {
        auto &s = b.samples[i];
        const auto o = 16 + 22 * i;
        if (p[o + 16] & 0x80U)
            return std::nullopt;
        s.mcu_ms = motion_wire::get(p, o, 4);
        for (unsigned axis = 0; axis < 3; ++axis)
            s.gyro_tenth_dps[axis] = motion_wire::i16(p, o + 4 + 2 * axis);
        s.roll_centidegrees = motion_wire::i16(p, o + 10);
        s.pitch_centidegrees = motion_wire::i16(p, o + 12);
        s.phase_u16 = motion_wire::get(p, o + 14, 2);
        s.active_mode = p[o + 16] & 7U;
        s.target_mode = (p[o + 16] >> 3) & 7U;
        s.coordination = (p[o + 16] >> 6) & 1U;
        s.backend = p[o + 17] & 3U;
        s.state = (p[o + 17] >> 2) & 3U;
        s.transition = p[o + 17] & 0x10U;
        s.phase_valid = p[o + 17] & 0x20U;
        s.gyro_valid = p[o + 17] & 0x40U;
        s.angle_valid = p[o + 17] & 0x80U;
        s.angle_age_ms = motion_wire::get(p, o + 18, 2);
        s.phase_age_ms = motion_wire::get(p, o + 20, 2);
    }
    return b;
}
} // namespace robobeetle::protocol
