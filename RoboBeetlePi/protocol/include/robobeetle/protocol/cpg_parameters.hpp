#pragma once
#include "robobeetle/protocol/frame.hpp"
#include <array>
#include <cmath>
#include <cstring>
#include <optional>
namespace robobeetle::protocol {
struct CpgParameters {
    double front_amp{10}, rear_amp{10}, nominal_period{2.5162}, beta{.75};
    double front_rear_phase{0}, left_right_phase{0}, coupling_strength{2};
    Byte coupling_mask{0x3c};
};
inline bool operator==(const CpgParameters &a, const CpgParameters &b) {
    return a.front_amp==b.front_amp && a.rear_amp==b.rear_amp && a.nominal_period==b.nominal_period &&
        a.beta==b.beta && a.front_rear_phase==b.front_rear_phase && a.left_right_phase==b.left_right_phase &&
        a.coupling_strength==b.coupling_strength && a.coupling_mask==b.coupling_mask;
}
inline bool valid_cpg_parameters(const CpgParameters &p) {
    const std::array<double,7> v{p.front_amp,p.rear_amp,p.nominal_period,p.beta,p.front_rear_phase,p.left_right_phase,p.coupling_strength};
    for (const auto x:v) if (!std::isfinite(x)) return false;
    return p.front_amp>=0 && p.front_amp<=28 && p.rear_amp>=0 && p.rear_amp<=30 &&
        p.nominal_period>=.5 && p.nominal_period<=10 && p.beta>=.1 && p.beta<=.9 &&
        std::abs(p.front_rear_phase)<=180 && std::abs(p.left_right_phase)<=180 &&
        p.coupling_strength>=0 && p.coupling_strength<=5 && p.coupling_mask<=63;
}
inline std::optional<Bytes> encode_cpg_parameters(const CpgParameters &p) {
    static_assert(sizeof(double)==8,"binary64 required");
    if (!valid_cpg_parameters(p)) return {};
    Bytes out(58); out[0]=1; out[1]=p.coupling_mask;
    const std::array<double,7> v{p.front_amp,p.rear_amp,p.nominal_period,p.beta,p.front_rear_phase,p.left_right_phase,p.coupling_strength};
    for (unsigned i=0;i<7;++i) {
        std::uint64_t bits; std::memcpy(&bits,&v[i],8);
        for (unsigned j=0;j<8;++j) out[2+8*i+j]=Byte(bits>>(8*j));
    }
    return out;
}
inline std::optional<CpgParameters> decode_cpg_parameters(const Bytes &p) {
    if (p.size()!=58 || p[0]!=1) return {};
    std::array<double,7> v{};
    for (unsigned i=0;i<7;++i) {
        std::uint64_t bits=0;
        for (unsigned j=0;j<8;++j) bits |= std::uint64_t(p[2+8*i+j])<<(8*j);
        std::memcpy(&v[i],&bits,8);
    }
    CpgParameters value{v[0],v[1],v[2],v[3],v[4],v[5],v[6],p[1]};
    return valid_cpg_parameters(value)?std::optional<CpgParameters>{value}:std::nullopt;
}
struct CpgParametersSnapshot {
    std::uint16_t request_sequence{}, version{};
    Byte feature_level{1};
    CpgParameters parameters;
};
inline std::optional<CpgParametersSnapshot> decode_cpg_snapshot(const Bytes &p) {
    if (p.size()!=63 || p[4]<1) return {};
    const auto params=decode_cpg_parameters(Bytes(p.begin()+5,p.end()));
    if (!params) return {};
    return CpgParametersSnapshot{std::uint16_t(p[0]|(p[1]<<8)),std::uint16_t(p[2]|(p[3]<<8)),p[4],*params};
}
inline std::optional<Bytes> encode_cpg_snapshot(const CpgParametersSnapshot &s) {
    const auto p=encode_cpg_parameters(s.parameters); if (!p || s.feature_level<1) return {};
    Bytes out{Byte(s.request_sequence),Byte(s.request_sequence>>8),Byte(s.version),Byte(s.version>>8),s.feature_level};
    out.insert(out.end(),p->begin(),p->end()); return out;
}
} // namespace robobeetle::protocol
