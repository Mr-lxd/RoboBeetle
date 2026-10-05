#pragma once
#include <QtGlobal>
#include <optional>

namespace rb {

struct DepthControlConfig {
    qint64 nominalSamplePeriodMs{100}; // [Provisional] 固件上报间隔；拿到传感器实测频率后更新
    qint64 controlFreshMs{500};   // [Provisional] 控制用新鲜度阈值（界面显示仍用 3500）
    int zeroMinSamples{5};        // 归零至少需要的新鲜样本数
    double zeroMaxRangeM{0.01};   // 归零样本的最大极差
    double surfaceMarginM{0.02};  // 归零后深度 <= 此值：禁止 ASCEND
    double softMaxM{0.40};        // >= 此值：DESCEND 改为 FORWARD（软限位）
    double softReleaseM{0.37};    // 回到此值以下才解除软限位（滞回）
    double hardMaxM{0.50};        // >= 此值：STOP 并撤防（硬限位）
};

struct DepthControlSample {
    double rawDepthM{0.0};                  // 固件上报的原始深度
    std::optional<double> calibratedDepthM; // 归零后的深度；未归零时为 nullopt
    qint64 ageMs{-1};                       // 本地收到后经过的时间 + 固件 sampleAgeMs
    bool fresh{false};                      // depthValid 且 ageMs <= controlFreshMs
};

enum class DepthEnvelopeState {
    Unavailable,  // 无样本 / 不新鲜 / 无效
    NotZeroed,    // 新鲜但本会话未归零
    Surface,      // calibrated <= surfaceMarginM
    Normal,
    SoftFloor,    // 已进入软限位（滞回锁存中）
    HardLimit     // calibrated >= hardMaxM
};

struct DepthEnvelopeMemory { bool softFloorLatched{false}; };
struct DepthEnvelopeResult { DepthEnvelopeState state; DepthEnvelopeMemory next; };

// 配置是否满足 0 <= surface < softRelease < softMax < hardMax 等约束。
[[nodiscard]] bool validDepthControlConfig(const DepthControlConfig &config) noexcept;

// 纯函数，无副作用；PR-C 的状态机在每次评估时调用。
[[nodiscard]] DepthEnvelopeResult evaluateDepthEnvelope(
    const std::optional<DepthControlSample> &sample,
    const DepthEnvelopeMemory &previous,
    const DepthControlConfig &config);

[[nodiscard]] const char *depthEnvelopeStateName(DepthEnvelopeState state) noexcept;

} // namespace rb
