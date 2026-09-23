#include "robobeetle/gateway/gateway_owner.hpp"

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <type_traits>
#include <utility>

namespace robobeetle::gateway {
namespace {

constexpr std::size_t kMaxInboundMessages = 32U;
constexpr std::size_t kMaxInboundPayloadBytes = 16U * 1024U;
constexpr std::size_t kMaxSourceLossSignals = 8U;
constexpr GatewayTimeMs kOwnerWaitMs = 100U;

} // namespace

GatewayOwner::GatewayOwner(std::string device_path, std::string bind_address,
                           std::uint16_t port, link_core::LinkCoreConfig config,
                           std::size_t max_tx_frames,
                           std::size_t max_tx_bytes)
    : application_(std::move(device_path), config, max_tx_frames, max_tx_bytes),
      tcp_(std::move(bind_address), port,
           TcpAdapterCallbacks{
               [this](ControlSourceId source) {
                   on_source_connected(source);
               },
               [this](const RemoteEnvelope &envelope) {
                   return enqueue_inbound(envelope);
               },
               [this](const SourceLostSignal &signal) {
                   on_source_lost(signal);
               },
               [this](const char *message) { on_diagnostic(message); },
               [this] { return monotonic_now(); },
           }),
      core_(application_,
             GatewayCoreCallbacks{
                 [this](const GatewayOutbound &output) {
                     return tcp_.publish(output);
                },
                [this](const CloseSourceSignal &signal) {
                    return tcp_.close_source(signal);
                },
                 [this](const char *message) { on_diagnostic(message); },
                 [this] { return monotonic_now(); },
             })
{
}

GatewayTimeMs GatewayOwner::monotonic_now() noexcept
{
    using Clock = std::chrono::steady_clock;
    static_assert(Clock::is_steady, "owner requires a monotonic clock");
    return static_cast<GatewayTimeMs>(
        std::chrono::duration_cast<std::chrono::milliseconds>(
            Clock::now().time_since_epoch())
            .count());
}

std::size_t GatewayOwner::remote_payload_size(
    const RemoteMessage &message) noexcept
{
    if (const auto *hello = std::get_if<HelloRequest>(&message.payload)) {
        (void)hello;
        return 2U;
    }
    if (std::holds_alternative<AcquireControlRequest>(message.payload) ||
        std::holds_alternative<ControlHeartbeatRequest>(message.payload) ||
        std::holds_alternative<ReleaseControlRequest>(message.payload)) {
        return 0U;
    }
    const auto *command = std::get_if<CommandRequest>(&message.payload);
    if (command == nullptr) {
        return 0U;
    }
    switch (static_cast<RobotCommandKind>(command->command_kind)) {
    case RobotCommandKind::EnableServos:
    case RobotCommandKind::DisableServos:
    case RobotCommandKind::NeutralServos:
        return 3U;
    case RobotCommandKind::SetServoAngle:
    case RobotCommandKind::SetServoPwm:
        return 4U;
    case RobotCommandKind::StartMotion:
    case RobotCommandKind::SetGaitBackend:
    case RobotCommandKind::SetFrontRearCoordination:
        return 2U;
    case RobotCommandKind::StopMotion:
        return 1U;
    default:
        return 1U;
    }
}

void GatewayOwner::notify_owner() noexcept
{
    bridge_changed_.notify_one();
}

void GatewayOwner::on_source_connected(ControlSourceId source) noexcept
{
    try {
        std::lock_guard<std::mutex> lock(bridge_mutex_);
        if (stop_requested_) {
            return;
        }
        if (source == 0U) {
            on_diagnostic("zero source registration ignored");
            return;
        }
        pending_connections_.clear();
        pending_connections_.push_back(source);
    } catch (...) {
        on_diagnostic("source connection bridge failure");
    }
    notify_owner();
}

bool GatewayOwner::enqueue_inbound(const RemoteEnvelope &envelope) noexcept
{
    try {
        const auto payload_size = remote_payload_size(envelope.message);
        std::lock_guard<std::mutex> lock(bridge_mutex_);
        if (stop_requested_) {
            return false;
        }
        if (pending_inbound_.size() >= kMaxInboundMessages ||
            pending_inbound_bytes_ + payload_size >
                kMaxInboundPayloadBytes) {
            ++inbound_overflow_count_;
            return false;
        }
        pending_inbound_bytes_ += payload_size;
        pending_inbound_.push_back(envelope);
        inbound_messages_high_water_ = std::max(
            inbound_messages_high_water_, pending_inbound_.size());
        inbound_payload_bytes_high_water_ = std::max(
            inbound_payload_bytes_high_water_, pending_inbound_bytes_);
    } catch (...) {
        return false;
    }
    notify_owner();
    return true;
}

void GatewayOwner::on_source_lost(const SourceLostSignal &signal) noexcept
{
    try {
        std::lock_guard<std::mutex> lock(bridge_mutex_);
        if (pending_source_losses_.size() >= kMaxSourceLossSignals) {
            // A source can only be current once. Coalescing an identical
            // lifecycle signal preserves the safety edge without dropping it.
            if (std::find_if(pending_source_losses_.begin(),
                             pending_source_losses_.end(),
                             [&signal](const auto &pending) {
                                 return pending.source == signal.source;
                             }) != pending_source_losses_.end()) {
                return;
            }
            stop_requested_ = true;
            on_diagnostic("source-loss bridge saturated; stopping owner");
            return;
        }
        if (std::find_if(pending_source_losses_.begin(),
                         pending_source_losses_.end(),
                         [&signal](const auto &pending) {
                             return pending.source == signal.source &&
                                    pending.reason == signal.reason;
                         }) == pending_source_losses_.end()) {
            pending_source_losses_.push_back(signal);
        }
    } catch (...) {
        stop_requested_ = true;
        on_diagnostic("source-loss bridge failure; stopping owner");
    }
    notify_owner();
}

void GatewayOwner::on_diagnostic(const char *) noexcept
{
    // Diagnostics are intentionally local to the gateway process. They never
    // become ServiceError or authority transitions.
}

void GatewayOwner::process_bridge_snapshot_and_check_time(
    GatewayTimeMs owner_now_ms)
{
    std::deque<ControlSourceId> connections;
    std::deque<SourceLostSignal> losses;
    std::deque<RemoteEnvelope> messages;
    {
        std::lock_guard<std::mutex> lock(bridge_mutex_);
        connections.swap(pending_connections_);
        losses.swap(pending_source_losses_);
        messages.swap(pending_inbound_);
        pending_inbound_bytes_ = 0U;
    }

    // Preserve the safety order for this finite snapshot: source loss, live
    // source registration, then all inbound envelopes captured at the start
    // of the phase. Work arriving during processing stays for the next phase.
    for (const auto &signal : losses) {
        core_.source_lost(signal, owner_now_ms);
    }
    for (const auto source : connections) {
        // A connection may close before the owner observes its registration.
        // Do not resurrect that source after its loss edge.
        if (tcp_.current_source_id() != source) {
            on_diagnostic("stale source registration ignored");
            continue;
        }
        core_.source_connected(source);
    }
    for (const auto &envelope : messages) {
        core_.process(envelope, owner_now_ms);
    }

    // The bridge lock defines the lease boundary. A worker that starts
    // decoding after this observation timestamps its envelope at or after
    // check_now; a worker already inside synchronous delivery prevents this
    // check until its callback has completed.
    {
        std::unique_lock<std::mutex> lock(bridge_mutex_);
        const auto check_now = monotonic_now();
        bool pending_timely_heartbeat = false;
        if (core_.authority_state() == AuthorityState::Owned &&
            core_.source_present()) {
            for (const auto &envelope : pending_inbound_) {
                if (envelope.source == core_.current_source_id() &&
                    envelope.message.kind ==
                        RbrpMessageKind::ControlHeartbeat &&
                    envelope.received_at_ms < core_.lease_deadline_ms()) {
                    pending_timely_heartbeat = true;
                    break;
                }
            }
        }
        if (!pending_connections_.empty() ||
            !pending_source_losses_.empty() || tcp_.inbound_delivery_in_flight() ||
            pending_timely_heartbeat) {
            return;
        }
        core_.check_time(check_now);
    }
}

void GatewayOwner::iteration()
{
    process_bridge_snapshot_and_check_time(monotonic_now());

    if (application_.session_state() !=
        GatewayApplicationSessionState::ReopenRequired) {
        const auto result = application_.run_once();
        core_.consume_application_run_result(result, monotonic_now());
    }

    process_bridge_snapshot_and_check_time(monotonic_now());
}

void GatewayOwner::owner_loop()
{
    for (;;) {
        {
            std::unique_lock<std::mutex> lock(bridge_mutex_);
            const auto work_available =
                [this] {
                    return stop_requested_ ||
                           !pending_connections_.empty() ||
                           !pending_source_losses_.empty() ||
                           !pending_inbound_.empty();
                };
            if (stop_requested_) {
                break;
            }
            if (!work_available() &&
                application_.session_state() ==
                    GatewayApplicationSessionState::ReopenRequired) {
                bridge_changed_.wait_for(
                    lock, std::chrono::milliseconds(kOwnerWaitMs),
                    work_available);
            }
            if (stop_requested_) {
                break;
            }
        }
        iteration();
    }

    if (!shutdown_consumed_) {
        shutdown_consumed_ = true;
        core_.shutdown(monotonic_now());
    }
}

int GatewayOwner::start()
{
    {
        std::lock_guard<std::mutex> lock(bridge_mutex_);
        if (started_) {
            return EALREADY;
        }
        stop_requested_ = false;
        shutdown_consumed_ = false;
        inbound_messages_high_water_ = 0U;
        inbound_payload_bytes_high_water_ = 0U;
        inbound_overflow_count_ = 0U;
        started_ = true;
    }

    try {
        owner_thread_ = std::thread([this] { owner_loop(); });
    } catch (...) {
        std::lock_guard<std::mutex> lock(bridge_mutex_);
        started_ = false;
        return ENOMEM;
    }

    const int error = tcp_.start();
    if (error != 0) {
        {
            std::lock_guard<std::mutex> lock(bridge_mutex_);
            stop_requested_ = true;
        }
        notify_owner();
        if (owner_thread_.joinable()) {
            owner_thread_.join();
        }
        {
            std::lock_guard<std::mutex> lock(bridge_mutex_);
            started_ = false;
        }
        return error;
    }
    return 0;
}

void GatewayOwner::stop() noexcept
{
    {
        std::lock_guard<std::mutex> lock(bridge_mutex_);
        if (!started_ && !owner_thread_.joinable()) {
            tcp_.stop();
            return;
        }
        stop_requested_ = true;
    }
    notify_owner();
    if (owner_thread_.joinable()) {
        owner_thread_.join();
    }
    tcp_.stop();
    {
        std::lock_guard<std::mutex> lock(bridge_mutex_);
        started_ = false;
    }
}

GatewayOwner::~GatewayOwner()
{
    stop();
}

std::uint16_t GatewayOwner::bound_port() const noexcept
{
    return tcp_.bound_port();
}

bool GatewayOwner::started() const noexcept
{
    std::lock_guard<std::mutex> lock(bridge_mutex_);
    return started_;
}

GatewayOwnerStats GatewayOwner::stats() const noexcept
{
    GatewayOwnerStats result;
    {
        std::lock_guard<std::mutex> lock(bridge_mutex_);
        result.inbound_messages_high_water = inbound_messages_high_water_;
        result.inbound_payload_bytes_high_water =
            inbound_payload_bytes_high_water_;
        result.inbound_overflow_count = inbound_overflow_count_;
    }
    result.tcp = tcp_.stats();
    return result;
}

} // namespace robobeetle::gateway
