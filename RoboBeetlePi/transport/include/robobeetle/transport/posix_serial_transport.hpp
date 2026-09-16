#pragma once

#if !defined(__linux__)
#error "PosixSerialTransport is supported only on Linux"
#endif

#include "robobeetle/transport/frame_tx_queue.hpp"
#include "robobeetle/transport/transport.hpp"

#include <cstddef>
#include <cstdint>
#include <sys/types.h>

namespace robobeetle::transport {

namespace detail { struct PosixSerialTestAccess; }

enum class IoStatus { Idle, Progress, WouldBlock, Fatal };

struct IoResult {
    IoStatus status{IoStatus::Idle};
    std::size_t bytes_processed{0U};
    // Only meaningful for Fatal. Zero identifies read EOF; otherwise errno.
    int error_number{0};
};

// Single-owner, single-thread/event-loop transport. No protocol processing.
// Descriptor lifetime and queue lifetime are separate: fatal I/O and close()
// retain unsent frames until the caller explicitly drops them.
class PosixSerialTransport final : public Transport {
public:
    PosixSerialTransport(std::size_t max_frames, std::size_t max_bytes);
    ~PosixSerialTransport() override;
    PosixSerialTransport(const PosixSerialTransport &) = delete;
    PosixSerialTransport &operator=(const PosixSerialTransport &) = delete;
    PosixSerialTransport(PosixSerialTransport &&) = delete;
    PosixSerialTransport &operator=(PosixSerialTransport &&) = delete;

    // Returns 0 or an errno value. Requires closed fd and empty queue; EBUSY
    // leaves an existing fd/queue untouched. Other failures leave fd closed.
    // Opens raw 115200 8N1, no flow control, nonblocking, close-on-exec.
    int open(const char *device_path) noexcept;
    void close() noexcept;
    bool is_open() const noexcept;
    // Borrowed descriptor; the caller must not close it or change its flags.
    int native_fd() const noexcept;

    // True transfers a copy of the complete frame into queue ownership, not
    // physical completion. False leaves queue unchanged. Allocation may throw.
    bool write(const protocol::Bytes &wire_bytes) override;
    bool accepting_new_tx() const noexcept;
    // Disabling admission still allows queued TX to drain. Fatal I/O cannot be
    // undone by reopening the gate; close, drop, then open a new fd explicitly.
    void set_accept_new_tx(bool accept) noexcept;
    bool has_pending_tx() const noexcept;
    void drop_pending_tx() noexcept;

    // One front-frame write per call, retrying EINTR only. A short write keeps
    // the remainder; no later frame can overtake it. Empty frames retire with
    // Progress/0 without a syscall. A zero write returns WouldBlock/0.
    IoResult pump_tx();
    // Reads raw bytes into caller storage (capacity==0 is Idle). read()==0 is
    // Fatal/0 per this low-level contract, including a VMIN=0 no-data return.
    // Fatal I/O latches failure and closes admission before returning.
    IoResult read_some(std::uint8_t *buffer, std::size_t capacity);

private:
    friend struct detail::PosixSerialTestAccess;
    IoResult fail(int error_number) noexcept;
    FrameTxQueue tx_queue_;
    int fd_{-1};
    bool accept_new_tx_{false};
    bool failed_{false};
    int fatal_errno_{0};
    // Private deterministic syscall seam; default functions are ::read/write.
    ssize_t (*read_fn_)(int, void *, std::size_t);
    ssize_t (*write_fn_)(int, const void *, std::size_t);
};

} // namespace robobeetle::transport
