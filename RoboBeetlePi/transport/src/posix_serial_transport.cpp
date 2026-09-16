#include "robobeetle/transport/posix_serial_transport.hpp"

#include <algorithm>
#include <cerrno>
#include <fcntl.h>
#include <limits>
#include <termios.h>
#include <unistd.h>

namespace robobeetle::transport {
namespace {

int configure_serial(int fd) noexcept
{
    termios settings{};
    if (::tcgetattr(fd, &settings) != 0) { return errno; }
    ::cfmakeraw(&settings);
    settings.c_iflag &= ~(IXON | IXOFF | IXANY | INPCK);
    settings.c_cflag &= ~(CSIZE | PARENB | PARODD | CSTOPB | CRTSCTS | CMSPAR);
    settings.c_cflag |= CS8 | CLOCAL | CREAD;
    settings.c_cc[VMIN] = 0;
    settings.c_cc[VTIME] = 0;
    if (::cfsetispeed(&settings, B115200) != 0 ||
        ::cfsetospeed(&settings, B115200) != 0) { return errno; }
    if (::tcsetattr(fd, TCSANOW, &settings) != 0) { return errno; }
    return 0;
}

std::size_t syscall_size(std::size_t requested) noexcept
{
    return std::min(requested,
                    static_cast<std::size_t>(std::numeric_limits<ssize_t>::max()));
}

} // namespace

PosixSerialTransport::PosixSerialTransport(std::size_t max_frames, std::size_t max_bytes)
    : tx_queue_(max_frames, max_bytes), read_fn_(::read), write_fn_(::write)
{
}

PosixSerialTransport::~PosixSerialTransport()
{
    close();
}

int PosixSerialTransport::open(const char *device_path) noexcept
{
    if (is_open() || has_pending_tx()) { return EBUSY; }
    accept_new_tx_ = false;
    failed_ = false;
    fatal_errno_ = 0;
    if (device_path == nullptr) { return EINVAL; }
    const int candidate = ::open(device_path, O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
    if (candidate < 0) { return errno; }
    const int error_number = configure_serial(candidate);
    if (error_number != 0) {
        // Linux releases the descriptor even if close reports EINTR. Retrying
        // could close a reused fd; save the configure error and close once.
        ::close(candidate);
        return error_number;
    }
    fd_ = candidate;
    accept_new_tx_ = true;
    return 0;
}

void PosixSerialTransport::close() noexcept
{
    const int owned_fd = fd_;
    fd_ = -1;
    accept_new_tx_ = false;
    failed_ = false;
    fatal_errno_ = 0;
    if (owned_fd >= 0) { ::close(owned_fd); }
}

bool PosixSerialTransport::is_open() const noexcept { return fd_ >= 0; }
int PosixSerialTransport::native_fd() const noexcept { return fd_; }

bool PosixSerialTransport::write(const protocol::Bytes &wire_bytes)
{
    return accepting_new_tx() && tx_queue_.try_accept(wire_bytes);
}

bool PosixSerialTransport::accepting_new_tx() const noexcept
{
    return is_open() && !failed_ && accept_new_tx_;
}

void PosixSerialTransport::set_accept_new_tx(bool accept) noexcept
{
    accept_new_tx_ = accept && is_open() && !failed_;
}

bool PosixSerialTransport::has_pending_tx() const noexcept
{
    return tx_queue_.owned_frames() != 0U;
}

void PosixSerialTransport::drop_pending_tx() noexcept { tx_queue_.clear(); }

IoResult PosixSerialTransport::fail(int error_number) noexcept
{
    failed_ = true;
    accept_new_tx_ = false;
    fatal_errno_ = error_number;
    return {IoStatus::Fatal, 0U, error_number};
}

IoResult PosixSerialTransport::pump_tx()
{
    if (failed_) { return {IoStatus::Fatal, 0U, fatal_errno_}; }
    if (!is_open()) { return fail(EBADF); }
    const auto *frame = tx_queue_.front_frame();
    if (frame == nullptr) { return {}; }
    const auto remaining = tx_queue_.front_remaining();
    if (remaining == 0U) {
        tx_queue_.consume_front(0U);
        return {IoStatus::Progress, 0U, 0};
    }
    for (;;) {
        const auto count = write_fn_(fd_, frame->data() + tx_queue_.front_offset(),
                                     syscall_size(remaining));
        if (count > 0) {
            const auto sent = static_cast<std::size_t>(count);
            if (!tx_queue_.consume_front(sent)) { return fail(EIO); }
            return {IoStatus::Progress, sent, 0};
        }
        if (count == 0) { return {IoStatus::WouldBlock, 0U, 0}; }
        const int error_number = errno;
        if (error_number == EINTR) { continue; }
        if (error_number == EAGAIN || error_number == EWOULDBLOCK) {
            return {IoStatus::WouldBlock, 0U, 0};
        }
        return fail(error_number);
    }
}

IoResult PosixSerialTransport::read_some(std::uint8_t *buffer, std::size_t capacity)
{
    if (failed_) { return {IoStatus::Fatal, 0U, fatal_errno_}; }
    if (!is_open()) { return fail(EBADF); }
    if (capacity == 0U) { return {}; }
    if (buffer == nullptr) { return fail(EINVAL); }
    for (;;) {
        const auto count = read_fn_(fd_, buffer, syscall_size(capacity));
        if (count > 0) { return {IoStatus::Progress, static_cast<std::size_t>(count), 0}; }
        if (count == 0) { return fail(0); }
        const int error_number = errno;
        if (error_number == EINTR) { continue; }
        if (error_number == EAGAIN || error_number == EWOULDBLOCK) {
            return {IoStatus::WouldBlock, 0U, 0};
        }
        return fail(error_number);
    }
}

} // namespace robobeetle::transport
