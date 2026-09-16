#include "test_support.hpp"
#include "robobeetle/transport/posix_serial_transport.hpp"

#include <array>
#include <cerrno>
#include <cstdlib>
#include <cstring>
#include <dirent.h>
#include <fcntl.h>
#include <iostream>
#include <poll.h>
#include <stdexcept>
#include <string>
#include <termios.h>
#include <type_traits>
#include <unistd.h>

namespace robobeetle::transport::detail {
// Private seam: neither Transport nor LinkCore sees syscall injection.
struct PosixSerialTestAccess {
    static void inject(PosixSerialTransport &transport,
                       ssize_t (*reader)(int, void *, std::size_t),
                       ssize_t (*writer)(int, const void *, std::size_t))
    {
        transport.read_fn_ = reader;
        transport.write_fn_ = writer;
    }
    static const FrameTxQueue &queue(const PosixSerialTransport &transport)
    {
        return transport.tx_queue_;
    }
};
} // namespace robobeetle::transport::detail

namespace rbp2_test { int failures = 0; }

namespace {
using rbp2_test::expect;
using robobeetle::protocol::Bytes;
using robobeetle::transport::IoResult;
using robobeetle::transport::IoStatus;
using robobeetle::transport::PosixSerialTransport;
using Access = robobeetle::transport::detail::PosixSerialTestAccess;

static_assert(!std::is_copy_constructible_v<PosixSerialTransport>);
static_assert(!std::is_move_constructible_v<PosixSerialTransport>);

struct Pty {
    int master{-1};
    std::string path;
    Pty()
    {
        master = ::posix_openpt(O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
        if (master < 0) { throw std::runtime_error("posix_openpt failed"); }
        if (::grantpt(master) != 0 || ::unlockpt(master) != 0) {
            ::close(master);
            throw std::runtime_error("PTY setup failed");
        }
        const char *name = ::ptsname(master);
        if (name == nullptr) {
            ::close(master);
            throw std::runtime_error("ptsname failed");
        }
        path = name;
    }
    ~Pty() { ::close(master); }
    Pty(const Pty &) = delete;
    Pty &operator=(const Pty &) = delete;
};

struct Step { ssize_t count; int error; };
struct Script {
    std::vector<Step> tx;
    std::vector<Step> rx;
    std::size_t tx_index{0U};
    std::size_t rx_index{0U};
    std::size_t rx_offset{0U};
    Bytes incoming;
    Bytes sent;
    std::vector<Bytes> attempts;
};
Script *active = nullptr;

ssize_t fake_write(int, const void *data, std::size_t size)
{
    if (active == nullptr || active->tx_index == active->tx.size()) {
        throw std::runtime_error("unexpected write syscall");
    }
    const auto *bytes = static_cast<const std::uint8_t *>(data);
    active->attempts.emplace_back(bytes, bytes + size);
    const auto step = active->tx[active->tx_index++];
    if (step.count > 0) {
        if (static_cast<std::size_t>(step.count) > size) {
            throw std::runtime_error("bad write script");
        }
        active->sent.insert(active->sent.end(), bytes, bytes + step.count);
    }
    errno = step.error;
    return step.count;
}

ssize_t fake_read(int, void *data, std::size_t size)
{
    if (active == nullptr || active->rx_index == active->rx.size()) {
        throw std::runtime_error("unexpected read syscall");
    }
    const auto step = active->rx[active->rx_index++];
    if (step.count > 0) {
        const auto count = static_cast<std::size_t>(step.count);
        if (count > size || count > active->incoming.size() - active->rx_offset) {
            throw std::runtime_error("bad read script");
        }
        std::memcpy(data, active->incoming.data() + active->rx_offset, count);
        active->rx_offset += count;
    }
    errno = step.error;
    return step.count;
}

struct Fixture {
    Pty pty;
    Script script;
    PosixSerialTransport port;
    explicit Fixture(std::size_t frames = 4U, std::size_t bytes = 32U)
        : port(frames, bytes)
    {
        if (port.open(pty.path.c_str()) != 0) {
            throw std::runtime_error("transport PTY open failed");
        }
        active = &script;
        Access::inject(port, fake_read, fake_write);
    }
    ~Fixture() { active = nullptr; }
};

void result(IoResult actual, IoStatus status, std::size_t bytes, int error,
            const char *message)
{
    expect(actual.status == status && actual.bytes_processed == bytes &&
               actual.error_number == error, message);
}

void state(const PosixSerialTransport &port, const Bytes &front,
           std::size_t offset, std::size_t frames, std::size_t bytes)
{
    const auto &queue = Access::queue(port);
    expect(queue.front_frame() != nullptr && *queue.front_frame() == front &&
               queue.front_offset() == offset && queue.owned_frames() == frames &&
               queue.owned_bytes() == bytes,
           "exact queued frame, offset, frame count and pending bytes");
}

void acceptance_and_fifo()
{
    Fixture f(2U, 5U);
    Bytes a{1, 2, 3};
    const Bytes b{4, 5};
    expect(f.port.accepting_new_tx(), "A: configured port accepts TX");
    expect(f.port.write(a), "A: whole frame accepted");
    a[0] = 99;
    state(f.port, {1, 2, 3}, 0, 1, 3);
    expect(f.script.tx_index == 0, "A: acceptance performs no physical write");
    expect(f.port.write(b), "A: second whole frame accepted");
    expect(!f.port.write({6}), "B: full queue refuses complete frame");
    state(f.port, {1, 2, 3}, 0, 2, 5);
    f.script.tx = {{1, 0}, {2, 0}, {2, 0}};
    result(f.port.pump_tx(), IoStatus::Progress, 1, 0, "C: partial TX count");
    state(f.port, {1, 2, 3}, 1, 2, 4);
    expect(!f.port.write({6}), "B: partial front still occupies frame slot");
    f.port.set_accept_new_tx(false);
    expect(!f.port.write({6}), "gate rejects without touching partial queue");
    state(f.port, {1, 2, 3}, 1, 2, 4);
    result(f.port.pump_tx(), IoStatus::Progress, 2, 0, "D: finishes only A");
    state(f.port, b, 0, 1, 2);
    result(f.port.pump_tx(), IoStatus::Progress, 2, 0, "D: then B");
    expect(f.script.sent == Bytes({1, 2, 3, 4, 5}), "D: no overtaking or loss");
    expect(f.script.attempts == std::vector<Bytes>({{1, 2, 3}, {2, 3}, {4, 5}}),
           "C/D: syscall sees front remainder only");
    expect(!f.port.has_pending_tx(), "completed frames leave empty queue");
    result(f.port.pump_tx(), IoStatus::Idle, 0, 0, "empty pump idle");
    f.port.set_accept_new_tx(true);
    expect(f.port.write({}), "empty encoded byte sequence is an owned frame");
    result(f.port.pump_tx(), IoStatus::Progress, 0, 0, "empty frame retires without syscall");
    expect(!f.port.has_pending_tx(), "empty frame cannot wedge TX queue");

    Fixture byte_full(4U, 4U);
    expect(byte_full.port.write({1, 2, 3}), "byte capacity setup");
    expect(!byte_full.port.write({4, 5}), "B: independent byte capacity rejection");
    state(byte_full.port, {1, 2, 3}, 0, 1, 3);
}

void tx_errors_and_drop()
{
    Fixture f;
    expect(f.port.write({1, 2, 3}) && f.port.write({4, 5}), "TX error setup");
    f.script.tx = {{1, 0}, {-1, EAGAIN}, {-1, EWOULDBLOCK},
                   {-1, EINTR}, {1, 0}, {-1, EIO}};
    result(f.port.pump_tx(), IoStatus::Progress, 1, 0, "partial error setup");
    for (int i = 0; i < 2; ++i) {
        result(f.port.pump_tx(), IoStatus::WouldBlock, 0, 0, "E: backpressure nonfatal");
        state(f.port, {1, 2, 3}, 1, 2, 4);
        expect(f.port.accepting_new_tx(), "E: backpressure keeps gate open");
    }
    result(f.port.pump_tx(), IoStatus::Progress, 1, 0, "F: EINTR retries success");
    expect(f.script.attempts.size() == 5 && f.script.attempts[3] == Bytes({2, 3}) &&
               f.script.attempts[4] == Bytes({2, 3}) && f.script.sent == Bytes({1, 2}),
           "F: EINTR neither consumes nor duplicates");
    state(f.port, {1, 2, 3}, 2, 2, 3);
    result(f.port.pump_tx(), IoStatus::Fatal, 0, EIO, "G: fatal TX errno");
    expect(!f.port.accepting_new_tx() && !f.port.write({6}), "G: fatal closes acceptance gate");
    state(f.port, {1, 2, 3}, 2, 2, 3);
    f.port.set_accept_new_tx(true);
    expect(!f.port.accepting_new_tx(), "fatal session cannot be revived by gate alone");
    result(f.port.pump_tx(), IoStatus::Fatal, 0, EIO, "fatal latch prevents another syscall");
    f.port.drop_pending_tx();
    expect(!f.port.has_pending_tx() && Access::queue(f.port).owned_bytes() == 0 &&
               Access::queue(f.port).front_offset() == 0,
           "H: explicit drop clears partial remainder and all successors");
    expect(f.script.sent == Bytes({1, 2}), "H: drop cannot claim unsent bytes completed");

    Fixture zero;
    zero.script.tx = {{0, 0}};
    expect(zero.port.write({7}), "zero TX setup");
    result(zero.port.pump_tx(), IoStatus::WouldBlock, 0, 0, "zero write returns without spinning");
    state(zero.port, {7}, 0, 1, 1);
}

void rx_results()
{
    Fixture f;
    std::array<std::uint8_t, 8> buffer{};
    f.script.incoming = {0x00, 0xFF, 0x11, 0x13, 0x0D, 0x0A};
    f.script.rx = {{2, 0}, {-1, EAGAIN}, {-1, EWOULDBLOCK},
                   {-1, EINTR}, {4, 0}};
    result(f.port.read_some(buffer.data(), buffer.size()), IoStatus::Progress, 2, 0,
           "I: RX reports exact first byte count");
    Bytes received(buffer.begin(), buffer.begin() + 2);
    const auto unchanged = buffer;
    for (int i = 0; i < 2; ++i) {
        result(f.port.read_some(buffer.data(), buffer.size()), IoStatus::WouldBlock, 0, 0,
               "J: RX EAGAIN/EWOULDBLOCK nonfatal");
        expect(buffer == unchanged && f.port.accepting_new_tx(), "J: RX unchanged");
    }
    result(f.port.read_some(buffer.data(), buffer.size()), IoStatus::Progress, 4, 0,
           "K: RX EINTR retries");
    received.insert(received.end(), buffer.begin(), buffer.begin() + 4);
    expect(received == f.script.incoming, "I/K: exact raw byte order, no filtering");
    result(f.port.read_some(nullptr, 0), IoStatus::Idle, 0, 0,
           "zero capacity does not read or invent EOF");

    for (const Step failure : {Step{-1, EIO}, Step{0, 0}}) {
        Fixture broken;
        expect(broken.port.write({1, 2}), "RX fatal pending TX setup");
        broken.script.rx = {failure};
        result(broken.port.read_some(buffer.data(), buffer.size()), IoStatus::Fatal, 0,
               failure.error, "L: RX fatal or EOF is Fatal");
        expect(!broken.port.accepting_new_tx() && !broken.port.write({3}),
               "L: fatal RX closes gate before return");
        state(broken.port, {1, 2}, 0, 1, 2);
        result(broken.port.pump_tx(), IoStatus::Fatal, 0, failure.error,
               "L: fatal RX also prevents further physical TX");
    }
}

std::size_t fd_count()
{
    DIR *directory = ::opendir("/proc/self/fd");
    if (directory == nullptr) { throw std::runtime_error("cannot count fds"); }
    std::size_t count = 0;
    while (::readdir(directory) != nullptr) { ++count; }
    ::closedir(directory);
    return count;
}

void pty_and_lifecycle()
{
    Pty pty;
    PosixSerialTransport port(4, 32);
    expect(!port.is_open() && port.native_fd() == -1 && !port.accepting_new_tx(),
           "closed construction is deterministic");
    port.set_accept_new_tx(true);
    expect(!port.accepting_new_tx() && !port.write({1}), "closed port cannot accept TX");
    expect(port.open(pty.path.c_str()) == 0, "M: PTY opens");
    const int fd = port.native_fd();
    const int flags = ::fcntl(fd, F_GETFL);
    expect((flags & O_NONBLOCK) != 0 && (flags & O_ACCMODE) == O_RDWR &&
               (::fcntl(fd, F_GETFD) & FD_CLOEXEC) != 0, "M: required descriptor flags");
    termios settings{};
    expect(::tcgetattr(fd, &settings) == 0, "M: termios readable");
    expect(::cfgetispeed(&settings) == B115200 && ::cfgetospeed(&settings) == B115200,
           "M: 115200 both directions");
    expect((settings.c_cflag & CSIZE) == CS8 &&
               (settings.c_cflag & (PARENB | PARODD | CSTOPB | CRTSCTS)) == 0 &&
               (settings.c_cflag & (CLOCAL | CREAD)) == (CLOCAL | CREAD), "M: 8N1 no hardware flow");
    expect((settings.c_iflag & (IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR |
                                ICRNL | IXON | IXOFF | IXANY | INPCK)) == 0 &&
               (settings.c_oflag & OPOST) == 0 &&
               (settings.c_lflag & (ECHO | ECHONL | ICANON | ISIG | IEXTEN)) == 0 &&
               settings.c_cc[VMIN] == 0 && settings.c_cc[VTIME] == 0, "M: raw termios");
    expect(port.open(pty.path.c_str()) == EBUSY && port.native_fd() == fd,
           "second open cannot replace an owned fd");
    expect(port.write({0, 0xFF, 0x11, 0x13, 0x0D, 0x0A}), "real TX accepted");
    result(port.pump_tx(), IoStatus::Progress, 6, 0, "real syscall TX");
    std::array<std::uint8_t, 16> bytes{};
    pollfd ready{pty.master, POLLIN, 0};
    expect(::poll(&ready, 1, 1000) == 1 && (ready.revents & POLLIN) != 0,
           "M: PTY output becomes readable");
    expect(::read(pty.master, bytes.data(), bytes.size()) == 6 &&
               Bytes(bytes.begin(), bytes.begin() + 6) == Bytes({0, 0xFF, 0x11, 0x13, 0x0D, 0x0A}),
           "M: real PTY raw bytes unchanged");
    const Bytes incoming{0x0D, 0x0A, 0x11, 0x13, 0, 0xFF};
    expect(::write(pty.master, incoming.data(), incoming.size()) == 6,
           "real RX setup writes raw bytes");
    ready = {fd, POLLIN, 0};
    expect(::poll(&ready, 1, 1000) == 1 && (ready.revents & POLLIN) != 0,
           "real RX input becomes readable");
    result(port.read_some(bytes.data(), bytes.size()), IoStatus::Progress, 6, 0,
           "real default read syscall");
    expect(Bytes(bytes.begin(), bytes.begin() + 6) == incoming,
           "M: real RX preserves control bytes and ordering");
    expect(port.write({9}), "close pending setup");
    port.close();
    errno = 0;
    expect(::fcntl(fd, F_GETFD) == -1 && errno == EBADF, "close releases owned fd");
    expect(!port.is_open() && port.native_fd() == -1 && !port.accepting_new_tx() &&
               port.has_pending_tx(), "close preserves pending bytes for explicit drop");
    expect(port.open(pty.path.c_str()) == EBUSY, "stale queue cannot enter a new fd session");
    port.drop_pending_tx();
    port.close();
    const auto before = fd_count();
    for (int i = 0; i < 16; ++i) {
        expect(port.open("/dev/null/robobeetle-missing") == ENOTDIR, "N: open failure returned");
        expect(port.open("/dev/null") == ENOTTY, "N: configure failure returned");
        expect(!port.is_open() && port.native_fd() == -1 && !port.accepting_new_tx() &&
                   !port.has_pending_tx(), "N: failure state deterministic");
    }
    expect(fd_count() == before, "N: repeated open/config failures do not leak fds");
    expect(port.open(nullptr) == EINVAL && !port.is_open(), "null path rejected closed");
    expect(port.open(pty.path.c_str()) == 0, "port reusable after configure failure");
    port.close();
    int destroyed_fd = -1;
    {
        PosixSerialTransport owned(1, 4);
        expect(owned.open(pty.path.c_str()) == 0, "destructor test open");
        destroyed_fd = owned.native_fd();
    }
    errno = 0;
    expect(::fcntl(destroyed_fd, F_GETFD) == -1 && errno == EBADF,
           "destructor releases sole fd ownership");
}
} // namespace

int main()
{
    try {
        acceptance_and_fifo();
        tx_errors_and_drop();
        rx_results();
        pty_and_lifecycle();
    } catch (const std::exception &error) {
        std::cerr << "FAIL: " << error.what() << '\n';
        return EXIT_FAILURE;
    }
    if (rbp2_test::failures != 0) { return EXIT_FAILURE; }
    std::cout << "All POSIX serial transport tests passed (A-N)\n";
    return EXIT_SUCCESS;
}
