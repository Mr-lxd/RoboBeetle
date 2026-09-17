#include "robobeetle/gateway/gateway_owner.hpp"

#include <arpa/inet.h>
#include <charconv>
#include <csignal>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <string>
#include <string_view>
#include <system_error>
#include <thread>

#include <chrono>

namespace {

constexpr const char *kDefaultDevice = "/dev/serial0";
constexpr const char *kDefaultBind = "127.0.0.1";
constexpr std::uint16_t kDefaultPort = 47000U;

volatile std::sig_atomic_t stop_requested = 0;

void request_stop(int) noexcept
{
    stop_requested = 1;
}

struct Options {
    std::string device{kDefaultDevice};
    std::string bind{kDefaultBind};
    std::uint16_t port{kDefaultPort};
};

void print_usage(std::ostream &stream)
{
    stream << "Usage: robobeetle_pi_gateway [--device <path>] "
              "[--bind <IPv4-address>] [--port <1..65535>]\n"
              "Defaults: --device /dev/serial0 --bind 127.0.0.1 "
              "--port 47000\n";
}

bool parse_port(std::string_view text, std::uint16_t &port)
{
    if (text.empty()) {
        return false;
    }
    std::uint32_t value = 0U;
    const auto parsed = std::from_chars(text.data(),
                                        text.data() + text.size(), value);
    if (parsed.ec != std::errc{} || parsed.ptr != text.data() + text.size() ||
        value == 0U || value > 65535U) {
        return false;
    }
    port = static_cast<std::uint16_t>(value);
    return true;
}

bool parse_options(int argc, char **argv, Options &options, bool &help,
                   std::string &error)
{
    for (int index = 1; index < argc; ++index) {
        const std::string_view argument(argv[index]);
        if (argument == "--help" || argument == "-h") {
            help = true;
            continue;
        }

        if (argument == "--device" || argument == "--bind" ||
            argument == "--port") {
            if (index + 1 >= argc || argv[index + 1][0] == '\0') {
                error = std::string(argument) + " requires a value";
                return false;
            }
            const std::string value(argv[++index]);
            if (argument == "--device") {
                options.device = value;
            } else if (argument == "--bind") {
                options.bind = value;
            } else if (!parse_port(value, options.port)) {
                error = "--port must be an integer in the range 1..65535";
                return false;
            }
            continue;
        }

        error = "unknown argument: " + std::string(argument);
        return false;
    }

    if (options.device.empty()) {
        error = "--device must not be empty";
        return false;
    }

    in_addr address{};
    if (::inet_pton(AF_INET, options.bind.c_str(), &address) != 1) {
        error = "--bind must be a valid IPv4 address";
        return false;
    }
    return true;
}

bool is_loopback_address(const std::string &address)
{
    in_addr parsed{};
    if (::inet_pton(AF_INET, address.c_str(), &parsed) != 1) {
        return false;
    }
    return (ntohl(parsed.s_addr) >> 24U) == 127U;
}

} // namespace

int main(int argc, char **argv)
{
    Options options;
    bool help = false;
    std::string error;
    if (!parse_options(argc, argv, options, help, error)) {
        std::cerr << "Argument error: " << error << '\n';
        print_usage(std::cerr);
        return EXIT_FAILURE;
    }
    if (help) {
        print_usage(std::cout);
        return EXIT_SUCCESS;
    }

    if (!is_loopback_address(options.bind)) {
        std::cerr << "WARNING: RBRP v1 has no authentication or TLS.\n"
                     "Trusted engineering network only.\n";
    }

    if (std::signal(SIGINT, request_stop) == SIG_ERR ||
        std::signal(SIGTERM, request_stop) == SIG_ERR) {
        std::cerr << "Failed to install SIGINT/SIGTERM handlers\n";
        return EXIT_FAILURE;
    }

    robobeetle::gateway::GatewayOwner owner(options.device, options.bind,
                                             options.port);
    const int error_number = owner.start();
    if (error_number != 0) {
        std::cerr << "Gateway start failed errno=" << error_number << '\n';
        return EXIT_FAILURE;
    }

    while (stop_requested == 0) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    owner.stop();
    return EXIT_SUCCESS;
}
