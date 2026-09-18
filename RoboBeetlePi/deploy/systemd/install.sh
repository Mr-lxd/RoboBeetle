#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
pi_root="$(cd -- "$script_dir/../.." && pwd)"
build_dir="${1:-$pi_root/build-linux}"
service_name="robobeetle-pi-gateway.service"
service_src="$script_dir/$service_name"
env_src="$script_dir/robobeetle-pi-gateway.env"
env_dst="/etc/default/robobeetle-pi-gateway"

if [[ "$(uname -s)" != "Linux" ]]; then
    echo "error: this installer is only supported on Linux" >&2
    exit 1
fi

for tool in cmake systemctl install pgrep; do
    if ! command -v "$tool" >/dev/null 2>&1; then
        echo "error: required tool not found: $tool" >&2
        exit 1
    fi
done

if [[ "${EUID}" -eq 0 ]]; then
    sudo_cmd=()
else
    if ! command -v sudo >/dev/null 2>&1; then
        echo "error: sudo is required when not running as root" >&2
        exit 1
    fi
    sudo_cmd=(sudo)
fi

if [[ ! -f "$build_dir/CMakeCache.txt" ]]; then
    cmake -S "$pi_root" -B "$build_dir" -DBUILD_TESTING=ON
fi

cmake --build "$build_dir" --target robobeetle_pi_gateway \
    --parallel "${ROBOBEETLE_BUILD_JOBS:-4}"

# Stop an older managed instance before replacing the executable. A manual
# gateway process is intentionally not killed because that could hide an
# operator mistake during APC/direct maintenance.
"${sudo_cmd[@]}" systemctl stop "$service_name" 2>/dev/null || true
if pgrep -f '(^|/)robobeetle_pi_gateway([[:space:]]|$)' >/dev/null 2>&1; then
    echo "error: a manually started robobeetle_pi_gateway is still running" >&2
    echo "stop it first, then rerun this installer" >&2
    exit 1
fi

"${sudo_cmd[@]}" cmake --install "$build_dir" --prefix /usr/local
"${sudo_cmd[@]}" install -m 0644 "$service_src" "/etc/systemd/system/$service_name"

if ! "${sudo_cmd[@]}" test -e "$env_dst"; then
    "${sudo_cmd[@]}" install -m 0644 "$env_src" "$env_dst"
fi

if command -v systemd-analyze >/dev/null 2>&1; then
    systemd-analyze verify "$service_src"
fi

"${sudo_cmd[@]}" systemctl daemon-reload
"${sudo_cmd[@]}" systemctl enable --now "$service_name"

if ! "${sudo_cmd[@]}" systemctl is-active --quiet "$service_name"; then
    "${sudo_cmd[@]}" journalctl -u "$service_name" -n 50 --no-pager || true
    echo "error: $service_name did not become active" >&2
    exit 1
fi

"${sudo_cmd[@]}" systemctl --no-pager --full status "$service_name"
