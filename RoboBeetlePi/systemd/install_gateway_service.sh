#!/usr/bin/env bash
set -euo pipefail

SERVICE_NAME="robobeetle-pi-gateway.service"
INSTALL_BINARY="/usr/local/bin/robobeetle_pi_gateway"
INSTALL_UNIT="/etc/systemd/system/${SERVICE_NAME}"
INSTALL_CONFIG="/etc/default/robobeetle-pi-gateway"

SCRIPT_DIR="$(CDPATH= cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PI_ROOT="$(CDPATH= cd -- "${SCRIPT_DIR}/.." && pwd)"
SOURCE_BINARY="${PI_ROOT}/build-linux/robobeetle_pi_gateway"
RUN_USER="${SUDO_USER:-}"
START_SERVICE=1

usage() {
    cat <<'EOF'
Usage:
  sudo ./RoboBeetlePi/systemd/install_gateway_service.sh [options]

Options:
  --binary PATH   Gateway executable to install.
                  Default: RoboBeetlePi/build-linux/robobeetle_pi_gateway
  --user USER     Unprivileged account used by the systemd service.
                  Default: SUDO_USER, otherwise "pi" when that account exists.
  --no-start      Install and enable the unit, but do not start/restart it now.
  -h, --help      Show this help.

The installer preserves an existing /etc/default/robobeetle-pi-gateway file.
EOF
}

while (($# > 0)); do
    case "$1" in
    --binary)
        if (($# < 2)); then
            echo "error: --binary requires a path" >&2
            exit 2
        fi
        SOURCE_BINARY="$2"
        shift 2
        ;;
    --user)
        if (($# < 2)); then
            echo "error: --user requires an account name" >&2
            exit 2
        fi
        RUN_USER="$2"
        shift 2
        ;;
    --no-start)
        START_SERVICE=0
        shift
        ;;
    -h|--help)
        usage
        exit 0
        ;;
    *)
        echo "error: unknown option: $1" >&2
        usage >&2
        exit 2
        ;;
    esac
done

if ((EUID != 0)); then
    echo "error: run this installer with sudo/root privileges" >&2
    exit 2
fi

for command in install mv rm sed mktemp systemctl id stat grep tr journalctl; do
    if ! command -v "${command}" >/dev/null 2>&1; then
        echo "error: required command not found: ${command}" >&2
        exit 2
    fi
done

if [[ -z "${RUN_USER}" || "${RUN_USER}" == "root" ]]; then
    if id pi >/dev/null 2>&1; then
        RUN_USER="pi"
    else
        echo "error: could not infer an unprivileged service user; pass --user USER" >&2
        exit 2
    fi
fi

if ! id "${RUN_USER}" >/dev/null 2>&1; then
    echo "error: service user does not exist: ${RUN_USER}" >&2
    exit 2
fi
if [[ ! "${RUN_USER}" =~ ^[a-z_][a-z0-9_-]*[$]?$ ]]; then
    echo "error: unsupported service user name: ${RUN_USER}" >&2
    exit 2
fi

if [[ ! -x "${SOURCE_BINARY}" ]]; then
    echo "error: gateway executable is missing or not executable:" >&2
    echo "       ${SOURCE_BINARY}" >&2
    echo "build it first with:" >&2
    echo "  cmake -S RoboBeetlePi -B RoboBeetlePi/build-linux -G Ninja -DBUILD_TESTING=ON" >&2
    echo "  cmake --build RoboBeetlePi/build-linux --parallel 4" >&2
    exit 2
fi

UNIT_TEMPLATE="${SCRIPT_DIR}/robobeetle-pi-gateway.service.in"
DEFAULT_CONFIG="${SCRIPT_DIR}/robobeetle-pi-gateway.default"
if [[ ! -f "${UNIT_TEMPLATE}" || ! -f "${DEFAULT_CONFIG}" ]]; then
    echo "error: systemd deployment files are incomplete" >&2
    exit 2
fi

tmp_binary="$(mktemp "${INSTALL_BINARY}.tmp.XXXXXX")"
tmp_unit=""
cleanup() {
    if [[ -n "${tmp_binary}" ]]; then
        rm -f "${tmp_binary}"
    fi
    if [[ -n "${tmp_unit}" ]]; then
        rm -f "${tmp_unit}"
    fi
}
trap cleanup EXIT

# Install beside the live binary, then atomically replace its directory entry.
# A currently running service can keep executing the old inode until the
# controlled restart below; updates therefore do not rewrite a live executable.
install -m 0755 "${SOURCE_BINARY}" "${tmp_binary}"
mv -f "${tmp_binary}" "${INSTALL_BINARY}"
tmp_binary=""

if [[ ! -e "${INSTALL_CONFIG}" ]]; then
    install -m 0644 "${DEFAULT_CONFIG}" "${INSTALL_CONFIG}"
    echo "installed default configuration: ${INSTALL_CONFIG}"
else
    echo "preserved existing configuration: ${INSTALL_CONFIG}"
fi

tmp_unit="$(mktemp)"
sed "s|@ROBOBEETLE_USER@|${RUN_USER}|g" "${UNIT_TEMPLATE}" > "${tmp_unit}"
if grep -q '@ROBOBEETLE_USER@' "${tmp_unit}"; then
    echo "error: service template substitution failed" >&2
    exit 2
fi
install -m 0644 "${tmp_unit}" "${INSTALL_UNIT}"

systemctl daemon-reload
systemctl enable "${SERVICE_NAME}"

if [[ -e /dev/serial0 ]]; then
    device_group="$(stat -Lc '%G' /dev/serial0 2>/dev/null || true)"
    if [[ -n "${device_group}" ]] &&
       ! id -nG "${RUN_USER}" | tr ' ' '\n' | grep -Fxq "${device_group}"; then
        echo "warning: ${RUN_USER} is not a member of the /dev/serial0 group (${device_group})." >&2
        echo "         the service can listen on TCP, but AcquireControl may fail to open UART." >&2
    fi
else
    echo "warning: /dev/serial0 does not currently exist." >&2
    echo "         the gateway can still listen, but AcquireControl will fail until UART exists." >&2
fi

if ((START_SERVICE)); then
    systemctl restart "${SERVICE_NAME}"
    sleep 1
    if ! systemctl is-active --quiet "${SERVICE_NAME}"; then
        echo "error: ${SERVICE_NAME} did not become active" >&2
        systemctl --no-pager --full status "${SERVICE_NAME}" >&2 || true
        journalctl -u "${SERVICE_NAME}" -n 40 --no-pager >&2 || true
        exit 1
    fi
    echo "${SERVICE_NAME} is active"
else
    echo "${SERVICE_NAME} installed and enabled; start deferred by --no-start"
fi

echo "binary: ${INSTALL_BINARY}"
echo "config: ${INSTALL_CONFIG}"
echo "service user: ${RUN_USER}"
echo "status: systemctl status ${SERVICE_NAME}"
echo "logs:   journalctl -u ${SERVICE_NAME} -f"
