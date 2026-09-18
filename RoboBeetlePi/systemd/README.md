# RoboBeetle Pi Gateway systemd service

This maintenance deployment makes `robobeetle_pi_gateway` start automatically
when the Raspberry Pi boots. It does not change RBRP, Protocol V2, the Gateway
runtime, STM32 firmware, or actuator semantics.

The daemon starts the TCP listener only. The STM32 serial/application session is
still opened by the existing Gateway only after an explicit RBRP
`AcquireControl`. Booting the Pi therefore does not acquire control, replay an
old command, enable a servo, or start motion.

## Security boundary

RBRP v1 has no authentication or TLS. The supplied default binds
`0.0.0.0:47000` so both Ethernet and Wi-Fi interfaces can reach the Gateway.
Use this only on a trusted engineering network. To restrict listening to one
interface, edit `/etc/default/robobeetle-pi-gateway` and set
`ROBOBEETLE_BIND` to that Pi IPv4 address.

TCP port `47000` is the Gateway service port, not a transmission speed. UART
baud rate remains a separate Pi-to-STM32 setting owned by the existing runtime.

## Build, test, install

Before the first service install, stop any foreground/manual
`robobeetle_pi_gateway` process so TCP port 47000 is free. Disconnect the Qt
client before replacing/restarting the Gateway.

From the repository root on the Raspberry Pi:

```sh
cmake -S RoboBeetlePi -B RoboBeetlePi/build-linux -G Ninja -DBUILD_TESTING=ON
cmake --build RoboBeetlePi/build-linux --parallel 4
ctest --test-dir RoboBeetlePi/build-linux --output-on-failure

sudo ./RoboBeetlePi/systemd/install_gateway_service.sh
```

The installer:

- atomically installs the tested executable as `/usr/local/bin/robobeetle_pi_gateway`, so updating a running service never rewrites the live executable inode;
- installs `/etc/systemd/system/robobeetle-pi-gateway.service`;
- creates `/etc/default/robobeetle-pi-gateway` only when it does not already
  exist, so later installs preserve local configuration;
- runs the service as the unprivileged invoking user (normally `pi`);
- enables the service for boot and starts/restarts it immediately.

If the build executable is elsewhere or the service should run as another
account:

```sh
sudo ./RoboBeetlePi/systemd/install_gateway_service.sh \
  --binary /path/to/robobeetle_pi_gateway \
  --user pi
```

Use `--no-start` to install and enable without starting/restarting immediately.

## Runtime configuration

The default configuration is:

```sh
ROBOBEETLE_DEVICE=/dev/serial0
ROBOBEETLE_BIND=0.0.0.0
ROBOBEETLE_PORT=47000
```

After changing `/etc/default/robobeetle-pi-gateway`, apply it with:

```sh
sudo systemctl restart robobeetle-pi-gateway
```

Normal experiments no longer require manually launching the Gateway. After the
Pi boots:

1. open RoboBeetleConsole on Windows;
2. connect to the Pi Ethernet or Wi-Fi IPv4 address on port 47000;
3. complete RBRP Hello;
4. explicitly Acquire control;
5. Release when finished.

The service never auto-Acquires for a client.

## Post-install acceptance

After the one-time install, verify the service before relying on it for experiments:

```sh
systemctl is-enabled robobeetle-pi-gateway
systemctl is-active robobeetle-pi-gateway
systemctl --no-pager --full status robobeetle-pi-gateway
ss -ltnp | grep 47000
```

Expected results are `enabled`, `active`, and a listener on `0.0.0.0:47000` (or the configured bind address). Then reboot the Pi once:

```sh
sudo reboot
```

After reconnecting over SSH, repeat `systemctl is-active` and the `ss` check. RoboBeetleConsole should be able to complete RBRP Hello, but the Gateway must remain `Unowned` until the operator explicitly presses Acquire.

## Operations and diagnostics

```sh
systemctl status robobeetle-pi-gateway
journalctl -u robobeetle-pi-gateway -f
ss -ltnp | grep 47000
```

`ss -ltnp | grep 47000` is diagnostic only. It does not create a listener; it
checks whether a process is already listening on that port.

To stop or disable automatic startup:

```sh
sudo systemctl stop robobeetle-pi-gateway
sudo systemctl disable robobeetle-pi-gateway
```

To remove the installed service while preserving its configuration for possible
reuse:

```sh
sudo systemctl disable --now robobeetle-pi-gateway
sudo rm -f /etc/systemd/system/robobeetle-pi-gateway.service
sudo rm -f /usr/local/bin/robobeetle_pi_gateway
sudo systemctl daemon-reload
```

Remove `/etc/default/robobeetle-pi-gateway` separately only when the local
configuration should also be discarded.

## Updating the installed Gateway

After updating the repository, rebuild and run the Pi regression suite first.
Then rerun the installer. Replacing the binary followed by the installer-managed
restart moves the service to the newly tested executable while preserving
`/etc/default/robobeetle-pi-gateway`.
