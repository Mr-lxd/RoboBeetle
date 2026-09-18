# RoboBeetle Pi Gateway systemd deployment

This deployment keeps the existing gateway runtime unchanged. It installs the
Linux gateway binary at a stable path and starts it automatically at boot.

## Normal remote-control mode

The service runs:

```text
/usr/local/bin/robobeetle_pi_gateway
  --device /dev/serial0
  --bind 0.0.0.0
  --port 47000
```

Starting the service does **not** acquire robot control. The Windows Qt console
must still connect and explicitly acquire authority before actuator commands are
accepted.

Install or update from the repository:

```bash
cd ~/RoboBeetle
bash RoboBeetlePi/deploy/systemd/install.sh
```

An alternate existing CMake build directory may be passed as the first
argument.

Runtime settings live in:

```text
/etc/default/robobeetle-pi-gateway
```

The installer creates that file only when it does not already exist.

Useful checks:

```bash
systemctl status robobeetle-pi-gateway
journalctl -u robobeetle-pi-gateway -f
ss -ltnp | grep 47000
```

The `ss` command is only a diagnostic check. It does not start or configure
the TCP listener.

## APC / direct maintenance fallback

The Raspberry Pi gateway and the APC/USART1 direct host must not be used as
simultaneous control owners.

Before launching the Qt console with `--direct` or `--direct-serial`:

```bash
sudo systemctl stop robobeetle-pi-gateway
```

After APC maintenance is finished:

```bash
sudo systemctl start robobeetle-pi-gateway
```

A manual `systemctl stop` is not treated as a service failure, so
`Restart=on-failure` does not immediately restart it.

## Changing the bind address, port, or serial device

Edit:

```bash
sudo nano /etc/default/robobeetle-pi-gateway
```

Then restart:

```bash
sudo systemctl restart robobeetle-pi-gateway
```

Defaults are:

```text
ROBOBEETLE_DEVICE=/dev/serial0
ROBOBEETLE_BIND=0.0.0.0
ROBOBEETLE_PORT=47000
```

Binding to `0.0.0.0` lets the same gateway accept connections through any
active Pi IPv4 interface, such as Ethernet and Wi-Fi.

## Removal

```bash
sudo systemctl disable --now robobeetle-pi-gateway
sudo rm -f /etc/systemd/system/robobeetle-pi-gateway.service
sudo rm -f /etc/default/robobeetle-pi-gateway
sudo rm -f /usr/local/bin/robobeetle_pi_gateway
sudo systemctl daemon-reload
```
