# Passwordless sudo for gimbal Ethernet setup

`buttons/scripts/start_recording.sh` brings up the gimbal Ethernet interface with `ip link` and `ip addr`. Those operations require root. In Elytra Bridge and detached tmux sessions, `sudo` cannot prompt for a password, so configure passwordless sudo for one narrow helper command.

## Allowed helper

Only this executable should be allowed as root without a password:

```text
/usr/local/sbin/drone-gimbal-net-setup.sh
```

The helper accepts an interface and CIDR, defaults to `eth0` and `192.168.144.10/24`, and validates the arguments before running network commands.

## Install on the physical target

From the package root on the drone:

```bash
cd /home/pi/drone_workspace/drone-2026

sudo install -m 755 real/scripts/drone-gimbal-net-setup.sh /usr/local/sbin/drone-gimbal-net-setup.sh

sudo cp real/scripts/sudoers-drone-gimbal-net /etc/sudoers.d/drone-gimbal-net
sudo chmod 440 /etc/sudoers.d/drone-gimbal-net
sudo chown root:root /etc/sudoers.d/drone-gimbal-net

sudo visudo -cf /etc/sudoers.d/drone-gimbal-net
```

Edit `/etc/sudoers.d/drone-gimbal-net` if your SSH user is not `pi`.

## Verify

```bash
sudo -n /usr/local/sbin/drone-gimbal-net-setup.sh eth0 192.168.144.10/24
echo $?
```

Exit code `0` and no password prompt means it worked.

## Overrides

Set these on the target if your hardware differs:

```bash
export ETH_GIMBAL_IF=eth0
export ETH_GIMBAL_IP=192.168.144.10/24
export DRONE_NET_SETUP_SCRIPT=/usr/local/sbin/drone-gimbal-net-setup.sh
```

For static Netplan configuration instead of runtime setup, see `docs/raspberry-pi-eth0-setup.md`.
