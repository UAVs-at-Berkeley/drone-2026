# Passwordless sudo for gimbal `eth0` setup (drone automation)

`start_recording.sh` brings up the gimbal Ethernet interface with `ip link` / `ip addr`, which normally requires **root**. In **detached tmux** (used by the web app), `sudo` cannot prompt for a password, so flights hang or fail until you configure **passwordless sudo for a single helper command**.

## What gets allowed

Only this executable, as root, without a password:

- `/usr/local/sbin/drone-gimbal-net-setup.sh`

It runs the same two operations as before (`ip link set … up`, `ip addr replace …`), with arguments `INTERFACE` and `CIDR` (defaults `eth0` and `192.168.144.10/24`). The script validates arguments to keep the NOPASSWD entry narrow.

## One-time install on the Pi

From your cloned repo on the drone (adjust paths and user):

```bash
cd ~/drone_workspace/drone-2026

sudo install -m 755 scripts/drone-gimbal-net-setup.sh /usr/local/sbin/drone-gimbal-net-setup.sh

sudo cp scripts/sudoers-drone-gimbal-net /etc/sudoers.d/drone-gimbal-net
sudo chmod 440 /etc/sudoers.d/drone-gimbal-net
sudo chown root:root /etc/sudoers.d/drone-gimbal-net

sudo visudo -cf /etc/sudoers.d/drone-gimbal-net
```

Edit `/etc/sudoers.d/drone-gimbal-net` if your login is not `pi` (replace the username in the last line).

## Verify

```bash
sudo -n /usr/local/sbin/drone-gimbal-net-setup.sh eth0 192.168.144.10/24
echo $?
```

Exit code `0` and no prompt means it worked.

## Custom interface or address

Set `ETH_GIMBAL_IF` / `ETH_GIMBAL_IP` when running the flight scripts as today. The helper is called as:

`sudo /usr/local/sbin/drone-gimbal-net-setup.sh "$ETH_GIMBAL_IF" "$ETH_GIMBAL_IP"`

Sudo allows **any arguments** to that single binary. If you want to lock the sudoers line to one interface only, edit `/etc/sudoers.d/drone-gimbal-net` to list the full command with fixed arguments (see `man sudoers`).

## Optional: different install path

If you install the helper somewhere else, set on the drone:

```bash
export DRONE_NET_SETUP_SCRIPT=/path/to/drone-gimbal-net-setup.sh
```

and add a matching `NOPASSWD` line in sudoers for that path.

## Related

- Static eth0 via Netplan (no runtime `sudo`): [raspberry-pi-eth0-setup.md](raspberry-pi-eth0-setup.md)

