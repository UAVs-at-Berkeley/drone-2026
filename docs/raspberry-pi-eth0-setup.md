# Raspberry Pi eth0 setup for the gimbal

Use this when the gimbal is on `192.168.144.x` and the Pi needs `eth0` on the same subnet.

## Check the interface

```bash
ip addr show eth0
ip link show eth0
ls /etc/netplan/
```

On Ubuntu for Raspberry Pi, Netplan configuration is usually under `/etc/netplan/`.

## Static IP example

Edit the active Netplan file, for example:

```bash
sudo nano /etc/netplan/50-cloud-init.yaml
```

Example static configuration:

```yaml
network:
  version: 2
  ethernets:
    eth0:
      addresses:
        - 192.168.144.10/24
```

Apply and test:

```bash
sudo netplan apply
ip addr show eth0
ping -c 2 192.168.144.108
```

## Runtime setup from Drone 2026 scripts

The Elytra package also supports runtime interface setup from `buttons/scripts/start_recording.sh`. For that path, install the helper documented in `docs/passwordless-sudo-gimbal-net.md`.

The helper source lives at:

```text
real/scripts/drone-gimbal-net-setup.sh
```

The sudoers template lives at:

```text
real/scripts/sudoers-drone-gimbal-net
```

## Connectivity checks

```bash
ping -c 2 192.168.144.108
nc -zv 192.168.144.108 554
```

If ping fails, fix eth0 IP and routing first, then retry the gimbal camera node or `check_gimbal_stream.py`.
