# Raspberry Pi (Ubuntu) – eth0 setup for gimbal

Use this when the gimbal is on `192.168.144.x` and you need eth0 on the Pi to be on the same subnet.

## 1. Check current eth0

```bash
# Is the interface up and what address does it have?
ip addr show eth0

# Is it managed by NetworkManager or systemd-networkd?
ip link show eth0
ls /etc/netplan/
cat /etc/netplan/*.yaml
```

Note: On Ubuntu for Raspberry Pi, **Netplan** is usually used (config in `/etc/netplan/`).

## 2. Static IP on same subnet as gimbal (192.168.144.x)

Edit Netplan (adjust filename to match what you have, e.g. `50-cloud-init.yaml` or `01-netcfg.yaml`):

```bash
sudo nano /etc/netplan/50-cloud-init.yaml
```

Example for **static** IP on eth0 in 192.168.144.x (gimbal at .108):

```yaml
network:
  version: 2
  ethernets:
    eth0:
      addresses:
        - 192.168.144.10/24
      # Optional: set gateway if this network has one
      # gateway4: 192.168.144.1
      # nameservers:
      #   addresses: [8.8.8.8]
```

If you want **DHCP** on eth0 but the gimbal network provides DHCP:

```yaml
network:
  version: 2
  ethernets:
    eth0:
      dhcp4: true
```

Apply and restart networking:

```bash
sudo netplan apply
```

Then check:

```bash
ip addr show eth0
ping -c 2 192.168.144.108
```

## 3. If eth0 is missing or not up

```bash
# List all interfaces
ip link

# Bring eth0 up (no config change)
sudo ip link set eth0 up

# If eth0 doesn’t exist, check if it’s named differently (e.g. end0, enx...)
ip link
```

## 4. Quick connectivity test to gimbal

```bash
# Ping gimbal
ping -c 2 192.168.144.108

# Test RTSP port (if nc available)
nc -zv 192.168.144.108 554
```

If ping fails, fix eth0 IP and routing first; then retry the camera node or `check_gimbal_stream.py`.
