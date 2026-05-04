#!/usr/bin/env bash
# Bring up gimbal Ethernet (eth0 on 192.168.144.x). Run as root via:
#   sudo /usr/local/sbin/drone-gimbal-net-setup.sh [IF] [CIDR]
# Defaults: eth0  192.168.144.10/24
#
# Install on the Pi (once):
#   sudo install -m 755 scripts/drone-gimbal-net-setup.sh /usr/local/sbin/drone-gimbal-net-setup.sh
#   sudo install -m 440 scripts/sudoers-drone-gimbal-net /etc/sudoers.d/drone-gimbal-net
#   sudo visudo -cf /etc/sudoers.d/drone-gimbal-net
#
# Replace `pi` in sudoers-drone-gimbal-net with your drone login if needed.

set -euo pipefail

IF="${1:-eth0}"
CIDR="${2:-192.168.144.10/24}"

if ! [[ "$IF" =~ ^[a-zA-Z0-9._-]+$ ]]; then
  echo "drone-gimbal-net-setup.sh: invalid interface name" >&2
  exit 1
fi
if ! [[ "$CIDR" =~ ^[0-9a-fA-F:./%-]+$ ]]; then
  echo "drone-gimbal-net-setup.sh: invalid address/CIDR" >&2
  exit 1
fi

IP_CMD="$(command -v ip)"
if [[ -z "$IP_CMD" ]]; then
  echo "drone-gimbal-net-setup.sh: ip(8) not found in PATH" >&2
  exit 1
fi

"$IP_CMD" link set "$IF" up
"$IP_CMD" addr replace "$CIDR" dev "$IF"
