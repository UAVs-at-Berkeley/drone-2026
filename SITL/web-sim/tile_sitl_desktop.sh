#!/usr/bin/env bash
# Stack Gazebo (top) and RViz (bottom) on the VNC desktop. Screen size from xdpyinfo.
# xterm (title contains image_data) is moved to a small strip so it is not under Gazebo/RViz.

set -eo pipefail

export DISPLAY="${DISPLAY:-:0}"
export XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR:-/tmp/runtime-${USER:-user}}"
mkdir -p "$XDG_RUNTIME_DIR" 2>/dev/null || true

command -v wmctrl >/dev/null 2>&1 || exit 0
command -v xdpyinfo >/dev/null 2>&1 || exit 0

read_screen() {
  local s W H
  s=$(xdpyinfo 2>/dev/null | sed -n 's/.*dimensions: *\([0-9]*\)x\([0-9]*\).*/\1 \2/p' | head -1)
  read W H <<< "$s"
  W="${W//[^0-9]/}"
  H="${H//[^0-9]/}"
  if [[ -z "$W" || -z "$H" || "$W" -lt 200 || "$H" -lt 200 ]]; then
    return 1
  fi
  echo "$W" "$H"
  return 0
}

# wmctrl -e: 0,x,y,w,h
place() {
  local wid=$1
  local x=$2
  local y=$3
  local w=$4
  local h=$5
  [[ -n "$wid" ]] || return 0
  # Some apps (notably RViz/OpenGL windows) start maximized; remove that state first
  # or wmctrl geometry requests can be ignored by the WM.
  wmctrl -i -r "$wid" -b remove,maximized_vert,maximized_horz 2>/dev/null || true
  wmctrl -i -r "$wid" -e 0,"$x","$y","$w","$h" 2>/dev/null || return 0
  wmctrl -i -a "$wid" 2>/dev/null || true
}

main_once() {
  local W H
  if ! read -r W H < <(read_screen); then
    return 0
  fi
  local HALF=$((H / 2))
  if ((HALF < 200)); then
    return 0
  fi

  local list listx gz rv xt
  list=$(wmctrl -l 2>/dev/null || true)
  listx=$(wmctrl -lx 2>/dev/null || true)

  # Prefer WM_CLASS matches (stable) and fall back to title matches.
  gz=$(printf '%s\n' "$listx" | grep -iE '\bgzclient\b|\bgazebo\b' | head -1 | awk '{print $1}') || true
  if [[ -z "$gz" ]]; then
    gz=$(printf '%s\n' "$list" | grep -iE 'gazebo' | head -1 | awk '{print $1}') || true
  fi

  rv=$(printf '%s\n' "$listx" | grep -iE '\brviz2\b|\brviz\b' | head -1 | awk '{print $1}') || true
  if [[ -z "$rv" ]]; then
    # RViz main: title has rviz; exclude helper xterm with image_data title.
    rv=$(printf '%s\n' "$list" | grep -i 'rviz' | grep -iv 'image_data' | head -1 | awk '{print $1}') || true
  fi

  xt=$(printf '%s\n' "$listx" | grep -iE '\bxterm\b' | grep -i 'image_data' | head -1 | awk '{print $1}') || true
  if [[ -z "$xt" ]]; then
    xt=$(printf '%s\n' "$list" | grep 'image_data' | head -1 | awk '{print $1}') || true
  fi

  if [[ -n "$gz" ]]; then
    place "$gz" 0 0 "$W" "$HALF"
  fi

  if [[ -n "$rv" ]]; then
    place "$rv" 0 "$HALF" "$W" $((H - HALF))
  fi

  if [[ -n "$xt" && -n "$rv" && "$xt" == "$rv" ]]; then
    xt=""
  fi
  if [[ -n "$xt" && "$xt" == "$gz" ]]; then
    xt=""
  fi
  if [[ -n "$xt" ]]; then
    local th=120
    local xtw=380
    local xty=$((H - th - 4))
    if ((xty < HALF + 4)); then
      xty=$((HALF + 4))
    fi
    place "$xt" 0 "$xty" "$xtw" "$th"
  fi
}

while true; do
  main_once || true
  sleep 4
done
