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
  # Keep all managed windows on desktop 0 so they are visible in the same VNC workspace.
  wmctrl -i -r "$wid" -t 0 2>/dev/null || true
  # RViz/Qt windows can come up with sticky states under Openbox; clear those first.
  wmctrl -i -r "$wid" -b remove,maximized_vert,maximized_horz,fullscreen,hidden,shaded 2>/dev/null || true
  wmctrl -i -r "$wid" -e 0,"$x","$y","$w","$h" 2>/dev/null || true
  # Retry once after a short delay: Openbox sometimes ignores the first geometry set.
  sleep 0.2
  wmctrl -i -r "$wid" -e 0,"$x","$y","$w","$h" 2>/dev/null || return 0
  wmctrl -i -a "$wid" 2>/dev/null || true
}

main_once() {
  local W H
  if ! read -r W H < <(read_screen); then
    return 0
  fi
  local HALF=$((H / 2))
  # Openbox/TigerVNC adds frame/title extents; keep a guard gap so RViz never hides behind Gazebo.
  local STACK_GAP_Y=${STACK_GAP_Y:-28}
  if ((HALF < 200)); then
    return 0
  fi
  if ((STACK_GAP_Y < 0)); then
    STACK_GAP_Y=0
  fi
  if ((STACK_GAP_Y > 120)); then
    STACK_GAP_Y=120
  fi

  local list listx listg gz rv xt
  list=$(wmctrl -l 2>/dev/null || true)
  listx=$(wmctrl -lx 2>/dev/null || true)
  listg=$(wmctrl -lGx 2>/dev/null || true)

  # Prefer WM_CLASS matches (stable) and fall back to title matches.
  # wmctrl -lx format: <id> <desktop> <host> <wm_class> <title...>
  # Match only the WM_CLASS column so "RViz /image_data" xterm titles are not misdetected.
  gz=$(printf '%s\n' "$listx" | awk 'tolower($4) ~ /(gzclient|gazebo)/ {print $1; exit}') || true
  if [[ -z "$gz" ]]; then
    gz=$(printf '%s\n' "$list" | grep -iE 'gazebo' | head -1 | awk '{print $1}') || true
  fi

  rv=$(printf '%s\n' "$listx" | awk 'tolower($4) ~ /(rviz2|rviz)/ && tolower($0) ~ /- rviz/ {print $1; exit}') || true
  if [[ -z "$rv" ]]; then
    rv=$(printf '%s\n' "$listx" | awk 'tolower($4) ~ /(rviz2|rviz)/ {print $1; exit}') || true
  fi
  if [[ -z "$rv" ]]; then
    # RViz main: title has rviz; exclude helper xterm with image_data title.
    rv=$(printf '%s\n' "$list" | grep -i 'rviz' | grep -iv 'image_data' | head -1 | awk '{print $1}') || true
  fi

  # Helper strip is only for the optional xterm wrapper; never infer from title text,
  # because RViz config path includes "image_data" and can be misclassified.
  xt=$(printf '%s\n' "$listx" | awk 'tolower($4) ~ /xterm/ && tolower($0) ~ /image_data/ {print $1; exit}') || true

  local rv_y rv_h
  rv_y=$((HALF + STACK_GAP_Y))
  rv_h=$((H - rv_y))
  if [[ -n "$gz" ]]; then
    place "$gz" 0 0 "$W" $((HALF - (STACK_GAP_Y / 2)))
    # If Gazebo moves itself after tiling, keep RViz strictly below Gazebo's live bottom.
    local gx gy gw gh gz_bottom
    read -r gx gy gw gh <<< "$(printf '%s\n' "$listg" | awk -v id="$gz" '$1==id{print $3, $4, $5, $6; exit}')"
    if [[ -n "${gy:-}" && -n "${gh:-}" ]]; then
      gz_bottom=$((gy + gh))
      if ((gz_bottom + STACK_GAP_Y > rv_y)); then
        rv_y=$((gz_bottom + STACK_GAP_Y))
        rv_h=$((H - rv_y))
      fi
    fi
  fi

  if ((rv_h < 200)); then
    rv_h=200
    rv_y=$((H - rv_h))
    if ((rv_y < HALF)); then
      rv_y=$HALF
    fi
  fi

  if [[ -n "$rv" ]]; then
    place "$rv" 0 "$rv_y" "$W" "$rv_h"
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
