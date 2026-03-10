# Roomba ROS2 Runbook

## Prerequisites

- Roomba connected via USB-serial cable (`/dev/ttyUSB0`)
- USB camera connected (`/dev/video0`)
- Foxglove Studio desktop app installed on your PC

---

## Full session (keyboard control + Foxglove + camera)

Open 4 terminals on the Raspberry Pi.

**Terminal 1 — Roomba nodes**
```bash
cd ~/code/roomba-ros2-work
bazel run //launch:roomba_bringup
```

**Terminal 2 — Foxglove bridge**
```bash
cd ~/code/roomba-ros2-work
./scripts/foxglove_bridge.sh
```

**Terminal 3 — Camera**
```bash
cd ~/code/roomba-ros2-work
./scripts/camera.sh
```

**Terminal 4 — Keyboard control**
```bash
cd ~/code/roomba-ros2-work
bazel run //keyboard_node:keyboard_node
```

Then open Foxglove Studio on your PC and connect:
- **Open connection → Foxglove WebSocket**
- URL: `ws://192.168.100.15:8765`

---

## With recording

Add a 5th terminal after starting the above:

**Terminal 5 — Recording**
```bash
cd ~/code/roomba-ros2-work
./scripts/record.sh              # auto-named: session_YYYYMMDD_HHMMSS
./scripts/record.sh my_session   # custom name → ~/bags/my_session/
```

Stop recording with `Ctrl+C`. Bags are saved to `~/bags/`.

---

## Stub mode (no hardware required)

Same as above but use `roomba_bringup_stub` instead:

**Terminal 1**
```bash
cd ~/code/roomba-ros2-work
bazel run //launch:roomba_bringup_stub
```

Terminals 2–4 are the same. Camera is optional.

---

## Keyboard controls

| Key | Action |
|-----|--------|
| `w` | Forward |
| `s` | Backward |
| `a` | Spin left |
| `d` | Spin right |
| `Space` | Stop |
| `q` | Quit |

---

## Storage management

Check bag sizes:
```bash
du -sh ~/bags/*/
```

Delete a session:
```bash
rm -rf ~/bags/session_YYYYMMDD_HHMMSS
```
