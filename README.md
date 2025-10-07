# PSN2MAVLink

A lightweight bridge between PosiStageNet (PSN) tracking feeds and MAVLink, enabling UAVs to localise using Ultra Wideband (UWB) anchors and tags. The bridge listens to PSN multicast/unicast traffic, converts tracker poses into MAVLink `VISION_POSITION_ESTIMATE` messages, and streams them to autopilots such as ArduPilot or PX4 in real time.

---

## Highlights

- **Protocol broker** – consumes PSN tracking packets and republishes them as MAVLink pose updates.
- **Flexible transport** – forward MAVLink over UDP _or_ TCP to match your ground station / autopilot link.
- **Frame alignment** – optional NED conversion (PSN Z-up ➜ MAVLink Z-down) for plug-and-play integration.
- **Multicast, unicast & interface selection** – subscribe to theatre networks or local simulators with ease.
- **Tiny footprint** – single C++17 executable with zero runtime dependencies beyond the MAVLink headers.

---

## Architecture

```mermaid
flowchart LR
    PSN[PosiStageNet Source\n(multicast / unicast UDP)] -->|tracker packets| Bridge(psn_to_mavlink.exe)
    subgraph Bridge
        Direction[Coordinate transform\n(Z-up -> NED)]
        Encoder[MAVLink encoder]
    end
    Bridge -->|VISION_POSITION_ESTIMATE| Autopilot[Autopilot / GCS\n(UDP or TCP)]
```

---

## Requirements

| Component | Notes |
|-----------|-------|
| C++ toolchain | C++17-capable compiler (MSVC, Clang, GCC) |
| CMake | 3.10 or newer |
| MAVLink headers | Point `MAVLINK_INCLUDE_DIR` to a `c_library_v2` checkout |
| Python (optional) | Needed for the included PSN simulator & MAVLink test scripts |

> 🛈 **Tip:** On Windows, ensure the Visual Studio developer tools or the MSYS2/MinGW toolchain is available in your shell before building.

---

## Building the bridge

```powershell
# Clone the project
git clone https://github.com/<your-account>/PSN2MAVLink.git
cd PSN2MAVLink

# Configure the build (adjust the absolute path to your c_library_v2 checkout)
cmake -B build -S . -DMAVLINK_INCLUDE_DIR="C:/path/to/mavlink/c_library_v2"

# Compile
cmake --build build --config Release
```

The executable `psn_to_mavlink.exe` (or `psn_to_mavlink` on Linux/macOS) lives inside the `build/` directory.

---

## Command-line reference

```powershell
psn_to_mavlink --group 236.10.10.10 --port 56565 `
               --mav-ip 127.0.0.1 --mav-port 14550 `
               [--mav-transport udp|tcp] [--iface 192.168.0.x] `
               [--sysid 245] [--compid 200] [--tracker <id>] `
               [--no-ned] [--unicast]
```

| Flag | Default | Description |
|------|---------|-------------|
| `--group` | `236.10.10.10` | PSN multicast or unicast group to listen on |
| `--port` | `56565` | UDP port for PSN traffic (watch for Windows excluded ranges) |
| `--iface` | (none) | Bind to a specific NIC when joining multicast |
| `--mav-ip` | `127.0.0.1` | Destination IP for MAVLink telemetry |
| `--mav-port` | `14550` | Destination port (UDP send or TCP connect) |
| `--mav-transport` | `udp` | Choose `udp` (sendto) or `tcp` (persistent socket) |
| `--sysid` / `--compid` | `245` / `200` | MAVLink system & component IDs for outbound messages |
| `--tracker` | all | Forward only the tracker with this PSN ID |
| `--no-ned` | disabled | Keep PSN Z-up coordinates (defaults to converting to NED) |
| `--unicast` | multicast autodetect | Skip multicast join (useful for local simulators) |
| `--help` | – | Print usage summary |

---

## Common workflows

### 1. Loopback test (UDP ➜ ArduPilot SITL)

```powershell
# Terminal 1 – launch PSN simulator (sends circular motion)
python psn_sim.py --unicast --group 127.0.0.1 --port 60000 --tracker 1 --rate 30

# Terminal 2 – bridge PSN to MAVLink UDP
./build/psn_to_mavlink.exe --unicast --group 127.0.0.1 --port 60000 `
    --mav-ip 127.0.0.1 --mav-port 14550 --mav-transport udp
```

Connect your autopilot/GCS (Mission Planner, QGroundControl, MAVProxy) to UDP port `14550` and observe `VISION_POSITION_ESTIMATE` updates.

### 2. Feeding a TCP-only ground station (e.g. MAVProxy TCP port 5762)

```powershell
python psn_sim.py --unicast --group 127.0.0.1 --port 60000 --tracker 1
./build/psn_to_mavlink.exe --group 127.0.0.1 --port 60000 --unicast `
    --mav-ip 127.0.0.1 --mav-port 5762 --mav-transport tcp
```

The bridge will open a TCP socket to `127.0.0.1:5762` and stream MAVLink packets over that connection.

---

## Coordinate frames

- PSN reports **X/Y in the stage plane** and **Z up**.
- MAVLink `VISION_POSITION_ESTIMATE` expects **NED** (Z down). By default the bridge flips the sign of Z.
- Use `--no-ned` if your consumer already expects Z-up data (e.g. for testing in a local visualisation tool).

---

## Simulation & diagnostics helpers

| Script | Purpose |
|--------|---------|
| `psn_sim.py` | Generates synthetic circular motion PSN packets for testing |
| `check_vision_position_estimate.py` | Sends `VISION_POSITION_ESTIMATE` frames over TCP to validate your autopilot setup |

> 🧪 Combine the simulator with the bridge to rehearse network and coordinate handling before deploying to a live UWB anchor network.

---

## Troubleshooting

- **`bind failed: 10013` on Windows** – the OS reserves port ranges (see `netsh interface ipv4 show excludedportrange protocol=udp`). Pick a free port such as `60000`.
- **`IP_ADD_MEMBERSHIP failed`** – ensure the chosen NIC can reach the multicast group and that the address is within 224.0.0.0/4. Use `--unicast` for loopback testing.
- **No MAVLink traffic** – verify firewall rules, confirm the transport (`--mav-transport`), and use a MAVLink sniffer (e.g. `mavutil.mavlink_connection`) to monitor the target endpoint.
- **TCP link drops** – the bridge will exit on hard TCP errors; simply restart once the ground station is reachable.

---

## Roadmap ideas

- Streaming velocity (`VISION_SPEED_ESTIMATE`) when PSN speed chunks are present
- Configurable covariance presets
- Packaging scripts for Windows/macOS releases
- Continuous integration build matrix

Contributions & pull requests are welcome!

---

## License

This project currently has no explicit license. Add a LICENSE file (e.g. MIT, Apache-2.0) before distributing binaries.
