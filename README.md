# Repo contains:
- `boot_configs/rpi/cm4/config_cm4.txt` and `boot_configs/rpi/cm5/config_cm5.txt` — Raspberry Pi boot configuration for CM4/CM5.
- `scripts/` — software setup scripts:
  - `scripts/os_provision/full_system_setup.sh` — run as root; creates `dev`/`rob` users and generates `/home/<user>/dev_setup.sh`.
  - `scripts/os_provision/legacy_initial_setup.sh` — legacy, kept for reference.
  - `scripts/ros/workspace_bootstrap.sh` — run as `dev` or `rob`; clones, builds, sets udev rules and bashrc.
  - `scripts/ros/select_workspace.sh` — source to interactively activate an existing ROS 2 workspace.
  - `scripts/network/tailscale_autostart_setup.sh` — configure Tailscale to auto-start and auto-connect.
- Before running, change default passwords for `dev` and `rob` (see Parameters).

## Ubuntu 22.04 install on RPI
[Write image to Raspberry Pi](https://orobotics.sharepoint.com/:w:/s/AMRSweeper/EYyAcXdbdvBDn3S_FSLEkB8BfrLMU5o2DUBJO345Z8hJNw?e=cCnbcv)

# Full System Setup (`scripts/os_provision/full_system_setup.sh`)

## Purpose

Two main workflows are provided:
- `scripts/os_provision/full_system_setup.sh` — run as root to provision the system and create `dev`/`rob` users.
- `scripts/ros/workspace_bootstrap.sh` — run as `dev` or `rob` to set up or rebuild the ROS 2 workspace only.

Note: use `dev` for development and `rob` for production/runtime.

## How to Use

### Full reset & setup (recommended for fresh system or recovery)

```bash
# Run as root
sudo bash scripts/os_provision/full_system_setup.sh

# Then switch to dev and run the generated script at /home/dev/dev_setup.sh
su - dev
./dev_setup.sh
```

What it does:
- Recreates `dev` and `rob` users (with default passwords; change them).
- Installs ROS 2 Humble, ROS 2 Control, GNSS/IMU dependencies.
- Generates `/home/<user>/dev_setup.sh` for both users.

### Only rebuild ROS 2 environment (no user change)

```bash
# Option A: if /home/dev/dev_setup.sh exists
su - dev
./dev_setup.sh

# Option B: run repo script directly (dev or rob)
bash scripts/ros/workspace_bootstrap.sh
```

This will:
- Re-download necessary ROS 2 packages
- Rebuild the ROS 2 workspace
- Reinstall udev rules (GNSS, IMU)
- Update `.bashrc` with ROS 2 sourcing

### Use select_workspace.sh

```
# This script must be sourced
source scripts/ros/select_workspace.sh
```

## Parameters

- DEV_USERNAME: default `dev`
- DEV_PASSWORD: default `password`
- ROB_USERNAME: default `rob`
- ROB_PASSWORD: default `robot`
(set at the top of `scripts/os_provision/full_system_setup.sh`)

## Final Checks

After setup:

```bash
ls -l /dev/imu_usb        # Check IMU symlink
colcon list               # Check workspace packages
ros2 launch amr_sweeper_bringup bringup.launch.py use_sim_time:=true  # Launch simulation
```

## Zenoh ROS2DDS (RPi/Jetson)

This repository includes optional tooling to bridge ROS 2 topics via Zenoh on endpoint devices (RPi/Jetson).

Directory: `zenoh_ros2dds/`

- `zenoh_ros2dds/setup_ros2dds_zenoh.sh` — installs the Zenoh ROS2DDS bridge (auto-detects ARM arch).
- `zenoh_ros2dds/bridge-rpi.json5` — minimal client configuration (edit IP and topics).
- `zenoh_ros2dds/install_service.sh` — installs a systemd unit to auto-start the bridge at boot.

Quick start:

```bash
# 1) Install bridge binary (run as your device user, e.g., dev)
bash zenoh_ros2dds/setup_ros2dds_zenoh.sh

# 2) Copy and edit the config
mkdir -p "$HOME/zenoh_ros2dds"
cp zenoh_ros2dds/bridge-rpi.json5 "$HOME/zenoh_ros2dds/bridge-rpi.json5"
$EDITOR "$HOME/zenoh_ros2dds/bridge-rpi.json5"  # set your zenoh router IP and topics

# 3) Run the bridge manually
ros2dds -c "$HOME/zenoh_ros2dds/bridge-rpi.json5"

# 4) Optional: install as a systemd service (autostart)
# Usage: sudo bash zenoh_ros2dds/install_service.sh [username] [config_path]
sudo bash zenoh_ros2dds/install_service.sh dev "$HOME/zenoh_ros2dds/bridge-rpi.json5"
# Manage service:
sudo systemctl start zenoh-ros2dds
sudo systemctl status zenoh-ros2dds
sudo systemctl enable zenoh-ros2dds
```

## Zenoh topology (RPi/Jetson ↔ PC)

```
[Jetson/RPi]
  ┌─────────────────────┐
  │ ROS 2 Nodes         │
  │ IMU / GNSS / etc    │
  └────────┬────────────┘
           │ ROS 2 Topic
           ▼
     zenoh-bridge-ros2dds
           │ Zenoh pub
           ▼
         Zenoh Router (zenohd)
           ▲
     zenoh-bridge-ros2dds
           │ ROS 2 Sub
  ┌────────┴────────────┐
  │ PC ROS 2 in Docker  │
  │ RViz / Localization │
  └─────────────────────┘
```

### How to set `connect.endpoints`

Edit the JSON5 config used by the bridge (for endpoints, `zenoh_ros2dds/bridge-rpi.json5`; for PC, `zenoh_ros2dds/bridge-pc.json5`).

Examples:

```json5
// Connect to a router running on the simulation PC at 192.168.1.100
connect: { endpoints: ["tcp/192.168.1.100:7447"] }

// Connect to a local router on the same machine (PC side)
connect: { endpoints: ["tcp/localhost:7447"] }

// Multiple endpoints (the bridge will try them in order)
connect: { endpoints: ["tcp/192.168.1.100:7447", "tcp/192.168.1.101:7447"] }
```

Alternatively, you can override the router address at runtime with the unified launcher (see below) using `--router tcp/<IP>:7447`.

### Unified launcher

Use a single script on both endpoint and PC. It auto-detects role by CPU arch (override with `ZENOH_ROLE`).

```bash
# Endpoint (RPi/Jetson) or PC — same command
bash zenoh_ros2dds/launch_zenoh_bridge.sh --router tcp/192.168.1.100:7447

# Options:
#   --config <path>   : use a custom config file
#   --router <addr>   : override connect.endpoints (e.g., tcp/192.168.1.100:7447)
#   --no-router       : on PC, do not auto-start zenohd
#   ZENOH_ROLE        : endpoint | simulation_pc (override role detection)
```

Behavior:

- Endpoint (RPi/Jetson): runs `ros2dds -c $HOME/zenoh_ros2dds/bridge-rpi.json5` (or `--config`), optionally overriding router.
- Simulation PC: starts `zenohd -l tcp/0.0.0.0:7447` unless `--no-router`, then runs `ros2dds -c $HOME/zenoh_ros2dds/bridge-pc.json5`.

Notes:

- Architecture is auto-detected (`aarch64-unknown-linux-gnu` for Jetson 64-bit,
  `armv7-unknown-linux-gnueabihf` for 32-bit). Adjust the script if your platform differs.
- Update `connect.endpoints` and `allow.topics` in the JSON5 config for your deployment.
