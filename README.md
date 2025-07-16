
# Repo contains:
- `Config.txt` for Rpi hardware level
- Scripts (only use `rpi_full_setup.sh`currently) for software level
    - You need to change the password for `dev` and `rob` before running the script.  
    - Do not contain: tailscale, foxglove, ssh key, minicom
    - Contains:ROS2 humble, ROS 2 control, GNSS, IMU packages


 
## Ubuntu 22.04 install on RPI
[Write image to Raspberry Pi](https://orobotics.sharepoint.com/:w:/s/AMRSweeper/EYyAcXdbdvBDn3S_FSLEkB8BfrLMU5o2DUBJO345Z8hJNw?e=cCnbcv)


#  Raspberry Pi Full Setup Script (`rpi_full_setup.sh`)

## Purpose

This repository provides **two scripts** to quickly set up and recover a clean ROS 2 development environment on a Raspberry Pi (tested on Ubuntu 22.04, RPI CM4), now supporting **separate developer and production users**.

| Script             | Role                                                                                              |
|---------------------|--------------------------------------------------------------------------------------------------|
| `rpi_full_setup.sh` | Run as **root**, deletes & recreates the `dev` and `rob` users, installs ROS 2 Humble, ROS 2 Control, GNSS/IMU packages, and sets up the dev environment from scratch. |
| `dev_setup.sh`      | Run as **dev** or **rob** user, sets up ROS 2 packages and workspace (no user deletion), useful for re-downloading, rebuilding, or recovering the environment. |

---

## Users Setup and Purpose

| User  | Purpose                                                                |
|-------|------------------------------------------------------------------------|
| `dev` | **Developer user:** for testing, development, debugging, experimental code. |
| `rob` | **Robot/production user:** for running stable production code only.      |

> ⚠️ **Recommended practice:**  
> Use `dev` for all development work and reserve `rob` only for production runtime.

---


##  How to Use

###  Full reset & setup (recommended for fresh system or recovery)

```bash
# Run as root
sudo bash rpi_full_setup.sh

# Follow instruction to setup dev:
su - dev
./dev_setup.sh
```

This will:
- Delete existing `dev` user (if exists)
- Recreate `dev` user with password `password` (default, can be changed)
- Generate `/home/dev/dev_setup.sh` and make it executable

---

###  Only rebuild ROS 2 environment setup (no user change)

```bash
# Switch to dev user
su - dev

# Run setup script
./dev_setup.sh
```

This will:
- Re-download necessary ROS 2 packages
- Rebuild the ROS 2 workspace
- Reinstall udev rules (GNSS, IMU)
- Update `.bashrc` with ROS 2 sourcing

---

## Installed Packages Comparison

| Category            | Installed by `rpi_full_setup.sh` (root -> dev)                                              | Installed by `dev_setup.sh` (dev)                                          |
|---------------------|---------------------------------------------------------------------------------------------|---------------------------------------------------------------------------|
| **System tools**    | curl, git, python3-pip, software-properties-common, can-utils, net-tools, locales, gnupg     | Same                                                                      |
| **ROS 2 core**      | ros-humble-ros-base, ros-dev-tools, python3-rosdep, python3-colcon-common-extensions, ros-humble-ros-environment | Same                                                                      |
| **ROS 2 packages**  | ublox_dgnss, rtcm_msgs, wit_ros2_imu, amr_sweeper_description, ROS2_Control (cloned + built)               | Same                                                                      |
| **Simulation & control dependencies** | ros-humble-ros-ignition, ros-humble-ros2-control, ros-humble-ros2-controllers, ros-humble-ign-ros2-control, ros-humble-xacro, ros-humble-joy, ros-humble-twist-mux, ros-humble-joint-state-broadcaster, ros-humble-joint-state-publisher | Same     
| **udev rules**      | GNSS (ublox), IMU (USB, symlink to /dev/imu_usb)                                            | Same                                                                      |

---

##  Design Rationale

1. Separate root-level system management and dev/rob-level ROS workspaces
2. Provide clear separation between developer (dev) and production (rob) environments
3. Allow easy recreate user & home if corrupted
4. Enable workspace-only rebuilds without touching system users
5. Clearly structured **step-by-step build**, including:
- Stepwise `colcon build` per package group
- Automatic `.bashrc` ROS source setup
- udev rules for GNSS & IMU hardware

---

##  Parameters

- **DEV_USERNAME**: default `dev`  
- **DEV_PASSWORD**: default `password`
- ROB_USERNAME: default rob
- ROB_PASSWORD: default robot
(change these at the top of rpi_full_setup.sh)

---

##  Final Checks

After setup:

```bash
ls -l /dev/imu_usb        # Check IMU symlink
colcon list               # Check workspace packages
ros2 launch amr_sweeper_bringup bringup.launch.py use_sim_time:=true  # Launch simulation
```

---

## Notes

- This setup is intended for **clean Ubuntu 22.04 installs on Raspberry Pi CM4**.
- For other hardware, adjust the udev rules accordingly.
- Remember to **change the default password** for security in production use.

