
#  Raspberry Pi Full Setup Script (`rpi_full_setup.sh`)

##  Purpose

This repository provides **two scripts** to quickly set up and recover a clean ROS 2 development environment on a Raspberry Pi (tested on Ubuntu 22.04, RPI CM4):

| Script             | Role                                                                                              |
|---------------------|--------------------------------------------------------------------------------------------------|
| `rpi_full_setup.sh` | Run as **root**, deletes & recreates the `dev` user, installs ROS 2 Humble, GNSS/IMU packages, and sets up the dev environment from scratch. |
| `dev_setup.sh`      | Run as **dev user**, only sets up ROS 2 packages and workspace (no user deletion), useful for re-downloading, rebuilding, or recovering the environment. |

---

##  How to Use

###  Full reset & setup (recommended for fresh system or recovery)

```bash
# Run as root
sudo bash rpi_full_setup.sh

# Follow final instruction:
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
| **ROS 2 packages**  | ublox_dgnss, rtcm_msgs, wit_ros2_imu, amr_sweeper_description (cloned + built)               | Same                                                                      |
| **udev rules**      | GNSS (ublox), IMU (USB, symlink to /dev/imu_usb)                                            | Same                                                                      |

---

##  Design Rationale

 Separate **root-level system management** and **dev-level ROS workspace**  
 Easy to **recreate the dev user & home** if corrupted  
 Easy to **rebuild only the workspace** if ROS or packages break  
 Clearly structured **step-by-step build**, including:
- Stepwise `colcon build` per package group
- Automatic `.bashrc` ROS source setup
- udev rules for GNSS & IMU hardware

---

##  Parameters

- **DEV_USERNAME**: default `dev`  
- **DEV_PASSWORD**: default `password`  
(change at the top of `rpi_full_setup.sh` if needed)

---

##  Final Checks

After setup:

```bash
ls -l /dev/imu_usb        # Check IMU symlink
colcon list               # Check workspace packages
```

---

## Notes

- This setup is intended for **clean Ubuntu 22.04 installs on Raspberry Pi CM4**.
- For other hardware, adjust the udev rules accordingly.
- Remember to **change the default password** for security in production use.






[Write image to Raspberry Pi ](https://orobotics.sharepoint.com/:w:/s/AMRSweeper/EYyAcXdbdvBDn3S_FSLEkB8BfrLMU5o2DUBJO345Z8hJNw?e=cCnbcv)
- `Config.txt` is for Rpi hardware level
- Script is for software level
  - `Setup_rpi.sh`: It contains ROS 2 and dependencies
    - You need to change the password for `dev` before running the script.
    - Do not contain: foxglove, ssh key, minicom
    - Contain: ROS 2 control, GNSS packages
