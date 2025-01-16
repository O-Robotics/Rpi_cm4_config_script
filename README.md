### Setup of Rpi
[Write image to Raspberry Pi ](https://orobotics.sharepoint.com/:w:/s/AMRSweeper/EYyAcXdbdvBDn3S_FSLEkB8BfrLMU5o2DUBJO345Z8hJNw?e=cCnbcv)
- `Config.txt` is for Rpi hardware level
- Script is for software level
  - `Setup_rpi.sh`: Contains ROS 2 and dependencies
    - You need to change the password for `dev` before running the script.
    - Do not contain: foxglove, ssh key
    - Contains: ROS 2 control, GNSS packages
      
  How to use? Clone script and give permission:
  ```
  chmod +x setup_rpi.sh
  ```
  Run it:
  ```
  sudo ./setup_rpi.sh
  ```

