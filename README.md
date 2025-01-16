### Setup of Rpi
[Write image to Raspberry Pi ](https://orobotics.sharepoint.com/:w:/s/AMRSweeper/EYyAcXdbdvBDn3S_FSLEkB8BfrLMU5o2DUBJO345Z8hJNw?e=cCnbcv)
- `Config.txt` is for Rpi hardware level
- Script is for software level
  - `Setup_rpi.sh`: It contains ROS 2 and dependencies
    - You need to change the password for `dev` before running the script.
    - Do not contain: foxglove, ssh key, minicom
    - Contain: ROS 2 control, GNSS packages
      
  How to use? Clone script and give permission:
  ```
  chmod +x setup_rpi.sh
  ```
  Run it:
  ```
  sudo ./setup_rpi.sh
  ```
  * During the process, tailscale will provide a link to login. Copy it to browser then `Ctrl C` to continue.
  
#### Other steps
Besides, you need to manually add minicom it will stuck otherwise the bash process.
```
# ------------ Modem ------------ 
# Setup Modem (for 4G modem connection)
sudo apt-get install minicom
sudo minicom -D /dev/ttyUSB2
sudo dhclient -v usb0
```
