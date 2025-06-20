### Setup of Rpi


```
chmod +x rpi_full_setup.sh
sudo ./rpi_full_setup.sh
```
This repo needs to be update. `rpi_full_setup` is the latest one. It will first create user `dev` and then run scripts in `dev` to install ROS2 and dependencies.
### Create User & run script
Please run this two lines one by one:


![image](https://github.com/user-attachments/assets/b6c74c41-43a5-4c67-9e4b-d39052ed00d8)

```
chmod +x dev_setup.sh
./dev_setup.sh
```


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
Besides, you need to manually add minicom, otherwise it will stuck the bash process.
```
# ------------ Modem ------------ 
# Setup Modem (for 4G modem connection)
sudo apt-get install minicom
sudo minicom -D /dev/ttyUSB2
sudo dhclient -v usb0
```


