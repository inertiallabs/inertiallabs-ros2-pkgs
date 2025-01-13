# inertiallabs_ros2_pkgs
Linux ROS2 driver for [Inertial Labs](https://inertiallabs.com/) products.
Supported devices: INS, IMU-P, AHRS, AHRS-10.

The package is developed based on the official `SDK v0.2` for Linux.

The package is tested on:
- [ROS2 Humble Hawksbill](https://docs.ros.org/en/humble/Installation.html) with Ubuntu 22.04 LTS
- [ROS2 Iron Irwini](https://docs.ros.org/en/iron/Installation/Ubuntu-Install-Debs.html) with Ubuntu 22.04 LTS

## License
* The license for the official SDK is the MIT license.
* The license for the other codes is Apache 2.0 whenever not specified.

## Build
It builds by colcon build-system.
Make sure that your folder with ROS2 packages included in environment variable `ROS_PACKAGE_PATH`.
Clone your package in `<your_work_space>/src`.
And build with the Colcon.

```bash
cd <your_work_space>/src
git clone https://github.com/inertiallabs/inertiallabs-ros2-pkgs.git
cd <your_work_space>
colcon build
```

## Run
```bash
sudo chmod 666 /dev/ttyUSB0
sudo stty -F /dev/ttyUSB0 115200
source install/setup.bash
ros2 run inertiallabs_ins il_ins --ros-args -p ins_url:=serial:/dev/ttyUSB0:115200 -p ins_output_format:=51
```

For ins OPVT2AHR packet via USB serial port:
```bash
ros2 run inertiallabs_ins il_ins --ros-args -p ins_url:=serial:/dev/ttyUSB0:115200 -p ins_output_format:=88
```
For ins OPVT packet via UDP (INS hostname is used):
```bash
ros2 run inertiallabs_ins il_ins --ros-args -p ins_url:=udp:INS-F2001234:23 -p ins_output_format:=82
```
For ins OPVT packet via UDP (INS IP address is used):
```bash
ros2 run inertiallabs_ins il_ins --ros-args -p ins_url:=udp:192.168.0.249:23 -p ins_output_format:=82
```

**Parameters**

`ins_url` (`string`, `default: serial:/dev/ttyUSB0:460800`)
Port the device is connected to. Can be:
- serial:[path to device]:[baudrate]
- tcp:[hostname or address]:[tcp server port]
- udp:[hostname or address]:[udp server port]

Inertial Labs Driver supports serial connection.

`ins_output_format` (`int`, `default: 82`)
The output data format of the INS data according to IL INS ICD.
```
 IL_IMU_Orientation         51  (0x33)
 IL_SENSOR_DATA             80  (0x50)
 IL_OPVT                    82  (0x52)
 IL_MINIMAL_DATA            83  (0x53)
 IL_QPVT                    86  (0x56)
 IL_OPVT2A                  87  (0x57)
 IL_OPVT2AHR                88  (0x58)
 IL_OPVT2AW                 89  (0x59)
 IL_OPVTAD                  97  (0x61)
 MRU_OPVTHSSHR              100 (0x64)
 IL_OPVT_RAWIMU_DATA        102 (0x66)
 IL_OPVT_GNSSEXT_DATA       103 (0x67)
 IL_USER_DEFINED_DATA       149 (0x95)
```

**Published Topics**

Feel free to modify using fields from IL::INSDataStruct

`/Inertial_Labs/sensor_data`
Gyro(x,y,z) , Accelation(x,y,z) , Magnetic (x,y,z) , Temprature , Input Voltage , Pressure , Barometric height.

`/Inertial_Labs/ins_data`
GPS INS Time, GPS IMU Time, Millisecond of the week, Latitude, Longitude, Altitude, Heading , Pitch , Roll, Orientation quaternion, East Velocity, North Velocity, Up Velocity values, Solution status, Position STD, Heading STD, Unit Status.

`/Inertial_Labs/gps_data`
Latitude, Longitude , Altitude , Ground Speed , Track Direction,  Vertical Speed values .

`/Inertial_Labs/gnss_data`
GNSS service Info 1, Info 2, Satellites Used, Velocity Latency, Heading status, Heading, Pitch, GDOP, PDOP, HDOP, VDOP, TDOP, New GNSS Flag, Age of differenctiol correction, Position STD, Heading STD, Pitch STD.

`/Inertial_Labs/marine_data`
Heave, Surge, Sway, Heave Velocity, Surge Velocity, Sway Velocity, Significant Wave Height.


## FAQ
1. **I use WSL2 with Linux and can't receive data from sensor.**\
You need to use [usbipd](https://learn.microsoft.com/en-us/windows/wsl/connect-usb#install-the-usbipd-win-project). It allows to transfer data from USB to WSL2. And `/dev/ttyUSBx` devices will appear in Linux.
Run PowerShell as Administrator and make something like:
```powershell
# Mount USB:
usbipd list
usbipd bind --busid 1-1
usbipd attach --wsl --busid 1-1

# Unmount USB:
usbipd detach --busid 1-1
```

2. **The driver can't open my serial device?**\
Make sure you have enough access to `/dev`.
```bash
sudo chmod 666 /dev/ttyUSB0
```

3. **Why I have permission error during the initialization process of the driver?**\
Most often, this is because the baud rate you set does not match the package size to be received. Try increase the baud rate.
```bash
sudo stty -F /dev/ttyUSB0 460800
ros2 run inertiallabs_ins il_ins --ros-args -p ins_url:=serial:/dev/ttyUSB0:460800 -p ins_output_format:=88
```

4. **How can I check data from sensor?**\
Be sure, that Inertial Labs node has the topic-subscribers. Because messages will not send with no topic subscribers!

Print topic example:
In the separate windows run
```bash
source install/setup.bash
ros2 topic list
ros2 topic echo /Inertial_Labs/sensor_data
```

5. **Why is the IMU data output rate much lower than what is set?**\
This may be due to a recent change in the FTDI USB-Serial driver in the Linux kernel, the following shell script might help:
```bash
# Reduce latency in the FTDI serial-USB kernel driver to 1ms
# This is required due to https://github.com/torvalds/linux/commit/c6dce262
for file in $(ls /sys/bus/usb-serial/devices/); do
  value=`cat /sys/bus/usb-serial/devices/$file/latency_timer`
  if [ $value -gt 1 ]; then
    echo "Setting low_latency mode for $file"
    sudo sh -c "echo 1 > /sys/bus/usb-serial/devices/$file/latency_timer"
  fi
done
```

6. **Why a field value is always zero?**\
Most likely, because this field is not provided in the selected INS data packet. The most versatile data packet is User-Defined Data, which allows to order any set of fields

## Bug Report
Prefer to open an issue. You can also send an E-mail to support@inertiallabs.com.
