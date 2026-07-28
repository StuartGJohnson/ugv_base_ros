# Waveshare UGV Robots
This is a lower computer example for the [Waveshare](https://www.waveshare.com/) UGV robots using the Arduino sketch in ```ROS_Driver```.

## About This Updated Version (Modified by Stuart Johnson, April–July 2026.)

This repository is an independently maintained, updated version of the
[Waveshare `ugv_base_ros` repository](https://github.com/waveshareteam/ugv_base_ros).
It is based on upstream commit
[`c967326`](https://github.com/waveshareteam/ugv_base_ros/commit/c967326f7f9e52b0229a7573a70ea6ba77c4411e)
and is not a GitHub fork.

The updates by Stuart Johnson add DMP-based IMU processing and synchronized
telemetry, revised OLED diagnostics, feed-forward/PI motor control with slew
limiting, coupled left/right motor control, revised timing, and Wi-Fi SNTP time
synchronization. The original Waveshare copyright and GPLv3-or-later licensing
are preserved.

## Basic Description
The Waveshare UGV robots utilize both an upper computer and a lower computer. This repository contains the program running on the lower computer, which is typically a ESP32 on **ROS Driver for Robots**.  

The program running on the lower computer is either named [ugv_base_ros](https://github.com/effectsmachine/ugv_base_ros.git) or [ugv_base_general](https://github.com/effectsmachine/ugv_base_general.git) depending on the type of robot driver being used.  

The upper computer communicates with the lower computer (the robot's driver based on ESP32) by sending JSON commands via USB UART. 

## Features
- Closed-loop Speed Control with PID
- IMU
- OLED Screen
- LED Lights(12V switches) Control
- Control via JSON Commands
- Telemetry via JSON

## Configure the compilation environment
You need to install **[Arduino IDE](https://www.arduino.cc/en/software)** and **[ESP32 Board](https://randomnerdtutorials.com/installing-the-esp32-board-in-arduino-ide-windows-instructions/)** first.

### Install libraries
Copy `SCServo` folder into `C:\Users\[username]\AppData\Local\Arduino15\libraries\`

Install libraries with **`Library Manager`**: ArduinoJson, Adafruit_SSD1306, Adafruit GFX, Adafruit BusIO, INA219_WE, ESP32Encoder, PID_v2, SimpleKalmanFilter,  SparkFun 9DoF IMU Breakout - ICM 20948 - Arduino Library.

See also ```ROS_Driver/sketch.yaml``` for a manifest of specific library versions. This can be installed via the command line using the arduino-cli, e.g. (from within the ```ROS_Driver``` directory) ```arduino-cli compile --profile esp32 --verbose .```. However, the Arduino IDE builds and deploys were used in testing.

### Upload ###
Built firmware can be uploaded to the Waveshare ESP32 board via the Arduino IDE. Note that you will need to hold down the "boot" button on the board. The USB-C connector on the board can be used for FW upload.

### SNTP timing sync ###
The FW uses SNTP to sync to a local SNTP server on the LAN (which could be the robot host). The settings for this are in the file ```ROS_Driver/Data/wifiConfig_example.json```. The file with the proper setting should be placed in ```ROS_Driver/Data/wifiConfig.json```. Generally speaking, other nodes on the LAN participating in the ROS2 nodes running the robot should be running some time synchronization scheme. This FW was tested with chrony NTP synchronization with a LAN clock master.

### Basic Use
You can send JSON command to robot via UART/USB@115200.

To ensure compatibility with various types of robots. You can configure the robot by entering the following command:

    {"T":900,"main":2,"module":0}

In this command, the s directive denotes a robot-type setting. The first digit, `2`, signifies that the main type of robot is a `UGV Rover`, with `1` representing `RaspRover` and `3` indicating `UGV Beast`. The second digit, `2`, specifies the module as `Camera PT`, where `0` denotes `Nothing` and `1` signifies `RoArm-M2`.

# License
ugv_base_ros for the Waveshare UGV Robots: an open source robotics platform for the Robots based on **ROS Driver**.
Copyright (C) 2024 [Waveshare](https://www.waveshare.com/)

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/gpl-3.0.txt>.
