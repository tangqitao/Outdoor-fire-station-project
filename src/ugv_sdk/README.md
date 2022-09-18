# UGV SDK

![GitHub Workflow Status](https://github.com/agilexrobotics/wrp_sdk/workflows/Cpp/badge.svg)
![GitHub Workflow Status](https://github.com/agilexrobotics/wrp_sdk/workflows/ROS/badge.svg)

This repository is a joint effort by the development teams at Weston Robot (Singapore) and AgileX Robotics (China).

- Copyright (c) 2020 [Weston Robot](https://www.agilexrobotics.com/) 
- Copyright (c) 2020 [AgileX Robotics](http://www.agilex.ai/?lang=zh-cn)

## Introduction

This software package provides a C++ interface to communicate with the mobile platforms from Weston Robot and AgileX Robotics, for sending commands to the robot and receiving the latest robot state. 

Supported robot platforms

* **Scout**: skid-steer mobile base
* **Hunter**: ackermann mobile base

Supported environments

* **Architecture**: x86_64/arm64
* **OS**: Ubuntu 16.04/18.04/20.04
* **ROS**: Kinetic/Melodic/Noetic

It should also work in other similar Linux environments but only the above listed environments are regularly tested.

Communication protocol

| Robot  | Protocol Version |
| :----: | :--------------: |
| Scout  |      V1, V2      |
| Hunter |      V1, V2      |
| Tracer |        V2        |
| Bunker |        V1        |

Currently we're transitioning the communication protocol from version 1 to version 2. Upgrading protocol won't affect your ROS packages but you will need to make sure the robot is running a compatible firmware.

Generally, you only need to instantiate an object of the robot base class (such as ScoutBase), then use the object to programmatically control the robot. Internally, the base class manages two background threads, one to process CAN/UART messages of the robot state and accordingly update state variables in the robot state data structure, and the other to maintain a 50Hz loop and send the latest command to the robot base. User can iteratively perform tasks in the main thread and check the robot state or set control commands. Advanced users may also use this setup as a reference and maintain the control and monitoring loops in a different way.

## Build SDK

### Install dependent libraries

```
$ sudo apt-get update
$ sudo apt-get install build-essential git cmake
```

### I. Use the package with ROS

```
$ cd <your-catkin-ws>/src
$ git clone --recursive https://github.com/agilexrobotics/ugv_sdk.git
$ cd ..
$ catkin_make
```

### II. Use the package without ROS

If you want to build the TUI monitor tool, additionally install libncurses

```
$ sudo apt install libncurses5-dev
```

Configure and build

```
$ git clone --recursive https://github.com/agilexrobotics/ugv_sdk.git
$ cd ugv_sdk 
$ mkdir build
$ cd build
$ cmake ..
$ make
```

### Update repository

If you have already cloned the repository, use the following commands to get latest updates

```
$ cd ugv_sdk
$ git pull origin master
$ git submodule update --init --recursive
```

## Hardware Interface

### Setup CAN-To-USB adapter 
 
1. Enable gs_usb kernel module
    ```
    $ sudo modprobe gs_usb
    ```
2. Bringup can device
   ```
   $ sudo ip link set can0 up type can bitrate 500000
   ```
3. If no error occured during the previous steps, you should be able to see the can device now by using command
   ```
   $ ifconfig -a
   ```
4. Install and use can-utils to test the hardware
    ```
    $ sudo apt install can-utils
    ```
5. Testing command
    ```
    # receiving data from can0
    $ candump can0
    ```

Two scripts inside the "./scripts" folder are provided for easy setup. You can run "./setup_can2usb.bash" for the first-time setup and run "./bringup_can2usb.bash" to bring up the device each time you unplug and re-plug the adapter.

### Setup UART

Generally your UART2USB cable should be automatically recognized as "/dev/ttyUSB0" or something similar and ready for use. If you get the error "... permission denied ..." when trying to open the port, you need to grant access of the port to your user accout:

```
$ sudo usermod -a -G dialout $USER
```

You need to re-login to get the change to take effect.


## Run Demo Apps

The demo code expects one parameter for the CAN bus mode.

```
$ ./app_scout_demo can0
```

Both the port name and baud rate need to be provided when using the RS232 interface.

```
$./app_scout_demo /dev/ttyUSB0 115200
```

If you've installed "libncurses5-dev" and built "app_scout_monitor", you can run it in a similar way:

```
$ ./app_scout_monitor can0
```

or

```
$./app_scout_monitor /dev/ttyUSB0 115200
```

Note: the monitor app is not built by default if you use this SDK with ROS.

## Reference

* [CAN command reference in Linux](https://wiki.rdu.im/_pages/Notes/Embedded-System/Linux/can-bus-in-linux.html)
