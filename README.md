# mc_embodied_kit_ros
MC Embodied Intelligence Suit ROS Package

## Packages

* tracer_base: a ROS wrapper around tracer SDK to monitor and control the robot
* tracer_bringup: launch and configuration files to start ROS nodes
* tracer_msgs: tracer related message definitions

## Communication interface setup

Please refer to the [README](./ugv_sdk/README.md) of "ugv_sdk" package for setup of communication interfaces.

#### Note on CAN interface on Nvidia Jetson Platforms

Nvidia Jeston TX2/Xavier/XavierNX have CAN controller(s) integrated in the main SOC. If you're using a dev kit, you need to add a CAN transceiver for proper CAN communication. 

## Basic usage of the ROS package

1. Install dependent packages

    ```
    $ sudo apt-get update
    $ sudo apt-get install build-essential git cmake libasio-dev
    $ sudo apt install ros-$ROS_DISTRO-joint-state-publisher-gui
    $ sudo apt install ros-$ROS_DISTRO-ros-controllers
    ```
    
2. Clone the packages into your catkin workspace and compile

    (the following instructions assume your catkin workspace is at: ~/catkin_ws/src)

    ```
    $ cd ~/catkin_ws/src
    $ git clone https://github.com/elephantrobotics/mc_embodied_kit_ros.git
    $ cd ..
    $ catkin_make
    ```

3. Setup CAN-To-USB adapter
* Enable gs_usb kernel module(If you have already added this module, you do not need to add it)
    ```
    $ sudo modprobe gs_usb
    ```
* first time use tracer-ros package
    ```
    $rosrun tracer_bringup setup_can2usb.bash
    ```
* If not the first time use tracer-ros package(Run this command every time you turn on the power)
    ```
    $rosrun tracer_bringup bringup_can2usb.bash
    ```
4. Launch ROS nodes

* Start the base node for the real robot whith can

    ```
    $ roslaunch tracer_bringup tracer_robot_base.launch
    ```
* Start the keyboard tele-op node

    ```
    $ roslaunch tracer_bringup tracer_teleop_keyboard.launch
    ```

* If the can-to-usb has been connected to the TRACER robot and the car has been turned on, use the following command to monitor the data from the TRACER chassis

    ```
    candump can0
    ```

## Python interface usage of ROS package

### Prerequisites for use

* 终端切换到目标目录之后，输入 `python3` 指令：

    ```
    $ cd ~/catkin_ws/src/mc_embodied_kit_ros/tracer_bringup/scripts
    $ python
    ```
* 在python的交互式环境导入底盘控制的接口库

    ```
    from chassis_controller import ChassisController
    ```
 * 简单使用
    ```
    # 示例
    from chassis_controller import ChassisController

    cc = ChassisController()

    # 前进、后退
    cc.move_forward(1.0, 2)  # 前进 2 秒, 速度为 1 m/s
    cc.move_backward(-1, 2)  # 后退 2 秒, 速度为 -1 m/s
    # 停止小车
    cc.stop()
    ```

### Python API使用说明

#### 1 `move_forward(speed, duration)`
- **function:** Move forward
  
- **Parameters:**
  - `speed`: forward speed (0.0 ~ 1.8 m/s).
  - `duration`: duration (positive number, in seconds).

#### 2 `move_backward(speed, duration)`
- **function:** Move forward
  
- **Parameters:**
  - `speed`: backward speed (-1.8 ~ 0 m/s).
  - `duration`: duration (positive number, in seconds).

#### 3 `move_clockwise(angular_speed, duration)`
- **function:** Rotate clockwise
  
- **Parameters:**
  - `angular_speed`: Rotate clockwise speed (-1.0 ~ 0.0 rad/s).
  - `duration`: duration (positive number, in seconds).

#### 4 `move_counterclockwise(angular_speed, duration)`
- **function:** Rotate counterclockwise
  
- **Parameters:**
  - `angular_speed`: Rotate counterclockwise speed (0.0 ~ 1.0 rad/s).
  - `duration`: duration (positive number, in seconds).

#### 5 `forward_turn(speed, duration)`
- **function:** Forward turn (forward and rotation at the same time)
  
- **Parameters:**
  - `speed`: forward speed (0.0 ~ 1.8 m/s).
  - `duration`: duration (positive number, in seconds).

#### 6 `backward_turn(speed, duration)`
- **function:** backward turn (backward and rotation at the same time)
  
- **Parameters:**
  - `speed`: backward speed (-1.8 ~ 0 m/s).
  - `duration`: duration (positive number, in seconds).

#### 7 `stop(speed, duration)`
- **function:** Stop Move


**SAFETY PRECAUSION**: 

Always have your remote controller ready to take over the control whenever necessary. 
