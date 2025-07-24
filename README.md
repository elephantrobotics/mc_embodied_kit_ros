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
    $ sudo apt-get install build-essential git cmake libasio-dev libpcap-dev
    $ sudo apt install ros-noetic-joint-state-publisher-gui
    $ sudo apt install ros-noetic-ros-controllers
    $ sudo apt install ros-noetic-gmapping
    $ sudo apt install ros-noetic-map-server
    $ sudo apt install ros-noetic-navigation
    ```
    
2. Clone the packages into your catkin workspace and compile

    (the following instructions assume your catkin workspace is at: ~/catkin_ws/src)

    ```
    $ cd ~/catkin_ws/src
    $ git clone https://github.com/elephantrobotics/mc_embodied_kit_ros.git
    $ cd ..
    $ catkin_make --pkg tracer_msgs
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

* 终端切换到目标目录之后，输入 `python` 指令：

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
  - `speed`: forward speed (0.0 ~ 0.5 m/s).
  - `duration`: duration (positive number, in seconds).

#### 2 `move_backward(speed, duration)`
- **function:** Move backward
  
- **Parameters:**
  - `speed`: backward speed (-0.5 ~ 0 m/s).
  - `duration`: duration (positive number, in seconds).

#### 3 `turn_left(speed, duration)`
- **function:** turn left rotation
  
- **Parameters:**
  - `speed`: move speed (0.0 ~ 0.5 m/s).
  - `duration`: duration (positive number, in seconds).

#### 4 `turn_right(speed, duration)`
- **function:** turn right rotation
  
- **Parameters:**
  - `speed`: backward speed (-0.5 ~ 0 m/s).

  - `duration`: duration (positive number, in seconds).

#### 5 `stop(speed, duration)`
- **function:** Stop Move

## tracer底盘导航实现

### 雷达建图 - Gmapping

1. **开启底盘节点和雷达通信**

    ```
    roslaunch tracer_odometry tracer_active.launch
    ```

    **注意：** 启动底盘节点前需确保CAN总线已经使能，系统重启或者重新拔插CAN总线，都需要执行使能指令： `rosrun tracer_bringup setup_can2usb.bash`

2. **打开gmapping - 建图launch文件**

    ```
    roslaunch tracer_navigation mapping.launch
    ```

3. **打开键盘控制文件**

    ```
    roslaunch tracer_bringup tracer_teleop_keyboard.launch
    ```

4. **开始建图**

    现在底盘小车可以在键盘控制下移动。同时，您可以在 Rviz 空间中观察到，随着小车的移动，我们的地图也在逐渐构建。

    注意：使用键盘操作小车时，请确保运行 tracer_teleop_keyboard.launch 文件的终端是当前选定的终端；否则，键盘控制程序将无法识别按键。此外，为了获得更好的映射效果，建议在键盘控制时将线速度设为 0.2，角速度设为 0.4，因为较低的速度往往会产生更好的映射效果。

5. **保存构建的地图**

    打开另一个新的终端控制台，在命令行中输入以下命令，保存 tracer 扫描的地图：

    ```
    cd ~/catkin_ws/src/mc_embodied_kit_ros/tracer_navigation/map

    rosrun map_server map_saver
    ```
    执行成功后，将在当前路径(`~/catkin_ws/src/mc_embodied_kit_ros/tracer_navigation/map`)下生成两个默认地图参数文件，即map.pgm和map.yaml。

### 地图导航

在此之前，我们已经成功创建了空间地图，并获得了一组地图文件，即位于 `~/catkin_ws/src/mc_embodied_kit_ros/tracer_navigation/map` 目录下的 map.pgm 和 map.yaml。

1. **开启底盘节点和雷达通信**

    ```
    roslaunch tracer_odometry tracer_active.launch
    ```

2. **运行导航launch文件**

    ```
    roslaunch tracer_navigation navigation_active.launch
    ```

**SAFETY PRECAUSION**: 

Always have your remote controller ready to take over the control whenever necessary. 
