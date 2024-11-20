#!/usr/bin/env python3
#coding=UTF-8
import rospy
from geometry_msgs.msg import Twist
import time
import threading
from datetime import datetime


class ChassisController:
    def __init__(self, debug=False):
        """
        Initialize chassis controller
        """
        rospy.init_node('chassis_controller', anonymous=True)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.rate = rospy.Rate(10)
        self.twist = Twist()  # init Twist
        self._stop_flag = False  # Flag used to control the stop motion
        self._lock = threading.Lock()
        self.debug = debug
    
    def _log_debug(self, action, linear_x, angular_z, duration):
        """
        Print debug information
        :param action: currently executed action
        :param linear_x: linear velocity
        :param angular_z: angular velocity
        """
        if self.debug:
            current_time = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
            print("{} DEBUG [{}] linear x: {}, angular z: {}, duration: {}".format(current_time, action, linear_x,angular_z, duration))

    def move_forward(self, speed=0.5, duration=1):
        """
        Move forward
        :param speed: forward speed (0.0 ~ 1.8 m/s)
        :param duration: duration (positive number, in seconds)
        """
        self._validate_linear_speed(speed, "forward_speed", 0.0, 1.8)
        self._validate_duration(duration)
        self._move(linear_x=speed, duration=duration)

    def move_backward(self, speed=-0.5, duration=1):
        """
        Move backward
        :param speed: backward speed (-1.8 ~ 0.0 m/s)
        :param duration: duration (positive number, in seconds)
        """
        self._validate_linear_speed(speed, "backward_speed", -1.8, 0.0)
        self._validate_duration(duration)
        self._move(linear_x=speed, duration=duration)

    def move_clockwise(self, angular_speed=-1, duration=1):
        """
        Rotate clockwise
        :param angular_speed: Rotate clockwise speed (-1.0 ~ 0.0 rad/s)
        :param duration: duration (positive number, in seconds)
        """
        self._validate_angular_speed(angular_speed, "clockwise_speed", -1.0, 0.0)
        self._validate_duration(duration)
        self._move(angular_z=angular_speed, duration=duration)

    def move_counterclockwise(self, angular_speed=1, duration=1):
        """
        Rotate counterclockwise
        :param angular_speed: Rotate counterclockwise speed (0.0 ~ 1.0 rad/s)
        :param duration: duration (positive number, in seconds)
        """
        self._validate_angular_speed(angular_speed, "counterclockwise_speed", 0.0, 1.0)
        self._validate_duration(duration)
        self._move(angular_z=angular_speed, duration=duration)

    def forward_turn(self, speed=0.5, duration=1):
        """
        Forward turn (forward and rotation at the same time)
        :param speed: forward speed (0.0 ~ 1.8 m/s)
        :param duration: duration (positive number, in seconds)
        """
        angular_speed = 1
        self._validate_linear_speed(speed, "forward_speed", 0.0, 1.8)
        self._validate_angular_speed(angular_speed, "forward_turn_speed", 0, 1.0)
        self._validate_duration(duration)
        self._move(linear_x=speed, angular_z=angular_speed, duration=duration)

    def backward_turn(self, speed=-0.5, duration=1):
        """
        backward turn (backward and rotation at the same time)
        :param speed: bacdward speed (-1.8 ~ 0.0 m/s)
        :param duration: duration (positive number, in seconds)
        """
        angular_speed = -1
        self._validate_linear_speed(speed, "backward_speed", -1.8, 0.0)
        self._validate_angular_speed(angular_speed, "backward_turn_speed", -1.0, 0)
        self._validate_duration(duration)
        self._move(linear_x=speed, angular_z=angular_speed, duration=duration)

    def stop(self):
        """
        stop move
        """
        with self._lock:
            self._stop_flag = True
        self._log_debug("stop move", 0.0, 0.0, 0)
        self.twist.linear.x = 0.0
        self.twist.angular.z = 0.0
        self.pub.publish(self.twist)

    def _move(self, linear_x=0.0, angular_z=0.0, duration=1):
        """
        Internal method, controls chassis movement (non-blocking, supports stopping at any time)
        :param linear_x: Moving speed (m/s)
        :param angular_z: rotation speed (rad/s)
        :param duration: duration (positive number, in seconds)
        """
        def _movement_task():
            self._stop_flag = False
            self.twist.linear.x = linear_x
            self.twist.angular.z = angular_z

            start_time = time.time()
            self._log_debug("start move", linear_x, angular_z, duration)
            while time.time() - start_time < duration:
                # self.pub.publish(self.twist)
                 with self._lock:
                    if self._stop_flag:
                        break
                    self.pub.publish(self.twist)
                    time.sleep(0.05)

            # stop
            self.twist.linear.x = 0.0
            self.twist.angular.z = 0.0
            self.pub.publish(self.twist)

        threading.Thread(target=_movement_task, daemon=True).start()

    def _validate_linear_speed(self, speed, name, min_val, max_val):
        """
        Check line speed
        """
        if not (min_val <= speed <= max_val):
            raise ValueError("{} is out of range, should be between [{}, {}], current value: {}".format(name, min_val, max_val, speed))

    def _validate_angular_speed(self, speed, name, min_val, max_val):
        """
        Calibration angular velocity
        """
        if not (min_val <= speed <= max_val):
            raise ValueError("{} is out of range, should be between [{}, {}], current value: {}".format(name, min_val, max_val, speed))
        
    def _validate_duration(self, duration):
        """
        Verification duration
        """
        if not (isinstance(duration, (int, float)) and duration > 0):
            raise ValueError("Duration must be a positive number (can be a decimal), current value: {}".format(duration))



if __name__ == '__main__':
    try:
        controller = ChassisController()

        # 示例：前进、后退、顺逆时针旋转和转弯
        # controller.move_forward(speed=1.0, duration=2)               # 前进 2 秒
        # controller.move_backward(speed=-1, duration=2)            # 后退 2 秒
        # controller.move_clockwise(angular_speed=-0.5, duration=2)   # 顺时针旋转 2 秒
        # controller.move_counterclockwise(angular_speed=0.5, duration=2)  # 逆时针旋转 2 秒
        # controller.forward_turn(speed=1.0, duration=2)  # 前进转弯 2 秒
        # controller.backward_turn(speed=-1.0, duration=2) # 后退转弯 2 秒

        # # 停止小车
        # controller.stop()

    except rospy.ROSInterruptException:
        pass
