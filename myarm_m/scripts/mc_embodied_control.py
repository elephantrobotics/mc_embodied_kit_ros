#!/usr/bin/env python3
# encoding:utf-8
from pymycobot import MyArmM, MyArmC
from sensor_msgs.msg import JointState
import rospy
from std_msgs.msg import Header
from math import pi
import subprocess
import sys
import time
import datetime
import copy
def shutdown_ros_node(node_name):
    try:
        subprocess.run(['rosnode', 'kill', node_name])
        print(f"Node {node_name} has been shutdown.")
    except subprocess.CalledProcessError as e:
        print(f"Error: {e}")


def linear_transform(x):
    # 两个已知数据点
    x1, y1 = -89.5, 0.022
    x2, y2 = 0, 0
    
    # 计算斜率
    m = (y2 - y1) / (x2 - x1)
    
    # 计算截距
    c = y1 - m * x1
    
    # 应用线性变换
    y = m * x + c
    
    return y


def set_angle(mam, angle, speed=40):
    # 弧度转角度
    # angle = [int(a*180/pi) for a in angles]
    # angle.append(0)
    # joint_id = {0:1, 1:2, 2:3, 3:4, 4:5, 5:6}
    # joint_id = [0,1,2,3,4,5,6]
    mam.set_joints_angle(angle, speed)
    
    # for i in range(6):
    #     # if joint_id[i] == 2 or joint_id[i] == 5:
    #     #     angle[i] *= -1
    #     mam.set_joint_angle(joint_id[i], -angle[i], 50)
def main():
    # 初始化ROS节点
    rospy.init_node("combined_control", anonymous=True)
    # 关闭节点
    # shutdown_ros_node('myarm_m/joint_state_publisher_gui')
    # shutdown_ros_node('myarm_c650/joint_state_publisher_gui')
    # 发布对象
    pub = rospy.Publisher('/joint_states', JointState, queue_size=10)
    # pub_c = rospy.Publisher('/joint_states', JointState, queue_size=10)
    # pub_m2 = rospy.Publisher('/joint_states', JointState, queue_size=10)
    # pub_c2 = rospy.Publisher('/joint_states', JointState, queue_size=10)
    # 设置发布频率
    rate = rospy.Rate(50)
    # 消息实例
    joint_state = JointState()
    # 初始化api对象
    myarm_m = MyArmM('/dev/ttyACM3', 1000000, debug=False)
    myarm_c = MyArmC('/dev/ttyACM2', 1000000, debug=False)
    myarm_m2 = MyArmM('/dev/ttyACM4', 1000000, debug=False)
    myarm_c2 = MyArmC('/dev/ttyACM5', 1000000, debug=False)
    # 使能
    for i in range(8):
        myarm_m.set_servo_enabled(i, 1)
        time.sleep(0.2)
        myarm_m2.set_servo_enabled(i, 1)
        time.sleep(0.2)
    # m1 = m2 = c1 = c2 = [0,0,0,0,0,0,0]
    car = [0,0,0,0,0,0,0,0,0,0]

    while not rospy.is_shutdown():
        joint_state.header = Header()
        # 填充消息内容，例如关节名称、位置、速度和力
        joint_state.header.stamp = rospy.Time.now()
        
        joint_state.name = ['m1_joint1', 'm1_joint2', 'm1_joint3','m1_joint4', 'm1_joint5', 'm1_joint6','m1_gripper',
                            'm2_joint1', 'm2_joint2', 'm2_joint3','m2_joint4', 'm2_joint5', 'm2_joint6','m2_gripper',
                            'c1_joint1', 'c1_joint2', 'c1_joint3','c1_joint4', 'c1_joint5', 'c1_joint6','c1_gripper',
                            'c2_joint1', 'c2_joint2', 'c2_joint3','c2_joint4', 'c2_joint5', 'c2_joint6','c2_gripper']
        anglesc = myarm_c.get_joints_angle()
        anglesc2 = myarm_c2.get_joints_angle()
        anglesm = copy.deepcopy(anglesc)
        anglesm2 = copy.deepcopy(anglesc2)
        
        gripper_angle_c_real = anglesc.pop(6)
        gripper_angle_c2_real = anglesc2.pop(6)
        angle_c = [a/180*pi for a in anglesc]
        angle_c2 = [a/180*pi for a in anglesc2]
        gripper_angle_c_sim = linear_transform(gripper_angle_c_real)
        gripper_angle_c2_sim = linear_transform(gripper_angle_c2_real)
        angle_c.append(gripper_angle_c_sim)
        angle_c2.append(gripper_angle_c2_sim)
        
        gripper_angle_c_real = anglesm.pop(6) # 原来的夹角
        gripper_angle_m_sim = gripper_angle_c_sim/0.022*0.0345
        angle_m = [a*pi/180 for a in anglesm]
        angle_m.append(gripper_angle_m_sim)

        gripper_angle_c2_real = anglesm2.pop(6) # 原来的夹角
        gripper_angle_m2_sim = gripper_angle_c2_sim/0.022*0.0345
        angle_m2 = [a*pi/180 for a in anglesm2]
        angle_m2.append(gripper_angle_m2_sim)

        # gripper_angle_m /= -3500
        # angle:弧度 angles:角度
     
        # angle_m = [1,3,5,6,8,9,7]
        # print("position2:",joint_state.position, len(joint_state.position))


        joint_state.position = angle_m + angle_m2 + angle_c + angle_c2 + car
        angle_m[2]*=-1
        angle_m2[2]*=-1
        joint_state.effort = []
        joint_state.velocity = []
        # 发布消息
        pub.publish(joint_state)
        current_time1 = datetime.datetime.now()
        # joint_state.position = angle_c + car
        # pub.publish(joint_state)
        current_time2 = datetime.datetime.now()
        gripper_angle_m_sim = angle_m.pop(6)
        gripper_angle_m_real = gripper_angle_m_sim*(-3500)
        angle_m = [a*180/pi for a in angle_m]
        angle_m.append(gripper_angle_m_real)
        myarm_m.set_joints_angle(angle_m, 30)

        gripper_angle_m2_sim = angle_m2.pop(6)
        gripper_angle_m2_real = gripper_angle_m2_sim*(-3500)
        angle_m2 = [a*180/pi for a in angle_m2]
        angle_m2.append(gripper_angle_m2_real)
        myarm_m2.set_joints_angle(angle_m2, 30)

        current_time = current_time2-current_time1
        print("m:",angle_m)
        print("m2:",angle_m2)
        rospy.loginfo('消息成功发布')
        rospy.loginfo(current_time)
        # 等待，允许其他节点处理
        rate.sleep()

if __name__ == '__main__':
    main()
