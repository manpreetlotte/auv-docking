#!/usr/bin/env python

import rospy
import numpy as np
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState

class PIDController:
    def __init__(self, kp, ki, kd, setpoint):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.setpoint = setpoint
        self.prev_error = 0
        self.integral = 0

    def compute(self, current_value):
        error = (self.setpoint - current_value)

        self.integral += error
        derivative = error - self.prev_error

        # PID control law
        output = ( self.kp * error + self.kd * derivative + self.ki * self.integral)

        self.prev_error = error
        return output

class DepthControllerNode:
    def __init__(self):
        rospy.init_node('depth_controller_node', anonymous=True)

        kp = 1.5
        kd = 5000
        ki = 0

        setpoint = -1

        self.pid_controller = PIDController(kp, ki, kd, setpoint)

        self.subscriber = rospy.Subscriber("/BlueRov2/state", Odometry, self.callback)
        self.publisher = rospy.Publisher("/BlueRov2/thruster_command", JointState, queue_size=1000)

    def callback(self, data):

        depth_position = data.pose.pose.position.z
        
        current_pose = depth_position

        # Compute PID control effort
        control_effort = self.pid_controller.compute(current_pose)
        
        # Create JointState message
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = rospy.Time.now()
        joint_state_msg.name = ["thr1", "thr2", "thr3", "thr4", "thr5", "thr6"]

        depth_effort = np.array([0, 0, 0, 0, control_effort, control_effort])

        joint_state_msg.effort = list(depth_effort)

        # Publish JointState message
        self.publisher.publish(joint_state_msg)

if __name__ == '__main__':
    depth_controller_node = DepthControllerNode()
    rospy.spin()