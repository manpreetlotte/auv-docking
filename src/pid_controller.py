#!/usr/bin/env python

import rospy
import numpy as np
import tf
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose, Quaternion
from tf.transformations import euler_from_quaternion

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

class PoseControllerNode:
    def __init__(self):
        rospy.init_node('pose_controller_node', anonymous=True)

        # Initialize a TransformListener
        # self.listener = tf.TransformListener()

        kp = np.array([1.5, 1.5, 1.5, 1.5, 1.5, 1.5])
        kd = np.array([5000, 5000, 5000, 5000, 5000, 5000])
        ki = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

        
        setpoint = np.array([1, -1, -1, 0, 0, 0])

        self.pid_controller = PIDController(kp, ki, kd, setpoint)

        self.subscriber = rospy.Subscriber("/BlueRov2/state", Odometry, self.callback)
        self.publisher = rospy.Publisher("/BlueRov2/thruster_command", JointState, queue_size=1000)

    def callback(self, data):
        x_position = data.pose.pose.position.x
        y_position = data.pose.pose.position.y
        z_position = data.pose.pose.position.z

        # get the orientation in world frame
        orientation = data.pose.pose.orientation

        #now = rospy.Time.now()

        #self.listener.waitForTransform('/base_link', data.header.frame_id, now, rospy.Duration(4.0))
        
        # try:
            # Transform the orientation to the base_link frame
        #(trans, rot) = self.listener.lookupTransform('/base_link', data.header.frame_id, rospy.Time(0))
        #orientation_base_link_frame = self.listener.transformQuaternion('/base_link', orientation_world_frame)
        
        # except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
        #    rospy.logerr("Transform error: %s", e)

        x = orientation.x
        y = orientation.y
        z = orientation.z
        w = orientation.w

        # euler angles rpy wrt base_link frame and NOT world frame
        roll, pitch, yaw = euler_from_quaternion ([x, y, z, w])
        
        current_pose = np.array([x_position, y_position, z_position, roll, pitch, yaw])

        # Compute PID control effort
        control_effort = self.pid_controller.compute(current_pose)
        
        # Create JointState message
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = rospy.Time.now()
        joint_state_msg.name = ["thr1", "thr2", "thr3", "thr4", "thr5", "thr6"]

        # control allocation
        # ex. to move along axis x I need to turn on motor 1 and motor 3 and giving them the same power
        x_effort = np.array([control_effort[0], 0, control_effort[0], 0, 0, 0])
        y_effort = np.array([-control_effort[1], -control_effort[1], 0, 0, 0, 0])
        z_effort = np.array([0, 0, 0, 0, control_effort[2], control_effort[2]])
        # r = np.array([0, 0, 0, 0, 0, 0])
        # p = np.array([0, 0, 0, 0, -control_effort[4], control_effort[4]])
        yaw_effort = np.array([0, control_effort[5], control_effort[5], 0, 0, 0])
        
        # not working if I add yaw_effort
        final_effort = x_effort + y_effort + z_effort # + yaw_effort

        joint_state_msg.effort = list(final_effort)

        # Publish JointState message
        self.publisher.publish(joint_state_msg)

if __name__ == '__main__':
    pose_controller_node = PoseControllerNode()
    rospy.spin()