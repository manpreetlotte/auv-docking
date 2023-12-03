#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image, JointState
from cv_bridge import CvBridge
import cv2
import numpy as np
import apriltag

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

class VisionBasedController:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('marker_detector_node', anonymous=True)

        kp = np.array([1.5, 0.8, 0.8, 0.8, 0.8, 0.8])
        kd = np.array([200, 80, 100, 80, 80, 80])
        ki = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        
        self.setpoint = np.array([0, 0, 2.5, 0, 0, 0])

        # I am satisfied if rov reaches setpoint-offset
        self.offset = 0.05

        self.MARKER_SIZE = 0.8

        # camera calibration given from topic /BlueRov2/camera/camera_info
        # Distortion coefficients
        self.distortion_coef = np.zeros((5, 1))
        
        # Camera matrix (intrinsic parameters)
        self.camera_matrix = [[1591.580740874196, 0.0, 960.5],
                        [0.0, 1591.580740874196, 540.5],
                        [0.0, 0.0, 1.0]]
        self.camera_matrix = np.array(self.camera_matrix, dtype='float32')

        # Set up a subscriber for the "BlueRov2/camera/image_raw" topic
        self.image_sub = rospy.Subscriber('BlueRov2/camera/image_raw', Image, self.image_callback)

        # Set up a publisher for the "/BlueRov2/thruster_command" in order to publish effort command
        self.effort_pub = rospy.Publisher("/BlueRov2/thruster_command", JointState, queue_size=10)

        # Initialize PID controller
        self.pid_controller = PIDController(kp, ki, kd, self.setpoint)

        # Create a CvBridge instance to convert ROS Image messages to OpenCV images
        self.bridge = CvBridge()

    def pose_estimation_PnP(self, color_img, camera_matrix, dist_coeffs, marker_size):
        
        tag_detected = False

        image = cv2.cvtColor(color_img, cv2.COLOR_BGR2GRAY)

        # Create an AprilTag detector
        options = apriltag.DetectorOptions(families="tag36h11")
        detector = apriltag.Detector(options)

        # Detect AprilTags in the image
        result = detector.detect(image)

        # Check if any AprilTags are found
        if result:
            for detection in result:
                # Get the corners of the AprilTag
                corners = detection.corners

                # Define 3D coordinates of the AprilTag in its own coordinate system
                object_points = np.array(
                [
                    [-marker_size / 2, marker_size / 2, 0],
                    [marker_size / 2, marker_size / 2, 0],
                    [marker_size / 2, -marker_size / 2, 0],
                    [-marker_size / 2, -marker_size / 2, 0],
                ] )

                # Convert corners to numpy array
                img_points = corners.astype(dtype='float32')

                # Use solvePnP to estimate pose
                success, rvec, tvec = cv2.solvePnP(object_points, img_points, camera_matrix, dist_coeffs)

                if success:

                    tag_detected = True

                    print(f"Tag detected: {tag_detected}")
                    print(f"Tag ID {detection.tag_id}:")
                    print(f"Rotation Vector (rvec): {rvec}")
                    print(f"Translation Vector (tvec): {tvec}")
                    print("---------------------")
                    
                    # Draw the detected AprilTag and its axis on the image
                    cv2.drawContours(color_img, [np.int32(corners)], -1, (0, 0, 255), 2)
                    
                    cv2.drawFrameAxes(color_img, camera_matrix, dist_coeffs, rvec, tvec, 1)
        
            return tag_detected, color_img, tvec # rvec, tvec

    def image_callback(self, msg):
        # try:

        # Convert the ROS Image message to an OpenCV image
        color_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        
        result = self.pose_estimation_PnP(
                    color_img,
                    self.camera_matrix,
                    self.distortion_coef,
                    self.MARKER_SIZE,
                )
    
        if result:
            tag_detected, detection_img, tvec = result
        else:
            return

        if(tag_detected == False):
            return
        
        x_position, y_position, z_position = tvec[0][0], tvec[1][0], tvec[2][0]

        roll, pitch, yaw = 0, 0, 0 # rvec[0][0], rvec[1][0], rvec[2][0]

        current_pose = np.array([x_position, y_position, z_position, roll, pitch, yaw])

        # Compute PID control effort
        control_effort = self.pid_controller.compute(current_pose)
        
        # Create JointState message
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = rospy.Time.now()
        joint_state_msg.name = ["thr1", "thr2", "thr3", "thr4", "thr5", "thr6"]

        x_effort = np.array([control_effort[0], 0, control_effort[0], 0, 0, 0])
        y_effort = np.array([0, 0, 0, 0, control_effort[1], control_effort[1]])
        z_effort = np.array([-control_effort[2], -control_effort[2], 0, 0, 0, 0])
        # r = np.array([0, 0, 0, 0, 0, 0])
        # p = np.array([0, 0, 0, 0, -control_effort[4], control_effort[4]])
        yaw_effort = np.array([0, control_effort[5], control_effort[5], 0, 0, 0])

        x_reached = False
        z_reached = False

        if (x_reached == False):

            joint_state_msg.effort = list(x_effort)

            # Publish JointState message
            self.effort_pub.publish(joint_state_msg)

            if (x_position < self.setpoint[0]+self.offset and x_position > self.setpoint[0]-self.offset):
                x_reached = True
                print("---------------------------------------------------------")
                print("X docking position reached: ", x_reached)
        
        if (x_reached == True):
            
            joint_state_msg.effort = list(z_effort)

            # Publish JointState message
            self.effort_pub.publish(joint_state_msg)

            if (z_position < self.setpoint[2]+self.offset):
                z_reached = True
                print("Z docking position reached: ", z_reached)
                print("---------------------------------------------------------")
                

        # final_effort = x_effort

        # joint_state_msg.effort = list(final_effort)

        # # Publish JointState message
        # self.effort_pub.publish(joint_state_msg)

        # Initialize OpenCV window
        cv2.namedWindow('Camera Image', cv2.WINDOW_NORMAL)
        # Display the image
        cv2.imshow('Camera Image', detection_img)
        cv2.waitKey(1)

        # except Exception as e:
        #     rospy.logerr(e)

    def run(self):
        # Run the ROS node
        rospy.spin()

        # Close OpenCV window when the node is shutdown
        cv2.destroyAllWindows()

if __name__ == '__main__':
    docking_controller = VisionBasedController()
    docking_controller.run()