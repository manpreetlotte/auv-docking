#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image, JointState
from cv_bridge import CvBridge
import cv2
import numpy as np
import apriltag

class MarkerDetector:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('marker_detector_node', anonymous=True)

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
        self.effort_pub = rospy.Publisher("/BlueRov2/thruster_command", JointState, queue_size=100)

        # Create a CvBridge instance to convert ROS Image messages to OpenCV images
        self.bridge = CvBridge()

    def pose_estimation_PnP(self, color_img, camera_matrix, dist_coeffs, marker_size):
        
        # frame is the colored image

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
                    print(f"Tag ID {detection.tag_id}:")
                    print(f"Rotation Vector (rvec): {rvec}")
                    print(f"Translation Vector (tvec): {tvec}")
                    print("---------------------")
                    
                    # Draw the detected AprilTag and its axis on the image
                    cv2.drawContours(color_img, [np.int32(corners)], -1, (0, 0, 255), 2)
                    
                    cv2.drawFrameAxes(color_img, camera_matrix, dist_coeffs, rvec, tvec, 1)
        
        return color_img, rvec, tvec

    def image_callback(self, msg):
        try:

            # Convert the ROS Image message to an OpenCV image
            color_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

            detection_img, rvec, tvec = self.pose_estimation_PnP(
                color_img,
                self.camera_matrix,
                self.distortion_coef,
                self.MARKER_SIZE,
            )

            # Initialize OpenCV window
            cv2.namedWindow('Camera Image', cv2.WINDOW_NORMAL)
            # Display the image
            cv2.imshow('Camera Image', detection_img)
            cv2.waitKey(1)

        except Exception as e:
            rospy.logerr(e)

    def run(self):
        # Run the ROS node
        rospy.spin()

        # Close OpenCV window when the node is shutdown
        cv2.destroyAllWindows()

if __name__ == '__main__':
    marker_detector = MarkerDetector()
    marker_detector.run()