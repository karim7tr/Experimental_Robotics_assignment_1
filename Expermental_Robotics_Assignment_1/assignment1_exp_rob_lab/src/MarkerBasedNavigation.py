#!/usr/bin/env python3

"""
This node locates Aruco AR markers in images and publishes their ids and poses.

Subscriptions:
   /camera/color/image_raw (sensor_msgs.msg.Image)

Published Topics:
    /aruco_ID (std_msgs.msg.String)
       String containing the IDs of detected ArUco markers

Parameters:
    marker_size - size of the markers in meters (default 0.0625)
    aruco_dictionary_id - dictionary that was used to generate markers (default DICT_6X6_250)
    image_topic - image topic to subscribe to (default /camera/color/image_raw)

Author: Bernard Maacaron
"""
        
        
import numpy as np
import cv2.aruco as aruco
import cv2
import rospy

from sensor_msgs.msg import Image
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import CameraInfo

from cv_bridge import CvBridge, CvBridgeError

# Marker IDs to detect, their order and their corresponding detection status (True/False)
markerDetected_dict = {11:False, 12:False, 13:False, 15:False}

# Robot states dictionary
robotState_dict = {1:"Searching", 2:"Moving", 3:"Finished"}

# Minimum required pixel size
requiredEdgeSize = 160

# Camera resolution
camera_resolution = (640, 480)

# Marker size in pixels
actual_marker_size = 0.0625


class MarkerDetector:
    # ==================== INITIALIZATION ====================
    def __init__(self):
        # Initialize variables
        self.robotState = robotState_dict[1] #initialize the state of the robot of course at first it is searching
        self.ids_list = None
        self.bridge = CvBridge()
        self.arucoParameters = aruco.DetectorParameters()
        self.arucoParameters.minMarkerPerimeterRate = 0.1
        self.arucoParameters.maxMarkerPerimeterRate = 6.0
        
        # Camera related topics
        self.camera_info_sub = rospy.Subscriber("/camera/color/camera_info", CameraInfo, self.camera_info_callback)

        # Image related topics
        self.image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.detector_callback)
        self.id_pub = rospy.Publisher("/aruco_ID", String, queue_size=10)
        self.image_pub = rospy.Publisher("/aruco_Image", Image, queue_size=10)
        # self.corners_pub = rospy.Publisher("/aruco_Corners", String, queue_size=10)

        # Movement related topics
        self.lookfm_pub = rospy.Subscriber('/aruco_ID', String, self.runControlLogic)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        
    # ==================== CAMERA INITIALIZATION ====================
    # Load camera matrix and distortion coefficients
    def camera_info_callback(self, msg):
        self.camera_matrix = np.array(msg.K).reshape(3, 3)
        self.dist_coeffs = np.array(msg.D)
        # self.camera_resolution = (msg.width, msg.height)
        
        # Extract the focal lengths
        self.focal_length_x = self.camera_matrix[0, 0]
        self.focal_length_y = self.camera_matrix[1, 1]
        

        
    # ==================== MARKER DETECTION ====================
    def detector_callback(self, raw_data):
        cv_image = self.ImageConverter_callback(raw_data)
        self.markers_img, self.ids_list, self.corners_list = self.detect_aruco(cv_image)
        
        try:
            if self.ids_list is None:
                self.ids_list = []
        except:
            pass
            
            
        if self.markers_img is not None:
            try:
                self.image_pub.publish(self.bridge.cv2_to_imgmsg(self.markers_img, "bgr8"))
                cv2.imshow("Markers", self.markers_img)
                cv2.waitKey(1)
            except CvBridgeError as e:
                print(e)


        if self.ids_list != []:
            ids_str = ''.join(str(e) for e in self.ids_list)
            self.id_pub.publish(ids_str)
        else:
            self.id_pub.publish("No markers found.")
 
    # Used to convert the ROS image to OpenCV format - called in detector_callback
    def ImageConverter_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        return cv_image
        
    # Used to detect the ArUco markers - called in detector_callback 
    def detect_aruco(self, cv_image):
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL)
        corners_list, ids_list, _ = aruco.detectMarkers(gray, aruco_dict, parameters=self.arucoParameters)
        output = aruco.drawDetectedMarkers(cv_image, corners_list, ids_list)
        return output, ids_list, corners_list
    
    
    #==================== ROBOT MOVEMENT ====================
    def runControlLogic(self, aruco_ID):
        try:
            marker = [key for key,values in markerDetected_dict.items() if values == False][0]
        except:
            print("All markers detected. Shutting down in 5 seconds")
            rospy.sleep(5)
            rospy.signal_shutdown("Shurting down")

        
        while self.robotState == "Searching":
            print("Robot State:", self.robotState)
            print("Searching for marker:", marker)
            print("ids list:", self.ids_list)
            
            if marker in self.ids_list:
                print("Correct Marker detected:", marker)
                self.move_robot(0.0, 0.0, 0.1)
                self.robotState = robotState_dict[2]
            else:
                if marker == 13 or marker == 15:
                    self.move_robot(0.0, 0.7, 1.0)
                if marker == 11 or marker == 12:
                    self.move_robot(0.0, -0.7, 1.0)
                print("THE ROBOT SHOULD ROTATE NOW")
                
        while self.robotState == "Moving":
            print("Robot State:", self.robotState)
            #DONE -  Extract desired marker corners
            idsList = self.ids_list.flatten().tolist()
            cornersList = [corner.reshape((4, 2)) for corner in self.corners_list]
            # print("ids LIST", self.ids_list)
            index =  idsList.index(marker)
            # print("Corners List", self.corners_list)
            corners = cornersList[index]
            
            #DONE: move towards the desired marker center until the marker edge size is confirmed
            marker_size = self.calculateEdgeSize(corners)
            if marker_size < requiredEdgeSize:
                self.move_towards_marker(corners, marker_size)
                print("Moving towards marker:", marker)
            else:  
                self.move_robot(0.0, 0.0, 0.1)
                markerDetected_dict[marker] = True
                print("Marker", marker, "detected")
                
                if all(value == True for value in markerDetected_dict.values()):
                    print("All markers detected")
                    self.robotState = robotState_dict[3]
                    print("Robot State:", self.robotState)
                else:
                    self.robotState = robotState_dict[1]
                    print("Robot State:", self.robotState)
                                       
    def move_towards_marker(self, corners, marker_size):
        # Calculate the center of the marker
        center_x = np.mean([corner[0] for corner in corners])
        center_y = np.mean([corner[1] for corner in corners])

        # Get the center of the camera frame
        camera_center_x = camera_resolution[0]/2
        camera_center_y = camera_resolution[1]/2

        # Calculate the movement required
        delta_x = center_x - camera_center_x

        # Move the robot proportionally to the distance from the center
        linear_speed = 0.2  # Adjust as needed
        angular_speed = 0.1  # Adjust as needed

        distance = self.distance_to_marker(marker_size)
        
        # linear = linear_speed/distance
        # Arbitrary rule to make the linear velocity proportional to the distance
        linear = linear_speed*distance
        angular = -angular_speed * delta_x / camera_center_x

        self.move_robot(linear, angular, 0.8)

                
    def calculateEdgeSize(self, corners):
        # Calculate Edge size
        edge_lengths = [np.linalg.norm(np.array(corners[i + 1]) - np.array(corners[i])) for i in range(3)]
        marker_size = max(edge_lengths)
        print("Marker Size:", marker_size, "Required Size:", requiredEdgeSize,
              "truth value:", marker_size < requiredEdgeSize)
        return marker_size

    def distance_to_marker(self, marker_size):
        # Calculate the distance to the marker using the focal length in x direction
        distance_x = (actual_marker_size * self.focal_length_x) / marker_size

        # Calculate the distance to the marker using the focal length in y direction
        distance_y = (actual_marker_size * self.focal_length_y) / marker_size

        # Average the distances calculated using the x and y focal lengths
        distance = (distance_x + distance_y) / 2

        return distance

    def move_robot_instant(self, linear_speed, angular_speed):
        move_cmd = Twist()
        move_cmd.linear.x = linear_speed
        move_cmd.angular.z = angular_speed
        self.cmd_vel_pub.publish(move_cmd)
        print(str(move_cmd))
        
    def move_robot(self, linear_speed, angular_speed, duration):
        move_cmd = Twist()
        move_cmd.linear.x = linear_speed
        move_cmd.angular.z = angular_speed
        duration = rospy.Duration(duration)

        stop_cmd = Twist()

        start_time = rospy.Time.now()
        while rospy.Time.now() - start_time < duration:
            self.cmd_vel_pub.publish(move_cmd)
            rospy.sleep(0.1)

        # self.cmd_vel_pub.publish(stop_cmd)
        # rospy.sleep(1)





def main():
    print("Initializing ROS Node - lookformarker")
    rospy.init_node('lookformarker')
    marker_detector = MarkerDetector()
    try:
        rospy.spin()
    except:
        print("Shutting down ROS Node - lookformarker")
    cv2.destroyAllWindows()



if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass

