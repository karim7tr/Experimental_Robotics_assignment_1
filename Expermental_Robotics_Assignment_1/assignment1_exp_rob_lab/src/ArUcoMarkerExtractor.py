
#!/usr/bin/env python

import rospy
from gazebo_msgs.msg import ModelStates
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Pose
from std_msgs.msg import Header

class ArUcoMarkerExtractor:
    def __init__(self):
        rospy.init_node('aruco_marker_extractor', anonymous=True)

        # Publishers
        self.marker_array_pub = rospy.Publisher('/visualization_marker_array', MarkerArray, queue_size=10)

        # Subscribers
        rospy.Subscriber('/gazebo/model_states', ModelStates, self.gazebo_callback)

    def gazebo_callback(self, data):
        # Extract ArUco marker information from Gazebo data
        aruco_markers = self.extract_aruco_markers(data)

        # Convert ArUco markers to MarkerArray
        marker_array = self.convert_to_marker_array(aruco_markers)

        # Publish the MarkerArray to RViz
        self.marker_array_pub.publish(marker_array)

    def extract_aruco_markers(self, model_states):
        aruco_markers = []

        # Customize this part based on your ArUco marker data structure
        for model_name, pose in zip(model_states.name, model_states.pose):
            if "marker" in model_name.lower():
                aruco_markers.append((model_name, pose))

        return aruco_markers

    def convert_to_marker_array(self, aruco_markers):
        marker_array = MarkerArray()

        # Customize this part based on your ArUco marker data structure
        for idx, (model_name, pose) in enumerate(aruco_markers):
            marker = Marker()
            marker.header = Header()
            marker.header.stamp = rospy.Time.now()
            marker.header.frame_id = "top"  # Replace with your frame_id
            marker.ns = "aruco_markers"
            marker.id = idx
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.pose = pose
            marker.scale.x = 0.2  # Adjust as needed
            marker.scale.y = 0.2  # Adjust as needed
            marker.scale.z = 0.2  # Adjust as needed
            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.color.b = 1.0
            marker.color.a = 1.0

            marker_array.markers.append(marker)

        return marker_array

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        aruco_extractor = ArUcoMarkerExtractor()
        aruco_extractor.run()
    except rospy.ROSInterruptException:
        pass
