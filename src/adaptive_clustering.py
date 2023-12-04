#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import Point

import math

class AdaptiveClustering:
    def __init__(self):
        rospy.init_node('adaptive_clustering')
        scan_topic = rospy.get_param('~scan_topic', '/scan')
        self.scan_sub = rospy.Subscriber(scan_topic, LaserScan, self.scan_callback)
        # self.scan_sub = rospy.Subscriber('scan', LaserScan, self.scan_callback)
        self.marker_pub = rospy.Publisher('visualization_marker_array', MarkerArray, queue_size=10)
        self.clusters = []
        self.cluster_size_min = rospy.get_param('~cluster_size_min', 3)
        self.tolerance_factor = rospy.get_param('~tolerance_factor', 0.1)

    def scan_callback(self, data):
        self.clusters = self.adaptive_clustering(data)
        self.publish_markers(data)

    # ... (other parts of your code)

    def adaptive_clustering(self, data):
        clusters = []
        cluster = []

        for i in range(1, len(data.ranges)):
            if data.ranges[i] < data.range_max and data.ranges[i-1] < data.range_max:
                # Calculate the adaptive threshold based on the range
                adaptive_threshold = self.calculate_adaptive_threshold(data.ranges[i])

                if abs(data.ranges[i] - data.ranges[i - 1]) < adaptive_threshold:
                    # If the change in range is less than the threshold, they are part of the same cluster
                    cluster.append((i, data.ranges[i]))
                else:
                    # If the change in range is greater, start a new cluster
                    if len(cluster) >= self.cluster_size_min:
                        clusters.append(cluster)
                    cluster = [(i, data.ranges[i])]
            else:
                # If the current range is invalid, end the current cluster
                if len(cluster) >= self.cluster_size_min:
                    clusters.append(cluster)
                cluster = []

        # Check the last cluster
        if len(cluster) >= self.cluster_size_min:
            clusters.append(cluster)

        return clusters

      


    def calculate_adaptive_threshold(self, range_val):
        # Implement the logic to calculate an adaptive threshold.
        # This can be a function of the range value if the sensor noise
        # is proportional to the distance, for instance.
        return self.tolerance_factor * range_val

    def publish_markers(self, data):
        marker_array = MarkerArray()

        # First, clear any existing markers
        delete_marker = Marker()
        delete_marker.action = Marker.DELETEALL
        delete_marker.header.frame_id = data.header.frame_id
        marker_array.markers.append(delete_marker)
        self.marker_pub.publish(marker_array)

        # Now, build the new markers
        marker_array = MarkerArray()
        for i, cluster in enumerate(self.clusters):
            marker = Marker()
            marker.header.frame_id = data.header.frame_id
            marker.header.stamp = rospy.Time.now()
            marker.ns = "cluster"  # Namespace for the marker
            marker.id = i  # Unique ID for each marker
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            marker.pose.orientation.w = 1
            marker.scale.x = 0.1  # Width of the line
            marker.color.a = 1.0  # Don't forget to set the alpha!
            marker.color.r = float(i) / len(self.clusters)
            marker.color.g = 1.0 - float(i) / len(self.clusters)
            marker.color.b = 0.5

            # Add points to the marker
            for point_index, distance in cluster:
                angle = data.angle_min + point_index * data.angle_increment
                x = distance * math.cos(angle)
                y = distance * math.sin(angle)
                p = Point()
                p.x = x
                p.y = y
                p.z = 0  # Since this is 2D data
                marker.points.append(p)

            marker.lifetime = rospy.Duration(1)  # How long the marker will be displayed
            marker_array.markers.append(marker)

    # Publish the new marker array
        self.marker_pub.publish(marker_array)


if __name__ == '__main__':
    try:
        ac = AdaptiveClustering()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
