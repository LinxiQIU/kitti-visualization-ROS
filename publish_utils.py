#!/usr/bin/env python

import rospy
from std_msgs.msg import Header
from sensor_msgs.msg import Image, PointCloud2, Imu 
import sensor_msgs.point_cloud2 as pcl2 
from cv_bridge import CvBridge
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import tf

FRAME_ID = 'map'

def publish_camera(cam_pub, bridge, image):
	cam_pub.publish(bridge.cv2_to_imgmsg(image, "bgr8"))

def publish_pcl(pcl_pub, point_cloud):
	header = Header()
	header.stamp = rospy.Time.now()
	header.frame_id = FRAME_ID
	pcl_pub.publish(pcl2.create_cloud_xyz32(header, point_cloud[:, :3]))

def publish_ego_car(ego_car_pub):
	marker = Marker()
	marker.header.frame_id = FRAME_ID
	marker.header.stamp = rospy.Time.now()

	marker.id = 0
	marker.action = Marker.ADD
	marker.lifetime = rospy.Duration()
	marker.type = Marker.LINE_STRIP

	marker.color.r = 0.0
	marker.color.g = 1.0
	marker.color.b = 0.0
	marker.color.a = 1.0
	marker.scale.x = 0.15

	marker.points = []
	marker.points.append(Point(12,-12, 0))
	marker.points.append(Point(0, 0, 0))
	marker.points.append(Point(12,12, 0))

	ego_car_pub.publish(marker)

def publish_imu(imu_pub, imu_data):
	imu = Imu()
	imu.header.frame_id = FRAME_ID
	imu.header.stamp = rospy.Time.now()

	q = tf.transformations.quaternion_from_euler(float(imu_data.roll), float(imu_data.pitch), float(imu_data.yaw))
	imu.orientation.x = q[0]
	imu.orientation.y = q[1]
	imu.orientation.z = q[2]
	imu.orientation.w = q[3]
	imu.linear_acceleration.x = imu_data.af
	imu.linear_acceleration.y = imu_data.al
	imu.linear_acceleration.z = imu_data.au
	imu.angular_velocity.x = imu_data.wf
	imu.angular_velocity.y = imu_data.wl
	imu.angular_velocity.z = imu_data.wu

	imu_pub.publish(imu)









