#!/usr/bin/env python

from data_utils import *
from publish_utils import *
import os

DATA_PATH = '/home/qlx/My_data/kitti/Rawdata/2011_09_26/2011_09_26_drive_0005_sync/'

if __name__ == '__main__':
	rospy.init_node('kitti_node', anonymous=True)
	cam_pub = rospy.Publisher('kitti_cam', Image, queue_size=10)
	pcl_pub = rospy.Publisher('kitti_pcl', PointCloud2, queue_size=10)
	ego_pub = rospy.Publisher('kitti_ego_car', Marker, queue_size=10)
	imu_pub = rospy.Publisher('kitti_imu', Imu, queue_size=10)
	bridge = CvBridge()
	frame = 0

	rate = rospy.Rate(10)
	while not rospy.is_shutdown():

		image = read_camera(os.path.join(DATA_PATH, 'image_02/data/%010d.png'%frame))
		point_cloud = read_pcl(os.path.join(DATA_PATH, 'velodyne_points/data/%010d.bin'%frame))
		publish_camera(cam_pub, bridge, image)
		publish_pcl(pcl_pub, point_cloud)
		publish_ego_car(ego_pub)
		imu_data = read_imu(os.path.join(DATA_PATH, 'oxts/data/%010d.txt'%frame))
		publish_imu(imu_pub, imu_data)
		rospy.loginfo("published")
		rate.sleep()
		frame += 1
		frame %= 154
