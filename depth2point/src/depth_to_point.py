#!/usr/bin/env python3

import rclpy
from sensor_msgs.msg import Image, PointCloud2, PointField
from cv_bridge import CvBridge
import numpy as np

from rclpy.node import Node

import sensor_msgs.msg as sensor_msgs
import std_msgs.msg as std_msgs

import cv2
import open3d as o3d



bridge = CvBridge()

class Pointcloud_publisher(Node):
    def __init__(self):
        super().__init__('depth_to_pointcloud')

        self.subscription = self.create_subscription(
            Image,
            '/depth_result',
            self.depth_image_callback,
            1)

        self.point_cloud_publisher = self.create_publisher(PointCloud2, '/point_cloud_topic', 10)
    

    def depth_image_callback(self, msg):
        depth_image = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        depth_image = cv2.flip(depth_image, 0 )
        depth_image = o3d.geometry.Image(np.array(depth_image).astype(np.float32))
        # Get camera intrinsics (replace these with your actual camera parameters)
        fx, fy, cx, cy = 554.3827128226441, 554.3827128226441, 320.5, 240.5

        intrinsic_matrix = np.array([[fx, 0, cx],[0, fy, cy],[0, 0, 1]])
        camera_intrinsic = o3d.camera.PinholeCameraIntrinsic()
        camera_intrinsic.set_intrinsics(width=640, height=480,
        fx=fx, fy=fy,
        cx=cx, cy=cy)  
        pcd = o3d.geometry.PointCloud.create_from_depth_image(depth_image, camera_intrinsic)
        points = np.asarray(pcd.points)  

        ros_dtype = sensor_msgs.PointField.FLOAT32
        dtype = np.float32
        itemsize = np.dtype(dtype).itemsize # A 32-bit float takes 4 bytes.

        data = points.astype(dtype).tobytes() 


        fields = [sensor_msgs.PointField(
            name=n, offset=i*itemsize, datatype=ros_dtype, count=1)
            for i, n in enumerate('yzx')]


        header = std_msgs.Header(frame_id='link')
        pcd_msg = sensor_msgs.PointCloud2(
            header=header,
            height=1, 
            width=points.shape[0],
            is_dense=False,
            is_bigendian=False,
            fields=fields,
            point_step=(itemsize * 3), # Every point consists of three float32s.
            row_step=(itemsize * 3 * points.shape[0]), 
            data=data
        )
        self.point_cloud_publisher.publish(pcd_msg)


if __name__ == '__main__':
    rclpy.init(args=None)
    point_image = Pointcloud_publisher()
    rclpy.spin(point_image)
    rclpy.shutdown()   
