#!/usr/bin/env python3

import argparse
import cv2
import numpy as np
import os
import torch
import torch.nn.functional as F
from torchvision.transforms import Compose
from tqdm import tqdm

from depth_anything.dpt import DepthAnything
from depth_anything.util.transform import Resize, NormalizeImage, PrepareForNet

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

bridge = CvBridge()

class Camera_subscriber(Node):
    def __init__(self):
        super().__init__('original_image')

        self.subscription = self.create_subscription(
            Image,
            'camera_sensor/image_raw',
            self.camera_callback,
            10)
        self.subscription

        self.rgbd_pub = self.create_publisher(Image, "/depth_result",1)

    def camera_callback(self, data):
        img = bridge.imgmsg_to_cv2(data, "bgr8")

        DEVICE = 'cuda' if torch.cuda.is_available() else 'cpu'

        depth_anything = DepthAnything.from_pretrained('LiheYoung/depth_anything_vits14').to(DEVICE).eval()

        total_params = sum(param.numel() for param in depth_anything.parameters())
        print('Total parameters: {:.2f}M'.format(total_params / 1e6))  

        transform = Compose([
            Resize(
                width=518,
                height=518,
                resize_target=False,
                keep_aspect_ratio=True,
                ensure_multiple_of=14,
                resize_method='lower_bound',
                image_interpolation_method=cv2.INTER_CUBIC,
            ),
            NormalizeImage(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225]),
            PrepareForNet(),
        ])

        image = cv2.cvtColor(img, cv2.COLOR_BGR2RGB) / 255.0

        h, w = image.shape[:2]
        
        image = transform({'image': image})['image']
        image = torch.from_numpy(image).unsqueeze(0).to(DEVICE)


        with torch.no_grad():
            depth = depth_anything(image)
        
        depth = F.interpolate(depth[None], (h, w), mode='bilinear', align_corners=False)[0, 0]
        depth = (depth - depth.min()) / (depth.max() - depth.min()) * 255.0
        
        depth = depth.cpu().numpy().astype(np.uint8)

        img_msg = bridge.cv2_to_imgmsg(depth)

        self.rgbd_pub.publish(img_msg)

if __name__ == '__main__':
    rclpy.init(args=None)
    original_image = Camera_subscriber()
    rclpy.spin(original_image)
    rclpy.shutdown()    






    
