#!/usr/bin/env python

"""
Extract images from a rosbag.
"""

import os
import argparse

import cv2
import rospy
import rosbag
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

def main():
    """Extract a folder of images from a rosbag.
    """
    parser = argparse.ArgumentParser(description="Extract images from a ROS bag.")
    parser.add_argument("image_number", type=int, help="Number of images to extract.")
    parser.add_argument("bag_file", help="Input ROS bag.")
    parser.add_argument("output_dir", help="Output directory.")
    parser.add_argument("image_topic", help="Image topic.")
    parser.add_argument("image_encoding", help="depth: passthrough, colored image: bgr8")

    args = parser.parse_args()

    print "Extract images from %s on topic %s into %s" % (args.bag_file,
                                                          args.image_topic, args.output_dir)

    bag = rosbag.Bag(args.bag_file, "r")
    bridge = CvBridge()
    count = 0
    for topic, msg, t in bag.read_messages(topics=[args.image_topic]):

        if args.image_encoding == "passthorugh":
            # The depth image is a single-channel float32 image
            # the values is the distance in mm in z axis
            depth_image = bridge.imgmsg_to_cv2(msg, '32FC1')
            max_m_for_kinect = 5.0 ## You'll have to check out online for exact value 
            depth_image = np.clip(depth_image,0,max_m_for_kinect) ## Filter all values > 5.0 m
            scaling_factor = 5000.0
            depth_image = depth_image*scaling_factor #scaling the image to [0;65535]
            cv_image_array = np.array(depth_image,dtype=np.uint16) ## Creating the cv2 image

            cv2.imwrite(os.path.join(args.output_dir, "%06i.png" % count), cv_image_array)


        elif args.image_encoding == "bgr8":
            cv_image_array = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            cv2.imwrite(os.path.join(args.output_dir, "%06i.png" % count), cv_image_array)

        cv2.imshow("Image from my node", cv_image_array)
        cv2.waitKey(1)

        # print "Wrote image %i" % count
        count += 1
        if args.image_number < count and args.image_number != 0: 
            break

    bag.close()

    return

if __name__ == '__main__':
    main()