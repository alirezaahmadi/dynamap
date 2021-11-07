#!/usr/bin/python

# Extract images from a bag file.
#
# Original author: Thomas Denewiler (http://answers.ros.org/users/304/thomas-d/)

# Start up ROS pieces.
PKG = 'kinect_extract_imgs'
import roslib; roslib.load_manifest(PKG)
import rosbag
import rospy
import cv2
import numpy as np
import matplotlib

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

# Reading bag filename from command line or roslaunch parameter.
import os
import sys

class ImageCreator():

    image_type = ".pgm"
    desired_topic = "image_raw"  
    image_format = "d"
    index_in_filename = True
    index_format = "06d"
    image_index = 0

    # Must have __init__(self) function for a class, similar to a C++ class constructor.
    def __init__(self):
        # Get parameters when starting node from a launch file.
        if len(sys.argv) < 1:
            save_dir = rospy.get_param('save_dir')

            filename = rospy.get_param('filename')

            rospy.loginfo("Bag filename = %s", filename)
        # Get parameters as arguments to 'rosrun my_package bag_to_images.py <save_dir> <filename>', where save_dir and filename exist relative to this executable file.
        else:
            save_dir = os.path.join(sys.path[0], sys.argv[1])
            filename = os.path.join(sys.path[0], sys.argv[2])
            rospy.loginfo("Bag filename = %s", filename)

        # Use a CvBridge to convert ROS images to OpenCV images so they can be saved.
        self.bridge = CvBridge()

        # Open bag file.
        with rosbag.Bag(filename, 'r') as bag:
            for topic, msg, t in bag.read_messages():
                topic_parts = topic.split('/')

                # first part is empty string
                if len(topic_parts) == 4 and topic_parts[3] == self.desired_topic:

                    try:
                        cv_image = self.bridge.imgmsg_to_cv2(msg)
                        cv_image_array = np.array(cv_image, dtype = np.dtype('f8'))
                        cv_image_norm = cv2.normalize(cv_image_array, cv_image_array, 0, 1, cv2.NORM_MINMAX)
                        #cv_image_array = np.array(cv_image_norm, dtype = np.dtype('8UC3'))
                        cv2.imshow("Image from my node",cv_image)
                        dup=[]          
                        for k in cv_image:
                            for i in k:
                                dup.append(i)
                        print (max(dup), min(dup))
                        cv2.waitKey()
                        print (cv_image.dtype)
                        print (cv_image)
                        print (cv_image_array)
                        #print np.amax(cv_image)
                    except CvBridgeError, e:
                        print e

                    timestr = "%.6f" % msg.header.stamp.to_sec()

                    if self.index_in_filename:

                        image_name = str(save_dir) + "/" + self.image_format + "-" + timestr + "-" + format(self.image_index, self.index_format) + self.image_type
                    else:
                        image_name = str(save_dir) + "/" + topic_parts[1] + "-" + timestr + self.image_type
                        #skimage.io.imsave(image_name,)
                        #matplotlib.pyplot.imsave(fname, arr, **kwargs)[source]
                    cv2.imwrite(image_name, cv_image)

                    self.image_index = self.image_index + 1

# Main function.    
if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node(PKG)
    # Go to class functions that do all the heavy lifting. Do error checking.
    try:
        image_creator = ImageCreator()
    except rospy.ROSInterruptException: pass