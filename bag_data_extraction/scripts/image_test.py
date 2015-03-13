#!/usr/bin/env python

import sys

import rospy
import rosbag
import h5py

import cv2
from cv_bridge import CvBridge, CvBridgeError

import h5py
import numpy

def main():
    bagFile = rosbag.Bag('/home/smichaud/Desktop/test.bag')
    topic = '/usb_cam/image_raw_drop'

    hdf5File = h5py.File('/home/smichaud/Desktop/image.hdf5', "w")
    dataset = hdf5File.create_dataset(name='Camera/images',
                                      shape=(480, 640, 3, 1),
                                      chunks=(480, 640, 3, 1),
                                      maxshape=(480, 640, 3, None),
                                      dtype='uint8')

    imageIndex = 0;
    bridge = CvBridge()
    for topic, msg, t in bagFile.read_messages(topics=[topic]):
        try:
            cv_image = bridge.imgmsg_to_cv(img_msg=msg, desired_encoding="passthrough")
        except CvBridgeError, e:
            print "Couldn't convert ROS sensor_msgs/Image to OpenCV data type: "
            print e

        numpyImage = numpy.asarray(cv_image)
        hdf5File['Camera/images'][:, :, :, imageIndex] = numpyImage

        imageIndex += 1
        dataset.resize(imageIndex+1, 3)

    bagFile.close()
    hdf5File.close()
    sys.exit(0)

if __name__=="__main__":
    main()