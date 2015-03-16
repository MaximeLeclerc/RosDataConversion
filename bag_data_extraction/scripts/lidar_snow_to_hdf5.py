#!/usr/bin/env python

import rosbag
import h5py
import numpy
import sys
import math

import cv2
from cv_bridge import CvBridge, CvBridgeError

import read_pointcloud2

def validateArgs(argv):
    if len(sys.argv) != 3:
        print("Usage: ")
        print("rosrun bag_data_extraction lidar_snow_to_hdf5 bagfile.bag hdf5File")
        sys.exit(0)
    else:
        bagFilename = argv[1]
        hdf5Filename = argv[2]

    return (bagFilename, hdf5Filename)

def openFiles(bagFilename, hdf5Filename):
    try:
        bagFile = rosbag.Bag(bagFilename)
    except:
        print("Unable to open bag file")
        sys.exit(0)
    try:
        hdf5File = h5py.File(hdf5Filename, "w")
    except:
        ("Unable to open hdf5 file")
        sys.exit(0)

    return (bagFile, hdf5File)

def closeFiles(bagFile, hdf5File):
    hdf5File.close()
    bagFile.close()

def extractLMS200(bagFile, hdf5File):
    beamsPerScan = 181
    scanPerChunk = 1000

    datasetName = "LMS200/ranges"
    rangesDataset = hdf5File.create_dataset(name=datasetName,
                                            shape=(scanPerChunk, beamsPerScan),
                                            chunks=(scanPerChunk, beamsPerScan),
                                            maxshape=(None, beamsPerScan))

    chunkIndex = 0
    scanIndex = 0
    chunk = numpy.zeros((scanPerChunk, beamsPerScan), 'f')

    for topic, msg, t in bagFile.read_messages(topics=['/lms200/scan']):
        beamsCount = len(msg.ranges)
        chunk[scanIndex, 0:beamsCount] = numpy.array(msg.ranges).reshape(beamsCount)
        scanIndex += 1

        if scanIndex == scanPerChunk:
            startIndex = scanPerChunk * chunkIndex
            endIndex = scanPerChunk * (chunkIndex + 1)
            hdf5File[datasetName][startIndex:endIndex, :] = chunk

            scanIndex = 0
            chunkIndex += 1

            rangesDataset.resize(scanPerChunk * (chunkIndex + 1), 0)

    startIndex = scanPerChunk * chunkIndex
    endIndex = startIndex + scanIndex
    hdf5File[datasetName][startIndex:endIndex, :] = chunk[0:scanIndex, :]

    rangesDataset.resize(endIndex,0)  # To discard unused space in the matrix (filled with zeros)

def extractLMS151(bagFile, hdf5File):
    beamsPerScan = 541
    scanPerChunk = 1000

    datasetRanges = "LMS151/ranges"
    rangesDataset = hdf5File.create_dataset(name=datasetRanges,
                                            shape=(scanPerChunk, beamsPerScan),
                                            chunks=(scanPerChunk, beamsPerScan),
                                            maxshape=(None, beamsPerScan))
    datasetIntensities = "LMS151/intensities"
    intensitiesDataset = hdf5File.create_dataset(name=datasetIntensities,
                                                 shape=(scanPerChunk, beamsPerScan),
                                                 chunks=(scanPerChunk, beamsPerScan),
                                                 maxshape=(None, beamsPerScan))

    chunkIndex = 0
    scanIndex = 0
    chunkRanges = numpy.zeros((scanPerChunk, beamsPerScan), 'f')
    chunkIntensities = numpy.zeros((scanPerChunk, beamsPerScan), 'f')

    for topic, msg, t in bagFile.read_messages(topics=['/lms151/scan']):
        beamsCount = len(msg.ranges)
        chunkRanges[scanIndex, 0:beamsCount] = numpy.array(msg.ranges).reshape(beamsCount)
        chunkIntensities[scanIndex, 0:beamsCount] = numpy.array(msg.intensities).reshape(beamsCount)
        scanIndex += 1

        if scanIndex == scanPerChunk:
            startIndex = scanPerChunk * chunkIndex
            endIndex = scanPerChunk * (chunkIndex + 1)
            hdf5File[datasetRanges][startIndex:endIndex, :] = chunkRanges
            hdf5File[datasetIntensities][startIndex:endIndex, :] = chunkIntensities

            scanIndex = 0
            chunkIndex += 1

            rangesDataset.resize(scanPerChunk * (chunkIndex + 1), 0)
            intensitiesDataset.resize(scanPerChunk * (chunkIndex + 1), 0)


    startIndex = scanPerChunk * chunkIndex
    endIndex = startIndex + scanIndex

    hdf5File[datasetRanges][startIndex:endIndex, :] = chunkRanges[0:scanIndex, :]
    hdf5File[datasetIntensities][startIndex:endIndex, :] = chunkIntensities[0:scanIndex, :]

    rangesDataset.resize(endIndex, 0)  # To discard unused space in the matrix (filled with zeros)
    intensitiesDataset.resize(endIndex, 0)  # To discard unused space in the matrix (filled with zeros)

def extractHokuyo(bagFile, hdf5File):
    beamsPerScan = 1081
    scanPerChunk = 1000
    maxEchoesCount = 5

    datasetRanges = "Hokuyo/ranges"
    rangesDataset = hdf5File.create_dataset(name=datasetRanges,
                                            shape=(maxEchoesCount, scanPerChunk, beamsPerScan),
                                            chunks=(maxEchoesCount, scanPerChunk, beamsPerScan),
                                            maxshape=(maxEchoesCount, None, beamsPerScan))
    datasetIntensities = "Hokuyo/intensities"
    intensitiesDataset = hdf5File.create_dataset(name=datasetIntensities,
                                                 shape=(maxEchoesCount, scanPerChunk, beamsPerScan),
                                                 chunks=(maxEchoesCount, scanPerChunk, beamsPerScan),
                                                 maxshape=(maxEchoesCount, None, beamsPerScan))

    chunkIndex = 0
    scanIndex = 0
    chunkRanges = numpy.zeros((maxEchoesCount, scanPerChunk, beamsPerScan), 'f')
    chunkIntensities = numpy.zeros((maxEchoesCount, scanPerChunk, beamsPerScan), 'f')

    for topic, msg, t in bagFile.read_messages(topics=['/echoes']):
        beamsCount = len(msg.ranges)
        if beamsCount == beamsPerScan:  # Ignore incomplete scans
            for beamIndex in range(0, beamsCount):
                echoesCount = len(msg.ranges[beamIndex].echoes)
                chunkRanges[0:echoesCount, scanIndex, beamIndex] = msg.ranges[beamIndex].echoes
                chunkIntensities[0:echoesCount, scanIndex, beamIndex] = msg.intensities[beamIndex].echoes

            scanIndex += 1

            if scanIndex == scanPerChunk:
                startIndex = scanPerChunk * chunkIndex
                endIndex = scanPerChunk * (chunkIndex + 1)

                hdf5File[datasetRanges][:, startIndex:endIndex, :] = chunkRanges
                hdf5File[datasetIntensities][:, startIndex:endIndex, :] = chunkIntensities

                scanIndex = 0
                chunkIndex += 1

                rangesDataset.resize(scanPerChunk * (chunkIndex + 1), 1)
                intensitiesDataset.resize(scanPerChunk * (chunkIndex + 1), 1)

    startIndex = scanPerChunk * chunkIndex
    endIndex = startIndex + scanIndex

    hdf5File[datasetRanges][:, startIndex:endIndex, :] = chunkRanges[:, 0:scanIndex, :]
    hdf5File[datasetIntensities][:, startIndex:endIndex, :] = chunkIntensities[:, 0:scanIndex, :]

    rangesDataset.resize(endIndex, 1)  # To discard unused space in the matrix (filled with zeros)
    intensitiesDataset.resize(endIndex, 1)  # To discard unused space in the matrix (filled with zeros)

def extractVelodyne(bagFile, hdf5File):
    pointsPerChunk = 100000
    valuesPerPoint = 5

    datasetName = "Velodyne/points"
    pointsDataset = hdf5File.create_dataset(name=datasetName,
                                            shape=(pointsPerChunk, valuesPerPoint),
                                            chunks=(pointsPerChunk, valuesPerPoint),
                                            maxshape=(None, valuesPerPoint))

    chunkIndex = 0
    pointIndex = 0
    chunk = numpy.zeros((pointsPerChunk, valuesPerPoint), 'f')
    for topic, msg, t in bagFile.read_messages(topics=['/velodyne_points']):
        stamp = msg.header.stamp.nsecs

        # input point = (x,y,z,intensity,ring)
        # output point = (timestamp | ring | angle | range | intensity)
        for point in read_pointcloud2.read_points(msg):
            range = numpy.linalg.norm((point[0],point[1],point[2]))
            angle = math.asin(point[1]/range)
            chunk[pointIndex, :] = (stamp, point[4], angle, range, point[3])

            pointIndex += 1         

            if pointIndex == pointsPerChunk:
                startIndex = pointsPerChunk * chunkIndex
                endIndex = pointsPerChunk * (chunkIndex + 1)

                hdf5File[datasetName][startIndex:endIndex, :] = chunk

                pointIndex = 0
                chunkIndex += 1

                pointsDataset.resize(pointsPerChunk * (chunkIndex + 1), 0)

    startIndex = pointsPerChunk * chunkIndex
    endIndex = startIndex + pointIndex

    hdf5File[datasetName][startIndex:endIndex, :] = chunk[0:pointIndex, :]

    pointsDataset.resize(endIndex, 0)  # To discard unused space in the matrix (filled with zeros)

def extractCamera(bagFile, hdf5File):
    datasetName = 'Camera/images'
    dataset = hdf5File.create_dataset(name='Camera/images',
                                      shape=(480, 640, 3, 1),
                                      chunks=(480, 640, 3, 1),
                                      maxshape=(480, 640, 3, None),
                                      dtype='uint8')

    imageIndex = 0;
    bridge = CvBridge()
    for topic, msg, t in bagFile.read_messages(topics=['/usb_cam/image_raw_drop']):
        dataset.resize(imageIndex+1, 3)
        try:
            cv_image = bridge.imgmsg_to_cv(img_msg=msg, desired_encoding="passthrough")
        except CvBridgeError, exception:
            print "Couldn't convert ROS sensor_msgs/Image to OpenCV data type: "
            print exception

        numpyImage = numpy.asarray(cv_image)
        hdf5File['Camera/images'][:, :, :, imageIndex] = numpyImage

        imageIndex += 1




def main(args):
    bagFilename, hdf5Filename = validateArgs(args)

    bagFile, hdf5File = openFiles(bagFilename, hdf5Filename)

    print("Start the extraction...")

    extractCamera(bagFile, hdf5File)
    extractLMS200(bagFile, hdf5File)
    extractLMS151(bagFile, hdf5File)
    extractHokuyo(bagFile, hdf5File)
    extractVelodyne(bagFile, hdf5File)

    closeFiles(bagFile, hdf5File)

    print("Jobs done !")

if __name__=="__main__":
   main(sys.argv)
