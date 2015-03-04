#include "ros/ros.h"

#include "sensor_msgs/PointCloud2.h"
#include "std_msgs/Int32.h"

#include "rosbag/bag.h"
#include "rosbag/view.h"

#include "boost/foreach.hpp"

#include "pointmatcher/PointMatcher.h"
#include "pointmatcher_ros/point_cloud.h"
#include <cmath>
#include <sstream>
#include <iostream>
#include <fstream>

using namespace std;
using namespace Eigen;

typedef PointMatcher<float> PM;

int main(int argc, char **argv) {
    if(argc < 3) {
        cerr << "Usage: "
             << "rosrun bag_pointcloud2_converter bag_pointcloud2_to_csv "
             << "/path/to/input.bag " << "/path/to/file/velodyne" << endl;
        return 0;
    }

    string inputFileName(argv[1]);
    string outputFileName(argv[2]);

    ofstream outputFile;
    try{
        ostringstream numOuputFile;
        numOuputFile << outputFileName
                     << std::setfill('0') << std::setw(3) << 0;
        outputFile.open(numOuputFile.str().c_str());
    } catch(...) {
        cerr << "Unable to open output file: " << outputFileName << endl;
        return 0;
    }

    rosbag::Bag bag(inputFileName, rosbag::bagmode::Read);

    const string subscribeTopic = "/velodyne_points";
    rosbag::View view(bag, rosbag::TopicQuery(subscribeTopic));

    unsigned splitLineLimit = 100000000;
    unsigned currentLineCount = 0;
    unsigned splitCount = 0;
    BOOST_FOREACH(rosbag::MessageInstance const msg, view) {
        sensor_msgs::PointCloud2ConstPtr pointCloud2 =
                msg.instantiate<sensor_msgs::PointCloud2>();

        if(pointCloud2 != NULL) {
            PM::DataPoints dataPoints(
                        PointMatcher_ros::rosMsgToPointMatcherCloud<float>(
                            *pointCloud2.get()));
            unsigned ringDescriptorIndex =
                    dataPoints.getDescriptorStartingRow("ring");
            unsigned intensityDescriptorIndex =
                    dataPoints.getDescriptorStartingRow("intensity");

            int pointsCount = dataPoints.features.cols();

            double timestamp = pointCloud2.get()->header.stamp.toSec();

            for(int i = 0; i < pointsCount ; ++i) {
                Vector3f point = dataPoints.features.col(i).head(3);

                int ringIndex =
                        dataPoints.descriptors.row(ringDescriptorIndex)(i);
                float range = point.norm();
                float angle = asin(point(1)/range);
                float intensity =
                        dataPoints.descriptors.row(intensityDescriptorIndex)(i);

                outputFile << std::setprecision (13)
                           << timestamp << ","
                           << std::setprecision (6)
                           << ringIndex << ","
                           << angle << ","
                           << range << ","
                           << intensity << endl;

                currentLineCount++;
            }
        }
        if(currentLineCount > splitLineLimit) {
            outputFile.close();
            currentLineCount = 0;
            splitCount++;
            try{
                ostringstream numOuputFile;
                numOuputFile << outputFileName
                             << std::setfill('0') << std::setw(3) << splitCount;
                outputFile.open(numOuputFile.str().c_str());
            } catch(...) {
                cerr << "Unable to open output file: "
                     << outputFileName
                     << std::setfill('0') << std::setw(3) << splitCount<< endl;
                return 0;
            }
        }
    }
    bag.close();

    outputFile.close();

    return 0;
}
