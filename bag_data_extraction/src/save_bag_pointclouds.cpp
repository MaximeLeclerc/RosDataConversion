#include "ros/ros.h"
#include "rosbag/bag.h"
#include "rosbag/view.h"

#include "pointmatcher_ros/point_cloud.h"
#include "pointmatcher/PointMatcher.h"
#include "boost/shared_ptr.hpp"
#include "boost/foreach.hpp"
#include <iostream>
#include <iomanip>

typedef PointMatcher<float> PM;
using boost::shared_ptr;
using PointMatcher_ros::rosMsgToPointMatcherCloud;
using std::string;

string getPaddedNum(const int &numSuffix, const int width);

int main(int argc, char **argv) {
    ros::init(argc, argv, "save_bag_pointclouds");
    ros::NodeHandle nodeHandle("~");

    string cloudTopic;
    string bagfile;
    string path;
    string filename;
    string fileExtension;
    int keepOneOutOf;
    nodeHandle.param("cloudTopic", cloudTopic, string("/cloud"));
    nodeHandle.param("bagfile", bagfile, string("cloud"));
    nodeHandle.param("filename", path, string("cloud"));
    nodeHandle.param("path", path, string("/home/maxime/Downloads"));
    nodeHandle.param("filename", filename, string("cloud"));
    nodeHandle.param("fileExtension", fileExtension, string("pcd"));
    nodeHandle.param("keepOneOutOf", keepOneOutOf, 1);

    rosbag::Bag bag;
    bag.open(bagfile);

    std::vector<std::string> topics;
    topics.push_back(cloudTopic);

    rosbag::View view(bag, rosbag::TopicQuery(topics));

    int index = 0;
    BOOST_FOREACH(rosbag::MessageInstance const msg, view) {
        shared_ptr<sensor_msgs::PointCloud2> cloudMsg =
            msg.instantiate<sensor_msgs::PointCloud2>();

        if(cloudMsg != NULL) {
            if(index%keepOneOutOf == 0) {
                shared_ptr<PM::DataPoints> cloud(new PM::DataPoints(
                            rosMsgToPointMatcherCloud<float>(*cloudMsg)));

                cloud->save(path + "/" 
                        + filename + "_" 
                        + getPaddedNum(index, 4)
                        + "." + fileExtension);
            }
            index++;
        }
    }

    bag.close();
}

string getPaddedNum(const int &numSuffix, const int width) {
    std::ostringstream output;
    output << std::setfill('0') << std::setw(width) << numSuffix;

    return output.str();
}
