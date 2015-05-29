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
using namespace PointMatcher_ros;
std::string appendNum(const std::string &input, const int &numSuffix);

int main(int argc, char **argv) {
    rosbag::Bag bag;
    bag.open("/home/smichaud/Desktop/test/test.bag", rosbag::bagmode::Read);

    std::vector<std::string> topics;
    topics.push_back(std::string("/cloud"));

    rosbag::View view(bag, rosbag::TopicQuery(topics));

    int index = 0;
    BOOST_FOREACH(rosbag::MessageInstance const msg, view) {
        shared_ptr<sensor_msgs::PointCloud2> cloudMsg =
            msg.instantiate<sensor_msgs::PointCloud2>();

        if(cloudMsg != NULL) {
            shared_ptr<PM::DataPoints> cloud(new PM::DataPoints(
                        rosMsgToPointMatcherCloud<float>(*cloudMsg)));

            std::string path = "/home/smichaud/Desktop/test/pointclouds/";
            std::string filename = "cloud_";
            filename = appendNum(filename, index);
            cloud->save(path + filename + ".vtk");
            index++;
        }
    }

    bag.close();
}

std::string appendNum(const std::string &input, const int &numSuffix) {
    std::ostringstream output;
    output << input << std::setfill('0') << std::setw(3) << numSuffix;

    return output.str();
}
