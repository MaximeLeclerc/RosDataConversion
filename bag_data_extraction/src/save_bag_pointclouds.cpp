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
    ros::NodeHandle privateNodeHandle("~");

    string bagfile;
    string path;
    string filename;
    string fileExtension;
    int keepOneOutOf;
    privateNodeHandle.param("bagfile", bagfile, string("cloud"));
    privateNodeHandle.param("filename", path, string("cloud"));
    privateNodeHandle.param("path", path, string("/home/smichaud/Desktop"));
    privateNodeHandle.param("filename", filename, string("cloud"));
    privateNodeHandle.param("fileExtension", fileExtension, string("vtk"));
    privateNodeHandle.param("keedOneOutOf", keepOneOutOf, int(1));

    rosbag::Bag bag;
    bag.open(bagfile);

    std::vector<std::string> topics;
    topics.push_back(string("/cloud"));

    rosbag::View view(bag, rosbag::TopicQuery(topics));

    int index = 0;
    BOOST_FOREACH(rosbag::MessageInstance const msg, view) {
        shared_ptr<sensor_msgs::PointCloud2> cloudMsg =
            msg.instantiate<sensor_msgs::PointCloud2>();

        // [TODO]: Make sure you get one out of... - 2015-05-29 05:24pm
        if(cloudMsg != NULL) {
            if((index+1)%keepOneOutOf == 0) {
                std::cout << keepOneOutOf << std::endl;
                std::cout << "Working here" << std::endl;
                shared_ptr<PM::DataPoints> cloud(new PM::DataPoints(
                            rosMsgToPointMatcherCloud<float>(*cloudMsg)));

                cloud->save(path + "/" 
                        + filename + "_" 
                        + getPaddedNum(index, 3)
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
