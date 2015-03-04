#include "ros/ros.h"

//#include "sensor_msgs/MultiEchoLaserScan"

#include "rosbag/bag.h"
#include "rosbag/view.h"

#include "boost/foreach.hpp"

#include <cmath>
#include <sstream>
#include <iostream>
#include <fstream>

using namespace std;

int main(int argc, char **argv) {
    if(argc < 3) {
        cerr << "Usage: "
             << "rosrun bag_pointcloud2_converter bag_hokuyo_to_csv "
             << "/path/to/input.bag " << "/path/to/file/hokuyo" << endl;
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

    const string subscribeTopic = "/echoes";
    rosbag::View view(bag, rosbag::TopicQuery(subscribeTopic));

    unsigned splitLineLimit = 20000;
    unsigned currentLineCount = 0;
    unsigned splitCount = 0;
    BOOST_FOREACH(rosbag::MessageInstance const msg, view) {
        //        sensor_msgs::PointCloud2ConstPtr pointCloud2 =
        //                msg.instantiate<sensor_msgs::PointCloud2>();

        //        if(pointCloud2 != NULL) {
        //        outputFile << std::setprecision (13)
        //                   << timestamp << ","
        //                   << std::setprecision (6)
        //                   << ringIndex << ","
        //                   << angle << ","
        //                   << range << ","
        //                   << intensity << endl;

        currentLineCount++;
    }
    //    }
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
    bag.close();

    outputFile.close();

    return 0;
}

