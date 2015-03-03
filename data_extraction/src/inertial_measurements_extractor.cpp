#include "ros/ros.h"
#include <iostream>
#include <fstream>
#include <sensor_msgs/Imu.h>

using namespace std;

class WheelsMeasurementsExtractor {
private:
    vector<ros::Time> timestamps;
    vector<double> xAcceleration;
    vector<double> yAcceleration;
    vector<double> zAcceleration;
    vector<double> xGyro;
    vector<double> yGyro;
    vector<double> zGyro;

    ros::Subscriber subscriber;
    ros::NodeHandle subscriberNodeHandle;
    const string subscribeTopic;

public:
    WheelsMeasurementsExtractor(ros::NodeHandle& subscriberNodeHandle):
        subscriberNodeHandle(subscriberNodeHandle),
        subscribeTopic("/imu/data"){
        subscriber = this->subscriberNodeHandle.subscribe(
                    subscribeTopic, 1000,
                    &WheelsMeasurementsExtractor::gotMeasurement, this);
    }
    void gotMeasurement(
            const sensor_msgs::Imu inertialMeasurements){
        xAcceleration.push_back(inertialMeasurements.linear_acceleration.x);
        yAcceleration.push_back(inertialMeasurements.linear_acceleration.y);
        zAcceleration.push_back(inertialMeasurements.linear_acceleration.z);

        xGyro.push_back(inertialMeasurements.angular_velocity.x);
        yGyro.push_back(inertialMeasurements.angular_velocity.y);
        zGyro.push_back(inertialMeasurements.angular_velocity.z);

        timestamps.push_back(inertialMeasurements.header.stamp);
    }
    void saveCSV(const string fileName){
        ofstream outputFile;
        outputFile.open(fileName);
        if(!outputFile.is_open()) {
            cout << "Impossible to write to: " << fileName << endl;
        } else {
            cout << endl << "Saving data as: " << fileName << "..." << endl;
            for(unsigned i = 0 ; i < this->timestamps.size(); ++i) {
                ros::Duration timeFromBeginning = this->timestamps[i]
                        - this->timestamps[0];
                outputFile << timeFromBeginning.toSec()
                           << ", " << this->xAcceleration[i]
                              << ", " << this->yAcceleration[i]
                                 << ", " << this->zAcceleration[i]
                                    << ", " << this->xGyro[i]
                                       << ", " << this->yGyro[i]
                                          << ", " << this->zGyro[i] << endl;
            }
        }
    }
    bool haveData(){
        return !(this->timestamps.empty());
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "inertial_measurements_extractor");

    ros::NodeHandle nodeHandle("~");
    string fileName;
    nodeHandle.param<string>("output", fileName,
                             "~/.ros/inertial_measurements.csv");
    cout << fileName << endl;

    WheelsMeasurementsExtractor inertialMeasurementsExtractor(nodeHandle);

    ros::spin();

    if(inertialMeasurementsExtractor.haveData()){
        inertialMeasurementsExtractor.saveCSV(fileName);
    } else {
        cout << endl << "No data to save ..." <<  endl;
    }

    return 0;
}



