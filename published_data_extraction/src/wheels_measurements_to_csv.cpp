#include "ros/ros.h"
#include <iostream>
#include <fstream>
#include "clearpath_base/Encoders.h"

using namespace std;

class WheelsMeasurementsExtractor {
private:
    vector<ros::Time> timestamps;
    vector<double> leftPosition;
    vector<double> rightPosition;
    vector<double> leftSpeed;
    vector<double> rightSpeed;

    ros::Subscriber subscriber;
    ros::NodeHandle subscriberNodeHandle;
    const string subscribeTopic;

public:
    WheelsMeasurementsExtractor(ros::NodeHandle& subscriberNodeHandle):
        subscriberNodeHandle(subscriberNodeHandle),
        subscribeTopic("/husky/data/encoders"){
        subscriber = this->subscriberNodeHandle.subscribe(
                    subscribeTopic, 1000,
                    &WheelsMeasurementsExtractor::gotMeasurement, this);
    }
    void gotMeasurement(
            const clearpath_base::Encoders wheelsMeasurements){
        leftPosition.push_back(wheelsMeasurements.encoders[0].travel);
        rightPosition.push_back(wheelsMeasurements.encoders[1].travel);

        leftSpeed.push_back(wheelsMeasurements.encoders[0].speed);
        rightSpeed.push_back(wheelsMeasurements.encoders[1].speed);

        timestamps.push_back(wheelsMeasurements.header.stamp);
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
                           << ", " << this->leftPosition[i]
                              << ", " << this->rightPosition[i]
                                    << ", " << this->leftSpeed[i]
                                       << ", " << this->rightSpeed[i] << endl;
            }
        }
    }
    bool haveData(){
        return !(this->timestamps.empty());
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "wheels_measurements_extractor");

    ros::NodeHandle nodeHandle("~");
    string fileName;
    nodeHandle.param<string>("output", fileName,
                             "~/.ros/wheels_measurements.csv");
    cout << fileName << endl;

    WheelsMeasurementsExtractor wheelsMeasurementsExtractor(nodeHandle);

    ros::spin();

    if(wheelsMeasurementsExtractor.haveData()){
        wheelsMeasurementsExtractor.saveCSV(fileName);
    } else {
        cout << endl << "No data to save ..." <<  endl;
    }

    return 0;
}




