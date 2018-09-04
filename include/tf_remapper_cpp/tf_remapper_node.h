#ifndef TF_REMAPPER_NODE_CPP_TF_REMAPPER_H
#define TF_REMAPPER_NODE_CPP_TF_REMAPPER_H

#include <ros/ros.h>
#include <tf2_msgs/TFMessage.h>
#include <ros/message.h>

#include <tf_remapper_cpp/tf_remapper.h>

namespace tf_remapper_cpp {

class TfRemapperNode {
public:
    TfRemapperNode();
    virtual ~TfRemapperNode() {};

protected:
    ros::NodeHandle publicNodeHandle, privateNodeHandle;
    ros::Subscriber oldTfSubscriber;
    ros::Publisher remappedTfPublisher;

    bool staticTf;

    std::string oldTfTopic, remappedTfTopic;

    TfRemapper tfRemapper;

    void oldTfCallback(const ros::MessageEvent<tf2_msgs::TFMessage const>& event);
};

};

#endif //TF_REMAPPER_NODE_CPP_TF_REMAPPER_H
