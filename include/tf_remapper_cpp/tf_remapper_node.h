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
    ros::Subscriber oldTfSubscriber, remappedTfSubscriber;
    ros::Publisher remappedTfPublisher, oldTfPublisher;

    bool staticTf;
    tf2_msgs::TFMessage staticTfCache, reverseStaticTfCache;

    std::string oldTfTopic, remappedTfTopic;

    TfRemapper tfRemapper, reverseTfRemapper;

    void oldTfCallback(const ros::MessageEvent<tf2_msgs::TFMessage const>& event);
    void remappedTfCallback(const ros::MessageEvent<tf2_msgs::TFMessage const>& event);
    void addToStaticTfCache(const tf2_msgs::TFMessage& message, tf2_msgs::TFMessage& cache) const;
};

};

#endif //TF_REMAPPER_NODE_CPP_TF_REMAPPER_H
