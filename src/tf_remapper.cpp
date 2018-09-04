#include <ros/ros.h>
#include <tf_remapper_cpp/tf_remapper.h>
#include <set>

tf_remapper_cpp::TfRemapper::TfRemapper() {}

tf_remapper_cpp::TfRemapper::~TfRemapper() {}

tf_remapper_cpp::TfRemapper::TfRemapper(const std::map<std::string, std::string> mappings) {
    for (TfRemapper::MappingsType::const_iterator it = mappings.begin(); it != mappings.end(); ++it) {
        if (it->first.empty())
            throw ros::InvalidParameterException("Key 'old' in each mapping has to be set to a nonempty string.");
    }

    this->mappings = mappings;
}

tf_remapper_cpp::TfRemapper::TfRemapper(const XmlRpc::XmlRpcValue& mappingsParam) {
    if(mappingsParam.getType() != XmlRpc::XmlRpcValue::TypeArray)
        throw ros::InvalidParameterException("The 'mappings' parameter must be an array of dictionaries.");

    for (size_t i = 0; i < mappingsParam.size(); ++i) {
        XmlRpc::XmlRpcValue mapping = mappingsParam[i];
        if (mapping.getType() != XmlRpc::XmlRpcValue::TypeStruct)
            throw ros::InvalidParameterException("The 'mappings' parameter must be an array of dictionaries.");

        std::string oldTopic;
        std::string newTopic;
        for (XmlRpc::XmlRpcValue::iterator it = mapping.begin(); it != mapping.end(); ++it)
        {
            if (it->second.getType() != XmlRpc::XmlRpcValue::TypeString)
                throw ros::InvalidParameterException("Dict values must be strings");
            if (it->first == "old")
                oldTopic = (const std::string&)it->second;
            else if (it->first == "new")
                newTopic = (std::string&)it->second;
            else
                throw ros::InvalidParameterException("Each dict in the mapping has to have only keys 'old' and 'new'.");
        }

        if (oldTopic.empty())
            throw ros::InvalidParameterException("Key 'old' in each mapping has to be set to a nonempty string.");

        if (oldTopic == newTopic)
            ROS_WARN_STREAM("Remapped topic '" << oldTopic << "' is the same as the old topic.");

        this->mappings[oldTopic] = newTopic;
    }
}

tf2_msgs::TFMessage tf_remapper_cpp::TfRemapper::doRemapping(const tf2_msgs::TFMessage& inMessage) const {
    tf2_msgs::TFMessage message = inMessage;
    this->doRemapping(message);
    return message;
}

void tf_remapper_cpp::TfRemapper::doRemapping(tf2_msgs::TFMessage& message) const {
    for (std::vector<geometry_msgs::TransformStamped>::iterator it = message.transforms.begin();
         it != message.transforms.end(); /* ++it is called at the end of the loop to allow erasing */) {
        geometry_msgs::TransformStamped& transform = *it;
        if (this->mappings.find(transform.header.frame_id) != this->mappings.end()) {
            if (this->mappings.at(transform.header.frame_id).empty()) { // delete the transform
                it = message.transforms.erase(it);
                continue; // it already points to the next element or end()
            } else {
                transform.header.frame_id = this->mappings.at(transform.header.frame_id);
            }
        }
        if (this->mappings.find(transform.child_frame_id) != this->mappings.end()) {
            if (this->mappings.at(transform.child_frame_id).empty()) { // delete the transform
                it = message.transforms.erase(it);
                continue; // it already points to the next element or end()
            } else {
                transform.child_frame_id = this->mappings.at(transform.child_frame_id);
            }
        }
        ++it; // we did not delete the transform, so go to the next one
    }
}

const tf_remapper_cpp::TfRemapper::MappingsType& tf_remapper_cpp::TfRemapper::getMappings() const {
    return this->mappings;
}
