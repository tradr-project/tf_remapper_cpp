#include <tf_remapper_cpp/tf_remapper_node.h>
#include <set>

tf_remapper_cpp::TfRemapperNode::TfRemapperNode() : privateNodeHandle("~")
{
    this->oldTfTopic = this->privateNodeHandle.param<std::string>("old_tf_topic_name", "/tf_old");
    this->remappedTfTopic = this->privateNodeHandle.param<std::string>("new_tf_topic_name", "/tf");

    if (this->privateNodeHandle.hasParam("static_tf"))
        this->staticTf = this->privateNodeHandle.param<bool>("static_tf", false);
    else
        this->staticTf = this->remappedTfTopic == "tf_static" || this->remappedTfTopic == "/tf_static";

    // Parse the 'mappings' parameter, which should be an array of dicts, e.g. [{"old": "b", "new": "d"}]
    XmlRpc::XmlRpcValue mappingsParam;
    const bool hasMappingsParam = this->privateNodeHandle.getParam("mappings", mappingsParam);
    if (!hasMappingsParam)
        throw ros::InvalidParameterException("tf_remapper_cpp needs the private parameter 'mappings' set.");

    this->tfRemapper = TfRemapper(mappingsParam);

    if (!tfRemapper.getMappings().empty()) {
        ROS_INFO("Applying the following mappings to incoming tf frame ids:");
        for (TfRemapper::MappingsType::const_iterator it = tfRemapper.getMappings().begin();
                it != tfRemapper.getMappings().end(); ++it) {
            ROS_INFO_STREAM("* " << it->first << " -> " << (it->second.empty() ? "DELETE" : it->second));
        }
    } else {
        ROS_WARN("No mappings defined.");
    }

    ROS_INFO_STREAM("Old TF topic: " << this->oldTfTopic);
    ROS_INFO_STREAM("Remapped TF topic: " << this->remappedTfTopic);
    if (this->staticTf)
        ROS_INFO("Running in static TF mode (caching all TFs, latched publisher)");

    this->oldTfSubscriber = this->publicNodeHandle.subscribe(
            this->oldTfTopic, 100, &TfRemapperNode::oldTfCallback, this);
    this->remappedTfPublisher = this->publicNodeHandle.advertise<tf2_msgs::TFMessage>(
        this->remappedTfTopic, 100, this->staticTf);
}

void tf_remapper_cpp::TfRemapperNode::oldTfCallback(const ros::MessageEvent<tf2_msgs::TFMessage const>& event) {
    // Prevent reacting to own messages
    const std::string& callerid = event.getPublisherName();
    if (callerid == ros::this_node::getName())
        return;

    tf2_msgs::TFMessage message = this->tfRemapper.doRemapping(*event.getConstMessage());

    // Since static TF can come from many latched publishers, and we are only a single publisher, we must gather all
    // the static TF messages in a cache and every time publish all of them.
    if (this->staticTf) {
        this->addToStaticTfCache(message);
        this->remappedTfPublisher.publish(this->staticTfCache);
    } else {
        this->remappedTfPublisher.publish(message);
    }
}

void tf_remapper_cpp::TfRemapperNode::addToStaticTfCache(const tf2_msgs::TFMessage& message) {
    // We do an inefficient O(N^2) search here, but there should not be that many static TFs that it would matter
    for (std::vector<geometry_msgs::TransformStamped>::const_iterator it = message.transforms.begin();
            it != message.transforms.end(); ++it) {
        bool found = false;
        for (std::vector<geometry_msgs::TransformStamped>::iterator cacheIt = this->staticTfCache.transforms.begin();
                cacheIt != this->staticTfCache.transforms.end(); ++cacheIt) {
            if (it->child_frame_id == cacheIt->child_frame_id) {
                found = true;
                *cacheIt = *it;
                break;
            }
        }
        if (!found)
            this->staticTfCache.transforms.push_back(*it);
    }
}

int main(int argc, char * argv[])
{
    ros::init(argc, argv, "tf_remapper");
    tf_remapper_cpp::TfRemapperNode remapper;
    ros::spin();
    return 0;
}
