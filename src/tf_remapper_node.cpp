#include <tf_remapper_cpp/tf_remapper_node.h>
#include <set>

tf_remapper_cpp::TfRemapperNode::TfRemapperNode() : privateNodeHandle("~")
{
    this->oldTfTopic = this->privateNodeHandle.param<std::string>("old_tf_topic_name", "/tf_old");
    this->remappedTfTopic = this->privateNodeHandle.param<std::string>("new_tf_topic_name", "/tf");

    if (this->privateNodeHandle.hasParam("static_tf"))
        this->staticTf = this->privateNodeHandle.param<bool>("static_tf", false);
    else
        // Autodetect if the parameter is not set
        this->staticTf = this->remappedTfTopic == "tf_static" || this->remappedTfTopic == "/tf_static";

    // Parse the 'mappings' parameter, which should be an array of dicts, e.g. [{"old": "b", "new": "d"}]
    XmlRpc::XmlRpcValue mappingsParam;
    const bool hasMappingsParam = this->privateNodeHandle.getParam("mappings", mappingsParam);
    if (!hasMappingsParam) {
        mappingsParam.setSize(0); // makes it an empty array
        ROS_WARN("The 'mappings' parameter to tf_remap is missing");
    }

    const bool bidirectional = this->privateNodeHandle.param<bool>("is_bidirectional", false);

    this->tfRemapper = TfRemapper(mappingsParam);
    if (bidirectional)
        this->reverseTfRemapper = TfRemapper(mappingsParam, true);

    if (!tfRemapper.getMappings().empty()) {
        ROS_INFO_STREAM("Applying the following mappings" << (bidirectional ? " bidirectionally" : "") <<
                        " to incoming tf frame ids:");
        for (TfRemapper::MappingsType::const_iterator it = tfRemapper.getMappings().begin();
                it != tfRemapper.getMappings().end(); ++it) {
            ROS_INFO_STREAM("* " << it->first << " " << (bidirectional && !it->second.empty() ? "<" : "") << "-> " <<
                                    (it->second.empty() ? "DELETE" : it->second));
        }
        if (bidirectional) {
            for (TfRemapper::MappingsType::const_iterator it = reverseTfRemapper.getMappings().begin();
                    it != reverseTfRemapper.getMappings().end(); ++it) {
                if (it->second.empty())
                    ROS_INFO_STREAM("* DELETE <- " << it->first);
            }
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

    if (bidirectional) {
        this->remappedTfSubscriber = this->publicNodeHandle.subscribe(
                this->remappedTfTopic, 100, &TfRemapperNode::remappedTfCallback, this);
        this->oldTfPublisher = this->publicNodeHandle.advertise<tf2_msgs::TFMessage>(
                this->oldTfTopic, 100, this->staticTf);
    }
}

void tf_remapper_cpp::TfRemapperNode::oldTfCallback(const ros::MessageEvent<tf2_msgs::TFMessage const>& event) {
    // Prevent reacting to own messages from bidirectional mode
    const std::string& callerid = event.getPublisherName();
    if (callerid == ros::this_node::getName())
        return;

    tf2_msgs::TFMessage message = this->tfRemapper.doRemapping(*event.getConstMessage());

    // Since static TF can come from many latched publishers, and we are only a single publisher, we must gather all
    // the static TF messages in a cache and every time publish all of them.
    if (this->staticTf) {
        this->addToStaticTfCache(message, this->staticTfCache);
        this->remappedTfPublisher.publish(this->staticTfCache);
    } else {
        this->remappedTfPublisher.publish(message);
    }
}

void tf_remapper_cpp::TfRemapperNode::remappedTfCallback(const ros::MessageEvent<tf2_msgs::TFMessage const>& event) {
    // Prevent reacting to own messages from bidirectional mode
    const std::string& callerid = event.getPublisherName();
    if (callerid == ros::this_node::getName())
        return;

    tf2_msgs::TFMessage message = this->reverseTfRemapper.doRemapping(*event.getConstMessage());

    // Since static TF can come from many latched publishers, and we are only a single publisher, we must gather all
    // the static TF messages in a cache and every time publish all of them.
    if (this->staticTf) {
        this->addToStaticTfCache(message, this->reverseStaticTfCache);
        this->oldTfPublisher.publish(this->reverseStaticTfCache);
    } else {
        this->oldTfPublisher.publish(message);
    }
}

void tf_remapper_cpp::TfRemapperNode::addToStaticTfCache(
        const tf2_msgs::TFMessage& message, tf2_msgs::TFMessage& cache) const {
    // We do an inefficient O(N^2) search here, but there should not be that many static TFs that it would matter
    for (std::vector<geometry_msgs::TransformStamped>::const_iterator it = message.transforms.begin();
            it != message.transforms.end(); ++it) {
        bool found = false;
        for (std::vector<geometry_msgs::TransformStamped>::iterator cacheIt = cache.transforms.begin();
                cacheIt != cache.transforms.end(); ++cacheIt) {
            if (it->child_frame_id == cacheIt->child_frame_id) {
                found = true;
                *cacheIt = *it;
                break;
            }
        }
        if (!found)
            cache.transforms.push_back(*it);
    }
}

int main(int argc, char * argv[])
{
    ros::init(argc, argv, "tf_remapper");
    tf_remapper_cpp::TfRemapperNode remapper;
    ros::spin();
    return 0;
}
