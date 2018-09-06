#ifndef TF_REMAPPER_NODE_CPP_TF_REMAPPER_H
#define TF_REMAPPER_NODE_CPP_TF_REMAPPER_H

#include <ros/ros.h>
#include <tf2_msgs/TFMessage.h>
#include <ros/message.h>

#include <tf_remapper_cpp/tf_remapper.h>

namespace tf_remapper_cpp {

//! \brief A node that remaps TF frame names according to the given set of rules.
//!
//! Parameters:
//! - mappings (array of dicts): The rules for TF frame name remapping, e.g. [{"old": "b", "new": "d"}]
//! - static_tf (bool): Whether the remapper acts on static TFs or not. If not set, it is autodetected from topic name.
//! - old_tf_topic_name (string, default '/tf_old'): The topic on which old TFs are subscribed.
//! - new_tf_topic_name (string, default '/tf'): The topic on which remapped TFs are published.
//! - is_bidirectional (bool, default False): If True, the remapper will also allow passing TFs published on the
//!                                           "remapped end" to the "old end". Pay special attention if you use any kind
//!                                           of multimaster solution or a custom topic transport. This node needs
//!                                           CallerIDs to be unchanged.
class TfRemapperNode {
public:
    //! Start the node.
    //! \throws ros::InvalidParameterException if the 'mappings' parameter has invalid value.
    TfRemapperNode();

    virtual ~TfRemapperNode() {};

protected:
    ros::NodeHandle publicNodeHandle; //! \brief ROS node handle.
    ros::NodeHandle privateNodeHandle; //! \brief Private ROS node handle.
    ros::Subscriber oldTfSubscriber; //! \brief Subscriber to /tf_old.
    ros::Subscriber remappedTfSubscriber; //! \brief Subscriber to /tf (only in bidirectional mode).
    ros::Publisher remappedTfPublisher; //! \brief Publisher of /tf.
    ros::Publisher oldTfPublisher; //! \brief Publisher of /tf_old (only in bidirectional mode).

    bool staticTf; //! \brief If true, this node works with static TF, which need special care.
    tf2_msgs::TFMessage staticTfCache; //! \brief Cache of static TF messages.
    tf2_msgs::TFMessage reverseStaticTfCache; //! \brief Cache of static TF messages in the reverse direction.

    std::string oldTfTopic; //! \brief Name of the old topic ("/tf_old").
    std::string remappedTfTopic; //! \brief Name of the remapped topic ("/tf").

    TfRemapper tfRemapper; //! \brief The remapper that actually changes the TF messages.
    TfRemapper reverseTfRemapper; //! \brief The remapper that actually changes the TF messages in reverse direction.

    //! \brief Callback when a TF message arrives on the old TF topic.
    //! \param event The TF message.
    void oldTfCallback(const ros::MessageEvent<tf2_msgs::TFMessage const>& event);

    //! \brief Callback when a TF message arrives on the remapped TF topic.
    //! \param event The TF message.
    void remappedTfCallback(const ros::MessageEvent<tf2_msgs::TFMessage const>& event);

    //! \brief Add the given TF message to the static TF cache. Newer TFs overwrite the older ones.
    //! \param message The message to add.
    //! \param cache The cache to update.
    void addToStaticTfCache(const tf2_msgs::TFMessage& message, tf2_msgs::TFMessage& cache) const;
};

};

#endif //TF_REMAPPER_NODE_CPP_TF_REMAPPER_H
