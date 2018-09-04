#ifndef TF_REMAPPER_CPP_TF_REMAPPER_H
#define TF_REMAPPER_CPP_TF_REMAPPER_H

#include <XmlRpc.h>
#include <tf2_msgs/TFMessage.h>

namespace tf_remapper_cpp {

//! \brief Remap frames in TF messages according to given rules.
class TfRemapper {
public:
    typedef std::map<std::string, std::string> MappingsType;

    //! \brief Empty constructor with no remappings.
    TfRemapper();

    //! \brief Create the remapper from a constructed set of mappings.
    //! \param mappings The mappings to use (keys are old frame names, values are remapped names).
    //! \param reverse If true, switch role of keys and values in the given map. Entries with empty values are omitted.
    //! \throws ros::InvalidParameterException if an 'old' key has empty value.
    explicit TfRemapper(MappingsType mappings, bool reverse = false);

    //! \brief Create the remapper from a XmlRpcValue as read by ros::NodeHandle::getParam().
    //! \param mappingsParam The XML-RPC-parsed value of the ROS parameter 'mappings'. It is expected to be an array of
    //!                      dicts, each of which contains keys 'old' and 'new'. At least on of them has to have
    //!                      nonempty value. Empty values mark tf frames to be deleted in the given direction.
    //! \param reverse If true, switch role of 'old' and 'new' in the given map. Entries with empty 'new' are omitted.
    //! \throws ros::InvalidParameterException if an 'old' key has empty value.
    //! \throws ros::InvalidParameterException if the mappings do not satisfy the given format.
    explicit TfRemapper(const XmlRpc::XmlRpcValue& mappingsParam, bool reverse = false);

    virtual ~TfRemapper();

    //! \brief Rewrite TF frame names according to the rules given in constructor.
    //! \param message The original message which is to be modified.
    void doRemapping(tf2_msgs::TFMessage& message) const;

    //! \brief Rewrite TF frame names according to the rules given in constructor.
    //! \param inMessage The original message.
    //! \return A copy of the original message with rewritten TF frame names.
    tf2_msgs::TFMessage doRemapping(const tf2_msgs::TFMessage& inMessage) const;

    //! \brief Get the mappings this remapper uses.
    //! \return The mappings (keys are old frame names, values are remapped names).
    const MappingsType& getMappings() const;

protected:
    //! \brief The mappings (keys are old frame names, values are remapped names).
    MappingsType mappings;
};

};

#endif //TF_REMAPPER_CPP_TF_REMAPPER_H
