#ifndef TF_REMAPPER_CPP_TF_REMAPPER_H
#define TF_REMAPPER_CPP_TF_REMAPPER_H

#include <XmlRpc.h>
#include <tf2_msgs/TFMessage.h>

namespace tf_remapper_cpp {

class TfRemapper {
public:
    typedef std::map<std::string, std::string> MappingsType;

    TfRemapper();
    explicit TfRemapper(MappingsType mappings, bool reverse = false);
    explicit TfRemapper(const XmlRpc::XmlRpcValue& mappingsParam, bool reverse = false);
    virtual ~TfRemapper();

    void doRemapping(tf2_msgs::TFMessage& message) const;
    tf2_msgs::TFMessage doRemapping(const tf2_msgs::TFMessage& inMessage) const;

    const MappingsType& getMappings() const;

protected:
    MappingsType mappings;
};

};

#endif //TF_REMAPPER_CPP_TF_REMAPPER_H
