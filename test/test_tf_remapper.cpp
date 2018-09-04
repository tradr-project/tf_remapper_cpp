#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wunknown-pragmas"
#pragma clang diagnostic ignored "cert-err58-cpp"
#pragma clang diagnostic ignored "cppcoreguidelines-avoid-goto"

#include <gtest/gtest.h>
#include <tf_remapper_cpp/tf_remapper.h>
#include <ros/exceptions.h>

using namespace tf_remapper_cpp;

TEST(tf_remapper_cpp, testDefaultConstructor)
{
    ASSERT_NO_THROW(TfRemapper remap = TfRemapper());
    TfRemapper* remapPointer;
    ASSERT_NO_THROW(remapPointer = new TfRemapper());
    delete remapPointer;
}

TEST(tf_remapper_cpp, testMappingsConstructor)
{
    std::map<std::string, std::string> mappings;
    mappings["a"] = "b";
    mappings["c"] = "d";
    mappings["e"] = "";
    TfRemapper remap;
    EXPECT_NO_THROW(remap = TfRemapper(mappings));
    EXPECT_EQ(3, remap.getMappings().size());
    EXPECT_EQ(mappings, remap.getMappings());

    std::map<std::string, std::string> reverseMappings;
    reverseMappings["b"] = "a";
    reverseMappings["d"] = "c";
    TfRemapper reverseRemap;
    EXPECT_NO_THROW(reverseRemap = TfRemapper(mappings, true));
    EXPECT_EQ(2, reverseRemap.getMappings().size());
    EXPECT_EQ(reverseMappings, reverseRemap.getMappings());
}

TEST(tf_remapper_cpp, testAllowsEmptyMappings)
{
    std::map<std::string, std::string> mappings;
    TfRemapper remap;
    EXPECT_NO_THROW(remap = TfRemapper(mappings));
    EXPECT_EQ(0, remap.getMappings().size());
}

TEST(tf_remapper_cpp, testAllMappingsAreCopied)
{
    std::map<std::string, std::string> mappings;
    mappings["a"] = "ab";
    mappings["c"] = "cb";
    mappings["d"] = "db";
    mappings["e"] = "eb";
    const TfRemapper remap = TfRemapper(mappings);
    EXPECT_EQ(mappings, remap.getMappings());
}

TEST(tf_remapper_cpp, testThrowsOnEmptyOldAndNew)
{
    std::map<std::string, std::string> mappings;
    mappings[""] = "";
    TfRemapper remap;
    EXPECT_THROW(remap = TfRemapper(mappings), ros::InvalidParameterException);
    EXPECT_EQ(0, remap.getMappings().size());
}

TEST(tf_remapper_cpp, testAllowsEmptyNew)
{
    std::map<std::string, std::string> mappings;
    mappings["a"] = "";
    TfRemapper remap;
    ASSERT_NO_THROW(remap = TfRemapper(mappings));
    EXPECT_EQ(1, remap.getMappings().size());
}

TEST(tf_remapper_cpp, testRemapping)
{
    std::map<std::string, std::string> mappings;
    mappings["a"] = "b";
    mappings["d"] = "e";
    TfRemapper remap;
    ASSERT_NO_THROW(remap = TfRemapper(mappings));
    ASSERT_EQ(2, remap.getMappings().size());
    
    tf2_msgs::TFMessage message;

    // should remap to b->c
    geometry_msgs::TransformStamped tf;
    tf.header.frame_id = "a";
    tf.child_frame_id = "c";
    tf.header.seq = 19;
    tf.header.stamp.sec = 20;
    tf.header.stamp.nsec = 21;
    tf.transform.translation.x = 1.0;
    tf.transform.translation.y = 2.0;
    tf.transform.translation.z = 3.0;
    tf.transform.rotation.x = 4.0;
    tf.transform.rotation.y = 5.0;
    tf.transform.rotation.z = 6.0;
    tf.transform.rotation.w = 7.0;
    message.transforms.push_back(tf);

    // should remap to b->e
    geometry_msgs::TransformStamped tf2;
    tf2.header.frame_id = "b";
    tf2.child_frame_id = "d";
    message.transforms.push_back(tf2);

    // should not remap
    geometry_msgs::TransformStamped tf3;
    tf3.header.frame_id = "b";
    tf3.child_frame_id = "c";
    message.transforms.push_back(tf3);

    // should allow remapping to the same name
    geometry_msgs::TransformStamped tf4;
    tf4.header.frame_id = "c";
    tf4.child_frame_id = "c";
    message.transforms.push_back(tf4);

    tf2_msgs::TFMessage messageCopy = remap.doRemapping(const_cast<const tf2_msgs::TFMessage&>(message));
    remap.doRemapping(message);

    ASSERT_EQ(4, message.transforms.size());

    EXPECT_EQ("b", message.transforms[0].header.frame_id);
    EXPECT_EQ("c", message.transforms[0].child_frame_id);
    EXPECT_EQ("b", message.transforms[1].header.frame_id);
    EXPECT_EQ("e", message.transforms[1].child_frame_id);
    EXPECT_EQ("b", message.transforms[2].header.frame_id);
    EXPECT_EQ("c", message.transforms[2].child_frame_id);
    EXPECT_EQ("c", message.transforms[3].header.frame_id);
    EXPECT_EQ("c", message.transforms[3].child_frame_id);

    EXPECT_EQ(19, message.transforms[0].header.seq);
    EXPECT_EQ(20, message.transforms[0].header.stamp.sec);
    EXPECT_EQ(21, message.transforms[0].header.stamp.nsec);
    EXPECT_EQ(1.0, message.transforms[0].transform.translation.x);
    EXPECT_EQ(2.0, message.transforms[0].transform.translation.y);
    EXPECT_EQ(3.0, message.transforms[0].transform.translation.z);
    EXPECT_EQ(4.0, message.transforms[0].transform.rotation.x);
    EXPECT_EQ(5.0, message.transforms[0].transform.rotation.y);
    EXPECT_EQ(6.0, message.transforms[0].transform.rotation.z);
    EXPECT_EQ(7.0, message.transforms[0].transform.rotation.w);

    ASSERT_EQ(4, messageCopy.transforms.size());

    EXPECT_EQ("b", messageCopy.transforms[0].header.frame_id);
    EXPECT_EQ("c", messageCopy.transforms[0].child_frame_id);
    EXPECT_EQ("b", messageCopy.transforms[1].header.frame_id);
    EXPECT_EQ("e", messageCopy.transforms[1].child_frame_id);
    EXPECT_EQ("b", messageCopy.transforms[2].header.frame_id);
    EXPECT_EQ("c", messageCopy.transforms[2].child_frame_id);
    EXPECT_EQ("c", messageCopy.transforms[3].header.frame_id);
    EXPECT_EQ("c", messageCopy.transforms[3].child_frame_id);

    EXPECT_EQ(19, messageCopy.transforms[0].header.seq);
    EXPECT_EQ(20, messageCopy.transforms[0].header.stamp.sec);
    EXPECT_EQ(21, messageCopy.transforms[0].header.stamp.nsec);
    EXPECT_EQ(1.0, messageCopy.transforms[0].transform.translation.x);
    EXPECT_EQ(2.0, messageCopy.transforms[0].transform.translation.y);
    EXPECT_EQ(3.0, messageCopy.transforms[0].transform.translation.z);
    EXPECT_EQ(4.0, messageCopy.transforms[0].transform.rotation.x);
    EXPECT_EQ(5.0, messageCopy.transforms[0].transform.rotation.y);
    EXPECT_EQ(6.0, messageCopy.transforms[0].transform.rotation.z);
    EXPECT_EQ(7.0, messageCopy.transforms[0].transform.rotation.w);
}

TEST(tf_remapper_cpp, testDelete)
{
    std::map<std::string, std::string> mappings;
    mappings["a"] = "";
    mappings["d"] = "e";
    TfRemapper remap;
    ASSERT_NO_THROW(remap = TfRemapper(mappings));
    ASSERT_EQ(2, remap.getMappings().size());

    tf2_msgs::TFMessage message;

    // should be deleted
    geometry_msgs::TransformStamped tf;
    tf.header.frame_id = "a";
    tf.child_frame_id = "c";
    message.transforms.push_back(tf);

    // should remap to b->e
    geometry_msgs::TransformStamped tf2;
    tf2.header.frame_id = "b";
    tf2.child_frame_id = "d";
    message.transforms.push_back(tf2);

    // should not remap
    geometry_msgs::TransformStamped tf3;
    tf3.header.frame_id = "b";
    tf3.child_frame_id = "c";
    message.transforms.push_back(tf3);

    // should be deleted
    geometry_msgs::TransformStamped tf4;
    tf4.header.frame_id = "f";
    tf4.child_frame_id = "a";
    message.transforms.push_back(tf4);

    remap.doRemapping(message);

    ASSERT_EQ(2, message.transforms.size());

    EXPECT_EQ("b", message.transforms[0].header.frame_id);
    EXPECT_EQ("e", message.transforms[0].child_frame_id);
    EXPECT_EQ("b", message.transforms[1].header.frame_id);
    EXPECT_EQ("c", message.transforms[1].child_frame_id);
}

TEST(tf_remapper_cpp, testXmlRpcValueConstructor)
{
    int offset = 0;
    XmlRpc::XmlRpcValue xmlRpcMappings(
         "<value>"
         "  <array>"
         "      <data>"
         "          <value>"
         "              <struct>"
         "                  <member>"
         "                      <name>new</name>"
         "                      <value>ab</value>"
         "                  </member>"
         "                  <member>"
         "                      <name>old</name>"
         "                      <value>a</value>"
         "                  </member>"
         "              </struct>"
         "          </value>"
         "          <value>"
         "              <struct>"
         "                  <member>"
         "                      <name>new</name>"
         "                      <value>cb</value>"
         "                  </member>"
         "                  <member>"
         "                      <name>old</name>"
         "                      <value>c</value>"
         "                  </member>"
         "              </struct>"
         "          </value>"
         "          <value>"
         "              <struct>"
         "                  <member>"
         "                      <name>new</name>"
         "                      <value>db</value>"
         "                  </member>"
         "                  <member>"
         "                      <name>old</name>"
         "                      <value>d</value>"
         "                  </member>"
         "              </struct>"
         "          </value>"
         "          <value>"
         "              <struct>"
         "                  <member>"
         "                      <name>new</name>"
         "                      <value>eb</value>"
         "                  </member>"
         "                  <member>"
         "                      <name>old</name>"
         "                      <value>e</value>"
         "                  </member>"
         "              </struct>"
         "          </value>"
         "          <value>"
         "              <struct>"
         "                  <member>"
         "                      <name>new</name>"
         "                      <value>f</value>"
         "                  </member>"
         "                  <member>"
         "                      <name>old</name>"
         "                      <value>f</value>"
         "                  </member>"
         "              </struct>"
         "          </value>"
         "          <value>"
         "              <struct>"
         "                  <member>"
         "                      <name>new</name>"
         "                      <value></value>"
         "                  </member>"
         "                  <member>"
         "                      <name>old</name>"
         "                      <value>g</value>"
         "                  </member>"
         "              </struct>"
         "          </value>"
         "          <value>"
         "              <struct>"
         "                  <member>"
         "                      <name>new</name>"
         "                      <value>h</value>"
         "                  </member>"
         "                  <member>"
         "                      <name>old</name>"
         "                      <value></value>"
         "                  </member>"
         "              </struct>"
         "          </value>"
         "      </data>"
         "  </array>"
         "</value>", &offset);
    std::map<std::string, std::string> mappings;
    mappings["a"] = "ab";
    mappings["c"] = "cb";
    mappings["d"] = "db";
    mappings["e"] = "eb";
    mappings["f"] = "f";
    mappings["g"] = "";
    const TfRemapper remap = TfRemapper(xmlRpcMappings);
    EXPECT_EQ(mappings, remap.getMappings());

    std::map<std::string, std::string> reverseMappings;
    reverseMappings["ab"] = "a";
    reverseMappings["cb"] = "c";
    reverseMappings["db"] = "d";
    reverseMappings["eb"] = "e";
    reverseMappings["f"] = "f";
    reverseMappings["h"] = "";
    const TfRemapper reverseRemap = TfRemapper(xmlRpcMappings, true);
    EXPECT_EQ(reverseMappings, reverseRemap.getMappings());
}

TEST(tf_remapper_cpp, testXmlRpcValueConstructorThrowsOnInvalidData)
{
    {
        XmlRpc::XmlRpcValue xmlRpcMappings(false);
        EXPECT_THROW(TfRemapper remap = TfRemapper(xmlRpcMappings), ros::InvalidParameterException);
    }
    {
        XmlRpc::XmlRpcValue xmlRpcMappings(1);
        EXPECT_THROW(TfRemapper remap = TfRemapper(xmlRpcMappings), ros::InvalidParameterException);
    }
    {
        XmlRpc::XmlRpcValue xmlRpcMappings(1.0);
        EXPECT_THROW(TfRemapper remap = TfRemapper(xmlRpcMappings), ros::InvalidParameterException);
    }
    {
        struct tm time = { 0, 0, 0, 0, 0, 0, 0, 0, 0 };
        XmlRpc::XmlRpcValue xmlRpcMappings(&time);
        EXPECT_THROW(TfRemapper remap = TfRemapper(xmlRpcMappings), ros::InvalidParameterException);
    }
    {
        XmlRpc::XmlRpcValue xmlRpcMappings("throw");
        EXPECT_THROW(TfRemapper remap = TfRemapper(xmlRpcMappings), ros::InvalidParameterException);
    }
    {
        std::string data = "asd";
        XmlRpc::XmlRpcValue xmlRpcMappings(static_cast<void*>(&data), data.size());
        EXPECT_THROW(TfRemapper remap = TfRemapper(xmlRpcMappings), ros::InvalidParameterException);
    }
    {
        XmlRpc::XmlRpcValue xmlRpcMappings;
        xmlRpcMappings["a"] = "b";
        EXPECT_THROW(TfRemapper remap = TfRemapper(xmlRpcMappings), ros::InvalidParameterException);
    }
    {
        XmlRpc::XmlRpcValue xmlRpcMappings;
        xmlRpcMappings[0] = "b";
        EXPECT_THROW(TfRemapper remap = TfRemapper(xmlRpcMappings), ros::InvalidParameterException);
    }
    {
        XmlRpc::XmlRpcValue xmlRpcMappings;
        xmlRpcMappings[0]["old"] = "";
        EXPECT_THROW(TfRemapper remap = TfRemapper(xmlRpcMappings), ros::InvalidParameterException);
    }
    {
        XmlRpc::XmlRpcValue xmlRpcMappings;
        xmlRpcMappings[0]["old"] = "";
        xmlRpcMappings[0]["new"] = "";
        EXPECT_THROW(TfRemapper remap = TfRemapper(xmlRpcMappings), ros::InvalidParameterException);
    }
    {
        XmlRpc::XmlRpcValue xmlRpcMappings;
        xmlRpcMappings[0]["old"] = false;
        EXPECT_THROW(TfRemapper remap = TfRemapper(xmlRpcMappings), ros::InvalidParameterException);
    }
    {
        XmlRpc::XmlRpcValue xmlRpcMappings;
        xmlRpcMappings[0]["old"] = "a";
        xmlRpcMappings[0]["new"] = "b";
        xmlRpcMappings[0]["foo"] = "c";
        EXPECT_THROW(TfRemapper remap = TfRemapper(xmlRpcMappings), ros::InvalidParameterException);
    }
    {
        XmlRpc::XmlRpcValue xmlRpcMappings;
        xmlRpcMappings[0]["old"] = "a";
        xmlRpcMappings[0]["new"] = "b";
        xmlRpcMappings[1]["old"] = "";
        EXPECT_THROW(TfRemapper remap = TfRemapper(xmlRpcMappings), ros::InvalidParameterException);
    }
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
#pragma clang diagnostic pop