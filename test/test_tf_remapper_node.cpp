#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <ros/ros.h>

#include <gtest/gtest.h>

TEST(TransformBroadcaster, remap)
{
    tf2_ros::Buffer buffer;
    tf2_ros::TransformListener listener(buffer);

    ros::Duration(1.0).sleep();

    std::string err;
    bool lookupSucceeded = buffer.canTransform("a", "c", ros::Time::now(), ros::Duration(1.0), &err);
    EXPECT_TRUE(lookupSucceeded);
    if (!lookupSucceeded)
        EXPECT_EQ("", err);  // To get the TF error printed out

    err.clear();
    lookupSucceeded = buffer.canTransform("a", "b", ros::Time::now(), ros::Duration(0.1), &err);
    EXPECT_FALSE(lookupSucceeded);
    if (lookupSucceeded)
        EXPECT_EQ("", err);  // To get the TF output printed out

    err.clear();
    lookupSucceeded = buffer.canTransform("b", "c", ros::Time::now(), ros::Duration(0.1), &err);
    EXPECT_FALSE(lookupSucceeded);
    if (lookupSucceeded)
        EXPECT_EQ("", err);  // To get the TF output printed out

    err.clear();
    lookupSucceeded = buffer.canTransform("d", "e", ros::Time::now(), ros::Duration(0.1), &err);
    EXPECT_FALSE(lookupSucceeded);
    if (lookupSucceeded)
        EXPECT_EQ("", err);  // To get the TF output printed out

    err.clear();
    lookupSucceeded = buffer.canTransform("f", "g", ros::Time::now(), ros::Duration(0.1), &err);
    EXPECT_FALSE(lookupSucceeded);
    if (lookupSucceeded)
        EXPECT_EQ("", err);  // To get the TF output printed out

    err.clear();
    lookupSucceeded = buffer.canTransform("h", "i", ros::Time::now(), ros::Duration(0.1), &err);
    EXPECT_FALSE(lookupSucceeded);
    if (lookupSucceeded)
        EXPECT_EQ("", err);  // To get the TF output printed out

    err.clear();
    lookupSucceeded = buffer.canTransform("i", "j", ros::Time::now(), ros::Duration(0.1), &err);
    EXPECT_FALSE(lookupSucceeded);
    if (lookupSucceeded)
        EXPECT_EQ("", err);  // To get the TF output printed out

    err.clear();
    lookupSucceeded = buffer.canTransform("h", "j", ros::Time::now(), ros::Duration(0.1), &err);
    EXPECT_FALSE(lookupSucceeded);
    if (lookupSucceeded)
        EXPECT_EQ("", err);  // To get the TF output printed out

    err.clear();
    lookupSucceeded = buffer.canTransform("y", "z", ros::Time::now(), ros::Duration(1.0), &err);
    EXPECT_TRUE(lookupSucceeded);
    if (!lookupSucceeded)
        EXPECT_EQ("", err);  // To get the TF error printed out
}

TEST(TransformBroadcaster, transform)
{
    tf2_ros::Buffer buffer;
    tf2_ros::TransformListener listener(buffer);

    ros::Duration(1.0).sleep();

    bool lookupSucceeded = buffer.canTransform("a", "c", ros::Time::now(), ros::Duration(1.0));
    ASSERT_TRUE(lookupSucceeded);

    geometry_msgs::TransformStamped transform = buffer.lookupTransform("a", "c", ros::Time::now());
    EXPECT_NEAR(1, transform.transform.translation.x, 1e-6);
    EXPECT_NEAR(2, transform.transform.translation.y, 1e-6);
    EXPECT_NEAR(3, transform.transform.translation.z, 1e-6);
    EXPECT_NEAR(0.585792, transform.transform.rotation.x, 1e-6);
    EXPECT_NEAR(0.143757, transform.transform.rotation.y, 1e-6);
    EXPECT_NEAR(0.756334, transform.transform.rotation.z, 1e-6);
    EXPECT_NEAR(-0.253261, transform.transform.rotation.w, 1e-6);
}

int main(int argc, char ** argv)
{
    testing::InitGoogleTest(&argc, argv);

    ros::init(argc, argv, "test_tf_remapper_node");
    ros::NodeHandle nh;

    int ret = RUN_ALL_TESTS();
    return ret;
};