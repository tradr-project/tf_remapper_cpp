#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <ros/ros.h>

#include <gtest/gtest.h>

ros::NodeHandle* nh;
ros::NodeHandle* pnh;

TEST(CheckTfExists, checkTransform)
{
    tf2_ros::Buffer buffer;
    tf2_ros::TransformListener listener(buffer);

    ros::Duration(1.0).sleep();

    const std::string sourceTf = pnh->param<std::string>("source_tf", "a");
    const std::string targetTf = pnh->param<std::string>("target_tf", "b");
    const bool shouldSucceed = pnh->param<bool>("should_succeed", true);

    std::string err;
    bool lookupSucceeded = buffer.canTransform(targetTf, sourceTf, ros::Time::now(),
            ros::Duration((shouldSucceed ? 1.0 : 0.1)), &err);
    EXPECT_EQ(shouldSucceed, lookupSucceeded);
    if (lookupSucceeded != shouldSucceed)
        EXPECT_EQ("", err);  // To get the TF error printed out
}

int main(int argc, char ** argv)
{
    testing::InitGoogleTest(&argc, argv);

    ros::init(argc, argv, "check_tf_exists");

    nh = new ros::NodeHandle;
    pnh = new ros::NodeHandle("~");

    int ret = RUN_ALL_TESTS();

    delete nh;
    delete pnh;

    return ret;
};