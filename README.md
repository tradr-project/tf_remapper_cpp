# TF remapper (more efficient and versatile C++ version)

[![License](https://img.shields.io/badge/License-BSD%203--Clause-blue.svg)](https://opensource.org/licenses/BSD-3-Clause) [![codecov](https://codecov.io/gh/tradr-project/tf_remapper_cpp/branch/master/graph/badge.svg)](https://codecov.io/gh/tradr-project/tf_remapper_cpp)

This package is an alternative to official ROS node [tf/tf_remap](https://github.com/ros/geometry/blob/melodic-devel/tf/scripts/tf_remap) with the following advantages:

- Works natively with **TF2**
    - that could probably save a few data conversions
- **Performance**: handles tens of TF subscribers with thousands of TF messages/sec with ease
    - the official node fully utilizes 2 i7 CPU cores if you have 10 subscribers and publish 2000 TF messages per second
    - this package needs about 0.4 CPU cores for the same
- Can also remap **`/tf_static`**
    - the official package can not do that correctly
- Can also **remove frames** (if you pass empty string to `new`)
    - this can come handy if you e.g. want to recompute your map in bagfiles using a brand new algorithm
- Can **work in both ways**
    - not only taking frames published on `/tf_old` and remapping and publishing them to `/tf`, but it can also watch `/tf`, perform a reverse remapping, and publish new TF messages to `/tf_old`
    - this can come handy if you e.g. want to run your robot in a restricted world where it does not know about the other robots (it has its own `/map` frame), and then you have a multirobot coordination algorithm which wants to see the maps of each robot as `/ugv1/map`, `/ugv2/map` and so on, and also wants to publish a `/global_map` frame available to all robots.
- Backwards **compatible with [tf/tf_remap](https://github.com/ros/geometry/blob/melodic-devel/tf/scripts/tf_remap)**
    - to use this remapper, just change package name from `tf` to `tf_remapper_cpp` in your launch files
    - all the new functionality should not endanger the basic usage, since the package is accompanied by an exhaustive test suite
- [API Documentation](https://tradr-project.github.io/tf_remapper_cpp/md_README.html) and [usage examples](#example-usage-in-launch-files)
    - the remapping logic is available in dynamic library `libtf_remapper_cpp.so`, class [`TfRemapper`](https://tradr-project.github.io/tf_remapper_cpp/classtf__remapper__cpp_1_1TfRemapper.html)

## Nodes

### tf\_remapper\_cpp/tf\_remap

The node that performs the TF remapping.

#### Private parameters

- **`mappings`** (`array of dicts`): The rules for TF frame name remapping, e.g. `[{"old": "b", "new": "d"}]`. Each dict presents one remapping rule. Each dict contains keys `old` and `new`. If either of these keys is missing or its value is empty string, it means the TF frame should be deleted.
    - Until [ros_comm#1498](https://github.com/ros/ros_comm/issues/1498) is fixed, you can not pass this parameter from `rosrun` commandline - either set it in a launch file, or use `rosparam set /tf_remapper/mappings '[{"old": "b", "new": "d"}]'` before launching this node.
- **`static_tf`** (`bool`): Whether the remapper acts on static TFs or not. If not set, it is autodetected from `new_tf_topic_name` parameter.
    - Static TFs need special handling, so be sure to have this parameter set correctly, otherwise it can cause performance issues (when used with non-static TFs) or incorrect operation (if not used with static TFs).
    - Autodetection checks if the `new_tf_topic_name` is `tf_static` or `/tf_static`, and if it is, then `static_tf` is set to `True`, otherwise it is set to `False`.
- **`old_tf_topic_name`** (`string`, default `'/tf_old'`): The topic on which old TFs are subscribed.
- **`new_tf_topic_name`** (`string`, default `'/tf'`): The topic on which remapped TFs are published.
- **`is_bidirectional`** (`bool`, default `False`): If `True`, the remapper will also allow passing TFs published on the "remapped end" to the "old end" via a reverse mapping. 
    - Pay special attention if you use any kind of multimaster solution or a custom topic transport. This node needs CallerIDs to be unchanged. If this contract is broken, the node will probably enter an infinite loop reacting to its own published messages.

#### Subscribed topics

- **`/tf_old`** (or any other topic set in `old_tf_topic_name`; type `tf2_ros/TFMessage`): The original TF messages that are to be remapped.
- **`/tf`** (or any other topic set in `new_tf_topic_name`; only if `is_bidirectional == True`; type `tf2_ros/TFMessage`): The TF messages with remapped frames. If some node publises to this topic and this remapper is running in bidirectional mode, it sends the newly published transforms back to `/tf_old`.

#### Published topics

- **`/tf`** (or any other topic set in `new_tf_topic_name`; type `tf2_ros/
TFMessage`): The TF messages with remapped frames.
- **`/tf_old`** (or any other topic set in `old_tf_topic_name`; only if `is_bidirectional == True`; type `tf2_ros/TFMessage`): The original TF messages. If some node publises to `/tf` and this remapper is running in bidirectional mode, it sends the newly published transforms back to this topic.

## Example usage in launch files

### simple.launch

    <launch>
        <group>
            <remap from="tf" to="tf_old" />
            <!-- The tf(1) static_transform_publisher does not use /tf_static, but periodically publises to /tf -->
            <node name="broadcaster_ab" pkg="tf" type="static_transform_publisher" args="1 2 3 4 5 6 a b 10"/>
            <!-- Usually, there would be e.g. a rosbag play instead of the static tf publisher. -->
        </group>

        <node name="remapper" pkg="tf_remapper_cpp" type="tf_remap">
            <rosparam param="mappings">[{old: b, new: c}]</rosparam>
        </node>

        <!-- This node will see transform a->c -->
        <node name="my_node" pkg="my_pkg" type="node_type" />
    </launch>
    
### static_tf.launch

    <launch>
        <group>
            <remap from="tf_static" to="tf_static_old" />
            <!-- The tf2 static_transform_publisher uses /tf_static -->
            <node name="broadcaster_ab" pkg="tf2_ros" type="static_transform_publisher" args="1 2 3 4 5 6 a b"/>
            <!-- Usually, there would be e.g. a rosbag play instead of the static tf publisher. -->
        </group>

        <node name="remapper" pkg="tf_remapper_cpp" type="tf_remap">
            <rosparam param="mappings">[{old: b, new: c}]</rosparam>
            <!--<param name="static_tf" value="true" />  - this is not needed, autodetection works in this case -->
        </node>

        <!-- This node will see static transform a->c -->
        <node name="my_node" pkg="my_pkg" type="node_type" />
    </launch>
    
### bidirectional.launch

    <launch>
        <group>
            <remap from="tf" to="tf_old" />
            <!-- The tf(1) static_transform_publisher does not use /tf_static, but periodically publises to /tf -->
            <node name="broadcaster_ab" pkg="tf" type="static_transform_publisher" args="1 2 3 4 5 6 a b 10"/>
            <!-- Usually, there would be e.g. a rosbag play instead of the static tf publisher. -->
            
            <!-- This node will see transforms a->b and d->e -->
            <node name="my_node2" pkg="my_pkg" type="node_type" />
        </group>

        <node name="remapper" pkg="tf_remapper_cpp" type="tf_remap">
            <rosparam param="mappings">[{old: b, new: c}, {old: e, new: f}]</rosparam>
            <param name="is_bidirectional" value="true" />
        </node>

        <!-- This node will see transforms a->c and d->f -->
        <node name="my_node" pkg="my_pkg" type="node_type" />
        
        <node name="broadcaster_df" pkg="tf" type="static_transform_publisher" args="1 2 3 4 5 6 d f 10"/>
    </launch>
