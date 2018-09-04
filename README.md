#TF remapper (more efficient and versatile C++ version)

This package is an alternative to official ROS node [tf/tf_remap](https://github.com/ros/geometry/blob/melodic-devel/tf/scripts/tf_remap) with the following advantages:

- Works natively with TF2
	- that could probably save a few data conversions
- Handles tens of TF subscribers with thousands of TF messages/sec with ease
	- the official node fully utilizes 2 i7 CPU cores if you have 10 subscribers and publish 2000 TF messages per second
	- this package needs about 0.4 CPU cores for the same
- Can also remap `/tf_static`
	- the official package can not do that correctly
- Can also remove frames (if you pass empty string to `new`)
	- this can come handy if you e.g. want to recompute your map in bagfiles using a brand new algorithm
- Can work in both ways
	- not only taking frames published on `/tf_old` and remapping and publishing them to `/tf`, but it can also watch `/tf`, perform a reverse remapping, and publish new TF messages to `/tf_old`
	- this can come handy if you e.g. want to run your robot in a restricted world where it does not know about the other robots (it has its own `/map` frame), and then you have a multirobot coordination algorithm which wants to see the maps of each robot as `/ugv1/map`, `/ugv2/map` and so on, and also wants to publish a `/global_map` frame available to all robots.
- Backwards compatible
	- all this new functionality should not endanger the basic usage, since the package is accompanied by an exhaustive test suite

## Nodes

### tf\_remapper\_cpp/tf\_remapper\_cpp\_node

#### Parameters

#### Subscribed topics

#### Published topics

## Example usage in launch files