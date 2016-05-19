# mocap_optitrack 

forked from [ros-drivers/mocap_optitrack](https://github.com/ros-drivers/mocap_optitrack)

ROS nodes for working with the NaturalPoint Optitrack motion capture setup.

This modified version will stop publishing if the object is lost for some millseconds in the Motive.

An extra node subscribes geometry_msgs/PoseStamped and publishes a downsampled aligned-with-launch-FLU-frame odometry. FLU frame is the same as [ros standard frame](http://www.ros.org/reps/rep-0103.html#axis-orientation)
