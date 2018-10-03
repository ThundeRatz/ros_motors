# ros_motors
ROS package for motor controll.

Subscribes to `motors` topic, collecting data from a custom `motor` message (see [trekking_msgs](https://github.com/ThundeRatz/trekking_msgs)) that should be included in the same workspace of this package, and sends it via USB.
