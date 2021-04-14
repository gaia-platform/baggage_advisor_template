# Baggage Advisor: Gaia Platform + ROS2
Baggage Advisor is an application that manages baggage loading for airport flights using a Gaia Platform ruleset. A top-down camera scans QR codes on cart areas, carts, and baggages to populate database tables and trigger the associated rules. It simplifies the complex state machine of airport baggage loading into a concise set of rules with a schema representing the business objects.

The ROS2 packages were developed for the [Foxy distribution](https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html) on Ubuntu 20.04. Find the Gaia Platform components in the [baggage_advisor](baggage_advisor) package.

1. Make a ROS2 workspace and a `src` directory inside of it.

```
mkdir -p ros_ws/src
cd ros_ws/src
```

2. Clone this project into `src` and install its dependencies using `rosdep`.

```
git clone git@github.com:gaia-platform/baggage_advisor_template.git
cd ..
rosdep install --ignore-src --from-paths ./src
```

3. Bring ROS2 functions and variables into your shell environment.

```
source /opt/ros/foxy/setup.bash
```

4. Initialize the Gaia database server with a storage directory.

```
mkdir ./storage
gaia_db_server --data-dir ./storage &
```

5. Build the ROS2 packages and the Gaia DDL and ruleset. Bring the functions and variables from those local packages into your shell environment.

```
colcon build
source install/setup.bash
```

6. Launch the demo! Use the `video_device` launch argument to set your camera device.

```
ros2 launch baggage_advisor baggage_advisor.launch.py video_device:=/dev/video0
```

7. An `rqt_image_view` window will open. Make sure to press the "refresh" icon in the top-left and select the `webcam/image_marked` topic.

8. Even if you close the GUI/video windows that pop up, the nodes are still running. Remember to press Ctrl+C in the terminal to stop the nodes.
