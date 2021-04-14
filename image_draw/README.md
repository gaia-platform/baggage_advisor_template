# image_draw

This package is for drawing 2D markers over images. It subscribes to image_draw_msgs/ImageMarker messages, which have an optional duration field, allowing temporary or persistent markers to be overlaid on a video feed. The ImageMarker from image_draw_msgs is used instead of that from [visualization_msgs](https://github.com/ros2/common_interfaces/blob/master/visualization_msgs/msg/ImageMarker.msg) because of its richer features, including drawing text.

The drawing capabilities currently come from OpenCV's [Image Processing drawing functions](https://docs.opencv.org/master/d6/d6e/group__imgproc__draw.html).

## Nodes

### [image_draw](include/image_draw.hpp)
A Composable Node that takes images from an `image_transport` stream and subscribes to ImageMarker messages to draw those markers over each image. The drawn-over images are then republished on another `image_transport` stream.

* Input:
    * Subscribes to [sensor_msgs/Image](https://github.com/ros2/common_interfaces/blob/master/sensor_msgs/msg/Image.msg) messages on the `image_raw` topic using [image_transport](https://github.com/ros-perception/image_common/tree/ros2/image_transport)
    * Subscribes to image_draw_msgs/ImageMarker messages on the `image_markers` topic
* Output: Publishes [sensor_msgs/Image](https://github.com/ros2/common_interfaces/blob/master/sensor_msgs/msg/Image.msg) messages on the `image_marked` topic using [image_transport](https://github.com/ros-perception/image_common/tree/ros2/image_transport)
