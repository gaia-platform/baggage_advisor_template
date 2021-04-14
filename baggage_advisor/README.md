# baggage_advisor

This package bridges ROS2 messages and a Gaia Platform declarative ruleset. It is a template for the Baggage Advisor scenario, an airport baggage scenario where scanned QR codes inform airport employees about how to load a cart with baggage.

You can "play along at home" by holding QR codes in front of your webcam. A video feed will appear in an [rqt_image_view](https://github.com/ros-visualization/rqt_image_view) window.

## Launching
Run `ros2 launch baggage_advisor baggage_advisor.launch.py` to try out the Baggage Advisor scenario. Even after closing the video feed window, make sure to press **Ctrl+C** in the terminal. If the video feed fails, make sure to select the correct video device in the `video_device` parameter in [config/camera.yaml](config/camera.yaml).

## Nodes

### [advisor](include/advisor.hpp)
A Composable Node that subscribes to ScanResult messages containing QR codes and inserts them into the Gaia Platform database to trigger rules in a Gaia Platform ruleset. The ruleset uses this node to publish ImageMarkers to draw an overlay on the QR code video feed.

* Input: Subscribes to barcode_msgs/ScanResult messages on the `barcodes` topic
* Output: Publishes image_draw_msgs/ImageMarker messages on the `image_markers` topic

## Gaia Platform components

### [Baggage Advisor DDL schema](src/baggage_advisor.ddl)
This schema describes Gaia Platform database tables that represent data about carts, baggages, baggage manifests, and other parts of the Baggage Advisor scenario.

### [Baggage Advisor ruleset](src/baggage_advisor.ruleset)
This ruleset reacts to inserts and updates in database tables to run the business logic of the Baggage Advisor scenario. It uses a pointer to a global singleton instance of the `advisor` node to construct ROS2 publishers and use ROS2 logging.
