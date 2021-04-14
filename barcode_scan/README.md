# barcode_scan

This package is for scanning 1D and 2D barcodes from images.

## Nodes

### [scanner](include/scanner.hpp)
A Composable Node that scans images from an `image_transport` stream and publishes barcodes any detected barcodes. The barcodes are published in a ScanResult message, which contains an array of Barcode messages and a header. Each ScanResult corresponds to one image so multiple barcodes with the same type and data can be distinguished.

* Input: Subscribes to [sensor_msgs/Image](https://github.com/ros2/common_interfaces/blob/master/sensor_msgs/msg/Image.msg) messages on the `image_raw` topic using [image_transport](https://github.com/ros-perception/image_common/tree/ros2/image_transport)
* Output: Publishes barcode_msgs/ScanResult messages on the `barcodes` topic
