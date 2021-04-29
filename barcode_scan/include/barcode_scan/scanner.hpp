/////////////////////////////////////////////
// Copyright (c) 2021 Gaia Platform LLC
// This file is made available under the MIT license.
// Do not use it if you have not received an associated LICENSE file.
/////////////////////////////////////////////

#pragma once

#include "rclcpp/rclcpp.hpp"
#include "image_transport/image_transport.hpp"

#include "barcode_msgs/msg/scan_result.hpp"

#include "zbar.h"

namespace barcode_scan
{

class scanner : public rclcpp::Node
{
public:
    scanner(const rclcpp::NodeOptions& options);

private:
    // This callback recieves an image and scans it with ZBar to publish the detected barcodes.
    void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr& image_msg);

    rclcpp::Publisher<barcode_msgs::msg::ScanResult>::SharedPtr m_scan_result_pub;
    image_transport::Subscriber m_image_sub;

    zbar::ImageScanner m_scanner;
};

} // namespace barcode_scan
