/////////////////////////////////////////////
// Copyright (c) 2021 Gaia Platform LLC
// This file is made available under the MIT license.
// Do not use it if you have not received an associated LICENSE file.
/////////////////////////////////////////////

#pragma once

#include <string>

#include "rclcpp/rclcpp.hpp"

#include "barcode_msgs/msg/scan_result.hpp"
#include "image_draw_msgs/msg/image_marker.hpp"

namespace baggage_advisor
{

enum e_color
{
    red,
    yellow,
    green
};

class advisor : public rclcpp::Node
{
public:
    advisor(const rclcpp::NodeOptions& options);

    static void send_status_message(const std::string& message, const e_color color);
    static void debug_print(const std::string& message);

private:
    static std_msgs::msg::ColorRGBA get_color(float r, float g, float b);

    template <typename T_edc, typename... T_args>
    T_edc insert_and_get(T_args... args);

    void init_storage();
    void clear_storage();

    // This callback recieves a ScanResult full of barcodes
    // and inserts the barcodes into a field in the camera table in the database.
    void scan_result_callback(
        const barcode_msgs::msg::ScanResult::SharedPtr scan_result_msg);

    void shutdown_callback();

    static advisor* s_node;

    rclcpp::Publisher<image_draw_msgs::msg::ImageMarker>::SharedPtr m_marker_pub;
    rclcpp::Subscription<barcode_msgs::msg::ScanResult>::SharedPtr m_scan_result_sub;

    const rclcpp::Duration c_last_code_duration;
    std::string m_last_code = "";
    rclcpp::Time m_last_code_time;
};

} // namespace baggage_advisor
