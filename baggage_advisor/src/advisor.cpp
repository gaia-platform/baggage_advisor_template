/////////////////////////////////////////////
// Copyright (c) Gaia Platform LLC
// All rights reserved.
/////////////////////////////////////////////

#include "baggage_advisor/advisor.hpp"

#include "gaia_baggage_advisor.h"
#include "gaia/rules/rules.hpp"
#include "gaia/system.hpp"
#include "gaia/db/db.hpp"
#include "gaia/logger.hpp"

using namespace gaia::common;
using namespace gaia::baggage_advisor;

namespace baggage_advisor
{

advisor* advisor::s_node;

advisor::advisor(const rclcpp::NodeOptions& options)
:   rclcpp::Node("advisor", options),
    c_last_code_duration(2, 0)
{
    s_node = this;

    gaia::system::initialize();
    clear_storage();
    init_storage();

    rclcpp::on_shutdown([this]
    {
        advisor::shutdown_callback();
    });

    using std::placeholders::_1;

    m_last_code_time = this->get_clock()->now();

    m_marker_pub = this->create_publisher<image_draw_msgs::msg::ImageMarker>("image_markers",
        rclcpp::QoS(10));

    m_scan_result_sub = this->create_subscription<barcode_msgs::msg::ScanResult>(
        "barcodes", rclcpp::QoS(10),
        std::bind(&advisor::scan_result_callback, this, _1));
}

void advisor::send_status_message(const std::string& message, const e_color color)
{
    image_draw_msgs::msg::ImageMarker marker;
    marker.ns = "text";
    marker.id = 0;
    marker.type = image_draw_msgs::msg::ImageMarker::TEXT;
    marker.position.x = 50;
    marker.position.y = 50;
    marker.scale = 0.75;
    marker.thickness = 2;

    marker.text = message;

    switch(color)
    {
        case e_color::red:
            marker.outline_color = get_color(1.0, 0.0, 0.0);
            break;
        case e_color::yellow:
            marker.outline_color = get_color(1.0, 1.0, 0.0);
            break;
        case e_color::green:
            marker.outline_color = get_color(0.0, 1.0, 0.0);
            break;
        default:
            marker.outline_color = get_color(1.0, 1.0, 1.0);
    }

    builtin_interfaces::msg::Duration lifetime;
    lifetime.sec = 2;
    lifetime.nanosec = 0;
    marker.lifetime = lifetime;

    s_node->m_marker_pub->publish(marker);
}

void advisor::debug_print(const std::string& message)
{
    RCLCPP_INFO(s_node->get_logger(), "%s", message.c_str());
}

std_msgs::msg::ColorRGBA advisor::get_color(float r, float g, float b)
{
    std_msgs::msg::ColorRGBA color;
    color.r = r;
    color.g = g;
    color.b = b;
    color.a = 1.0;
    return color;
}

template <typename T_edc, typename... T_args>
T_edc advisor::insert_and_get(T_args... args)
{
    return T_edc::get(T_edc::insert_row(args...));
}

void advisor::init_storage()
{
    gaia::db::begin_transaction();

    terminal_t terminal = insert_and_get<terminal_t>("Terminal 17");

    cart_area_t cart_area = insert_and_get<cart_area_t>("Cart Area 7", "", "", "");
    terminal.cart_area__terminal_cart_area_list().insert(cart_area);

    gaia_id_t camera_id = camera_t::insert_row("Camera 7 IN", "IN", "");
    cart_area.camera__cart_area_camera_list().insert(camera_id);

    manifest_t manf_1 = insert_and_get<manifest_t>("MANIFEST1", "NEW", 1, 0);
    manifest_t manf_2 = insert_and_get<manifest_t>("MANIFEST2", "NEW", 2, 0);
    manifest_t manf_3 = insert_and_get<manifest_t>("MANIFEST3", "NEW", 3, 0);

    gaia_id_t manf_bg_0_id = manifest_baggage_t::insert_row("MB0", "BGA", 1, 0);
    manf_1.manifest_baggage__manifest_manifest_baggage_list().insert(manf_bg_0_id);
    gaia_id_t manf_bg_1_id = manifest_baggage_t::insert_row("MB1", "BGA", 1, 0);
    manf_2.manifest_baggage__manifest_manifest_baggage_list().insert(manf_bg_1_id);
    gaia_id_t manf_bg_2_id = manifest_baggage_t::insert_row("MB2", "BGB", 1, 0);
    manf_2.manifest_baggage__manifest_manifest_baggage_list().insert(manf_bg_2_id);
    gaia_id_t manf_bg_3_id = manifest_baggage_t::insert_row("MB3", "BGC", 3, 0);
    manf_3.manifest_baggage__manifest_manifest_baggage_list().insert(manf_bg_3_id);

    gaia::db::commit_transaction();
}

void advisor::scan_result_callback(
    const barcode_msgs::msg::ScanResult::SharedPtr scan_result_msg)
{
    gaia::db::begin_transaction();
    // This insertion strategy is not ideal because it assumes only one active camera.
    camera_writer cam_writer = camera_t::get_first().writer();

    for(const auto& barcode : scan_result_msg->barcodes)
    {
        rclcpp::Time time_now = this->get_clock()->now();

        if((time_now - m_last_code_time) > c_last_code_duration
            || m_last_code != barcode.data)
        {
            m_last_code = barcode.data;
            cam_writer.camera_data_code = barcode.data;
            m_last_code_time = time_now;
        }
        else
        {
            m_last_code_time = time_now;
        }
    }

    cam_writer.update_row();
    gaia::db::commit_transaction();
}

void advisor::shutdown_callback()
{
    gaia::system::shutdown();
}

} // namespace baggage_advisor

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(baggage_advisor::advisor)
