/////////////////////////////////////////////
// Copyright (c) Gaia Platform LLC
// All rights reserved.
/////////////////////////////////////////////

#include "image_draw/image_draw.hpp"

#include <functional>
#include <limits>
#include <vector>

#include "opencv2/imgproc.hpp"
#include "cv_bridge/cv_bridge.h"

namespace image_draw
{

image_draw::image_draw(const rclcpp::NodeOptions& options)
:   Node("image_draw", options)
{
    // Cannot use Best-Effort QoS reliability because third-party subscribers such as
    // rqt_image_view tend to use a Reliable QoS reliability setting.
    m_image_pub = image_transport::create_publisher(this, "image_marked",
        rmw_qos_profile_default);

    using std::placeholders::_1;

    // Using rmw_qos_profile_sensor_data decreases latency but often drops frames.
    // rmw_qos_profile_default should be used if the frame dropping is excessive.
    m_image_sub = image_transport::create_subscription(this, "image_raw",
        std::bind(&image_draw::image_callback, this, _1),
        "raw", rmw_qos_profile_default);

    m_image_marker_sub = this->create_subscription<marker_t>(
        "image_markers", rclcpp::QoS(rclcpp::KeepAll()),
        std::bind(&image_draw::image_marker_callback, this, _1));
}

std::string image_draw::get_marker_id(const image_draw_msgs::msg::ImageMarker& msg)
{
    return msg.ns + '/' + std::to_string(msg.id);
}

cv::Scalar image_draw::get_cv_color(const std_msgs::msg::ColorRGBA& color_msg)
{
    return CV_RGB(255 * color_msg.b, 255 * color_msg.g, 255 * color_msg.r);
}

cv::Point image_draw::get_cv_point(const geometry_msgs::msg::Point& point_msg)
{
    return cv::Point(point_msg.x, point_msg.y);
}

void image_draw::draw_circle(const cv::Mat& image, const marker_t& marker)
{
    auto center = get_cv_point(marker.position);
    auto color = get_cv_color(marker.outline_color);
    
    if(marker.filled)
    {
        auto fill_color = get_cv_color(marker.fill_color);
        cv::circle(image, center, (marker.scale / 2), fill_color, cv::FILLED);
    }

    cv::circle(image, center, (marker.scale / 2), color, marker.thickness);
}

void image_draw::draw_line_strip(const cv::Mat& image, const marker_t& marker)
{
    std::vector<cv::Point> cv_points;
    for(const auto& point_msg : marker.points)
    {
        cv_points.emplace_back(get_cv_point(point_msg));
    }

    auto color = get_cv_color(marker.outline_color);
    cv::polylines(image, cv_points, false, color, marker.thickness);
}

void image_draw::draw_line_list(const cv::Mat& image, const marker_t& marker)
{
    for(size_t i = 1; i < marker.points.size(); i += 2)
    {
        auto point_1 = get_cv_point(marker.points[i - 1]);
        auto point_2 = get_cv_point(marker.points[i]);
        auto color = get_cv_color(marker.outline_colors[i]);

        cv::line(image, point_1, point_2, color, marker.thickness);
    }
}

void image_draw::draw_polygon(const cv::Mat& image, const marker_t& marker)
{
    std::vector<cv::Point> cv_points;
    for(const auto& point_msg : marker.points)
    {
        cv_points.emplace_back(get_cv_point(point_msg));
    }

    auto color = get_cv_color(marker.outline_color);
    
    if(cv_points.size() == 2)
    {
        cv::rectangle(image, cv_points[0], cv_points[1], color, marker.thickness);
    }
    else
    {
        cv::polylines(image, cv_points, true, color, marker.thickness);
    }
}

void image_draw::draw_points(const cv::Mat& image, const marker_t& marker)
{
    for(size_t i = 0; i < marker.points.size(); ++i)
    {
        auto position = get_cv_point(marker.points[i]);
        auto color = get_cv_color(marker.outline_colors[i]);

        cv::drawMarker(image, position, color, cv::MarkerTypes::MARKER_CROSS,
            marker.scale, marker.thickness);
    }
}

void image_draw::draw_text(const cv::Mat& image, const marker_t& marker)
{
    auto color = get_cv_color(marker.outline_color);
    cv::Point position(marker.position.x, marker.position.y);
    
    cv::putText(image, marker.text, position, cv::FONT_HERSHEY_SIMPLEX, marker.scale,
        color, marker.thickness);
}

void image_draw::draw(const cv::Mat& image, const marker_t& marker)
{
    switch(marker.type)
    {
        case marker_t::CIRCLE :
        {
            draw_circle(image, marker);
            break;
        }
        case marker_t::LINE_STRIP :
        {
            draw_line_strip(image, marker);
            break;
        }
        case marker_t::LINE_LIST :
        {
            draw_line_list(image, marker);
            break;
        }
        case marker_t::POLYGON :
        {
            draw_polygon(image, marker);
            break;
        }
        case marker_t::POINTS :
        {
            draw_points(image, marker);
            break;
        }
        case marker_t::TEXT :
        {
            draw_text(image, marker);
            break;
        }
    }
}

void image_draw::image_callback(const sensor_msgs::msg::Image::ConstSharedPtr& image_msg) 
{
    std::vector<marker_t> markers;

    std::unique_lock<std::mutex> map_lock(m_marker_map_mutex);
    rclcpp::Time time_now = this->get_clock()->now();

    // While iterating over the ImageMarkers, the expired ImageMarkers must be removed.
    for (auto it = m_marker_map.cbegin(); it != m_marker_map.cend();)
    {
        if (it->second.second <= time_now)
        {
            it = m_marker_map.erase(it);
        }
        else
        {
            ++it;
        }
    }

    for(auto& map_key_value_pair : m_marker_map)
    {
        markers.emplace_back(*map_key_value_pair.second.first);
    }

    map_lock.unlock();
    
    cv_bridge::CvImageConstPtr image_ptr = cv_bridge::toCvCopy(image_msg,
        sensor_msgs::image_encodings::RGB8);

    for(const marker_t& marker : markers)
    {
        draw(image_ptr->image, marker);
    }

    m_image_pub.publish(image_ptr->toImageMsg());
}

void image_draw::image_marker_callback(const marker_t::SharedPtr msg)
{
    std::string marker_id = get_marker_id(*msg);
    std::lock_guard<std::mutex> map_lock(m_marker_map_mutex);

    if(msg->action == marker_t::ADD)
    {
        rclcpp::Time time_now = this->get_clock()->now();
        rclcpp::Time expire_time;

        // ImageMarkers with 0 duration will not expire.
        if(msg->lifetime.sec != 0 || msg->lifetime.nanosec != 0)
        {
            expire_time = time_now + msg->lifetime;
        }
        else
        {
            expire_time = rclcpp::Time::max();
        }
        m_marker_map[marker_id] = {msg, expire_time};
    }
    else if(msg->action == marker_t::REMOVE)
    {
        m_marker_map.erase(marker_id);
    }
}

} // namespace image_draw

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(image_draw::image_draw)
