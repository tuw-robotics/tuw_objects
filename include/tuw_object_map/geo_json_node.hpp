#pragma once

#include <rclcpp/rclcpp.hpp>

#include <tuw_object_map_msgs/msg/geo_json.hpp>
#include <tuw_object_map/object_map.hpp>

namespace tuw_object_map
{

    class GeoJSONNode : public rclcpp::Node
    {
    public:
        GeoJSONNode(const std::string &node_name);

    private:
        void callback_geo_json(const tuw_object_map_msgs::msg::GeoJSON::SharedPtr msg);

        // rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_occupancy_grid_map_;
        // rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_occupancy_grid_img_;

        ObjectMap object_map_;
        void draw(const tuw_object_map_msgs::msg::GeoJSON::SharedPtr msg);

        // nav_msgs::msg::OccupancyGrid::SharedPtr occupancy_map_;
        // nav_msgs::msg::OccupancyGrid::SharedPtr occupancy_img_;
        // ObjectMap object_map_;
        // std::string json_file_;
    };
}