#include "tuw_object_map/geo_json_node.hpp"

#include <tuw_object_map_msgs/geo_json_json.hpp>
#include <tuw_json/json.hpp>

using std::placeholders::_1;
using namespace tuw_object_map;

GeoJSONNode::GeoJSONNode(const std::string &node_name)
    : Node(node_name)
{
    tuw_object_map_msgs::msg::GeoJSON::SharedPtr geoJson = std::make_shared<tuw_object_map_msgs::msg::GeoJSON>();
    tuw_json::fromJson(tuw_json::read("geojson.json", "object_map"), *geoJson);

    // TODO: Proper initialization of object_map_
    object_map_.info().resolution = 1. / 5.0;
    object_map_.info().size.width = 1000;
    object_map_.info().size.height = 1000;
    object_map_.info().origin.x() = 0;
    object_map_.info().origin.y() = 0;
    object_map_.init_map(46.80213975, 15.83715523, 338.917);

    RCLCPP_INFO(this->get_logger(), "%s", object_map_.info().info_map().c_str());

    callback_geo_json(geoJson);
}

void GeoJSONNode::callback_geo_json(const tuw_object_map_msgs::msg::GeoJSON::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "I received a map");

    draw(msg);
}

void GeoJSONNode::draw(const tuw_object_map_msgs::msg::GeoJSON::SharedPtr msg)
{
    // Free
    for (auto &feature : msg->features)
    {
        for (auto &feature : msg->features)
        {
            if (feature.geometry.coordinates.size() == 1)
            { // Point
                tuw_object_map_msgs::msg::GeoPoint p = feature.geometry.coordinates[0];

                cv::Vec3d p0 = cv::Vec3d(p.latitude, p.longitude, p.altitude);
                object_map_.line(p0, p0, ObjectMap::CELL_FREE, 2);
            }
            else
            { // Polygon
                tuw_object_map_msgs::msg::GeoPoint p0 = feature.geometry.coordinates[0];

                for (auto &point : feature.geometry.coordinates)
                {
                    cv::Vec3d p1 = cv::Vec3d(p0.latitude, p0.longitude, p0.altitude);
                    cv::Vec3d p2 = cv::Vec3d(point.latitude, point.longitude, point.altitude);
                    object_map_.line(p1, p2, ObjectMap::CELL_FREE, 2);
                    p0 = point;
                }
            }
        }
    }

    // Occupied
    for (auto &feature : msg->features)
    {
        for (auto &feature : msg->features)
        {
            if (feature.geometry.coordinates.size() == 1)
            { // Point
                tuw_object_map_msgs::msg::GeoPoint p = feature.geometry.coordinates[0];

                cv::Vec3d p0 = cv::Vec3d(p.latitude, p.longitude, p.altitude);
                object_map_.line(p0, p0, ObjectMap::CELL_OCCUPIED, 0.2);
            }
            else
            { // Polygon
                tuw_object_map_msgs::msg::GeoPoint p0 = feature.geometry.coordinates[0];

                for (auto &point : feature.geometry.coordinates)
                {
                    cv::Vec3d p1 = cv::Vec3d(p0.latitude, p0.longitude, p0.altitude);
                    cv::Vec3d p2 = cv::Vec3d(point.latitude, point.longitude, point.altitude);
                    object_map_.line(p1, p2, ObjectMap::CELL_OCCUPIED, 0.2);
                    p0 = point;
                }
            }
        }
    }

    object_map_.imshow(1000);
}

int main(int argc, char *argv[])
{
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);

    rclcpp::executors::SingleThreadedExecutor exe;
    std::shared_ptr<tuw_object_map::GeoJSONNode> node =
        std::make_shared<tuw_object_map::GeoJSONNode>("geo_json_node");
    exe.add_node(node->get_node_base_interface());
    exe.spin();

    rclcpp::shutdown();

    return 0;
}
