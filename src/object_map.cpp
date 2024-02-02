#include "tuw_object_map/object_map.hpp"

#include <iostream>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <GeographicLib/UTMUPS.hpp>

using namespace tuw_object_map;

ObjectMap::ObjectMap()
{
}

void ObjectMap::process(const std::vector<Object> &objects)
{
        objects_.clear();
        for (const auto &obj : objects)
        {
                objects_.push_back(obj);
                std::cout << obj.geometry.points[0] << std::endl;
                std::cout << obj.geometry.points[1] << std::endl;
        }
}

void ObjectMap::set_origin(cv::Vec3d lla, double resolution)
{
        origin_lla_ = lla;
        bool northp;
        GeographicLib::UTMUPS::Forward(origin_lla_[0], origin_lla_[1], zone_utm_, northp, origin_utm_[0], origin_utm_[1]);
        origin_utm_[2] = origin_utm_[2];
        map_resolution_ = resolution;
}

cv::Vec3d &ObjectMap::convert_LLA_to_MAP(const cv::Vec3d &p_lla, cv::Vec3d &p_map)
{

        int zone;
        bool northp;
        double x, y, gamma, k;
        GeographicLib::UTMUPS::Forward(p_lla[0], p_lla[1], zone, northp, x, y, gamma, k, zone_utm_);
        p_map[0] = x - origin_utm_[0];
        p_map[1] = y - origin_utm_[1];
        p_map[2] = p_lla[3] - origin_utm_[3];
        return p_map;
}
