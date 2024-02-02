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
        objects_utm_.clear();
        objects_map_.clear();
        for(const auto &obj_lla: objects){
                Object obj_utm = convert_LLA_to_UTM(obj_lla);
                Object obj_map = convert_UTM_to_MAP(obj_utm);
                objects_utm_.push_back(obj_utm);
                objects_map_.push_back(obj_map);
                
                std::cout << obj_map.geometry.points[0] << std::endl;
        }
}

int ObjectMap::find_utm_zone(const cv::Vec3d &lla)
{
        int zone;
        bool northp;
        double x, y, gamma, k;
        GeographicLib::UTMUPS::Forward(lla[0], lla[1], zone, northp, x, y, gamma, k);
        return zone;
}
cv::Vec3d &ObjectMap::convert_LLA_to_UTM(const cv::Vec3d &lla, cv::Vec3d &utm, int setzone)
{
        int zone;
        bool northp;
        double x, y, gamma, k;
        GeographicLib::UTMUPS::Forward(lla[0], lla[1], zone, northp, x, y, gamma, k, setzone);
        utm[0] = x, utm[1] = y;
        return utm;
}
void ObjectMap::set_origin(cv::Vec3d lla, double resolution)
{
        origin_lla_ = lla;
        bool northp;
        GeographicLib::UTMUPS::Forward(origin_lla_[0], origin_lla_[1], zone_utm_, northp, origin_utm_[0], origin_utm_[1]);
        origin_utm_[2] = origin_utm_[3];
        map_resolution_ = resolution;
}

cv::Vec3d ObjectMap::convert_LLA_to_UTM(const cv::Vec3d &lla)
{
        cv::Vec3d utm;
        return convert_LLA_to_UTM(lla, utm, zone_utm_);
}
cv::Vec3d ObjectMap::convert_UTM_to_MAP(const cv::Vec3d &utm)
{
        cv::Vec3d pm = utm - origin_utm_;
        pm[0] = pm[0] * map_resolution_;
        pm[1] = pm[1] * map_resolution_;
        pm[2] = 1;
        return pm;
}

Object ObjectMap::convert_LLA_to_UTM(const Object &src)
{
        Object des;
        des.id = src.id;
        des.type = src.type;
        des.geometry.type = des.geometry.type;
        for (const auto &point_src : src.geometry.points)
                des.geometry.points.push_back(convert_LLA_to_UTM(point_src));
        for (const auto &enflation_src : src.geometry.enflation)
                des.geometry.enflation.push_back(enflation_src);
        for (const auto &bondary_src : src.geometry.bondary)
                des.geometry.bondary.push_back(bondary_src);
        return des;
}
Object ObjectMap::convert_UTM_to_MAP(const Object &src)
{
        Object des;
        des.id = src.id;
        des.type = src.type;
        des.geometry.type = des.geometry.type;
        for (const auto &point_src : src.geometry.points)
                des.geometry.points.push_back(convert_UTM_to_MAP(point_src));
        for (const auto &enflation_src : src.geometry.enflation)
                des.geometry.enflation.push_back(enflation_src);
        for (const auto &bondary_src : src.geometry.bondary)
                des.geometry.bondary.push_back(bondary_src);
        return des;
}