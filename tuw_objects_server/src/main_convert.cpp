#include <memory>
#include <iostream>
#include <fstream>
#include <filesystem>

#include "rclcpp/rclcpp.hpp"
#include <json/json.h>
#include <tuw_json/json.hpp>
#include <tuw_object_map_msgs/objects_json.hpp>
#include <tuw_object_msgs/shape_array_json.hpp>

void convert(const tuw_object_map_msgs::Objects src, tuw_object_msgs::ShapeArray &des, bool use_wgs84){
    for(auto &object: src.objects){
        tuw_object_msgs::Shape shape;
        shape.id = object.id;
        shape.type = object.type;
        shape.shape = tuw_object_msgs::Shape::SHAPE_NA;
        if (use_wgs84){
            for(auto &wgs84: object.geo_points){
                tuw_geometry_msgs::Point p(wgs84.latitude, wgs84.longitude, wgs84.altitude);
                shape.points.push_back(std::move(p));
            }
        } else 
        {
            shape.points = object.map_points;
        }
        if (shape.type == tuw_object_msgs::Shape::TYPE_PLANT)
            shape.shape = tuw_object_msgs::Shape::SHAPE_POINT;
        else if (shape.type == tuw_object_msgs::Shape::TYPE_PLANT_WINE_ROW)
            shape.shape = tuw_object_msgs::Shape::SHAPE_LINE_STRIP;
        else if (shape.type == tuw_object_msgs::Shape::TYPE_OBSTACLE)
            shape.shape = tuw_object_msgs::Shape::SHAPE_POINT;
        else if (shape.type == tuw_object_msgs::Shape::TYPE_OBSTACLE_HOUSE)
            shape.shape = tuw_object_msgs::Shape::SHAPE_POLYGON;
        else if (shape.type == tuw_object_msgs::Shape::TYPE_OBSTACLE_TREE)
            shape.shape = tuw_object_msgs::Shape::SHAPE_CIRCLE;
        
        shape.params_points.clear();
        shape.params.id = object.id;
        for(size_t i = 0; i < object.enflation_radius.size(); i++){
            tuw_std_msgs::ParameterArray param;
            param.add("free", object.enflation_radius[i]);
            param.add("occupied", object.bondary_radius[i]);
            param.id = i;
            shape.params_points.push_back(std::move(param));
        } 
        des.shapes.push_back(std::move(shape));       
    }
    des.header = src.header;
    if (use_wgs84){
        des.header.frame_id = "wgs84";
    }
}

int main(int argc, char* argv[]) {
    bool use_wgs84 = false;
    if ((argc != 3) && (argc != 4))  {
        std::cerr << "Usage: " << argv[0] << " <object_map.json> <shape_array.json> [use_wgs84]" << std::endl;
        return 1;
    }

    // Check if the source file exists
    if (!std::filesystem::exists(argv[1])) {
        std::cerr << "Error: Source file " << argv[1] << " does not exist." << std::endl;
        return 1;
    }
    std::string src_file = argv[1];

    // Check if the destination file exists (or its parent directory)
    std::filesystem::path dest_path(argv[2]);
    if (!std::filesystem::exists(dest_path.parent_path())) {
        std::cerr << "Error: Destination directory " << dest_path.parent_path() << " does not exist." << std::endl;
        return 1;
    }
    std::string des_file = argv[2];

    if(argc == 4){
        if(strcmp(argv[3], "use_wgs84") == 0){
            use_wgs84 = true;
            std::cout << "using gps coordiantes!" << std::endl;
        }
    }

    tuw_object_map_msgs::Objects src_obj;
    tuw_json::fromJson(tuw_json::read(src_file, "objects"), src_obj);
    std::cout << "shape map created." << std::endl;

    tuw_object_msgs::ShapeArray des_obj;
    convert(src_obj, des_obj, use_wgs84);

    tuw_json::write(des_file, "shapes", tuw_json::toJson(des_obj));


    return 0;
}
