# tuw_object_map_server
## tuw_object_map_server
### Parameters
* __frame_id__: 
  * Name of the map frame used
  * default: *WGS84*
* __tuw_object_map_server__: 
  * Filename to load the object map from a json file
  * default: empty
* __loop_rate__: 
  * loop or publishing rate in seconds. If 0 or less, the graph is published once [sec]
  * default: *10.0*
### Publisher
* __tuw_object_map_msgs::ObjectMap__
  * object map from json
  * Topic: *objects*
### Service
* __tuw_object_map_msgs::srv::GetObjectMap__
  * object map from json on service request
  * Topic: *get_objects*

## Demo
RViz
```
ros2 run tuw_object_map_server object_map_server_node --ros-args -p object_map_json:=./ws02/src/tuw_greenhive/config/object_map/starden-2024-01-14.json
```

Make a service call
```
ros2 service call /get_objects tuw_object_map_msgs/srv/GetObjectMap "{}"
```
