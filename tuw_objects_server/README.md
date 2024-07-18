# tuw_object_map_server
## tuw_object_map_server
### Parameters
* __frame_id__: 
  * Name of the map frame used
  * default: *WGS84*
* __objects_json__: 
  * Filename to load the object map from a json file
  * default: empty
* __pub_interval__: 
  * publishing interval in seconds. If 0 or less, the graph is published once [sec] and whats for a service request *get_objects* or *publish*
  * default: *10.0*
### Publisher
* __tuw_object_map_msgs::ObjectMap__
  * object map from json
  * Topic: *objects*
### Service
* __get_objects__
  * objects from json on service request
  * Type: *tuw_object_map_msgs::srv::GetObjects*
* __publish__
  * triggers the node to publish the objects again
  * Topic: *std_srvs::srv::Trigger*

## Demo
RViz
```
ros2 run tuw_objects_server objects_server_node --ros-args -p objects_json:=./ws02/src/tuw_greenhive/config/object_map/starden-2024-01-14.json
```

Make a service call
```
ros2 service call /get_objects tuw_object_map_msgs/srv/GetObjects "{}"
ros2 service call /publish std_srvs/srv/Trigger "{}"
```
