# tuw_object_map
## Parameters
* __publish_tf__: 
  * On true a tf from frame_utm to frame_map is published
  * default: *true*
* __frame_map__: 
  * Name of the map frame, only need if *publish_tf == true*
  * default: *object_map*
* __frame_utm__: 
  * Name of the utm frame, only need if *publish_tf == true*
  * default: *utm*
* __json_file__: 
  * Filename to load the object map from a json file if not set the node will wait for a msg on the topic *objects*
  * default: empty
* __resolution__: 
  * Resolution of the generated map [m/pix]
  * default: *0.1*
* __border__: 
  * Border on the created map [meter]
  * default: *10.0*
* __debug_folder__: 
  * If set it stores debug information and images there
  * default: */tmp/ros/object_map*
* __show_map__: 
  * Shows the map in a opencv window
  * default: *false*
## Publisher
* __nav_msg::OccupancyGrid__
  * Costmap with Objects 
  * Topic: *object_map*
## Subscriber
* __tuw_object_map_msgs::ObjectMap__
  * ObjectMap, as alternative one can provide the node with a json file (json_file)
  * Topic: *objects*

## Demo
RViz
```
ros2 run rviz2 rviz2 -d ./ws02/src/tuw_object_map/config/object_map.rviz
```

Object_map (Replace the command with one from above)
```
ros2 run tuw_object_map object_map_node --ros-args  -p json_file:=./ws02/src/tuw_greenhive/config/object_map/starden-2024-01-14.json -p publish_tf:=true -p map_border:=2.0 -p show_map:=false
```

Voronoi graph if needed
```
ros2 run tuw_graph_voronoi graph_voronoi_node --ros-args -p map_topic:=object_map -p loop_rate:=5
```


### tmuxinator
```bash
tmuxinator start -p ./ws02/src/tuw_object_map/tmux/object_map_with_satelitt_image.yml
```