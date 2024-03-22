# tuw_object_map

## Demo
### Map

#### Using a satellite image
* Scale and map size is defined by the satellite image 
  *  mapimage from https://gis.stmk.gv.at
* publishes no utm frame
* a debug image is drawn 

```
ros2 run tuw_object_map object_map_node --ros-args  -p json_file:=./ws02/src/tuw_greenhive/config/object_map/starden-2024-01-14.json -p mapimage_folder:=./ws02/src/tuw_greenhive/res/greenhive-straden/satelittenbild/00/ -p show_map:=true
```

#### Auto mansfen
* comptues the map size
* publishes the utm frame
* using a map offset
```
ros2 run tuw_object_map object_map_node --ros-args  -p json_file:=./ws02/src/tuw_greenhive/config/object_map/starden-2024-01-14.json -p resolution:=0.1 -p show_map:=true -p auto_mansfen:=true -p publish_utm:=true -p map_origin_x:=-10.0  -p map_origin_y:=20.0
```

#### All map parameters are defined by hand
```
ros2 run tuw_object_map object_map_node --ros-args  -p json_file:=./ws02/src/tuw_greenhive/config/object_map/starden-2024-01-14.json -p resolution:=0.1 -p map_width:=1000  -p map_height:=500 -p show_map:=true
```

### Demo to view the object map.
RViz
```
ros2 run rviz2 rviz2 -d ./ws02/src/tuw_object_map/config/object_map.rviz
```

Object_map (Replace the command with one from above)
```
ros2 run tuw_object_map object_map_node --ros-args  -p json_file:=./ws02/src/tuw_greenhive/config/object_map/starden-2024-01-14.json -p mapimage_folder:=./ws02/src/tuw_greenhive/res/greenhive-straden/satelittenbild/01/ -p show_map:=false
```

Voronoi graph if needed
```
ros2 run tuw_graph_voronoi graph_voronoi_node --ros-args -p map_topic:=object_costmap -p loop_rate:=5
```