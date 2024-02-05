# tuw_object_map

## Demo
### Map
using ros parameter
```
ros2 run tuw_object_map object_map_node --ros-args  -p json_file:=./ws02/src/tuw_greenhive/config/object_map/starden-2024-01-14.json -p mapimage_folder:=./ws02/src/tuw_greenhive/res/greenhive-straden/satelittenbild/00/ -p show_map:=true
```

using mapimage from https://gis.stmk.gv.at
```
ros2 run tuw_object_map object_map_node --ros-args  -p json_file:=./ws02/src/tuw_greenhive/config/object_map/starden-2024-01-14.json -p resolution:=0.1 -p map_width:=1000  -p map_height:=500 -p show_map:=true
```
```
ros2 run tuw_object_map object_map_node --ros-args  -p json_file:=./ws02/src/tuw_greenhive/config/object_map/starden-2024-01-14.json -p resolution:=0.1 -p map_width:=1000  -p map_height:=500 -p origin_latitude:=46.80213975  -p origin_longitude:=15.83715523 -p origin_altitude:=338.917 -p show_map:=true
```

### Voronoi
```
# rviz
ros2 run rviz2 rviz2 -d ./ws02/src/tuw_object_map/config/object_map.rviz

# object_map
ros2 run tuw_object_map object_map_node --ros-args  -p json_file:=./ws02/src/tuw_greenhive/config/object_map/starden-2024-01-14.json -p mapimage_folder:=./ws02/src/tuw_greenhive/res/greenhive-straden/satelittenbild/01/ -p show_map:=false

# voronoi graph
ros2 run tuw_graph_voronoi graph_voronoi_node --ros-args -p map_topic:=object_costmap -p loop_rate:=5
```