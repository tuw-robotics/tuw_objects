# tuw_object_map

## Demo

using ros parameter
```
ros2 run tuw_object_map object_map_node --ros-args  -p json_file:=./ws02/src/tuw_greenhive/config/object_map/starden-2024-01-14.json -p mapimage_folder:=./ws02/src/tuw_greenhive/res/greenhive-straden/satelittenbild/00/
```

using mapimage from https://gis.stmk.gv.at
```
ros2 run tuw_object_map object_map_node --ros-args  -p json_file:=./ws02/src/tuw_greenhive/config/object_map/starden-2024-01-14.json -p resolution:=0.1 -p map_width:=1000  -p map_height:=500 -p center_origin:=true
```
