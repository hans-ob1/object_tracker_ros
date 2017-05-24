FILE(REMOVE_RECURSE
  "../msg_gen"
  "../msg_gen"
  "../src/object_tracker_ros/msg"
  "CMakeFiles/ROSBUILD_genmsg_py"
  "../src/object_tracker_ros/msg/__init__.py"
  "../src/object_tracker_ros/msg/_obstacleStack.py"
  "../src/object_tracker_ros/msg/_obstacle.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
