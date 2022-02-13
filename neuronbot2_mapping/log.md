# scan rosbag 실패

```
$ ros2 bag info scan_rosbag/

Files:             scan_rosbag/scan_rosbag_0.db3
Bag size:          180.8 KiB
Storage id:        sqlite3
Duration:          12.763s
Start:             Feb 13 2022 21:41:06.198 (1644756066.198)
End                Feb 13 2022 21:41:18.961 (1644756078.961)
Messages:          892
Topic information: Topic: /scan | Type: sensor_msgs/msg/LaserScan | Count: 0 | Serialization Format: cdr
                   Topic: /tf_static | Type: tf2_msgs/msg/TFMessage | Count: 12 | Serialization Format: cdr
                   Topic: /tf | Type: tf2_msgs/msg/TFMessage | Count: 752 | Serialization Format: cdr
                   Topic: /clock | Type: rosgraph_msgs/msg/Clock | Count: 128 | Serialization Format: cdr
```