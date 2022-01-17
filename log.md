
```
[ekf_node-1] Warning: Invalid frame ID "imu_link" passed to canTransform argument source_frame - frame does not exist
```

=> robot_localization 틀었는데 자꾸 이런 에러 난다.
=> ekf.yaml에 이렇게 use_sim_time을 주면 됨
```
### ekf config file ###
ekf_filter_node:
    ros__parameters:
        use_sim_time: true

```

현재 tf odom은 바퀴에서 나오는 것이다.
ekf.yaml에서 publish_tf: true로 하면 되긴 하는데 rviz에서 자꾸 순간이동하는 현상 발생하기 때문에 하나만 보이게 false로 하였음.


laser scan rviz에서 보려면 qos option Best Effort로 해야 한다.
