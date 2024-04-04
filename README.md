# imu_localizer

## Subscriber
|topic name|type|purpose|
|:--:|:--:|:--:|
|/imu|sensor_msgs/msg/Imu|Get IMU data|

## Publisher
|topic name|type|purpose|
|:--:|:--:|:--:|
|/odom|nav_msgs/msg/Odometry| publish result(EKF posture and integral pose)|
|/euler|geometry_msgs/msg/Vector3|publish euler|

## Parameters
|name|default|purpose|
|:--:|:--:|:--:|
|delta_milli|10|set time duration for calc ekf|
|odom_frame_id|"odom"|set publish odometry frame id|
