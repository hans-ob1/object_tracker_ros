# object_tracker_ros

Based on ROS and cpp. This object tracker serves an extension to neural-cam-ros package. The tracker is implemented based on extended kalman filter and hungarian algorithm. 

##### Additional Information
  - requires neural_cam_ros running in parallel
  - open another terminal and type "rostopic echo /obstacle" to see detected objects been published
  - tracks and marks the position of the object detected with their respective ID
  - ID starting with 1 - pedestrian, 2 - bike and 3 - vehicle

##### System Requirement
- cmake 2.8 or above
- ROS indigo or above
- runs on opencv 3.1 (optional if you only intend to poccess images)
- ubuntu 14.04 above
- dlib v1.19

##### Usage Detail
requires ROS to run rosmake
  ```sh
    $ roscd object_tracker_ros
    $ rosmake
    $ rosrun object_tracker_ros tracker
 ```
