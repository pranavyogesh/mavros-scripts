To set up, run:
 Launch typhoon h480 drone(with cam)(In px4 installation folder) make px4_sitl gazebo-classic_typhoon_h480
 roscore
 roslaunch mavros px4.launch fcu_url:="udp://:14540@127.0.0.1:14557"	
 Change camera orientation to downward-facing:rosrun typhoon_drone_camera_rotate.sh
 To set camera link in transform, roslaunch task3 camera_tf.launch
 Start aruco_detect node: roslaunch task3 aruco_detect.launch
 Manually insert aruco cube(downloaded from https://github.com/FabianReister/gazebo_aruco_box)in gazebo, somewhere along x-axis
 Finally, run the mission script, aruco_landing.py
