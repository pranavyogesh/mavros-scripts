mavros scripts for drone flight control in gazebo environment

Before running python script, run the following:

 (In px4 installation folder) make px4_sitl gazebo
 
 roscore
 
 roslaunch mavros px4.launch fcu_url:="udp://:14540@127.0.0.1:14557"


![](spiral.mp4)

