import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.srv import CommandBool, CommandBoolRequest
from mavros_msgs.srv import SetMode, SetModeRequest
import math
from aruco_class import ArucoCheck #custom library for aruco checking

# arm the drone
rospy.init_node("Sample")
rospy.wait_for_service("/mavros/cmd/arming")
try:
    arming = CommandBoolRequest()
    arming.value = True
    client = rospy.ServiceProxy("/mavros/cmd/arming",CommandBool)
    client.call(arming)

except rospy.ServiceException:
    pass

rospy.loginfo("armed")

setpoint_position = PoseStamped()
setpoint_position.pose.position.z=5     

local_position=PoseStamped()

def positioncb(msg: PoseStamped):
    global local_position
    local_position=msg

rospy.Subscriber("/mavros/local_position/pose",PoseStamped,callback=positioncb)
position_pub = rospy.Publisher("/mavros/setpoint_position/local",PoseStamped,queue_size=10)
rospy.loginfo("getting the local position")
count=0
rospy.loginfo("Publishing some initial setpoints")
rate = rospy.Rate(20)
while not rospy.is_shutdown():
    if count < 100:
        position_pub.publish(setpoint_position)
    else :
        break
    count+=1
    rate.sleep()

rospy.loginfo("Setting to offboard mode")

rospy.wait_for_service("/mavros/set_mode")
try :
    offb_client = rospy.ServiceProxy("/mavros/set_mode",SetMode)
    offb =SetModeRequest()
    offb.custom_mode="OFFBOARD"
    offb_client.call(offb)
except rospy.ServiceException:
    rospy.loginfo("Could not set to offboard")
    pass

rospy.loginfo("Set to offboard mode")
client.call(arming)
while not rospy.is_shutdown():
    if local_position.pose.position.z <= 4.95:
        position_pub.publish(setpoint_position)
    else: 
        break
    rate.sleep()

    
aruco_check=ArucoCheck()
# incrementally move in steps of 1m
while not rospy.is_shutdown():
    setpoint_position.pose.position.x += 1
    #after moving every 1m along x, stop and check for aruco tag in transforms
    while not rospy.is_shutdown():
        err=((local_position.pose.position.x-setpoint_position.pose.position.x)**2+(local_position.pose.position.y-setpoint_position.pose.position.y)**2+(local_position.pose.position.z-setpoint_position.pose.position.z)**2)**0.5
        if err >= 0.1:
            position_pub.publish(setpoint_position)
        else: 
            break
        rate.sleep()
    relative_pos=aruco_check.check_for_aruco() #returns an array of x, y, z, r, p, y of aruco tag wrt camera frame
    if relative_pos==-1:
        continue
    else:
        #required Aruco tag has been found!
        #relative position to absolute
        setpoint_position.pose.position.x+=relative_pos[0]
        setpoint_position.pose.position.y+=relative_pos[1]
        setpoint_position.pose.position.z+=relative_pos[2]
        setpoint_position.pose.position.z+=5 #5m above the Aruco

        #go to this location, 
        while not rospy.is_shutdown():
            err=((local_position.pose.position.x-setpoint_position.pose.position.x)**2+(local_position.pose.position.y-setpoint_position.pose.position.y)**2+(local_position.pose.position.z-setpoint_position.pose.position.z)**2)**0.5
            if err >= 0.1:
                position_pub.publish(setpoint_position)
            else: 
                break
            rate.sleep()

#land the drone on the Aruco
setpoint_position.pose.position.z = 0
while not rospy.is_shutdown():
    err=((local_position.pose.position.x-setpoint_position.pose.position.x)**2+(local_position.pose.position.y-setpoint_position.pose.position.y)**2+(local_position.pose.position.z-setpoint_position.pose.position.z)**2)**0.5
    if err >= 0.1:
        position_pub.publish(setpoint_position)
    else: 
        break
    rate.sleep()
        

