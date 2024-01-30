import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.srv import CommandBool, CommandBoolRequest
from mavros_msgs.srv import SetMode, SetModeRequest

# arm
# initial setpoint
# set to offboard
# go to points
# land

# arm the drone
rospy.init_node("Drone")
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
#initial setpoint, 5m upward
client.call(arming)
while not rospy.is_shutdown():
    if local_position.pose.position.z <= 4.95:
        position_pub.publish(setpoint_position)
    else: 
        break
    rate.sleep()

rospy.loginfo("Target altitude reached")

#move diagonally in space, upward
setpoint_position.pose.position.x += 2
setpoint_position.pose.position.y += 2
setpoint_position.pose.position.z += 2
while not rospy.is_shutdown():
    err=((local_position.pose.position.x-setpoint_position.pose.position.x)**2+(local_position.pose.position.y-setpoint_position.pose.position.y)**2+(local_position.pose.position.z-setpoint_position.pose.position.z)**2)**0.5
    if err >= 0.1:
        position_pub.publish(setpoint_position)
    else: 
        break
    rate.sleep()

#move diagonally,in reverse
setpoint_position.pose.position.x -= 2
setpoint_position.pose.position.y -= 2
setpoint_position.pose.position.z -= 2
while not rospy.is_shutdown():
    err=((local_position.pose.position.x-setpoint_position.pose.position.x)**2+(local_position.pose.position.y-setpoint_position.pose.position.y)**2+(local_position.pose.position.z-setpoint_position.pose.position.z)**2)**0.5
    if err >= 0.1:
        position_pub.publish(setpoint_position)
    else: 
        break
    rate.sleep()

#land the drone
setpoint_position.pose.position.z = 0
while not rospy.is_shutdown():
    err=((local_position.pose.position.x-setpoint_position.pose.position.x)**2+(local_position.pose.position.y-setpoint_position.pose.position.y)**2+(local_position.pose.position.z-setpoint_position.pose.position.z)**2)**0.5
    if err >= 0.1:
        position_pub.publish(setpoint_position)
    else: 
        break
    rate.sleep()