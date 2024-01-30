import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.srv import CommandBool, CommandBoolRequest
from mavros_msgs.srv import SetMode, SetModeRequest
import math

# arm
# setpoints
# set to offboard
# setpoint as 0,0,5
# initiate spiral path

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

rospy.loginfo("Target altitude reached")



set_vel= TwistStamped()

vel_pub =rospy.Publisher("/mavros/setpoint_velocity/cmd_vel",TwistStamped,queue_size=10)

# set velocity paramaters, based on given velocity equation
vf=8
vt=2.5
n=0.5
while not rospy.is_shutdown():
    t=rospy.get_time()
    set_vel.twist.linear.y=vf
    set_vel.twist.linear.z=vt*math.sin(2*math.pi*n*t)
    set_vel.twist.linear.x=vt*math.cos(2*math.pi*n*t)
    vel_pub.publish(set_vel)
    rate.sleep()