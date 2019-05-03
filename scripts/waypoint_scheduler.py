#!/usr/bin/env python
# ROS python API
import rospy

# import needed geometry messages
from geometry_msgs.msg import Point, Vector3, PoseStamped, TwistStamped
# import all mavros messages and services
from mavros_msgs.msg import *
from mavros_msgs.srv import *

# import other system/utils
import time, sys, math

# Global variables
waypoints = [[0, 0, 0], [1, 0, 1], [-1, 0, 1], [2, 0, 1], [-2, 0, 1], [3, 0, 1], [-3, 0, 1], [2, 0, 1], [-2, 0, 1], [1, 0, 1], [-1, 0, 1], [10, 0, 1], [-10, 0, 1]]
threshold = 0.25

# Flight modes class
# Flight modes are activated using ROS services
class FlightModes:
    def __init__(self):
        pass

    def setArm(self):
        rospy.wait_for_service('mavros/cmd/arming')
        try:
            armService = rospy.ServiceProxy('mavros/cmd/arming', mavros_msgs.srv.CommandBool)
            armService(True)
        except rospy.ServiceException, e:
            print "Service arming call failed: %s"%e

    def setOffboardMode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='OFFBOARD')
        except rospy.ServiceException, e:
            print "service set_mode call failed: %s. Offboard Mode could not be set."%e

# Flight parameters class
# Flight parameters are activated using ROS services
class FlightParams:
    def __init__(self):
        # pull all parameters
        # rospy.wait_for_service('mavros/param/pull')
        pulled = False
        while not pulled:
            try:
                paramService = rospy.ServiceProxy('mavros/param/pull', mavros_msgs.srv.ParamPull)
                parameters_recv = paramService(True)
                pulled = parameters_recv.success
            except rospy.ServiceException, e:
                print "service param_pull call failed: %s. Could not retrieve parameter list."%e

    def getTakeoffHeight(self):
        return rospy.get_param('mavros/param/MIS_TAKEOFF_ALT')

    def getHoverThrust(self):
        return rospy.get_param('mavros/param/MPC_THR_HOVER')

# Offboard controller for sending setpoints
class Controller:
    def __init__(self):
        # Drone state
        self.state = State()

        # A Message for the current local position of the drone
        self.local_pos = Point(0.0, 0.0, 0.0)

        # A Message for the current linear velocity of the drone
        self.local_vel = Vector3(0.0, 0.0, 0.0)

        # Instantiate the position setpoint message
        self.pos_sp = PositionTarget()
        # set the flag to control height
        self.pos_sp.type_mask = int('110111111000', 2)
        # LOCAL_NED
        self.pos_sp.coordinate_frame = 1
        # initial values for setpoints
        self.pos_sp.position.x = 0.0
        self.pos_sp.position.y = 0.0
        self.pos_sp.position.z = 0.0

        # Obtain flight parameters
        params = FlightParams()
        self.takeoffHeight = params.getTakeoffHeight()
        self.hoverThrust = params.getHoverThrust()

    # Update setpoint message
    def updateSp(self, n_step, e_step, d_step):
        # Set step value
        self.pos_sp.position.y = n_step
        self.pos_sp.position.x = e_step
        self.pos_sp.position.z = -d_step

        # Set mask
        self.pos_sp.type_mask = int('110111111000', 2) 

    # Callbacks.

    ## Drone State callback
    def stateCb(self, msg):
        self.state = msg

    ## Drone local position callback
    def posCb(self, msg):
        self.local_pos.x = msg.pose.position.x
        self.local_pos.y = msg.pose.position.y
        self.local_pos.z = msg.pose.position.z

    ## Drone linear velocity callback
    def velCb(self, msg):
        self.local_vel.x = msg.twist.linear.x
        self.local_vel.y = msg.twist.linear.y
        self.local_vel.z = msg.twist.linear.z

def publish_setpoint(cnt, pub_pos):
    pub_pos.publish(cnt.pos_sp)

def run(argv):
    # initiate node
    rospy.init_node('setpoint_node', anonymous=True)

    # flight mode object
    modes = FlightModes()

    # controller object
    cnt = Controller()

    # ROS loop rate
    rate = rospy.Rate(200.0)

    # Subscribe to drone state
    rospy.Subscriber('mavros/state', State, cnt.stateCb)

    # Subscribe to drone's local position
    rospy.Subscriber('mavros/local_position/pose', PoseStamped, cnt.posCb)

    # Subscribe to drone's linear velocity
    rospy.Subscriber('mavros/local_position/velocity', TwistStamped, cnt.velCb)

    # Setpoint publishers   
    sp_pos_pub = rospy.Publisher('mavros/setpoint_raw/local', PositionTarget, queue_size=1)

    # Arm the drone
    print("Arming")
    while not (cnt.state.armed or rospy.is_shutdown()):
        modes.setArm()
        rate.sleep()
    print("Armed\n")

    # activate OFFBOARD mode
    print("Activate OFFBOARD mode")
    while not (cnt.state.mode == "OFFBOARD" or rospy.is_shutdown()):
        # We need to send few setpoint messages, then activate OFFBOARD mode, to take effect
        k=0
        while k<10:
            sp_pos_pub.publish(cnt.pos_sp)
            rate.sleep()
            k = k + 1

        modes.setOffboardMode()
        rate.sleep()
    print("OFFBOARD mode activated\n")

    # Make sure the drone is armed
    print("Arming")
    while not (cnt.state.armed or rospy.is_shutdown()):
        modes.setArm()
        rate.sleep()
    print("Armed\n")

    # Takeoff
    print("Taking off")
    while not (abs(cnt.local_pos.z - cnt.takeoffHeight) < 0.2 or rospy.is_shutdown()):
        cnt.updateSp(0, 0, -cnt.takeoffHeight)
        sp_pos_pub.publish(cnt.pos_sp)
        rate.sleep()
    print("Reached takeoff height\n")

    # ROS main loop - first set value to zero before stepping
    print("Following waypoints...")
    current_wp = 0
    while current_wp < len(waypoints) and not rospy.is_shutdown():
        y = waypoints[current_wp][0]
        x = waypoints[current_wp][1]
        z = waypoints[current_wp][2] + cnt.takeoffHeight

        if abs(cnt.local_pos.x - x) < threshold and abs(cnt.local_vel.x) < threshold and abs(cnt.local_pos.y - y) < threshold and abs(cnt.local_vel.y) < threshold and abs(cnt.local_pos.z - z) < threshold and abs(cnt.local_vel.z) < threshold:
            current_wp = current_wp + 1
            print("Reached waypoint %d / %d" % (current_wp, len(waypoints)))

        # print("%f %f %f" % (abs(cnt.local_pos.x - x), abs(cnt.local_pos.y - y), abs(cnt.local_pos.z - z)))
        cnt.updateSp(waypoints[current_wp][0], waypoints[current_wp][1], -waypoints[current_wp][2] - cnt.takeoffHeight)
        publish_setpoint(cnt, sp_pos_pub)
        rate.sleep()
    print("Last waypoint reached\n")
    
    # Land quadrotor
    print("Activate LAND mode")
    while not (cnt.state.mode == "AUTO.LAND" or rospy.is_shutdown()):
        modes.setLandMode()
        rate.sleep()
    print("LAND mode activated\n")

def main(argv):
    try:
		run(argv)
    except rospy.ROSInterruptException:
        pass
    print("Terminated.\n")

if __name__ == "__main__":
    main(sys.argv[1:])
