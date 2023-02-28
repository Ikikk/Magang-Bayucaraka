import cv2
import rospy
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest
from mavros_msgs.msg import State
from geometry_msgs.msg import PoseStamped
import math

import os
os.chdir("finalproject")

current_state = State()

def state_cb(msg):
    global current_state
    current_state = msg

def scanedges(img) :
    # resize img
    h, w, c = img.shape
    img = cv2.resize(img, (int(w/2), int(h/2)))

    # convert to grayscale
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # detecting edges
    edges = cv2.Canny(gray, 170, 255)

    # take areas with more intensity
    ret,thresh = cv2.threshold(gray,240,255,cv2.THRESH_BINARY)

    # get all contours
    contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # for each contour
    for contour in contours:
        # get length from each contour
        peri = cv2.arcLength(contour, True)
        # get how many dots
        approx = cv2.approxPolyDP(contour, 0.02 * peri, True)
    length = len(approx)
    # getRotate(len(approx))
    return length


def getRotate(num_rotation) :
    circle_radius = 5.0 # meters

    t0 = rospy.get_time()
    # num_rotation = 5
    while not rospy.is_shutdown() and (rospy.get_time() - t0) < num_rotation * (2 * math.pi):
        x = circle_radius * math.cos(rospy.get_time() - t0)
        y = circle_radius * math.sin(rospy.get_time() - t0)
        z = 3

        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z

        local_pos_pub.publish(pose)

        rate.sleep()

    
if __name__ == '__main__':
    rospy.init_node("finalproject_py")

    state_sub = rospy.Subscriber("mavros/state", State, callback = state_cb)

    local_pos_pub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=10)
    
    rospy.wait_for_service("/mavros/cmd/arming")
    arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)    

    rospy.wait_for_service("/mavros/set_mode")
    set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)
    

    # Setpoint publishing MUST be faster than 2Hz
    rate = rospy.Rate(20)

    # Wait for Flight Controller connection
    while(not rospy.is_shutdown() and not current_state.connected):
        rate.sleep()

    pose = PoseStamped()

    pose.pose.position.x = 0
    pose.pose.position.y = 0
    pose.pose.position.z = 2

    # Send a few setpoints before starting
    for i in range(100):   
        if(rospy.is_shutdown()):
            break

        local_pos_pub.publish(pose)
        rate.sleep()

    offb_set_mode = SetModeRequest()
    offb_set_mode.custom_mode = 'OFFBOARD'

    arm_cmd = CommandBoolRequest()
    arm_cmd.value = True

    last_req = rospy.Time.now()

    while(not rospy.is_shutdown()):
        if(current_state.mode != "OFFBOARD" and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
            if(set_mode_client.call(offb_set_mode).mode_sent == True):
                rospy.loginfo("OFFBOARD enabled")
            
            last_req = rospy.Time.now()
        else:
            if(not current_state.armed and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
                if(arming_client.call(arm_cmd).success == True):
                    rospy.loginfo("Vehicle armed")
            
                last_req = rospy.Time.now()

        local_pos_pub.publish(pose)

        rate.sleep()

    # img = cv2.imread('shape3.png')
    # num = scanedges(img)
        # scanedges(img)
    getRotate(5)
    rate.sleep()