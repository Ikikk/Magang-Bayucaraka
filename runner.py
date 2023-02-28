import rospy
from geometry_msgs.msg import Twist
import subprocess
from math import radians
PI = 3.1415926535897
speed = 3

def half_circle(is_up):
	t0 = rospy.Time.now().to_sec()
	current_angle = 0

	vel.linear.x = 3
	vel.linear.y = 0
	vel.linear.z = 0
	vel.angular.x = 0
	vel.angular.y = 0
	if is_up:
		vel.angular.z = -radians(90)
	else :
		vel.angular.z = radians(90)

	while current_angle < radians(180):
		t1 = rospy.Time.now().to_sec()
		current_angle = radians(87)*(t1-t0)
		pub.publish(vel)
		rospy.loginfo("half circle")
		rate.sleep()
	
    
def line():
	distance = 4

	t0 = rospy.Time.now().to_sec()
	current_distance = 0
	
	vel.linear.x = 1
	vel.linear.y = 0
	vel.linear.z = 0
	vel.angular.x = 0
	vel.angular.y = 0
	vel.angular.z = 0
	while current_distance < distance:
		t1 = rospy.Time.now().to_sec()
		current_distance = 1*(t1-t0)
		pub.publish(vel)
		rospy.loginfo("forward")
		rate.sleep()


def rotate(is_clockwise):
	t0 = rospy.Time.now().to_sec()
	current_angle = 0

	angle = PI//2

	vel.linear.x = 0
	vel.linear.y = 0
	vel.linear.z = 0
	vel.angular.x = 0
	vel.angular.y = 0
	if is_clockwise:
		vel.angular.z = -abs(radians(90))
	else :
		vel.angular.z = abs(radians(90))

	while current_angle < angle:


		

		t1 = rospy.get_time()
		current_angle = 1*(t1-t0)
		pub.publish(vel)
		rospy.loginfo("rotate")
		rate.sleep()
	

if __name__ == '__main__':
	try:
		teleport_command = "rosservice call /turtle1/teleport_absolute 3.7 7.3 0"
		clear_command = "rosservice call /clear"
		subprocess.call(teleport_command, shell=True)
		subprocess.call(clear_command, shell=True)

		pub = rospy.Publisher('/turtle1/cmd_vel',Twist, queue_size=10)
		rospy.init_node('turtlesim', anonymous=True)
		rate = rospy.Rate(10)
		vel = Twist()
		
		line()
		half_circle(True)
			# half_circle(True)
		line()
		half_circle(True)
			# half_circle(True)
		rotate(False)
		half_circle(True)
		line()
		half_circle(True)
		line()


	except rospy.ROSInterruptException:
		pass