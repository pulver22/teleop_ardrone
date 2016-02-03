#!/usr/bin/env python
import roslib; roslib.load_manifest('teleop_twist_keyboard')
import rospy

from geometry_msgs.msg import Twist
from std_msgs.msg import Empty

import sys, select, termios, tty

msg = """
Reading from the keyboard  and Publishing to Twist!
---------------------------
Taking off ==> Press 't'

Landing ==> Press 'g'

Moving around:
   u    i    o
   j    k    l
   m    ,    .

q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%
anything else : stop

CTRL-C to quit
"""
#each button is associated with a 6-dim tuple: (x,th_x,y,th_y,z,th_z)

#(linear,angular) velocity on the three axis
moveBindingsAxis = {
		# X axis
			#pure linear
		'i':(1,0,0,0,0,0),
		',':(-1,0,0,0,0,0),
			#linear and angular(on Z)
				#forward
		'o':(1,0,0,0,0,-1),
		'u':(1,0,0,0,0,1),
				#backward
		'.':(-1,0,0,0,0,1),
		'm':(-1,0,0,0,0,-1),
			
		
		# Z axis
			#linear
		'v':(0,0,0,0,1,0),
		'b':(0,0,0,0,-1,0),
			#angular
		'j':(0,0,0,0,0,1),
		'l':(0,0,0,0,0,-1),
		
		#reset
		'k':(0,0,0,0,0,0)
	       }

#increase/decrease velocity on X axis
speedBindingsAxis={
		'q':(1.1,1.1), #increase linear and angular velocity
		'z':(.9,.9), #decrease linear and angular velocity
		'w':(1.1,1), #increase only linear vel
		'x':(.9,1), #decrease only linear vel
		'e':(1,1.1), #increase only rot vel
		'c':(1,.9), #decrease only rot vel
	      }

landingTakingOff={
		't':(0,0),
		'g':(0,0),
	      }

def getKey():
	tty.setraw(sys.stdin.fileno())
	select.select([sys.stdin], [], [], 0)
	key = sys.stdin.read(1)
	termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
	return key

speed = .5
turn = 1

def vels(speed,turn):
	return "currently:\tspeed %s\tturn %s " % (speed,turn)

if __name__=="__main__":
    	settings = termios.tcgetattr(sys.stdin)
	
	pub_twist = rospy.Publisher('cmd_vel', Twist)
	pub_empty_takeoff = rospy.Publisher('/ardrone/takeoff', Empty)
	pub_empty_landing = rospy.Publisher('/ardrone/land', Empty)
	rospy.init_node('teleop_ardrone_keyboard')

	x =y = z = 0
	th_x = th_y = th_z = 0
	status = 0

	try:
		print msg
		print vels(speed,turn)
		while(1):
			key = getKey()
			if key in moveBindingsAxis.keys():
				# x is linear speed, th is the angular one
				x = moveBindingsAxis[key][0]
				th_x = moveBindingsAxis[key][1]
				y = moveBindingsAxis[key][2]
				th_y = moveBindingsAxis[key][3]
				z = moveBindingsAxis[key][4]
				th_z = moveBindingsAxis[key][5]
			
				
			elif key in speedBindingsAxis.keys():
				# increase or decrease linear or angular speed
				speed = speed * speedBindingsAxis[key][0]
				turn = turn * speedBindingsAxis[key][1]

				print vels(speed,turn)
				if (status == 14):
					print msg
				status = (status + 1) % 15
				
			elif key in landingTakingOff.keys():
				
				if (key == 't'):
					# publish mex to take off
					pub_empty_takeoff.publish(Empty());
					continue;
				else:
					#publish mex to landing
					pub_empty_landing.publish(Empty());
					continue;
			else:
				x = 0
				th = 0
				if (key == '\x03'):
					break
				
			
			twist = Twist()
			twist.linear.x = x*speed; twist.linear.y = y*speed; twist.linear.z = z*speed
			twist.angular.x = th_x*turn; twist.angular.y = th_y*turn; twist.angular.z = th_z*turn
			pub_twist.publish(twist)
			

	except:
		print e

	finally:
		twist = Twist()
		twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
		twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
		pub_twist.publish(twist)

    		termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


