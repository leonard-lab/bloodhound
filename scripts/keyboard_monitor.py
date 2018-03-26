#!/usr/bin/env python
import roslib; roslib.load_manifest('teleop_twist_keyboard')
import rospy

from std_msgs.msg import String

import sys, select, termios, tty

msg = """
Reading from the keyboard  and Publishing to String

CTRL-C to quit
"""

def getKey():
	tty.setraw(sys.stdin.fileno())
	select.select([sys.stdin], [], [], 0)
	key = sys.stdin.read(1)
	termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
	return key


def vels(speed,turn):
	return "currently:\tspeed %s\tturn %s " % (speed,turn)

if __name__=="__main__":
    	settings = termios.tcgetattr(sys.stdin)
	
	pub = rospy.Publisher('keyboard', String, queue_size = 1)
	rospy.init_node('keyboard_monitor')


	try:
		print msg
		while(1):
			key = getKey()
			PubChar = key
			print PubChar
			if (key == '\x03'):
				break

			pub.publish(PubChar)

	except:
		print e

	finally:
		pub.publish('a')

    		termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


