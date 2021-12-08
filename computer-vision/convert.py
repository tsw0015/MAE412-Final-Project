#!usr/bin/env/ python3

import rospy
from geometry_msgs.msg import Transform
from fiducial_msgs.msg import FiducialTransform
from std_msgs.msg import Int32


def convert(data:FiducialTransform):
	global PUB, PUB2
	
	trans = data.transform
	id = data.fiducial_id
	id_msg = Int32()
	id_msg.data = id
	PUB.publish(trans)
	PUB2.publish(id_msg)

	


def main():
	global RATE, RATE_HZ, SUB, PUB, PUB2

	rospy.init_node('fiducial_message_convert')
	RATE = rospy.Rate(RATE_HZ)

	PUB = rospy.Publisher('marker/pose', Transform, queue_size=1)
	PUB2 = rospy.Publisher('marker/ID', Int32, queue_size=1)
	SUB = rospy.Subscribe('fiducial_transforms', FiducialTransform, convert)

	rospy.spin()


if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass