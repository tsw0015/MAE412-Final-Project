#!/usr/bin/env python3


import rospy
from geometry_msgs.msg import Transform
from fiducial_msgs.msg import FiducialTransformArray
from std_msgs.msg import Int32




def convert(data: FiducialTransformArray):
	global PUB1, PUB2
	
	if data.transforms:
		trans = data.transforms[0].transform
		id = data.transforms[0].fiducial_id
		id_msg = Int32()
		id_msg.data = id
		PUB1.publish(trans)
		PUB2.publish(id_msg)

	


def main():
	global RATE, SUB, PUB1, PUB2

	rospy.init_node('fiducial_message_convert')
	RATE = rospy.Rate(10) #hz

	PUB1 = rospy.Publisher('marker/transform', Transform, queue_size=1)
	PUB2 = rospy.Publisher('marker/ID', Int32, queue_size=1)
	SUB = rospy.Subscriber('fiducial_transforms', FiducialTransformArray, convert)

	rospy.spin()


if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass