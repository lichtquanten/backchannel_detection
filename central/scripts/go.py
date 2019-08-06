#!/usr/bin/env python
import rospy
from std_msgs.msg import Time, Int32

def main():
	rospy.init_node('central')

	# Start time
	tp = rospy.Publisher('~start_time', Time, queue_size=10, latch=True)
	tp.publish(rospy.Time.now())

	# Window duration
	window_duration = rospy.get_param('~duration', 100)
	wp = rospy.Publisher('~window_duration', Int32, queue_size=10, latch=True)
	wp.publish(window_duration)

	rospy.loginfo("Ready.")
	rospy.spin()

if __name__ == '__main__':
	main()
