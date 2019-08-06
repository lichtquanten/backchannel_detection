#!/usr/bin/env python
from audio_features import compute_audio_features
from audio_io.utils import width_to_dtype
from conditional import conditional
import grouper
import numpy as np
import rosbag
import rospy
from rospywrapper import BagSource, TopicSource, TopicSink, BagSink

from audio_io_msgs.msg import AudioData
from audio_features.msg import AudioFeatures
from std_msgs.msg import Header, Time, Int32

def main():
	# Get topic parameters
	audio_topic = rospy.get_param('~audio_topic')
	start_time_topic = rospy.get_param('~start_time_topic', '/bc/start_time')
	window_duration_topic = rospy.get_param('~window_duration_topic', '/bc/window_duration')
	features_topic = rospy.get_param('~features_topic', '/bc/audio_features')

	# Get source, sink parameters
	src_bag_path = rospy.get_param('~source_bag_path', None)
	sink_bag_path = rospy.get_param('~sink_bag_path', None)

	# Instantiate sources
	if src_bag_path:
		audio_src = BagSource(src_bag_path, audio_topic)
		start_time_src = BagSource(src_bag_path, start_time_topic)
		window_duration_src = BagSource(src_bag_path, window_duration_topic)
	else:
		audio_src = TopicSource(audio_topic, AudioData)
		start_time_src = TopicSource(start_time_topic, Time)
		window_duration_src = TopicSource(window_duration_topic, Int32)

	# Instantiate sinks
	if sink_bag_path:
		bag = rosbag.Bag(sink_bag_path, 'w')
		features_sink = BagSink(bag, features_topic, AudioFeatures)
	else:
		bag = None
		features_sink = TopicSink(features_topic, AudioFeatures)

	# Get the start time
	rospy.loginfo('Finding start time, window duration.')
	with start_time_src:
		msg, _ = next(start_time_src)
		start_time = msg.data

	# Get the window duration
	with window_duration_src:
		msg, _ = next(window_duration_src)
		window_duration = rospy.Duration(msg.data / 1000.)
	rospy.loginfo('Found start time, window duration.')

	with conditional(bag, bag):
		with audio_source, features_sink:
			compute_audio_features(audio_source, features_sink, start_time, window_duration)

if __name__ == '__main__':
	rospy.init_node('audio_features', anonymous=True)
	main()
