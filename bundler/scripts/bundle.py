#!/usr/bin/env python
import rospy

from conditional import conditional
from grouper import Combiner
import rosbag
from rospywrapper import TopicSource, BagSource, TopicSink, BagSink, Synchronizer

from audio_features.msg import AudioFeatures
from bundler.msg import Bundle
from std_msgs.msg import Header, Time, Int32

def main():
    # listens for messages on 2 topics, combines them all into a single, flattens them into a joint message by time
    rospy.init_node('bundler', anonymous=True)

	# Get parameters
    audio_features_P1_bag_path = rospy.get_param('~audio_features_P1_bag_path', None)
    audio_features_P2_bag_path = rospy.get_param('~audio_features_P2_bag_path', None)
    audio_features_P3_bag_path = rospy.get_param('~audio_features_P3_bag_path', None)

    audio_features_P1_topic = rospy.get_param('~audio_features_P1_topic', '/pid1/audio/features')
    audio_features_P2_topic = rospy.get_param('~audio_features_P2_topic', '/pid2/audio/features')
    audio_features_P3_topic = rospy.get_param('~audio_features_P3_topic', '/pid3/audio/features')

    start_time_topic = rospy.get_param('~start_time_topic', '/bc/start_time')
    start_time_bag_path = rospy.get_param('~start_time_bag_path', None)
    window_duration = rospy.get_param('~window_duration', None)
    window_duration_topic = rospy.get_param('~window_duration_topic', '/bc/window_duration')
    window_duration_bag_path = rospy.get_param('~window_duration_bag_path', None)

    output_topic = rospy.get_param('~output_topic', '/bc/bundle')
    sink_bag_path = rospy.get_param('~sink_bag_path', None)

    # Instantiate sources
    if audio_features_P1_bag_path:
        audio_features_P1_src = BagSource(audio_features_P1_bag_path, audio_features_P1_topic)
    else:
        audio_features_P1_src = TopicSource(audio_features_P1_topic, AudioFeatures)
    if audio_features_P2_bag_path:
        audio_features_P2_src = BagSource(audio_features_P2_bag_path, audio_features_P2_topic)
    else:
        audio_features_P2_src = TopicSource(audio_features_P2_topic, AudioFeatures)
    if audio_features_P3_bag_path:
        audio_features_P3_src = BagSource(audio_features_P3_bag_path, audio_features_P3_topic)
    else:
        audio_features_P3_src = TopicSource(audio_features_P3_topic, AudioFeatures)

    # Instantiate sink
    if sink_bag_path:
        sink_bag = rosbag.Bag(sink_bag_path, 'w')
        sink = BagSink(sink_bag, output_topic, Bundle)
    else:
        sink_bag = None
        sink = TopicSink(output_topic, Bundle)

    # Find start time, window_duration
    if start_time_bag_path:
        start_time_src = BagSource(start_time_bag_path, start_time_topic)
    else:
        start_time_src = TopicSource(start_time_topic, Time)
    with start_time_src:
        rospy.loginfo('Finding start time.')
        msg, _ = next(start_time_src)
        start_time = msg.data
        rospy.loginfo('Found start time.')

    # Set window duration
    if window_duration:
        window_duration = rospy.Duration(window_duration)
    else:
        if window_duration_bag_path:
            window_duration_src = BagSource(window_duration_bag_path, window_duration_topic)
        else:
            window_duration_src = TopicSource(window_duration_topic, Int32)
        with window_duration_src:
            rospy.loginfo('Finding window duration')
            msg, _ = next(window_duration_src)
            rospy.loginfo('Found window duration')
            window_duration = rospy.Duration(msg.data / 1000.)

    # Set-up combiner
    combiner = Combiner(start_time, window_duration, ['audio_features_P1', 'audio_features_P2', 'audio_features_P3'])

    with conditional(sink_bag, sink_bag):
        with audio_features_P1_src, audio_features_P2_src, audio_features_P3_src, sink:
            sync = Synchronizer([audio_features_P1_src, audio_features_P2_src, audio_features_P3_src])
            for (audio_features_P1, audio_features_P2, audio_features_P3) in sync:
                for (msg, t) in audio_features_P1:
                    combiner.put('audio_features_P1', msg, t, t + window_duration)
                for (msg, t) in audio_features_P2:
                    combiner.put('audio_features_P2', msg, t, t + window_duration)
                for (msg, t) in audio_features_P3:
                    combiner.put('audio_features_P3', msg, t, t + window_duration)
                for bundle, start, end in combiner:
                    h = Header()
                    h.stamp = start
                    bundle['header'] = h
                    sink.put(bundle, start)

if __name__ == '__main__':
	main()
