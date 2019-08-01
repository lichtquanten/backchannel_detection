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
    audio_topic = rospy.get_param('~audio_topic', '/bc/audio_features')
    audio_bag_path = rospy.get_param('~audio_bag_path', None)
    nod_topic = rospy.get_param('~nod_topic', '/bc/nod_features')
    nod_bag_path = rospy.get_param('~nod_bag_path', None)
    start_time_topic = rospy.get_param('~start_time_topic', '/bc/start_time')
    start_time_bag_path = rospy.get_param('~start_time_bag_path', None)
    window_duration_topic = rospy.get_param('~window_duration_topic', '/bc/window_duration')
    window_duration_bag_path = rospy.get_param('~window_duration_bag_path', None)
    output_topic = rospy.get_param('~output_topic', '/bc/bundle')
    sink_bag_path = rospy.get_param('~sink_bag_path', None)

    # Instantiate sources
    if audio_bag_path:
        audio_src = BagSource(audio_bag_path, audio_topic)
    else:
        audio_src = TopicSource(audio_topic, AudioFeatures)
    if nod_bag_path:
        # nod_src = BagSource(nod_bag_path, nod_topic)
        nod_src = BagSource(nod_bag_path, nod_topic)
    else:
        # nod_src = TopicSource(nod_topic, NodFeature)
        nod_src = TopicSource(nod_topic, AudioFeatures)
    if start_time_bag_path:
        start_time_src = BagSource(start_time_bag_path, start_time_topic)
    else:
        start_time_src = TopicSource(start_time_topic, Time)
    if window_duration_bag_path:
        window_duration_src = BagSource(window_duration_bag_path, window_duration_topic)
    else:
        window_duration_src = TopicSource(window_duration_topic, Int32)

    # Instantiate sink
    if sink_bag_path:
        sink_bag = rosbag.Bag(sink_bag_path, 'w')
        sink = BagSink(sink_bag, output_topic, Bundle)
    else:
        sink_bag = None
        sink = TopicSink(output_topic, Bundle)

    # Set-up combiner
    rospy.loginfo('Finding start time, window duration.')
    with start_time_src:
        msg, _ = next(start_time_src)
        start_time = msg.data
    with window_duration_src:
        msg, _ = next(window_duration_src)
        window_duration = rospy.Duration(msg.data / 1000.)
    rospy.loginfo('Found start time, window duration.')

    combiner = Combiner(start_time, window_duration, ['audio', 'nod'])

    with conditional(sink_bag, sink_bag):
        with audio_src, nod_src, sink:
            sync = Synchronizer([audio_src, nod_src])
            for (audio, nod) in sync:
                for (msg, t) in audio:
                    combiner.put('audio', msg, t, t + window_duration)
                for (msg, t) in nod:
                    combiner.put('nod', msg, t, t + window_duration)
                for bundle, start, end in combiner:
                    h = Header()
                    h.stamp = start
                    bundle['header'] = h
                    sink.put(bundle, start)

if __name__ == '__main__':
	main()
