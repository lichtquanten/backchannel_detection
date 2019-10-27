#!/usr/bin/env python
from conditional import conditional
import csv
import rospy
import random
from rospywrapper import BagSource, TopicSource, BagSink, TopicSink

from bundler.msg import Bundle
from model.msg import Backchannel
from std_msgs.msg import Header

def main():
    # Get parameters
    bundle_topic = rospy.get_param('~bundle_topic', '/bc/bundle')
    src_bag_path = rospy.get_param('~source_bag_path', None)
    backchannel_topic = rospy.get_param('~backchannel_topic', '/bc/backchannels')
    sink_bag_path = rospy.get_param('~sink_bag_path', None)

    # Instantiate source
    if src_bag_path:
        bundle_source = BagSource(src_bag_path, bundle_topic)
    else:
        bundle_source = TopicSource(bundle_topic, Bundle)

    # Instantiate sinks
    if sink_bag_path:
        bag = rosbag.Bag(sink_bag_path, 'w')
        backchannel_sink = BagSink(bag, backchannel_topic, Backchannel)
    else:
        bag = None
        backchannel_sink = TopicSink(backchannel_topic, Backchannel)

    with conditional(bag, bag):
        with bundle_source, backchannel_sink:
            for msg, t in bundle_source:
                if random.random() > 0.2:
                    backchannel_msg = Backchannel()
                    backchannel_msg.header.stamp = t
                    pids = [1,2,3]
                    to_pid = random.choice(pids)
                    pids.remove(to_pid)
                    from_pid = random.choice(pids)
                    backchannel_msg.to_pid = to_pid
                    backchannel_msg.from_pid = from_pid
                    backchannel_sink.put(backchannel_msg, t)

if __name__ == '__main__':
    rospy.init_node('recognizer')
    main()
