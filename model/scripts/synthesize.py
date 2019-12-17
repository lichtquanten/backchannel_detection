#!/usr/bin/env python
from conditional import conditional
import rospy
from rospywrapper import BagSource, TopicSource, BagSink, TopicSink

from model.msg import Backchannel, BackchannelProbability

from model import bundle_msg_to_dict, create_pairwise_rows

class Predictor(object):
    def __init__(self, actor_pid, receiver_pid, threshold=0.5):
        self.actor_pid = actor_pid
        self.receiver_pid = receiver_pid
        self.threshold = threshold
        self._bc_starts = []
        self._current_bc_start = None
        self._counter = 0

    def __iter__(self):
        return self

    def next(self):
        if self._bc_starts:
            return self._bc_starts.pop(0)
        raise StopIteration

    def put(self, probability, t):
        if probability > self.threshold:
            if self._current_bc_start is None:
                self._current_bc_start = t
            self._counter += 1
        else:
            if self._current_bc_start is not None:
                if self._counter > 3:
                    self._bc_starts.append(self._current_bc_start)
                self._current_bc_start = None
                self._counter = 0

def main():
    # Get parameters
    probability_topic = rospy.get_param('~probability_topic', '/bc/probabilities')
    backchannel_topic = rospy.get_param('~backchannel_topic', '/bc/backchannels')
    threshold = rospy.get_param('~threshold', 0.9)
    src_bag_path = rospy.get_param('~source_bag_path', None)
    sink_bag_path = rospy.get_param('~sink_bag_path', None)

    # Instantiate source
    if src_bag_path:
        probability_source = BagSource(src_bag_path, probability_topic)
    else:
        probability_source = TopicSource(probability_topic, BackchannelProbability)

    # Instatiate bag, if  necessary
    if sink_bag_path:
        bag = rosbag.Bag(sink_bag_path, 'w')
        bc_sink = BagSink(bag, backchannel_topic, Backchannel)
    else:
        bag = None
        bc_sink = TopicSink(backchannel_topic, Backchannel)

    predictors = {1: {}, 2: {}, 3: {}}
    for actor_pid in [1,2,3]:
        for receiver_pid in [1,2,3]:
            if actor_pid == receiver_pid:
                continue
            predictors[actor_pid][receiver_pid] = Predictor(actor_pid, receiver_pid, threshold)


    with conditional(bag, bag):
        with probability_source, bc_sink:
            for msg, t in probability_source:
                predictors[msg.from_pid][msg.to_pid].put(msg.probability, t)
                for bc_t in predictors[msg.from_pid][msg.to_pid]:
                    new_msg = Backchannel()
                    new_msg.header.stamp = bc_t
                    new_msg.from_pid = msg.from_pid
                    new_msg.to_pid = msg.to_pid
                    bc_sink.put(new_msg, bc_t)

if __name__ == '__main__':
    rospy.init_node('synthesize')
    main()
