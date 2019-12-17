#!/usr/bin/env python
from conditional import conditional
import pandas as pd
import rospy
from rospywrapper import BagSource, TopicSource, BagSink, TopicSink
from sklearn.externals import joblib
from statsmodels.discrete.discrete_model import LogitResults

from bundler.msg import Bundle
from model.msg import BackchannelProbability

from model import bundle_msg_to_dict, create_pairwise_rows

def main():
    # Get model path
    model_path = rospy.get_param('~model_path', None)
    scaler_path = rospy.get_param('~scaler_path', None)

    # Get parameters
    bundle_topic = rospy.get_param('~bundle_topic', '/bc/bundle')
    probability_topic = rospy.get_param('~probability_topic', '/bc/probabilities')
    src_bag_path = rospy.get_param('~source_bag_path', None)
    sink_bag_path = rospy.get_param('~sink_bag_path', None)

    # Instantiate source
    if src_bag_path:
        bundle_source = BagSource(src_bag_path, bundle_topic)
    else:
        bundle_source = TopicSource(bundle_topic, Bundle)

    # Instatiate bag, if  necessary
    if sink_bag_path:
        bag = rosbag.Bag(sink_bag_path, 'w')
        probability_sink = BagSink(bag, probability_topic, BackchannelProbability)
    else:
        bag = None
        probability_sink = TopicSink(probability_topic, BackchannelProbability)

    # Load model
    log_model = LogitResults.load(model_path)

    # Load scaler
    scaler = joblib.load(scaler_path)

    with conditional(bag, bag):
        with bundle_source, probability_sink:
            for msg, t in bundle_source:
                row = bundle_msg_to_dict(msg)
                for key in row.keys():
                    if row[key] == 'True':
                        row[key] = 1
                    elif row[key] == 'False':
                        row[key] = 0
                pairwise_rows = create_pairwise_rows(row)
                for pairwise_row in pairwise_rows:
                    # Create new message
                    new_msg = BackchannelProbability()
                    actor, receiver = pairwise_row['actor'], pairwise_row['receiver']
                    new_msg.header.stamp = t
                    new_msg.from_pid = pairwise_row['actor']
                    new_msg.to_pid = pairwise_row['receiver']
                    new_msg.type = BackchannelProbability.TYPE_VERBAL

                    # Prepare data for model
                    del pairwise_row['actor']
                    del pairwise_row['receiver']
                    del pairwise_row['bystander']
                    df = pd.DataFrame.from_dict([pairwise_row])
                    df.reindex(sorted(df.columns), axis=1)
                    scaled = scaler.transform(df)

                    # Make prediction
                    new_msg.probability = log_model.predict(scaled)[0]

                    probability_sink.put(new_msg, t)

if __name__ == '__main__':
    rospy.init_node('run_model')
    main()
