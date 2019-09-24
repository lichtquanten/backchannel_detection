#!/usr/bin/env python
from conditional import conditional
import csv
import rospy
from rospywrapper import BagSource, TopicSource

from bundler.msg import Bundle

def flatten_bundle(bundle):
    out = {}
    audio = bundle.audio
    nod = bundle.nod
    out['all_speech'] = audio.all_speech
    out['contains_speech'] = audio.contains_speech
    out['speech_duration'] = audio.speech_duration
    out['time_since_speech'] = audio.time_since_speech
    out['mean_relative_pitch'] = audio.mean_relative_pitch
    out['low_pitch_duration'] = audio.low_pitch_duration
    out['mean_relative_pitch_slope'] = audio.mean_relative_pitch_slope
    return out

def main():
    # Get parameters
    bundle_topic = rospy.get_param('~bundle_topic', '/bc/bundle')
    src_bag_path = rospy.get_param('~source_bag_path', None)
    csv_path = rospy.get_param('~csv_path')

    # Instantiate source
    if src_bag_path:
        bundle_source = BagSource(src_bag_path, bundle_topic)
    else:
        bundle_source = TopicSource(bundle_topic, Bundle)

    with bundle_source, open(csv_path, 'w') as csv_file:
        headers = ['time', 'all_speech', 'contains_speech', 'speech_duration', 'time_since_speech', 'mean_relative_pitch', 'low_pitch_duration', 'mean_relative_pitch_slope']
        writer = csv.DictWriter(csv_file, headers)
        writer.writeheader()
        for msg, t in bundle_source:
            row = flatten_bundle(msg)
            row['time'] = t
            writer.writerow(row)

if __name__ == '__main__':
    rospy.init_node('bundle_to_csv', anonymous=True)
    main()
