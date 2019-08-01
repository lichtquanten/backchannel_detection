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
    out['mean'] = audio.mean
    out['meme'] = audio.meme
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
        headers = ['time', 'mean', 'meme']
        writer = csv.DictWriter(csv_file, headers)
        writer.writeheader()
        for msg, t in bundle_source:
            row = flatten_bundle(msg)
            row['time'] = t
            writer.writerow(row)

if __name__ == '__main__':
    rospy.init_node('bundle_to_csv', anonymous=True)
    main()
