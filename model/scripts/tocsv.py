#!/usr/bin/env python
from conditional import conditional
import csv
import rospy
from rospywrapper import BagSource, TopicSource

from bundler.msg import Bundle

def add_pid(msg, pid):
    out = {}
    for key in msg:
        out[key + '_' + pid] = msg[key]
    return out

def msg_to_dict(msg):
    dictionary = {}
    message_fields = zip(msg.__slots__, msg._slot_types)
    for field_name, field_type in message_fields:
        if field_name == 'header':
            continue
        field_value = getattr(msg, field_name)
        dictionary[field_name] = str(field_value)

    return dictionary


def bundle_to_dict(bundle):
    # Convert audio messages to dictionaries
    audio_p1 = msg_to_dict(bundle.audio_features_P1)
    audio_p2 = msg_to_dict(bundle.audio_features_P2)
    audio_p3 = msg_to_dict(bundle.audio_features_P3)
    # Add pid
    audio_p1 = add_pid(audio_p1, 'P1')
    audio_p2 = add_pid(audio_p2, 'P2')
    audio_p3 = add_pid(audio_p3, 'P3')
    out = {}
    out.update(audio_p1)
    out.update(audio_p2)
    out.update(audio_p3)
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
        writer = None
        for msg, t in bundle_source:
            row = bundle_to_dict(msg)
            row['time'] = t
            if not writer:
                headers = row.keys()
                writer = csv.DictWriter(csv_file, headers)
                writer.writeheader()
            writer.writerow(row)

if __name__ == '__main__':
    rospy.init_node('bundle_to_csv', anonymous=True)
    main()
