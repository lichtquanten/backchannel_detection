#!/usr/bin/env python
import csv
import rospy
from rospywrapper import BagSource, TopicSource

from model import bundle_msg_to_dict, create_pairwise_rows

from bundler.msg import Bundle

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
            row = bundle_msg_to_dict(msg)
            for key in row.keys():
                if row[key] == 'True':
                    row[key] = 1
                elif row[key] == 'False':
                    row[key] = 0
            row['time'] = t.to_sec()
            pairwise_rows = create_pairwise_rows(row)
            if not writer:
                headers = pairwise_rows[0].keys()
                writer = csv.DictWriter(csv_file, headers)
                writer.writeheader()
            for pairwise_row in pairwise_rows:
                writer.writerow(pairwise_row)

if __name__ == '__main__':
    rospy.init_node('bundle_to_csv', anonymous=True)
    main()
