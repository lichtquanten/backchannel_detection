"""Remove unneeded messages. Convert messages to desire data_class. Add start time."""
import re
import rosbag
import rospy
import sys
from tf.transformations import quaternion_from_euler

from audio_io_msgs.msg import AudioData
from geometry_msgs.msg import Point, Pose, Quaternion, Vector3
from opencv_apps.msg import Point2D
from openface2_wrapper.msg import FaceFeaturesArray, FaceFeatures
from std_msgs.msg import Time


face_topics = [
	'/camera1/face_info',
	'/camera2/face_info',
	'/camera3/face_info',
]

audio_topics = [
	'/pid1/chunk',
	'/pid2/chunk',
	'/pid3/chunk',
]

def main(argv):
    in_bag = rosbag.Bag(argv[0], 'r')
    out_bag = rosbag.Bag(argv[1], 'w')
    with in_bag, out_bag:
    	start_time = Time(rospy.Time(in_bag.get_start_time()))
    	out_bag.write('/bc/start_time', start_time)
    	for topic, msg, t in in_bag.read_messages(topics=face_topics + audio_topics):
    		pid = re.findall(r'\d', topic)[0]
    		if topic in face_topics:
    			if not msg.landmarks:
    				continue

    			ff_msg = FaceFeatures()
    			# Success
    			ff_msg.success = True

    			# Landmarks
    			# 2D landmarks not provided

    			# Landmarks 3D
    			ff_msg.landmarks_3D = []
    			for landmark in msg.landmarks:
    				ff_msg.landmarks_3D.append(Vector3(landmark.x, landmark.y, landmark.z))

    			# Landmarks visibilities (unused)
    			ff_msg.landmarks_visibilities = []

    			# Gaze angle
    			ff_msg.gaze_angle = Point2D(msg.gazeAngle.x, msg.gazeAngle.y)

    			# Gaze left
    			direction = msg.gazeDirection0
    			ff_msg.gaze_left = Vector3(direction.x, direction.y, direction.z)

    			# Gaze right
    			direction = msg.gazeDirection1
    			ff_msg.gaze_right = Vector3(direction.x, direction.y, direction.z)

    			# Head pose
    			pose = msg.head_pose
    			point = Point(pose.x, pose.y, pose.z)
    			pitch, yaw, roll = pose.rot_x, pose.rot_y, pose.rot_z
    			x, y, z, w = quaternion_from_euler(roll, pitch, yaw)
    			quaternion = Quaternion(x, y, z, w)
    			ff_msg.head_pose = Pose(point, quaternion)

    			# au name (unused)
    			ff_msg.au_name = ''

    			# au intensity (unused)
    			ff_msg.au_intensity = []

    			array = FaceFeaturesArray()
    			array.header = msg.header
    			array.features = [ff_msg]

    			out_bag.write('/pid' + pid + '/face_features', array, t)
    		elif topic in audio_topics:
    			ad_msg = AudioData()
    			ad_msg.header.stamp = msg.time
    			ad_msg.data = msg.chunk
    			ad_msg.sample_rate = 16000
    			ad_msg.num_channels = 1
    			ad_msg.sample_width = 2
    			ad_msg.is_bigendian = True

    			out_bag.write('/pid' + pid + '/audio/data', ad_msg, t)

if __name__ == '__main__':
    if len(sys.argv) != 3:
        print "Error - Incorrect number of arguments"
        print "Usage: python prune.py [SOURCE] [DEST]"
        exit(1)
    main(sys.argv[1:])
