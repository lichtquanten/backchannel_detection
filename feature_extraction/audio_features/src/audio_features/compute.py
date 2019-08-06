from audio_io.utils import width_to_dtype
from features import *
import webrtc

from audio_features.msg import AudioFeatures

FEATURE_LIST = [
    'mean',
    'meme'
]

def compute_audio_features(audio_source, start_time, window_duration):
	combiner = grouper.Combiner(start_time, window_duration, FEATURE_LIST)
	windows = grouper.Window(start_time, window_duration)

    first_pass = True
    for msg, t in audio_src:
        # Create the feature objects, if necessary
        if first_pass:
            first_pass = False
            dtype = width_to_dtype(msg.sample_width)
            pitches = Pitch(msg.sample_rate)
            relative_pitches = Relative(10000)
            is_speeches = Is_Speech(msg.sample_rate)

        # Pre-process the data
        data_np = np.fromstring(msg.data, dtype)

        # Compute duration (s) of message
        duration = rospy.Duration(
            float(len(msg.data)) / msg.num_channels / msg.sample_width / msg.sample_rate)

        # Add the data to feature objects
        pitches.put(data_np, t, t + duration)
        is_speeches.put(data_np, t, t + duration)

        for pitch, start, end in pitches:
            print pitch
            relative_pitches.put(pitch, start, end)

        for relative_pitch, start, end in relative_pitches:
            relative_pitches.put(relative_pitch, start, end)

        for is_speech, start, end in is_speeches:
            print is_speech

        # Add to combiner
        for pitch, start, end in mean_pitches:
            combiner.put('pitch', pitch, start, end)
        # 
        # for mean_relative_pitch, start, end in mean_relative_pitches:
        #     combiner.put('relative_pitch', mean_relative_pitch, start, end)

        for bundle, start, end in combiner:
            print bundle
