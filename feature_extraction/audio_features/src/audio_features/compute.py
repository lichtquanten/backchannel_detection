from audio_io.utils import width_to_dtype
from features import *
from grouper import Combiner, Window
import rospy

from audio_io_msgs.msg import AudioData
from audio_features.msg import AudioFeatures
from std_msgs.msg import Header, Time, Int32

FEATURE_LIST = [
    'mean',
    'meme'
]

def compute_audio_features(audio_source, features_sink, start_time, window_duration):
    combiner = Combiner(start_time, window_duration, FEATURE_LIST)
    windows = Window(start_time, window_duration)

    first_pass = True
    for msg, t in audio_source:
        # Create the feature objects, if necessary
        if first_pass:
            first_pass = False
            dtype = width_to_dtype(msg.sample_width)

            pitches = Pitch(msg.sample_rate, confidence_threshold=0.6)
            relative_pitches = Relative(10000)
            is_speeches = Is_Speech(msg.sample_rate, dtype)
            def is_valid(nbhd):
                return (float(sum(nbhd)) / len(nbhd)) / .9
            speech_nbhd = Neighborhood(is_valid, 100)

            # Continuous speech counter
            cont_speech_ct = Counter(is_valid=lambda is_speech : is_speech)

        # Pre-process the data
        data_np = np.fromstring(msg.data, dtype)

        # Compute duration (s) of message
        duration = rospy.Duration(
            float(len(msg.data)) / msg.num_channels / msg.sample_width / msg.sample_rate)

        # Add the data to feature objects
        pitches.put(data_np, t, t + duration)
        is_speeches.put(data_np, t, t + duration)

        for (pitch, confidence), start, end in pitches:
            pitch
            # relative_pitches.put(pitch, start, end)

        for is_speech, start, end in is_speeches:
            is_speeches_boosted.put(is_speech, start, end)
            cont_speech_duration.put(is_speech, start, end)
        for is_speech, start, end in is_speeches_boosted:
            continuous_speech_duration.put(is_speech, start, end)

        # Prepare for the combiner

        # Add to combiner
        # for mean_pitch, start, end in mean_pitches:
        #     combiner.put('mean_pitch', pitch, start, end)
        #
        # for mean_relative_pitch, start, end in mean_relative_pitches:
        #     combiner.put('relative_pitch', mean_relative_pitch, start, end)

        for bundle, start, end in combiner:
            print bundle
