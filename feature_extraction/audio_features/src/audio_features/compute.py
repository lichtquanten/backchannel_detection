from audio_io.utils import width_to_dtype
from features import *
from grouper import Aligner, Combiner, Counter, Window
import rospy

# FEATURE_LIST = [
#     'all_speech',
#     'contains_speech',
#     'mean_relative_pitch'
#     'mean_pitch'
#     'mean_energy'
# ]

FEATURE_LIST = [
    'all_speech',
    'contains_speech',
    'speech_duration',
    'time_since_speech'
]

def compute_audio_features(
    audio,
    put_features,
    start_time,
    window_duration,
    get_duration,
    sample_rate,
    sample_width,
    num_channels,
    is_bigendian
):
    """
    audio: generator of str, time

    put_features: function that accepts (bundle, t)
    """
    dtype = width_to_dtype(sample_width)

    # Speech
    is_speech_block_length = int(sample_rate * 0.02)
    is_speech_block_length = int(48000 * 0.02)
    is_speech_block_duration = get_duration(is_speech_block_length / float(sample_rate))
    speech_blocks = BlockArrLike(is_speech_block_length * num_channels, np.array([], dtype), np.append)
    is_speech = Is_Speech(sample_rate)

    # Speech counter
    is_speech_counter = Counter(lambda x: x)
    is_not_speech_counter = Counter(lambda x: not x)

    # Create window objects for each feature
    is_speech_windows = Window(start_time, window_duration)
    is_speech_counter_windows = Window(start_time, window_duration)
    is_not_speech_counter_windows = Window(start_time, window_duration)

    # Create combiner for all features
    combiner = Combiner(start_time, window_duration, FEATURE_LIST)

    for data, t in audio:
        # Convert data to numpy array
        data_np = np.fromstring(data, dtype)

        # Compute duration (seconds) of message
        duration = get_duration(
            float(len(data)) / num_channels / sample_width / sample_rate)

        # Divide audio into 20ms blocks
        speech_blocks.put(data_np, t, t + duration)

        # Determine if block contains speech
        for block, start, end in speech_blocks:
            is_speech.put(block, start, end)
            # block_speech_align.put('audio', block, t, t + duration)

        for datum, start, end in is_speech:
            # Windowify is speech
            is_speech_windows.put(datum, start, end)
            # Send to counter
            is_speech_counter.put(datum, start, end)
            # Send to other counter
            is_not_speech_counter.put(datum, start, end)
            # # Send to aligner
            # block_speech_align.put('is_speech', datum, start, end)

        for count, start, end in is_speech_counter:
            is_speech_counter_windows.put(count, start, end)

        for count, start, end in is_not_speech_counter:
            is_not_speech_counter_windows.put(count, start, end)

        for window, start, end in is_speech_windows:
            combiner.put(
                'contains_speech',
                any(window), start, end
            )
            combiner.put(
                'all_speech',
                all(window), start, end
            )

        for window, start, end in is_speech_counter_windows:
            combiner.put(
                'speech_duration',
                is_speech_block_duration * max(window),
                start,
                end
            )

        for window, start, end in is_not_speech_counter_windows:
            combiner.put(
                'time_since_speech',
                is_speech_block_duration * max(window),
                start,
                end
            )

        for bundle, start, end in combiner:
            put_features(bundle, start)
