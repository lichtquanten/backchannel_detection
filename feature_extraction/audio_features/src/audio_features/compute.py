from audio_io.utils import width_to_dtype
from features import *
from grouper import Aligner, Combiner, Counter, History, Window
import rospy
from scipy import stats

FEATURE_LIST = [
    'all_speech',
    'contains_speech',
    'speech_duration',
    'time_since_speech',
    'mean_relative_pitch',
    'low_pitch_duration',
    'mean_relative_pitch_slope',
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

    # Create a generator of audio that is speech
    is_speech_audio_aligner = Aligner(['is_speech', 'audio'])
    def speech():
        for bundle, start, end in is_speech_audio_aligner:
            if bundle['is_speech']:
                yield bundle['audio'], start, end

    # Pitch
    pitch_block_length = 512
    pitch_block_duration = get_duration(pitch_block_length / float(sample_rate))
    pitch_blocks = BlockArrLike(pitch_block_length, np.array([], dtype), np.append)
    pitch_calc = Pitch(sample_rate, buffer_size=(2 * pitch_block_length),
        hop_size=pitch_block_length)
    relative_pitch = Relative(1000)
    low_rel_pitch_counter = Counter(lambda x: x < 0.26)
    rel_pitch_history_short = History(1000)

    # Create window objects for each feature
    is_speech_windows = Window(start_time, window_duration)
    is_speech_counter_windows = Window(start_time, window_duration)
    is_not_speech_counter_windows = Window(start_time, window_duration)
    rel_pitch_windows = Window(start_time, window_duration)
    low_rel_pitch_counter_windows = Window(start_time, window_duration)
    rel_pitch_slope_windows = Window(start_time, window_duration)

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
            is_speech_audio_aligner.put('audio', block, start, end)
            # block_speech_align.put('audio', block, t, t + duration)

        for datum, start, end in is_speech:
            # Windowify is speech
            is_speech_windows.put(datum, start, end)
            # Send to counter
            is_speech_counter.put(datum, start, end)
            # Send to other counter
            is_not_speech_counter.put(datum, start, end)
            # # Send to aligner
            is_speech_audio_aligner.put('is_speech', datum, start, end)

        for count, start, end in is_speech_counter:
            is_speech_counter_windows.put(count, start, end)

        for count, start, end in is_not_speech_counter:
            is_not_speech_counter_windows.put(count, start, end)

        for audio, start, end in speech():
            pitch_blocks.put(audio, start, end)

        for audio, start, end in is_speech_audio_aligner:
            pitch_blocks.put(audio, start, end)

        for block, start, end in pitch_blocks:
            if (end - start) / pitch_block_duration > 1.3:
                continue
            pitch, confidence = pitch_calc.calculate(block)
            rel_pitch = relative_pitch.calculate(pitch)

            rel_pitch_windows.put(rel_pitch, start, end)
            low_rel_pitch_counter.put(rel_pitch, start ,end)
            rel_pitch_history_short.put(rel_pitch, start, end)

        for count, start, end in low_rel_pitch_counter:
            low_rel_pitch_counter_windows.put(count, start, end)

        for history, start, end in rel_pitch_history_short:
            dur = end - start
            dur = dur.to_sec()
            x = np.array(range(len(history))) / dur
            slope, _, _, _, _ = stats.linregress(x, history)
            rel_pitch_slope_windows.put(slope, start, end)

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
                start, end
            )

        for window, start, end in is_not_speech_counter_windows:
            combiner.put(
                'time_since_speech',
                is_speech_block_duration * max(window),
                start, end
            )

        for window, start, end in rel_pitch_windows:
            combiner.put(
                'mean_relative_pitch',
                np.mean(window), start, end
            )

        for window, start, end in low_rel_pitch_counter_windows:
            combiner.put(
                'low_pitch_duration',
                max(window) * pitch_block_duration,
                start, end
            )

        for window, start, end in rel_pitch_slope_windows:
            combiner.put(
                'mean_relative_pitch_slope',
                max(window), start, end
            )

        for bundle, start, end in combiner:
            put_features(bundle, start)
