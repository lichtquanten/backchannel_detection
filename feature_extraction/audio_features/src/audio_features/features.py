from abc import ABCMeta, abstractmethod
import aubio
from grouper import BlockArrLike
import numpy as np
import webrtcvad

class Feature(object):
    __metaclass__ = ABCMeta

    def __init__(self):
        return

    def __iter__(self):
        return self

    @abstractmethod
    def next(self):
        pass

    @abstractmethod
    def put(self, data, t):
        pass

# This is done
# Outputs the pitch, confidence corresponding `hop_size` blocks
class Pitch(Feature):
    def __init__(self, sample_rate, buffer_size=1024, hop_size=512):
        self._pitch_detector = aubio.pitch('yin', buffer_size, hop_size, sample_rate)
        self._pitch_detector.set_unit("Hz")
        self._blocks = BlockArrLike(hop_size, np.array([], np.float32), np.append)
        self._output_buffer = []

    def next(self):
        if not self._output_buffer:
            raise StopIteration
        return self._output_buffer.pop(0)

    def put(self, audio, start, end):
        # Convert to float [-1,1]
        audio = audio / float(np.iinfo(audio.dtype).max)
        audio = audio.astype(np.float32)
        self._blocks.put(audio, start, end)
        for block, b_start, b_end in self._blocks:
            pitch =  self._pitch_detector(block)[0]
            confidence = self._pitch_detector.get_confidence()
            self._output_buffer.append(((pitch, confidence), b_start, b_end))

# This is done
# Outputs True or false of blocks of length
# `_BLOCK_DURATION` * sample_rate
class Is_Speech(Feature):
    _BLOCK_DURATION = 0.02
    def __init__(self, sample_rate, dtype, aggressiveness=1):
        self._vad = webrtcvad.Vad(aggressiveness)
        self._sample_rate = sample_rate
        block_length = int(sample_rate * self._BLOCK_DURATION)
        self._blocks = BlockArrLike(block_length, np.array([], dtype), np.append)
        self._output_buffer = []

    def next(self):
        if not self._output_buffer:
            raise StopIteration
        return self._output_buffer.pop(0)

    def put(self, audio, start, end):
        self._blocks.put(audio, start, end)
        for block, b_start, b_end in self._blocks:
            block = str(bytearray(block))
            is_speech = self._vad.is_speech(block, self._sample_rate)
            self._output_buffer.append((is_speech, b_start, b_end))

# Keep a buffer of the last buffer_size samples. Compute the mean
# compare earliest value to that mean
# class Relative(Feature):
#     def __init__(self, dtype, buffer_size=1000):
#         self._full =
#
#     def put(self, value, start, end):
#         pass
