import aubio
from grouper import BlockArrLike, Neighborhood
import numpy as np
import webrtcvad

class Energy():
    def __init__(self):
        return

    @staticmethod
    def calculate(audio):
        # Convert to float [-1,1]
        audio = audio / float(np.iinfo(audio.dtype).max)
        audio = audio.astype(np.float32)
        return np.mean(np.square(audio))

# This is done
# Outputs the pitch, confidence corresponding `hop_size` blocks
class Pitch():
    def __init__(self, sample_rate, buffer_size=1024, hop_size=512):
        self._pitch_detector = aubio.pitch('yin', buffer_size, hop_size, sample_rate)
        self._pitch_detector.set_unit("Hz")

    def calculate(self, audio):
        # Convert to float [-1,1]
        audio = audio / float(np.iinfo(audio.dtype).max)
        audio = audio.astype(np.float32)
        pitch =  self._pitch_detector(audio)[0]
        confidence = self._pitch_detector.get_confidence()
        return (pitch, confidence)

class Is_Speech():
    def __init__(self, sample_rate, aggressiveness=1, ratio=.8,
        neighborhood_size=10):
        self._sample_rate = sample_rate
        self._vad = webrtcvad.Vad(aggressiveness)
        def is_valid(nbhd):
            return sum(nbhd) / float(len(nbhd)) > ratio
        self._nbhds = Neighborhood(is_valid, neighborhood_size)

    def __iter__(self):
        return self

    def next(self):
        return next(self._nbhds)

    def put(self, audio, start, end):
        audio = str(bytearray(audio))
        # is_speech = self._vad.is_speech(audio, self._sample_rate)
        is_speech = self._vad.is_speech(audio, 48000)
        self._nbhds.put(is_speech, start, end)

class Relative():
    def __init__(self, length, interval=1):
        self._length = length
        self._indices = np.array([])
        self._sorted = np.array([])
        self._interval = interval
        self._wait = interval
    def calculate(self, value):
        if not len(self._sorted):
            i = 0
        else:
            i = np.searchsorted(self._sorted, value)
            vfunc = np.vectorize(lambda x: x + (1 if x >= i else 0))
            self._indices = vfunc(self._indices)
        if not len(self._sorted):
            relative = 1
        else:
            relative = i /float(len(self._sorted))
        if self._wait > 0:
            self._wait -= 1
        if self._wait == 0 or len(self._sorted) < self._length:
            self._sorted = np.insert(self._sorted, i, value)
            self._indices = np.append(self._indices, i)
            self._wait = self._interval
            if len(self._sorted) > self._length:
                self._sorted = np.delete(self._sorted, self._indices[0])
                self._indices = self._indices[1:]
        return relative
