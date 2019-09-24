import aubio
from grouper import BlockArrLike, Neighborhood
import numpy as np
import webrtcvad

class Energy():
    def __init__(self):
        return

    def calculate(audio):
        return 5

# This is done
# Outputs the pitch, confidence corresponding `hop_size` blocks
class Pitch():
    def __init__(self, sample_rate, buffer_size=1024, hop_size=512):
        self._pitch_detector = aubio.pitch('yin', buffer_size, hop_size, sample_rate)
        self._pitch_detector.set_unit("Hz")

    def calculate(audio):
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
#
# class Rolling():
#     """Provides a list of the last `length` data input before each datum.
#
#     Consider a history with length 100. After inputting 101 data, a list of
#     100 data would be produced, with its timestamp given by the timestamp of
#     the 101st datum.
#     """
#     def __init__(self, length):
#         """
#         Parameters
#         ----------
#         length : int
#             The number of data to be included in each history.
#         """
#         self._buffer = np.array([])
#         self._length = length
#         self._data = []
#         self._times = []
#         self._output_buffer = []
#
#     def next(self):
#         """
#         Returns
#         -------
#         list
#             The earliest `length` list of data that have not yet been returned.
#         any
#             The start time of the first received datum after the last datum
#             in the list.
#         any
#             The end time of the first received datum after the last datum in
#             the list.
#
#         Raises
#         ------
#         StopIteration
#             When there are not enough data in the buffer to create a history of
#             length `length`.
#         """
#         if len(self._data) <= self._length:
#             raise StopIteration
#         start, end = self._times[self._length]
#         out = self._data[:self._length]
#         del self._data[0]
#         del self._times[0]
#         return out, start, end
#
#     def put(self, datum, start_time, end_time):
#         """Add `datum` to the buffer.
#
#         `datum` assumed to be after, by timestamp, all previously put data.
#
#         Parameters
#         ----------
#         datum: any
#             Anything.
#         start_time : any
#             The start time associated with `datum`.
#         end_time : any
#             The end time associated with `datum`.
#         """
#         self._data.append(datum)
#         self._times.append((start_time, end_time))
