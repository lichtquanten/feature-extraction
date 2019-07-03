from abc import ABCMeta, abstractmethod
import pyaudio
import rosbag
from rospy_msg_converter import convert_ros_message_to_dictionary
import Queue
import time
import rospy

class Source(object):
    __metaclass__ = ABCMeta
    def __init__(self):
        pass

    @abstractmethod
    def __iter__(self):
        pass

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        pass


class PyAudio(Source):
    def __init__(self, preprocess, channels, rate, frames_per_buffer):
        self.preprocess = preprocess
        self.channels = channels
        self.rate = rate
        self.frames_per_buffer = frames_per_buffer
        self._p = None
        self._stream = None

        self.start_time = 0
        self.chunk_duration = frames_per_buffer / float(self.rate)

    def __iter__(self):
        return self

    def next(self):
        out = (
            self.preprocess(self._stream.read(self.frames_per_buffer)),
            self.start_time,
            self.start_time + self.chunk_duration)
        self.start_time += self.chunk_duration
        return out

    def __enter__(self):
        self._p = pyaudio.PyAudio()
        self._stream = self._p.open(format=pyaudio.paInt16,
                                    channels=self.channels,
                                    rate=self.rate,
                                    input=True,
                                    frames_per_buffer=self.frames_per_buffer)
        return self

    def __exit__(self, *exc):
        self._stream.stop_stream()
        self._stream.close()
        self._p.terminate()

class ROSbag(Source):
    def __init__(self, preprocess, filename, topic):
        self.preprocess = preprocess
        self._filename = filename
        self._topic = topic
        self.start_time = None
        self.rate = 25
        self.chunk_duration = 1 / float(self.rate)

    def __iter__(self):
        return self

    def next(self):
        topic, msg, t = next(self._messages)
        msg = convert_ros_message_to_dictionary(msg)
        start = msg['header']['stamp']['secs'] + msg['header']['stamp']['nsecs'] * (10 ** -9)
        return self.preprocess(msg), start, start + self.chunk_duration

    def __enter__(self):
        self._bag = rosbag.Bag(self._filename, 'r')
        self._bag.__enter__()
        self.start_time = self._bag.get_start_time()
        self._messages = self._bag.read_messages(
            connection_filter=lambda topic, *args: topic == self._topic)
        return self

    def __exit__(self, *exc):
        self._bag.__exit__(None, None, None)

class ROSlive(Source):
    def __init__(self, preprocess, topic, msg_type, rate, start_time=None):
        self.preprocess = preprocess
        self._topic = topic
        self._type = msg_type
        self.rate = rate
        self._buffer = Queue.Queue()
        if start_time is None:
            t = rospy.Time.now()
            self.start_time = t.secs + t.nsecs / (10 ** 9)
        else:
            self.start_time = start_time

    def __iter__(self):
        while True:
            try:
                yield self._buffer.get(block=False)
            except Queue.Empty:
                time.sleep(0.001)

    def _callback(self, msg):
        msg = convert_ros_message_to_dictionary(msg)
        start_time = msg['time']['secs'] + msg['time']['nsecs'] / (10 ** 9)
        data = self.preprocess(msg)
        self._buffer.put((data, start_time, start_time + len(data) / float(self.rate)))

    def __enter__(self):
        self._subscriber = rospy.Subscriber(
            self._topic, self._type, self._callback)
        return self

    def __exit__(self, *exc):
        self._subscriber.unregister()
