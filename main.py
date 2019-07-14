import csv
import numpy as np

from ros_speech2text.msg import AudioChunk
from rospywrapper import Sink, ROSBagSource, ROSTopicSource, ROSBagSink, ROSTopicSink
from std_msgs.msg import Float32

WINDOW_DURATION = .1

class CSVSink(Sink):
    def __init__(self, filename):
        self.filename = filename
        self._file = None
        self._writer = None

    def put(self, topic, data_class, data, t):
        row = [t] + [data[key] for key in data]
        self._writer.writerow(row)

    def __enter__(self):
        self._file = open(self.filename, 'w')
        self._writer = csv.writer(self.file)
        return self

    def __exit__(self, *exc):
        self._file.close()

def main():
    source = ROSBagSource(
        topic='/mic1/chunk',
        filename='bag.bag')
    # source = ROSTopicSource(
    #     topic='/mic1/chunk',
    #     data_class=AudioChunk
    # )

    sink = ROSBagSink(filename='out.bag')
    # sink = ROSTopicSink()
    # sink = CSVSink(filename='out.csv')

    with source, sink:
        for msg, t in source:
            data = np.fromstring(msg['data'], np.int16)
            volume = np.mean(np.abs(data))
            sink.put('/volume', Float32, np.mean(msg['data']), t)

if __name__ == '__main__':
    main()
