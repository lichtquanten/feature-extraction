from abc import ABCMeta, abstractmethod
import collections
import buffer
import numpy as np

class Analyzer:
    __metaclass__ = ABCMeta

    def __init__(self, callbacks):
        pass

    @abstractmethod
    def put(self, data, start_time, end_time):
        pass

class SimpleAnalyzer:
    def __init__(self, analyze, callbacks):
        self._analyze = analyze
        self._callbacks = callbacks

    def put(self, data, start_time, end_time):
        val = self._analyze(data)
        for cb in self._callbacks:
            cb(val, start_time, end_time)

# calls analyze on every block
class BlockAnalyzer(Analyzer):
    """Breaks input data into blocks of size `block_size`.
    Calls the analyze function on each block.
    """
    def __init__(self, block_buffer, analyze, callbacks):
        self._buffer = buffer.BlockBufferTimeWrapper(block_buffer)
        self._analyze = analyze
        self._callbacks = callbacks

    def put(self, data, start_time, end_time):
        """Add data to the buffer. Process available blocks."""
        self._buffer.put(data, start_time, end_time)

        for (block, blk_start, blk_end) in self._buffer:
            val = self._analyze(block)
            for cb in self._callbacks:
                cb(val, blk_start, blk_end)


class NeighborAnalyzer(Analyzer):

    def __init__(self, analyze, length, callbacks):
        self._analyze = analyze
        self._length = length
        self._callbacks = callbacks
        self._blocks = None
        self.reset()

    def reset(self):
        self._blocks = collections.deque(maxlen=self._length)

    def put(self, data, start_time, end_time):
        """
        Assumes that data is received in order, contiguously
        """
        self._blocks.append({
            'block': data,
            'start_time': start_time,
            'end_time': end_time,
            'handled': False
        })
        if len(self._blocks) >= self._length:
            valid = self._analyze([x['block'] for x in self._blocks])
            if not valid:
                x = self._blocks.popleft()
                if not x['handled']:
                    for cb in self._callbacks:
                        cb(False, x['start_time'], x['end_time'])
            else:
                for i in range(len(self._blocks)):
                    if not self._blocks[i]['handled']:
                        for cb in self._callbacks:
                            cb(True, self._blocks[i]['start_time'],
                               self._blocks[i]['end_time'])
                        self._blocks[i]['handled'] = True

class WindowAnalyzer(Analyzer):
    """Groups all input data into time windows and analyzes.

    Windows are of duration window_duration. First window starts
    at start_time. Any data from earlier is ignored. Assumes that data is
    received in temporal order.
    """
    def __init__(self, combine, start_time, window_duration, callbacks):
        self._combine = combine
        self._start_time = start_time
        self._window_duration = window_duration
        self._callbacks = callbacks
        self._buffer = []

    @property
    def _end_time(self):
        return self._start_time + self._window_duration

    def _dump_buffer(self):
        if not self._buffer:
            for cb in self._callbacks:
                cb(None, self._start_time, self._end_time)
        else:
            for cb in self._callbacks:
                cb(self._combine(self._buffer), self._start_time, self._end_time)

    def finish_window(self):
        self._dump_buffer()
        self._buffer = []
        self._start_time += self._window_duration

    def put(self, data, start_time, end_time):
        """
        While it starts after end, dump and ship.
        While it overlaps, add and ship.
        """
        # Ignore data ending before current window starts
        if end_time < self._start_time:
            return
        # Ship window if it ends before the data ends
        while start_time > self._end_time:
            self.finish_window()
        # Add to and ship all windows that it ends after
        while end_time > self._end_time:
            self._buffer.append(data)
            self.finish_window()
        # Add to the window that it ends in
        self._buffer.append(data)
