class Combiner():
    _windows = []

    def __init__(self, start_time, window_duration, topics, callbacks):
        self._last_time = start_time
        self._window_duration = window_duration
        self.topics = topics
        self._callbacks = callbacks
        self._add_window()

    def _add_window(self):
        self._windows.append(
            {
                'data': {topic: None for topic in self.topics},
                'start_time': self._last_time,
                'end_time': self._last_time + self._window_duration,
                'statuses': {topic: False for topic in self.topics},
                'overlaps': {topic: 0 for topic in self.topics},
            })
        self._last_time += self._window_duration

    def _is_window_ready(self, index):
        return all(self._windows[index]['statuses'].values())

    @staticmethod
    def _overlap(start_time, end_time, window_start_time, window_end_time):
        if start_time > window_end_time or end_time < window_start_time:
            return 0
        start = max(start_time, window_start_time)
        end = min(end_time, window_end_time)
        return (end - start) / float(window_end_time - window_start_time)

    # If there is not at least one window add a window
    # add windows until you have somewhere to put this viable data!
    def put(self, topic, data, start_time, end_time):
        # Add a window if all have been sent
        if not self._windows:
            self._add_window()
        # If it ends before the earliest buffer starts, delete it.
        if end_time < self._windows[0]['start_time']:
            return
        # Make sure that all of the needed windows are there
        while not self._windows[-1]['start_time'] > end_time:
            self._add_window()
        for window in self._windows:
            # Mark earlier windows as ready, for this topic
            if window['end_time'] <= start_time:
                window['statuses'][topic] = True
            # Break if the windows start after the data ends
            elif window['start_time'] > end_time:
                break
            # Edit the overlap if there is overlap
            else:
                overlap = self._overlap(
                    start_time, end_time,
                    window['start_time'], window['end_time'])
                if overlap > window['overlaps'][topic]:
                    window['data'][topic] = data
                    window['overlaps'][topic] = overlap
        # Ship ready windows
        i = 0
        while i < len(self._windows):
            if self._is_window_ready(i):
                for cb in self._callbacks:
                    cb(self._windows[i]['data'],
                        self._windows[i]['start_time'],
                        self._windows[i]['end_time'])
                del self._windows[i]
            else:
                i += 1
