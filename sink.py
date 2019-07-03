from abc import ABCMeta, abstractmethod
import csv
# import rospy

class Sink(object):
    __metaclass__ = ABCMeta
    def __init__(self):
        pass

    @abstractmethod
    def put(self, data, start_time, end_time):
        pass

    def set_topics(self, headers):
        self.headers = headers

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        pass

class CSV(Sink):
    def __init__(self, window_duration, filename):
        self.window_duration = window_duration
        self.filename = filename
        self.headers = None
        self.file = None
        self.writer = None

    def set_topics(self, headers):
        self.headers = headers
        self.writer.writerow(['start_time', 'end_time'] + headers)

    def put(self, data, start_time, end_time):
        if self.headers is None:
            raise Exception('Must set headers before putting data to Sink.')
        row = [start_time, end_time]
        for label in self.headers:
            row.append(data[label])
        self.writer.writerow(row)

    def __enter__(self):
        self.file = open(self.filename, 'w')
        self.writer = csv.writer(self.file)
        return self

    def __exit__(self, *exc):
        self.file.close()

# class ROSMsg(Sink):
#     def __init__(self, window_duration, topic, msg_type):
#         self.window_duration = window_duration
#         self.topic = topic
#         self.msg_type = msg_type
#         self.headers = None
#         self.publisher = None
#
#     def put(self, data, start_time, end_time):
#         if self.headers is None:
#             raise Exception('Must set headers before putting data to Sink.')
#         self.publisher.publish(
#             {
#                 'start_time': start_time,
#                 'end_time': end_time,
#                 'data': {data[label] for label in self.headers}
#             })
#
#     def __enter__(self):
#         self.publisher = rospy.Publisher(self.topic, self.msg_type)
#         return self
#
#     def __exit__(self, *exc):
#         self.publisher.unregister()
