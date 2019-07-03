import source
import sink
import numpy as np
import feature_extractor
from rospy_msg_converter import convert_ros_message_to_dictionary

WINDOW_DURATION = .1

def main():
    keep = ['gazeAngle', 'gazeDirection0', 'gazeDirection1', 'landmarks', 'head_pose', ]
    with source.ROSbag(
        lambda m: {key: m[key] for key in keep},
        filename='bag.bag',
        topic='/camera2/face_info') as face_source, \
         sink.CSV(WINDOW_DURATION, 'out.csv') as face_sink:

        feature_extractor.face(face_source, face_sink)

if __name__ == '__main__':
    main()
