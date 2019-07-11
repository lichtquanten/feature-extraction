#!/usr/bin/env python
import source
import sink
import feature_extractor
import numpy as np
import rospy
from ros_speech2text.msg import AudioChunk

WINDOW_DURATION = .1

def main():
    rospy.init_node('main_live', anonymous=True)
    with source.ROSlive(
        lambda m: np.fromstring(m['chunk'], np.int16),
        '/mic1/chunk',
        AudioChunk,
        16000) as audio_source, \
         sink.CSV(WINDOW_DURATION, 'out.csv') as audio_sink:


        audio_extractor.extract(audio_source, audio_sink)

if __name__ == '__main__':
    main()
