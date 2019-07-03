#!/usr/bin/env python
import source
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
        44100) as audio_source:

        for audio in audio_source:
            print audio

if __name__ == '__main__':
    main()
