import source
import sink
import numpy as np
import feature_extractor

WINDOW_DURATION = .1

def main():
    with source.PyAudio(
        lambda x: np.fromstring(x, dtype=np.int16),
        channels=1,
        rate=44100,
        frames_per_buffer=1024) as audio_source, \
         sink.CSV(WINDOW_DURATION, 'out.csv') as audio_sink:

        feature_extractor.extract(audio_source, audio_sink)

if __name__ == '__main__':
    main()


    # with source.ROSbag(
    #     lambda x: np.fromstring(x['data'], dtype=np.int16),
    #     filename='bag.bag',
    #     topic='pid1/chunk') as audio_source:

    # with source.ROSLive(
    #     lambda x: np.fromstring(x['data'], dtype=np.int16),
    #     topic='pid1/chunk',
    #     msg_type=AudioChunk) as audio_source:
