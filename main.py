from functools import partial
import csv
import numpy as np
import pyaudio
import aubio

import analysis
import buffer
import combine

CHUNK = 4096
RATE = 44100
CHANNELS = 1
START_TIME = 0
WINDOW_DURATION = .1

def main():
    with open('output.csv', 'wb') as csvfile:
        topics = [
            'start_time',
            'end_time',
            'pitch',
            'mean_volume',
            'high_volume_nbhd',
            'high_pitch_nbhd',
        ]
        writer = csv.DictWriter(csvfile, topics)
        writer.writeheader()

        ### combine ###
        def cb(data, start_time, end_time):
            data['start_time'] = start_time
            data['end_time'] = end_time
            writer.writerow(data)
            print data
        comb = combine.Combiner(START_TIME, WINDOW_DURATION, topics[2:], [cb, ])
        #############

        ##### pitch, window #####
        def analyze(data):
            data = np.hstack(data)
            block_size = len(data)
            pitch_o = aubio.pitch("yin", block_size, block_size, RATE)
            pitch_o.set_unit('Hz')
            return pitch_o(data)[0]
        cbs = [partial(comb.put, 'pitch'), ]
        pitch_window = analysis.SimpleAnalyzer(analyze, cbs)
        ################

        ##### mean volume, window #####
        def analyze(data):
            return np.mean(np.abs(data))
        cbs = [partial(comb.put, 'mean_volume'), ]
        mean_volume_window = analysis.SimpleAnalyzer(analyze, cbs)
        ################

        ##### audio, window #####
        cbs = [mean_volume_window.put, pitch_window.put]
        audio_window = analysis.WindowAnalyzer(
            np.hstack, START_TIME, WINDOW_DURATION, cbs)
        ################

        # please rename
        ### contains high pitch neighbhorhood, window ###
        cbs = [partial(comb.put, 'high_pitch_nbhd'), ]
        window_cont_high_pitch_nbhd = analysis.WindowAnalyzer(
            any, START_TIME, WINDOW_DURATION, cbs)
        ################

        ### in high pitch neighborhood, block ##
        def is_valid(data):
            """Figures out if the pitch fits what we want"""
            return all([x > 500 for x in data])
        cbs = [window_cont_high_pitch_nbhd.put, ]
        blk_in_high_pitch_nbhd = analysis.NeighborAnalyzer(is_valid, 5, cbs)
        #################

        ##### pitch, block #####
        block_size = 4096
        def analyze(data):
            block_size = 4096
            pitch_o = aubio.pitch("yin", block_size, block_size, RATE)
            pitch_o.set_unit('Hz')
            return pitch_o(data)[0]
        block_buffer = buffer.BlockBufferNP(block_size, np.float32)
        cbs = [blk_in_high_pitch_nbhd.put, ]
        pitch_block = analysis.BlockAnalyzer(block_buffer, analyze, cbs)
        #################

        ### contains high volume neighbhorhood, window ###
        cbs = [partial(comb.put, 'high_volume_nbhd'), ]
        window_cont_high_volume_nbhd = analysis.WindowAnalyzer(
            any, START_TIME, WINDOW_DURATION, cbs)
        ################

        # need not be a block
        ### in high volume neighborhood ##
        def is_valid(data):
            """Figures out if the volume fits what we want"""
            return all([x > .4 for x in data])
        cbs = [window_cont_high_volume_nbhd.put ]
        in_high_volume_nbhd = analysis.NeighborAnalyzer(is_valid, 5, cbs)
        #################

        ## volume, block #
        def analyze(data):
            return np.mean(np.abs(data))
        block_size = int(.1 * RATE) * CHANNELS
        block_buffer = buffer.BlockBufferNP(block_size, np.float32)
        cbs = [in_high_volume_nbhd.put, ]
        volume_block = analysis.BlockAnalyzer(block_buffer, analyze, cbs)
        ################

        ##### audio #####
        audio_cbs = [volume_block.put, pitch_block.put, audio_window.put ]
        #################

        p = pyaudio.PyAudio()
        stream = p.open(format=pyaudio.paInt16,
                        channels=1,
                        rate=RATE,
                        input=True,
                        frames_per_buffer=CHUNK)

        start_time = START_TIME
        while True:
            data = stream.read(CHUNK)
            if not data:
                break
            data = np.fromstring(data, dtype=np.int16)
            dataf = data.astype(np.float32) / 32786

            end_time = start_time + data.size / float(RATE)

            ##### audio #####
            for cb in audio_cbs:
                cb(dataf, start_time, end_time)
            #################

            start_time = end_time

if __name__ == "__main__":
    main()
