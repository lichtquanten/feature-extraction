import numpy as np
import grouper
import buffer
import combine
import aubio

def get_pitch(data, rate):
    pitch_o = aubio.pitch('yin', len(data), len(data), rate)
    pitch_o.set_unit('Hz')
    return (pitch_o(data)[0], pitch_o.get_confidence())

def extract(source, sink):
    topics = [
        'mean_volume',
        'pitch',
        'pitch_confidence',
        'pitch_std',
    ]
    sink.set_topics(topics)

    # Group the audio
    blocks_10ms = grouper.Block(
        block_buffer=buffer.BlockBufferNP(int(.01 * float(source.rate)), np.float32),
    )
    windows_blocks_10ms = grouper.Window(source.start_time, sink.window_duration)
    windows = grouper.Window(source.start_time, sink.window_duration)

    # Use a combiner to bundle the results
    comb = combine.Combiner(source.start_time, sink.window_duration, topics)

    for (chunk, start, end) in source:
        # Put into the groupers
        windows.put(chunk, start, end)
        blocks_10ms.put(chunk, start, end)

        # Analyze the data from the groupers
        for (window, w_start, w_end) in windows:
            data = np.hstack(window)
            dataf = data.astype(np.float32) / 32786

            # Mean volume
            mean_volume = np.mean(np.abs(dataf))
            comb.put('mean_volume', mean_volume, w_start, w_end)

            # Pitch
            pitch, confidence = get_pitch(dataf, source.rate)
            comb.put('pitch', pitch, w_start, w_end)
            comb.put('pitch_confidence', confidence, w_start, w_end)

        for (block, b_start, b_end) in blocks_10ms:
            windows_blocks_10ms.put(block, b_start, b_end)

        for (window, w_start, w_end) in windows_blocks_10ms:
            windowf = [x.astype(np.float32) / 32786 for x in window]
            pitch = [get_pitch(x, source.rate)[0] for x in windowf]
            pitch_std = np.std(pitch)
            comb.put('pitch_std', pitch_std, w_start, w_end)

        # Process the combiner
        for (bundle, b_start, b_end) in comb:
            sink.put(bundle, start, end)
