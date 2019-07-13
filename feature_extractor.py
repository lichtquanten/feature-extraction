import numpy as np
import grouper
import buffer
import combine
import aubio

def get_pitch(data, rate):
    pitch_o = aubio.pitch('yin', len(data), len(data), rate)
    pitch_o.set_unit('Hz')
    return (pitch_o(data)[0], pitch_o.get_confidence())

def is_silent(data):
    """
    Args:
        data: A numpy array.
    """
    return np.mean(np.abs(data)) < 0.25

def audio_test(source, sink):
    # create a curried function for computing time from length or something?
    topics = [
        'in_high_vol_nbhd',
        'long_silence_before'
    ]
    sink.set_topics(topics)

    # Group the audio
    windows_50ms = grouper.Window(source.start_time, .05)
    windows = grouper.Window(source.start_time, sink.window_duration)

    def is_valid(data):
        return all(map(is_silent, data))

    neighborhood = grouper.Neighborhood(is_valid, 10)
    history = grouper.History(10)
    counter = grouper.Counter(is_silent)

    # Use a combiner to bundle the results
    # Note that start_time is not constant!
    comb = combine.Combiner(source.start_time, sink.window_duration, topics)

    for (chunk, start, end) in source:
        chunkf = chunk.astype(np.float32) / 32786.
        # Put into the groupers
        windows_50ms.put(chunkf, start, end)

        for window, w_start, w_end in windows_50ms:
            stacked = np.hstack(window)
            neighborhood.put(stacked, w_start, w_end)
            history.put(stacked, w_start, w_end)
            counter.put(stacked, w_start, w_end)

        for neighbor, n_start, n_end in neighborhood:
            windows.put(neighbor, n_start, n_end)


        for window, w_start, w_end in windows:
            comb.put('in_high_vol_nbhd', any(window), w_start, w_end)

        for cnt, c_start, c_end in counter:
            comb.put('long_silence_before', cnt > 10, c_start, c_end)

        # Process the combiner
        for (bundle, b_start, b_end) in comb:
            print b_start, b_end, bundle
            sink.put(bundle, b_start, b_end)

def audio(source, sink):
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
            dataf = data.astype(np.float32) / 32786.

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
            windowf = [x.astype(np.float32) / 32786. for x in window]
            pitch = [get_pitch(x, source.rate)[0] for x in windowf]
            pitch_std = np.std(pitch)
            comb.put('pitch_std', pitch_std, w_start, w_end)

        # Process the combiner
        for (bundle, b_start, b_end) in comb:
            sink.put(bundle, b_start, b_end)

def face(source, sink):
    topics = [
        'mean_rot_y'
    ]
    sink.set_topics(topics)

    # Group the face info
    windows = grouper.Window(source.start_time, sink.window_duration)

    # Use a combiner to bundle the results
    comb = combine.Combiner(source.start_time, sink.window_duration, topics)

    for (chunk, start, end) in source:
        # Put into the groupers
        windows.put(chunk, start, end)

        # Analyze the data from the groupers
        for (window, w_start, w_end) in windows:
            if not window:
                mean_rot_y = None
            else:
                mean_rot_y = np.mean([x['head_pose']['rot_y'] for x in window])
            comb.put('mean_rot_y', mean_rot_y, w_start, w_end)

        # Process the combiner
        for (bundle, b_start, b_end) in comb:
            sink.put(bundle, b_start, b_end)
