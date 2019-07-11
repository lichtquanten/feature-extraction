import numpy as np
import grouper
import buffer
import combine

# TO DO: Implement these modules
# import nod_detector
# import gaze_detector

def extract(source, sink, features):
    # TO-DO: Uncomment lines if this part of code is implemented

    # topics = []
    # for feature in features:
    #     topics.append(lookupTopicsForFeature(feature))
    
    topics = ['mean_rot_y']
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

            # TO-DO: 
            # add function for gaze and nod detection here!

            if not window:
                mean_rot_y = None
            else:
                mean_rot_y = np.mean([x['head_pose']['rot_y'] for x in window])
            comb.put('mean_rot_y', mean_rot_y, w_start, w_end)

        # Process the combiner
        for (bundle, b_start, b_end) in comb:
            sink.put(bundle, b_start, b_end)
