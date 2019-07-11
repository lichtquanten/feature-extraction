def LookupTopicsForFeature(feature):
	# Video features
	if feature == 'gaze':
		return 
	elif feature == 'landmarks':
		return
	elif feature == 'pose':
		return
	# Audio features
	elif feature == 'pitch':
		topics = [
        'pitch',
        'pitch_confidence',
        'pitch_std',
        ]
		return topics
	elif feature == 'mean_volume':
		topics = [
        'mean_volume'
        ]
		return topics

	else:
		print 'Feature is not supported'
		exit(1)