import sys
import argparse
import source
import sink
import numpy as np
import feature_extractor
from rospy_msg_converter import convert_ros_message_to_dictionary

WINDOW_DURATION = .1

def extractBagAudio(device, bag):
	with source.ROSbag(
		lambda m: np.fromstring(m['chunk'], np.int16),
		filename=bag,
		topic='/{}/chunk'.format(device)) as audio_source, \
		sink.CSV(WINDOW_DURATION, 'out.csv') as audio_sink:

		audio_extractor.extract(audio_source, audio_sink)

def extractBagVideo(device, bag, features):
	with source.ROSbag(
        lambda m: {key: m[key] for feature in features},
        filename=bag,
        topic='/{}/face_info'.format(device)) as face_source, \
        sink.CSV(WINDOW_DURATION, 'out.csv') as face_sink:

        face_extractor.extract(face_source, face_sink)

if __name__ == '__main__':
	
	parser = argparse.ArgumentParser(description='Code to extract bag information')
	parser.add_argument('device', nargs=1, help='device of interest')
	parser.add_argument('bag', nargs=1, help='bag of interest')
	parser.add_argument('--window',
		nargs=1, type=float,
		help='change window size')
	parser.add_argument('--face_features', 
		nargs=argparse.REMAINDER,
		help='list of facial features to extract')
	args = parser.parse_args()

	if args.window:
	    WINDOW_DURATION = args.window[0]
	device = args.device[0]
	bag = args.bag[0]
	
	if 'mic' in device:
		extractBagAudio(device, bag)
	elif 'camera' in device and features is not None:
		extractBagVideo(device, bag, face_features)
	else:
		print 'Device or combination of device and features not valid'
		exit(1)
