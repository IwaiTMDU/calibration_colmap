#!/usr/bin/env python
import rospy
import numpy as np
import yaml
import cv2
 
# A yaml constructor is for loading from a yaml node.
# This is taken from: http://stackoverflow.com/a/15942429
def opencv_matrix_constructor(loader, node):
    mapping = loader.construct_mapping(node, deep=True)
    mat = np.array(mapping["data"])
    mat.resize(mapping["rows"], mapping["cols"])
    return mat
 
# A yaml representer is for dumping structs into a yaml node.
# So for an opencv_matrix type (to be compatible with c++'s FileStorage) we save the rows, cols, type and flattened-data
def opencv_matrix_representer(dumper, mat):
    mapping = {'rows': mat.shape[0], 'cols': mat.shape[1], 'dt': 'd', 'data': mat.reshape(-1).tolist()}
    return dumper.represent_mapping(u'tag:yaml.org,2002:opencv-matrix', mapping)

if __name__ == '__main__':
	rospy.init_node('yaml_test_node', anonymous=True)
	yaml.add_constructor(u'tag:yaml.org,2002:opencv-matrix', opencv_matrix_constructor)
	yaml.add_representer(np.ndarray, opencv_matrix_representer)
	'''
	with open('output.yaml', 'w') as f:
   		f.write("%YAML:1.0")
   		#yaml.dump(np.zeros((10,10)).tolist(), f)
		mat = np.ndarray(shape=(2,2), dtype='float64')
		yaml.dump(mat, f)
	'''
	f=cv2.FileStorage('test.yml', flags=1)
	if not f.isOpened():
		mat = np.ndarray(shape=(2,2), dtype='float64')
		f.write(name='mat',val=mat)
	f.release()
	
	rospy.spin()
