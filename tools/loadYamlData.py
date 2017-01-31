import yaml
import cv2
import numpy as np

#http://www.morethantechnical.com/2016/03/02/opencv-python-yaml-persistance/
# remove the first line 

def loadYamlData(dataFile = "cam_left_1.yml"):
	# A yaml constructor is for loading from a yaml node.
	# This is taken from: http://stackoverflow.com/a/15942429
	def opencv_matrix_constructor(loader, node):
	    mapping = loader.construct_mapping(node, deep=True)
	    mat = np.array(mapping["data"])
	    mat.resize(mapping["rows"], mapping["cols"])
	    return mat
	yaml.add_constructor(u"tag:yaml.org,2002:opencv-matrix", opencv_matrix_constructor)
	 
	# A yaml representer is for dumping structs into a yaml node.
	# So for an opencv_matrix type (to be compatible with c++'s FileStorage) we save the rows, cols, type and flattened-data
	def opencv_matrix_representer(dumper, mat):
	    mapping = {'rows': mat.shape[0], 'cols': mat.shape[1], 'dt': 'd', 'data': mat.reshape(-1).tolist()}
	    return dumper.represent_mapping(u"tag:yaml.org,2002:opencv-matrix", mapping)
	yaml.add_representer(np.ndarray, opencv_matrix_representer)
	 
	 
	#example
	#with open('output.yaml', 'w') as f:
	#    f.write("%YAML:1.0")
	#    yaml.dump({"a matrix": np.zeros((10,10)), "another_one": np.zeros((2,4))}, f)
	 

	with open(dataFile, 'r') as f:
	    data = yaml.load(f)   

        return data

if __name__ == "__main__":
    data = loadYamlData("cam_left_1.yml")
    print(data)
   

