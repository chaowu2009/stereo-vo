
import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D



def loadPose(basedir = '/home/cwu/Downloads/dataset/poses', sequence = "00"):

	pose_file = basedir + "/" +sequence + ".txt"

	position = []
	with open(pose_file, 'r') as f:
	    for line in f.readlines():
	        T = np.fromstring(line, dtype=float, sep=' ')
	        T = T.reshape(3, 4)
	        position.append([T[0][3], T[1][3], T[2][3] ])
	
	position = np.asarray(position)
	print('data length = ', len(position[:,0]))
	return position

def loadResult(resultFile = '/home/cwu/project/stereo-vo/src/result.txt'):

    position3D = [];
    position2D = [] ;

    with open(resultFile) as fp:
        for line in fp.readlines():
	        data = [float(d) for d in line.split('\t')]
	#        print(data)
	        position3D.append([data[1], data[2],data[3]])
	        position2D.append([data[4], data[5]])
   
    #save to file

#    with open("/home/cwu/project/stereo-vo/src/truthPosition3D.txt", 'w') as fp:
#        for l in position3D:
#            fp.write(str(l[0]) + '\t' + str(l[1])+ '\t' + str(l[2]) + '\n')

    position3D = np.asarray(position3D)	

    return position3D
	
if __name__ == "__main__":

    position = loadPose()	
    position3D = loadResult()


    plt.subplot(311)
    lx, = plt.plot(position[:,0], label= 'x')
    plt.hold(True)
    ly, = plt.plot(position[:,1], label= 'y')
    lz, = plt.plot(position[:,2], label= 'z')

    plt.legend([lx,ly,lz],['x','y','z'])
    plt.title('true position')
    plt.grid(True)

    plt.subplot(312)
    lx, = plt.plot(position3D[:,0], label= 'x')
    plt.hold(True)
    ly, = plt.plot(position3D[:,1], label= 'y')
    lz, = plt.plot(position3D[:,2], label= 'z')

    plt.legend([lx,ly,lz],['x','y','z'])
    plt.title('vo position')
    plt.grid(True)
        

    plt.subplot(313)
    l1, = plt.plot(position[:,0], position[:,2], marker = 'o', label='truth')
    plt.hold(True)
    l2, = plt.plot(position3D[:,0], position3D[:,2], marker = 'x', label='vo')
    plt.legend([l1,l2],['truth','vo'])
    plt.xlabel('x')
    plt.ylabel('z')
    plt.grid(True)
    plt.show()