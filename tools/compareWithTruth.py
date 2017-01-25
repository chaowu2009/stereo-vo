
import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D



def loadPose(basedir = '/home/cwu/Downloads/dataset/poses', sequence = "00"):
    pose_file = basedir + "/" + sequence + ".txt"

    position = []
    firstLine = True
    firstLineValue = [0,0,0]
    DCM = []
    with open(pose_file, 'r') as f:
        for line in f.readlines():
            T = np.fromstring(line, dtype=float, sep=' ')  
            # This is a 3 *4 matrix, the last column (first three elements) is the position
            T = T.reshape(3, 4)
            currentLine = [T[0][3], T[1][3], T[2][3]];
            dcm = T[0:2]
            if firstLine:
                print("dcm = ", dcm)
                print(currentLine)
                firstLineValue = currentLine;
                firstLine = False
            newValueLine = [currentLine[i] - firstLineValue[i] for i in range(0,3)]
            DCM.append(dcm)
            position.append(newValueLine)

    position = np.asarray(position)
    print('data length = ', len(position[:,0]))
    return (position, DCM)

def loadResult(resultFile = '/home/cwu/project/stereo-vo/src/result.txt'):

    position3D = []
    imgPosition=[] 

    with open(resultFile) as fp:
        for line in fp.readlines():
            data = [float(d) for d in line.split('\t')]
            position3D.append([data[1], data[2],data[3]])
            imgPosition.append([data[4], data[5]])
    position3D = np.asarray(position3D)	
    imgPosition = np.asarray(imgPosition) 

    return (position3D, imgPosition)
	
if __name__ == "__main__":

    (position, DCM) = loadPose()	
    #position3D = loadResult("/home/cwu/project/SimpleStereoVO/src/result.txt")
    (position3D, imgPosition)= loadResult("/home/cwu/project/stereo-vo/src/vo_result_original.txt")

    plt.subplot(511)
    lx, = plt.plot(position[:,0], label= 'x')
    plt.hold(True)
    ly, = plt.plot(position[:,1], label= 'y')
    lz, = plt.plot(position[:,2], label= 'z')

    plt.legend([lx,ly,lz],['x','y','z'])
    plt.title('True position')
    plt.grid(True)


    plt.subplot(512)
    lx, = plt.plot(position3D[:,0], label= 'x')
    plt.hold(True)
    ly, = plt.plot(position3D[:,1], label= 'y')
    lz, = plt.plot(position3D[:,2], label= 'z')

    plt.legend([lx,ly,lz],['x','y','z'])
    plt.title('vo position')
    plt.grid(True)
        
    plt.subplot(513)
    lx, = plt.plot(position[:,0], label= 'truth')
    plt.hold(True)
    ly, = plt.plot(position3D[:,0], label= 'vo')
    
    plt.legend([lx,ly],['truth','vo'])
    plt.title('x')
    plt.grid(True)

    plt.subplot(514)
    lx, = plt.plot(position[:,2], label= 'truth')
    plt.hold(True)
    ly, = plt.plot(position3D[:,2], label= 'vo')
    
    plt.legend([lx,ly],['truth','vo'])
    plt.title('z')
    plt.grid(True)
        

    plt.subplot(515)
    l1, = plt.plot(position[:,0], position[:,2], marker = 'o', label='truth')
    plt.hold(True)
    l2, = plt.plot(position3D[:,0], position3D[:,2], marker = 'x', label='vo')
    plt.legend([l1,l2],['truth','vo'])
    plt.grid(True)
    plt.show()
