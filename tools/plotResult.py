import numpy as np
from matplotlib.pyplot import *

imgDir="/home/cwu/project/dataset/images/10/";
imgDir ="D:/vision/dataset/sequences/planar/"

resultFile = imgDir+ 'vo_result.txt'

print("result location is " , imgDir);

position3D = [];
position2D = [] ;

with open(resultFile) as fp:
    for line in fp.readlines():
        data = [float(d) for d in line.split('\t')]
#        print(data)
        position3D.append([data[1], data[2],data[3]])
        position2D.append([data[4], data[5]])

position2D = np.asarray(position2D)
position3D = np.asarray(position3D)

subplot(411)
labels=['x','y','z']
[xl,yl,zl] = plot(position3D)
legend([xl,yl,zl], ['x','y','z'], loc=1)
title('position3D')
grid(True)

subplot(412)
plot(position3D[:,0],position3D[:,1])
title('position2D( image plane)')
xlabel('x')
ylabel('y')
grid(True)

subplot(413)
plot(position3D[:,0],position3D[:,2])
title('position2D( image plane)')
xlabel('x')
ylabel('z')
grid(True)

subplot(414)
plot(position3D[:,1],position3D[:,2])
title('position2D( image plane)')
xlabel('y')
ylabel('z')
grid(True)


show()


