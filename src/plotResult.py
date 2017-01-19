import numpy as np
from matplotlib.pyplot import *

imgDir="/home/cwu/project/dataset/images/9/";

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

subplot(211)
labels=['x','y','z']
[xl,yl,zl] = plot(position3D)
legend([xl,yl,zl], ['x','y','z'], loc=1)
title('position3D')
grid(True)

subplot(212)
labels=['x','y']
plot(position2D[:,0],position2D[:,1])
#legend([xl,yl], ['x','y'], loc=1)
title('position2D( image plane)')
xlabel('x')
ylabel('y')
grid(True)
show()


