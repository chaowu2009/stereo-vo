import matplotlib.pylab as plt
import numpy as np

folder = "/home/cwu/project/dataset/images/planar/"
print("data folder =", folder )

fileName = folder + "timeStamp.txt"

timeStamp = []
with open(fileName) as fp:
    lines = fp.readlines()
    for line in lines:
        ln = line.split(',')
#        print(ln[1])
        timeStamp.append(float(ln[1]))

deltaTime = []
timeScale = 1.0;
for k in range(1,len(timeStamp)):
    deltaTime.append((timeStamp[k] - timeStamp[k-1])/timeScale)        
    
freq = [1.0/x for x in deltaTime]
#print("freq(HZ) = ", freq)

flg = plt.figure()
plt.plot(freq)
plt.xlabel('Sample')
plt.ylabel('Hz')
plt.grid(True)
plt.title('Freq')
plt.show()
