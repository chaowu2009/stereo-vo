
folder = "/home/cwu/project/dataset/images/7/"

fileName = folder + "timeStamp.txt"

timeStamp = []
with open(fileName) as fp:
    lines = fp.readlines()
    for line in lines:
        ln = line.split(',')
#        print(ln[1])
        timeStamp.append(float(ln[1]))

deltaTime = []
for k in range(1,len(timeStamp)):
    deltaTime.append((timeStamp[k] - timeStamp[k-1])/1e3)        
    
for dt in deltaTime:
    print("freq(HZ) = ",1.0/dt)
