import RPi.GPIO as GPIO
import smbus
import time
import numpy

bus = smbus.SMBus(1)
address = 0x48

BNO_INTN = 4
BNO_RSTN = 17

#Game Rotation Vector enabled at 100Hz
enableSensor=[
	0x00,0x38,0x03,0x06,0x00,0x11,0x00,0x00,0x00,0x00,0x10,0x27,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,
]

def printSensor(inData):
	i = 0.0
	j = 0.0
	k = 0.0
	w = 0.0

	if inData[0] >= 14 and inData[2] == 0x08:
		#Q point 14 = 16384
		i = numpy.float32(numpy.int16(inData[6]+inData[7]*256))/16384.0
		j = numpy.float32(numpy.int16(inData[8]+inData[9]*256))/16384.0
		k = numpy.float32(numpy.int16(inData[10]+inData[11]*256))/16384.0 		
		w = numpy.float32(numpy.int16(inData[12]+inData[13]*256))/16384.0

	#for some reason, there's occasionally abnormal data received
	#only print the reasonable data when sum of squares = (0.99, 1.01)	
	q = i*i+j*j+k*k+w*w	
	if q > 0.99 and q < 1.01:
		print str(i)+", "+str(j)+", "+str(k)+", "+str(w)



if __name__=="__main__":

	GPIO.setmode(GPIO.BCM)
	GPIO.setup(BNO_INTN, GPIO.IN, pull_up_down = GPIO.PUD_UP)
	GPIO.setup(BNO_RSTN, GPIO.OUT, initial = GPIO.HIGH)


	GPIO.output(BNO_RSTN, 0)
	time.sleep(0.1)
	GPIO.output(BNO_RSTN, 1)



	GPIO.wait_for_edge(BNO_INTN, GPIO.FALLING, timeout=1000) 
	#first all 0    
	bus.read_i2c_block_data(address, 0, 18)


	time.sleep(0.01)    
	#enable 100Hz GRV
	bus.write_i2c_block_data(address, 0x05, enableSensor)

	while True:
		if GPIO.input(BNO_INTN):
			GPIO.wait_for_edge(BNO_INTN, GPIO.FALLING, timeout=1)     
		if GPIO.input(BNO_INTN) == False:	
			sensorData = bus.read_i2c_block_data(address, 0, 18) 
			#print sensorData
			printSensor(sensorData)#
		else:
			continue
	


