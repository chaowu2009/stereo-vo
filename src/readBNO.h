#include <stdio.h>
#include <fcntl.h>
#include <string.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdint.h>
#include <poll.h>

#include "wiringPi.h"

#define GPIO_INTN 4
#define GPIO_RSTN 17

#define I2C_ADDRESS 0x48

#define ROTATION_VECTOR 0x35
#define GAME_ROTATION_VECTOR 0x38

#define SCALE_Q(n) (1.0f / (1<<n))

const float scaleRadToDeg = 180 /3.1415926;

int16_t read16(const uint8_t *p)
{
	int16_t retVal = p[0] | (p[1] << 8 );
	return retVal;
}

void enableSensor(unsigned char sensorId, unsigned int interval, int fd)
{
	if(fd<=0)
	{
		printf("Failed to open the device\n");
		return;
	}

	// 100Hz RV
	unsigned char buf[23]={0x05, 0x00,0x35,0x03,0x06,0x00,0x11,0x00,0x00,0x00,0x00,0x10,0x27,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
	buf[2] = sensorId,
	*((int *)(&buf[11])) = interval;
	write(fd, buf, 23); 	

	return;
}

void printSensor(const unsigned char* buf, int len, float quaternion[4])
{
	float i,j,k,r,acc = 0;
	if(len<14)
	{
		printf("Too short message, length = %d\n", len);
		return;
	}
	if(0x08 == buf[2]) {
		i = read16(&buf[6]) * SCALE_Q(14);
		j = read16(&buf[8]) * SCALE_Q(14);
		k = read16(&buf[10]) * SCALE_Q(14);
		r = read16(&buf[12]) * SCALE_Q(14);
		//printf("Game Rotation Vector: %.3f, %.3f, %.3f, %.3f\n", i, j, k, r);
		quaternion[0] = r;  // w
		quaternion[1] = i;  // x
		quaternion[2] = j;  // y
		quaternion[3] = k;  // z
	} else if(0x05 == buf[2]) {	
		i = read16(&buf[6]) * SCALE_Q(14);
		j = read16(&buf[8]) * SCALE_Q(14);
		k = read16(&buf[10]) * SCALE_Q(14);
		r = read16(&buf[12]) * SCALE_Q(14);
		acc = read16(&buf[14]) * SCALE_Q(12);	
		printf("Rotation Vector: %.3f, %.3f, %.3f, %.3f, Accuracy: %.3f\n", i, j, k, r, acc);
	} else {
		printf("Unrecognized sensor ID\n");
	}
}

volatile int flagInt = 1;
static unsigned char buf_recv[100];

void intnCallback()
{
	flagInt = 0;
}

int initBNO(){

		
	int fd = open("/dev/i2c-1", O_RDWR);
	if(fd<=0)
	{
		printf("Error in openning i2c-1\n");
		while(1)
		;
	}
	if(ioctl(fd, I2C_SLAVE, I2C_ADDRESS)<0)
	{	
		printf("Failed to open BNO070\n");
		while(1)
		;
	}

	wiringPiSetupGpio();
	
	pinMode(GPIO_RSTN, OUTPUT);
	pinMode(GPIO_INTN, INPUT);

	wiringPiISR(GPIO_INTN, INT_EDGE_FALLING, &intnCallback);

	digitalWrite(GPIO_RSTN, LOW);
	usleep(10000);
	digitalWrite(GPIO_RSTN, HIGH);
	usleep(300000);	

	//First all 0 message
	int count = read(fd, buf_recv, 18);

	usleep(10000);

	//enableSensor(ROTATION_VECTOR, 10000, fd);
	enableSensor(GAME_ROTATION_VECTOR, 10000, fd);

    return fd;
}

void readQuaternion(int fd, float q[4])
{
    if(0 == flagInt) {
	    flagInt = 1;
		int count = read(fd, buf_recv, 18);
		unsigned char tmpBuf[20];
		memcpy(tmpBuf, buf_recv, 18);
		printSensor(tmpBuf, count, q);
		printf("Game Rotation Vector (w,x,y,z): %.6f, %.6f, %.6f, %.6f\n", q[0], q[1], q[2], q[3]);
	}
	
}
