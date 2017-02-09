#include "iRobot.h"
#include <stdio.h>

void main()
{
	int fd;

	fd = initRobot();

	if(fd <= 0)
	{
		printf("Can't initialize the serial port\n");
		return;
	}


	char read_buffer[100];   /* Buffer to store the data received              */
	int  bytes_read = 0;    /* Number of bytes read by the read() system call */
	int i = 0;
	double timeStamp = 0.0;

	robotReset(fd);

	robotStart(fd);

	robotSafeMode(fd);
	sleep(1);
	robotLED(fd);
	sleep(1);

	robotStartRotating(fd);
	sleep(2);
	robotStopRotating(fd);
	
	sleep(2);

	robotStartDrive(fd);
	sleep(2);
	robotStopDrive(fd);


	while(1)
	{
		//printf("T:%f \n", current_timestamp());

		bytes_read = read(fd,read_buffer,1); /* Read the data */
		if (bytes_read > 0)
		{
				
				printf("%c",read_buffer[0]);
		}

	}


	close(fd);
}
