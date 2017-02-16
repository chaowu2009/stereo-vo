#include "iRobot.h"
#include <stdio.h>
#include "BNO080.h"
//#include "rotation.h"
#include <math.h>

int main()
{
	int fd;

	fd = initRobot();

        int fd_bno = initBNO080();
        float q[4];


	if(fd <= 0)
	{
		printf("Can't initialize the serial port\n");
		return -1;
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


         robotStartDrive(fd);
         sleep(5);
         robotStopDrive(fd);
         sleep(1); 
      
        float init_heading = 0;
        float current_heading = 0;	 
        float delta = 0;

// first line
         readQ(fd_bno, q);
         init_heading =  quaternionToYaw(q[1],q[2],q[3],q[0]);        
         robotStartRotating(fd);
        
	while(1)
	{
		
	readQ(fd_bno, q);

         current_heading = quaternionToYaw(q[1],q[2],q[3],q[0]);       
         delta = abs(current_heading - init_heading);
         printf( "current = %f\t, init = %f\tdelta = %f\n",  current_heading, init_heading, delta);
         if (delta > 180)
             delta = 360-delta; 
        if ( delta>= 90.0) {
            robotStopRotating(fd);
            break;         
         }

	}

// second line
        robotStartDrive(fd);
         sleep(5);
         robotStopDrive(fd);
         sleep(1); 
      
         readQ(fd_bno, q);
         init_heading =  quaternionToYaw(q[1],q[2],q[3],q[0]);        
         robotStartRotating(fd);
        
	while(1)
	{
		
	readQ(fd_bno, q);

         current_heading =quaternionToYaw(q[1],q[2],q[3],q[0]);        
         delta = abs(current_heading - init_heading);
         printf( "current = %f\t, init = %f\tdelta = %f\n",  current_heading, init_heading, delta);
         if (delta > 180)
             delta = 360-delta;
         if ( delta>= 90.0) {
            robotStopRotating(fd);
            break;         
         }

	}

// third line
        robotStartDrive(fd);
         sleep(5);
         robotStopDrive(fd);
         sleep(1); 
      
         readQ(fd_bno, q);
         init_heading =  quaternionToYaw(q[1],q[2],q[3],q[0]);        
         robotStartRotating(fd);
        
	while(1)
	{
		
	readQ(fd_bno, q);

         current_heading =quaternionToYaw(q[1],q[2],q[3],q[0]);        
        delta = abs(current_heading - init_heading);
         printf( "current = %f\t, init = %f\tdelta = %f\n",  current_heading, init_heading, delta);
        if (delta > 180)
             delta = 360-delta;
         if ( delta>= 90.0) {
            robotStopRotating(fd);
            break;         
         }
	}

// fourth line
        robotStartDrive(fd);
         sleep(5);
         robotStopDrive(fd);
         sleep(1); 
      
         readQ(fd_bno, q);
         init_heading =  quaternionToYaw(q[1],q[2],q[3],q[0]);        
         robotStartRotating(fd);
        
	while(1)
	{
		
	readQ(fd_bno, q);

         current_heading =quaternionToYaw(q[1],q[2],q[3],q[0]);        
        delta = abs(current_heading - init_heading);
        printf( "current = %f\t, init = %f\tdelta = %f\n",  current_heading, init_heading, delta);
        if (delta > 180)
             delta = 360-delta; 
        if ( delta>= 90.0) {
            robotStopRotating(fd);
            break;         
         }
	}

	close(fd);
}
