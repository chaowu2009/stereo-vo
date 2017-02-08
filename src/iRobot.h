#include <stdio.h>
#include <fcntl.h>  /* File Control Definitions          */
#include <termios.h>/* POSIX Terminal Control Definitions*/
#include <unistd.h> /* UNIX Standard Definitions         */
#include <errno.h>  /* ERROR Number Definitions          */


#include <sys/time.h>

double current_timestamp() {
    struct timeval te; 
    gettimeofday(&te, NULL); // get current time

    double microseconds = (double)te.tv_sec + (double)te.tv_usec/1000000;

    return microseconds;
}

void robotReset(int fd)
{
	printf("Reset\n");
	//Reset Command 7
	char writeBuff = 7;
    write(fd, &writeBuff, 1);
    sleep(5);
}


void robotStart(int fd)
{
	printf("Start\n");
	//Started Command 128
	char writeBuff = 128;
    write(fd, &writeBuff, 1);
    sleep(2);
}

void robotSafeMode(int fd)
{
	printf("Saft mode\n");
	//Switch to Saft mode Command 131
	char writeBuff = 131;
    write(fd, &writeBuff, 1);
    sleep(0.2);
}

void robotLED(int fd)
{
	printf("LEDs\n");
	//Turn on the LEDs 139 10 255 255, Advance & Play both RED
	char writeBuff[4] = {139,10,255,255};
    write(fd, writeBuff, 4);
    sleep(0.2);
}               

void robotStartRotating(int fd)
{
	printf("Rotating\n");
	//Drive in straight and turn: [137] [Velocity high byte] [Velocity low byte] [Radius high byte] [Radius low byte]
	char writeBuff[5] = {0x89,0x00,0x20,0x00,0x01};
    write(fd, writeBuff, 5);
    sleep(0.2);
}               

void robotStopRotating(int fd)
{
	printf("Rotating\n");
	//Drive in straight and turn: [137] [Velocity high byte] [Velocity low byte] [Radius high byte] [Radius low byte]
	char writeBuff[5] = {0x89,0x00,0x00,0x00,0x01};
    write(fd, writeBuff, 5);
    sleep(0.2);
}                  

struct termios SerialPortSettings;	/* Create the structure                          */

void initRobot()
{
	int fd;

	fd = open("/dev/ttyUSB0",O_RDWR | O_NOCTTY);// iRobot Serial Port
	if (fd<=0)
	{
		printf("Fail");
		return;
	}
	else {
		printf("ttyUSB0 Opened Successfully\n");
	}

	tcgetattr(fd, &SerialPortSettings);	/* Get the current attributes of the Serial port */

	cfsetispeed(&SerialPortSettings,B115200); /* Set Read  Speed as 115200                       */
	cfsetospeed(&SerialPortSettings,B115200); /* Set Write Speed as 115200                       */

	SerialPortSettings.c_cflag &= ~PARENB;   /* Disables the Parity Enable bit(PARENB),So No Parity   */
	SerialPortSettings.c_cflag &= ~CSTOPB;   /* CSTOPB = 2 Stop bits,here it is cleared so 1 Stop bit */
	SerialPortSettings.c_cflag &= ~CSIZE;	 /* Clears the mask for setting the data size             */
	SerialPortSettings.c_cflag |=  CS8;      /* Set the data bits = 8                                 */

	SerialPortSettings.c_cflag &= ~CRTSCTS;       /* No Hardware flow Control                         */
	SerialPortSettings.c_cflag |= CREAD | CLOCAL; /* Enable receiver,Ignore Modem Control lines       */ 


	SerialPortSettings.c_iflag &= ~(IXON | IXOFF | IXANY);          /* Disable XON/XOFF flow control both i/p and o/p */
	SerialPortSettings.c_iflag &= ~(ICANON | ECHO | ECHOE | ISIG);  /* Non Cannonical mode                            */

	SerialPortSettings.c_oflag &= ~OPOST;/*No Output Processing*/

	/* Setting Time outs */
	SerialPortSettings.c_cc[VMIN] = 1; /* Read at least 1 characters */
	SerialPortSettings.c_cc[VTIME] = 1; /* Wait 0.1 second   */
}

void test()

    initRobot();
	
	if((tcsetattr(fd,TCSANOW,&SerialPortSettings)) != 0) /* Set the attributes to the termios structure*/
	    printf("\n  ERROR ! in Setting attributes \n");
	else
	    printf("\n  BaudRate = 115200 \n  StopBits = 1 \n  Parity   = none \n");

	tcflush(fd, TCIFLUSH); 

	char read_buffer[100];   /* Buffer to store the data received              */
	int  bytes_read = 0;    /* Number of bytes read by the read() system call */
	int i = 0;
	double timeStamp = 0.0;

	robotReset(fd);

	robotStart(fd);

	robotLED(fd);
        
    robotStartRotating(fd);

	close(fd);
}


