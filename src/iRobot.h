#include <stdio.h>
#include <fcntl.h>  /* File Control Definitions          */
#include <termios.h>/* POSIX Terminal Control Definitions*/
#include <unistd.h> /* UNIX Standard Definitions         */
#include <errno.h>  /* ERROR Number Definitions          */


#include <sys/time.h>

double current_timestamp_1() {
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

struct termios SerialPortSettings_1;	/* Create the structure                          */

int initRobot()
{
	int fd;

	fd = open("/dev/ttyUSB0",O_RDWR | O_NOCTTY);// iRobot Serial Port
	if (fd<=0)
	{
		printf("Fail");
		return 0;
	}
	else {
		printf("ttyUSB0 Opened Successfully\n");
	}

	tcgetattr(fd, &SerialPortSettings_1);	/* Get the current attributes of the Serial port */

	cfsetispeed(&SerialPortSettings_1,B115200); /* Set Read  Speed as 115200                       */
	cfsetospeed(&SerialPortSettings_1,B115200); /* Set Write Speed as 115200                       */

	SerialPortSettings_1.c_cflag &= ~PARENB;   /* Disables the Parity Enable bit(PARENB),So No Parity   */
	SerialPortSettings_1.c_cflag &= ~CSTOPB;   /* CSTOPB = 2 Stop bits,here it is cleared so 1 Stop bit */
	SerialPortSettings_1.c_cflag &= ~CSIZE;	 /* Clears the mask for setting the data size             */
	SerialPortSettings_1.c_cflag |=  CS8;      /* Set the data bits = 8                                 */

	SerialPortSettings_1.c_cflag &= ~CRTSCTS;       /* No Hardware flow Control                         */
	SerialPortSettings_1.c_cflag |= CREAD | CLOCAL; /* Enable receiver,Ignore Modem Control lines       */ 


	SerialPortSettings_1.c_iflag &= ~(IXON | IXOFF | IXANY);          /* Disable XON/XOFF flow control both i/p and o/p */
	SerialPortSettings_1.c_iflag &= ~(ICANON | ECHO | ECHOE | ISIG);  /* Non Cannonical mode                            */

	SerialPortSettings_1.c_oflag &= ~OPOST;/*No Output Processing*/

	/* Setting Time outs */
	SerialPortSettings_1.c_cc[VMIN] = 1; /* Read at least 1 characters */
	SerialPortSettings_1.c_cc[VTIME] = 1; /* Wait 0.1 second   */

      return fd;
}

void test() {

    int fd = initRobot();
	
	if((tcsetattr(fd,TCSANOW,&SerialPortSettings_1)) != 0) /* Set the attributes to the termios structure*/
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


