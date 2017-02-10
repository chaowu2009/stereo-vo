#include <stdio.h>
#include <string.h>
#include <fcntl.h>  /* File Control Definitions          */
#include <termios.h>/* POSIX Terminal Control Definitions*/
#include <unistd.h> /* UNIX Standard Definitions         */
#include <errno.h>  /* ERROR Number Definitions          */


#include <sys/time.h>
#include <stdlib.h>     /* atof */

double current_timestamp() {
    struct timeval te; 
    gettimeofday(&te, NULL); // get current time

    double microseconds = (double)te.tv_sec + (double)te.tv_usec/1000000;

    return microseconds;
}


float stringToFloat(char* data)
{

	float q = (float)atof(data);
	//printf("%f\n",q);

	return q;


}

int parse(char* buff, int len, float* q)
{
	int i = 0;
	int j = 0;
	//for(i=0; i<len; i++)
	//	printf("%c", buff[i]);
	char data[10];

	memset(data, 0, 10);

	for(i=0; i<len; i++)
	{
		if(buff[i] == 'r' && buff[i+1] == ':' )
		{
			i+=2;
			break;
		}
	}

	j = 0;
	while(buff[i]!=' ')
	{
		
		data[j] = buff[i];
		//printf("%c",data[j]);
		j++;
		i++;
	}
	q[0] = stringToFloat(data);
	//printf(" ");

	memset(data, 0, 10);
	for(; i<len; i++)
	{
		if(buff[i] == 'i' && buff[i+1] == ':' )
		{
			i+=2;
			break;
		}
	}

	j = 0;
	while(buff[i]!=' ')
	{
		
		data[j] = buff[i];
		//printf("%c",data[j]);
		j++;
		i++;
	}


	q[1] = stringToFloat(data);
	//printf(" ");

	memset(data, 0, 10);
	for(; i<len; i++)
	{
		if(buff[i] == 'j' && buff[i+1] == ':' )
		{
			i+=2;
			break;
		}
	}

	j = 0;
	while(buff[i]!=' ')
	{
		
		data[j] = buff[i];
		//printf("%c",data[j]);
		j++;
		i++;
	}

	q[2] = stringToFloat(data);
	//printf(" ");
	memset(data, 0, 10);
	for(; i<len; i++)
	{
		if(buff[i] == 'k' && buff[i+1] == ':' )
		{
			i+=2;
			break;
		}
	}

	j = 0;
	while(buff[i]!='\n')
	{
		
		data[j] = buff[i];
		//printf("%c",data[j]);
		j++;
		i++;
	}

	q[3] = stringToFloat(data);
	//printf(" ");

	//printf("\n");

	return 0;
}

int initBNO080()
{
	int fd;

	fd = open("/dev/ttyACM0",O_RDWR | O_NOCTTY);// ST Nucleo Virtual Serial Port
	if (fd<=0)
	{
		printf("Fail");
		return -1;
	}
	else
		printf("ttyACM0 Opened Successfully\n");



	struct termios SerialPortSettings;	/* Create the structure                          */

	tcgetattr(fd, &SerialPortSettings);	/* Get the current attributes of the Serial port */

	cfsetispeed(&SerialPortSettings,B921600); /* Set Read  Speed as 921600                       */
	cfsetospeed(&SerialPortSettings,B921600); /* Set Write Speed as 921600                       */

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
	SerialPortSettings.c_cc[VTIME] = 1; /* Wait   100ms*/

	if((tcsetattr(fd,TCSANOW,&SerialPortSettings)) != 0) /* Set the attributes to the termios structure*/
	{
	    printf("\n  ERROR ! in Setting attributes \n");
	    return -1;
	}
	else
	    printf("\n  BaudRate = 921600 \n  StopBits = 1 \n  Parity   = none \n");

	tcflush(fd, TCIFLUSH); 

	return fd;
}


int readQ(int fd, float* q)
{

	char read_buffer[512];   /* Buffer to store the data received              */
	int  bytes_read = 0;    /* Number of bytes read by the read() system call */
	int i = 0;
	int startLastFrame = 0;
	int endLastFrame = 0;
	//double timeStamp = 0.0;


	bytes_read = read(fd,read_buffer,512); 

	if(bytes_read>=500)
	{
		printf("A lot of data in buffer: %d\n", bytes_read);
		tcflush(fd, TCIFLUSH); 
		bytes_read = read(fd,read_buffer,512); 

	}

	if(bytes_read <= 0)
	{
		return -1;//no data available
	}

	for(i = (bytes_read-1); i>=0; i--)
	{
		if(read_buffer[i] == '\n')
			endLastFrame = i;

		if(read_buffer[i] == 'G')
		{
			startLastFrame = i;
			break;
		}
	}

	int len = endLastFrame-startLastFrame+1;
	if(len > 40)
	{
		if(parse(&read_buffer[startLastFrame], len, q)>0)
			return 0;
		else
			return -1;
	}
	return -1;


	/*while(1)
	{

		bytes_read = read(fd,&read_buffer[i],1); /* Read the data */
		/*if(read_buffer[i] == 'G')
		{
			if(0 == i)
			{
				timeStamp = current_timestamp();
				i = 1;
			}
			else
			{
				i = 0;
			}

		}
		else if(read_buffer[i] == '\n')
		{
			if(i>36)
			{
				printf("T:%f ", timeStamp);
				int j = 0;
				for(j=0;j<i+1;j++)
					printf("%c",read_buffer[j]);

				// parse the data. There might be some corrupts as "T:1486495961.133880 GRV: r:0.996 i:0.070 j:0.051 k0.079 j:0.059 k:-0.003"
			}


			i = 0;
		} 
		else 
		{
			i++;

		}

	}


	close(fd);*/
}


