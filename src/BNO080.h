#include <stdio.h>
#include <fcntl.h>  /* File Control Definitions          */
//#include <termios.h>/* POSIX Terminal Control Definitions*/
#ifdef _WIN32
#include <io.h>
#else
#include <unistd.h>
#endif
#include <errno.h>  /* ERROR Number Definitions          */

#ifndef _WIN32
#include <sys/time.h>
#endif

double current_timestamp() {
    struct timeval te;
  //  gettimeofday(&te, NULL); // get current time

    double microseconds = (double)te.tv_sec + (double)te.tv_usec / 1000000;

    return microseconds;
}

struct termios SerialPortSettings;	/* Create the structure                          */

int initBNO080()
{
    int fd;

    fd = open("/dev/ttyACM0", O_RDWR | O_NOCTTY);// ST Nucleo Virtual Serial Port
    if (fd <= 0)
        printf("Fail");
    else
        printf("ttyACM0 Opened Successfully\n");


    tcgetattr(fd, &SerialPortSettings);	/* Get the current attributes of the Serial port */

    cfsetispeed(&SerialPortSettings, B921600); /* Set Read  Speed as 921600                       */
    cfsetospeed(&SerialPortSettings, B921600); /* Set Write Speed as 921600                       */

    SerialPortSettings.c_cflag &= ~PARENB;   /* Disables the Parity Enable bit(PARENB),So No Parity   */
    SerialPortSettings.c_cflag &= ~CSTOPB;   /* CSTOPB = 2 Stop bits,here it is cleared so 1 Stop bit */
    SerialPortSettings.c_cflag &= ~CSIZE;	 /* Clears the mask for setting the data size             */
    SerialPortSettings.c_cflag |= CS8;      /* Set the data bits = 8                                 */

    SerialPortSettings.c_cflag &= ~CRTSCTS;       /* No Hardware flow Control                         */
    SerialPortSettings.c_cflag |= CREAD | CLOCAL; /* Enable receiver,Ignore Modem Control lines       */


    SerialPortSettings.c_iflag &= ~(IXON | IXOFF | IXANY);          /* Disable XON/XOFF flow control both i/p and o/p */
    SerialPortSettings.c_iflag &= ~(ICANON | ECHO | ECHOE | ISIG);  /* Non Cannonical mode                            */

    SerialPortSettings.c_oflag &= ~OPOST;/*No Output Processing*/

    /* Setting Time outs */
    SerialPortSettings.c_cc[VMIN] = 1; /* Read at least 1 characters */
    SerialPortSettings.c_cc[VTIME] = 0; /* Wait indefinitely   */

    if ((tcsetattr(fd, TCSANOW, &SerialPortSettings)) != 0) /* Set the attributes to the termios structure*/
        printf("\n  ERROR ! in Setting attributes \n");
    else
        printf("\n  BaudRate = 921600 \n  StopBits = 1 \n  Parity   = none \n");

    tcflush(fd, TCIFLUSH);

    return fd;
}

char read_buffer[100];   /* Buffer to store the data received              */

void testBNO(int fd) {

    initBNO080();
    
    int  bytes_read = 0;    /* Number of bytes read by the read() system call */
    int i = 0;
    double timeStamp = 0.0;

    while (1)
    {

        bytes_read = read(fd, &read_buffer[i], 1); /* Read the data */
        if (read_buffer[i] == 'G')
        {
            if (0 == i)
            {
                timeStamp = current_timestamp();
                i = 1;
            }
            else
            {
                i = 0;
            }

        }
        else if (read_buffer[i] == '\n')
        {
            if (i > 36)
            {
                printf("T:%f ", timeStamp);
                int j = 0;
                for (j = 0; j < i + 1; j++)
                    printf("%c", read_buffer[j]);

                // parse the data. There might be some corrupts as "T:1486495961.133880 GRV: r:0.996 i:0.070 j:0.051 k0.079 j:0.059 k:-0.003"
            }


            i = 0;
        }
        else
        {
            i++;

        }

    }

    close(fd);
}

// read quaternion into q
void readQ(int fd, float q[4]){

    int  bytes_read = 0;    /* Number of bytes read by the read() system call */
    int i = 0;
    double timeStamp = 0.0;

    
      bytes_read = read(fd, &read_buffer[i], 1); /* Read the data */
        if (read_buffer[i] == 'G')
        {
            if (0 == i)
            {
                timeStamp = current_timestamp();
                i = 1;
            }
            else
            {
                i = 0;
            }

        }
        else if (read_buffer[i] == '\n')
        {
            if (i > 36)
            {
                printf("T:%f ", timeStamp);
                int j = 0;
                for (j = 0; j < i + 1; j++) {
                    printf("%c", read_buffer[j]);
                    q[j]= read_buffer[j];
                 }
                // parse the data. There might be some corrupts as "T:1486495961.133880 GRV: r:0.996 i:0.070 j:0.051 k0.079 j:0.059 k:-0.003"
            }


            i = 0;
        }
        else
        {
            i++;

        }



}

