#include "BNO080.h"

int main()
{
    int fd = initBNO080();
    float q[4];

    while(1)
	{
	readQ(fd, q);
       // printf("%f \t%f \t%f \t%f \n", q[0],q[1],q[2],q[3]);
  
	}
	
	return 0;
}
