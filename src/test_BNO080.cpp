#include "readBNO080.h"

int main()
{
    int fd = initBNO();
    float quaternion[4];

	while(1)
	{
		readQ(fd, quaternion);
	}
	
	return 0;
}
