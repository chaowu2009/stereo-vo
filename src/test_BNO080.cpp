#include "readBNO080.h"

int main()
{
    int fd = initBNO();
    float quaternion[4];

	while(1)
	{
		readQuaternion(fd, quaternion);
	}
	
	return 0;
}
