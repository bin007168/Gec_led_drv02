#include <sys/types.h>
#include <unistd.h>
#include <stdio.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <string.h>
#include <sys/ioctl.h>
#include <dirent.h>
#include <stdlib.h>
#include <pthread.h>


void *dev_thread(void *arg)
{
	int fd;
	fd = open("/dev/new_cdev",O_RDWR);
	if(fd == -1){
		perror("open");
	}else{
		printf("open success,the fd is %d\n",fd);
	}

}

int main(int argc,char *argv[])
{
	int count = 10;
	pthread_t pid;

	printf("entry the pthread\n");
	for(; count>0 ;count --)
	{
		pthread_create(&pid,NULL,dev_thread,NULL);
	}

	return 0;
}


