#include <sys/types.h>
#include <unistd.h>
#include <stdio.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <string.h>
#include <sys/ioctl.h>

#define LED1 1
#define LED2 2
#define LED3 3
#define LED4 4
#define LED_ON   0
#define LED_OFF  1

#define LED_SET  0
#define LED_GET  1
#define LED_TYPE 0


int fd;  //文件描述符

int main(int argc,char *argv[])
{
	int ret,cmd;
	int count = 0;
	char wbuf[50];
	char rbuf[50];
	
	fd = open("/dev/new_cdev",O_RDWR);
	if(fd < 0){
		printf("open failed\n");
		return -1;
	}else{
		printf("open fd success 14:44\n");
	}
	
    strcpy(wbuf,"hello new");
	ret = write(fd,wbuf,strlen(wbuf));
	printf("write count of ret to kernel is %d\n",ret);
	
	ret = read(fd,rbuf,20);
	printf("read count of ret from kernel is %d \n",ret);
	printf("the data from kernel is [%s]  read the count is %d\n",rbuf,ret);

	_IOC(dir, type, nr, size)
	while(1)
	{
		cmd = _IOC(LED_SET, LED_TYPE, LED_ON, LED1);
		ioctl(fd, cmd, 0);
		sleep(1);
		cmd = _IOC(LED_SET, LED_TYPE, LED_ON, LED2);
		ioctl(fd, cmd, 0);
		sleep(1);
		cmd = _IOC(LED_SET, LED_TYPE, LED_ON, LED3);
		ioctl(fd, cmd, 0);
		sleep(1);
		cmd = _IOC(LED_SET, LED_TYPE, LED_ON, LED4);
		ioctl(fd, cmd, 0);
		sleep(1);

		cmd = _IOC(LED_SET, LED_TYPE, LED_OFF, LED1);
		ioctl(fd, cmd, 0);
		sleep(1);
		cmd = _IOC(LED_SET, LED_TYPE, LED_OFF, LED2);
		ioctl(fd, cmd, 0);
		sleep(1);
		cmd = _IOC(LED_SET, LED_TYPE, LED_OFF, LED3);
		ioctl(fd, cmd, 0);
		sleep(1);
		cmd = _IOC(LED_SET, LED_TYPE, LED_OFF, LED4);
		ioctl(fd, cmd, 0);
		sleep(1);
	}

	close(fd);
      _IOC_DIR(nr)
	return 0;
}


