#include <sys/types.h>
#include <unistd.h>
#include <stdio.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <string.h>
#include <sys/ioctl.h>
#include <pthread.h>

#define LED1 1
#define LED2 2
#define LED3 3
#define LED4 4
#define LED_ON   0
#define LED_OFF  1

#define LED_ALL1 5
#define LED_ALL2 6
#define LED_ALL3 7
#define LED_ALL4 8

#define LED_SET  0
#define LED_GET  1
#define LED_TYPE 0

int fd;  //文件描述符
char led_value[4];
int cmd;
unsigned int value;
unsigned int read_value;


void *dev_thread1(void *arg)
{
	while(1)
	{
//		printf("dev_thread1\n");
		cmd = _IOC(LED_GET, LED_TYPE, 0, 0);
		ioctl(fd, cmd, &read_value);
		printf("111 led1: %d  led2: %d  led3: %d  led4:  %d\n",
				read_value&0xFF,(read_value>>8)&0xFF,(read_value>>16)&0xFF,(read_value>>24)&0xFF); 

		cmd = _IOC(LED_SET, LED_TYPE,LED_ON, LED_ALL1);
		ioctl(fd, cmd, &value);
				
		sleep(1);
	}
}

void *dev_thread2(void *arg)
{
	while(1)
	{
	//	printf("dev_thread2\n");
		cmd = _IOC(LED_GET, LED_TYPE, 0, 0);
		ioctl(fd, cmd, &read_value);
		printf("222 led1: %d  led2: %d  led3: %d  led4:  %d\n",
				read_value&0xFF,(read_value>>8)&0xFF,(read_value>>16)&0xFF,(read_value>>24)&0xFF); 

		cmd = _IOC(LED_SET, LED_TYPE,LED_ON, LED_ALL2);
		ioctl(fd, cmd, &value);
		sleep(1);
	}
}

void *dev_thread3(void *arg)
{
	while(1)
	{
	//	printf("dev_thread3\n");
		cmd = _IOC(LED_GET, LED_TYPE, 0, 0);
		ioctl(fd, cmd, &read_value);
		printf("333 led1: %d  led2: %d  led3: %d  led4:  %d\n",
				read_value&0xFF,(read_value>>8)&0xFF,(read_value>>16)&0xFF,(read_value>>24)&0xFF); 

		cmd = _IOC(LED_SET, LED_TYPE,LED_ON, LED_ALL3);
		ioctl(fd, cmd, &value);
		sleep(1);
	}
}

void *dev_thread4(void *arg)
{
	while(1)
	{
	//	printf("dev_thread4\n");
		cmd = _IOC(LED_GET, LED_TYPE, 0, 0);
		ioctl(fd, cmd, &read_value);
		printf("444 led1: %d  led2: %d  led3: %d  led4:  %d\n",
				read_value&0xFF,(read_value>>8)&0xFF,(read_value>>16)&0xFF,(read_value>>24)&0xFF); 

		cmd = _IOC(LED_SET, LED_TYPE,LED_ON, LED_ALL4);
		ioctl(fd, cmd, &value);
		sleep(1);
	}
}




int main(int argc,char *argv[])
{
	int ret;
	int count = 0;
	char wbuf[50];
	char rbuf[50];
	pthread_t pid1,pid2,pid3,pid4;
	
	fd = open("/dev/new_cdev",O_RDWR);
	if(fd < 0){
		printf("open failed\n");
		return -1;
	}
	
	ret = pthread_create(&pid1,NULL,dev_thread1,NULL);
	if(ret != 0){
		printf("pthread1 create failed\n");
		return -1;
	}else{
		printf("pthread1 create success\n");
	}

	ret = pthread_create(&pid2,NULL,dev_thread2,NULL);
	if(ret != 0){
		printf("pthread2 create failed\n");
		return -1;
	}else{
		printf("pthread2 create success\n");
	}

	ret = pthread_create(&pid3,NULL,dev_thread3,NULL);
	if(ret != 0){
		printf("pthread3 create failed\n");
		return -1;
	}else{
		printf("pthread3 create success\n");
	}

	ret = pthread_create(&pid4,NULL,dev_thread4,NULL);
	if(ret != 0){
		printf("pthread4 create failed\n");
		return -1;
	}else{
		printf("pthread4 create success\n");
	}
	
/*	cmd = _IOC(LED_SET, LED_TYPE, LED_ON, LED1);
	ioctl(fd, cmd, &value);
	printf("%#x\n",value); //GPIOE   //加入#表示 输出形式为 0x43279fd   类似如此的
	sleep(1);


	cmd = _IOC(LED_SET, LED_TYPE, LED_OFF, LED1);
	ioctl(fd, cmd, &value);
	printf("%#x\n",value); //GPIOE   //加入#表示 输出形式为 0x43279fd   类似如此的
	sleep(1);   */

	while(1);
	

	close(fd);

	return 0;
}


