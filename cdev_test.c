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
	unsigned int value;
	
	fd = open("/dev/new_cdev",O_RDWR);
	if(fd < 0){
		printf("open failed\n");
		return -1;
	}
	
    strcpy(wbuf,"hello new interupt");
	ret = write(fd,wbuf,strlen(wbuf));
//	printf("write count of ret to kernel is %d\n",ret);
	
	ret = read(fd,rbuf,20);
	printf("read count of ret from kernel is %d \n",ret);
	printf("the data from kernel is [%s]  read the count is %d\n",rbuf,ret);


/*		
    cmd = _IOC(LED_SET, LED_TYPE, LED_ON, LED1);
	ioctl(fd, cmd, &value);
//	printf("%#x\n",value); //GPIOE	 //加入#表示 输出形式为 0x43279fd	 类似如此的
	sleep(1);
	
	cmd = _IOC(LED_SET, LED_TYPE, LED_ON, LED2);
	ioctl(fd, cmd, &value);
//	printf("%#x\n",value); //GPIOE	 //加入#表示 输出形式为 0x43279fd	 类似如此的
	sleep(1);
	
	cmd = _IOC(LED_SET, LED_TYPE, LED_ON, LED3);
	ioctl(fd, cmd, &value);
//	printf("%#x\n",value); //GPIOE	 //加入#表示 输出形式为 0x43279fd	 类似如此的
	sleep(1);
	
	cmd = _IOC(LED_SET, LED_TYPE, LED_ON, LED4);
	ioctl(fd, cmd, &value);
//	printf("%#x\n",value); //GPIOE	 //加入#表示 输出形式为 0x43279fd	 类似如此的
	sleep(1);

	cmd = _IOC(LED_SET, LED_TYPE, LED_OFF, LED1);
	ioctl(fd, cmd, &value);
//	printf("%#x\n",value); //GPIOE	 //加入#表示 输出形式为 0x43279fd	 类似如此的
	sleep(1);
	
	cmd = _IOC(LED_SET, LED_TYPE, LED_OFF, LED2);
	ioctl(fd, cmd, &value);
//	printf("%#x\n",value); //GPIOE	 //加入#表示 输出形式为 0x43279fd	 类似如此的
	sleep(1);
	
	cmd = _IOC(LED_SET, LED_TYPE, LED_OFF, LED3);
	ioctl(fd, cmd, &value);
//	printf("%#x\n",value); //GPIOE	 //加入#表示 输出形式为 0x43279fd	 类似如此的
	sleep(1);
	
	cmd = _IOC(LED_SET, LED_TYPE, LED_OFF, LED4);
	ioctl(fd, cmd, &value);
//	printf("%#x\n",value); //GPIOE	 //加入#表示 输出形式为 0x43279fd	 类似如此的
	sleep(1);
*/
	memset(rbuf, 0, sizeof(rbuf));
	while(1)
	{
		read(fd,rbuf,4);
		printf("the rbuf is %s\n",rbuf);
		if(strcmp(rbuf, "key2") == 0)
			printf("the rbuf is key2 strcmp\n");
		else if(strcmp(rbuf,"key3") == 0)
			printf("the rbuf is key3 strcmp\n");
		memset(rbuf, 0, sizeof(rbuf));
		count ++;
		if(count == 10)
			break;
		
	}

	close(fd);

	return 0;
}


