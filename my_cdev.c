/******************************************************
** File Name   : new_cdev.c
** The author  : bin~
** E-mail      : bin007168@163.com
** Created Time: Wed 17 Oct 2018 01:53:39 PM CST
*******************************************************/
#include <linux/kernel.h>
#include <linux/module.h>
#include <asm/uaccess.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/ioport.h>
#include <asm/io.h>
#include <asm-generic/ioctl.h>

#define SIZE 1024
#define LED1     1
#define LED2     2
#define LED3     3
#define LED4     4

struct resource * led_res;
void __iomem *GPIOCOUT_VA;  //0x00
void __iomem *GPIOCOUTENB_VA; //0x04
void __iomem *GPIOCALTFN0_VA;//0x20
void __iomem *GPIOCALTFN1_VA;//0x24

void __iomem *GPIOEOUT_VA;  //0x00
void __iomem *GPIOEOUTENB_VA; //0x04
void __iomem *GPIOEALTFN0_VA;//0x20

struct cdev *my_cdev = NULL;
dev_t MyCdevNum;
int major = 241;
int minor = 0;

char Kbuf[SIZE];

int new_cdev_open(struct inode *inode, struct file * filp)
{
	printk("new_cdev_open\n");

	return 0;
}

int new_cdev_release(struct inode *inode, struct file *filp)
{
	printk("new_cdev_release\n");

	return 0;
}

ssize_t new_cdev_read(struct file *filp, char __user *buf, size_t count, loff_t *offset)
{
	int ret = 0;
	printk("new_cdev_read\n");
	if(count <= SIZE){
		ret = copy_to_user(buf, Kbuf, count);
		if(ret)
			goto copy_to_user_err;
	}else{
		goto read_size_err;
	}

	offset += count;
	return count;

copy_to_user_err:
	printk("copy_to_user_err\n");
	return -EINVAL;
read_size_err:
	printk("read_size_err");
	return -EFBIG;

}

ssize_t new_cdev_write(struct file *filp, const char __user *buf, size_t count, loff_t *offset)
{
	int ret = 0;
	printk("new_cdev_open\n");
	if(count <= SIZE){
		ret = copy_from_user(Kbuf, buf, count);
		if(ret){
			goto copy_from_user_err;
		}
	}else{
		goto write_size_err;
	}

	offset += count;
	return count;

copy_from_user_err:
	printk("copy_from_user_err\n");
	return -EINVAL;
write_size_err:
	printk("write_size_err\n");
	return -EFBIG;

}

long new_cdev_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	switch(arg)
	{
		case LED1:
			*(unsigned int *)GPIOEOUT_VA   = (*(unsigned int *)GPIOEOUT_VA  &  ~(0x1 << 13)  ) | cmd << 13 ;
			break;
			
		case LED2:
			*(unsigned int *)GPIOCOUT_VA   = (*(unsigned int *)GPIOCOUT_VA  &  ~(0x1 << 17)  ) | cmd << 17;
			break;
			
		case LED3:
			*(unsigned int *)GPIOCOUT_VA   = (*(unsigned int *)GPIOCOUT_VA  &  ~(0x1 << 8)  ) |  cmd << 8 ;
			break;
			
		case LED4:
			*(unsigned int *)GPIOCOUT_VA   = (*(unsigned int *)GPIOCOUT_VA  &  ~(0x1 << 7)  ) |  cmd << 7 ;
			break;
			
		default:
			printk("input error , try again please \n");
	}

	return 0;
}

struct file_operations new_cdev_ops = {
	.open = new_cdev_open,
	.release = new_cdev_release,
	.read = new_cdev_read,
	.write = new_cdev_write,
	.unlocked_ioctl = new_cdev_ioctl,
};

int new_cdev_init(void)
{
	//【1】定义变量，分配空间
	my_cdev = cdev_alloc();
	if(my_cdev == NULL)
		goto cdev_alloc_err;

	//【2】初始化变量和信息
	cdev_init(my_cdev, &new_cdev_ops);

	//【3】初始化主设备号和次设备号
	if(major == 0){
		if(register_chrdev_region(0, 1, "new_cdev"))
			goto register_chrdev_region_err;	
	}else{
		MyCdevNum = MKDEV(major, minor);
		if(register_chrdev_region(MyCdevNum, 1, "new_cdev"))
			goto register_chrdev_region_err; 
	}

	//【4】注册字符设备驱动
	if(cdev_add(my_cdev, MyCdevNum, 1))
		goto cdev_add_err; 

	//【5】申请GPIO内存资源
	led_res = request_mem_region(0xC001C000, 0x3000, "gec6818_led_res");
	if(led_res == NULL){
			goto request_mem_region_err;		
	}

	 //【6】对申请成功的GPIO地址，进行内存映射，获得对应的虚拟地址
	GPIOCOUT_VA = ioremap(0xC001C000, 0x3000);
	GPIOCOUTENB_VA = GPIOCOUT_VA + 0x04;
	GPIOCALTFN0_VA = GPIOCOUT_VA + 0x20;
	GPIOCALTFN1_VA = GPIOCOUT_VA + 0x24;
	
	GPIOEOUT_VA = GPIOCOUT_VA + 0x2000;
	GPIOEOUTENB_VA = GPIOEOUT_VA + 0x04;
	GPIOEALTFN0_VA = GPIOEOUT_VA + 0x20;

     //将引脚初始化成function1--GPIOC
	*(unsigned int *)GPIOCALTFN0_VA &= ~(3<<26);   //GPIOE13-->func0->GPIO
	 
	*(unsigned int *)GPIOCALTFN1_VA &= ~(3<<2); 
	*(unsigned int *)GPIOCALTFN1_VA |= (1<<2);    //GPIOC17-->func1->GPIO
	
	*(unsigned int *)GPIOCALTFN0_VA &= ~(3<<16);   //GPIOC8-->func0->GPIO
	*(unsigned int *)GPIOCALTFN0_VA |= (1<<16);
	
	*(unsigned int *)GPIOCALTFN0_VA &= ~(3<<14);   //GPIOC7-->func0->GPIO
	*(unsigned int *)GPIOCALTFN0_VA |= (1<<14);
	
	*(unsigned int *)GPIOCALTFN0_VA &= ~(3<<28);   //GPIOC14-->func0->GPIO
	*(unsigned int *)GPIOCALTFN0_VA |= (1<<28);    //beep
	
	//将CPIOC设置为OUTPUT
	*(unsigned int *)GPIOCOUTENB_VA |= (1<<17) | (1<<7) | (1<<8) ;   //led
	*(unsigned int *)GPIOEOUTENB_VA |= (1<<13);
	*(unsigned int *)GPIOCOUTENB_VA |= (1<<14);   //beep

    //  设置GPIOC output 1，LED BEEP -->off
	*(unsigned int *)GPIOCOUT_VA |= (1<<17) | (1<<7) | (1<<8);	  //led
	*(unsigned int *)GPIOEOUT_VA |= (1<<13);
	*(unsigned int *)GPIOCOUT_VA &= ~(1<<14);	  //beep

	return 0;

cdev_alloc_err:
	printk("cdev_alloc_err\n");
	return -ENOMEM;
register_chrdev_region_err:
	printk("register_chrdev_region_err\n");
	return -ENODEV;
cdev_add_err:
	printk("cdev_add_err\n");
	return -EINVAL;
request_mem_region_err:	
	printk(" request_mem_region Failed!\n");
	return -EINVAL;	
}

void new_cdev_exit(void)
{
	printk("new_cdev_exit\n");
	unregister_chrdev_region(MyCdevNum, 1);
	cdev_del(my_cdev);
	
	//设置GPIOC output 1，LED BEEP -->on
	*(unsigned int *)GPIOCOUT_VA &= ~((1<<17) | (1<<7) | (1<<8)) ;	  //led
	*(unsigned int *)GPIOEOUT_VA &= ~(1<<13);
	iounmap(GPIOCOUT_VA);
	release_mem_region(0xC001C000, 0x3000);
}


module_init(new_cdev_init);
module_exit(new_cdev_exit);

MODULE_LICENSE("GPL");
