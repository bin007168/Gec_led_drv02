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
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <mach/s5p6818_irq.h>
#include <cfg_type.h>
#include <linux/wait.h>
#include <linux/sched.h>


#define SIZE     1024
#define LED1     1
#define LED2     2
#define LED3     3
#define LED4     4

#define LED_ON   0
#define LED_OFF  1
#define LED_SET  0
#define LED_GET  1
#define LED_TYPE 0


struct resource * led_res;
void __iomem *GPIOAOUT_VA;  //0x00
void __iomem *GPIOAOUTENB_VA; //0x04
void __iomem *GPIOAALTFN0_VA;//0x20
void __iomem *GPIOAALTFN1_VA;//0x24

void __iomem *GPIOBOUT_VA;  //0x00
void __iomem *GPIOBOUTENB_VA; //0x04
void __iomem *GPIOBALTFN0_VA;//0x20
void __iomem *GPIOBALTFN1_VA;//0x24

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

int condition = 0;  //进程唤醒标志

DECLARE_WAIT_QUEUE_HEAD(cdev_wq);  //定义一个等待队列


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
	
	printk("wait_event(cdev_wq, condition) ahead\n");
	wait_event(cdev_wq, condition);
	printk("wait_event(cdev_wq, condition) after\n");
	
	if(count <= SIZE){
		ret = copy_to_user(buf, Kbuf, count);
		if(ret)
			goto copy_to_user_err;
	}else{
		goto read_size_err;
	}
	condition = 0;   //重新将标志位变成0，以便重复检测

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
	//将用户空间传过来的地址给 value
	unsigned int  __user * value =	(unsigned int  __user *)arg;

	if(_IOC_TYPE(cmd) == LED_TYPE){
		switch(_IOC_SIZE(cmd))
		{
			case LED1:
			//	*(unsigned int *)GPIOEOUT_VA   = (*(unsigned int *)GPIOEOUT_VA  &  ~(0x1 << 13)  ) | _IOC_NR(cmd) << 13 ;
				writel(((readl(GPIOEOUT_VA) &  ~(0x1 << 13)) | (_IOC_NR(cmd) << 13) ),GPIOEOUT_VA);
				if(copy_to_user(value,GPIOEOUT_VA,sizeof(unsigned int)))
					printk("copy to user failed\n");
				break;
			
			case LED2:
			//	*(unsigned int *)GPIOCOUT_VA   = (*(unsigned int *)GPIOCOUT_VA  &  ~(0x1 << 17)  ) | _IOC_NR(cmd) << 17;
				writel(((readl(GPIOCOUT_VA) &  ~(0x1 << 17)) | (_IOC_NR(cmd) << 17) ),GPIOCOUT_VA);
				if(copy_to_user(value,GPIOCOUT_VA,sizeof(unsigned int)))
					printk("copy to user failed\n");
				break;
			
			case LED3:
			//	*(unsigned int *)GPIOCOUT_VA   = (*(unsigned int *)GPIOCOUT_VA  &  ~(0x1 << 8)  ) |  _IOC_NR(cmd) << 8 ;
				writel(((readl(GPIOCOUT_VA) &  ~(0x1 << 8)) | (_IOC_NR(cmd) << 8) ),GPIOCOUT_VA);
				if(copy_to_user(value,GPIOCOUT_VA,sizeof(unsigned int)))
					printk("copy to user failed\n");
				break;
			
			case LED4:
			//	*(unsigned int *)GPIOCOUT_VA   = (*(unsigned int *)GPIOCOUT_VA  &  ~(0x1 << 7)  ) |  _IOC_NR(cmd) << 7 ;
				writel(((readl(GPIOCOUT_VA) &  ~(0x1 << 7)) | (_IOC_NR(cmd) << 7) ),GPIOCOUT_VA);
				if(copy_to_user(value,GPIOCOUT_VA,sizeof(unsigned int)))
					printk("copy to user failed\n");
				break;
			
			default:
				printk("input error , try again please \n");
		}
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

irqreturn_t  new_cdev_handler(int irqno, void *arg)
{
	int value;
	memset(Kbuf, 0, sizeof(Kbuf));
	switch(irqno)
	{
		case IRQ_GPIO_A_START+28:
			
				// 获取当前的引脚电平状态
			    value =	(readl(GPIOAOUT_VA) >> 28) & 0x1;
			    // 延时
			    mdelay(100);
			    // 再次获取当前的引脚电平状态进行比较
			    if(value ==	((readl(GPIOAOUT_VA) >> 28) & 0x1))  
			    {
			    	while((readl(GPIOAOUT_VA) >> 28) & 0x1);
			   		strcpy(Kbuf,"key2");
					printk("key2\n");
			   		condition = 1;
			   		wake_up(&cdev_wq);
			    }  
				break;
			
		case IRQ_GPIO_B_START+30:

			   // 获取当前的引脚电平状态
			   value = (readl(GPIOBOUT_VA) >> 30) & 0x1;
			   // 延时
			   mdelay(100);
			   // 再次获取当前的引脚电平状态进行比较
			   if(value ==	((readl(GPIOBOUT_VA) >> 30) & 0x1))  
			   {	
			   		while((readl(GPIOBOUT_VA) >> 30) & 0x1);
			   		strcpy(Kbuf,"key3");
			   		printk("key3\n");
			  		condition = 1;
			   		wake_up(&cdev_wq);  		
			   } 
			   break;

		default: 
			   	break;
			
	}
	       
   return IRQ_HANDLED;
}

int new_cdev_init(void)
{
	//【1】定义变量，分配空间
	int value;
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
	led_res = request_mem_region(0xC001A000, 0x5000, "gec_GPIO");
	if(led_res == NULL){
			goto request_mem_region_err;		
	}

	 //【6】对申请成功的GPIO地址，进行内存映射，获得对应的虚拟地址
	GPIOAOUT_VA = ioremap(0xC001A000, 0x5000);
	GPIOAOUTENB_VA = GPIOAOUT_VA + 0x04;
	GPIOAALTFN0_VA = GPIOAOUT_VA + 0x20;
	GPIOAALTFN1_VA = GPIOAOUT_VA + 0x24;

	GPIOBOUT_VA  = GPIOAOUT_VA + 0x1000;
	GPIOBOUTENB_VA = GPIOBOUT_VA + 0x04;
	GPIOBALTFN0_VA = GPIOBOUT_VA + 0x20;
	GPIOBALTFN1_VA = GPIOBOUT_VA + 0x24;
	
	GPIOCOUT_VA =  GPIOAOUT_VA + 0x2000;
	GPIOCOUTENB_VA = GPIOCOUT_VA + 0x04;
	GPIOCALTFN0_VA = GPIOCOUT_VA + 0x20;
	GPIOCALTFN1_VA = GPIOCOUT_VA + 0x24;
	
	GPIOEOUT_VA  = GPIOCOUT_VA + 0x2000;
	GPIOEOUTENB_VA = GPIOEOUT_VA + 0x04;
	GPIOEALTFN0_VA = GPIOEOUT_VA + 0x20;



	//将引脚初始化为 GPIOA28的funtion1         GPIOA28 为 输入功能   
	writel((readl(GPIOAALTFN1_VA) & (~(3<<24))),GPIOAALTFN1_VA);        //GPIOA28 设置为GPIO模式
	writel((readl(GPIOAOUTENB_VA)  & (~ (0x1 << 28))),GPIOAOUTENB_VA); //GPIOA28 设置为GPIO 输入模式
	
	writel((readl(GPIOBALTFN0_VA) & (~(3<<18))),GPIOBALTFN0_VA);    //GPIOB9 设置为GPIO模式
	writel((readl(GPIOBOUTENB_VA)  & (~ (0x1 << 9))),GPIOBOUTENB_VA);//GPIOB9 设置为GPIO 输入模式

	writel((readl(GPIOBALTFN1_VA) & ~(3<<28)) | (1<<28),GPIOBALTFN1_VA);  //GPIOB30 设置为GPIO模式
	writel((readl(GPIOBOUTENB_VA)  & ~ (0x1 << 30)),GPIOBOUTENB_VA);      //GPIOB30 设置为GPIO 输入模式

	writel((readl(GPIOBALTFN1_VA) & ~(3<<30)) | (1<<30),GPIOBALTFN1_VA);  //GPIOB31 设置为GPIO模式
	writel((readl(GPIOBOUTENB_VA)  & ~ (0x1 << 31)),GPIOBOUTENB_VA);      //GPIOB31 设置为GPIO 输入模式

	//将引脚初始化成function1--GPIOC
	*(unsigned int *)GPIOCALTFN0_VA &= ~(3<<26);   //GPIOE13-->func0->GPIO
	 
	*(unsigned int *)GPIOCALTFN1_VA &= ~(3<<2); 
	*(unsigned int *)GPIOCALTFN1_VA |= (1<<2);    //GPIOC17-->func1->GPIO
	
	*(unsigned int *)GPIOCALTFN0_VA &= ~(3<<16);   //GPIOC8-->funcB0->GPIO
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

	//中断key2申请
	if( request_irq(IRQ_GPIO_A_START+28,          //中断源(编号)
	                    new_cdev_handler  , //中断服务程序
	                    IRQF_DISABLED | IRQF_TRIGGER_FALLING, //独占性中断，下降沿触发
	                    "gec_key2",
	                    NULL))
	                    goto request_irq_err2;

	//中断key3申请 GPIOB30
	if( request_irq(IRQ_GPIO_B_START+30,          //中断源(编号)
	                    new_cdev_handler  , //中断服务程序
	                    IRQF_DISABLED | IRQF_TRIGGER_FALLING, //独占性中断，下降沿触发
	                    "gec_key3",
	                    NULL))
	                    goto request_irq_err3;


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
request_irq_err2:
	printk("request_irq_err2 \n");
	return -EINVAL;
request_irq_err3:
	printk("request_irq_err3 \n");
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
	iounmap(GPIOAOUT_VA);
	release_mem_region(0xC001A000, 0x5000);

	//中断释放
	free_irq(IRQ_GPIO_A_START+28,   //中断源(编号)  ----->  芯片商提供
					NULL);          //中断服务程序传参
	free_irq(IRQ_GPIO_B_START+30,   //中断源(编号)  ----->  芯片商提供
	NULL);          //中断服务程序传参
}


module_init(new_cdev_init);
module_exit(new_cdev_exit);

MODULE_LICENSE("GPL");

