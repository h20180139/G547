#include<linux/init.h>
#include<linux/module.h>
#include<linux/version.h>
#include<linux/kernel.h>
#include<linux/types.h>
#include<linux/kdev_t.h>
#include<linux/cdev.h>
#include<linux/fs.h>
#include<linux/device.h>
#include <linux/types.h>
#include <linux/random.h>
#include <linux/uaccess.h>
static dev_t accel;

static struct cdev x_dev;
static struct cdev y_dev;
static struct cdev z_dev;

static struct class *cls;

dev_t x,y,z;
uint16_t val;

uint16_t getrand(void){
	uint16_t retval;
	get_random_bytes(&retval, 2);
	return retval;
}

static int x_open(struct inode *i,struct file *f)
{
printk(KERN_INFO "x:open()\n");
return 0;
}
static int x_close(struct inode *i,struct file *f)
{
printk(KERN_INFO "x:close()\n");
return 0;
}
static ssize_t x_read(struct file *f,char __user *buf,size_t len,loff_t *off)
{
printk(KERN_INFO "x:read()\n");
val=getrand()&0x3ff;
printk(KERN_INFO "%d\n",val);
copy_to_user(buf, &val, 2);
return 0;
}
static int y_open(struct inode *i,struct file *f)
{
printk(KERN_INFO "y:open()\n");
return 0;
}
static int y_close(struct inode *i,struct file *f)
{
printk(KERN_INFO "y:close()\n");
return 0;
}
static ssize_t y_read(struct file *f,char __user *buf,size_t len,loff_t *off)
{
printk(KERN_INFO "y:read()\n");
val=getrand()&0x3ff;
printk(KERN_INFO "%d\n",val);
copy_to_user(buf, &val, 2);
return 0;
}
static int z_open(struct inode *i,struct file *f)
{
printk(KERN_INFO "z:open()\n");
return 0;
}
static int z_close(struct inode *i,struct file *f)
{
printk(KERN_INFO "z:close()\n");
return 0;
}
static ssize_t z_read(struct file *f,char __user *buf,size_t len,loff_t *off)
{
printk(KERN_INFO "z:read()\n");
val=getrand()&0x3ff;
printk(KERN_INFO "%d\n",val);
copy_to_user(buf, &val, 2);
printk(KERN_INFO "%d\n",val);
return 0;
}



//ssize_t (*write) (struct file *, const char __user *, size_t, loff_t *);


static struct file_operations xops =
{
.owner=THIS_MODULE,
.open=x_open,
.release=x_close,
.read=x_read,
};
static struct file_operations yops =
{
.owner=THIS_MODULE,
.open=y_open,
.release=y_close,
.read=y_read,
};
static struct file_operations zops =
{
.owner=THIS_MODULE,
.open=z_open,
.release=z_close,
.read=z_read,
};



static __init int demo_init(void)
{
	printk(KERN_INFO "Accelerometer registered\n\n");
	if((alloc_chrdev_region(&accel,0,3,"accelerometer"))<0) 
		{
			return -1;
		}
	x=MKDEV(MAJOR(accel), MINOR(accel)+0);
	y=MKDEV(MAJOR(accel), MINOR(accel)+1);
	z=MKDEV(MAJOR(accel), MINOR(accel)+2);
	if((cls=class_create(THIS_MODULE,"chardev"))==NULL)
		{
			unregister_chrdev_region(accel,3);
			return -1;
		}
	cdev_init(&x_dev,&xops);
	device_create(cls,NULL,x,NULL,"x-axis");
	if(cdev_add(&x_dev,x,1)<0)
		{
			device_destroy(cls, x);
			class_destroy(cls);
			unregister_chrdev_region(accel, 3);
			return -1;
		}
	cdev_init(&y_dev,&yops);
	device_create(cls,NULL,y,NULL,"y-axis");
	if(cdev_add(&y_dev,y,1)<0)
		{
			cdev_del(&y_dev);
			device_destroy(cls, y);
			device_destroy(cls, x);
			class_destroy(cls);
			unregister_chrdev_region(accel, 3);
			return -1;
		}
	cdev_init(&z_dev,&zops);
	device_create(cls,NULL,z,NULL,"z-axis");
	if(cdev_add(&z_dev,z,1)<0)
		{
			cdev_del(&y_dev);
			cdev_del(&x_dev);
			device_destroy(cls, z);
			device_destroy(cls, y);
			device_destroy(cls, x);
			class_destroy(cls);
			unregister_chrdev_region(accel, 3);
			return -1;
		}
	return 0;

}
static __exit void demo_exit(void)
{
//printk(KERN_ALERT "BYE WORLD\n");
cdev_del(&x_dev);
cdev_del(&y_dev);
cdev_del(&z_dev);
device_destroy(cls,x);
device_destroy(cls,y);
device_destroy(cls,z);
class_destroy(cls);
unregister_chrdev_region(accel,3);
printk(KERN_INFO "Accelerometer unregistered\n\n");
}

module_init(demo_init);
module_exit(demo_exit);

MODULE_DESCRIPTION("Assignment 1");
MODULE_AUTHOR("Shyam krishnan KV");
MODULE_LICENSE("GPL");
