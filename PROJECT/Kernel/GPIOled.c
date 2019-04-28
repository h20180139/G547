/**
 * @file   led.c
 * @author Renuka Ramrakhiani , Manas Khare
 * @date   19 April 2019
 * @brief  A kernel module for controlling a simple LED (or any signal) that is connected to
 * a GPIO. It is threaded in order that it can flash the LED.
 * The sysfs entry appears at /sys/ebb/led25
*/

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/gpio.h>       
#include <linux/kobject.h>    // Using kobjects for the sysfs bindings
#include <linux/kthread.h>    // Using kthreads for the flashing functionality
#include <linux/delay.h>      // Using this header for the msleep() function
#include <linux/interrupt.h>  
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/io.h>
#include <asm/uaccess.h>
#include <linux/uaccess.h>
#include <linux/slab.h>

static volatile uint32_t *gpio_base = NULL;


#define RPI_GPIO_P2MASK (uint32_t)0xffffffff

#define RPI_REG_BASE 0x3f000000

#define RPI_GPIO_OFFSET 0x200000
#define RPI_GPIO_SIZE 0xC0
#define RPI_GPIO_BASE (RPI_REG_BASE + RPI_GPIO_OFFSET)
#define REG_GPIO_NAME "RaspberryPi GPIO"

#define RPI_GPF_INPUT 0x00
#define RPI_GPF_OUTPUT 0x01

#define RPI_GPFSEL0_INDEX 0
#define RPI_GPFSEL1_INDEX 1
#define RPI_GPFSEL2_INDEX 2
#define RPI_GPFSEL3_INDEX 3

#define RPI_GPSET0_INDEX 7
#define RPI_GPCLR0_INDEX 10

#define GPIO_PULLNONE 0x0
#define GPIO_PULLDOWN 0x1
#define GPIO_PULLUP 0x2

static unsigned int gpioLED = 25;  /// modes LED        
static unsigned int gpioLED1 = 24; /// button interrupt LED
static unsigned int gpioButton =20; /// the button gpio for this module to GPIO20
static unsigned int numberPresses = 0;  ///store the number of button presses
static unsigned int irqNumber;

static irq_handler_t  gpio_irq_handler(unsigned int irq, void *dev_id, struct pt_regs *regs);


module_param(gpioLED, uint, S_IRUGO);       ///  Param desc. S_IRUGO can be read/not changed
MODULE_PARM_DESC(gpioLED, " GPIO LED number (default=25)");     

module_param(gpioLED1, uint, S_IRUGO);       /// Param desc. S_IRUGO can be read/not changed
MODULE_PARM_DESC(gpioLED1, " GPIO LED number (default=24)");  

static unsigned int blinkPeriod = 1000;     /// The blink period in ms
module_param(blinkPeriod, uint, S_IRUGO);   /// Param desc. S_IRUGO can be read/not changed
MODULE_PARM_DESC(blinkPeriod, " LED blink period in ms (min=1, default=1000, max=10000)");

static char ledName[7] = "ledXXX";          
static bool ledOn = 0; 
static bool on = false;                    
enum modes { OFF, ON, FLASH };              /// The available LED modes 
static enum modes mode = FLASH;             /// Default mode is flashing



static int rpi_gpio_function_set(int pin, uint32_t func) {
  int index = RPI_GPFSEL0_INDEX + pin / 10;
  uint32_t shift = (pin % 10) * 3;
  uint32_t mask = ~(0x07 << shift);
  gpio_base[index] = (gpio_base[index] & mask) | ((func & 0x07) << shift);
  return 1;
}

static void rpi_gpio_set32(uint32_t mask, uint32_t val) {
		
  gpio_base[RPI_GPSET0_INDEX] = val & mask;
  
}

static void rpi_gpio_clear32(uint32_t mask, uint32_t val) {
	
   
  gpio_base[RPI_GPCLR0_INDEX] = val & mask;
  
}

static int led_gpio_map(void) {
  if (gpio_base == NULL) {
    gpio_base = ioremap_nocache(RPI_GPIO_BASE, RPI_GPIO_SIZE);//Remap I/O memory into kernel address space (no cache). 
  }
  return 0;
}

/** @brief A callback function to display the LED mode
 *  @param kobj represents a kernel object device that appears in the sysfs filesystem
 *  @param attr the pointer to the kobj_attribute struct
 *  @param buf the buffer to which to write the number of presses
 *  @return return the number of characters of the mode string successfully displayed
 */
static ssize_t mode_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf){
   switch(mode){
      case OFF:   return sprintf(buf, "off\n");       
      case ON:    return sprintf(buf, "on\n");
      case FLASH: return sprintf(buf, "flash\n");
      default:    return sprintf(buf, "LKM Error\n"); 
   }
}

/** @brief A callback function to store the LED mode using the enum above */
static ssize_t mode_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count){
   // the count-1 is important as otherwise the \n is used in the comparison
   if (strncmp(buf,"on",count-1)==0) { mode = ON; }   // strncmp() compare with fixed number chars
   else if (strncmp(buf,"off",count-1)==0) { mode = OFF; }
   else if (strncmp(buf,"flash",count-1)==0) { mode = FLASH; }
   return count;
}

/** @brief A callback function to display the LED period */
static ssize_t period_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf){
   return sprintf(buf, "%d\n", blinkPeriod);
}

/** @brief A callback function to store the LED period value */
static ssize_t period_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count){
   unsigned int period;                     // Using a variable to validate the data sent
   sscanf(buf, "%du", &period);             // Read in the period as an unsigned int
   if ((period>1)&&(period<=10000)){        // Must be 2ms or greater, 10secs or less
      blinkPeriod = period;                 // Within range, assign to blinkPeriod variable
   }
   return period;
}

/** Use these helper macros to define the name and access levels of the kobj_attributes
 *  The kobj_attribute has an attribute attr (name and mode), show and store function pointers
 *  The period variable is associated with the blinkPeriod variable and it is to be exposed
 *  with mode 06660 using the period_show and period_store functions above
 */
static struct kobj_attribute period_attr = __ATTR(blinkPeriod, 0660, period_show, period_store);
static struct kobj_attribute mode_attr = __ATTR(mode, 0660, mode_show, mode_store);

/** The ebb_attrs[] is an array of attributes that is used to create the attribute group below.
 *  The attr property of the kobj_attribute is used to extract the attribute struct
 */
static struct attribute *ebb2_attrs[] = {
   &period_attr.attr,                       // The period at which the LED flashes
   &mode_attr.attr,                         // Is the LED on or off?
   NULL,
};

/** The attribute group uses the attribute array and a name, which is exposed on sysfs -- in this
 *  case it is gpio25, which is automatically defined in the GPIOLED_init() function below
 *  using the custom kernel parameter that can be passed when the module is loaded.
 */
static struct attribute_group attr_group = {
   .name  = ledName,                        // The name is generated in GPIOinit()
   .attrs = ebb2_attrs,                      // The attributes array defined just above
};

static struct kobject *ebb2_kobj;            /// The pointer to the kobject
static struct task_struct *task;            /// The pointer to the thread task

/** @brief The LED Flasher main kthread loop 
 *  @param arg A void pointer used in order to pass data to the thread
 *  @return returns 0 if successful
 */
static int flash(void *arg){  
   
      printk(KERN_INFO "GPIO LED : Thread started running \n");
      while(!kthread_should_stop()){           // Returns true when kthread_stop() is called
            set_current_state(TASK_RUNNING);
            if (mode==FLASH) 
            {
               ledOn = !ledOn; 
               if(ledOn)
                  rpi_gpio_set32(RPI_GPIO_P2MASK, 1 << gpioLED); 
               else  
                  rpi_gpio_clear32(RPI_GPIO_P2MASK, 1 << gpioLED);   

            }
            else if (mode==ON)  rpi_gpio_set32(RPI_GPIO_P2MASK, 1 << gpioLED); //ledOn = true;
            else rpi_gpio_clear32(RPI_GPIO_P2MASK, 1 << gpioLED);

            set_current_state(TASK_INTERRUPTIBLE);
            msleep(blinkPeriod/2);                // millisecond sleep for half of the period
      }
      printk(KERN_INFO "GPIO LED : Thread has run to completion ");
      return 0;
      }

      static irq_handler_t gpio_irq_handler(unsigned int irq, void *dev_id, struct pt_regs *regs){
      on = !on;   // Invert the LED state on each button press

      if(on)  
      rpi_gpio_set32(RPI_GPIO_P2MASK, 1 << gpioLED1); 

      else    
      rpi_gpio_clear32(RPI_GPIO_P2MASK, 1 << gpioLED1);

      printk(KERN_INFO "GPIO_TEST: Interrupt! (button state is %d)\n", gpio_get_value(gpioButton));
      numberPresses++;                         // Global counter, will be outputted when the module is unloaded
      return (irq_handler_t) IRQ_HANDLED;      // Announce that the IRQ has been handled correctly
}

/** @brief The Driver initialization function
 * In this module , the function sets up the GPIOs and the IRQ
 *  @return returns 0 if successful
 */
static int __init GPIOLED_init(void){
         int result = 0;
         int retval;

         printk(KERN_INFO "GPIO LED: Initializing the GPIO LED LKM\n");
         sprintf(ledName, "led%d", gpioLED);      

         ebb2_kobj = kobject_create_and_add("ebb2", kernel_kobj->parent); // kernel_kobj points to /sys/kernel
         if(!ebb2_kobj){
            printk(KERN_ALERT "GPIO LED: failed to create kobject\n");
            return -ENOMEM;
         }

         result = sysfs_create_group(ebb2_kobj, &attr_group);  // add the attributes to /sys/ebb/led25
         if(result) {
            printk(KERN_ALERT "GPIO LED: failed to create sysfs group\n");
            kobject_put(ebb2_kobj);          // clean up -- remove the kobject sysfs entry
            return result;
         }


         retval = led_gpio_map();
         if (retval != 0) {
            printk(KERN_ALERT "Can not use GPIO registers.\n");
            return -EBUSY;
         }

         rpi_gpio_function_set(gpioLED, RPI_GPF_OUTPUT);
         rpi_gpio_function_set(gpioButton, RPI_GPF_INPUT);
         rpi_gpio_function_set(gpioLED1, RPI_GPF_OUTPUT);
         ledOn = true;
   irqNumber=170;
   printk(KERN_INFO "GPIO_LED : The button is mapped to IRQ: %d\n", irqNumber);
   result = request_irq(irqNumber,             // The interrupt number requested
                        (irq_handler_t) gpio_irq_handler, // The pointer to the handler function below
                        IRQF_TRIGGER_RISING,   // Interrupt on rising edge 
                        "gpio_led_handler",    // Used in /proc/interrupts to identify the owner
                        NULL);                 // The *dev_id for shared interrupt lines, NULL is okay

   printk(KERN_INFO "GPIO_LED: The interrupt request result is: %d\n", result);
   
   task = kthread_run(flash, NULL, "LED_GPIO_thread");  // Start the  thread , Kthread name is LED_GPIO_thread
   if(IS_ERR(task)){                                   
      printk(KERN_ALERT "GPIO LED: failed to create the task\n");
      return PTR_ERR(task);
   }
   return result;
}

/** @brief  cleanup function
 */
static void __exit GPIOLED_exit(void){
   
      kthread_stop(task);                      // Stop the LED flashing thread
      kobject_put(ebb2_kobj);                   // clean up -- remove the kobject sysfs entry
          
      printk(KERN_INFO "GPIO LED: Goodbye from GPIO LED Driver!\n");

      free_irq(irqNumber, NULL); 
      rpi_gpio_function_set(gpioLED, 0);  	//Configure the pin as input
      rpi_gpio_function_set(gpioLED1, 0);  	//Configure the pin as input
      rpi_gpio_function_set(gpioButton, 0);  	//Configure the pin as input

      
      //gpio_free(gpioLED1);  
      //gpio_free(gpioLED); 
      //gpio_free(gpioButton);   
   
}


module_init(GPIOLED_init); // call init module
module_exit(GPIOLED_exit); // call exit module


MODULE_LICENSE("GPL");
MODULE_AUTHOR("Renuka & Manas");
MODULE_DESCRIPTION("Linux LED Driver using GPIO Subsystem");
