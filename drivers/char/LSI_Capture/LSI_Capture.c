/*
 * Wrapper Driver used to control a two-channel Xilinx DMA Engine
 */
#include <linux/dmaengine.h>

#include <linux/module.h>
#include <linux/version.h>
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/kdev_t.h>
#include <linux/of.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/cdev.h>
#include <linux/clk.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/poll.h>

#include <asm/uaccess.h>
#include <linux/dma-mapping.h>
#include <xen/page.h>

#include <linux/slab.h>
#include <linux/platform_device.h>

#include "LSI.h"
#include "LSI_system.h"

#define DRIVER_NAME "LSI"
#define MODULE_NAME "LSI"
#define LSI_DEVICES 1

LIST_HEAD( LSI_full_dev_list );

/* static struct LSI_drvdata *get_elem_from_list_by_inode(struct inode *i)
   {
   struct list_head *pos;
   struct LSI_drvdata *LSI = NULL;

   list_for_each( pos, &LSI_full_dev_list ) {
   struct LSI_drvdata *tmp;
   tmp = list_entry( pos, struct LSI_drvdata, dev_list );
   if (tmp->devt == i->i_rdev)
   {
   LSI = tmp;
   break;
   }
   }
   return LSI;
   }
   */

static int LSI_open(struct inode *i, struct file *filp)
{
   struct LSI_drvdata *LSI;

   LSI = container_of(i->i_cdev, struct LSI_drvdata, cdev);

   atomic_set(&LSI->irq_count, 0);

   init_waitqueue_head(&LSI->irq_wait_queue);

   printk(KERN_DEBUG "<%s> file: open()\n", MODULE_NAME);
   filp->private_data = LSI;
   return 0;
}

static int LSI_release(struct inode *i, struct file *f)
{
   struct LSI_drvdata *LSI;

   LSI = container_of(i->i_cdev, struct LSI_drvdata, cdev);

   //   LSI_write_reg(LSI, R_INTERRUPT_ADDR, K_DISABLE_INTERRUPT);
   //   LSI_write_reg(LSI, R_MODE_CONFIG_ADDR, MODE_PPS_DEBUG);

   printk(KERN_DEBUG "<%s> file: close()\n", MODULE_NAME);
   return 0;
}

static int LSI_mmap(struct file *filp, struct vm_area_struct *vma)
{
   struct LSI_drvdata *LSI = filp->private_data;
   int result;
   unsigned long requested_size;

   requested_size = vma->vm_end - vma->vm_start;

   printk(KERN_DEBUG "<%s> file: mmap()\n", MODULE_NAME);
   printk(KERN_DEBUG
         "<%s> file: memory size reserved: %d, mmap size requested: %lu\n",
         MODULE_NAME, DMA_LENGTH, requested_size);

   if (requested_size > DMA_LENGTH) {
      printk(KERN_DEBUG "<%s> Error: %d reserved != %lu requested)\n",
            MODULE_NAME, DMA_LENGTH, requested_size);

      return -EAGAIN;
   }

   vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);

   result = remap_pfn_range(vma, vma->vm_start,LSI->dma_handle >> PAGE_SHIFT, requested_size, vma->vm_page_prot);


   if (result) {
      printk(KERN_DEBUG "<%s> Error: in calling remap_pfn_range: returned %d\n",
            MODULE_NAME, result);

      return -EAGAIN;
   }

   printk(KERN_DEBUG "<%s> : mmap complete \n",MODULE_NAME);
   return 0;
}

static ssize_t LSI_read(struct file *f, char __user * buf, size_t
      len, loff_t * off)
{
   printk(KERN_INFO "<%s> file: read()\n", MODULE_NAME);

   if (len > 65536)
   {
      return 0;
   }

   return len;
}

static long LSI_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
   struct LSI_drvdata *LSI = filp->private_data;
   //   int __user *ip = (int __user *)arg;
   void  *arg_ptr = (void *)arg;
   long  ret = 0;
   unsigned int val;
  // unsigned int s2mm_status, timeout,val;
   //   struct LSI_read_data_struct read_cmd;
   struct LSI_debug_struct debug_cmd;
   struct LSI_cmd_struct user_cmd;
   struct LSI_pn9_struct user_pn9;
   struct LSI_scale_struct user_scale;
   struct LSI_gpio_struct user_gpio;

   //printk(KERN_DEBUG "<%s> ioctl: entered LSI_ioctl\n", MODULE_NAME);

   switch (cmd) {
      case LSI_USER_RESET:
         printk(KERN_DEBUG "LSI_USER_RESET: Asserting FPGA_RESET bit\n");
         LSI_write_reg(LSI, R_MODE_CONFIG_ADDR, FPGA_RESET);
         LSI->config_state = FPGA_RESET;
         udelay(10);

         printk(KERN_DEBUG "LSI_USER_RESET: Deasserting FPGA_RESET bit\n");
         LSI_write_reg(LSI, R_MODE_CONFIG_ADDR, 0);
         LSI->config_state = 0;
         udelay(10);

         printk(KERN_DEBUG "LSI_USER_RESET: FPGA Reset complete\n");

         return 0;

      case LSI_USER_DMA_RESET:
         LSI_write_reg(LSI, R_MODE_CONFIG_ADDR, LSI->config_state|DMA_RESET);
         udelay(10);
         LSI_write_reg(LSI, R_MODE_CONFIG_ADDR, LSI->config_state);
         udelay(10);
         return 0;


      case LSI_USER_SET_MODE:
         if (copy_from_user(&user_cmd, arg_ptr, sizeof(user_cmd))) {
            printk(KERN_DEBUG "LSI_REG_DEBUG: copy failed\n");

            return -EFAULT;
         }
         ret = LSI_Set_User_Mode(LSI, &user_cmd);
         return ret;

      case LSI_USER_SET_ADDRESS:
         LSI_write_reg(LSI, R_CAPT_DMA_ADDR, (LSI->dma_handle + arg));
         return 0;

      case LSI_USER_SET_PN9_TEST:
         if (copy_from_user(&user_pn9, arg_ptr, sizeof(user_pn9))) {
            printk(KERN_DEBUG "LSI_USER_SET_PN9_TEST: copy failed\n");
         }
         LSI_write_reg(LSI, R_ADC_CONTROL_ADDR, user_pn9.command);
         user_pn9.status[0] = LSI_read_reg(LSI,R_ADC_STATUS_0_ADDR);
         user_pn9.status[1] = LSI_read_reg(LSI,R_ADC_STATUS_1_ADDR);
         user_pn9.status[2] = LSI_read_reg(LSI,R_ADC_STATUS_2_ADDR);
         user_pn9.status[3] = LSI_read_reg(LSI,R_ADC_STATUS_3_ADDR);
         user_pn9.status[4] = LSI_read_reg(LSI,R_ADC_STATUS_4_ADDR);
         user_pn9.command = LSI_read_reg(LSI,R_ADC_CONTROL_ADDR);

         if (copy_to_user(arg_ptr, &user_pn9, sizeof(user_pn9))) {
            printk(KERN_DEBUG "LSI_USER_SET_PN9_TEST: copy_to_user failed\n");
            return -EFAULT;
         }
         return 0;

      case LSI_USER_READ_PN9_TEST:
         user_pn9.status[0] = LSI_read_reg(LSI,R_ADC_STATUS_0_ADDR);
         user_pn9.status[1] = LSI_read_reg(LSI,R_ADC_STATUS_1_ADDR);
         user_pn9.status[2] = LSI_read_reg(LSI,R_ADC_STATUS_2_ADDR);
         user_pn9.status[3] = LSI_read_reg(LSI,R_ADC_STATUS_3_ADDR);
         user_pn9.status[4] = LSI_read_reg(LSI,R_ADC_STATUS_4_ADDR);
         user_pn9.command = LSI_read_reg(LSI,R_ADC_CONTROL_ADDR);

         if (copy_to_user(arg_ptr, &user_pn9, sizeof(user_pn9))) {
            printk(KERN_DEBUG "LSI_USER_READ_PN9_TEST: copy_to_user failed\n");
            return -EFAULT;
         }
         return 0;

      case LSI_USER_INIT_LMK03000:
         lmk03000_init(LSI,arg);
         return 0;

      case LSI_USER_TRIG_PPS:
         if ((arg != MODE_TRIGGER_PPS)&&(arg != GENERATE_PPS)) {
            printk(KERN_DEBUG "<%s> : LSI_USER_TRIG_PPS error\n",MODULE_NAME);
            return -1;
         }
         val = (LSI->config_state|arg) & ~START_DMA;

         printk(KERN_DEBUG "<%s> : started trigger, write = 0x%x\n",MODULE_NAME,val);

         LSI_write_reg(LSI, R_MODE_CONFIG_ADDR, val);

         return ret;

      case LSI_USER_SPI_ACCESS:
         ret = LSI_SPI_Access(LSI, arg_ptr);
         return ret;

      case LSI_USER_STATUS:
         ret = LSI_Status(LSI);
         if (copy_to_user(arg_ptr, &ret, sizeof(u32))) {
            return -EFAULT;
         }
         return 0;
      case LSI_USER_VERSION:

         ret = (LSI_VERSION_MAJOR<<8)|LSI_VERSION_MINOR;
         if (copy_to_user(arg_ptr, &ret, sizeof(u32))) {
            return -EFAULT;
         }
         return 0;

      case LSI_USER_FPGA_VERSION:

         val = LSI_read_reg(LSI,R_FPGA_VERSION_ADDR);
         if ((val & 0xffff0000) != 0x87650000) {
            printk(KERN_DEBUG "<%s> : Read version failed !!!, read = 0x%x\n",MODULE_NAME,val);
            return -EFAULT;
         }
         val &=0x0000ffff;

         if (copy_to_user(arg_ptr, &val, sizeof(u32))) {
            return -EFAULT;
         }
         return 0;

      case LSI_USER_SET_INTERRUPT:
         if (arg == ENABLE_INTERRUPT)
            LSI_write_reg(LSI, R_INTERRUPT_ADDR, K_CLEAR_INTERRUPT);
         else
            LSI_write_reg(LSI, R_INTERRUPT_ADDR, K_DISABLE_INTERRUPT);

         return 0;

      case LSI_USER_GET_SEM:
         ret = atomic_read(&LSI->semaphore);
         if (copy_to_user(arg_ptr, &ret, sizeof(u32))) {
            return -EFAULT;
         }
         return 0;

      case LSI_USER_SET_SEM:
         atomic_set(&LSI->semaphore, arg);
         return 0;

      case LSI_USER_REG_DEBUG:

         if (copy_from_user(&debug_cmd, arg_ptr, sizeof(debug_cmd))) {
            printk(KERN_DEBUG "LSI_REG_DEBUG: copy failed\n");

            return -EFAULT;
         }

         if (debug_cmd.cmd == LSI_DEBUG_WRITE){
            LSI_write_reg(LSI, debug_cmd.reg, debug_cmd.data);
            return 0;
         }

         if (debug_cmd.cmd == LSI_DEBUG_READ)
            debug_cmd.data = LSI_read_reg(LSI, debug_cmd.reg);

         if (copy_to_user(arg_ptr, &debug_cmd, sizeof(debug_cmd))) {
            return -EFAULT;
         }
         return 0;

       case LSI_USER_SET_INPUT_SCALE:

         if (copy_from_user(&user_scale, arg_ptr, sizeof(user_scale))) {
            printk(KERN_DEBUG "LSI_REG_DEBUG: copy failed\n");

            return -EFAULT;
         }

         if (user_scale.channel > 39) {
            return -EFAULT;
         }

         LSI_write_reg(LSI, R_OFFSET_BASE_ADDR + user_scale.channel*4, user_scale.offset);
         LSI_write_reg(LSI, R_SCALE_BASE_ADDR + user_scale.channel*4, user_scale.gain);
          return 0;

     case LSI_USER_WRITE_TAPS:
         ret = LSI_Write_Adc_Taps(LSI, arg_ptr);
         return ret;

     case LSI_USER_READ_TAPS:
         ret = LSI_Read_Adc_Taps(LSI, arg_ptr);
         return ret;

     case LSI_USER_READ_GPIO:
         user_gpio.data0 = LSI_read_gpio(LSI, 0x60);
         user_gpio.data1 = LSI_read_gpio(LSI, 0x64);
         printk(KERN_DEBUG "LSI_GPIO: 0x204 = 0x%08x\n",LSI_read_gpio(LSI, 0x204));
         printk(KERN_DEBUG "LSI_GPIO: 0x244 = 0x%08x\n",LSI_read_gpio(LSI, 0x244));
         if (copy_to_user(arg_ptr, &user_gpio, sizeof(user_gpio))) {
            return -EFAULT;
         }
         return 0;

      default:
         break;
   }

   return ret;
}

static unsigned int LSI_poll(struct file *filp, poll_table *ptp)
{
   struct LSI_drvdata *LSI = filp->private_data;
   unsigned int mask = 0;

   poll_wait(filp, &LSI->irq_wait_queue, ptp);

   if (atomic_read(&LSI->semaphore)) {
      mask |= (POLLIN | POLLRDNORM);
   }

   return mask;
}

/**
 * LSI_isr() - The main interrupt handler.
 * @irq:	The interrupt number.
 * @data:	Pointer to the driver data structure.
 * returns: IRQ_HANDLED after the interrupt is handled.
 **/
static irqreturn_t LSI_isr(int irq, void *data)
{
   struct LSI_drvdata *LSI = data;

   spin_lock(&LSI->lock);

   LSI->int_status = LSI_read_reg(LSI, R_LSI_STATUS) & (BIT_S2MM_ERR|BIT_MM2S_RD_CMPLT);

   // clear interrupt
   LSI_write_reg(LSI, R_INTERRUPT_ADDR,K_CLEAR_INTERRUPT);

   atomic_inc(&LSI->irq_count);
   atomic_inc(&LSI->semaphore);

   // wake up the irq wait queue to notify processes using select/poll/epoll.
   wake_up_interruptible(&LSI->irq_wait_queue);

   //   LSI_write_reg(LSI, R_INTERRUPT_ADDR,K_DISABLE_INTERRUPT);

   spin_unlock(&LSI->lock);

   return IRQ_HANDLED;
}

static const struct file_operations LSI_fops = {
   .owner = THIS_MODULE,
   .read = LSI_read,
   .poll = LSI_poll,
   .mmap = LSI_mmap,
   .unlocked_ioctl = LSI_ioctl,
   .open = LSI_open,
   .release = LSI_release,
};



static const struct of_device_id LSI_of_match_table[] = {
   { .compatible = "kutu,axi4-LSI-controller-1.00-a", (void *)&LSI_fops },
   { },
};
MODULE_DEVICE_TABLE(of, LSI_of_match_table);

static int LSI_probe(struct platform_device *pdev)
{
   const struct of_device_id *id;
   struct resource *mem;
   struct LSI_drvdata *LSI;
   dev_t devt;
   int ret;
   struct device *dev;

   if (!pdev->dev.of_node)
      return -ENODEV;

   id = of_match_node(LSI_of_match_table, pdev->dev.of_node);
   if (!id)
      return -EINVAL;

   LSI = devm_kzalloc(&pdev->dev, sizeof(*LSI), GFP_KERNEL);
   if (!LSI)
      return -ENOMEM;

   dev_info(&pdev->dev, "Kutu LSI trying to call platform get resource\n");

   mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
   LSI->base = devm_ioremap_resource(&pdev->dev, mem);

   LSI->gpio_base = ioremap(0xe000a000, 1024);


   if (IS_ERR(LSI->base))
      return PTR_ERR(LSI->base);

   // reset fgpa
   LSI_write_reg(LSI, R_MODE_CONFIG_ADDR, FPGA_RESET);
   udelay(5);
   LSI->config_state = 0;
   LSI_write_reg(LSI, R_MODE_CONFIG_ADDR, (LSI->config_state));
   udelay(5);
   // setup clock and reset fpga
   lmk03000_init(LSI,100);
   LSI_write_reg(LSI, R_MODE_CONFIG_ADDR, FPGA_RESET);
   udelay(5);
   LSI->config_state = 0;
   LSI_write_reg(LSI, R_MODE_CONFIG_ADDR, (LSI->config_state));
   udelay(5);
   atomic_set(&LSI->semaphore, 0);
   LSI->int_status = 0;
   LSI_write_reg(LSI, R_INTERRUPT_ADDR,K_DISABLE_INTERRUPT);

   dev_info(&pdev->dev, "Kutu LSI finished call to platform get resource\n");

   dev_info(&pdev->dev, "Kutu LSI trying to allocate irq\n");

   LSI->irq = platform_get_irq(pdev, 0);
   ret = devm_request_irq(&pdev->dev, LSI->irq, &LSI_isr, 0, dev_name(&pdev->dev), LSI);
   if (ret) {
      dev_err(&pdev->dev, "No IRQ available");
      return ret;
   }

   dev_info(&pdev->dev, "Kutu LSI successfully setup irq\n");

   platform_set_drvdata(pdev, LSI);
   spin_lock_init(&LSI->lock);
   mutex_init(&LSI->mutex);

   LSI->is_open = 0;
   LSI->dma_done = 0;
   LSI->error_status = 0;
   dev_info(&pdev->dev, "ioremap %pa to %p\n", &mem->start, LSI->base);

   LSI->clk = devm_clk_get(&pdev->dev, NULL);
   if (IS_ERR(LSI->clk)) {
      ret = PTR_ERR(LSI->clk);
      goto failed5;
   }
   dev_info(&pdev->dev, "Successfully got device clock\n");
   dev_info(&pdev->dev, "device clock rate is %ld\n", clk_get_rate(LSI->clk));

   //
   // reset LSI device
   //

   ret = alloc_chrdev_region(&devt, 0, LSI_DEVICES, DRIVER_NAME);
   if (ret < 0)
      goto failed5;
   dev_info(&pdev->dev, "Successfully allocated chrdev region\n");

   LSI->devt = devt;

   cdev_init(&LSI->cdev, &LSI_fops);
   LSI->cdev.owner = THIS_MODULE;
   ret = cdev_add(&LSI->cdev, devt, 1);
   if (ret) {
      dev_err(&pdev->dev, "cdev_add() failed\n");
      goto failed6;
   }

   LSI->class = class_create(THIS_MODULE, DRIVER_NAME);
   if (IS_ERR(LSI->class)) {
      dev_err(&pdev->dev, "failed to create class\n");
      goto failed6;
   }

   dev = device_create(LSI->class, &pdev->dev, devt, LSI, DRIVER_NAME);
   if (IS_ERR(dev)) {
      dev_err(&pdev->dev, "unable to create device\n");
      goto failed7;
   }
   dev_info(&pdev->dev, "Successfully created device\n");

   //
   // allocate mmap area
   //
   LSI->dma_addr = dma_alloc_coherent(NULL, DMA_LENGTH, &LSI->dma_handle, GFP_KERNEL);

   dev_info(&pdev->dev, "dma_addr = 0x%x, dma_handle = 0x%x\n",(u32)LSI->dma_addr,(u32)LSI->dma_handle);
   dev_info(&pdev->dev, "LSI base = 0x%x\n",(u32)LSI->base);

   if (!LSI->dma_addr) {
      printk(KERN_ERR "<%s> Error: allocating dma memory failed\n", MODULE_NAME);

      ret = -ENOMEM;
      goto failed8;
   }
   dev_info(&pdev->dev, "Successfully allocated dma memory\n");

   //platform_driver_register(pdev);
   dev_info(&pdev->dev, "Kutu LSI finished loading driver\n");

   return 0;

failed8:
   device_destroy(LSI->class, LSI->devt);
failed7:
   class_destroy(LSI->class);
failed6:
   /* Unregister char driver */
   unregister_chrdev_region(devt, LSI_DEVICES);
failed5:

   return ret;
}

static int LSI_remove(struct platform_device *pdev)
{
   struct LSI_drvdata *LSI;

   LSI = platform_get_drvdata(pdev);

   if (!LSI)
      return -ENODEV;

   unregister_chrdev_region(LSI->devt, LSI_DEVICES);

   //	sysfs_remove_group(&pdev->dev.kobj, &LSI_attr_group);

   device_destroy(LSI->class, LSI->devt);
   class_destroy(LSI->class);
   cdev_del(&LSI->cdev);
   clk_unprepare(LSI->clk);

   /* free mmap area */
   if (LSI->dma_addr) {
      dma_free_coherent(NULL, DMA_LENGTH, LSI->dma_addr, LSI->dma_handle);
   }

   return 0;
}

static struct platform_driver LSI_driver = {
   .probe = LSI_probe,
   .remove = LSI_remove,
   .driver = {
      .name = "LSI",
      .of_match_table = LSI_of_match_table,
   },
};
module_platform_driver(LSI_driver);

MODULE_AUTHOR("Greg Smart <Greg.Smart@kutu.com.au>");
MODULE_DESCRIPTION("LSI Linux driver");
MODULE_LICENSE("GPL v2");
