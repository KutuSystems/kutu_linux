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

#include "LXS.h"
#include "LXS_system.h"
#include "lmk0482x.h"

#define DRIVER_NAME "LXS"
#define MODULE_NAME "LXS"
#define LXS_DEVICES 1

LIST_HEAD( LXS_full_dev_list );

/* static struct LXS_drvdata *get_elem_from_list_by_inode(struct inode *i)
   {
   struct list_head *pos;
   struct LXS_drvdata *LXS = NULL;

   list_for_each( pos, &LXS_full_dev_list ) {
   struct LXS_drvdata *tmp;
   tmp = list_entry( pos, struct LXS_drvdata, dev_list );
   if (tmp->devt == i->i_rdev)
   {
   LXS = tmp;
   break;
   }
   }
   return LXS;
   }
   */

static int LXS_open(struct inode *i, struct file *filp)
{
   struct LXS_drvdata *LXS;

   LXS = container_of(i->i_cdev, struct LXS_drvdata, cdev);

   atomic_set(&LXS->irq_count, 0);

   init_waitqueue_head(&LXS->irq_wait_queue);

   printk(KERN_DEBUG "<%s> file: open()\n", MODULE_NAME);
   filp->private_data = LXS;
   return 0;
}

static int LXS_release(struct inode *i, struct file *f)
{
   struct LXS_drvdata *LXS;

   LXS = container_of(i->i_cdev, struct LXS_drvdata, cdev);

   //   LXS_write_reg(LXS, R_INTERRUPT_ADDR, K_DISABLE_INTERRUPT);
   //   LXS_write_reg(LXS, R_MODE_CONFIG_ADDR, MODE_PPS_DEBUG);

   printk(KERN_DEBUG "<%s> file: close()\n", MODULE_NAME);
   return 0;
}

static int LXS_mmap(struct file *filp, struct vm_area_struct *vma)
{
   struct LXS_drvdata *LXS = filp->private_data;
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

   result = remap_pfn_range(vma, vma->vm_start,LXS->dma_handle >> PAGE_SHIFT, requested_size, vma->vm_page_prot);


   if (result) {
      printk(KERN_DEBUG "<%s> Error: in calling remap_pfn_range: returned %d\n",
            MODULE_NAME, result);

      return -EAGAIN;
   }

   printk(KERN_DEBUG "<%s> : mmap complete \n",MODULE_NAME);
   return 0;
}

static ssize_t LXS_read(struct file *f, char __user * buf, size_t
      len, loff_t * off)
{
   printk(KERN_INFO "<%s> file: read()\n", MODULE_NAME);

   if (len > 65536)
   {
      return 0;
   }

   return len;
}

static long LXS_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
   struct LXS_drvdata *LXS = filp->private_data;
   //   int __user *ip = (int __user *)arg;
   void  *arg_ptr = (void *)arg;
   long  ret = 0;
   unsigned int val;
  // unsigned int s2mm_status, timeout,val;
   //   struct LXS_read_data_struct read_cmd;
   struct LXS_debug_struct debug_cmd;
   struct LXS_cmd_struct user_cmd;
   struct LXS_scale_struct user_scale;
   struct LXS_gpio_struct user_gpio;

   //printk(KERN_DEBUG "<%s> ioctl: entered LXS_ioctl\n", MODULE_NAME);

   switch (cmd) {
      case LXS_USER_RESET:
         printk(KERN_DEBUG "LXS_USER_RESET: Asserting FPGA_RESET bit\n");
         LXS_write_reg(LXS, R_MODE_CONFIG_ADDR, FPGA_RESET);
         LXS->config_state = FPGA_RESET;
         udelay(10);

         printk(KERN_DEBUG "LXS_USER_RESET: Deasserting FPGA_RESET bit\n");
         LXS_write_reg(LXS, R_MODE_CONFIG_ADDR, 0);
         LXS->config_state = 0;
         udelay(10);

         printk(KERN_DEBUG "LXS_USER_RESET: FPGA Reset complete\n");

         return 0;

      case LXS_USER_DMA_RESET:
         LXS_write_reg(LXS, R_MODE_CONFIG_ADDR, LXS->config_state|DMA_RESET);
         udelay(10);
         LXS_write_reg(LXS, R_MODE_CONFIG_ADDR, LXS->config_state);
         udelay(10);
         return 0;


      case LXS_USER_SET_MODE:
         if (copy_from_user(&user_cmd, arg_ptr, sizeof(user_cmd))) {
            printk(KERN_DEBUG "LXS_REG_DEBUG: copy failed\n");

            return -EFAULT;
         }
         ret = LXS_Set_User_Mode(LXS, &user_cmd);
         return ret;

      case LXS_USER_SET_ADDRESS:
         LXS_write_reg(LXS, R_CAPT_DMA_ADDR, (LXS->dma_handle + arg));
         return 0;

      case LXS_USER_INIT_LMK0482X:
         lmk0482x_init(LXS);
         return 0;

      case LXS_USER_TRIG_PPS:
         if ((arg != MODE_TRIGGER_PPS)&&(arg != GENERATE_PPS)) {
            printk(KERN_DEBUG "<%s> : LXS_USER_TRIG_PPS error\n",MODULE_NAME);
            return -1;
         }
         val = (LXS->config_state|arg) & ~START_DMA;

         printk(KERN_DEBUG "<%s> : started trigger, write = 0x%x\n",MODULE_NAME,val);

         LXS_write_reg(LXS, R_MODE_CONFIG_ADDR, val);

         return ret;

      case LXS_USER_SPI_ACCESS:
         ret = LXS_SPI_Access(LXS, arg_ptr);
         return ret;

      case LXS_USER_STATUS:
         ret = LXS_Status(LXS);
         if (copy_to_user(arg_ptr, &ret, sizeof(u32))) {
            return -EFAULT;
         }
         return 0;
      case LXS_USER_VERSION:

         ret = (LXS_VERSION_MAJOR<<8)|LXS_VERSION_MINOR;
         if (copy_to_user(arg_ptr, &ret, sizeof(u32))) {
            return -EFAULT;
         }
         return 0;

      case LXS_USER_FPGA_VERSION:

         val = LXS_read_reg(LXS,R_FPGA_VERSION_ADDR);
         if ((val & 0xffff0000) != 0x87650000) {
            printk(KERN_DEBUG "<%s> : Read version failed !!!, read = 0x%x\n",MODULE_NAME,val);
            return -EFAULT;
         }
         val &=0x0000ffff;

         if (copy_to_user(arg_ptr, &val, sizeof(u32))) {
            return -EFAULT;
         }
         return 0;

      case LXS_USER_SET_INTERRUPT:
         if (arg == ENABLE_INTERRUPT)
            LXS_write_reg(LXS, R_INTERRUPT_ADDR, K_CLEAR_INTERRUPT);
         else
            LXS_write_reg(LXS, R_INTERRUPT_ADDR, K_DISABLE_INTERRUPT);

         return 0;

      case LXS_USER_GET_SEM:
         ret = atomic_read(&LXS->semaphore);
         if (copy_to_user(arg_ptr, &ret, sizeof(u32))) {
            return -EFAULT;
         }
         return 0;

      case LXS_USER_SET_SEM:
         atomic_set(&LXS->semaphore, arg);
         return 0;

      case LXS_USER_REG_DEBUG:

         if (copy_from_user(&debug_cmd, arg_ptr, sizeof(debug_cmd))) {
            printk(KERN_DEBUG "LXS_REG_DEBUG: copy failed\n");

            return -EFAULT;
         }

         if (debug_cmd.cmd == LXS_DEBUG_WRITE){
            LXS_write_reg(LXS, debug_cmd.reg, debug_cmd.data);
            return 0;
         }

         if (debug_cmd.cmd == LXS_DEBUG_READ)
            debug_cmd.data = LXS_read_reg(LXS, debug_cmd.reg);

         if (copy_to_user(arg_ptr, &debug_cmd, sizeof(debug_cmd))) {
            return -EFAULT;
         }
         return 0;

       case LXS_USER_SET_INPUT_SCALE:

         if (copy_from_user(&user_scale, arg_ptr, sizeof(user_scale))) {
            printk(KERN_DEBUG "LXS_REG_DEBUG: copy failed\n");

            return -EFAULT;
         }

         if (user_scale.channel > 39) {
            return -EFAULT;
         }

         LXS_write_reg(LXS, R_OFFSET_BASE_ADDR + user_scale.channel*4, user_scale.offset);
         LXS_write_reg(LXS, R_SCALE_BASE_ADDR + user_scale.channel*4, user_scale.gain);
          return 0;

     case LXS_USER_WRITE_BRAM:
         ret = LXS_Write_bram(LXS, arg_ptr);
         return ret;

     case LXS_USER_READ_BRAM:
         ret = LXS_Read_bram(LXS, arg_ptr);
         return ret;

     case LXS_USER_READ_GPIO:
         user_gpio.data0 = LXS_read_gpio(LXS, 0x60);
         user_gpio.data1 = LXS_read_gpio(LXS, 0x64);
         printk(KERN_DEBUG "LXS_GPIO: 0x204 = 0x%08x\n",LXS_read_gpio(LXS, 0x204));
         printk(KERN_DEBUG "LXS_GPIO: 0x244 = 0x%08x\n",LXS_read_gpio(LXS, 0x244));
         if (copy_to_user(arg_ptr, &user_gpio, sizeof(user_gpio))) {
            return -EFAULT;
         }
         return 0;

      default:
         break;
   }

   return ret;
}

static unsigned int LXS_poll(struct file *filp, poll_table *ptp)
{
   struct LXS_drvdata *LXS = filp->private_data;
   unsigned int mask = 0;

   poll_wait(filp, &LXS->irq_wait_queue, ptp);

   if (atomic_read(&LXS->semaphore)) {
      mask |= (POLLIN | POLLRDNORM);
   }

   return mask;
}

/**
 * LXS_isr() - The main interrupt handler.
 * @irq:	The interrupt number.
 * @data:	Pointer to the driver data structure.
 * returns: IRQ_HANDLED after the interrupt is handled.
 **/
static irqreturn_t LXS_isr(int irq, void *data)
{
   struct LXS_drvdata *LXS = data;

   spin_lock(&LXS->lock);

   LXS->int_status = LXS_read_reg(LXS, R_LXS_STATUS) & (BIT_S2MM_ERR|BIT_MM2S_RD_CMPLT);

   // clear interrupt
   LXS_write_reg(LXS, R_INTERRUPT_ADDR,K_CLEAR_INTERRUPT);

   atomic_inc(&LXS->irq_count);
   atomic_inc(&LXS->semaphore);

   // wake up the irq wait queue to notify processes using select/poll/epoll.
   wake_up_interruptible(&LXS->irq_wait_queue);

   //   LXS_write_reg(LXS, R_INTERRUPT_ADDR,K_DISABLE_INTERRUPT);

   spin_unlock(&LXS->lock);

   return IRQ_HANDLED;
}

static const struct file_operations LXS_fops = {
   .owner = THIS_MODULE,
   .read = LXS_read,
   .poll = LXS_poll,
   .mmap = LXS_mmap,
   .unlocked_ioctl = LXS_ioctl,
   .open = LXS_open,
   .release = LXS_release,
};



static const struct of_device_id LXS_of_match_table[] = {
   { .compatible = "kutu,axi4-LXS-controller-1.00-a", (void *)&LXS_fops },
   { },
};
MODULE_DEVICE_TABLE(of, LXS_of_match_table);

static int LXS_probe(struct platform_device *pdev)
{
   const struct of_device_id *id;
   struct resource *mem;
   struct LXS_drvdata *LXS;
   dev_t devt;
   int ret;
   struct device *dev;

   if (!pdev->dev.of_node)
      return -ENODEV;

   id = of_match_node(LXS_of_match_table, pdev->dev.of_node);
   if (!id)
      return -EINVAL;

   LXS = devm_kzalloc(&pdev->dev, sizeof(*LXS), GFP_KERNEL);
   if (!LXS)
      return -ENOMEM;

   dev_info(&pdev->dev, "Kutu LXS trying to call platform get resource\n");

   mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
   LXS->base = devm_ioremap_resource(&pdev->dev, mem);

   LXS->gpio_base = ioremap(0xe000a000, 1024);


   if (IS_ERR(LXS->base))
      return PTR_ERR(LXS->base);

   // reset fgpa
/*   LXS_write_reg(LXS, R_MODE_CONFIG_ADDR, FPGA_RESET);
   udelay(5);
   LXS->config_state = 0;
   LXS_write_reg(LXS, R_MODE_CONFIG_ADDR, (LXS->config_state));
   udelay(5);
   // setup clock and reset fpga
   lmk0482x_init(LXS);
   LXS_write_reg(LXS, R_MODE_CONFIG_ADDR, FPGA_RESET);
   udelay(5);
   LXS->config_state = 0;
   LXS_write_reg(LXS, R_MODE_CONFIG_ADDR, (LXS->config_state));
   udelay(5);
*/
   atomic_set(&LXS->semaphore, 0);
   LXS->int_status = 0;
   LXS_write_reg(LXS, R_INTERRUPT_ADDR,K_DISABLE_INTERRUPT);

   dev_info(&pdev->dev, "Kutu LXS finished call to platform get resource\n");

   dev_info(&pdev->dev, "Kutu LXS trying to allocate irq\n");

   LXS->irq = platform_get_irq(pdev, 0);
   ret = devm_request_irq(&pdev->dev, LXS->irq, &LXS_isr, 0, dev_name(&pdev->dev), LXS);
   if (ret) {
      dev_err(&pdev->dev, "No IRQ available");
      return ret;
   }

   dev_info(&pdev->dev, "Kutu LXS successfully setup irq\n");

   platform_set_drvdata(pdev, LXS);
   spin_lock_init(&LXS->lock);
   mutex_init(&LXS->mutex);

   LXS->is_open = 0;
   LXS->dma_done = 0;
   LXS->error_status = 0;
   dev_info(&pdev->dev, "ioremap %pa to %p\n", &mem->start, LXS->base);

   LXS->clk = devm_clk_get(&pdev->dev, NULL);
   if (IS_ERR(LXS->clk)) {
      ret = PTR_ERR(LXS->clk);
      goto failed5;
   }
   dev_info(&pdev->dev, "Successfully got device clock\n");
   dev_info(&pdev->dev, "device clock rate is %ld\n", clk_get_rate(LXS->clk));

   //
   // reset LXS device
   //

   ret = alloc_chrdev_region(&devt, 0, LXS_DEVICES, DRIVER_NAME);
   if (ret < 0)
      goto failed5;
   dev_info(&pdev->dev, "Successfully allocated chrdev region\n");

   LXS->devt = devt;

   cdev_init(&LXS->cdev, &LXS_fops);
   LXS->cdev.owner = THIS_MODULE;
   ret = cdev_add(&LXS->cdev, devt, 1);
   if (ret) {
      dev_err(&pdev->dev, "cdev_add() failed\n");
      goto failed6;
   }

   LXS->class = class_create(THIS_MODULE, DRIVER_NAME);
   if (IS_ERR(LXS->class)) {
      dev_err(&pdev->dev, "failed to create class\n");
      goto failed6;
   }

   dev = device_create(LXS->class, &pdev->dev, devt, LXS, DRIVER_NAME);
   if (IS_ERR(dev)) {
      dev_err(&pdev->dev, "unable to create device\n");
      goto failed7;
   }
   dev_info(&pdev->dev, "Successfully created device\n");

   //
   // allocate mmap area
   //
   LXS->dma_addr = dma_alloc_coherent(NULL, DMA_LENGTH, &LXS->dma_handle, GFP_KERNEL);

   dev_info(&pdev->dev, "dma_addr = 0x%x, dma_handle = 0x%x\n",(u32)LXS->dma_addr,(u32)LXS->dma_handle);
   dev_info(&pdev->dev, "LXS base = 0x%x\n",(u32)LXS->base);

   if (!LXS->dma_addr) {
      printk(KERN_ERR "<%s> Error: allocating dma memory failed\n", MODULE_NAME);

      ret = -ENOMEM;
      goto failed8;
   }
   dev_info(&pdev->dev, "Successfully allocated dma memory\n");

   //platform_driver_register(pdev);
   dev_info(&pdev->dev, "Kutu LXS finished loading driver\n");

   return 0;

failed8:
   device_destroy(LXS->class, LXS->devt);
failed7:
   class_destroy(LXS->class);
failed6:
   /* Unregister char driver */
   unregister_chrdev_region(devt, LXS_DEVICES);
failed5:

   return ret;
}

static int LXS_remove(struct platform_device *pdev)
{
   struct LXS_drvdata *LXS;

   LXS = platform_get_drvdata(pdev);

   if (!LXS)
      return -ENODEV;

   unregister_chrdev_region(LXS->devt, LXS_DEVICES);

   //	sysfs_remove_group(&pdev->dev.kobj, &LXS_attr_group);

   device_destroy(LXS->class, LXS->devt);
   class_destroy(LXS->class);
   cdev_del(&LXS->cdev);
   clk_unprepare(LXS->clk);

   /* free mmap area */
   if (LXS->dma_addr) {
      dma_free_coherent(NULL, DMA_LENGTH, LXS->dma_addr, LXS->dma_handle);
   }

   return 0;
}

static struct platform_driver LXS_driver = {
   .probe = LXS_probe,
   .remove = LXS_remove,
   .driver = {
      .name = "LXS",
      .of_match_table = LXS_of_match_table,
   },
};
module_platform_driver(LXS_driver);

MODULE_AUTHOR("Greg Smart <Greg.Smart@kutu.com.au>");
MODULE_DESCRIPTION("LXS Linux driver");
MODULE_LICENSE("GPL v2");
