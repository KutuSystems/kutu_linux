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

#include <asm/uaccess.h>
#include <linux/dma-mapping.h>
#include <xen/page.h>

#include <linux/slab.h>
#include <linux/platform_device.h>

#include "gr1000.h"
#include "gr1000_system.h"

#define DRIVER_NAME "gr1000"
//#define DRIVER_NAME "gr1000_pdrv"
#define MODULE_NAME "gr1000"
#define GR1000_DEVICES 1

LIST_HEAD( gr1000_full_dev_list );

static struct gr1000_drvdata *get_elem_from_list_by_inode(struct inode *i)
{
   struct list_head *pos;
   struct gr1000_drvdata *gr1000 = NULL;

   list_for_each( pos, &gr1000_full_dev_list ) {
      struct gr1000_drvdata *tmp;
      tmp = list_entry( pos, struct gr1000_drvdata, dev_list );
      if (tmp->devt == i->i_rdev)
      {
         gr1000 = tmp;
         break;
      }
   }
   return gr1000;
}

static int gr1000_open(struct inode *i, struct file *filp)
{
   struct gr1000_drvdata *gr1000;

   gr1000 = container_of(i->i_cdev, struct gr1000_drvdata, cdev);

   gr1000->irq_count = 0;

   printk(KERN_DEBUG "<%s> file: open()\n", MODULE_NAME);
   filp->private_data = gr1000;
   return 0;
}

static int gr1000_release(struct inode *i, struct file *f)
{
   struct gr1000_drvdata *gr1000;

   gr1000 = container_of(i->i_cdev, struct gr1000_drvdata, cdev);

//   gr1000_write_reg(gr1000, R_RUN_TEST, STOP_TEST);
//   gr1000_write_reg(gr1000, R_BOTDA_END_FREQ, 0);

   printk(KERN_DEBUG "<%s> file: close()\n", MODULE_NAME);
   return 0;
}

static int gr1000_mmap(struct file *filp, struct vm_area_struct *vma)
{
   struct gr1000_drvdata *gr1000 = filp->private_data;
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

   result = remap_pfn_range(vma, vma->vm_start,gr1000->dma_handle >> PAGE_SHIFT, requested_size, vma->vm_page_prot);


   if (result) {
      printk(KERN_DEBUG "<%s> Error: in calling remap_pfn_range: returned %d\n",
            MODULE_NAME, result);

      return -EAGAIN;
   }

   printk(KERN_DEBUG "<%s> : mmap complete \n",MODULE_NAME);
   return 0;
}

static ssize_t gr1000_read(struct file *f, char __user * buf, size_t
      len, loff_t * off)
{
   /* printk(KERN_INFO "<%s> file: read()\n", MODULE_NAME); */
   struct gr1000_drvdata *gr1000;
   int      count;
   char     *ptr;

   if (len > 65536)
   {
      return 0;
   }
   gr1000 = get_elem_from_list_by_inode(f->f_inode);

   count = len;
   ptr   = buf;

   while (count > 1024) {
      memcpy(ptr, gr1000->base + R_GR1000_FIFO_BASE, 1024);
      count -= 1024;
      ptr+= 1024;
   }
   memcpy(ptr, gr1000->base + R_GR1000_FIFO_BASE, count);

   return len;
}

static long gr1000_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
   struct gr1000_drvdata *gr1000 = filp->private_data;
   //   int __user *ip = (int __user *)arg;
   void  *arg_ptr = (void *)arg;
   long  ret = 0;
   unsigned int s2mm_status, timeout;
//   struct GR1000_read_data_struct read_cmd;
   struct GR1000_debug_struct debug_cmd;

   //printk(KERN_DEBUG "<%s> ioctl: entered gr1000_ioctl\n", MODULE_NAME);

   switch (cmd) {
      case GR1000_USER_RESET:
         if (arg == FPGA_RESET) {
            gr1000_write_reg(gr1000, R_MODE_CONFIG_ADDR, FPGA_RESET|DMA_RESET);
            gr1000->config_state = FPGA_RESET|DMA_RESET;
         } else {
            gr1000_write_reg(gr1000, R_MODE_CONFIG_ADDR, 0);
            gr1000->config_state = 0;
         }

         return 0;

      case GR1000_USER_DMA_RESET:
         gr1000_write_reg(gr1000, R_MODE_CONFIG_ADDR, gr1000->config_state|DMA_RESET);
         udelay(10);
         gr1000_write_reg(gr1000, R_MODE_CONFIG_ADDR, gr1000->config_state);
         udelay(10);
        return 0;

      case GR1000_USER_SET_CLK:
         /*
          * clock control not implemented yet
          */
         return 0;

      case GR1000_USER_SET_MODE:
         ret = GR1000_Set_User_Mode(gr1000, arg);
         return ret;

      case GR1000_USER_RUN_SCAN:
         ret = GR1000_Run_Scan(gr1000, arg_ptr);
         return ret;

      case GR1000_USER_DMA_TEST:
         if (arg >= 0x800000)
            return -EFAULT;

         s2mm_status = GR1000_Status(gr1000);
         if (s2mm_status & (STAT_S2MM_ERR|STAT_MM2S_ERR)) {
            // DMA error so reset DMA
            gr1000_write_reg(gr1000, R_MODE_CONFIG_ADDR, DMA_RESET|MODE_DMA_DEBUG);
            gr1000_write_reg(gr1000, R_MODE_CONFIG_ADDR, MODE_DMA_DEBUG);
            printk(KERN_DEBUG "<%s> : clearing dma error\n",MODULE_NAME);
         }

         if (s2mm_status & STAT_MM2S_RD_CMPLT) {
            // clear complete bit and ensure interrupt is off
            gr1000_write_reg(gr1000, R_INTERRUPT_ADDR, DISABLE_INTERRUPT);
            printk(KERN_DEBUG "<%s> : clearing complete flag\n",MODULE_NAME);
         }

         // start read from memory
         gr1000_write_reg(gr1000, R_DMA_READ_ADDR, gr1000->dma_handle);

         // start write to memory
         gr1000_write_reg(gr1000, R_DMA_WRITE_ADDR, (gr1000->dma_handle + (DMA_LENGTH/2)));
         gr1000_write_reg(gr1000, R_DMA_SIZE_ADDR, arg);
         gr1000_write_reg(gr1000, R_CAPTURE_COUNT_ADDR, (arg>>3));

         // start dma into loopback mode
         gr1000_write_reg(gr1000, R_MODE_CONFIG_ADDR, (DMA_DEBUG_MODE|DEBUG_START_DMA));

         s2mm_status = GR1000_Status(gr1000);
         printk(KERN_DEBUG "<%s> : started dma, status = 0x%x\n",MODULE_NAME,s2mm_status);

         timeout = 0;
         while (((s2mm_status & (STAT_MM2S_RD_CMPLT|STAT_S2MM_ERR|STAT_MM2S_ERR)) == 0) && (timeout <MAX_WAIT_COUNT))  {
            udelay(100);
            s2mm_status = GR1000_Status(gr1000);
            timeout++;
         }
         printk(KERN_DEBUG "<%s> : dma status = 0x%x\n",MODULE_NAME,s2mm_status);

         if (timeout > (MAX_WAIT_COUNT - 10))
            printk(KERN_DEBUG "<%s> : dma timeout\n",MODULE_NAME);

         // clear complete bit
         gr1000_write_reg(gr1000, R_INTERRUPT_ADDR, DISABLE_INTERRUPT);

         // set configuration back to original state
         gr1000_write_reg(gr1000, R_MODE_CONFIG_ADDR, gr1000->config_state);

         return 0;

      case GR1000_USER_TRIG_PPS:
         if ((arg != MODE_TRIGGER_PPS)&&(arg != GENERATE_PPS))
            return -1;

         gr1000_write_reg(gr1000, R_MODE_CONFIG_ADDR, (gr1000->config_state|arg));

         return ret;

      case GR1000_USER_SPI_WRITE:
         ret = GR1000_SPI_Write(gr1000, arg_ptr);
         return ret;

      case GR1000_USER_STATUS:
         ret = GR1000_Status(gr1000);
         if (copy_to_user(arg_ptr, &ret, sizeof(u32))) {
            return -EFAULT;
         }
         return 0;

      case GR1000_USER_REG_DEBUG:

         if (copy_from_user(&debug_cmd, arg_ptr, sizeof(debug_cmd))) {
            printk(KERN_DEBUG "GR1000_REG_DEBUG: copy failed\n");

            return -EFAULT;
         }

         if (debug_cmd.cmd == GR1000_DEBUG_WRITE){
            gr1000_write_reg(gr1000, debug_cmd.reg, debug_cmd.data);
            return 0;
         }

         if (debug_cmd.cmd == GR1000_DEBUG_READ)
            debug_cmd.data = gr1000_read_reg(gr1000, debug_cmd.reg);

         if (copy_to_user(arg_ptr, &debug_cmd, sizeof(debug_cmd))) {
            return -EFAULT;
         }

         return 0;

      default:
         break;
   }

   return ret;
}

/**
 * gr1000_isr() - The main interrupt handler.
 * @irq:	The interrupt number.
 * @data:	Pointer to the driver data structure.
 * returns: IRQ_HANDLED after the interrupt is handled.
 **/
static irqreturn_t gr1000_isr(int irq, void *data)
{
   struct gr1000_drvdata *gr1000 = data;

   spin_lock(&gr1000->lock);

   // clear interrupt
   gr1000_write_reg(gr1000, R_INTERRUPT_ADDR,CLEAR_INTERRUPT);

   gr1000->irq_count++;

   gr1000_write_reg(gr1000, R_INTERRUPT_ADDR,DISABLE_INTERRUPT);

   spin_unlock(&gr1000->lock);

   return IRQ_HANDLED;
}

static const struct file_operations gr1000_fops = {
   .owner = THIS_MODULE,
   .read = gr1000_read,
   .mmap = gr1000_mmap,
   .unlocked_ioctl = gr1000_ioctl,
   .open = gr1000_open,
   .release = gr1000_release,
};

static const struct of_device_id gr1000_of_match_table[] = {
   { .compatible = "kutu,axi4-gr1000-controller-1.00-a", (void *)&gr1000_fops },
   { },
};
MODULE_DEVICE_TABLE(of, gr1000_of_match_table);

static int gr1000_probe(struct platform_device *pdev)
{
   const struct of_device_id *id;
   struct resource *mem;
   struct gr1000_drvdata *gr1000;
   dev_t devt;
   int ret;
   struct device *dev;

   if (!pdev->dev.of_node)
      return -ENODEV;

   id = of_match_node(gr1000_of_match_table, pdev->dev.of_node);
   if (!id)
      return -EINVAL;

   gr1000 = devm_kzalloc(&pdev->dev, sizeof(*gr1000), GFP_KERNEL);
   if (!gr1000)
      return -ENOMEM;

   dev_info(&pdev->dev, "Kutu GR1000 trying to call platform get resource\n");

   mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
   gr1000->base = devm_ioremap_resource(&pdev->dev, mem);

   if (IS_ERR(gr1000->base))
      return PTR_ERR(gr1000->base);

   gr1000->config_state = 0;

   dev_info(&pdev->dev, "Kutu GR1000 finished call to platform get resource\n");

   dev_info(&pdev->dev, "Kutu GR1000 trying to allocate irq\n");

   gr1000->irq = platform_get_irq(pdev, 0);
   ret = devm_request_irq(&pdev->dev, gr1000->irq, &gr1000_isr, 0, dev_name(&pdev->dev), gr1000);
   if (ret) {
      dev_err(&pdev->dev, "No IRQ available");
      return ret;
   }

   dev_info(&pdev->dev, "Kutu GR1000 successfully setup irq\n");

   platform_set_drvdata(pdev, gr1000);
   spin_lock_init(&gr1000->lock);
   mutex_init(&gr1000->mutex);

   gr1000->is_open = 0;
   gr1000->dma_done = 0;
   gr1000->error_status = 0;
   dev_info(&pdev->dev, "ioremap %pa to %p\n", &mem->start, gr1000->base);

   gr1000->clk = devm_clk_get(&pdev->dev, NULL);
   if (IS_ERR(gr1000->clk)) {
      ret = PTR_ERR(gr1000->clk);
      goto failed5;
   }
   dev_info(&pdev->dev, "Successfully got device clock\n");
   dev_info(&pdev->dev, "device clock rate is %ld\n", clk_get_rate(gr1000->clk));

   //
   // reset GR1000 device
   //

   ret = alloc_chrdev_region(&devt, 0, GR1000_DEVICES, DRIVER_NAME);
   if (ret < 0)
      goto failed5;
   dev_info(&pdev->dev, "Successfully allocated chrdev region\n");

   gr1000->devt = devt;

   cdev_init(&gr1000->cdev, &gr1000_fops);
   gr1000->cdev.owner = THIS_MODULE;
   ret = cdev_add(&gr1000->cdev, devt, 1);
   if (ret) {
      dev_err(&pdev->dev, "cdev_add() failed\n");
      goto failed6;
   }

   gr1000->class = class_create(THIS_MODULE, DRIVER_NAME);
   if (IS_ERR(gr1000->class)) {
      dev_err(&pdev->dev, "failed to create class\n");
      goto failed6;
   }

   dev = device_create(gr1000->class, &pdev->dev, devt, gr1000, DRIVER_NAME);
   if (IS_ERR(dev)) {
      dev_err(&pdev->dev, "unable to create device\n");
      goto failed7;
   }
   dev_info(&pdev->dev, "Successfully created device\n");

   //
   // allocate mmap area
   //
   gr1000->dma_addr = dma_alloc_coherent(NULL, DMA_LENGTH, &gr1000->dma_handle, GFP_KERNEL);

   dev_info(&pdev->dev, "dma_addr = 0x%x, dma_handle = 0x%x\n",(u32)gr1000->dma_addr,(u32)gr1000->dma_handle);
   dev_info(&pdev->dev, "gr1000 base = 0x%x\n",(u32)gr1000->base);

   if (!gr1000->dma_addr) {
      printk(KERN_ERR "<%s> Error: allocating dma memory failed\n", MODULE_NAME);

      ret = -ENOMEM;
      goto failed8;
   }
   dev_info(&pdev->dev, "Successfully allocated dma memory\n");

   //platform_driver_register(pdev);
   dev_info(&pdev->dev, "Kutu GR1000 finished loading driver\n");

   return 0;

failed8:
   device_destroy(gr1000->class, gr1000->devt);
failed7:
   class_destroy(gr1000->class);
failed6:
   /* Unregister char driver */
   unregister_chrdev_region(devt, GR1000_DEVICES);
failed5:

   return ret;
}

static int gr1000_remove(struct platform_device *pdev)
{
   struct gr1000_drvdata *gr1000;

   gr1000 = platform_get_drvdata(pdev);

   if (!gr1000)
      return -ENODEV;

   unregister_chrdev_region(gr1000->devt, GR1000_DEVICES);

   //	sysfs_remove_group(&pdev->dev.kobj, &gr1000_attr_group);

   device_destroy(gr1000->class, gr1000->devt);
   class_destroy(gr1000->class);
   cdev_del(&gr1000->cdev);
   clk_unprepare(gr1000->clk);

   /* free mmap area */
   if (gr1000->dma_addr) {
      dma_free_coherent(NULL, DMA_LENGTH, gr1000->dma_addr, gr1000->dma_handle);
   }

   return 0;
}

static struct platform_driver gr1000_driver = {
   .probe = gr1000_probe,
   .remove = gr1000_remove,
   .driver = {
      .name = "gr1000",
      .of_match_table = gr1000_of_match_table,
   },
};
module_platform_driver(gr1000_driver);

MODULE_AUTHOR("Greg Smart <Greg.Smart@kutu.com.au>");
MODULE_DESCRIPTION("GPSat GR1000 Linux driver");
MODULE_LICENSE("GPL v2");
