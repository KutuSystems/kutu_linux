/*
 * Wrapper Driver used to control a two-channel Xilinx DMA Engine
 */

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
#include <linux/dma-mapping.h>
#include <linux/platform_device.h>
#include <linux/time.h>

#include "MSP430-SBW.h"
#include "MSP430-SBW-system.h"
#include "SBWLowLevel.h"

#define DRIVER_NAME "MSP430-SBW"
#define MODULE_NAME "MSP430-SBW"
#define MSP430_SBW_DEVICES 4

LIST_HEAD( MSP430_SBW_full_dev_list );


static int MSP430_SBW_open(struct inode *i, struct file *filp)
{
   struct MSP430_SBW_drvdata *MSP430_SBW;

   MSP430_SBW = container_of(i->i_cdev, struct MSP430_SBW_drvdata, cdev);

   atomic_set(&MSP430_SBW->irq_count, 0);

   //init_waitqueue_head(&MSP430_SBW->irq_wait_queue);

//   printk(KERN_DEBUG "<%s> file: open()\n", MODULE_NAME);
   filp->private_data = MSP430_SBW;
   return 0;
}

static int MSP430_SBW_release(struct inode *i, struct file *f)
{
   struct MSP430_SBW_drvdata *MSP430_SBW;

   MSP430_SBW = container_of(i->i_cdev, struct MSP430_SBW_drvdata, cdev);

//   MSP430_SBW_write_reg(MSP430_SBW, R_INTERRUPT_ADDR, K_DISABLE_INTERRUPT);
//   MSP430_SBW_write_reg(MSP430_SBW, R_MODE_CONFIG_ADDR, MODE_PPS_DEBUG);

//   printk(KERN_DEBUG "<%s> file: close()\n", MODULE_NAME);
   return 0;
}


static long MSP430_SBW_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
   struct MSP430_SBW_drvdata *MSP430_SBW = filp->private_data;
   void  *arg_ptr = (void *)arg;
   long  ret = 0;
   unsigned int val;
   struct MSP430_SBW_debug_struct debug_cmd;

   //printk(KERN_DEBUG "<%s> ioctl: entered MSP430_SBW_ioctl\n", MODULE_NAME);

   switch (cmd) {

      case MSP430_SBW_USER_SETDIR:
         MSP430_SBW_write_reg(MSP430_SBW, R_DIRECTION_ADDR, arg);
         return 0;

      case MSP430_SBW_USER_WRITE_OUTPUT:
         MSP430_SBW_write_reg(MSP430_SBW, R_WRITE_ADDR, arg);
         return 0;

      case MSP430_SBW_USER_READ_INPUT:
         val = MSP430_SBW_read_reg(MSP430_SBW, R_READ_ADDR);
         if (copy_to_user(arg_ptr, &val, sizeof(val))) {
            return -EFAULT;
         }
         return 0;

      case MSP430_SBW_USER_READBACK_OUTPUT:
         val = MSP430_SBW_read_reg(MSP430_SBW, R_READBACK_WR_ADDR);
         if (copy_to_user(arg_ptr, &val, sizeof(val))) {
            return -EFAULT;
         }
         return 0;

      case MSP430_SBW_USER_REG_DEBUG:
         if (copy_from_user(&debug_cmd, arg_ptr, sizeof(debug_cmd))) {
            printk(KERN_DEBUG "MSP430_SBW_REG_DEBUG: copy failed\n");

            return -EFAULT;
         }

         if (debug_cmd.cmd == MSP430_SBW_DEBUG_WRITE){
            MSP430_SBW_write_reg(MSP430_SBW, debug_cmd.reg, debug_cmd.data);
            return 0;
         }

         if (debug_cmd.cmd == MSP430_SBW_DEBUG_READ)
            debug_cmd.data = MSP430_SBW_read_reg(MSP430_SBW, debug_cmd.reg);

         if (copy_to_user(arg_ptr, &debug_cmd, sizeof(debug_cmd))) {
            return -EFAULT;
         }
         return 0;

      case MSP430_SBW_USER_RELEASE:
         SBWRelease(MSP430_SBW);
         return 0;

      case MSP430_SBW_USER_RESETTAP:
         SBWResetTAP(MSP430_SBW);
         return 0;

      case MSP430_SBW_USER_RESTART:
         SBWRestart(MSP430_SBW);
         return 0;

      case MSP430_SBW_USER_START:
         SBWStart(MSP430_SBW);
         return 0;

      case MSP430_SBW_USER_SHIFTIR:
         val = SBWShiftIR(MSP430_SBW, arg);
         if (copy_to_user(arg_ptr, &val, sizeof(val))) {
            return -EFAULT;
         }
         return 0;

      case MSP430_SBW_USER_SHIFTDR16:
         val = SBWShiftDR16(MSP430_SBW, arg);
         if (copy_to_user(arg_ptr, &val, sizeof(val))) {
            return -EFAULT;
         }
         return 0;

      case MSP430_SBW_USER_SHIFTDR20:
         val = SBWShiftDR20(MSP430_SBW, arg);
         if (copy_to_user(arg_ptr, &val, sizeof(val))) {
            return -EFAULT;
         }
         return 0;

      case MSP430_SBW_USER_TCLKHIGH:
         SBWTCLKHigh(MSP430_SBW);
         return 0;

      case MSP430_SBW_USER_TCLKLOW:
         SBWTCLKLow(MSP430_SBW);
         return 0;

      case MSP430_SBW_USER_UPDATEDR:
         SBWUpdateDR(MSP430_SBW);
         return 0;

      default:
	      break;

   } // switch

   return ret;
}

/**
 * MSP430_SBW_isr() - The main interrupt handler.
 * @irq:	The interrupt number.
 * @data:	Pointer to the driver data structure.
 * returns: IRQ_HANDLED after the interrupt is handled.
 **/
static irqreturn_t MSP430_SBW_isr(int irq, void *data)
{
	struct MSP430_SBW_drvdata *MSP = data;

	spin_lock(&MSP->lock);

	spin_unlock(&MSP->lock);

	return IRQ_HANDLED;
}

static const struct file_operations MSP430_SBW_fops = {
   .owner = THIS_MODULE,
   .unlocked_ioctl = MSP430_SBW_ioctl,
   .open = MSP430_SBW_open,
   .release = MSP430_SBW_release,
};

static const struct of_device_id MSP430_SBW_of_match_table[] = {
   { .compatible = "kutu,MSP430-SBW-controller-1.00-a", (void *)&MSP430_SBW_fops },
   { },
};
MODULE_DEVICE_TABLE(of, MSP430_SBW_of_match_table);

static int MSP430_SBW_probe(struct platform_device *pdev)
{
   const struct of_device_id *id;
   struct resource *mem;
   struct MSP430_SBW_drvdata *MSP430_SBW;
   dev_t devt;
   int ret;
   struct device *dev;

   if (!pdev->dev.of_node)
      return -ENODEV;

   id = of_match_node(MSP430_SBW_of_match_table, pdev->dev.of_node);
   if (!id)
      return -EINVAL;

   MSP430_SBW = devm_kzalloc(&pdev->dev, sizeof(*MSP430_SBW), GFP_KERNEL);
   if (!MSP430_SBW)
      return -ENOMEM;

   dev_info(&pdev->dev, "Kutu MSP430_SBW trying to call platform get resource\n");

   mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
   MSP430_SBW->base = devm_ioremap_resource(&pdev->dev, mem);

   if (IS_ERR(MSP430_SBW->base))
      return PTR_ERR(MSP430_SBW->base);

   // setup registers
   MSP430_SBW_write_reg(MSP430_SBW, R_WRITE_ADDR, DEFAULT_OUTPUT_VAL);
   MSP430_SBW_write_reg(MSP430_SBW, R_DIRECTION_ADDR, DEFAULT_DIRECTION_VAL);

   dev_info(&pdev->dev, "MSP430_SBW finished call to platform get resource\n");

   dev_info(&pdev->dev, "MSP430_SBW trying to allocate irq\n");

   MSP430_SBW->irq = platform_get_irq(pdev, 0);
   ret = devm_request_irq(&pdev->dev, MSP430_SBW->irq, &MSP430_SBW_isr, 0, dev_name(&pdev->dev), MSP430_SBW);
   if (ret) {
      dev_err(&pdev->dev, "No IRQ available");
      return ret;
   }

   dev_info(&pdev->dev, "MSP430_SBW successfully setup irq\n");

   platform_set_drvdata(pdev, MSP430_SBW);
   spin_lock_init(&MSP430_SBW->lock);

   MSP430_SBW->is_open = 0;
   dev_info(&pdev->dev, "ioremap %pa to %p\n", &mem->start, MSP430_SBW->base);

   MSP430_SBW->clk = devm_clk_get(&pdev->dev, NULL);
   if (IS_ERR(MSP430_SBW->clk)) {
      ret = PTR_ERR(MSP430_SBW->clk);
      goto failed5;
   }
   dev_info(&pdev->dev, "Successfully got device clock\n");
   dev_info(&pdev->dev, "device clock rate is %ld\n", clk_get_rate(MSP430_SBW->clk));

   //
   // reset MSP430_SBW device
   //

   ret = alloc_chrdev_region(&devt, 0, MSP430_SBW_DEVICES, DRIVER_NAME);
   if (ret < 0)
      goto failed5;
   dev_info(&pdev->dev, "Successfully allocated chrdev region\n");

   MSP430_SBW->devt = devt;

   cdev_init(&MSP430_SBW->cdev, &MSP430_SBW_fops);
   MSP430_SBW->cdev.owner = THIS_MODULE;
   ret = cdev_add(&MSP430_SBW->cdev, devt, 1);
   if (ret) {
      dev_err(&pdev->dev, "cdev_add() failed\n");
      goto failed6;
   }

   MSP430_SBW->class = class_create(THIS_MODULE, DRIVER_NAME);
   if (IS_ERR(MSP430_SBW->class)) {
      dev_err(&pdev->dev, "failed to create class\n");
      goto failed6;
   }

   dev = device_create(MSP430_SBW->class, &pdev->dev, devt, MSP430_SBW, DRIVER_NAME);
   if (IS_ERR(dev)) {
      dev_err(&pdev->dev, "unable to create device\n");
      goto failed7;
   }
   dev_info(&pdev->dev, "Successfully created device\n");

   init_waitqueue_head(&MSP430_SBW->irq_wait_queue);

   //platform_driver_register(pdev);
   dev_info(&pdev->dev, "Kutu MSP430_SBW finished loading driver\n");

   return 0;

failed7:
   class_destroy(MSP430_SBW->class);
failed6:
   /* Unregister char driver */
   unregister_chrdev_region(devt, MSP430_SBW_DEVICES);
failed5:

   return ret;
}

static int MSP430_SBW_remove(struct platform_device *pdev)
{
   struct MSP430_SBW_drvdata *MSP430_SBW;

   MSP430_SBW = platform_get_drvdata(pdev);

   if (!MSP430_SBW)
      return -ENODEV;

   unregister_chrdev_region(MSP430_SBW->devt, MSP430_SBW_DEVICES);

   //	sysfs_remove_group(&pdev->dev.kobj, &MSP430_SBW_attr_group);

   device_destroy(MSP430_SBW->class, MSP430_SBW->devt);
   class_destroy(MSP430_SBW->class);
   cdev_del(&MSP430_SBW->cdev);
   clk_unprepare(MSP430_SBW->clk);

   return 0;
}

static struct platform_driver MSP430_SBW_driver = {
   .probe = MSP430_SBW_probe,
   .remove = MSP430_SBW_remove,
   .driver = {
   .name = "MSP430-SBW",
   .of_match_table = MSP430_SBW_of_match_table,
   },
};
module_platform_driver(MSP430_SBW_driver);

MODULE_AUTHOR("Universal Biosensors Pty Ltd");
MODULE_DESCRIPTION("MSP430 SBW Linux driver");
MODULE_LICENSE("GPL v2");
