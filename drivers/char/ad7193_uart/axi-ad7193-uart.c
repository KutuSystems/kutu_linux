#include <linux/module.h>
#include <linux/version.h>
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/kdev_t.h>
#include <linux/of.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/cdev.h>
#include <linux/init.h>
#include <linux/interrupt.h>

#include <asm/uaccess.h>
#include <xen/page.h>

#include <linux/slab.h>
#include <linux/platform_device.h>

#include "adc.h"
#include "adc_system.h"



#define DRIVER_NAME "adc"
#define ADC_DEVICES 1



LIST_HEAD( adc_full_dev_list );


static int adc_open(struct inode *i, struct file *filp)
{
	struct adc_drvdata *adc;

  adc = container_of(i->i_cdev, struct adc_drvdata, cdev);

   printk(KERN_DEBUG "<%s> file: open()\n", DRIVER_NAME);
	filp->private_data = adc;
   return 0;
}

static int adc_release(struct inode *i, struct file *f)
{
   printk(KERN_DEBUG "<%s> file: close()\n", DRIVER_NAME);
   return 0;
}

static long adc_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
   struct adc_drvdata *adc = filp->private_data;

   u32 *arg_ptr = (u32 *)arg;
   long  ret = 0; 
   struct uart_data_struct U1;
	
   switch (cmd) {

      case UART_ADC0:
	update_struct(adc,&U1,0);
         if (copy_to_user(arg_ptr, &U1, sizeof(U1))) {
            return -EFAULT;
         }
         return 0;

         break;

      case UART_ADC1:
	update_struct(adc,&U1,1);
         if (copy_to_user(arg_ptr, &U1, sizeof(U1))) {
            return -EFAULT;
         }
         return 0;

         break;

      case UART_ADC2:
	update_struct(adc,&U1,2);
         if (copy_to_user(arg_ptr, &U1, sizeof(U1))) {
            return -EFAULT;
         }
         return 0;

         break;

      case UART_ADC3:
	update_struct(adc,&U1,3);
         if (copy_to_user(arg_ptr, &U1, sizeof(U1))) {
            return -EFAULT;
         }
         return 0;

         break;

      case UART_ADC4:
	update_struct(adc,&U1,4);
         if (copy_to_user(arg_ptr, &U1, sizeof(U1))) {
            return -EFAULT;
         }
         return 0;

         break;

      case UART_ADC5:
	update_struct(adc,&U1,5);
         if (copy_to_user(arg_ptr, &U1, sizeof(U1))) {
            return -EFAULT;
         }
         return 0;

         break;

      case UART_ADC6:
	update_struct(adc,&U1,6);
         if (copy_to_user(arg_ptr, &U1, sizeof(U1))) {
            return -EFAULT;
         }
         return 0;

         break;

      case UART_ADC7:
	update_struct(adc,&U1,7);
         if (copy_to_user(arg_ptr, &U1, sizeof(U1))) {
            return -EFAULT;
         }
         return 0;

         break;	
      case UART_BAUD0:
         adc_write_reg(adc, UART_BAUD0_ADDR, arg);
         return 0;

         break;

      case UART_BAUD1:
         adc_write_reg(adc, UART_BAUD1_ADDR, arg);
         return 0;

         break;

      case UART_BAUD2:
         adc_write_reg(adc, UART_BAUD2_ADDR, arg);
         return 0;

         break;

      case UART_BAUD3:
         adc_write_reg(adc, UART_BAUD3_ADDR, arg);
         return 0;

         break;

      case UART_BAUD4:
         adc_write_reg(adc, UART_BAUD4_ADDR, arg);
         return 0;

         break;

      case UART_BAUD5:
         adc_write_reg(adc, UART_BAUD5_ADDR, arg);
         return 0;

         break;

      case UART_BAUD6:
         adc_write_reg(adc, UART_BAUD6_ADDR, arg);
         return 0;

         break;

      case UART_BAUD7:
         adc_write_reg(adc, UART_BAUD7_ADDR, arg);
         return 0;

         break;

      case UART_TX0:
         adc_write_reg(adc, UART_TX0_ADDR, arg);
         return 0;

         break;

      case UART_TX1:
         adc_write_reg(adc, UART_TX1_ADDR, arg);
         return 0;

	 break;

      case UART_TX2:
         adc_write_reg(adc, UART_TX2_ADDR, arg);
         return 0;

	 break;

      case UART_TX3:
         adc_write_reg(adc, UART_TX3_ADDR, arg);
         return 0;

	 break;

      case UART_TX4:
         adc_write_reg(adc, UART_TX4_ADDR, arg);
         return 0;

         break;

      case UART_TX5:
         adc_write_reg(adc, UART_TX5_ADDR, arg);
         return 0;

	 break;

      case UART_TX6:
         adc_write_reg(adc, UART_TX6_ADDR, arg);
         return 0;

	 break;

      case UART_TX7:
         adc_write_reg(adc, UART_TX7_ADDR, arg);
         return 0;

	 break;

      case UART_RX0:
         ret = adc_read_reg(adc, UART_RX0_ADDR);
         if (copy_to_user(arg_ptr, &ret, sizeof(u32))) {
            return -EFAULT;
         }
         return 0;

         break;

      case UART_RX1:
         ret = adc_read_reg(adc, UART_RX1_ADDR);
         if (copy_to_user(arg_ptr, &ret, sizeof(u32))) {
            return -EFAULT;
         }
         return 0;

         break;

      case UART_RX2:
         ret = adc_read_reg(adc, UART_RX2_ADDR);
         if (copy_to_user(arg_ptr, &ret, sizeof(u32))) {
            return -EFAULT;
         }
         return 0;

         break;

      case UART_RX3:
         ret = adc_read_reg(adc, UART_RX3_ADDR);
         if (copy_to_user(arg_ptr, &ret, sizeof(u32))) {
            return -EFAULT;
         }
         return 0;

         break;

      case UART_RX4:
         ret = adc_read_reg(adc, UART_RX4_ADDR);
         if (copy_to_user(arg_ptr, &ret, sizeof(u32))) {
            return -EFAULT;
         }
         return 0;

         break;

      case UART_RX5:
         ret = adc_read_reg(adc, UART_RX5_ADDR);
         if (copy_to_user(arg_ptr, &ret, sizeof(u32))) {
            return -EFAULT;
         }
         return 0;

         break;

      case UART_RX6:
         ret = adc_read_reg(adc, UART_RX6_ADDR);
         if (copy_to_user(arg_ptr, &ret, sizeof(u32))) {
            return -EFAULT;
         }
         return 0;

         break;

      case UART_RX7:
         ret = adc_read_reg(adc, UART_RX7_ADDR);
         if (copy_to_user(arg_ptr, &ret, sizeof(u32))) {
            return -EFAULT;
         }
         return 0;

         break;
      case ADC_RAW0:
         ret = adc_read_reg(adc, ADC_CH0_ADDR);
         if (copy_to_user(arg_ptr, &ret, sizeof(u32))) {
            return -EFAULT;
         }
         return 0;

         break;

      case ADC_RAW1:
	 ret = adc_read_reg(adc, ADC_CH1_ADDR);
         if (copy_to_user(arg_ptr, &ret, sizeof(u32))) {
            return -EFAULT;
         }
         return 0;

	 break;

      case ADC_RAW2:
	 ret = adc_read_reg(adc, ADC_CH2_ADDR);
         if (copy_to_user(arg_ptr, &ret, sizeof(u32))) {
            return -EFAULT;
         }
         return 0;

	 break;

      case ADC_RAW3:
	 ret = adc_read_reg(adc, ADC_CH3_ADDR);
         if (copy_to_user(arg_ptr, &ret, sizeof(u32))) {
            return -EFAULT;
         }
         return 0;

	 break;

      case ADC_RAW4:
         ret = adc_read_reg(adc, ADC_CH4_ADDR);
         if (copy_to_user(arg_ptr, &ret, sizeof(u32))) {
            return -EFAULT;
         }
         return 0;

         break;

      case ADC_RAW5:
	 ret = adc_read_reg(adc, ADC_CH5_ADDR);
         if (copy_to_user(arg_ptr, &ret, sizeof(u32))) {
            return -EFAULT;
         }
         return 0;

	 break;

      case ADC_RAW6:
	 ret = adc_read_reg(adc, ADC_CH6_ADDR);
         if (copy_to_user(arg_ptr, &ret, sizeof(u32))) {
            return -EFAULT;
         }
         return 0;

	 break;

      case ADC_RAW7:
	 ret = adc_read_reg(adc, ADC_CH7_ADDR);
         if (copy_to_user(arg_ptr, &ret, sizeof(u32))) {
            return -EFAULT;
         }
         return 0;

	 break;

      case ADC_CONFIG:
	 adc_write_reg(adc, ADC_CONFIG_ADDR, arg);

         return 0;

	 break;

      default:

         break;
   }

   return ret;
}

static irqreturn_t adc_isr(int irq, void *data)
{
   struct adc_drvdata *drvdata = data;

   spin_lock(&drvdata->lock);

   spin_unlock(&drvdata->lock);

   return IRQ_HANDLED;
}

static const struct file_operations adc_fops = {
   .owner = THIS_MODULE,
   .unlocked_ioctl = adc_ioctl,
   .open = adc_open,
   .release = adc_release,
};

static const struct of_device_id adc_of_match_table[] = {
   { .compatible = "par,axi-ad7193-uart-controller-1.00-a", (void *)&adc_fops },
   { },
};
MODULE_DEVICE_TABLE(of, adc_of_match_table);

static int adc_probe(struct platform_device *pdev)
{
   const struct of_device_id *id;
   struct resource *mem;
   struct adc_drvdata *adc;
   dev_t devt;
   int ret;
   struct device *dev;

   if (!pdev->dev.of_node)
      return -ENODEV;

   id = of_match_node(adc_of_match_table, pdev->dev.of_node);
   if (!id)
      return -EINVAL;

   adc = devm_kzalloc(&pdev->dev, sizeof(*adc), GFP_KERNEL);
   if (!adc)
      return -ENOMEM;

   dev_info(&pdev->dev, "ADC trying to call platform get resource\n");

   mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
   adc->base = devm_ioremap_resource(&pdev->dev, mem);

   if (IS_ERR(adc->base))
      return PTR_ERR(adc->base);

   adc->run_state = 0;

   dev_info(&pdev->dev, "ADC finished call to platform get resource\n");

   dev_info(&pdev->dev, "ADC trying to allocate irq\n");

   adc->irq = platform_get_irq(pdev, 0);
   ret = devm_request_irq(&pdev->dev, adc->irq, &adc_isr,
         0, dev_name(&pdev->dev), adc);
   if (ret) {
      dev_err(&pdev->dev, "No IRQ available");
      return ret;
   }

   dev_info(&pdev->dev, "ADC successfully setup irq\n");

   platform_set_drvdata(pdev, adc);
   spin_lock_init(&adc->lock);
   mutex_init(&adc->mutex);

   adc->is_open = 0;
   adc->error_status = 0;
   dev_info(&pdev->dev, "ioremap %pa to %p\n", &mem->start, adc->base);

   ret = alloc_chrdev_region(&devt, 0, ADC_DEVICES, DRIVER_NAME);
   if (ret < 0)
      goto failed5;
   dev_info(&pdev->dev, "Successfully allocated chrdev region\n");

   adc->devt = devt;

   cdev_init(&adc->cdev, &adc_fops);
   adc->cdev.owner = THIS_MODULE;
   ret = cdev_add(&adc->cdev, devt, 1);
   if (ret) {
      dev_err(&pdev->dev, "cdev_add() failed\n");
      goto failed6;
   }

   adc->class = class_create(THIS_MODULE, DRIVER_NAME);
   if (IS_ERR(adc->class)) {
      dev_err(&pdev->dev, "failed to create class\n");
      goto failed6;
   }

   dev = device_create(adc->class, &pdev->dev, devt, adc, DRIVER_NAME);
   if (IS_ERR(dev)) {
      dev_err(&pdev->dev, "unable to create device\n");
      goto failed7;
   }
   dev_info(&pdev->dev, "Successfully created device\n");


   //platform_driver_register(pdev);
   dev_info(&pdev->dev, "ADC and UART Finished loading driver\n");

   return 0;

//failed8:
//   device_destroy(adc->class, adc->devt);
failed7:
   class_destroy(adc->class);
failed6:
   /* Unregister char driver */
   unregister_chrdev_region(devt, ADC_DEVICES);
failed5:
   return ret;
}

static int adc_remove(struct platform_device *pdev)
{
   struct adc_drvdata *adc;

   adc = platform_get_drvdata(pdev);

   if (!adc)
      return -ENODEV;

   unregister_chrdev_region(adc->devt, ADC_DEVICES);

   device_destroy(adc->class, adc->devt);
   class_destroy(adc->class);
   cdev_del(&adc->cdev);

   return 0;
}

static struct platform_driver adc_driver = {
   .probe = adc_probe,
   .remove = adc_remove,
   .driver = {
      .name = "adc",
      .of_match_table = adc_of_match_table,
   },
};
module_platform_driver(adc_driver);

MODULE_AUTHOR("Haig Parseghian");
MODULE_DESCRIPTION("Ad7193 and UART Linux driver");
MODULE_LICENSE("GPL v2");
