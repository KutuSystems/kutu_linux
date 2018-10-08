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

#include "IND.h"
#include "IND_system.h"

#define DRIVER_NAME "IND"
#define MODULE_NAME "IND"
#define IND_DEVICES 1

#define USE_DMA_ALLOC_COHERENT		0	/* 0 => asynchronous, 1 => synchronous  */
#define USE_DMA_ALLOC_NONCOHERENT	1	/* 0 => synchronous,  1 => asynchronous */
#define USE_DMA_MAPPING_SINGLE		0
#define USE_PGPROT_CACHED		1

LIST_HEAD( IND_full_dev_list );

/*
 * Static global variables
 */
// static DECLARE_WAIT_QUEUE_HEAD(irq_wait_queue);

/* static struct IND_drvdata *get_elem_from_list_by_inode(struct inode *i)
{
   struct list_head *pos;
   struct IND_drvdata *IND = NULL;

   list_for_each( pos, &IND_full_dev_list ) {
      struct IND_drvdata *tmp;
      tmp = list_entry( pos, struct IND_drvdata, dev_list );
      if (tmp->devt == i->i_rdev)
      {
         IND = tmp;
         break;
      }
   }
   return IND;
}
*/

static int IND_open(struct inode *i, struct file *filp)
{
   struct IND_drvdata *IND;

   IND = container_of(i->i_cdev, struct IND_drvdata, cdev);

   atomic_set(&IND->irq_count, 0);

   //init_waitqueue_head(&IND->irq_wait_queue);

//   printk(KERN_DEBUG "<%s> file: open()\n", MODULE_NAME);
   filp->private_data = IND;
   return 0;
}

static int IND_release(struct inode *i, struct file *f)
{
   struct IND_drvdata *IND;

   IND = container_of(i->i_cdev, struct IND_drvdata, cdev);

//   IND_write_reg(IND, R_INTERRUPT_ADDR, K_DISABLE_INTERRUPT);
//   IND_write_reg(IND, R_MODE_CONFIG_ADDR, MODE_PPS_DEBUG);

//   printk(KERN_DEBUG "<%s> file: close()\n", MODULE_NAME);
   return 0;
}

static int IND_mmap(struct file *filp, struct vm_area_struct *vma)
{
   struct IND_drvdata *IND = filp->private_data;
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

#if ! USE_PGPROT_CACHED
   vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);
#else
   vma->vm_page_prot = __pgprot_modify(vma->vm_page_prot,
		                       L_PTE_MT_MASK,
                                       L_PTE_MT_WRITEALLOC | L_PTE_XN);
#endif

   result = remap_pfn_range(vma, vma->vm_start,IND->dma_handle >> PAGE_SHIFT, requested_size, vma->vm_page_prot);


   if (result) {
      printk(KERN_DEBUG "<%s> Error: in calling remap_pfn_range: returned %d\n",
            MODULE_NAME, result);

      return -EAGAIN;
   }

   printk(KERN_DEBUG "<%s> : mmap complete \n",MODULE_NAME);
   return 0;
}

static ssize_t IND_read(struct file *f, char __user * buf, size_t
      len, loff_t * off)
{
   printk(KERN_INFO "<%s> file: read()\n", MODULE_NAME);

   if (len > 65536)
   {
      return 0;
   }

   return len;
}

static long IND_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
   struct IND_drvdata *IND = filp->private_data;
   //   int __user *ip = (int __user *)arg;
   void  *arg_ptr = (void *)arg;
   long  ret = 0;
   unsigned int s2mm_status;
   unsigned int timeout;
   unsigned int val;
   unsigned int val2;
   int i;
//   struct IND_read_data_struct read_cmd;
   struct IND_debug_struct debug_cmd;

   //printk(KERN_DEBUG "<%s> ioctl: entered IND_ioctl\n", MODULE_NAME);

   switch (cmd) {
      /*
       * time critical IOCTLs first (assumes switch statement is executed in order).
       */

      case IND_USER_CAPTURE_INFO_0_GET:
	      ret = IND_capture_info_get(IND, arg_ptr, 0);
	      return ret;

      case IND_USER_CAPTURE_INFO_1_GET:
	      ret = IND_capture_info_get(IND, arg_ptr, 1);
	      return ret;

      case IND_USER_ADC_CLOCK_COUNT_PER_PPS:
	      // Standard register logic used in FPGA to store clock counts per pps !!
	      // Read multiple times to ensure not reading during an update.
	      // Alternative solution is to use dual port RAM in FPGA to cross clock domains.
	      val = IND_read_reg(IND, R_CLOCK_COUNT_PER_PPS_ADDR);
	      for (i = 3; i > 0; i--) {
		      val2 = IND_read_reg(IND, R_CLOCK_COUNT_PER_PPS_ADDR);
		      if (val2 == val)
			      break;
		      val = val2;
	      } // for
	      if (i == 0)
		      return -EFAULT;

	      if (copy_to_user(arg_ptr, &val, sizeof(val)))
		      return -EFAULT;

	      return 0;

      case IND_USER_READ_MAXMIN_NORMAL:
         ret = IND_Maxmin_Read(IND, arg_ptr, R_IND_MAXMIN_NORMAL_BASE );
         return ret;

      case IND_USER_READ_MAXMIN_SQUARED:
         ret = IND_Maxmin_Read(IND, arg_ptr, R_IND_MAXMIN_SQUARED_BASE );
         return ret;

      case IND_USER_STATUS:
         val = IND_Status(IND);
         if (copy_to_user(arg_ptr, &val, sizeof(val))) {
            return -EFAULT;
         }
         return 0;

      case IND_USER_SET_LEDS:
         IND->led_status |= arg;
         IND_write_reg(IND, R_GPIO_LED_ADDR, (IND->led_status));
         return 0;

      case IND_USER_CLEAR_LEDS:
         IND->led_status &= ~arg;
         IND_write_reg(IND, R_GPIO_LED_ADDR, (IND->led_status));
         return 0;

      case IND_USER_MODIFY_LEDS:
      {
         IND_bit_flag_t bit_flags;

         if (copy_from_user(&bit_flags, arg_ptr, sizeof(bit_flags))) {
            printk(KERN_DEBUG "IND_USER_MODIFY_LEDS: copy_from_user failed\n");
            return -EFAULT;
         }

         IND->led_status |=  bit_flags.set;
         IND->led_status &= ~bit_flags.clear;
         IND->led_status ^=  bit_flags.toggle;

         IND_write_reg(IND, R_GPIO_LED_ADDR, (IND->led_status));

         return 0;
      }

      case IND_USER_SET_CTRL:
         IND->ctrl_status |= arg;
         IND_write_reg(IND, R_GPIO_CTRL_ADDR, (IND->ctrl_status));
         return 0;

      case IND_USER_CLEAR_CTRL:
         IND->ctrl_status &= ~arg;
         IND_write_reg(IND, R_GPIO_CTRL_ADDR, (IND->ctrl_status));
         return 0;

      case IND_USER_MODIFY_CTRL:
      {
         IND_bit_flag_t bit_flags;

         if (copy_from_user(&bit_flags, arg_ptr, sizeof(bit_flags))) {
            printk(KERN_DEBUG "IND_USER_MODIFY_CTRL: copy_from_user failed\n");
            return -EFAULT;
         }

         IND->ctrl_status |=  bit_flags.set;
         IND->ctrl_status &= ~bit_flags.clear;
         IND->ctrl_status ^=  bit_flags.toggle;

         IND_write_reg(IND, R_GPIO_CTRL_ADDR, (IND->ctrl_status));

         return 0;
      }

      case IND_USER_GET_SEM:
         val = (uint32_t)atomic_read(&IND->semaphore);
         if (copy_to_user(arg_ptr, &val, sizeof(val))) {
            return -EFAULT;
         }
         return 0;

      case IND_USER_SET_SEM:
         atomic_set(&IND->semaphore, arg);
         return 0;

      case IND_USER_RESET:
         printk(KERN_DEBUG "IND_USER_RESET: Asserting FPGA_RESET bit\n");
         IND_write_reg(IND, R_MODE_CONFIG_ADDR, FPGA_RESET);
         IND->config_state = FPGA_RESET;
         udelay(10);

         printk(KERN_DEBUG "IND_USER_RESET: Deasserting FPGA_RESET bit\n");
         IND_write_reg(IND, R_MODE_CONFIG_ADDR, 0);
         IND->config_state = 0;
         udelay(10);

         printk(KERN_DEBUG "IND_USER_RESET: FPGA Reset complete\n");
         return 0;

      /*
       * Non-time critical IOCTLs can go here.  Order is not significant.
       */

      case IND_USER_DMA_RESET:
         IND_write_reg(IND, R_MODE_CONFIG_ADDR, IND->config_state | DMA_RESET);
         udelay(10);
         IND_write_reg(IND, R_MODE_CONFIG_ADDR, IND->config_state);
         udelay(10);
         return 0;

       case IND_USER_SET_MODE:
         ret = IND_Set_User_Mode(IND, arg_ptr);
         return ret;

      case IND_USER_SET_ADDRESS:
         IND_write_reg(IND, R_DMA_WRITE_ADDR, (IND->dma_handle + arg));
	 IND->bank = (arg == 0) ? 0 : 1;
         return 0;

      case IND_USER_DMA_TEST:

         if (arg >= 0x800000)
            return -EFAULT;

         s2mm_status = IND_Status(IND);
         if (s2mm_status & (BIT_S2MM_ERR | BIT_MM2S_ERR)) {
            // DMA error so reset DMA
            IND_write_reg(IND, R_MODE_CONFIG_ADDR, DMA_RESET | MODE_DMA_DEBUG);
            IND_write_reg(IND, R_MODE_CONFIG_ADDR, MODE_DMA_DEBUG);
            printk(KERN_DEBUG "<%s> : clearing dma error\n",MODULE_NAME);
         }

         if (s2mm_status & BIT_MM2S_RD_CMPLT) {
            // clear complete bit and ensure interrupt is off
            IND_write_reg(IND, R_INTERRUPT_ADDR, K_CLEAR_INTERRUPT);
            printk(KERN_DEBUG "<%s> : clearing complete flag\n",MODULE_NAME);
         }

         // start read from memory
         IND_write_reg(IND, R_DMA_READ_ADDR, IND->dma_handle);

         // start write to memory
         IND_write_reg(IND, R_DMA_WRITE_ADDR, (IND->dma_handle + (DMA_LENGTH/2)));
         IND_write_reg(IND, R_DMA_SIZE_ADDR, arg);
         IND_write_reg(IND, R_CAPTURE_COUNT_ADDR, (arg>>3));

         // start dma into loopback mode
         IND_write_reg(IND, R_MODE_CONFIG_ADDR, (DMA_DEBUG_MODE | DEBUG_START_DMA));

         s2mm_status = IND_Status(IND);
         printk(KERN_DEBUG "<%s> : started dma, status = 0x%x\n",MODULE_NAME,s2mm_status);

         timeout = 0;
         while (((s2mm_status & (BIT_MM2S_RD_CMPLT)) == 0) && (timeout <MAX_WAIT_COUNT))  {
            udelay(100);
            s2mm_status = IND_Status(IND);
            timeout++;
         }
         printk(KERN_DEBUG "<%s> : dma status = 0x%x\n",MODULE_NAME,s2mm_status);

         if (timeout > (MAX_WAIT_COUNT - 10))
            printk(KERN_DEBUG "<%s> : dma timeout\n",MODULE_NAME);

         // clear complete bit
         IND_write_reg(IND, R_INTERRUPT_ADDR, K_CLEAR_INTERRUPT);

         // set configuration back to original state
         IND_write_reg(IND, R_MODE_CONFIG_ADDR, IND->config_state);

         return 0;

      case IND_USER_TRIG_PPS:
         if ((arg != MODE_TRIGGER_PPS) && (arg != GENERATE_PPS))
            return -1;

         IND_write_reg(IND, R_MODE_CONFIG_ADDR, (IND->config_state | arg));

         return ret;

      case IND_USER_SPI_ACCESS:
         ret = IND_SPI_Access(IND, arg_ptr);
         return ret;

      case IND_USER_SET_INTERRUPT:
         if (arg == ENABLE_INTERRUPT)
            // enable and clear pending interrupt.
            IND_write_reg(IND, R_INTERRUPT_ADDR, K_CLEAR_INTERRUPT);
         else
            IND_write_reg(IND, R_INTERRUPT_ADDR, K_DISABLE_INTERRUPT);

         return 0;

      case IND_USER_REG_DEBUG:
         if (copy_from_user(&debug_cmd, arg_ptr, sizeof(debug_cmd))) {
            printk(KERN_DEBUG "IND_REG_DEBUG: copy failed\n");

            return -EFAULT;
         }

         if (debug_cmd.cmd == IND_DEBUG_WRITE){
            IND_write_reg(IND, debug_cmd.reg, debug_cmd.data);
            return 0;
         }

         if (debug_cmd.cmd == IND_DEBUG_READ)
            debug_cmd.data = IND_read_reg(IND, debug_cmd.reg);

         if (copy_to_user(arg_ptr, &debug_cmd, sizeof(debug_cmd))) {
            return -EFAULT;
         }

         return 0;

      case IND_USER_FPGA_VERSION:
      {
         struct IND_fpga_version_struct fpga_version;

         val = IND_read_reg(IND, R_FPGA_VERSION_ADDR);
         if ((val & 0xffff0000) != 0xaaaa0000) {
            printk(KERN_DEBUG "<%s> : Read FPGA version failed !!!, read = 0x%08X\n", MODULE_NAME, val);
//            return -EFAULT;
         }

         fpga_version._reserved_0 = 0;
         fpga_version.major = (val >> 8) & 0xff;
         fpga_version.minor = val & 0xff;

         if (copy_to_user(arg_ptr, &fpga_version, sizeof(fpga_version))) {
            return -EFAULT;
         }

         return 0;
      }

      case IND_USER_ADC_OFFSET_SET:
	      IND_write_reg(IND, R_ADC_OFFSET, arg);
	      return 0;

      case IND_USER_ADC_OFFSET_GET:
	      val = IND_read_reg(IND, R_ADC_OFFSET);
	      if (copy_to_user(arg_ptr, &val, sizeof(val)))
		      return -EFAULT;
	      return 0;

      default:
	      break;

   } // switch

   return ret;
}

static unsigned int IND_poll(struct file *filp, poll_table *ptp)
{
    struct IND_drvdata *IND = filp->private_data;
    unsigned int mask = 0;

    poll_wait(filp, &IND->irq_wait_queue, ptp);

    if (atomic_read(&IND->semaphore)) {
        mask |= (POLLIN | POLLRDNORM);
    }

    return mask;
}

/**
 * IND_isr() - The main interrupt handler.
 * @irq:	The interrupt number.
 * @data:	Pointer to the driver data structure.
 * returns: IRQ_HANDLED after the interrupt is handled.
 **/
static irqreturn_t IND_isr(int irq, void *data)
{
	struct IND_drvdata *IND = data;
	struct IND_capture_info *capture_info;
	uint32_t int_status;

	spin_lock(&IND->lock);

#if 1 //BJS DEBUG
	IND->led_status ^= LED_PPS_OK;
	IND_write_reg(IND, R_GPIO_LED_ADDR, (IND->led_status));
#endif

	// clear interrupt
	IND_write_reg(IND, R_INTERRUPT_ADDR,K_CLEAR_INTERRUPT);

	atomic_inc(&IND->irq_count);
	atomic_inc(&IND->semaphore);

	int_status = IND_read_reg(IND, R_IND_STATUS) & (BIT_S2MM_ERR | BIT_MM2S_RD_CMPLT | BIT_MM2S_ERR);
	IND->int_status = int_status;

	// which bank has been captured?
	capture_info = &IND->capture_info[IND->bank];

	/*
	 *  fill out capture_info structure
	 */
	getnstimeofday(&capture_info->irq_time);
	capture_info->int_status = int_status;
	capture_info->irq_count = (uint32_t)atomic_read(&IND->irq_count);
	capture_info->semaphore = (uint32_t)atomic_read(&IND->semaphore);
	capture_info->adc_clock_count_per_pps = IND_read_reg(IND, R_CLOCK_COUNT_PER_PPS_ADDR);
	capture_info->bank = IND->bank;
	_ind_maxmin_read(&capture_info->maxmin_normal, IND, R_IND_MAXMIN_NORMAL_BASE);
	_ind_maxmin_read(&capture_info->maxmin_squared, IND, R_IND_MAXMIN_SQUARED_BASE);

#if USE_PGPROT_CACHED
	/* dma_sync_single_for_cpu(dev, dma_handle, size, direction); */
	dma_sync_single_for_cpu(NULL, IND->dma_handle, DMA_LENGTH, DMA_BIDIRECTIONAL);
#endif

	// wake up the irq wait queue to notify processes using select/poll/epoll.
	wake_up_interruptible(&IND->irq_wait_queue);

//	IND_write_reg(IND, R_INTERRUPT_ADDR,K_DISABLE_INTERRUPT);

	spin_unlock(&IND->lock);

	return IRQ_HANDLED;
}

static const struct file_operations IND_fops = {
   .owner = THIS_MODULE,
   .read = IND_read,
   .poll = IND_poll,
   .mmap = IND_mmap,
   .unlocked_ioctl = IND_ioctl,
   .open = IND_open,
   .release = IND_release,
};

static const struct of_device_id IND_of_match_table[] = {
   { .compatible = "kutu,axi4-IND-controller-1.00-a", (void *)&IND_fops },
   { },
};
MODULE_DEVICE_TABLE(of, IND_of_match_table);

static int IND_probe(struct platform_device *pdev)
{
   const struct of_device_id *id;
   struct resource *mem;
   struct IND_drvdata *IND;
   dev_t devt;
   int ret;
   struct device *dev;

   if (!pdev->dev.of_node)
      return -ENODEV;

   id = of_match_node(IND_of_match_table, pdev->dev.of_node);
   if (!id)
      return -EINVAL;

   IND = devm_kzalloc(&pdev->dev, sizeof(*IND), GFP_KERNEL);
   if (!IND)
      return -ENOMEM;

   dev_info(&pdev->dev, "Kutu IND trying to call platform get resource\n");

   mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
   IND->base = devm_ioremap_resource(&pdev->dev, mem);

   if (IS_ERR(IND->base))
      return PTR_ERR(IND->base);

   IND->bank = 0;

   // setup fgpa
   IND_write_reg(IND, R_MODE_CONFIG_ADDR, FPGA_RESET);
   udelay(5);
   IND->config_state = 0;
   IND_write_reg(IND, R_MODE_CONFIG_ADDR, (IND->config_state));
   udelay(5);
   IND->ctrl_status = 0;
   IND_write_reg(IND, R_GPIO_CTRL_ADDR, (IND->ctrl_status));
   IND->led_status = 0;
   IND_write_reg(IND, R_GPIO_LED_ADDR, (IND->led_status));
   atomic_set(&IND->semaphore, 0);
   IND->int_status = 0;
   IND_write_reg(IND, R_INTERRUPT_ADDR,K_DISABLE_INTERRUPT);

   dev_info(&pdev->dev, "Kutu IND finished call to platform get resource\n");

   dev_info(&pdev->dev, "Kutu IND trying to allocate irq\n");

   IND->irq = platform_get_irq(pdev, 0);
   ret = devm_request_irq(&pdev->dev, IND->irq, &IND_isr, 0, dev_name(&pdev->dev), IND);
   if (ret) {
      dev_err(&pdev->dev, "No IRQ available");
      return ret;
   }

   dev_info(&pdev->dev, "Kutu IND successfully setup irq\n");

   platform_set_drvdata(pdev, IND);
   spin_lock_init(&IND->lock);
   mutex_init(&IND->mutex);

   IND->is_open = 0;
   IND->dma_done = 0;
   IND->error_status = 0;
   dev_info(&pdev->dev, "ioremap %pa to %p\n", &mem->start, IND->base);

   IND->clk = devm_clk_get(&pdev->dev, NULL);
   if (IS_ERR(IND->clk)) {
      ret = PTR_ERR(IND->clk);
      goto failed5;
   }
   dev_info(&pdev->dev, "Successfully got device clock\n");
   dev_info(&pdev->dev, "device clock rate is %ld\n", clk_get_rate(IND->clk));

   //
   // reset IND device
   //

   ret = alloc_chrdev_region(&devt, 0, IND_DEVICES, DRIVER_NAME);
   if (ret < 0)
      goto failed5;
   dev_info(&pdev->dev, "Successfully allocated chrdev region\n");

   IND->devt = devt;

   cdev_init(&IND->cdev, &IND_fops);
   IND->cdev.owner = THIS_MODULE;
   ret = cdev_add(&IND->cdev, devt, 1);
   if (ret) {
      dev_err(&pdev->dev, "cdev_add() failed\n");
      goto failed6;
   }

   IND->class = class_create(THIS_MODULE, DRIVER_NAME);
   if (IS_ERR(IND->class)) {
      dev_err(&pdev->dev, "failed to create class\n");
      goto failed6;
   }

   dev = device_create(IND->class, &pdev->dev, devt, IND, DRIVER_NAME);
   if (IS_ERR(dev)) {
      dev_err(&pdev->dev, "unable to create device\n");
      goto failed7;
   }
   dev_info(&pdev->dev, "Successfully created device\n");

   //
   // allocate mmap area
   //
#if USE_DMA_ALLOC_COHERENT

   IND->dma_addr = dma_alloc_coherent(NULL, DMA_LENGTH, &IND->dma_handle, GFP_KERNEL);

   dev_info(&pdev->dev, "dma_addr = 0x%x, dma_handle = 0x%x\n",(u32)IND->dma_addr,(u32)IND->dma_handle);
   dev_info(&pdev->dev, "IND base = 0x%x\n",(u32)IND->base);

   if (!IND->dma_addr) {
      printk(KERN_ERR "<%s> Error: allocating dma memory failed\n", MODULE_NAME);

      ret = -ENOMEM;
      goto failed8;
   }
   dev_info(&pdev->dev, "Successfully allocated dma memory\n");

#elif USE_DMA_ALLOC_NONCOHERENT

   IND->dma_addr = dma_alloc_noncoherent(NULL, DMA_LENGTH, &IND->dma_handle, GFP_KERNEL);

   dev_info(&pdev->dev, "dma_addr = 0x%x, dma_handle = 0x%x\n",(u32)IND->dma_addr,(u32)IND->dma_handle);
   dev_info(&pdev->dev, "IND base = 0x%x\n",(u32)IND->base);

   if (!IND->dma_addr) {
      printk(KERN_ERR "<%s> Error: allocating dma memory failed\n", MODULE_NAME);

      ret = -ENOMEM;
      goto failed8;
   }
   dev_info(&pdev->dev, "Successfully allocated dma memory\n");

#else

   /* dma_handle = dma_map_single(dev, addr, size, direction); */
   IND->dma_handle = dma_map_single(NULL, NULL, DMA_LENGTH, DMA_BIDIRECTIONAL);

//   dev_info(&pdev->dev, "dma_addr = 0x%x, dma_handle = 0x%x\n",(u32)IND->dma_addr,(u32)IND->dma_handle);
//   dev_info(&pdev->dev, "dma_addr   = 0x%x\n",(u32)IND->dma_addr);
   dev_info(&pdev->dev, "dma_handle = 0x%x\n",(u32)IND->dma_handle);
   dev_info(&pdev->dev, "IND base   = 0x%x\n",(u32)IND->base);

   /* dma_mapping_error(dev, dma_handle) */
   if (dma_mapping_error(NULL, IND->dma_handle)) {
      printk(KERN_ERR "<%s> Error: allocating dma memory failed\n", MODULE_NAME);
      ret = -ENOMEM;
      goto failed8;
   }
   dev_info(&pdev->dev, "Successfully allocated dma memory\n");

#endif

   init_waitqueue_head(&IND->irq_wait_queue);

   //platform_driver_register(pdev);
   dev_info(&pdev->dev, "Kutu IND finished loading driver\n");

   return 0;

failed8:
   device_destroy(IND->class, IND->devt);
failed7:
   class_destroy(IND->class);
failed6:
   /* Unregister char driver */
   unregister_chrdev_region(devt, IND_DEVICES);
failed5:

   return ret;
}

static int IND_remove(struct platform_device *pdev)
{
   struct IND_drvdata *IND;

   IND = platform_get_drvdata(pdev);

   if (!IND)
      return -ENODEV;

   unregister_chrdev_region(IND->devt, IND_DEVICES);

   //	sysfs_remove_group(&pdev->dev.kobj, &IND_attr_group);

   device_destroy(IND->class, IND->devt);
   class_destroy(IND->class);
   cdev_del(&IND->cdev);
   clk_unprepare(IND->clk);

   /* free mmap area */
#if USE_DMA_ALLOC_COHERENT
   if (IND->dma_addr) {
      dma_free_coherent(NULL, DMA_LENGTH, IND->dma_addr, IND->dma_handle);
   }
#elif USE_DMA_ALLOC_NONCOHERENT
   if (IND->dma_addr) {
      dma_free_noncoherent(NULL, DMA_LENGTH, IND->dma_addr, IND->dma_handle);
   }
#else
   /* dma_unmap_single(dev, dma_handle, size, direction); */
   dma_unmap_single(NULL, IND->dma_handle, DMA_LENGTH, DMA_BIDIRECTIONAL);
#endif

   return 0;
}

static struct platform_driver IND_driver = {
   .probe = IND_probe,
   .remove = IND_remove,
   .driver = {
   .name = "IND",
   .of_match_table = IND_of_match_table,
   },
};
module_platform_driver(IND_driver);

MODULE_AUTHOR("Successful Endeavours Pty Ltd <ray@successful.com.au>");
MODULE_DESCRIPTION("IND Linux driver");
MODULE_LICENSE("GPL v2");
