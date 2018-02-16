/* The industrial I/O core
 *
 * Copyright (c) 2015 Greg Smart
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 * Handling of device specific ioctls.
 *
 *
 * Things to look at here.
 * - create generic link into driver
 *
 */
#include <linux/kernel.h>
#include <linux/export.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/slab.h>
#include <linux/poll.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>

#include <linux/iio/iio.h>
#include "gr1000.h"
#include "gr1000_system.h"
#include <linux/iio/sysfs.h>
#include <linux/iio/buffer.h>

#define DEBUG

//
// GR1000_Set_User_Mode()
//
// Set the user operation mode
//
int GR1000_Set_User_Mode(struct gr1000_drvdata *gr1000, u32 arg)
{
   if (arg & (~(ADC_TEST_DATA|PPS_DEBUG_MODE|DMA_DEBUG_MODE))) {
      printk(KERN_DEBUG "GR1000_USER_SET_MODE: invalid argument\n");
      return -EFAULT;
   }

   gr1000->config_state |= arg;
   gr1000_write_reg(gr1000, R_MODE_CONFIG_ADDR, gr1000->config_state);

   return 0;
}

//
// GR1000_SPI_Write()
//
// Write a command to SPI port
// 3byte_mode only affects dac port (SPI 2)
//
int GR1000_SPI_Write(struct gr1000_drvdata *gr1000, void *user_ptr)
{
   u32   i,j,data;

   struct GR1000_spi_cmd_struct  cmd;

   if (copy_from_user(&cmd, user_ptr, sizeof(cmd))) {
      printk(KERN_DEBUG "GR1000_SPI_Write: copy failed\n");

      return -EFAULT;
   }

   if (cmd.num_spi_writes == 0)
     return 0;

   if (cmd.num_spi_writes > 16)
      return -EFAULT;

   for (j = 0; j < cmd.num_spi_writes; j++) {
      //
      // Wait for SPI access to finish
      //
      i = 0;
      while ((GR1000_Status(gr1000) & STAT_SPI_BUSY) && (i < MAX_WAIT_COUNT))
         i++;
#ifdef DEBUG
      printk(KERN_DEBUG "Looped through SPI wait %d times\n",i);
#endif
      if (cmd.port_addr[j] > 16)
         return -EFAULT;

      data =  (cmd.port_addr[j] << 28)|(cmd.port_data[j]&0x0fffffff);
      gr1000_write_reg(gr1000, R_SPI_DATA_ADDR, data);
      gr1000_write_reg(gr1000, R_SPI_DEVICE_ADDR, (cmd.port_device[j]&0x03));

      // wait until SPI write completes
      i = 0;
      while ((GR1000_Status(gr1000) & STAT_SPI_BUSY) && (i < MAX_WAIT_COUNT))
         i++;

   }
   return 0;
}

//
// GR1000_Run_Scan()
//
// Set the user operation mode
//
int GR1000_Run_Scan(struct gr1000_drvdata *gr1000, void *user_ptr)
{
   struct GR1000_cmd_struct   cmd;
   u32                        config;

   if (copy_from_user(&cmd, user_ptr, sizeof(cmd))) {
      printk(KERN_DEBUG "GR1000_Set_Run_Scan: copy failed\n");

      return -EFAULT;
   }

   // set mode (dma_debug and reset disabled)
   config = cmd.config & (ADC_TEST_DATA|PPS_DEBUG_MODE);

   gr1000->config_state = config;
   gr1000_write_reg(gr1000, R_MODE_CONFIG_ADDR, config);

   // configure dma channel and capture size
   gr1000_write_reg(gr1000, R_DMA_WRITE_ADDR, gr1000->dma_handle + cmd.addr_offset);
   gr1000_write_reg(gr1000, R_DMA_SIZE_ADDR, cmd.num_samples<<3);
   gr1000_write_reg(gr1000, R_CAPTURE_COUNT_ADDR, cmd.num_samples);

   return 0;
}
