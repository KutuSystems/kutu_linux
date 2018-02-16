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
#include "IND.h"
#include "IND_system.h"
#include <linux/iio/sysfs.h>
#include <linux/iio/buffer.h>

#define DEBUG

//
// IND_Set_User_Mode()
//
// Set the user operation mode
//
int IND_Set_User_Mode(struct IND_drvdata *IND, struct IND_cmd_struct *cmd)
{
   u32 arg,dma_size;

   arg = cmd->config;

//   if (arg & (~(ADC_TEST_DATA|PPS_DEBUG_MODE|DMA_DEBUG_MODE))) {
//      printk(KERN_DEBUG "IND_USER_SET_MODE: invalid argument\n");
//      return -EFAULT;
//   }

   if (cmd->interrupt == ENABLE_INTERRUPT)
       IND_write_reg(IND, R_INTERRUPT_ADDR, K_CLEAR_INTERRUPT);
   else
       IND_write_reg(IND, R_INTERRUPT_ADDR, K_DISABLE_INTERRUPT);

   //printk(KERN_DEBUG "IND_USER_SET_MODE: config=0x%08x address=0x%08x capture_count=0x%08x delay_count=0x%08x peak_detect_start=0x%08x peak_detect_end=0x%08X\n", cmd->config, cmd->address, cmd->capture_count, cmd->delay_count, cmd->peak_detect_start, cmd->peak_detect_end);

   dma_size = cmd->capture_count * 6;
   IND_write_reg(IND, R_DMA_WRITE_ADDR, (IND->dma_handle + cmd->address));
   IND_write_reg(IND, R_DMA_SIZE_ADDR, dma_size);
   IND_write_reg(IND, R_CAPTURE_COUNT_ADDR, (cmd->capture_count));
   IND_write_reg(IND, R_DELAY_COUNT_ADDR, (cmd->delay_count));

   if (cmd->peak_detect_start > PEAK_START_DISABLE)
      IND_write_reg(IND, R_PEAK_START_ADDR, PEAK_START_DISABLE);
   else
      IND_write_reg(IND, R_PEAK_START_ADDR, cmd->peak_detect_start);

   if (cmd->peak_detect_end > PEAK_STOP_DISABLE)
      IND_write_reg(IND, R_PEAK_END_ADDR, PEAK_STOP_DISABLE);
   else
      IND_write_reg(IND, R_PEAK_END_ADDR, cmd->peak_detect_end);

   IND->config_state &= ~(CONFIG_MODE_MASK);
   IND->config_state |= (arg & CONFIG_MODE_MASK);
   IND_write_reg(IND, R_MODE_CONFIG_ADDR, IND->config_state);
   //printk(KERN_DEBUG "IND_USER_SET_MODE: config_state=0x%08x\n", IND->config_state);

   return 0;
}

//
// IND_SPI_Access()
//
// Write a command to SPI port
//
int IND_SPI_Access(struct IND_drvdata *IND, void *user_ptr)
{
   u32   i,j,data;
   u32   rd_nwr_mode,count;

   struct IND_spi_cmd_struct  cmd;

   if (copy_from_user(&cmd, user_ptr, sizeof(cmd))) {
      printk(KERN_DEBUG "IND_SPI_Write: copy failed\n");

      return -EFAULT;
   }

   if (cmd.num_spi_writes > 16)
      return -EFAULT;

   if (cmd.num_spi_reads > 16)
      return -EFAULT;

   rd_nwr_mode = SPI_CTRL_WRITE; // write by default
   count = cmd.num_spi_writes;
   if (cmd.num_spi_writes == 0) {
      rd_nwr_mode = SPI_CTRL_READ;
      count = cmd.num_spi_reads;
   }
   // if both reads and writes non-zero, then error
   if (rd_nwr_mode == SPI_CTRL_WRITE)
      if (cmd.num_spi_reads != 0)
         return -EFAULT;

   // if both reads and writes 0, then exit
   if (rd_nwr_mode == SPI_CTRL_READ)
      if (cmd.num_spi_reads == 0)
         return 0;

   for (j = 0; j < count; j++) {
      //
      // Wait for SPI access to finish
      //
      i = 0;
      while ((IND_Status(IND) & BIT_SPI_BUSY) && (i < MAX_WAIT_COUNT))
         i++;
#ifdef DEBUG
      printk(KERN_DEBUG "Looped through SPI wait %d times\n",i);
#endif
      if (cmd.port_addr[j] > 0x1fff)
         return -EFAULT;

      data = rd_nwr_mode|cmd.port_device[j]; // write to AD9467
      data |= cmd.port_addr[j];
#ifdef DEBUG
      printk(KERN_DEBUG "output to device register = 0x%x\n",data);
#endif
      IND_write_reg(IND, R_SPI_DEVICE_ADDR, data);

      data = cmd.port_data[j];
#ifdef DEBUG
      printk(KERN_DEBUG "output to data register = 0x%x\n",data);
#endif
       IND_write_reg(IND, R_SPI_DATA_ADDR, data);

      // wait until SPI write completes
      i = 0;
      while ((IND_Status(IND) & BIT_SPI_BUSY) && (i < MAX_WAIT_COUNT))
         i++;

      // read back data
//      if (rd_nwr_mode == SPI_CTRL_READ) {
         data = IND_read_reg(IND,R_SPI_READ_ADDR);
#ifdef DEBUG
         printk(KERN_DEBUG "Read data = 0x%x\n",data);
#endif
         if ((data & 0xffffff00) == 0x87654300)
            cmd.port_data[j] = data & 0xff;
         else
            cmd.port_data[j] = data;
//      }
   }
   //
   // Wait for SPI access to finish
   //
   i = 0;
   while ((IND_Status(IND) & BIT_SPI_BUSY) && (i < MAX_WAIT_COUNT))
      i++;

   if (rd_nwr_mode == SPI_CTRL_READ) {
      if (copy_to_user(user_ptr, &cmd, sizeof(cmd))) {
         return -EFAULT;
      }
   }
   return 0;
}

//
// IND_Maxmin_Read()
//
int IND_Maxmin_Read(struct IND_drvdata *IND, void *user_ptr)
{
   struct IND_maxmin_struct  cmd;

   cmd.max_ch0_data = IND_read_reg(IND,R_MAX_CH0_VAL_ADDR) & 0x00ffffff;
   cmd.max_ch0_addr = IND_read_reg(IND,R_MAX_CH0_LOC_ADDR) & 0x00ffffff;
   cmd.min_ch0_data = IND_read_reg(IND,R_MIN_CH0_VAL_ADDR) & 0x00ffffff;
   cmd.min_ch0_addr = IND_read_reg(IND,R_MIN_CH0_LOC_ADDR) & 0x00ffffff;
   cmd.max_ch1_data = IND_read_reg(IND,R_MAX_CH1_VAL_ADDR) & 0x00ffffff;
   cmd.max_ch1_addr = IND_read_reg(IND,R_MAX_CH1_LOC_ADDR) & 0x00ffffff;
   cmd.min_ch1_data = IND_read_reg(IND,R_MIN_CH1_VAL_ADDR) & 0x00ffffff;
   cmd.min_ch1_addr = IND_read_reg(IND,R_MIN_CH1_LOC_ADDR) & 0x00ffffff;
   cmd.max_ch2_data = IND_read_reg(IND,R_MAX_CH2_VAL_ADDR) & 0x00ffffff;
   cmd.max_ch2_addr = IND_read_reg(IND,R_MAX_CH2_LOC_ADDR) & 0x00ffffff;
   cmd.min_ch2_data = IND_read_reg(IND,R_MIN_CH2_VAL_ADDR) & 0x00ffffff;
   cmd.min_ch2_addr = IND_read_reg(IND,R_MIN_CH2_LOC_ADDR) & 0x00ffffff;

   if (copy_to_user(user_ptr, &cmd, sizeof(cmd))) {
      return -EFAULT;
   }

   return 0;
}


//
// IND_Run_Scan()
//
// Set the user operation mode
//
int IND_Run_Scan(struct IND_drvdata *IND, void *user_ptr)
{
   struct IND_cmd_struct   cmd;
   u32                        config;

   if (copy_from_user(&cmd, user_ptr, sizeof(cmd))) {
      printk(KERN_DEBUG "IND_Set_Run_Scan: copy failed\n");

      return -EFAULT;
   }

   // set mode (dma_debug and reset disabled)
   config = cmd.config & (ADC_TEST_DATA|PPS_DEBUG_MODE);

   IND->config_state = config;
   IND_write_reg(IND, R_MODE_CONFIG_ADDR, config);

   IND_write_reg(IND, R_DMA_READ_ADDR, cmd.address);


   return 0;
}
