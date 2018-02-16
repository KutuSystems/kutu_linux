/* The Industrial I/O core
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

#include "LXS.h"
#include "LXS_system.h"
#include "lmk0482x.h"
#include "ads52j90.h"

#define DEBUG
#define TAPS_DEBUG

//
// LXS_Set_User_Mode()
//
// Set the user operation mode
//
int LXS_Set_User_Mode(struct LXS_drvdata *LXS, struct LXS_cmd_struct *cmd)
{
   u32 dma_size,capture_count,val,config;

   //
   // set capture channel and mode

   if (cmd->capture_channel > 39) {
      printk(KERN_DEBUG "LXS_USER_SET_MODE: Only channel 0 .. 39 is valid!\n");
      return -EFAULT;
   }

   config = cmd->config & (USE_PEAK_DETECT|USE_HISTOGRAM|USE_CAPTURE|USE_TEST_DATA|ADC_TEST_DATA|PPS_DEBUG_MODE|START_DMA|ARM_SYNC|USE_BRAM|USE_GATE|LOOP_ACTIVE);
   config |= cmd->capture_channel << 16;
   if (cmd->single_channel)
      config |= SINGLE_MODE;

   if (!(config & (USE_HISTOGRAM|USE_CAPTURE))) {
      printk(KERN_DEBUG "LXS_USER_SET_MODE: No capture mode selected!\n");
      return -EFAULT;
   }

   // check for logical commands
   if (config & ADC_TEST_DATA) {
      // this mode is only valid if capture is enabled, histogram is disabled and test data is disabled
      val = config & (USE_HISTOGRAM|USE_CAPTURE|USE_TEST_DATA);
      if (val != USE_CAPTURE) {
         printk(KERN_DEBUG "LXS_USER_SET_MODE: ADC_TEST_DATA configuration is invalid!\n");
         return -EFAULT;
      }
   }

   if (config & USE_HISTOGRAM) {
      if ((cmd->histogram_rate < 5) || (cmd->histogram_rate > 1000)) {
         printk(KERN_DEBUG "LXS_USER_SET_MODE: Histogram rate is limited from 0.1ms to 20ms in 20us steps\n");
         return -EFAULT;
      }

      if ((cmd->histogram_count < 1) || (cmd->histogram_count > 8000)) {
         printk(KERN_DEBUG "LXS_USER_SET_MODE: Histogram count is limited from 1 to 8000\n");
         return -EFAULT;
      }

      LXS_write_reg(LXS, R_HISTOGRAM_RATE_ADDR, cmd->histogram_rate);
      LXS_write_reg(LXS, R_HISTOGRAM_NUM_ADDR, cmd->histogram_count);

      if ((cmd->histogram_address + cmd->histogram_count*65536) > DMA_LENGTH) {
         printk(KERN_DEBUG "LXS_USER_SET_MODE: Invalid histogram address\n");
         return -EFAULT;
      }
      LXS_write_reg(LXS, R_HIST_DMA_ADDR, LXS->dma_handle + cmd->histogram_address);
      LXS_write_reg(LXS, R_HIST_SIZE_ADDR, cmd->histogram_count*65536); // each histogram is 64kbytes
      printk(KERN_DEBUG "LXS_USER_SET_MODE: Histogram address=0x%08x size=0x%08x\n", cmd->histogram_address, cmd->histogram_count*65536);
   }

   if (config & ADC_TEST_DATA) {
      if (config & USE_HISTOGRAM) {
         dma_size = cmd->histogram_count * cmd->histogram_rate * 16000;
      } else {
         dma_size = cmd->capture_count * 8;
      }

      if ((cmd->test_data_address + dma_size) > DMA_LENGTH) {
         printk(KERN_DEBUG "LXS_USER_SET_MODE: Invalid test data address\n");
         return -EFAULT;
      }
      LXS_write_reg(LXS, R_TEST_DMA_ADDR, LXS->dma_handle + cmd->test_data_address);
      LXS_write_reg(LXS, R_TEST_SIZE_ADDR, dma_size);
      printk(KERN_DEBUG "LXS_USER_SET_MODE: Test Data address=0x%08x size=0x%08x\n", cmd->test_data_address, dma_size);
   }

   if (config & USE_CAPTURE) {
      if (config & USE_HISTOGRAM) {
         dma_size = cmd->histogram_count * cmd->histogram_rate * 16000;
      } else {
         dma_size = cmd->capture_count * 8;
      }
      if (cmd->single_channel) {
         dma_size /= 4;
      }

      if ((cmd->capture_address + dma_size) > DMA_LENGTH) {
         printk(KERN_DEBUG "LXS_USER_SET_MODE: Invalid test data address\n");
         return -EFAULT;
      }
      LXS_write_reg(LXS, R_CAPT_DMA_ADDR, LXS->dma_handle + cmd->capture_address);
      LXS_write_reg(LXS, R_CAPT_SIZE_ADDR, dma_size);
      printk(KERN_DEBUG "LXS_USER_SET_MODE: Capture address=0x%08x size=0x%08x\n", cmd->capture_address, dma_size);
   }

   if (cmd->interrupt == ENABLE_INTERRUPT)
       LXS_write_reg(LXS, R_INTERRUPT_ADDR, K_CLEAR_INTERRUPT);
   else
       LXS_write_reg(LXS, R_INTERRUPT_ADDR, K_DISABLE_INTERRUPT);

   if (config & USE_HISTOGRAM) {
      capture_count = cmd->histogram_count * cmd->histogram_rate * 2000;
   } else {
      capture_count = cmd->capture_count * 8;
   }

   printk(KERN_DEBUG "LXS_USER_SET_MODE: capture_count=0x%08x\n", capture_count);

   LXS_write_reg(LXS, R_CAPTURE_COUNT_ADDR, capture_count);
   LXS->int_status = 0;

   LXS_write_reg(LXS, R_DDT_ADDR, cmd->ddt);
   LXS_write_reg(LXS, R_ALPHA_ADDR, cmd->alpha);

   LXS->config_state = config;
   LXS_write_reg(LXS, R_MODE_CONFIG_ADDR, LXS->config_state);
   printk(KERN_DEBUG "LXS_USER_SET_MODE: config_state=0x%08x\n", LXS->config_state);

   return 0;
}

//
// LXS_SPI_Access()
//
// Write a command to SPI port
//
int LXS_SPI_Access(struct LXS_drvdata *LXS, void *user_ptr)
{
   u32   data;

   struct LXS_spi_cmd_struct  cmd;

   if (copy_from_user(&cmd, user_ptr, sizeof(cmd))) {
      printk(KERN_DEBUG "LXS_SPI_Write: copy failed\n");

      return -EFAULT;
   }

   if (cmd.rd_nwr > 1)
      return -EFAULT;

   if ((cmd.rd_nwr == SPI_DEVICE_READ) && (cmd.device != SPI_DEVICE_LMK)) {
      printk(KERN_DEBUG "LXS_SPI_Access: read not supported for this device\n");
      return -EFAULT;
   }

   if (cmd.device != SPI_DEVICE_LMK) {
      if (cmd.rd_nwr == SPI_DEVICE_READ) {
         lmk_spi_read(LXS, cmd.addr, &data);
         cmd.data = data;
      } else {
         lmk_spi_write(LXS, cmd.addr, cmd.data);
      }

   } else {
      // ADC SPI write
      adc_spi_write(LXS, cmd.addr, cmd.data, cmd.device);
   }

   if (cmd.rd_nwr == SPI_DEVICE_READ) {
      if (copy_to_user(user_ptr, &cmd, sizeof(cmd))) {
         return -EFAULT;
      }
   }
   return 0;
}

//
// LXS_Write_bram()
//
// Set the user operation mode
//
int LXS_Write_bram(struct LXS_drvdata *LXS, void *user_ptr)
{
   struct LXS_bram_struct   taps;
   u32                      addr,data;

   if (copy_from_user(&taps, user_ptr, sizeof(taps))) {
      printk(KERN_DEBUG "LXS_Set_Run_Scan: copy failed\n");

      return -EFAULT;
   }

   addr = R_BRAM_ADDR + taps.address * 4;
   data = taps.value;

   if (taps.address > 1023)
      return -EFAULT;

   LXS_write_reg(LXS, addr, data);

   return 0;
}

//
// LXS_Read_bram()
//
// Set the user operation mode
//
int LXS_Read_bram(struct LXS_drvdata *LXS, void *user_ptr)
{
   struct LXS_bram_struct   taps;
   u32                      addr;

   if (copy_from_user(&taps, user_ptr, sizeof(taps))) {
      printk(KERN_DEBUG "LXS_Set_Run_Scan: copy failed\n");

      return -EFAULT;
   }

   addr = R_BRAM_ADDR + taps.address * 4;

   if (taps.address > 1023)
      return -EFAULT;

   taps.value = LXS_read_reg(LXS, addr);

  if (copy_to_user(user_ptr, &taps, sizeof(taps))) {
     return -EFAULT;
  }

  return 0;
}
