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

//#include <linux/iio/iio.h>
#include "LSI.h"
#include "LSI_system.h"
//#include <linux/iio/sysfs.h>
//#include <linux/iio/buffer.h>

#define DEBUG
#define TAPS_DEBUG

//
// LSI_Set_User_Mode()
//
// Set the user operation mode
//
int LSI_Set_User_Mode(struct LSI_drvdata *LSI, struct LSI_cmd_struct *cmd)
{
   u32 dma_size,capture_count,val,config;

   //
   // set capture channel and mode

   if (cmd->capture_channel > 39) {
      printk(KERN_DEBUG "LSI_USER_SET_MODE: Only channel 0 .. 39 is valid!\n");
      return -EFAULT;
   }

   config = cmd->config & (USE_PEAK_DETECT|USE_HISTOGRAM|USE_CAPTURE|USE_TEST_DATA|ADC_TEST_DATA|PPS_DEBUG_MODE|START_DMA|ARM_SYNC|USE_GATE);
   config |= cmd->capture_channel << 16;
   if (cmd->single_channel)
      config |= SINGLE_MODE;

   if (!(config & (USE_HISTOGRAM|USE_CAPTURE))) {
      printk(KERN_DEBUG "LSI_USER_SET_MODE: No capture mode selected!\n");
      return -EFAULT;
   }

   // check for logical commands
   if (config & ADC_TEST_DATA) {
      // this mode is only valid if capture is enabled, histogram is disabled and test data is disabled
      val = config & (USE_HISTOGRAM|USE_CAPTURE|USE_TEST_DATA);
      if (val != USE_CAPTURE) {
         printk(KERN_DEBUG "LSI_USER_SET_MODE: ADC_TEST_DATA configuration is invalid!\n");
         return -EFAULT;
      }
   }

   if (config & USE_HISTOGRAM) {
      if ((cmd->histogram_rate < 5) || (cmd->histogram_rate > 1000)) {
         printk(KERN_DEBUG "LSI_USER_SET_MODE: Histogram rate is limited from 0.1ms to 20ms in 20us steps\n");
         return -EFAULT;
      }

      if ((cmd->histogram_count < 1) || (cmd->histogram_count > 8000)) {
         printk(KERN_DEBUG "LSI_USER_SET_MODE: Histogram count is limited from 1 to 8000\n");
         return -EFAULT;
      }

      LSI_write_reg(LSI, R_HISTOGRAM_RATE_ADDR, cmd->histogram_rate);
      LSI_write_reg(LSI, R_HISTOGRAM_NUM_ADDR, cmd->histogram_count);

      if ((cmd->histogram_address + cmd->histogram_count*32768) > DMA_LENGTH) {
         printk(KERN_DEBUG "LSI_USER_SET_MODE: Invalid histogram address\n");
         return -EFAULT;
      }
      LSI_write_reg(LSI, R_HIST_DMA_ADDR, LSI->dma_handle + cmd->histogram_address);
      LSI_write_reg(LSI, R_HIST_SIZE_ADDR, cmd->histogram_count*32768); // each histogram is 32kbytes
      printk(KERN_DEBUG "LSI_USER_SET_MODE: Histogram address=0x%08x size=0x%08x\n", cmd->histogram_address, cmd->histogram_count*32768);
   }

   if (config & ADC_TEST_DATA) {
      if (config & USE_HISTOGRAM) {
         dma_size = cmd->histogram_count * cmd->histogram_rate * 16000;
      } else {
         dma_size = cmd->capture_count * 8;
      }

      if ((cmd->test_data_address + dma_size) > DMA_LENGTH) {
         printk(KERN_DEBUG "LSI_USER_SET_MODE: Invalid test data address\n");
         return -EFAULT;
      }
      LSI_write_reg(LSI, R_TEST_DMA_ADDR, LSI->dma_handle + cmd->test_data_address);
      LSI_write_reg(LSI, R_TEST_SIZE_ADDR, dma_size);
      printk(KERN_DEBUG "LSI_USER_SET_MODE: Test Data address=0x%08x size=0x%08x\n", cmd->test_data_address, dma_size);
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
         printk(KERN_DEBUG "LSI_USER_SET_MODE: Invalid test data address\n");
         return -EFAULT;
      }
      LSI_write_reg(LSI, R_CAPT_DMA_ADDR, LSI->dma_handle + cmd->capture_address);
      LSI_write_reg(LSI, R_CAPT_SIZE_ADDR, dma_size);
      printk(KERN_DEBUG "LSI_USER_SET_MODE: Capture address=0x%08x size=0x%08x\n", cmd->capture_address, dma_size);
   }

   if (cmd->interrupt == ENABLE_INTERRUPT)
       LSI_write_reg(LSI, R_INTERRUPT_ADDR, K_CLEAR_INTERRUPT);
   else
       LSI_write_reg(LSI, R_INTERRUPT_ADDR, K_DISABLE_INTERRUPT);

   if (config & USE_HISTOGRAM) {
      capture_count = cmd->histogram_count * cmd->histogram_rate * 2000;
   } else {
      capture_count = cmd->capture_count * 8;
   }

   printk(KERN_DEBUG "LSI_USER_SET_MODE: capture_count=0x%08x\n", capture_count);

   LSI_write_reg(LSI, R_CAPTURE_COUNT_ADDR, capture_count);
   LSI->int_status = 0;

   LSI_write_reg(LSI, R_DDT_ADDR, cmd->ddt);
   LSI_write_reg(LSI, R_ALPHA_ADDR, cmd->alpha);

   LSI->config_state = config;
   LSI_write_reg(LSI, R_MODE_CONFIG_ADDR, LSI->config_state);
   printk(KERN_DEBUG "LSI_USER_SET_MODE: config_state=0x%08x\n", LSI->config_state);

   return 0;
}

//
// LSI_SPI_Access()
//
// Write a command to SPI port
//
int LSI_SPI_Access(struct LSI_drvdata *LSI, void *user_ptr)
{
   u32   i,j,data;
   u32   rd_nwr_mode,count;
   u32   addr_offset;

   struct LSI_spi_cmd_struct  cmd;

   if (copy_from_user(&cmd, user_ptr, sizeof(cmd))) {
      printk(KERN_DEBUG "LSI_SPI_Write: copy failed\n");

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
      while (LMK_SPI_Busy(LSI) && (i < MAX_WAIT_COUNT))
         i++;
#ifdef DEBUG
      printk(KERN_DEBUG "Looped through SPI wait %d times\n",i);
#endif
      if (cmd.port_addr[j] > 0x1fff)
         return -EFAULT;

      if (cmd.port_data[j] > 0xff)
         return -EFAULT;

      if (cmd.port_device[j] > 9)
         return -EFAULT;

      data = SPI_DEVICE_0;
      if (cmd.port_device[j] & 1)
         data = SPI_DEVICE_1;

      addr_offset = R_SPI_0_ADDR + ((cmd.port_device[j]>>1)*4);
      data |= rd_nwr_mode;             // read or write
      data |= cmd.port_addr[j] << 8;   // port address
      data |= cmd.port_data[j];        // port data

#ifdef DEBUG
      printk(KERN_DEBUG "output to device register at address 0x%x = 0x%x\n",addr_offset,data);
#endif
      LSI_write_reg(LSI, addr_offset, data);

      // wait until SPI write completes
      i = 0;
      while (LMK_SPI_Busy(LSI) && (i < MAX_WAIT_COUNT))
         i++;
#ifdef DEBUG
      printk(KERN_DEBUG "Looped through SPI wait %d times\n",i);
#endif

      // if read then read back data
      if (rd_nwr_mode == SPI_CTRL_READ) {
         data = LSI_read_reg(LSI,addr_offset);
#ifdef DEBUG
         printk(KERN_DEBUG "Read data = 0x%x\n",data);
#endif
         cmd.port_data[j] = data;
      }
   }
   //
   // Wait for SPI access to finish
   //
   i = 0;
   while (LMK_SPI_Busy(LSI) && (i < MAX_WAIT_COUNT))
      i++;

   if (rd_nwr_mode == SPI_CTRL_READ) {
      if (copy_to_user(user_ptr, &cmd, sizeof(cmd))) {
         return -EFAULT;
      }
   }
   return 0;
}

//
// LSI_Write_Adc_Taps()
//
// Set the user operation mode
//
int LSI_Write_Adc_Taps(struct LSI_drvdata *LSI, void *user_ptr)
{
   struct LSI_adc_tap_struct   taps;
   u32                         lo_word, hi_word, addr_lo, addr_hi;

   if (copy_from_user(&taps, user_ptr, sizeof(taps))) {
      printk(KERN_DEBUG "LSI_Set_Run_Scan: copy failed\n");

      return -EFAULT;
   }

   lo_word = (taps.clk_taps << 16)|(taps.adc1_1_taps << 12)|(taps.adc1_0_taps << 8)|(taps.adc0_1_taps << 4)|(taps.adc0_0_taps);
   hi_word = (taps.frame_taps << 16)|(taps.adc3_1_taps << 12)|(taps.adc3_0_taps << 8)|(taps.adc2_1_taps << 4)|(taps.adc2_0_taps);

   if (taps.device > 9)
      return -EFAULT;

   addr_lo = R_ADC0_L_TAPS_ADDR + taps.device*8;
   addr_hi = R_ADC0_H_TAPS_ADDR + taps.device*8;

#ifdef TAPS_DEBUG
      printk(KERN_DEBUG "LSI_Write_Adc_taps:clk_taps = 0x%x\n", taps.clk_taps);
      printk(KERN_DEBUG "LSI_Write_Adc_taps:frame_taps = 0x%x\n", taps.frame_taps);
      printk(KERN_DEBUG "LSI_Write_Adc_taps:adc0_0_taps = 0x%x\n", taps.adc0_0_taps);
      printk(KERN_DEBUG "LSI_Write_Adc_taps:adc0_1_taps = 0x%x\n", taps.adc0_1_taps);
      printk(KERN_DEBUG "LSI_Write_Adc_taps:adc1_0_taps = 0x%x\n", taps.adc1_0_taps);
      printk(KERN_DEBUG "LSI_Write_Adc_taps:adc1_1_taps = 0x%x\n", taps.adc1_1_taps);
      printk(KERN_DEBUG "LSI_Write_Adc_taps:adc2_0_taps = 0x%x\n", taps.adc2_0_taps);
      printk(KERN_DEBUG "LSI_Write_Adc_taps:adc2_1_taps = 0x%x\n", taps.adc2_1_taps);
      printk(KERN_DEBUG "LSI_Write_Adc_taps:adc3_0_taps = 0x%x\n", taps.adc3_0_taps);
      printk(KERN_DEBUG "LSI_Write_Adc_taps:adc3_1_taps = 0x%x\n", taps.adc3_1_taps);

      printk(KERN_DEBUG "LSI_Write_Adc_taps:addr_lo = 0x%x, addr_hi = 0x%x, lo_word = 0x%x, hi_word = 0x%x\n",addr_lo, addr_hi, lo_word, hi_word);
#endif

   LSI_write_reg(LSI, addr_lo, lo_word);
   LSI_write_reg(LSI, addr_hi, hi_word);
   udelay(10);
   LSI_write_reg(LSI, R_TAPS_LOAD_ADDR, (1 << taps.device));
   udelay(10);

   return 0;
}

//
// LSI_Read_Adc_Taps()
//
// Set the user operation mode
//
int LSI_Read_Adc_Taps(struct LSI_drvdata *LSI, void *user_ptr)
{
   struct LSI_adc_tap_struct   taps;
   u32                         lo_word, hi_word, addr_lo, addr_hi,i;

   if (copy_from_user(&taps, user_ptr, sizeof(taps))) {
      printk(KERN_DEBUG "LSI_Set_Run_Scan: copy failed\n");

      return -EFAULT;
   }

   if (taps.device > 9)
      return -EFAULT;

   addr_lo = R_ADC0_L_TAPS_ADDR + taps.device*8;
   addr_hi = R_ADC0_H_TAPS_ADDR + taps.device*8;
   lo_word = LSI_read_reg(LSI, addr_lo);
   hi_word = LSI_read_reg(LSI, addr_hi);

#ifdef TAPS_DEBUG
   printk(KERN_DEBUG "LSI_Read_Adc_taps:addr_lo = 0x%x, addr_hi = 0x%x, lo_word = 0x%x, hi_word = 0x%x\n",addr_lo, addr_hi, lo_word, hi_word);

   // dump registers
   for (i=0; i < 128; i+=4)
      printk(KERN_DEBUG "LSI_Read_Adc_taps:addr = 0x%x, word = 0x%x\n",i, LSI_read_reg(LSI, i));
#endif

  taps.clk_taps      = (lo_word >> 16) & 0xf;
  taps.adc1_1_taps   = (lo_word >> 12) & 0xf;
  taps.adc1_0_taps   = (lo_word >> 8) & 0xf;
  taps.adc0_1_taps   = (lo_word >> 4) & 0xf;
  taps.adc0_0_taps   = (lo_word >> 0) & 0xf;

  taps.frame_status  = (lo_word >> (20 + taps.device)) & 0x1;

  taps.frame_taps    = (hi_word >> 16) & 0xf;
  taps.adc3_1_taps   = (hi_word >> 12) & 0xf;
  taps.adc3_0_taps   = (hi_word >> 8) & 0xf;
  taps.adc2_1_taps   = (hi_word >> 4) & 0xf;
  taps.adc2_0_taps   = (hi_word >> 0) & 0xf;

  if (copy_to_user(user_ptr, &taps, sizeof(taps))) {
     return -EFAULT;
  }

  return 0;
}
