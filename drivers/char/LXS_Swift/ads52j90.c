/*
 * ads52j90.c
 *
 *  Created on: 9 Aug 2017
 *      Author: root
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

#include "lmk0482x.h"
#include "ads52j90.h"

int adc_spi_write(struct LXS_drvdata *LXS, u8 spi_address, u16 spi_data, int spi_device_id)
{
   int status;
   u32 spi_word;
   u32 num_busy_counts;

   spi_word = (spi_address << 16)|(spi_data & 0xff);

   num_busy_counts = MAX_BUSY_RETRIES;

   // wait while busy
   num_busy_counts = 0;
   status = 1;
   while ((num_busy_counts < MAX_BUSY_RETRIES)&&(status)) {
      if (spi_device_id == 1) {
         status = LXS_read_reg(LXS,R_SPI_STATUS_ADDR) & ADC1_SPI_DEVICE_STATUS;
      } else {
         status = LXS_read_reg(LXS,R_SPI_STATUS_ADDR) & ADC0_SPI_DEVICE_STATUS;
      }
      num_busy_counts++;
   }

   if (num_busy_counts > (MAX_BUSY_RETRIES - 2))
   {
      return -1;
   }

   if (spi_device_id == 1) {
      LXS_write_reg(LXS, R_SPI_ADC1_ADDR, spi_word);
   } else {
      LXS_write_reg(LXS, R_SPI_ADC0_ADDR, spi_word);
   }

   return 0;
}


int adc_reset(struct LXS_drvdata *LXS)
{
   u32 local_control;

   local_control = LXS->current_control;

   // reset ADC's
   local_control |= ADC_RESET;
   LXS_write_reg(LXS, R_SPI_CONTROL_ADDR, local_control);
   local_control &= ~ADC_RESET;
   LXS_write_reg(LXS, R_SPI_CONTROL_ADDR, local_control);

   return 0;
}


int adc_power_down(struct LXS_drvdata *LXS)
{

   LXS->current_control = (ADC_POWERDOWN|ADC_POWERDOWN_FAST);
   LXS_write_reg(LXS, R_SPI_CONTROL_ADDR,LXS->current_control);

	return 0;
}


int adc_initialize(struct LXS_drvdata *LXS)
{
   int status;
   u32 local_control;

   local_control = 0;
   LXS->current_control = 0;

   // reset ADC's
   local_control |= ADC_RESET;
   LXS_write_reg(LXS, R_SPI_CONTROL_ADDR, local_control);
   local_control &= ~ADC_RESET;
   LXS_write_reg(LXS, R_SPI_CONTROL_ADDR, local_control);

	status = 0;

	status |= adc_spi_write(LXS, 0x0A, 0x3000, 0);
	// Reset is self-clearing.
	status |= adc_spi_write(LXS, 0x00, 0x0001, 0);
	// Clear manually anyway.
	status |= adc_spi_write(LXS, 0x00, 0x0000, 0);
	status |= adc_spi_write(LXS, 0x01, 0x0000, 0);

	status |= adc_spi_write(LXS, 0x03, 0x6000, 0);
	status |= adc_spi_write(LXS, 0x04, 0x0013, 0);
	status |= adc_spi_write(LXS, 0x41, 0x0002, 0);

	status |= adc_spi_write(LXS, 0x0A, 0x3000, 1);
	// Reset is self-clearing.
	status |= adc_spi_write(LXS, 0x00, 0x0001, 1);
	// Clear manually anyway.
	status |= adc_spi_write(LXS, 0x00, 0x0000, 1);
	status |= adc_spi_write(LXS, 0x01, 0x0000, 1);

   status |= adc_spi_write(LXS, 0x03, 0x6000, 1);
   status |= adc_spi_write(LXS, 0x04, 0x0013, 1);
   status |= adc_spi_write(LXS, 0x41, 0x0002, 1);

   // sync ADC's
   local_control |= ADC_SYNC;
   LXS_write_reg(LXS, R_SPI_CONTROL_ADDR, local_control);
   local_control &= ~ADC_SYNC;
   LXS_write_reg(LXS, R_SPI_CONTROL_ADDR, local_control);

   return status;
}
