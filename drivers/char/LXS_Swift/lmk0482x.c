/*
 * lmk0482x.c
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


/**
 * @brief Write data to the specified SPI device.
 */
int lmk_spi_write( struct LXS_drvdata *LXS, u16 spi_address, u16 spi_data)
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
      status = LXS_read_reg(LXS,R_SPI_STATUS_ADDR) & LMK_SPI_DEVICE_STATUS;
      num_busy_counts++;
   }

   if (num_busy_counts > (MAX_BUSY_RETRIES - 2))
   {
      return -1;
   }

   LXS_write_reg(LXS, R_SPI_LMK_ADDR, spi_word);

   return 0;
}

/**
 * @brief Read from the specified SPI device.
 * This is a two-stage process, first the desired address is written to the spi device,
 * then the next read will have the spi data word.
 */
int lmk_spi_read(struct LXS_drvdata *LXS, u16 spi_address, u32* data_out)
{
   int status;
   u32 spi_word,data;
   u32 num_busy_counts;

   spi_word = (spi_address << 16)|LMK_SPI_READ_CMD;

   num_busy_counts = MAX_BUSY_RETRIES;

   // wait while busy
   num_busy_counts = 0;
   status = 1;
   while ((num_busy_counts < MAX_BUSY_RETRIES)&&(status)) {
      status = LXS_read_reg(LXS,R_SPI_STATUS_ADDR) & LMK_SPI_DEVICE_STATUS;
      num_busy_counts++;
   }

   if (num_busy_counts > (MAX_BUSY_RETRIES - 2))
   {
      return -1;
   }

   LXS_write_reg(LXS, R_SPI_LMK_ADDR, spi_word);

   // wait while busy
   num_busy_counts = 0;
   status = 1;
   while ((num_busy_counts < MAX_BUSY_RETRIES)&&(status)) {
      status = LXS_read_reg(LXS,R_SPI_STATUS_ADDR) & LMK_SPI_DEVICE_STATUS;
      num_busy_counts++;
   }

   if (num_busy_counts > (MAX_BUSY_RETRIES - 2))
   {
      return -1;
   }

   data = LXS_read_reg(LXS,R_SPI_LMK_ADDR);

   *data_out = data;

   return 0;
}

int lmk_lock_spi(struct LXS_drvdata *LXS)
{
	int status = 0;
	// From p94 of the datasheet.

	status |= lmk_spi_write(LXS, 0x1FFD, 1);
	status |= lmk_spi_write(LXS, 0x1FFE, 1);
	status |= lmk_spi_write(LXS, 0x1FFF, 1);

	return status;
}

int lmk_unlock_spi(struct LXS_drvdata *LXS)
{
	int status = 0;

	// From p94 of the datasheet.
	status |= lmk_spi_write(LXS, 0x1FFD,  0);
	status |= lmk_spi_write(LXS, 0x1FFE,  0);
	status |= lmk_spi_write(LXS, 0x1FFF, 83); // Note decimal value.

	return status;
}

void lmk0482x_init(struct LXS_drvdata *LXS)
{
   int lmk_status;
   u8 adc_status;
   u32 lmk_id_device_type = 0;
   u32 lmk_id_prod0 = 0;
   u32 lmk_id_prod1 = 0;
   u32 lmk_maskrev  = 0;
   u32 lmk_vendor0  = 0;
   u32 lmk_vendor1  = 0;
   u32 magic  = 0;

   lmk_status = lmk_initialize_raw(LXS);
   adc_status = adc_initialize(LXS);

   lmk_status |= lmk_spi_write(LXS, 0x15f, 0xd0);
   lmk_status |= lmk_spi_read(LXS, 0x15f, &magic );
   printk(KERN_DEBUG "LXS_LMK0482x readback value : 0x%x\n",magic);

   lmk_status |= lmk_spi_write(LXS, 0x15f, 0xff);
   lmk_status |= lmk_spi_read(LXS, 0x15f, &magic );
   printk(KERN_DEBUG "LXS_LMK0482x readback value : 0x%x\n",magic);

   lmk_status |= lmk_spi_write(LXS, 0x15f, 0x51);
   lmk_status |= lmk_spi_read(LXS, 0x15f, &magic );
   printk(KERN_DEBUG "LXS_LMK0482x readback value : 0x%x\n",magic);

   lmk_status |= lmk_spi_write(LXS, 0x15f, 0x81);
   lmk_status |= lmk_spi_read(LXS, 0x15f, &magic );
   printk(KERN_DEBUG "LXS_LMK0482x readback value : 0x%x\n",magic);

   lmk_status |= lmk_spi_read(LXS, 0x003, &lmk_id_device_type);
   printk(KERN_DEBUG "LXS_LMK0482x id_device_type: 0x%x\n",lmk_id_device_type);

   lmk_status |= lmk_spi_read(LXS, 0x004, &lmk_id_prod0);
   printk(KERN_DEBUG "LXS_LMK0482x id_prod0: 0x%x\n",lmk_id_prod0);

   lmk_status |= lmk_spi_read(LXS, 0x005, &lmk_id_prod1);
   printk(KERN_DEBUG "LXS_LMK0482x id_prod1: 0x%x\n",lmk_id_prod1);

   lmk_status |= lmk_spi_read(LXS, 0x006, &lmk_maskrev);
   printk(KERN_DEBUG "LXS_LMK0482x maskrev: 0x%x\n",lmk_maskrev);

   lmk_status |= lmk_spi_read(LXS, 0x00C, &lmk_vendor0);
   printk(KERN_DEBUG "LXS_LMK0482x vendor0: 0x%x\n",lmk_vendor0);

   lmk_status |= lmk_spi_read(LXS, 0x00D, &lmk_vendor1);
   printk(KERN_DEBUG "LXS_LMK0482x vendor1: 0x%x\n",lmk_vendor1);

   printk(KERN_DEBUG "LXS_LMK0482x status: %d\n",lmk_status);

   //    u32 adc_0_val = 0xF00D;

   udelay(100);
   // adc_spi_write( 0x05, 0xf0f0, ADC_0_SPIC_DEVICE_ID);

   //reg_002.PAT_MODES = 0x03; // custom
   //reg_002.PAT_MODES = 0x06; // zeroes
   //reg_002.PAT_MODES = 0x07; // ramp
   //reg_002.PAT_MODES = 0x05; // toggle
   //reg_002.PAT_MODES = 0x04; // ones

   // spi command to tell adc to output selected pattern:
   adc_spi_write(LXS, 0x02, 0x07, ADC_DEVICE_ID_0);
   adc_spi_write(LXS, 0x02, 0x07, ADC_DEVICE_ID_1);

   return;
}

int lmk_initialize(struct LXS_drvdata *LXS)
{
	int status = 0;

	status |= lmk_unlock_spi(LXS);

	// First device setting is to be this value, as per
	// "SNAS605AR – MARCH 2013 – REVISED DECEMBER 2015" page 49

	status |= lmk_spi_write(LXS, 0x000, 0x80);
	status |= lmk_spi_write(LXS, 0x000, 0x00);

   return status;
}


int lmk_initialize_raw(struct LXS_drvdata *LXS)
{
	u8 status = 0;
	// Allow reg write
	status |= lmk_spi_write(LXS, 0x1FFD, 0x00);
	status |= lmk_spi_write(LXS, 0x1FFE, 0x00);
	status |= lmk_spi_write(LXS, 0x1FFF, 0x53);

	//reset
	status |= lmk_spi_write(LXS, 0x0000, 0x80);
	status |= lmk_spi_write(LXS, 0x0000, 0x00);

	// push-pull
	status |= lmk_spi_write(LXS, 0x0149, 0x01);


	status |= lmk_spi_write(LXS, 256, 0x02);
	status |= lmk_spi_write(LXS, 257, 0x00);
	status |= lmk_spi_write(LXS, 259, 0x02);
	status |= lmk_spi_write(LXS, 260, 0x00);
	status |= lmk_spi_write(LXS, 261, 0x00);
	status |= lmk_spi_write(LXS, 262, 0x00);
	status |= lmk_spi_write(LXS, 263, 0x01);
	status |= lmk_spi_write(LXS, 264, 0x01);
	status |= lmk_spi_write(LXS, 265, 0x00);
	status |= lmk_spi_write(LXS, 267, 0x02);
	status |= lmk_spi_write(LXS, 268, 0x00);
	status |= lmk_spi_write(LXS, 269, 0x00);
	status |= lmk_spi_write(LXS, 270, 0x00);
	status |= lmk_spi_write(LXS, 271, 0x01);
	status |= lmk_spi_write(LXS, 272, 0x01);
	status |= lmk_spi_write(LXS, 273, 0x00);
	status |= lmk_spi_write(LXS, 275, 0x02);
	status |= lmk_spi_write(LXS, 276, 0x00);
	status |= lmk_spi_write(LXS, 277, 0x00);
	status |= lmk_spi_write(LXS, 278, 0x00);
	status |= lmk_spi_write(LXS, 279, 0x01);
	status |= lmk_spi_write(LXS, 280, 0x01);
	status |= lmk_spi_write(LXS, 281, 0x00);
	status |= lmk_spi_write(LXS, 283, 0x02);
	status |= lmk_spi_write(LXS, 284, 0x00);
	status |= lmk_spi_write(LXS, 285, 0x00);
	status |= lmk_spi_write(LXS, 286, 0x00);
	status |= lmk_spi_write(LXS, 287, 0x00);
	status |= lmk_spi_write(LXS, 288, 0x01);
	status |= lmk_spi_write(LXS, 289, 0x00);
	status |= lmk_spi_write(LXS, 291, 0x02);
	status |= lmk_spi_write(LXS, 292, 0x00);
	status |= lmk_spi_write(LXS, 293, 0x00);
	status |= lmk_spi_write(LXS, 294, 0x00);
	status |= lmk_spi_write(LXS, 295, 0x01);
	status |= lmk_spi_write(LXS, 296, 0x01);
	status |= lmk_spi_write(LXS, 297, 0x00);
	status |= lmk_spi_write(LXS, 299, 0x02);
	status |= lmk_spi_write(LXS, 300, 0x00);
	status |= lmk_spi_write(LXS, 301, 0x00);
	status |= lmk_spi_write(LXS, 302, 0x00);
	status |= lmk_spi_write(LXS, 303, 0x00);
	status |= lmk_spi_write(LXS, 304, 0x01);
	status |= lmk_spi_write(LXS, 305, 0x00);
	status |= lmk_spi_write(LXS, 307, 0x02);
	status |= lmk_spi_write(LXS, 308, 0x00);
	status |= lmk_spi_write(LXS, 309, 0x00);
	status |= lmk_spi_write(LXS, 310, 0x00);
	status |= lmk_spi_write(LXS, 311, 0x00);
	status |= lmk_spi_write(LXS, 312, 0x40);
	status |= lmk_spi_write(LXS, 313, 0x00);
	status |= lmk_spi_write(LXS, 314, 0x00);
	status |= lmk_spi_write(LXS, 315, 0x01);
	status |= lmk_spi_write(LXS, 316, 0x00);
	status |= lmk_spi_write(LXS, 317, 0x00);
	status |= lmk_spi_write(LXS, 318, 0x00);
	status |= lmk_spi_write(LXS, 319, 0x00);
	status |= lmk_spi_write(LXS, 320, 0xE0);
	status |= lmk_spi_write(LXS, 321, 0x00);
	status |= lmk_spi_write(LXS, 322, 0x00);
	status |= lmk_spi_write(LXS, 323, 0x00);
	status |= lmk_spi_write(LXS, 324, 0x00);
	status |= lmk_spi_write(LXS, 325, 0x00);
	status |= lmk_spi_write(LXS, 326, 0x10);
	status |= lmk_spi_write(LXS, 327, 0x00);
	status |= lmk_spi_write(LXS, 328, 0x00);
	status |= lmk_spi_write(LXS, 330, 0x00);
	status |= lmk_spi_write(LXS, 331, 0x00);
	status |= lmk_spi_write(LXS, 332, 0x00);
	status |= lmk_spi_write(LXS, 333, 0x00);
	status |= lmk_spi_write(LXS, 334, 0x00);
	status |= lmk_spi_write(LXS, 335, 0x00);
	status |= lmk_spi_write(LXS, 336, 0x00);
	status |= lmk_spi_write(LXS, 337, 0x00);
	status |= lmk_spi_write(LXS, 338, 0x00);
	status |= lmk_spi_write(LXS, 339, 0x00);
	status |= lmk_spi_write(LXS, 340, 0x00);
	status |= lmk_spi_write(LXS, 341, 0x00);
	status |= lmk_spi_write(LXS, 342, 0x01);
	status |= lmk_spi_write(LXS, 343, 0x00);
	status |= lmk_spi_write(LXS, 344, 0x00);
	status |= lmk_spi_write(LXS, 345, 0x00);
	status |= lmk_spi_write(LXS, 346, 0x00);
	status |= lmk_spi_write(LXS, 347, 0x00);
	status |= lmk_spi_write(LXS, 348, 0x00);
	status |= lmk_spi_write(LXS, 349, 0x00);
	status |= lmk_spi_write(LXS, 350, 0x00);
	status |= lmk_spi_write(LXS, 351, 0x00);
	status |= lmk_spi_write(LXS, 352, 0x00);
	status |= lmk_spi_write(LXS, 353, 0x01);
	status |= lmk_spi_write(LXS, 354, 0x00);
	status |= lmk_spi_write(LXS, 355, 0x00);
	status |= lmk_spi_write(LXS, 356, 0x00);
	status |= lmk_spi_write(LXS, 357, 0x00);
	status |= lmk_spi_write(LXS, 380, 0x00);
	status |= lmk_spi_write(LXS, 381, 0x00);
	status |= lmk_spi_write(LXS, 358, 0x00);
	status |= lmk_spi_write(LXS, 359, 0x00);
	status |= lmk_spi_write(LXS, 360, 0x00);
	status |= lmk_spi_write(LXS, 361, 0x00);
	status |= lmk_spi_write(LXS, 362, 0x00);
	status |= lmk_spi_write(LXS, 363, 0x00);
	status |= lmk_spi_write(LXS, 364, 0x00);
	status |= lmk_spi_write(LXS, 365, 0x00);
	status |= lmk_spi_write(LXS, 366, 0x00);
	status |= lmk_spi_write(LXS, 371, 0x60);

	return status;

}
