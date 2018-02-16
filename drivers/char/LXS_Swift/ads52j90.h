/*
 * ads52j90.h
 *
 *  Created on: 8 Aug 2017
 *      Author: root
 */

#ifndef SRC_ADS52J90_H
#define SRC_ADS52J90_H

#define ADC_DEVICE_ID_0 0
#define ADC_DEVICE_ID_1 1

#define ADC0_SPI_DEVICE_STATUS 	 0x10
#define ADC1_SPI_DEVICE_STATUS 	 0x20

int adc_spi_write(struct LXS_drvdata *LXS, u8 spi_address, u16 spi_data, int spi_device_id);
int adc_reset(struct LXS_drvdata *LXS);
int adc_power_down(struct LXS_drvdata *LXS);
int adc_initialize(struct LXS_drvdata *LXS);

#define ADC_POWERDOWN      0x3
#define ADC_RESET          0x18
#define ADC_POWERDOWN_FAST 0x60
#define ADC_SYNC           0x300


#endif /* SRC_ADS52J90_H */
