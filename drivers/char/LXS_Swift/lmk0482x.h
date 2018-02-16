/*
 * lmk0482x.h
 *
 *  Created on: 8 Aug 2017
 *      Author: root
 */

#ifndef LMK0482X_H_
#define LMK0482X_H_

#include "LXS.h"
#include "LXS_system.h"

#define MAX_BUSY_RETRIES 10000

#define LMK_SPI_DEVICE_STATUS 	0x40
#define LMK_SPI_READ_CMD         0x40000000

int lmk_initialize(struct LXS_drvdata *LXS);
int lmk_initialize_raw(struct LXS_drvdata *LXS);
void lmk0482x_init(struct LXS_drvdata *LXS);
int lmk_spi_write( struct LXS_drvdata *LXS, u16 spi_address, u16 spi_data);
int lmk_spi_read(struct LXS_drvdata *LXS, u16 spi_address, u32* data_out);
int lmk_lock_spi(struct LXS_drvdata *LXS);
int lmk_unlock_spi(struct LXS_drvdata *LXS);


#endif /* LMK0482X_H_ */
