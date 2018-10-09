/*
 * SBW_IO.h
 *
 * Copyright (c) 2018 Universal Biosensors Pty. Ltd.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 * These functions are the lowest level I/O functions for device specific
 * I/O functions.  This module provides a plafomr independent interface
 * to the rest of the driver.
 *
 */

#ifndef _SBW_IO_H_
#define _SBW_IO_H_

// Function Definitions
void DelayMicroSeconds(u32 delay);
void RSTInput(struct MSP430_SBW_drvdata *MSP430_SBW);
void RSTOutput(struct MSP430_SBW_drvdata *MSP430_SBW);
u8   RSTRead(struct MSP430_SBW_drvdata *MSP430_SBW);
void RSTLow(struct MSP430_SBW_drvdata *MSP430_SBW);
void RSTHigh(struct MSP430_SBW_drvdata *MSP430_SBW);
void TSTLow(struct MSP430_SBW_drvdata *MSP430_SBW);
void TSTHigh(struct MSP430_SBW_drvdata *MSP430_SBW);

#endif /* _SBW_IO_H_ */
