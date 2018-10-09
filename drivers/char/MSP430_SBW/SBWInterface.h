/*
 * SBWInterface.h
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

#ifndef _SBW_INTERFACE_H_
#define _SBW_INTERFACE_H_

// Function Definitions
void    TDILow(struct MSP430_SBW_drvdata *MSP430_SBW);
void    TDIHigh(struct MSP430_SBW_drvdata *MSP430_SBW);
u8      TDORead(struct MSP430_SBW_drvdata *MSP430_SBW);
void    TMSLow(struct MSP430_SBW_drvdata *MSP430_SBW);
void    TMSLowHigh(struct MSP430_SBW_drvdata *MSP430_SBW);
void    TMSHigh(struct MSP430_SBW_drvdata *MSP430_SBW);

#endif /* _SBW_INTERFACE_H_ */
