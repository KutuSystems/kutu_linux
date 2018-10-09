//***************************************************************************
//! \file       SBWLowLevel.h
//! \brief      Contains function definitions for the configuration of the
//!             SBW Low Level routines
//! \author     Rod Macdonald
//! \date       14/09/2018
//! \version    $Id$
//!
//! \par        Platform:
//!             Avatar CPU (Eventually)
//!             Code Composer Studio Version: 8.1.0.00011
//!
//! \par        Processor variant:
//!             MSP430FR5859
//!
//! \par        (c) Copyright Universal Biosensors Pty. Ltd. 2018
//!
//! @{
//***************************************************************************

#ifndef _SBW_LOW_LEVEL_H_
#define _SBW_LOW_LEVEL_H_

// Application includes

// Definitions and constants

// Function Definitions
void     SBWRelease  (struct MSP430_SBW_drvdata *MSP430_SBW);
void     SBWResetTAP (struct MSP430_SBW_drvdata *MSP430_SBW);
void     SBWRestart  (struct MSP430_SBW_drvdata *MSP430_SBW);
void     SBWStart    (struct MSP430_SBW_drvdata *MSP430_SBW);
u8       SBWShiftIR  (struct MSP430_SBW_drvdata *MSP430_SBW, u8 instruction);
u16      SBWShiftDR16(struct MSP430_SBW_drvdata *MSP430_SBW, u16 data);
u32      SBWShiftDR20(struct MSP430_SBW_drvdata *MSP430_SBW, u32 address);
void     SBWTCLKHigh (struct MSP430_SBW_drvdata *MSP430_SBW);
void     SBWTCLKLow  (struct MSP430_SBW_drvdata *MSP430_SBW);
void     SBWUpdateDR (struct MSP430_SBW_drvdata *MSP430_SBW);

#endif /* _SBW_LOW_LEVEL_H_ */
