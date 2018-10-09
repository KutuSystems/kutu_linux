/*
 * SBWInterface.c
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

// System Includes
#include <linux/module.h>
#include <linux/version.h>
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/kdev_t.h>
#include <linux/of.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/cdev.h>
#include <linux/clk.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/poll.h>
#include <linux/dma-mapping.h>
#include <linux/platform_device.h>
#include <linux/time.h>

// Application includes
#include "MSP430-SBW.h"
#include "MSP430-SBW-system.h"
#include "SBW_IO.h"
#include "SBWInterface.h"

//***************************************************************************
//! \fn      TDILow()
//! \brief   Sets the SBW TDI low
//!
//! \returns void
//***************************************************************************
void TDILow(struct MSP430_SBW_drvdata *MSP430_SBW)
{
  // Set the TDI data low by setting reset low
  RSTLow(MSP430_SBW);

  // Execute a one microsecond delay
  DelayMicroSeconds(1);

  // Set the clock low to clock in the TMS data by setting test low
  TSTLow(MSP430_SBW);

  // Execute a one microsecond delay
  DelayMicroSeconds(1);

  // Set the clock back to the default high state by setting test high
  TSTHigh(MSP430_SBW);
}

//***************************************************************************
//! \fn      TDIHigh()
//! \brief   Sets the SBW TDI high
//!
//! \returns void
//***************************************************************************
void TDIHigh(struct MSP430_SBW_drvdata *MSP430_SBW)
{
  // Set the TDI data high by setting reset high
  RSTHigh(MSP430_SBW);

  // Execute a one microsecond delay
  DelayMicroSeconds(1);

  // Set the clock low to clock in the TMS data by setting test low
  TSTLow(MSP430_SBW);

  // Execute a one microsecond delay
  DelayMicroSeconds(1);

  // Set the clock back to the default high state by setting test high
  TSTHigh(MSP430_SBW);

  // Return nothing
  return;
}

//***************************************************************************
//! \fn      TDORead()
//! \brief   Reads the SBW TDI signal (TDO from the target)
//!
//! \returns uint8_t containing the data from the target
//***************************************************************************
u8 TDORead(struct MSP430_SBW_drvdata *MSP430_SBW)
{
  uint8_t tdiData;

  // Set the reset pin to an input
  RSTInput(MSP430_SBW);

  // Execute a one microsecond delay
  DelayMicroSeconds(1);

  // Set the clock low to clock in the TMS data by setting test low
  TSTLow(MSP430_SBW);

  // Execute a one microsecond delay
  DelayMicroSeconds(1);

  // Check the TDI data
  tdiData = RSTRead(MSP430_SBW);

  // Set the clock back to the default high state by setting test high
  TSTHigh(MSP430_SBW);

  // Set the reset pin to an output
  RSTOutput(MSP430_SBW);

  // Return nothing the read data
  return(tdiData);
}

//***************************************************************************
//! \fn      TMSLow()
//! \brief   Sets the SBW TMS low
//!
//! \returns void
//***************************************************************************
void TMSLow(struct MSP430_SBW_drvdata *MSP430_SBW)
{
  // Set the TMS data low by setting reset low
  RSTLow(MSP430_SBW);

  // Execute a one microsecond delay
  DelayMicroSeconds(1);

  // Set the clock low to clock in the TMS data by setting test low
  TSTLow(MSP430_SBW);

  // Execute a one microsecond delay
  DelayMicroSeconds(1);

  // Set the clock back to the default high state by setting test high
  TSTHigh(MSP430_SBW);

  // Return nothing
  return;
}

//***************************************************************************
//! \fn      TMSLowHigh()
//! \brief   Sets the SBW TMS low and then high before the cycle end
//!
//! \returns void
//***************************************************************************
void TMSLowHigh(struct MSP430_SBW_drvdata *MSP430_SBW)
{
  // Set the TMS data low by setting reset low
  RSTLow(MSP430_SBW);

  // Execute a one microsecond delay
  DelayMicroSeconds(1);

  // Set the clock low to clock in the TMS data by setting test low
  TSTLow(MSP430_SBW);

  // Execute a one microsecond delay
  DelayMicroSeconds(1);

  // Set the TMS data high by setting reset high
  RSTHigh(MSP430_SBW);

  // Set the clock back to the default high state by setting test high
  TSTHigh(MSP430_SBW);

  // Return nothing
  return;
}

//***************************************************************************
//! \fn      TMSHigh()
//! \brief   Sets the SBW TMS high
//!
//! \returns void
//***************************************************************************
void TMSHigh(struct MSP430_SBW_drvdata *MSP430_SBW)
{
  // Set the TMS data high by setting reset high
  RSTHigh(MSP430_SBW);

  // Execute a one microsecond delay
  DelayMicroSeconds(1);

  // Set the clock low to clock in the TMS data by setting test low
  TSTLow(MSP430_SBW);

  // Execute a one microsecond delay
  DelayMicroSeconds(1);

  // Set the clock back to the default high state by setting test high
  TSTHigh(MSP430_SBW);
}
