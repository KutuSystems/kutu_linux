/*
 * SBW_IO.c
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

//***************************************************************************
//! \fn      DelayMicroSeconds(delay)
//! \brief   Sets a delay of the specified micro seconds
//!
//! \param   delay      uint32_t    number of microseconds
//!
//! \returns void
//***************************************************************************
void DelayMicroSeconds(u32 delay)
{
   udelay(delay);
}

//***************************************************************************
//! \fn      RSTInput()
//! \brief   Sets the reset pin to an input
//!
//! \returns void
//***************************************************************************
void RSTInput(struct MSP430_SBW_drvdata *MSP430_SBW)
{
   u32 val;

   // Set the reset bit to an input
   val = MSP430_SBW_read_reg(MSP430_SBW, R_DIRECTION_ADDR) | BIT_RST;
   MSP430_SBW_write_reg(MSP430_SBW, R_DIRECTION_ADDR, val);
}

//***************************************************************************
//! \fn      RSTOutput()
//! \brief   Sets the reset pin to an output
//!
//! \returns void
//***************************************************************************
void RSTOutput(struct MSP430_SBW_drvdata *MSP430_SBW)
{
   u32 val;

   // Set the reset bit to an output
   val = MSP430_SBW_read_reg(MSP430_SBW, R_DIRECTION_ADDR) & ~BIT_RST;
   MSP430_SBW_write_reg(MSP430_SBW, R_DIRECTION_ADDR, val);
}
//***************************************************************************
//! \fn      RSTRead()
//! \brief   returns the value of the RST line (for TDO)
//!
//! \returns void
//***************************************************************************
u8   RSTRead(struct MSP430_SBW_drvdata *MSP430_SBW)
{
   u32 val;

   // Set the reset bit to an output
   val = MSP430_SBW_read_reg(MSP430_SBW, R_READ_ADDR) & BIT_RST;

   if (val)
      return 1;
   else
      return 0;
}

//***************************************************************************
//! \fn      RSTLow()
//! \brief   Sets the reset output low
//!
//! \returns void
//***************************************************************************
void RSTLow(struct MSP430_SBW_drvdata *MSP430_SBW)
{
   u32 val;

   // Set the nRST output low
   val = MSP430_SBW_read_reg(MSP430_SBW, R_READBACK_WR_ADDR) & ~BIT_RST;
   MSP430_SBW_write_reg(MSP430_SBW, R_WRITE_ADDR, val);
}

//***************************************************************************
//! \fn      RSTHigh()
//! \brief   Sets the reset output high
//!
//! \returns void
//***************************************************************************
void RSTHigh(struct MSP430_SBW_drvdata *MSP430_SBW)
{
   u32 val;

   // Set the nRST output high
   val = MSP430_SBW_read_reg(MSP430_SBW, R_READBACK_WR_ADDR) | BIT_RST;
   MSP430_SBW_write_reg(MSP430_SBW, R_WRITE_ADDR, val);
}

//***************************************************************************
//! \fn      TSTLow()
//! \brief   Sets the test output low
//!
//! \returns void
//***************************************************************************
void TSTLow(struct MSP430_SBW_drvdata *MSP430_SBW)
{
   u32 val;

   // Set the TEST output low
   val = MSP430_SBW_read_reg(MSP430_SBW, R_READBACK_WR_ADDR) & ~BIT_TEST;
   MSP430_SBW_write_reg(MSP430_SBW, R_WRITE_ADDR, val);
}

//***************************************************************************
//! \fn      TSTHigh()
//! \brief   Sets the test output high
//!
//! \returns void
//***************************************************************************
void TSTHigh(struct MSP430_SBW_drvdata *MSP430_SBW)
{
   u32 val;

   // Set the TEST output high
   val = MSP430_SBW_read_reg(MSP430_SBW, R_READBACK_WR_ADDR) | BIT_RST;
   MSP430_SBW_write_reg(MSP430_SBW, R_WRITE_ADDR, val);
}
