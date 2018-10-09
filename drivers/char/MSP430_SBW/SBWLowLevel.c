//***************************************************************************
//! \file       SBWLowLevel.c
//! \brief      Contains the low level functions for the SBW controller.
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
#include "SBWLowLevel.h"

// External References

//***************************************************************************
//! \fn      SBWRelease()
//! \brief   Performs a brown out reset via JTAG
//!
//! \returns void
//***************************************************************************
void SBWRelease(struct MSP430_SBW_drvdata *MSP430_SBW)
{
  // Set the reset output low
  RSTLow(MSP430_SBW);

  // Wait for 1 millisecond
  DelayMicroSeconds(1000);

  // Set the test output low
  TSTLow(MSP430_SBW);

  // Wait for 10 milliseconds
  DelayMicroSeconds(10000);

  // Set the reset output high
  RSTHigh(MSP430_SBW);

  // Return nothing
  return;
}

//***************************************************************************
//! \fn      SBWResetTAP()
//! \brief   Resets the SBW Test Access Port (TAP)
//!
//! \returns void
//***************************************************************************
void SBWResetTAP(struct MSP430_SBW_drvdata *MSP430_SBW)
{
  uint8_t count;

  // Execute the following sequence at least 6 times to reset the TAP FSM
  for (count = 6; count > 0; count--)
  {
    // Set TMS High
    TMSHigh(MSP430_SBW);

    // Set TDI high
    TDIHigh(MSP430_SBW);

    // Read the TDO data but ignore
    TDORead(MSP430_SBW);
  }

  // Set TMS Low to set the TAP FSM to idle
  TMSLow(MSP430_SBW);

  // Set TDI high
  TDIHigh(MSP430_SBW);

  // Read the TDO data but ignore
  TDORead(MSP430_SBW);

  // Wait for 20 microseconds
  DelayMicroSeconds(20);
}

//***************************************************************************
//! \fn      SBWRestart()
//! \brief   Restarts SBW JTAG communication with reset high
//!
//! \returns void
//***************************************************************************
void SBWRestart(struct MSP430_SBW_drvdata *MSP430_SBW)
{
  // Set the test output low
  TSTLow(MSP430_SBW);

  // Wait for 4 milliseconds
  DelayMicroSeconds(4000);

  // Set the reset output high
  RSTHigh(MSP430_SBW);

  // Set the test output high
  TSTHigh(MSP430_SBW);

  // Wait for 20 millisecond
  DelayMicroSeconds(20000);

  // Set the reset output high
  RSTHigh(MSP430_SBW);

  // Wait for 60 microseconds
  DelayMicroSeconds(60);

  // Set the test output low
  TSTLow(MSP430_SBW);

  // Wait for 1 microsecond
  DelayMicroSeconds(1);

  // Set the test output high
  TSTHigh(MSP430_SBW);

  // Wait for 5 millisecond
  DelayMicroSeconds(5000);
}

//***************************************************************************
//! \fn      SBWShiftIR()
//! \brief   This function transfers data to/from the SBW IR register
//!
//! \param   instruction    uint8_t     IR register instruction
//!
//! \returns the 8-bit IR data from the target
//***************************************************************************
u8 SBWShiftIR(struct MSP430_SBW_drvdata *MSP430_SBW, u8 instruction)
{
  u8  result;
  u8  count;

  // Set TMS High to select DR-Scan SBW FSM state
  TMSHigh(MSP430_SBW);

  // Check the TCLK state
  if( MSP430_SBW->tclkState == 0 )
  {
    // Set TDI low
    TDILow(MSP430_SBW);
  }
  else
  {
    // Set TDI high
    TDIHigh(MSP430_SBW);
  }

  // Read the TDO data but ignore
  TDORead(MSP430_SBW);

  // Set TMS High to select IR-Scan SBW FSM state
  TMSHigh(MSP430_SBW);

  // Set TDI high
  TDIHigh(MSP430_SBW);

  // Read the TDO data but ignore
  TDORead(MSP430_SBW);

  // Set TMS Low to select Capture-IR SBW FSM state
  TMSLow(MSP430_SBW);

  // Set TDI high
  TDIHigh(MSP430_SBW);

  // Read the TDO data but ignore
  TDORead(MSP430_SBW);

  // Set TMS Low to select Shift-IR SBW FSM state
  TMSLow(MSP430_SBW);

  // Set TDI high
  TDIHigh(MSP430_SBW);

  // Read the TDO data but ignore
  TDORead(MSP430_SBW);

  // Clear the result
  result = 0u;

  // Repeat for all 8 bits
  for( count = 8; count > 0; count--)
  {
    // Check if the last bit is being sent
    if( count == 1 )
    {
      // Set TMS high to select Exit1-IR SBW FSM state
      TMSHigh(MSP430_SBW);
    }
    else
    {
      // Set TMS low
      TMSLow(MSP430_SBW);
    }

    // Check if the bit to be transferred is high
    if(( instruction & (1 << ( count - 1 ))) > 0 )
    {
      // Set TDI high
      TDIHigh(MSP430_SBW);
    }
    else
    {
      // Set TDI low
      TDILow(MSP430_SBW);
    }

    // Shift the result left to make space for the new data
    result <<= 1;

    // Get the data from the target
    result += TDORead(MSP430_SBW);
  }

  // Set TMS High to select Update-IR SBW FSM state
  TMSHigh(MSP430_SBW);

  // Set TDI high
  TDIHigh(MSP430_SBW);

  // Read the TDO data but ignore
  TDORead(MSP430_SBW);

  // Set TMS Low to select Idle SBW FSM state
  TMSLow(MSP430_SBW);

  // Check the TCLK state
  if( MSP430_SBW->tclkState == 0 )
  {
    // Set TDI low
    TDILow(MSP430_SBW);
  }
  else
  {
    // Set TDI high
    TDIHigh(MSP430_SBW);
  }

  // Read the TDO data but ignore
  TDORead(MSP430_SBW);

  // Wait for 20 microseconds
  DelayMicroSeconds(20);

  // Return the result
  return(result);
}

//***************************************************************************
//! \fn      SBWShiftDR16()
//! \brief   This function transfers 16-bit data to/from the SBW DR register
//!
//! \param   instruction    uint16_t    DR register data
//!
//! \returns the 16-bit data from the target
//***************************************************************************
u16 SBWShiftDR16(struct MSP430_SBW_drvdata *MSP430_SBW, u16 data)
{
  u16 result;
  u16 mask;
  u8  count;

  // Set the initial mask for the bit transfer
  mask = 0x8000;

  // Clear the result
  result = 0u;

  // Set TMS High to select DR-Scan SBW FSM state
  TMSHigh(MSP430_SBW);

  // Check the TCLK state
  if(MSP430_SBW->tclkState == 0 )
  {
    // Set TDI low
    TDILow(MSP430_SBW);
  }
  else
  {
    // Set TDI high
    TDIHigh(MSP430_SBW);
  }

  // Read the TDO data but ignore
  TDORead(MSP430_SBW);

  // Set TMS Low to select Capture-DR SBW FSM state
  TMSLow(MSP430_SBW);

  // Set TDI high
  TDIHigh(MSP430_SBW);

  // Read the TDO data but ignore
  TDORead(MSP430_SBW);

  // Set TMS Low to select Shift-DR SBW FSM state
  TMSLow(MSP430_SBW);

  // Set TDI high
  TDIHigh(MSP430_SBW);

  // Read the TDO data but ignore
  TDORead(MSP430_SBW);

  // Repeat for all 16 bits
  for( count = 16; count > 0; count--)
  {
    // Check if the last bit is being sent
    if( count == 1 )
    {
      // Set TMS high to select Exit1-DR SBW FSM state
      TMSHigh(MSP430_SBW);
    }
    else
    {
      // Set TMS low
      TMSLow(MSP430_SBW);
    }

    // Check if the bit to be transferred is high
    if(( data & mask ) > 0 )
    {
      // Set TDI high
      TDIHigh(MSP430_SBW);
    }
    else
    {
      // Set TDI low
      TDILow(MSP430_SBW);
    }

    // Shift the result left to make space for the new data
    result <<= 1;

    // Get the data from the target
    result += TDORead(MSP430_SBW);

    // Update the mask
    mask = ( mask >> 1 ) & 0x7FFF;
  }

  // Set TMS High to select Update-DR SBW FSM state
  TMSHigh(MSP430_SBW);

  // Set TDI high
  TDIHigh(MSP430_SBW);

  // Read the TDO data but ignore
  TDORead(MSP430_SBW);

  // Set TMS Low to select Idle SBW FSM state
  TMSLow(MSP430_SBW);

  // Check the TCLK state
  if(MSP430_SBW->tclkState == 0 )
  {
    // Set TDI low
    TDILow(MSP430_SBW);
  }
  else
  {
    // Set TDI high
    TDIHigh(MSP430_SBW);
  }

  // Read the TDO data but ignore
  TDORead(MSP430_SBW);

  // Wait for 10 microseconds
  DelayMicroSeconds(10);

  // Return the result
  return(result);
}

//***************************************************************************
//! \fn      SBWShiftDR20()
//! \brief   This function transfers 20-bit data to/from the SBW DR register
//!
//! \param   instruction    uint32_t     DR register address
//!
//! \returns the 20-bit DR data from the target
//***************************************************************************
u32 SBWShiftDR20(struct MSP430_SBW_drvdata *MSP430_SBW, u32 address)
{
  u32 result;
  u32 mask;
  u8  count;

  // Set the initial mask for the bit transfer
  mask = 0x00080000;

  // Clear the result
  result = 0u;

  // Set TMS High to select DR-Scan SBW FSM state
  TMSHigh(MSP430_SBW);

  // Check the TCLK state
  if(MSP430_SBW->tclkState == 0 )
  {
    // Set TDI low
    TDILow(MSP430_SBW);
  }
  else
  {
    // Set TDI high
    TDIHigh(MSP430_SBW);
  }

  // Read the TDO data but ignore
  TDORead(MSP430_SBW);

  // Set TMS Low to select Capture-DR SBW FSM state
  TMSLow(MSP430_SBW);

  // Set TDI high
  TDIHigh(MSP430_SBW);

  // Read the TDO data but ignore
  TDORead(MSP430_SBW);

  // Set TMS Low to select Shift-DR SBW FSM state
  TMSLow(MSP430_SBW);

  // Set TDI high
  TDIHigh(MSP430_SBW);

  // Read the TDO data but ignore
  TDORead(MSP430_SBW);

  // Repeat for all 20 bits
  for( count = 20; count > 0; count--)
  {
    // Check if the last bit is being sent
    if( count == 1 )
    {
      // Set TMS high to select Exit1-DR SBW FSM state
      TMSHigh(MSP430_SBW);
    }
    else
    {
      // Set TMS low
      TMSLow(MSP430_SBW);
    }

    // Check if the bit to be transferred is high
    if(( address & mask ) > 0 )
    {
      // Set TDI high
      TDIHigh(MSP430_SBW);
    }
    else
    {
      // Set TDI low
      TDILow(MSP430_SBW);
    }

    // Shift the result left to make space for the new data
    result <<= 1;

    // Get the data from the target
    result += TDORead(MSP430_SBW);

    // Update the mask
    mask = ( mask >> 1 ) & 0x7FFFFFFF;
  }

  // Unscramble the return data
  //result = (( result << 16 ) + ( result >> 4 )) & 0x00FFFFF;

  // Set TMS High to select Update-DR SBW FSM state
  TMSHigh(MSP430_SBW);

  // Set TDI high
  TDIHigh(MSP430_SBW);

  // Read the TDO data but ignore
  TDORead(MSP430_SBW);

  // Set TMS Low to select Idle SBW FSM state
  TMSLow(MSP430_SBW);

  // Check the TCLK state
  if(MSP430_SBW->tclkState == 0 )
  {
    // Set TDI low
    TDILow(MSP430_SBW);
  }
  else
  {
    // Set TDI high
    TDIHigh(MSP430_SBW);
  }

  // Read the TDO data but ignore
  TDORead(MSP430_SBW);

  // Wait for 10 microseconds
  DelayMicroSeconds(10);

  // Return the result
  return(result);
}

//***************************************************************************
//! \fn      SBWStart()
//! \brief   Starts SBW JTAG communication. Must exit with the test output
//!          high
//!
//! \returns void
//***************************************************************************
void SBWStart(struct MSP430_SBW_drvdata *MSP430_SBW)
{
  // Set the test output low
  TSTLow(MSP430_SBW);

  // Wait for 1 milliseconds
  DelayMicroSeconds(1000);

  // Set the reset output low
  RSTLow(MSP430_SBW);

  // Wait for 50 milliseconds
  DelayMicroSeconds(50000);

  // Set the test output high
  TSTHigh(MSP430_SBW);

  // Wait for 100 millisecond
  DelayMicroSeconds(100000);

  // Set the reset output high
  RSTHigh(MSP430_SBW);

  // Wait for 40 microseconds
  DelayMicroSeconds(40);

  // Set the test output low
  TSTLow(MSP430_SBW);

  // Wait for 1 microseconds
  DelayMicroSeconds(1);

  // Set the test output high
  TSTHigh(MSP430_SBW);

  // Wait for 5 millisecond
  DelayMicroSeconds(5000);
}

//***************************************************************************
//! \fn      SBWTCLKHigh()
//! \brief   Sets the SBW TCLK high
//!
//! \returns void
//***************************************************************************
void SBWTCLKHigh(struct MSP430_SBW_drvdata *MSP430_SBW)
{
  // Check the previous state of TCLK
  if(MSP430_SBW->tclkState == 1 )
  {
    // Set the TMS low then high
    TMSLowHigh(MSP430_SBW);
  }
  else
  {
    // Set the TMS low
    TMSLow(MSP430_SBW);
  }

  // Set the TDI high
  TDIHigh(MSP430_SBW);

  // Read the TDO data but ignore
  TDORead(MSP430_SBW);

  // Set the TCLK state
 MSP430_SBW->tclkState = 1;

  // Wait for 20 microseconds
  DelayMicroSeconds(20);
}

//***************************************************************************
//! \fn      SBWTCLKLow()
//! \brief   Sets the SBW TCLK low
//!
//! \returns void
//***************************************************************************
void SBWTCLKLow(struct MSP430_SBW_drvdata *MSP430_SBW)
{
  // Check the previous state of TCLK
  if(MSP430_SBW->tclkState == 1 )
  {
    // Set the TMS low then high
    TMSLowHigh(MSP430_SBW);
  }
  else
  {
    // Set the TMS low
    TMSLow(MSP430_SBW);
  }

  // Set the TDI low
  TDILow(MSP430_SBW);

  // Read the TDO data but ignore
  TDORead(MSP430_SBW);

  // Clear the TCLK state
 MSP430_SBW->tclkState = 0;

  // Wait for 20 microseconds
  DelayMicroSeconds(20);
}

//***************************************************************************
//! \fn      SBWUpdateDR()
//! \brief   Updates the JTAG Data Register with no data transfer
//!
//! \returns void
//***************************************************************************
void SBWUpdateDR(struct MSP430_SBW_drvdata *MSP430_SBW)
{
  // Set the TMS high to move to Select DR Scan
  TMSHigh(MSP430_SBW);

  // Check the TCLK state
  if(MSP430_SBW->tclkState == 0 )
  {
    // Set TDI low
    TDILow(MSP430_SBW);
  }
  else
  {
    // Set TDI high
    TDIHigh(MSP430_SBW);
  }

  // Read the TDO data but ignore
  TDORead(MSP430_SBW);

  // Set the TMS low to move to Capture DR
  TMSLow(MSP430_SBW);

  // Set the TDI high
  TDIHigh(MSP430_SBW);

  // Read the TDO data but ignore
  TDORead(MSP430_SBW);

  // Set the TMS high to move to Exit1 DR
  TMSHigh(MSP430_SBW);

  // Set the TDI high
  TDIHigh(MSP430_SBW);

  // Read the TDO data but ignore
  TDORead(MSP430_SBW);

  // Set the TMS high to move to Update DR Scan
  TMSHigh(MSP430_SBW);

  // Set the TDI high
  TDIHigh(MSP430_SBW);

  // Read the TDO data but ignore
  TDORead(MSP430_SBW);

  // Set the TMS low to move to Idle
  TMSLow(MSP430_SBW);

  // Check the TCLK state
  if(MSP430_SBW->tclkState == 0 )
  {
    // Set TDI low
    TDILow(MSP430_SBW);
  }
  else
  {
    // Set TDI high
    TDIHigh(MSP430_SBW);
  }

  // Read the TDO data but ignore
  TDORead(MSP430_SBW);

  // Wait for 20 microseconds
  DelayMicroSeconds(20);
}
