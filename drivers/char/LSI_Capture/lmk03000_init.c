/*****************************************************************************/
/* Filename:    lmk03000_test.c                                              */
/*****************************************************************************/
// Hardware signals controlled by the software driver are:
//
// 'Clocks_vco_goe'   - Controlled by setting/clearing bit 0 in the CONTROL register
// 'Clocks_vco_sync'  - Controlled by setting/clearing bit 1 in the CONTROL register
// 'Clocks_vco_clk'   - Automatically controlled by writing to TX or RX register
// 'Clocks_vco_data'  - Automatically controlled by writing to TX or RX register
// 'Clocks_vco_le'    - Automatically controlled by writing to TX or RX register
// 'Clocks_vco_ld'    - Not used
//
// There are three registers:
//
// (1) STATUS (R) - Cleared when read
//
//   31  30  29  28  27  26  25  24  23  22  21  20  19  18  17  16
// +---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+
// | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 |
// +---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+
//
//   15  14  13  12  11  10  9   8   7   6   5   4   3   2   1   0
// +---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+
// | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 |LD |TX |
// +---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+
//
// Initial state is "00"
//
// TX = '1' TX is complete
// L  = '1' Lock detected
//
// (2) CONTROL (R/W)
//
//   31  30  29  28  27  26  25  24  23  22  21  20  19  18  17  16
// +---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+
// | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 |
// +---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+
//
//   15  14  13  12  11  10  9   8   7   6   5   4   3   2   1   0
// +---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+
// | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 |GO |SY |
// +---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+
//
// Initial state is "00"
//
// SY = 0 "Clocks_vco_sync" is de-asserted (Clocks_vco_sync = 1)
// SY = 1 "Clocks_vco_sync" is asserted (Clocks_vco_sync = 0)
//
// GO = 0 "Clocks_vco_goe" is de-asserted (Clocks_vco_goe = 0)
// GO = 1 "Clocks_vco_goe" is asserted (Clocks_vco_goe = 1)
//
// (3) TX (W)
//
//   31  30  29  28  27  26  25  24  23  22  21  20  19  18  17  16
// +---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+
// |D31|D30|D29|D28|D27|D26|D25|D24|D23|D22|D21|D20|D19|D18|D17|D16|
// +---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+
//
//   15  14  13  12  11  10  9   8   7   6   5   4   3   2   1   0
// +---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+
// |D15|D14|D13|D12|D11|D10|D9 |D8 |D7 |D6 |D5 |D4 |D3 |D2 |D1 |D0 |
// +---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+
//
// D[31:0] = TX Data
//

/***************************** Include Files *********************************/
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

#include <asm/uaccess.h>
#include <linux/dma-mapping.h>
#include <xen/page.h>

#include <linux/slab.h>
#include <linux/platform_device.h>

//#include <linux/dma-mapping.h>
//#include "xparameters.h"
//#include "xil_printf.h"
//#include "xil_io.h"
//#include "sleep.h"
#include "lmk03000.h"
#include "LSI.h"
#include "LSI_system.h"

// Use 120MHz otherwise 100MHz
#define USE_120MHz

/*
#ifdef USE_120MHz
#define LMK03000_CLK_DIV_USE        LMK03000_CLK_DIV_2
#define LMK03000_VCO_DIV_USE        LMK03000_VCO_DIV_5_120MHz
#define LMK03000_PLL_N_USE          LMK03000_PLL_N_120MHz
#else
#define LMK03000_CLK_DIV_USE        LMK03000_CLK_DIV_4
#define LMK03000_VCO_DIV_USE        LMK03000_VCO_DIV_3_100MHz
#define LMK03000_PLL_N_USE          LMK03000_PLL_N_100MHz
#endif
*/
void lmk03000_init (struct LSI_drvdata *LSI, u32 freq)
{
   u32 lmk03000_init[15];
   u32 status;
   int index = 0;

   u32 lmk03000_clk_div_use;
   u32 lmk03000_vco_div_use;
   u32 lmk03000_pll_n_use;

   if (freq == 120) {
      lmk03000_clk_div_use = LMK03000_CLK_DIV_2;
      lmk03000_vco_div_use = LMK03000_VCO_DIV_5_120MHz;
      lmk03000_pll_n_use = LMK03000_PLL_N_120MHz;
   } else {
      // 100Mhz
      lmk03000_clk_div_use = LMK03000_CLK_DIV_4;
      lmk03000_vco_div_use = LMK03000_VCO_DIV_3_100MHz;
      lmk03000_pll_n_use = LMK03000_PLL_N_100MHz;
   }

   // Drive "Clocks_vco_goe" high to enable outputs
   printk(KERN_DEBUG "\n\rWrite => Control Register = %02x", (LMK03000_GOE_HIGH) );
   LSI_write_reg(LSI, R_LMK_CNTL_ADDR, LMK03000_GOE_HIGH);  //LMK03000_CONTROL_WRITE(LMK03000_GOE_HIGH);

   // R0 - Init (CLK_DIV has to be non zero)
   lmk03000_init[0]        = (LMK03000_RESET)            + (LMK03000_CLK_DIV_2)                + (LMK03000_R0);

   // R0 (CLKOUT0_P/N) - 120MHz LVDS (VCO_FPGA_CLK_P/N)
   lmk03000_init[1]        = (LMK03000_CLK_MUX_DIVIDED)  + (LMK03000_CLK_EN_ENABLED)           + (lmk03000_clk_div_use) +
      (LMK03000_CLK_DLY_0)        + (LMK03000_R0);

   // R1 (CLKOUT1_P/N) - Not used
   lmk03000_init[2]        = (LMK03000_CLK_EN_DISABLED)  + (LMK03000_CLK_DIV_2)                + (LMK03000_R1);

   // R2 (CLKOUT2_P/N) - Not used
   lmk03000_init[3]        = (LMK03000_CLK_EN_DISABLED)  + (LMK03000_CLK_DIV_2)                + (LMK03000_R2);

   // R3 (CLKOUT3_P/N) - 120MHz LVPECL (VCO_OADC_CLK0_P/N)
   lmk03000_init[4]        = (LMK03000_CLK_MUX_DIVIDED)  + (LMK03000_CLK_EN_ENABLED)           + (lmk03000_clk_div_use) +
      (LMK03000_CLK_DLY_0)        + (LMK03000_R3);

   // R4 (CLKOUT4_P/N) - 120MHz LVPECL (VCO_OADC_CLK1_P/N)
   lmk03000_init[5]        = (LMK03000_CLK_MUX_DIVIDED)  + (LMK03000_CLK_EN_ENABLED)           + (lmk03000_clk_div_use) +
      (LMK03000_CLK_DLY_0)        + (LMK03000_R4);

   // R5 (CLKOUT5_P/N) - 120MHz LVPECL (VCO_OADC_CLK2_P/N)
   lmk03000_init[6]        = (LMK03000_CLK_MUX_DIVIDED)  + (LMK03000_CLK_EN_ENABLED)           + (lmk03000_clk_div_use) +
      (LMK03000_CLK_DLY_0)        + (LMK03000_R5);

   // R6 (CLKOUT6_P/N) - 120MHz LVPECL (VCO_OADC_CLK3_P/N)
   lmk03000_init[7]        = (LMK03000_CLK_MUX_DIVIDED)  + (LMK03000_CLK_EN_ENABLED)           + (lmk03000_clk_div_use) +
      (LMK03000_CLK_DLY_0)        + (LMK03000_R6);

   // R7 (CLKOUT7_P/N) - 120MHz LVPECL (VCO_OADC_CLK4_P/N)
   lmk03000_init[8]        = (LMK03000_CLK_MUX_DIVIDED)  + (LMK03000_CLK_EN_ENABLED)           + (lmk03000_clk_div_use) +
      (LMK03000_CLK_DLY_0)        + (LMK03000_R7);

   // R8
   lmk03000_init[9]        = (LMK03000_R8);
   // R9
   lmk03000_init[10]       = (LMK03000_VBOOST_DISABLED)  + (LMK03000_R9);
   // R11
   lmk03000_init[11]       = (LMK03000_DIV4_DISABLED)    + (LMK03000_R11);
   //R13
   lmk03000_init[12]       = (LMK03000_OSC_FREQ)         + (LMK03000_VCO_R4_LF)                + (LMK03000_VCO_R3_LF) +
      (LMK03000_VCO_C3_C4_LF)     + (LMK03000_R13);

   // R14
   lmk03000_init[13]       = (LMK03000_EN_FOUT_DISABLED) + (LMK03000_EN_CLKOUT_GLOBAL_ENABLED) + (LMK03000_POWERDOWN_DISABLED) +
      (LMK03000_PLL_MUX_PP_AH)    + (LMK03000_PLL_R)                    + (LMK03000_R14);

   // R15
   lmk03000_init[14]       = (LMK03000_PLL_CP_GAIN_32x)  + (lmk03000_vco_div_use)         +
      (lmk03000_pll_n_use)        + (LMK03000_R15);

   status = LSI_read_reg(LSI, R_LMK_STATUS_ADDR); //LMK03000_STATUS_READ;
   printk(KERN_DEBUG "\n\rRead  => Status Register = %02x\n", status);
   // Check if locked
   if ((LMK03000_LD_MASK & status) == LMK03000_LD_MASK) {
      printk(KERN_DEBUG "\n\rInfo  => Locked LD is 1\n");
   } else {
      printk(KERN_DEBUG "\n\rInfo  => Not Locked LD is 0\n");
   }

   udelay(10); // wait 10 us

   // Write R0 to R15
   for (index = 0; index < 15; index++) {
      printk(KERN_DEBUG "\n\rWrite => Data = %08x", lmk03000_init[index]);
      LSI_write_reg(LSI, R_LMK_DATA_ADDR, lmk03000_init[index]); // LMK03000_TX_WRITE(lmk03000_init[index]);
      udelay(10);
      status = LSI_read_reg(LSI, R_LMK_STATUS_ADDR); //LMK03000_STATUS_READ;
      printk(KERN_DEBUG "\n\rRead  => Status Register = %02x", status);
   }

   // Wait for Lock
   udelay(1000); // wait for 1 s

   // Check Lock
   status = LSI_read_reg(LSI, R_LMK_STATUS_ADDR); //LMK03000_STATUS_READ;
   printk(KERN_DEBUG "\n\rRead  => Status Register = %02x", status );
   // Check if locked
   if ((LMK03000_LD_MASK & status) == LMK03000_LD_MASK) {
      printk(KERN_DEBUG "\n\rInfo  => Locked LD is 1\n\r");
   } else {
      printk(KERN_DEBUG "\n\rInfo  => Not Locked LD is 0\n\r");
   }

   // Init has been done
   return;
}
