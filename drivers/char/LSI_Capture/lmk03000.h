#ifndef LMK03000_H		/* prevent circular inclusions */
#define LMK03000_H		/* by using protection macros */

// R0    - Bit 31
#define LMK03000_RESET                           1 << 31     // 1 bit

// R0-R7 - Bits 18:17
#define LMK03000_CLK_MUX_BYPASSED                0 << 17     // 2 bit
#define LMK03000_CLK_MUX_DIVIDED                 1 << 17     // 2 bit
#define LMK03000_CLK_MUX_DELAYED                 2 << 17     // 2 bit
#define LMK03000_CLK_MUX_DIVIDED_DELAYED         3 << 17     // 2 bit
// R0-R7 - Bit 16
#define LMK03000_CLK_EN_ENABLED                  1 << 16     // 1 bit
#define LMK03000_CLK_EN_DISABLED                 0 << 16     // 1 bit
// R0-R7 - Bits 15:8
#define LMK03000_CLK_DIV_2                       1 << 8      // 8 bit
#define LMK03000_CLK_DIV_4                       2 << 8      // 8 bit
// R0-R7 - Bits 7:4
#define LMK03000_CLK_DLY_0                       0 << 4      // 4 bit 0 ps
#define LMK03000_CLK_DLY_150                     1 << 4      // 4 bit 150 ps

// R0-R7
#define LMK03000_R0                              0
#define LMK03000_R1                              1
#define LMK03000_R2                              2
#define LMK03000_R3                              3
#define LMK03000_R4                              4
#define LMK03000_R5                              5
#define LMK03000_R6                              6
#define LMK03000_R7                              7

// R8
#define LMK03000_R8                              0x10000908

// R9
#define LMK03000_R9                              0xA0022A09
#define LMK03000_VBOOST_ENABLED                  1 << 16
#define LMK03000_VBOOST_DISABLED                 0 << 16

// R11 (DIV4=0)
#define LMK03000_R11                             0x0082000B
#define LMK03000_DIV4_ENABLED                    1 << 15
#define LMK03000_DIV4_DISABLED                   0 << 15

// R13
#define LMK03000_R13                             0x0280000D  // 13
#define LMK03000_OSC_FREQ                        10 << 14    // 10MHz
#define LMK03000_VCO_R4_LF                       0  << 11    // 0.2K
#define LMK03000_VCO_R3_LF                       0  << 8     // 0.6K
#define LMK03000_VCO_C3_C4_LF                    0  << 4     // C3=0pF C4=10pF

// R14
#define LMK03000_R14                             0x0000000E  // 14
#define LMK03000_EN_FOUT_ENABLED                 1  << 28    // 1 bit
#define LMK03000_EN_FOUT_DISABLED                0  << 28    // 1 bit
#define LMK03000_EN_CLKOUT_GLOBAL_ENABLED        1  << 27    // 1 bit
#define LMK03000_EN_CLKOUT_GLOBAL_DISABLED       0  << 27    // 1 bit
#define LMK03000_POWERDOWN_ENABLED               1  << 26    // 1 bit
#define LMK03000_POWERDOWN_DISABLED              0  << 26    // 1 bit
#define LMK03000_PLL_MUX_HZ                      0  << 20    // 4 bit
#define LMK03000_PLL_MUX_PP_AH                   3  << 20    // 4 bit - Digital Lock Detect
#define LMK03000_PLL_MUX_OD_NMOS                 6  << 20    // 4 bit
#define LMK03000_PLL_MUX_OD_PMOS                 7  << 20    // 4 bit
#define LMK03000_PLL_R                           1  << 8     // 12 bit

// R15
#define LMK03000_R15                             0x0000000F  // 15
#define LMK03000_PLL_CP_GAIN_1x                  0  << 30    // 2  bit
#define LMK03000_PLL_CP_GAIN_4x                  1  << 30    // 2  bit
#define LMK03000_PLL_CP_GAIN_16x                 2  << 30    // 2  bit
#define LMK03000_PLL_CP_GAIN_32x                 3  << 30    // 2  bit
#define LMK03000_VCO_DIV_3_100MHz                3  << 26    // 4  bit
#define LMK03000_VCO_DIV_5_120MHz                5  << 26    // 4  bit
#define LMK03000_PLL_N_100MHz                    40 << 8     // 18 bit
#define LMK03000_PLL_N_120MHz                    24 << 8     // 18 bit

// Change XPAR_SLOT_0_BASEADDR to match xparameters.h
#define LMK03000_BASEADDR                        0x43c10000

// LMK03000 Register offsets
//#define LMK03000_TX_REG_OFFSET                   0x00000000
//#define LMK03000_CONTROL_REG_OFFSET              0x00000004
//#define LMK03000_STATUS_REG_OFFSET               0x00000008

#define LMK03000_SYNC_LOW                        0x00000001
#define LMK03000_SYNC_HIGH                       0x00000000
#define LMK03000_GOE_LOW                         0x00000000  // disable clocks
#define LMK03000_GOE_HIGH                        0x00000002  // enable clocks

#define LMK03000_TX_MASK                         0x00000001
#define LMK03000_LD_MASK                         0x00000002

//#define LMK03000_STATUS_READ                     readl((LMK03000_BASEADDR) + (LMK03000_STATUS_REG_OFFSET))
//#define LMK03000_CONTROL_READ                    readl((LMK03000_BASEADDR) + (LMK03000_CONTROL_REG_OFFSET))
//#define LMK03000_CONTROL_WRITE(Data)             writel((u32)(Data), ((LMK03000_BASEADDR) + (LMK03000_CONTROL_REG_OFFSET)))
//#define LMK03000_TX_WRITE(Data)                  writel((u32)(Data), ((LMK03000_BASEADDR) + (LMK03000_TX_REG_OFFSET)))


#endif /* end of protection macro */
