/*
 *  LSI_system.h -- Register definitions for LSI implementation
 *
 *  Greg Smart
 *
 *  Version 0.1 11/11/15
 *
 */

/*--------------------------------------------------------------------
 *
 *  LSI register include file
 *  This file contains all the macros and defines needed by the driver
 *  and user programs using it.
 *
 *--------------------------------------------------------------------*/

#ifndef _LSI_SYSTEM_H
#define _LSI_SYSTEM_H

/* general register memory location */

#define LSI_BASE                 0x43C00000
#define LSI_LMK_BASE             0x43C10000
#define LSI_ADC_BASE             0x43C20000

#define R_LSI_REG_BASE           0x0000
#define R_LSI_SPI_BASE           0x0800

#define R_LSI_STATUS             0x0000
#define R_SPI_READ_ADDR          0x0800

#define R_MODE_CONFIG_ADDR       0x0000
#define R_INTERRUPT_ADDR         0x0004
#define R_HISTOGRAM_RATE_ADDR    0x0008
#define R_HISTOGRAM_NUM_ADDR     0x000C

#define R_CAPTURE_COUNT_ADDR     0x0010

#define R_CAPT_DMA_ADDR          0x0018
#define R_CAPT_SIZE_ADDR         0x001C

#define R_HIST_DMA_ADDR          0x0020
#define R_HIST_SIZE_ADDR         0x0024

#define R_TEST_DMA_ADDR          0x0028
#define R_TEST_SIZE_ADDR         0x002C

#define R_ALPHA_ADDR             0x0030
#define R_ALPHA_SQ_ADDR          0x0034
#define R_DDT_ADDR               0x0038

#define R_OFFSET_BASE_ADDR       0x0100   // base address for 40 32-bit words
#define R_SCALE_BASE_ADDR        0x0200   // base address for 40 32-bit words

#define R_FPGA_VERSION_ADDR      0x0800

// LMK registers
#define R_LMK_DATA_ADDR          0x10000
#define R_LMK_CNTL_ADDR          0x10004
#define R_LMK_STATUS_ADDR        0x10008
#define R_SPI_0_ADDR             0x10020
#define R_SPI_1_ADDR             0x10024
#define R_SPI_2_ADDR             0x10028
#define R_SPI_3_ADDR             0x1002C
#define R_SPI_4_ADDR             0x10030
#define R_ADC_STATUS_ADDR        0x1003C

// ADC Registers
#define R_ADC_CONTROL_ADDR       0x20000
#define R_ADC_STATUS_0_ADDR      0x20004
#define R_ADC_STATUS_1_ADDR      0x20008
#define R_ADC_STATUS_2_ADDR      0x2000C
#define R_ADC_STATUS_3_ADDR      0x20010
#define R_ADC_STATUS_4_ADDR      0x20014

// These registers are only mapped for debug build
#define R_TAPS_LOAD_ADDR         0x2002C

#define R_ADC0_L_TAPS_ADDR       0x20030
#define R_ADC0_H_TAPS_ADDR       0x20034
#define R_ADC1_L_TAPS_ADDR       0x20038
#define R_ADC1_H_TAPS_ADDR       0x2003C

#define R_ADC2_L_TAPS_ADDR       0x20040
#define R_ADC2_H_TAPS_ADDR       0x20044
#define R_ADC3_L_TAPS_ADDR       0x20048
#define R_ADC3_H_TAPS_ADDR       0x2004C

#define R_ADC4_L_TAPS_ADDR       0x20050
#define R_ADC4_H_TAPS_ADDR       0x20054
#define R_ADC5_L_TAPS_ADDR       0x20058
#define R_ADC5_H_TAPS_ADDR       0x2005C

#define R_ADC6_L_TAPS_ADDR       0x20060
#define R_ADC6_H_TAPS_ADDR       0x20064
#define R_ADC7_L_TAPS_ADDR       0x20068
#define R_ADC7_H_TAPS_ADDR       0x2006C

#define R_ADC8_L_TAPS_ADDR       0x20070
#define R_ADC8_H_TAPS_ADDR       0x20074
#define R_ADC9_L_TAPS_ADDR       0x20078
#define R_ADC9_H_TAPS_ADDR       0x2007C

/*
** configuration constants
*/
#define GENERATE_PPS             0x01
#define DEBUG_START_DMA          0x02
#define PPS_DEBUG_MODE           0x40
#define DMA_DEBUG_MODE           0x80

#define MODE_NORMAL              0x00
#define MODE_DMA_DEBUG           (DMA_DEBUG_MODE)
#define MODE_TRIGGER_DMA         (DMA_DEBUG_MODE|DEBUG_START_DMA)
#define MODE_PPS_DEBUG           (PPS_DEBUG_MODE)
#define MODE_TRIGGER_PPS         (PPS_DEBUG_MODE|GENERATE_PPS)

/*
 * interrupt constants
 */
#define K_ENABLE_INTERRUPT       0x01
#define K_DISABLE_INTERRUPT      0x02
#define K_CLEAR_INTERRUPT        0x03

/*
** SPI constants
*/

#define SPI_MAX_WAIT_COUNT 1000000
#define MAX_WAIT_COUNT     10000


/*
 *  struct LSI_axi_lite_base.
 *  This structure provides general access for the 3 blocks within the system
 */

struct LSI_axi_lite_base {
   u32                           registers[512];
   u32                           fifo[512];
   u32                           bram[1024];
} ;

/*
 *  struct LSI_registers.
 *  This structure points to the first block where the registers are located
 */

struct LSI_registers {
   u32                           pulse_rate_gen;      // 0x0
   u32                           pulse_width_gen;     // 0x4
   u32                           sweep_test;          // 0x8
   u32                           run_test;            // 0xC
   u32                           data_read_addr;      // 0x10
   u32                           data_read_stride;    // 0x14
} ;

#define MAX_DEVICES     4

/**
 * struct LSI_drvdata - Device Configuration driver structure
 *
 * @dev: Pointer to the device structure
 * @cdev: Instance of the cdev structure
 * @devt: Pointer to the dev_t structure
 * @class: Pointer to device class
 * @fclk_class: Pointer to fclk device class
 * @dma_done: The dma_done status bit for the DMA command completion
 * @error_status: The error status captured during the DMA transfer
 * @irq: Interrupt number
 * @clk: Peripheral clock for devcfg
 * @fclk: Array holding references to the FPGA clocks
 * @fclk_exported: Flag inidcating whether an FPGA clock is exported
 * @is_open: The status bit to LSIicate whether the device is opened
 * @sem: Instance for the mutex
 * @lock: Instance of spinlock
 * @base_address: The virtual device base address of the device registers
 * @is_partial_bitstream: Status bit to indicate partial/full bitstream
 */
struct LSI_drvdata {
   struct device *dev;
   struct cdev cdev;
   dev_t devt;
   struct class *class;
   int irq;
   atomic_t irq_count;
   int dma_block_count;
   struct clk *clk;
   volatile bool dma_done;
   volatile int error_status;
   bool is_open;
   struct mutex mutex;
   spinlock_t lock;
   void __iomem *base;
   void __iomem *gpio_base;
  uint32_t config_state;
   uint32_t int_status;
   atomic_t semaphore;
   char *dma_addr;
   dma_addr_t dma_handle;
   struct list_head dev_list;
   wait_queue_head_t irq_wait_queue;
};

static inline void LSI_write_reg(struct LSI_drvdata *LSI, unsigned int reg, uint32_t val)
{
	writel(val, LSI->base + reg);
}

static inline uint32_t LSI_read_reg(struct LSI_drvdata *LSI, unsigned int reg)
{
	return(readl(LSI->base + reg));
}

static inline void LSI_write_gpio(struct LSI_drvdata *LSI, unsigned int reg, uint32_t val)
{
	writel(val, LSI->gpio_base + reg);
}

static inline uint32_t LSI_read_gpio(struct LSI_drvdata *LSI, unsigned int reg)
{
	return(readl(LSI->gpio_base + reg));
}


/*
static inline void LMK_write_reg(struct LSI_drvdata *LSI, unsigned int reg, uint32_t val)
{
	writel(val, LSI->lmk_base + reg);
}

static inline uint32_t LMK_read_reg(struct LSI_drvdata *LSI, unsigned int reg)
{
	return(readl(LSI->lmk_base + reg));
}

static inline void ADC_write_reg(struct LSI_drvdata *LSI, unsigned int reg, uint32_t val)
{
	writel(val, LSI->adc_base + reg);
}

static inline uint32_t ADC_read_reg(struct LSI_drvdata *LSI, unsigned int reg)
{
	return(readl(LSI->adc_base + reg));
}
*/


//
// LSI_Status()
//
// read the FOS status registers
//
// status if system is running test, sweep or is idle
//
static inline u32 LSI_Status(struct LSI_drvdata *LSI)
{
   u32 status;

   status = LSI_read_reg(LSI, R_LSI_STATUS)|LSI->int_status;
//   status &= 0x000001ff;

   return status;
}

//
// LSI_ADC_Status()
//
// read the FOS status registers
//
// status if system is running test, sweep or is idle
//
static inline u32 LMK_SPI_Busy(struct LSI_drvdata *LSI)
{
   u32 status;

   status = LSI_read_reg(LSI, R_ADC_STATUS_ADDR)>>16;

   return status;
}

//
//
//
//
//  Prototypes
//
//
//
//
//
void lmk03000_init (struct LSI_drvdata *LSI, u32 freq);

//
// LSI_Open()
//
// Open LSI system and mmap registers for user access.
//
// Returns pointer to virtual address.
//
u32 LSI_Open(u32 init_fpga);

//
// LSI_Close()
//
// Close LSI system and unmap memory.  Function is called with
// virtual address that was returned when opened.
//
u32 LSI_Close(int fd);

//
// LSI_Set_User_Mode()
//
// Setup the system configuration
//
int LSI_Set_User_Mode(struct LSI_drvdata *LSI, struct LSI_cmd_struct *cmd);

//
// LSI_Run_Scan()
//
// Initiate a scan
//
// Function will return immediately after initiating test
// before test completion
//
int LSI_Run_Scan(struct LSI_drvdata *LSI, void *user_ptr);

//
// LSI_SPI_Write()
//
// Write a command to SPI port
//
int LSI_SPI_Access(struct LSI_drvdata *LSI, void *user_ptr);

//
// LSI_Maxmin_Read()
//
int LSI_Maxmin_Read(struct LSI_drvdata *LSI, void *user_ptr);

//
// LSI_Write_Adc_Taps()
//
// Set the user operation mode
//
int LSI_Write_Adc_Taps(struct LSI_drvdata *LSI, void *user_ptr);

//
// LSI_Read_Adc_Taps()
//
// Set the user operation mode
//
int LSI_Read_Adc_Taps(struct LSI_drvdata *LSI, void *user_ptr);


#endif /* _LSI_SYSTEM_H */
