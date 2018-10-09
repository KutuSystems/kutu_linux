/*
 *  MSP430_SBW_system.h -- Register definitions for MSP430_SBW implementation
 *
 *  Greg Smart
 *
 *  Version 0.1 11/11/15
 *
 */

/*--------------------------------------------------------------------
 *
 *  MSP430_SBW register include file
 *  This file contains all the macros and defines needed by the driver
 *  and user programs using it.
 *
 *--------------------------------------------------------------------*/

#ifndef _MSP430_SBW_SYSTEM_H
#define _MSP430_SBW_SYSTEM_H

/* general register memory location */
#define R_MSP430_SBW_REG_BASE			0x0000

// FPGA user read/write registers.
// address decoder uses bits 5..2 => address offsets of 0x00..0x3C
#define R_WRITE_ADDR		      0x0000
#define R_READ_ADDR			   0x0000
#define R_DIRECTION_ADDR	   0x0004
#define R_READBACK_WR_ADDR    0x0008

#define DEFAULT_OUTPUT_VAL    0x00000000
#define DEFAULT_DIRECTION_VAL 0xffffffff


/*
 * interrupt constants
 */
#define K_ENABLE_INTERRUPT       0x01
#define K_DISABLE_INTERRUPT      0x02
#define K_CLEAR_INTERRUPT        0x03


/*
 *  struct MSP430_SBW_axi_lite_base.
 *  This structure provides general access for the 3 blocks within the system
 */

struct MSP430_SBW_axi_lite_base {
   u32                           registers[512];
   u32                           fifo[512];
   u32                           bram[1024];
} ;

/*
 *  struct MSP430_SBW_registers.
 *  This structure points to the first block where the registers are located
 */

struct MSP430_SBW_registers {
   u32                           pulse_rate_gen;      // 0x0
   u32                           pulse_width_gen;     // 0x4
   u32                           sweep_test;          // 0x8
   u32                           run_test;            // 0xC
   u32                           data_read_addr;      // 0x10
   u32                           data_read_stride;    // 0x14
} ;

#define MAX_DEVICES     4

/**
 * struct MSP430_SBW_drvdata - Device Configuration driver structure
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
 * @is_open: The status bit to indicate whether the device is opened
 * @sem: Instance for the mutex
 * @lock: Instance of spinlock
 * @base_address: The virtual device base address of the device registers
 * @is_partial_bitstream: Status bit to indicate partial/full bitstream
 */
struct MSP430_SBW_drvdata {
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
   spinlock_t lock;
   void __iomem *base;
   uint32_t config_state;
   uint32_t int_status;
   atomic_t semaphore;
   char *dma_addr;
   dma_addr_t dma_handle;
   struct list_head dev_list;
   wait_queue_head_t irq_wait_queue;
   struct MSP430_SBW_cmd_struct command;
   uint32_t bank;
};

static inline void MSP430_SBW_write_reg(struct MSP430_SBW_drvdata *MSP430_SBW, size_t reg, uint32_t val)
{
    writel(val, MSP430_SBW->base + reg);
}

static inline uint32_t MSP430_SBW_read_reg(struct MSP430_SBW_drvdata *MSP430_SBW, size_t reg)
{
    return readl(MSP430_SBW->base + reg);
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

//
// MSP430_SBW_Open()
//
// Open MSP430_SBW system and mmap registers for user access.
//
// Returns pointer to virtual address.
//
u32 MSP430_SBW_Open(u32 init_fpga);

//
// MSP430_SBW_Close()
//
// Close MSP430_SBW system and unmap memory.  Function is called with
// virtual address that was returned when opened.
//
u32 MSP430_SBW_Close(int fd);

//
// MSP430_SBW_Set_User_Mode()
//
// Setup the system configuration
//
int MSP430_SBW_Set_User_Mode(struct MSP430_SBW_drvdata *MSP430_SBW, void *user_ptr);

#endif /* _MSP430_SBW_SYSTEM_H */
