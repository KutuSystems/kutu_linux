/*
 *  IND.h -- Register definitions for IND implementation
 *
 *  Greg Smart
 *
 *  Version 0.1 10/03/14
 *
 */

#ifndef _MSP430_SBW_H
#define _MSP430_SBW_H



#define MSP430_SBW_DEBUG_READ        	(1)
#define MSP430_SBW_DEBUG_WRITE       	(2)
#define MSP430_SBW_DEBUG_DMA_READ    	(3)
#define MSP430_SBW_DEBUG_DMA_WRITE   	(4)

/*
** Status constants
*/
#define BIT_PORT0       (0x00000001)
#define BIT_RST         (0x00000002)
#define BIT_TEST        (0x00000004)
#define BIT_PORT3       (0x00000008)
#define BIT_PORT4       (0x00000010)
#define BIT_PORT5       (0x00000020)
#define BIT_PORT6       (0x00000040)
#define BIT_PORT7       (0x00000080)


enum MSP430_SBW_user_cmds
{
   MSP430_SBW_USER_SETDIR,
   MSP430_SBW_USER_WRITE_OUTPUT,
   MSP430_SBW_USER_READ_INPUT,
   MSP430_SBW_USER_READBACK_OUTPUT,
   MSP430_SBW_USER_REG_DEBUG
};

struct MSP430_SBW_debug_struct {
   __u32                           cmd;
   __u32                           reg;
   __u32                           data;
} ;


/*
 *  struct MSP430_SBW_cmd_struct.
 *  This structure points to the first block where the registers are located
 */

struct MSP430_SBW_cmd_struct {
   __u32                            config;
   __u32                            interrupt;
   __u32                            address;
   __u32                            capture_count;
   __u32                            delay_count;
   __u32                            peak_detect_start;
   __u32                            peak_detect_end;
   __s32                            adc_offset;
} ;


/*
 *  IOCTL definitions.
 */

#define MSP430_SBW_IOCTL_BASE	't'

#define MSP430_SBW_USER_SETDIR                     _IOWR(MSP430_SBW_IOCTL_BASE, 0x80, struct MSP430_SBW_cmd_struct)
#define MSP430_SBW_USER_WRITE_OUTPUT               _IOWR(MSP430_SBW_IOCTL_BASE, 0x81, struct MSP430_SBW_cmd_struct)
#define MSP430_SBW_USER_READ_INPUT                 _IOWR(MSP430_SBW_IOCTL_BASE, 0x82, struct MSP430_SBW_cmd_struct)
#define MSP430_SBW_USER_READBACK_OUTPUT            _IOWR(MSP430_SBW_IOCTL_BASE, 0x83, struct MSP430_SBW_cmd_struct)
#define MSP430_SBW_USER_REG_DEBUG                  _IOWR(MSP430_SBW_IOCTL_BASE, 0x84, struct MSP430_SBW_debug_struct)

#endif /* _MSP430_SBW_H */
