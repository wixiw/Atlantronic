/******************** (C) COPYRIGHT 2011 STMicroelectronics ********************
* File Name          : otgd_fs_cal.h
* Author             : STMicroelectronics
* Version            : V3.3.0
* Date               : 21-March-2011
* Description        : Header of OTG FS Device Core Access Layer interface.
********************************************************************************
* THE PRESENT SOFTWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH SOFTWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/
#ifndef __OTG_CORE_H__
#define __OTG_CORE_H__

#ifdef STM32F10X_CL

#include "kernel/cpu/cpu.h"
#include "kernel/driver/usb/usb_type.h"
#include "kernel/rcc.h"

#if defined ( __CC_ARM   )
  #define __packed        __packed                     /*!< packing keyword for ARM Compiler */
#elif defined ( __ICCARM__ )
  #define __packed        __packed                     /*!< packing keyword for IAR Compiler */
#elif defined   (  __GNUC__  )
  #define __packed        __attribute__ ((__packed__)) /*!< packing keyword for GNU Compiler */
#elif defined   (  __TASKING__  )                      /*!< packing keyword for TASKING Compiler */
  #define __packed   
#endif /* __CC_ARM */

/*******************************************************************************
                                define and types
*******************************************************************************/

#define DEVICE_MODE_ENABLED

#ifndef NULL
#define NULL ((void *)0)
#endif


#define DEV_EP_TX_DIS       0x0000
#define DEV_EP_TX_STALL     0x0010
#define DEV_EP_TX_NAK       0x0020
#define DEV_EP_TX_VALID     0x0030
 
#define DEV_EP_RX_DIS       0x0000
#define DEV_EP_RX_STALL     0x1000
#define DEV_EP_RX_NAK       0x2000
#define DEV_EP_RX_VALID     0x3000

#define USB_OTG_TIMEOUT     200000
/*****************          GLOBAL DEFINES          ***************************/

#define GAHBCFG_TXFEMPTYLVL_EMPTY              1
#define GAHBCFG_TXFEMPTYLVL_HALFEMPTY          0

#define GAHBCFG_GLBINT_ENABLE                  1
#define GAHBCFG_INT_DMA_BURST_SINGLE           0
#define GAHBCFG_INT_DMA_BURST_INCR             1
#define GAHBCFG_INT_DMA_BURST_INCR4            3
#define GAHBCFG_INT_DMA_BURST_INCR8            5
#define GAHBCFG_INT_DMA_BURST_INCR16           7
#define GAHBCFG_DMAENABLE                      1
#define GAHBCFG_TXFEMPTYLVL_EMPTY              1
#define GAHBCFG_TXFEMPTYLVL_HALFEMPTY          0

#define GRXSTS_PKTSTS_IN                       2
#define GRXSTS_PKTSTS_IN_XFER_COMP             3
#define GRXSTS_PKTSTS_DATA_TOGGLE_ERR          5
#define GRXSTS_PKTSTS_CH_HALTED                7

/*****************           DEVICE DEFINES         ***************************/

#define DSTS_ENUMSPD_HS_PHY_30MHZ_OR_60MHZ     0
#define DSTS_ENUMSPD_FS_PHY_30MHZ_OR_60MHZ     1
#define DSTS_ENUMSPD_LS_PHY_6MHZ               2
#define DSTS_ENUMSPD_FS_PHY_48MHZ              3

#define DCFG_FRAME_INTERVAL_80                 0
#define DCFG_FRAME_INTERVAL_85                 1
#define DCFG_FRAME_INTERVAL_90                 2
#define DCFG_FRAME_INTERVAL_95                 3

#define DEP0CTL_MPS_64                         0
#define DEP0CTL_MPS_32                         1
#define DEP0CTL_MPS_16                         2
#define DEP0CTL_MPS_8                          3

#define EP_SPEED_LOW                           0
#define EP_SPEED_FULL                          1
#define EP_SPEED_HIGH                          2

#define EP_TYPE_CTRL                           0
#define EP_TYPE_ISOC                           1
#define EP_TYPE_BULK                           2
#define EP_TYPE_INTR                           3

#define STS_GOUT_NAK                           1
#define STS_DATA_UPDT                          2
#define STS_XFER_COMP                          3
#define STS_SETUP_COMP                         4
#define STS_SETUP_UPDT                         6




typedef enum {

  USB_OTG_OK,
  USB_OTG_FAIL

}
USB_OTG_Status;

typedef struct USB_OTG_ep
{
  uint8_t        num;
  uint8_t        is_in;
  uint32_t       tx_fifo_num;
  uint32_t       type;
  uint8_t        even_odd_frame;
  uint32_t       maxpacket;
  uint8_t        *xfer_buff;
  uint32_t       xfer_len;
  uint32_t       xfer_count;
}
USB_OTG_EP , *PUSB_OTG_EP;

/********************************************************************************
                                      MACRO'S
********************************************************************************/

#define CLEAR_IN_EP_INTR(epnum,intr) \
  diepint.d32=0; \
  diepint.b.intr = 1; \
  USB_OTG_WRITE_REG32(&USB_OTG_FS_regs.DINEPS[epnum]->DIEPINTx,diepint.d32);

#define CLEAR_OUT_EP_INTR(epnum,intr) \
  doepint.d32=0; \
  doepint.b.intr = 1; \
  USB_OTG_WRITE_REG32(&USB_OTG_FS_regs.DOUTEPS[epnum]->DOEPINTx,doepint.d32);


#define USB_OTG_READ_REG32(reg)  (*(__IO uint32_t *)reg)

#define USB_OTG_WRITE_REG32(reg,value) (*(__IO uint32_t *)reg = value)

#define USB_OTG_MODIFY_REG32(reg,clear_mask,set_mask) \
  USB_OTG_WRITE_REG32(reg, (((USB_OTG_READ_REG32(reg)) & ~clear_mask) | set_mask ) )


#define uDELAY(usec)  wait_active(us_to_tick(usec))
#define mDELAY(msec)  wait_active(ms_to_tick(msec))

#define _OTGD_FS_GATE_PHYCLK     *(__IO uint32_t*)(0x50000E00) = 0x03
#define _OTGD_FS_UNGATE_PHYCLK   *(__IO uint32_t*)(0x50000E00) = 0x00

/********************************************************************************
                     EXPORTED FUNCTIONS FROM THE OTGD_FS_CAL LAYER
********************************************************************************/
USB_OTG_Status  OTGD_FS_CoreInit(void);
USB_OTG_Status  OTGD_FS_SetAddress(uint32_t BaseAddress);
USB_OTG_Status  OTGD_FS_EnableGlobalInt(void);
USB_OTG_Status  OTGD_FS_DisableGlobalInt(void);
USB_OTG_Status  OTGD_FS_FlushTxFifo (uint32_t num);
USB_OTG_Status  OTGD_FS_FlushRxFifo (void);
USB_OTG_Status  OTGD_FS_CoreInitDev (void);
USB_OTG_Status  OTGD_FS_EnableDevInt(void);
USB_OTG_Status  OTGD_FS_EP0Activate(void);
USB_OTG_Status  OTGD_FS_EPActivate(USB_OTG_EP *ep);
USB_OTG_Status  OTGD_FS_EPDeactivate(USB_OTG_EP *ep);
USB_OTG_Status  OTGD_FS_EPStartXfer(USB_OTG_EP *ep);
USB_OTG_Status  OTGD_FS_EP0StartXfer(USB_OTG_EP *ep);
USB_OTG_Status  OTGD_FS_EPSetStall(USB_OTG_EP *ep);
USB_OTG_Status  OTGD_FS_EPClearStall(USB_OTG_EP *ep);
uint32_t        OTGD_FS_ReadDevAllOutEp_itr(void);
uint32_t        OTGD_FS_ReadDevOutEP_itr(USB_OTG_EP *ep);
uint32_t        OTGD_FS_ReadDevAllInEPItr(void);
uint32_t        OTGD_FS_GetEPStatus(USB_OTG_EP *ep);
uint32_t        USBD_FS_IsDeviceMode(void);
uint32_t        OTGD_FS_ReadCoreItr(void);
USB_OTG_Status  OTGD_FS_WritePacket(uint8_t *src, 
                                    uint8_t ep_num, 
                                    uint16_t bytes);
void*           OTGD_FS_ReadPacket(uint8_t *dest, 
                                   uint16_t bytes);

void            OTGD_FS_SetEPStatus(USB_OTG_EP *ep, uint32_t Status);
void            OTGD_FS_SetRemoteWakeup(void);
void            OTGD_FS_ResetRemoteWakeup(void);

#endif /* STM32F10X_CL */

#endif
/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/

