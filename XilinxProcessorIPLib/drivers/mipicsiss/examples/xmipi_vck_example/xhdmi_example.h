/******************************************************************************
* Copyright (C) 2014 - 2022 Xilinx, Inc.  All rights reserved.
* Copyright 2022-2024 Advanced Micro Devices, Inc. All Rights Reserved.
* SPDX-License-Identifier: MIT
******************************************************************************/

/*****************************************************************************/
/**
*
* @file xhdmi_example.h
*
* This file contains set of definition for the main application
*
* <pre>
* MODIFICATION HISTORY:
*
* Ver   Who    Date     Changes
* ----- ------ -------- --------------------------------------------------
* 1.00         12/02/18 Initial release.
* 3.03  YB     08/14/18 Adding macro 'ENABLE_HDCP_REPEATER' to allow application
*                       to select/deselect the Repeater specific code.
*       EB     09/21/18 Added new API ToggleHdmiRxHpd
* </pre>
*
******************************************************************************/
#ifndef _XHDMI_EXAMPLE_H_
/**  prevent circular inclusions by using protection macros */
#define _XHDMI_EXAMPLE_H_

#ifdef __cplusplus
extern "C" {
#endif

/***************************** Include Files *********************************/
#include <stdio.h>
#include <stdlib.h>
#include "platform.h"
#include "xparameters.h"
#if defined (ARMR5) || ((__aarch64__) && (!defined XPS_BOARD_ZCU104))
#include "xiicps.h"
#endif
#include "xiic.h"

#include "xil_io.h"
#if defined (XPAR_XUARTLITE_NUM_INSTANCES) && (!defined (versal))
#include "xuartlite_l.h"
#elif defined versal
#include "xuartpsv.h"
#else
#include "xuartps.h"
#endif
#include "xil_types.h"
#include "xil_exception.h"
#include "string.h"
#if ((defined XPS_BOARD_ZCU104) || (defined versal))
#include "idt_8t49n24x.h"
#include "si570drv.h"
#else
#include "si5324drv.h"
#endif
#include "xvidc.h"
#include "xv_hdmic.h"
#include "xv_hdmic_vsif.h"
#include "sleep.h"
#include "xhdmi_edid.h"
#include "xhdmi_menu.h"
#ifdef XPAR_XV_HDMITXSS_NUM_INSTANCES
#include "xv_hdmitxss.h"
#endif
#include "xhdmiphy1.h"
#ifdef XPAR_XV_HDMITXSS_NUM_INSTANCES
#endif
#ifdef XPAR_XGPIO_NUM_INSTANCES
#include "xgpio.h"
#endif
#if defined (ARMR5) || (__aarch64__) || (__arm__)
#include "xscugic.h"
#else
#include "xintc.h"
#endif

/* AUXFIFOSIZE: Must be set to 3 or higher*/
#define AUXFIFOSIZE 10

#if defined (XPAR_XUARTLITE_NUM_INSTANCES)
#define UART_BASEADDR XPAR_XUARTPSV_0_BASEADDR
#else
#define UART_BASEADDR XPAR_XUARTPS_0_BASEADDR
#endif

/************************** Constant Definitions *****************************/
#define I2C_MUX_ADDR    0x74  /**< I2C Mux Address */
#if ((defined XPS_BOARD_ZCU104) || (defined versal))
#define I2C_CLK_ADDR    0x6C  /**< I2C Clk Address IDT_8T49N241*/
#else
#define I2C_CLK_ADDR    0x68  /**< I2C Clk Address */
#endif

#define ANSI_COLOR_RED     "\x1b[31m"
#define ANSI_COLOR_YELLOW  "\x1b[33m"
#define ANSI_COLOR_RESET   "\x1b[0m"


/************************** Constant Definitions *****************************/

/******************************** OPTIONS ************************************/
/* Enabling this will disable Pass-through mode and TX and RX will operate
 * separately
 */
#define LOOPBACK_MODE_EN 1

/* Enabling this will enable a debug UART menu */
#define HDMI_DEBUG_TOOLS 0

/* Enabling this will register a custom resolution to the video timing table
 */
#define CUSTOM_RESOLUTION_ENABLE 1

/* Enabling this will enable HDCP Debug menu */
#define HDCP_DEBUG_MENU_EN 0

/* Enabling this will enable Video Masking menu */
#define VIDEO_MASKING_MENU_EN 0

/************************** Variable Definitions *****************************/
/* VHdmiphy structure */
extern XHdmiphy1     Hdmiphy1;

extern int I2cClk(u32 InFreq, u32 OutFreq);

#ifdef XPAR_XV_HDMITXSS_NUM_INSTANCES
/* HDMI TX SS structure */
extern XV_HdmiTxSs HdmiTxSs;

extern u8 TxCableConnect;
#endif


/* TX busy flag. This flag is set while the TX is initialized*/
extern u8 TxBusy;
extern u8 IsPassThrough;

#if defined (ARMR5) || ((__aarch64__) && (!defined XPS_BOARD_ZCU104))
extern XIicPs Ps_Iic0, Ps_Iic1;
#define PS_IIC_CLK 100000
#endif

/************************** Function Prototypes ******************************/
#ifdef SDT
void enable_hdmi_interrupt();
void disable_hdmi_interrupt();
#endif
#ifdef __cplusplus
}
#endif

#endif /* _XHDMI_EXAMPLE_H_ */
