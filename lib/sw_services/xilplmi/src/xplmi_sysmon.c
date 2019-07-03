/******************************************************************************
*
* Copyright (C) 2019 Xilinx, Inc.  All rights reserved.
*
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the "Software"), to deal
* in the Software without restriction, including without limitation the rights
* to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
* copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in
* all copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
* THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMANGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
* THE SOFTWARE.
*
*
*
******************************************************************************/

/*****************************************************************************/
/**
 *
 * @file xplmi_sysmon.c
 *
 * This is the file which contains SysMon manager code.
 *
 * <pre>
 * MODIFICATION HISTORY:
 *
 * Ver   Who  Date        Changes
 * ----- ---- -------- -------------------------------------------------------
 * 1.00  sn   07/01/2019 Initial release
 *
 * </pre>
 *
 * @note
 *
 ******************************************************************************/
/***************************** Include Files *********************************/
#include "xplmi_sysmon.h"
#include "xplmi_proc.h"
/************************** Constant Definitions *****************************/

/**************************** Type Definitions *******************************/

/***************** Macros (Inline Functions) Definitions *********************/

/************************** Function Prototypes ******************************/
extern XStatus XPm_OvertempHandler(void);

/************************** Variable Definitions *****************************/

/*****************************************************************************/
/* Instance of SysMon Driver */
static XSysMonPsv SysMonInst;
static XSysMonPsv *SysMonInstPtr = &SysMonInst;

/*****************************************************************************/
/**
 * @brief This function initializes the SysMon
 *
 * @param	void
 *
 * @return	Status	SysMon initialization status
 *
 *****************************************************************************/
int XPlmi_SysMonInit(void)
{
	int Status = XST_SUCCESS;
	XSysMonPsv_Config *ConfigPtr;

	ConfigPtr = XSysMonPsv_LookupConfig();
	if (ConfigPtr == NULL) {
		Status = XST_FAILURE;
		goto END;
	}

	XSysMonPsv_CfgInitialize(SysMonInstPtr, ConfigPtr);

	XPlmi_RegisterHandler(XPLMI_SYSMON_ROOT_0, XPlmi_SysMonDispatchHandler,
			      (void *)0);

	/*
	 * Enable Over-temperature handling.  We need to unlock PCSR to
	 * enable SysMon interrupt.  Lock it back in after write.
	 */
	XPlmi_PlmIntrEnable(XPLMI_SYSMON_ROOT_0);
	XPlmi_Out32(ConfigPtr->BaseAddress + XSYSMONPSV_PCSR_LOCK,
		  PCSR_UNLOCK_VAL);
	XSysMonPsv_IntrEnable(SysMonInstPtr, XSYSMONPSV_IER0_OT_MASK, 0);
	XPlmi_Out32(ConfigPtr->BaseAddress + XSYSMONPSV_PCSR_LOCK, 0);

END:
	XPlmi_Printf(DEBUG_DETAILED,
		    "%s: SysMon init status: 0x%x\n\r", __func__, Status);
	return Status;
}

/*****************************************************************************/
/**
 * @brief This is the handler for SysMon interrupts
 *
 * @param	void
 *
 * @return	Status	Status of received SysMon processing
 *
 *****************************************************************************/
int XPlmi_SysMonDispatchHandler(void *Data)
{
	int Status = XST_FAILURE;

	/* For MISRA C */
	(void )Data;

	Status = XPm_OvertempHandler();

	return Status;
}
