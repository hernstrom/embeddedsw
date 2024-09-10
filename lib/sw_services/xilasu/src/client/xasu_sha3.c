/**************************************************************************************************
* Copyright (c) 2024 Advanced Micro Devices, Inc. All Rights Reserved.
* SPDX-License-Identifier: MIT
**************************************************************************************************/

/*************************************************************************************************/
/**
 *
 * @file xasu_sha3.c
 * @addtogroup Overview
 * @{
 *
 * This file contains the implementation of the client interface functions for
 * SHA 3 driver.
 *
 * <pre>
 * MODIFICATION HISTORY:
 *
 * Ver   Who  Date     Changes
 * ----- ---- -------- ----------------------------------------------------------------------------
 * 1.0   vns  06/04/24 Initial release
 *       ma   06/14/24 Updated XAsufw_ShaOperationCmd structure to have 64-bit hash address
 *
 * </pre>
 *
 *************************************************************************************************/

/*************************************** Include Files *******************************************/
#include "xasu_sha3.h"
#include "xasu_def.h"
#include "xasu_client.h"
#include "xasu_status.h"

/************************************ Constant Definitions ***************************************/

/************************************** Type Definitions *****************************************/

/*************************** Macros (Inline Functions) Definitions *******************************/

/************************************ Function Prototypes ****************************************/

/************************************ Variable Definitions ***************************************/

/*************************************************************************************************/
/**
 * @brief	This function sends request to ASU for calculating the hash
 *              on single block of data
 *
 * @param	ClientParamPtr          Pointer to the XAsu_ClientParams structure which holds
 *              the client input arguments.
 * @param	ShaClientParamPtr       Pointer to XAsu_ShaOperationCmd structure which holds the
 *              parameters of sha input arguments.
 *
 * @return
 *	        - XST_SUCCESS - If the sha3 hash calculation is successful
 *	        - XST_FAILURE - If there is a failure
 *
 *************************************************************************************************/
s32 XAsu_Sha3Operation(XAsu_ClientParams *ClientParamPtr, XAsu_ShaOperationCmd *ShaClientParamPtr)
{
	s32 Status = XST_FAILURE;
	XAsu_ChannelQueueBuf *QueueBuf;
	XAsu_QueueInfo *QueueInfo;

	/* Validatations of inputs */
	if ((ShaClientParamPtr->ShaMode != XASU_SHA_MODE_SHA256) &&
	    (ShaClientParamPtr->ShaMode != XASU_SHA_MODE_SHA384) &&
	    (ShaClientParamPtr->ShaMode != XASU_SHA_MODE_SHA512) &&
	    (ShaClientParamPtr->ShaMode != XASU_SHA_MODE_SHAKE256)) {
		Status = XASU_INVALID_ARGUMENT;
		goto END;
	}
	if ((ClientParamPtr->Priority != XASU_PRIORITY_HIGH) &&
	    (ClientParamPtr->Priority != XASU_PRIORITY_LOW)) {
		Status = XASU_INVALID_ARGUMENT;
		goto END;
	}
	if (ShaClientParamPtr->HashBufSize == 0U) {
		Status = XASU_INVALID_ARGUMENT;
		goto END;
	}
	if ((ShaClientParamPtr->OperationFlags &
	     (XASU_SHA_START | XASU_SHA_UPDATE | XASU_SHA_FINISH)) == 0x0U) {
		Status = XASU_INVALID_ARGUMENT;
		goto END;
	}

	QueueInfo = XAsu_GetQueueInfo(ClientParamPtr->Priority);
	if (QueueInfo == NULL) {
		goto END;
	}
	/* Get Queue memory */
	QueueBuf = XAsu_GetChannelQueueBuf(QueueInfo);
	if (QueueBuf == NULL) {
		goto END;
	}

	QueueBuf->ReqBuf.Header = XAsu_CreateHeader(XASU_SHA_OPERATION_CMD_ID, 0U,
				  XASU_MODULE_SHA3_ID, 0U);

	Status = Xil_SecureMemCpy((XAsu_ShaOperationCmd *)QueueBuf->ReqBuf.Arg,
				  sizeof(QueueBuf->ReqBuf.Arg), ShaClientParamPtr,
				  sizeof(XAsu_ShaOperationCmd));
	if (Status != XST_SUCCESS) {
		goto END;
	}

	Status = XAsu_UpdateQueueBufferNSendIpi(QueueInfo);

END:
	return Status;
}
