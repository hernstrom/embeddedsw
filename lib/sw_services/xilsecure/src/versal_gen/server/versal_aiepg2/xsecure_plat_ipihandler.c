/******************************************************************************
* Copyright (c) 2024 Advanced Micro Devices, Inc. All Rights Reserved.
* SPDX-License-Identifier: MIT
******************************************************************************/

/*****************************************************************************/
/**
*
* @file xsecure_plat_ipihandler.c
*
* This file contains the xilsecure versalnet IPI handlers implementation.
*
* <pre>
* MODIFICATION HISTORY:
*
* Ver   Who  Date        Changes
* ----- ---- -------- -------------------------------------------------------
* 5.4  kpt   01/13/2023 Initial release
*
* </pre>
*
*
******************************************************************************/
/**
* @addtogroup xsecure_helper_server_apis Xilsecure Helper Server APIs
* @{
*/
/***************************** Include Files *********************************/
#include "xsecure_defs.h"
#include "xil_util.h"
#include "xplmi_plat.h"
#include "xsecure_plat_ipihandler.h"
#include "xsecure_rsa_q.h"
#include "xsecure_plat.h"
#include "xsecure_error.h"
#include "Rsa.h"

/************************** Constant Definitions *****************************/
#define XSECURE_RSA_KEY_ADDRESS		(0xF2008000U) /**< Address to copy RSA input parameters */
/************************** Function Prototypes *****************************/
#ifndef PLM_RSA_EXCLUDE
static int XSecure_RsaPrivateOperationIpi(u32 RsaParamAddrLow, u32 RsaParamAddrHigh,
	u32 DstAddrLow, u32 DstAddrHigh);
#endif

/*****************************************************************************/
/**
 * @brief   This function calls respective IPI handler based on the API_ID
 *
 * @param 	Cmd is pointer to the command structure
 *
 * @return
 *	-	XST_SUCCESS - If the handler execution is successful
 *	-	ErrorCode - If there is a failure
 *
 ******************************************************************************/
int XSecure_PlatIpiHandler(XPlmi_Cmd *Cmd)
{
	volatile int Status = XST_FAILURE;
	const u32 *Pload = NULL;
	u32 CryptoMask = 0U;

	if (Cmd == NULL || Cmd->Payload == NULL) {
		Status = XST_INVALID_PARAM;
		goto END;
	}

	Pload = Cmd->Payload;
	if ((Cmd->CmdId & XSECURE_API_ID_MASK) == XSECURE_API(XSECURE_API_RSA_PRIVATE_DECRYPT)) {
#ifndef PLM_RSA_EXCLUDE
		Status = XSecure_RsaPrivateOperationIpi(Pload[0U], Pload[1U], Pload[2U], Pload[3U]);
#endif
	}

END:
	return Status;
}

#ifndef PLM_RSA_EXCLUDE

/*****************************************************************************/
/**
 * @brief       This function handler calls XSecure_RsaInitialize and
 *              XSecure_RsaExp server API.
 *
 * @param   RsaParamAddrLow  - Lower 32 bit address of the XSecure_RsaInParam
 *                             structure
 * @param   RsaParamAddrHigh - Higher 32 bit address of the XSecure_RsaInParam
 *                             structure
 * @param   DstAddrLow 	     - Lower 32 bit address of the output data
 *                             where decrypted data to be stored
 * @param   DstAddrHigh	     - Higher 32 bit address of the output data
 *                             where decrypted data to be stored
 *
 * @return
 *	-	XST_SUCCESS - If the Rsa decryption is successful
 *	-	ErrorCode - If there is a failure
 *
 ******************************************************************************/
static int XSecure_RsaPrivateOperationIpi(u32 RsaParamAddrLow, u32 RsaParamAddrHigh,
	u32 DstAddrLow, u32 DstAddrHigh)
{
	volatile int Status = XST_FAILURE;
	u64 RsaParamAddr = ((u64)RsaParamAddrHigh << XSECURE_ADDR_HIGH_SHIFT) | (u64)RsaParamAddrLow;
	u64 DstAddr = ((u64)DstAddrHigh << XSECURE_ADDR_HIGH_SHIFT) | (u64)DstAddrLow;
	XSecure_RsaInParam RsaParams;
	XSecure_RsaKeyParam RsaKeyParam;
	XSecure_RsaOperationParam *RsaOperationParamPtr = (XSecure_RsaOperationParam*)(UINTPTR)XSECURE_RSA_KEY_ADDRESS;
	u8 *OutDataPtr = (u8 *)(UINTPTR)(XSECURE_RSA_KEY_ADDRESS + sizeof(XSecure_RsaOperationParam));
	u8 *PubExponentPtr = NULL;
	u8 *P = NULL;
	u8 *Q = NULL;
	u8 *Tot = NULL;
	u32 PubModulus[XSECURE_RSA_4096_SIZE_WORDS];

	Status = XPlmi_MemCpy64((UINTPTR)&RsaParams, RsaParamAddr, sizeof(XSecure_RsaInParam));
	if (Status != XST_SUCCESS) {
		goto END;
	}

	Status = Xil_SMemSet(RsaOperationParamPtr, sizeof(XSecure_RsaOperationParam), 0U, sizeof(XSecure_RsaOperationParam));
	if (Status != XST_SUCCESS) {
		goto END;
	}

	Status = XPlmi_MemCpy64((UINTPTR)&RsaKeyParam, RsaParams.KeyAddr, sizeof(XSecure_RsaKeyParam));
	if (Status != XST_SUCCESS) {
		goto END;
	}

	if ((RsaKeyParam.OpMode != XSECURE_RSA_CRT_MODE) &&
		(RsaKeyParam.OpMode != XSECURE_RSA_EXPOPT_MODE) &&
		(RsaKeyParam.OpMode != XSECURE_RSA_EXPQ_MODE)) {
		Status = (int)XST_INVALID_PARAM;
		goto END;
	}

	if ((RsaParams.Size != XSECURE_RSA_4096_KEY_SIZE) &&
		(RsaParams.Size != XSECURE_RSA_3072_KEY_SIZE) &&
		(RsaParams.Size != XSECURE_RSA_2048_KEY_SIZE)) {
		Status = (int)XST_INVALID_PARAM;
		goto END;
	}

	Status = XSecure_MemCpyAndChangeEndianness((UINTPTR)RsaOperationParamPtr->InData, RsaParams.DataAddr,
		RsaParams.Size);
	if (Status != XST_SUCCESS) {
		goto END;
	}

	Status = XSecure_MemCpyAndChangeEndianness((UINTPTR)RsaOperationParamPtr->Mod, RsaKeyParam.ModAddr, RsaParams.Size);
	if (Status != XST_SUCCESS) {
		goto END;
	}

	if (RsaKeyParam.OpMode != XSECURE_RSA_CRT_MODE) {
		Status = XSecure_MemCpyAndChangeEndianness((UINTPTR)RsaOperationParamPtr->Exp, RsaKeyParam.ExpAddr, RsaParams.Size);
		if (Status != XST_SUCCESS) {
			goto END;
		}

		if (RsaKeyParam.OpMode == XSECURE_RSA_EXPOPT_MODE) {
			Status = XSecure_MemCpyAndChangeEndianness((UINTPTR)RsaOperationParamPtr->RN, RsaKeyParam.RNAddr, RsaParams.Size);
			if (Status != XST_SUCCESS) {
				goto END;
			}

			Status = XSecure_MemCpyAndChangeEndianness((UINTPTR)RsaOperationParamPtr->RRN, RsaKeyParam.RRNAddr, RsaParams.Size);
			if (Status != XST_SUCCESS) {
				goto END;
			}
		}

		if (RsaKeyParam.IsTotAvail == TRUE) {
			Status = XSecure_MemCpyAndChangeEndianness((UINTPTR)RsaOperationParamPtr->Tot, RsaKeyParam.TotAddr, RsaParams.Size);
			if (Status != XST_SUCCESS) {
				goto END;
			}
			Tot = RsaOperationParamPtr->Tot;
		}
	}

	if ((RsaKeyParam.IsPrimeAvail == TRUE) || (RsaKeyParam.OpMode == XSECURE_RSA_CRT_MODE)) {
		if ((RsaKeyParam.PSize != RsaParams.Size / 2U) ||
			(RsaKeyParam.QSize != RsaParams.Size / 2U)) {
			Status = (int)XST_INVALID_PARAM;
			goto END;
		}

		if (RsaKeyParam.OpMode == XSECURE_RSA_CRT_MODE) {
			Status = XSecure_MemCpyAndChangeEndianness((UINTPTR)RsaOperationParamPtr->DP, RsaKeyParam.DPAddr, RsaKeyParam.PSize);
			if (Status != XST_SUCCESS) {
				goto END;
			}

			Status = XSecure_MemCpyAndChangeEndianness((UINTPTR)RsaOperationParamPtr->DQ, RsaKeyParam.DQAddr, RsaKeyParam.QSize);
			if (Status != XST_SUCCESS) {
				goto END;
			}

			Status = XSecure_MemCpyAndChangeEndianness((UINTPTR)RsaOperationParamPtr->QInv, RsaKeyParam.QInvAddr, RsaKeyParam.QSize);
			if (Status != XST_SUCCESS) {
				goto END;
			}
		}

		Status = XSecure_MemCpyAndChangeEndianness((UINTPTR)RsaOperationParamPtr->P, RsaKeyParam.PAddr, RsaKeyParam.PSize);
		if (Status != XST_SUCCESS) {
			goto END;
		}

		Status = XSecure_MemCpyAndChangeEndianness((UINTPTR)RsaOperationParamPtr->Q, RsaKeyParam.QAddr, RsaKeyParam.QSize);
		if (Status != XST_SUCCESS) {
			goto END;
		}

		P = RsaOperationParamPtr->P;
		Q = RsaOperationParamPtr->Q;
	}

	if (RsaKeyParam.IsPubExpAvail == TRUE) {
		Status = Xil_SMemSet(&PubModulus, XSECURE_RSA_4096_KEY_SIZE, 0U, XSECURE_RSA_4096_KEY_SIZE);
		if (Status != XST_SUCCESS) {
			goto END;
		}
		Status = Xil_SReverseData(&RsaKeyParam.PubExp, XSECURE_RSA_PUB_EXP_SIZE);
		if (Status != XST_SUCCESS) {
			goto END;
		}
		PubModulus[0U] = RsaKeyParam.PubExp;
		PubExponentPtr = (u8*)(UINTPTR)PubModulus;
	}

	if (RsaKeyParam.OpMode == XSECURE_RSA_EXPQ_MODE) {
		Status = XSecure_RsaExp((unsigned char *)(UINTPTR)RsaOperationParamPtr->InData,
			RsaOperationParamPtr->Exp, RsaOperationParamPtr->Mod, P, Q, PubExponentPtr,
			Tot, (int)(RsaParams.Size * 8U), OutDataPtr);
	}
	else if (RsaKeyParam.OpMode == XSECURE_RSA_CRT_MODE) {
		Status = XSecure_RsaExpCRT((unsigned char *)(UINTPTR)RsaOperationParamPtr->InData,
			RsaOperationParamPtr->P, RsaOperationParamPtr->Q, RsaOperationParamPtr->DP,
			RsaOperationParamPtr->DQ, RsaOperationParamPtr->QInv, PubExponentPtr,
			RsaOperationParamPtr->Mod, (int)(RsaParams.Size * 8U), OutDataPtr);
	}
	else {
		Status = XSecure_RsaExpopt((unsigned char *)(UINTPTR)RsaOperationParamPtr->InData,
			RsaOperationParamPtr->Exp, RsaOperationParamPtr->Mod, RsaOperationParamPtr->RN, RsaOperationParamPtr->RRN, P, Q, PubExponentPtr,
			Tot, (int)(RsaParams.Size * 8U), OutDataPtr);
	}
	if (Status != XST_SUCCESS) {
		Status = (int)XSECURE_RSA_GEN_SIGN_FAILED_ERROR;
		goto END;
	}

	Status = Xil_SReverseData(OutDataPtr, RsaParams.Size);
	if (Status != XST_SUCCESS) {
		goto END;
	}

	Status = XPlmi_MemCpy64(DstAddr, (UINTPTR)OutDataPtr, RsaParams.Size);

END:
	return Status;
}
#endif