/******************************************************************************
* Copyright (c) 2024 Advanced Micro Devices, Inc. All Rights Reserved.
* SPDX-License-Identifier: MIT
******************************************************************************/


/*****************************************************************************/
/**
*
* @file xsecure_plat.c
* This file contains versal_aiepg2 specific code for xilsecure server.
*
* <pre>
* MODIFICATION HISTORY:
*
* Ver   Who     Date     Changes
* ----- ------  -------- ------------------------------------------------------
* 5.4   kal      07/24/24 Initial release
*
* </pre>
*
******************************************************************************/

/***************************** Include Files *********************************/
#include "xsecure_sss.h"
#include "xsecure_sha.h"
#include "xsecure_aes.h"
#include "xsecure_init.h"
#include "xplmi.h"

/************************** Constant Definitions *****************************/

#define XSECURE_AES_ADDRESS			  (0xF11E0000U) /**< AES BaseAddress */
#define XSECURE_SHA_ADDRESS			  (0xF1210000U) /**< SHA BaseAddress */
#define XSECURE_RSA_ECDSA_RSA_ADDRESS (0xF1200000U) /**< RSA ECDSA BaseAddress */

/************************** Variable Definitions *****************************/

/* XSecure_SssLookupTable[Input source][Resource] */
const u8 XSecure_SssLookupTable
		[XSECURE_SSS_MAX_SRCS][XSECURE_SSS_MAX_SRCS] = {
	/*+----+------+------+------+------+-----+------+--------+
	*|DMA0| DMA1  | PTPI | AES  | SHA3 | SBI | SHA2 | Invalid|
	*+----+-------+------+------+------+-----+------+--------+
	* 0x00 = INVALID value
	*/
	{0x0DU, 0x00U, 0x00U, 0x06U, 0x00U, 0x0BU, 0x03U, 0x00U}, /* DMA0 */
	{0x00U, 0x09U, 0x00U, 0x07U, 0x00U, 0x0EU, 0x04U, 0x00U}, /* DMA1 */
	{0x0DU, 0x0AU, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U}, /* PTPI */
	{0x0EU, 0x05U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U}, /* AES  */
	{0x0CU, 0x07U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U}, /* SHA3 */
	{0x05U, 0x0BU, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U}, /* SBI  */
	{0x0AU, 0x0FU, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U}, /* SHA2 */
	{0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U}, /* Invalid */
};

/*
* The configuration table for devices
*/
const XSecure_ShaConfig ShaConfigTable[XSECURE_SHA_NUM_OF_INSTANCES] =
{
	{
		XSECURE_SSS_SHA3,
		XSECURE_SHA3_BASE_ADDRESS,
		XSECURE_SHA3_DEVICE_ID,
	},
	{
		XSECURE_SSS_SHA2,
		XSECURE_SHA2_BASE_ADDRESS,
		XSECURE_SHA2_DEVICE_ID,
	}
};
/************************** Function Prototypes ******************************/

static void XSecure_UpdateEcdsaCryptoStatus(u32 Op);

/************************** Function Definitions *****************************/

/*****************************************************************************/
/**
 * @brief	This function masks the secure stream switch value
 *
 * @param	InputSrc	- Input source to be selected for the resource
 * @param	OutputSrc	- Output source to be selected for the resource
 * @param   Value       - Register Value of SSS cfg register
 *
 * @return
 *	-	Mask - Mask value of corresponding InputSrc and OutputSrc
 *
 * @note	InputSrc, OutputSrc are of type XSecure_SssSrc
 *
 *****************************************************************************/
 u32 XSecure_SssMask(XSecure_SssSrc InputSrc, XSecure_SssSrc OutputSrc,
							u32 Value)
{
	u32 Mask = 0U;
	u32 RegVal = Value;

	if ((InputSrc == XSECURE_SSS_DMA0) || (OutputSrc == XSECURE_SSS_DMA0)) {
		if ((RegVal & XSECURE_SSS_SBI_MASK) == XSECURE_SSS_SBI_DMA0_VAL) {
			Mask |= XSECURE_SSS_SBI_MASK;
		}
		if ((RegVal & XSECURE_SSS_SHA3_MASK) == XSECURE_SSS_SHA3_DMA0_VAL) {
			Mask |= XSECURE_SSS_SHA3_MASK;
		}
		if ((RegVal & XSECURE_SSS_AES_MASK) == XSECURE_SSS_AES_DMA0_VAL) {
			Mask |= XSECURE_SSS_AES_MASK;
		}
		if ((RegVal & XSECURE_SSS_SHA2_MASK) == XSECURE_SSS_SHA2_DMA0_VAL) {
			Mask |= XSECURE_SSS_SHA2_MASK;
		}
		if ((RegVal & XSECURE_SSS_DMA0_MASK) != 0U) {
			Mask |= XSECURE_SSS_DMA0_MASK;
		}
	}
	if ((InputSrc == XSECURE_SSS_DMA1) || (OutputSrc == XSECURE_SSS_DMA1)) {
		if ((RegVal & XSECURE_SSS_SBI_MASK) == XSECURE_SSS_SBI_DMA1_VAL) {
			Mask |= XSECURE_SSS_SBI_MASK;
		}
		if ((RegVal & XSECURE_SSS_SHA3_MASK) == XSECURE_SSS_SHA3_DMA1_VAL) {
			Mask |= XSECURE_SSS_SHA3_MASK;
		}
		if ((RegVal & XSECURE_SSS_SHA2_MASK) == XSECURE_SSS_SHA2_DMA1_VAL) {
			Mask |= XSECURE_SSS_SHA2_MASK;
		}
		if ((RegVal & XSECURE_SSS_AES_MASK) == XSECURE_SSS_AES_DMA1_VAL) {
			Mask |= XSECURE_SSS_AES_MASK;
		}
		if ((RegVal & XSECURE_SSS_DMA1_MASK) != 0U) {
			Mask |= XSECURE_SSS_DMA1_MASK;
		}
	}

	return Mask;
}

/*****************************************************************************/
/**
 * @brief	This function updates TRNG crypto indicator
 *
 *****************************************************************************/
void XSecure_UpdateTrngCryptoStatus(u32 Op)
{
#ifdef VERSALNET_PLM
	XPlmi_UpdateCryptoStatus(XPLMI_SECURE_TRNG_MASK, (XPLMI_SECURE_TRNG_MASK & ~Op));
#else
	(void)Op;
#endif
}

/*****************************************************************************/
/**
 * @brief	This function updates RSA crypto indicator
 *
 *****************************************************************************/
void XSecure_SetRsaCryptoStatus()
{
#ifdef VERSALNET_PLM
	XPlmi_UpdateCryptoStatus(XPLMI_SECURE_RSA_MASK, XPLMI_SECURE_RSA_MASK);
#endif
}

/*****************************************************************************/
/**
 * @brief	This function updates the crypto indicator bit of AES, SHA and ECC
 *
 * @param	BaseAddress	- Base address of the core
 * @param   Op          - To set or clear the bit
 *
 *****************************************************************************/
void XSecure_UpdateCryptoStatus(UINTPTR BaseAddress, u32 Op)
{
#ifdef VERSALNET_PLM
	if (BaseAddress == XSECURE_AES_ADDRESS) {
		XPlmi_UpdateCryptoStatus(XPLMI_SECURE_AES_MASK, (XPLMI_SECURE_AES_MASK & ~Op));
	}
	else if (BaseAddress == XSECURE_RSA_ECDSA_RSA_ADDRESS) {
		XSecure_UpdateEcdsaCryptoStatus(Op);
	}
	else if (BaseAddress == XSECURE_SHA_ADDRESS) {
		XPlmi_UpdateCryptoStatus(XPLMI_SECURE_SHA3_384_MASK, (XPLMI_SECURE_SHA3_384_MASK & ~Op));
	}
	else {
		/* Do Nothing */
	}
#else
	(void)BaseAddress;
	(void)Op;
#endif
}

/*****************************************************************************/
/**
 * @brief	This function updates ECC crypto indicator
 *
 *****************************************************************************/
static void XSecure_UpdateEcdsaCryptoStatus(u32 Op)
{
#ifdef VERSALNET_PLM
	u32 RsaInUseFlag = 0U;


	if (Op == XSECURE_SET_BIT) {
		RsaInUseFlag = XPlmi_GetCryptoStatus(XPLMI_SECURE_RSA_MASK);
		if (RsaInUseFlag == 0U) {
			XPlmi_UpdateCryptoStatus(XPLMI_SECURE_ECDSA_MASK, XPLMI_SECURE_ECDSA_MASK);
		}
	}
	else {
		/* Clear both RSA and ECDSA bits */
		XPlmi_UpdateCryptoStatus(XPLMI_SECURE_ECDSA_MASK, 0U);
		XPlmi_UpdateCryptoStatus(XPLMI_SECURE_RSA_MASK, 0U);
	}
#else
	(void)Op;
#endif
}

/*****************************************************************************/
/**
 * @brief	This function configures DMA Byte Swap based on the user input
 *
 *****************************************************************************/
void XSecure_ConfigureDmaByteSwap(u32 Op)
{
	XSecure_Aes *AesInstance = XSecure_GetAesInstance();

	AesInstance->DmaSwapEn = Op;
}

/*****************************************************************************/
/**
 *
 * @brief      This function validates the size
 *
 * @param      Size            Size of data in bytes.
 * @param      IsLastChunk     Last chunk indication
 *
 * @return
 *     -       XST_SUCCESS on successful valdation
 *     -       Error code on failure
 *
 ******************************************************************************/
int XSecure_AesValidateSize(u32 Size, u8 IsLastChunk)
{
	(void)Size;
	(void)IsLastChunk;

	return XST_SUCCESS;
}

/*****************************************************************************/
/**
 *
 * @brief       This function configures the PMC DMA channels and transfers data
 *
 * @param       PmcDmaPtr       Pointer to the XPmcDma instance.
 * @param       AesDmaCfg       DMA SRC and DEST channel configuration
 * @param       Size            Size of data in bytes.
 *
 * @return
 *	-	XST_SUCCESS on successful configuration
 *	-	Error code on failure
 *
 ******************************************************************************/
int XSecure_AesPlatPmcDmaCfgAndXfer(XPmcDma *PmcDmaPtr, XSecure_AesDmaCfg *AesDmaCfg, u32 Size, UINTPTR BaseAddress)
{
	int Status = XST_FAILURE;
	XSecure_Aes *InstancePtr = XSecure_GetAesInstance();
	(void)BaseAddress;

	if ((PmcDmaPtr == NULL) || (AesDmaCfg == NULL) || (Size == 0U)) {
		Status = (int)XSECURE_AES_INVALID_PARAM;
		goto END;
	}
	/* Enable PMC DMA Src and Dst channels for byte swapping.*/
	if ((AesDmaCfg->SrcChannelCfg == TRUE) &&
		(InstancePtr->DmaSwapEn == XSECURE_DISABLE_BYTE_SWAP)) {
		XSecure_AesPmcDmaCfgEndianness(PmcDmaPtr,
				XPMCDMA_SRC_CHANNEL, XSECURE_ENABLE_BYTE_SWAP);
	}

	if ((AesDmaCfg->DestChannelCfg == TRUE) &&
			(InstancePtr->DmaSwapEn == XSECURE_DISABLE_BYTE_SWAP) &&
			((u32)AesDmaCfg->DestDataAddr != XSECURE_AES_NO_CFG_DST_DMA)) {
		XSecure_AesPmcDmaCfgEndianness(PmcDmaPtr,
			XPMCDMA_DST_CHANNEL, XSECURE_ENABLE_BYTE_SWAP);
	}

	if ((AesDmaCfg->DestChannelCfg == TRUE) &&
		((u32)AesDmaCfg->DestDataAddr != XSECURE_AES_NO_CFG_DST_DMA)) {
		XPmcDma_64BitTransfer(PmcDmaPtr, XPMCDMA_DST_CHANNEL,
			(u32)AesDmaCfg->DestDataAddr, (u32)(AesDmaCfg->DestDataAddr >> XSECURE_ADDR_HIGH_SHIFT),
			Size / XSECURE_WORD_SIZE, AesDmaCfg->IsLastChunkDest);
	}

	if (AesDmaCfg->SrcChannelCfg == TRUE) {
		XPmcDma_64BitTransfer(PmcDmaPtr, XPMCDMA_SRC_CHANNEL,
			(u32)AesDmaCfg->SrcDataAddr, (u32)(AesDmaCfg->SrcDataAddr >> XSECURE_ADDR_HIGH_SHIFT),
			Size / XSECURE_WORD_SIZE, AesDmaCfg->IsLastChunkSrc);
	}

	if (AesDmaCfg->SrcChannelCfg == TRUE) {
		/* Wait for the SRC DMA completion. */
		Status = XPmcDma_WaitForDoneTimeout(PmcDmaPtr,
			XPMCDMA_SRC_CHANNEL);
		if (Status != XST_SUCCESS) {
			goto END;
		}

		/* Acknowledge the transfer has completed */
		XPmcDma_IntrClear(PmcDmaPtr, XPMCDMA_SRC_CHANNEL,
			XPMCDMA_IXR_DONE_MASK);
	}

	if ((AesDmaCfg->DestChannelCfg == TRUE) &&
		((u32)AesDmaCfg->DestDataAddr != XSECURE_AES_NO_CFG_DST_DMA)) {
		/* Wait for the DEST DMA completion. */
		Status = XPmcDma_WaitForDoneTimeout(PmcDmaPtr,
			XPMCDMA_DST_CHANNEL);
		if (Status != XST_SUCCESS) {
			goto END;
		}

		/* Acknowledge the transfer has completed */
		XPmcDma_IntrClear(PmcDmaPtr, XPMCDMA_DST_CHANNEL,
			XPMCDMA_IXR_DONE_MASK);
	}

END:
	return Status;
}

/******************************************************************************/
/**
 * @brief	This is a helper function to enable/disable byte swapping feature
 * 		of PMC DMA
 *
 * @param	InstancePtr  Pointer to the XPmcDma instance
 * @param	Channel 	 Channel Type
 *			- XPMCDMA_SRC_CHANNEL
 *			 -XPMCDMA_DST_CHANNEL
 * @param	EndianType
 *			- 1 : Enable Byte Swapping
 *			- 0 : Disable Byte Swapping
 *
 *
 ******************************************************************************/
void XSecure_AesPmcDmaCfgEndianness(XPmcDma *InstancePtr,
	XPmcDma_Channel Channel, u8 EndianType)
{
	XPmcDma_Configure ConfigValues = {0U};

	/* Assert validates the input arguments */
	XSecure_AssertVoid(InstancePtr != NULL);

	/* Updates the XPmcDma_Configure structure with PmcDma's channel values */
	XPmcDma_GetConfig(InstancePtr, Channel, &ConfigValues);
	ConfigValues.EndianType = EndianType;
	/* Updates the PmcDma's channel with XPmcDma_Configure structure values */
	XPmcDma_SetConfig(InstancePtr, Channel, &ConfigValues);
}

/*****************************************************************************/
/**
 * @brief	This function generates Random number of given size
 *
 * @param Output is pointer to the output buffer
 * @param Size is the number of random bytes to be read
 *
 * @return
 *	-	XST_SUCCESS - On Success
 *  -   XST_FAILURE - On Failure
 *
 *****************************************************************************/
int XSecure_GetRandomNum(u8 *Output, u32 Size)
{
	volatile int Status = XST_FAILURE;
	u8 *RandBufPtr = Output;
	u32 TotalSize = Size;
	u32 RandBufSize = XTRNGPSX_SEC_STRENGTH_IN_BYTES;
	u32 Index = 0U;
	u32 NoOfGenerates = (Size + XTRNGPSX_SEC_STRENGTH_IN_BYTES - 1U) >> 5U;
	XTrngpsx_Instance *TrngInstance = XSecure_GetTrngInstance();

	if ((TrngInstance->UserCfg.Mode != XTRNGPSX_HRNG_MODE) ||
		(TrngInstance->State == XTRNGPSX_UNINITIALIZED_STATE )) {
			if (TrngInstance->ErrorState != XTRNGPSX_HEALTHY) {
				Status = XTrngpsx_PreOperationalSelfTests(TrngInstance);
				if (Status != XST_SUCCESS) {
					goto END;
				}
			}
		Status = XSecure_TrngInitNCfgHrngMode();
		if (Status != XST_SUCCESS) {
			goto END;
		}
	}

	for (Index = 0U; Index < NoOfGenerates; Index++) {
		if (Index == (NoOfGenerates - 1U)) {
			RandBufSize = TotalSize;
		}

		XSECURE_TEMPORAL_CHECK(END, Status, XTrngpsx_Generate, TrngInstance,
					RandBufPtr, RandBufSize, FALSE);
		RandBufPtr += RandBufSize;
		TotalSize -= RandBufSize;
	}

END:
	return Status;
}

/*****************************************************************************/
/**
 * @brief	This function initializes the trng in HRNG mode if it is not initialized
 *          and it is applicable only for VersalNet
 *
 * @return
 *		- XST_SUCCESS On Successful initialization
 *      - XST_FAILURE On Failure
 *
 *****************************************************************************/
int XSecure_ECCRandInit(void)
{
	volatile int Status = XST_FAILURE;
	XTrngpsx_Instance *TrngInstance = XSecure_GetTrngInstance();

	if ((XPlmi_IsKatRan(XPLMI_SECURE_TRNG_KAT_MASK) != TRUE) ||
		(TrngInstance->ErrorState != XTRNGPSX_HEALTHY)) {
		Status = XTrngpsx_PreOperationalSelfTests(TrngInstance);
		if (Status != XST_SUCCESS) {
			XPlmi_ClearKatMask(XPLMI_SECURE_TRNG_KAT_MASK);
			goto END;
		}
		else {
			XPlmi_SetKatMask(XPLMI_SECURE_TRNG_KAT_MASK);
		}
	}
	if ((TrngInstance->UserCfg.Mode != XTRNGPSX_HRNG_MODE) ||
		(TrngInstance->State == XTRNGPSX_UNINITIALIZED_STATE )) {
		Status = XSecure_TrngInitNCfgHrngMode();
		if(Status != XST_SUCCESS)
		{
			goto END;
		}
	}
	Status = XST_SUCCESS;
END:

	return Status;
}

/*****************************************************************************/
/**
 * @brief	This function initialize and configures the TRNG into HRNG mode of operation.
 *
 * @return
 *			- XST_SUCCESS upon success.
 *			- Error code on failure.
 *
 *****************************************************************************/
int XSecure_TrngInitNCfgHrngMode(void)
{
	volatile int Status = XST_FAILURE;
	XTrngpsx_UserConfig UsrCfg;
	XTrngpsx_Instance *TrngInstance = XSecure_GetTrngInstance();

	if (TrngInstance->State != XTRNGPSX_UNINITIALIZED_STATE ) {
		Status = XTrngpsx_Uninstantiate(TrngInstance);
		if (Status != XST_SUCCESS) {
			goto END;
		}
		XSecure_UpdateTrngCryptoStatus(XSECURE_CLEAR_BIT);
	}
	/* Initiate TRNG */
	UsrCfg.Mode = XTRNGPSX_HRNG_MODE;
	UsrCfg.AdaptPropTestCutoff = XSECURE_TRNG_USER_CFG_ADAPT_TEST_CUTOFF;
	UsrCfg.RepCountTestCutoff = XSECURE_TRNG_USER_CFG_REP_TEST_CUTOFF;
	UsrCfg.DFLength = XSECURE_TRNG_USER_CFG_DF_LENGTH ;
	UsrCfg.SeedLife = XSECURE_TRNG_USER_CFG_SEED_LIFE ;
	Status = XTrngpsx_Instantiate(TrngInstance, NULL, 0U, NULL, &UsrCfg);
	if (Status != XST_SUCCESS) {
		(void)XTrngpsx_Uninstantiate(TrngInstance);
		XSecure_UpdateTrngCryptoStatus(XSECURE_CLEAR_BIT);
		goto END;
	}
	XSecure_UpdateTrngCryptoStatus(XSECURE_SET_BIT);

END:
	return Status;
}

/*****************************************************************************/
/**
 * @brief	This function provides the pointer to the common trng instance
 *
 * @return	Pointer to the XSecure_TrngInstance instance
 *
 *****************************************************************************/
XTrngpsx_Instance *XSecure_GetTrngInstance(void)
{
	static XTrngpsx_Instance TrngInstance = {0U};

	return &TrngInstance;
}