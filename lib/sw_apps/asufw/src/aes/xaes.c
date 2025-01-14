/**************************************************************************************************
* Copyright (c) 2024 Advanced Micro Devices, Inc. All Rights Reserved.
* SPDX-License-Identifier: MIT
**************************************************************************************************/

/*************************************************************************************************/
/**
 *
 * @file xaes.c
 *
 * This file contains low level implementation of AES hardware interface APIs
 *
 * <pre>
 * MODIFICATION HISTORY:
 *
 * Ver   Who  Date       Changes
 * ----- ---- -------- ----------------------------------------------------------------------------
 * 1.0   am   06/26/24 Initial release
 *       am   08/01/24 Replaced variables of type enums with u32 type.
 *       am   08/24/24 Added AES DPA CM KAT support
 *       yog  08/25/24 Integrated FIH library
 *       am   08/25/24 Initialized ASU DMA pointer before XAsufw_DmaXfr() function call.
 *
 * </pre>
 *
 *************************************************************************************************/
/**
* @addtogroup xaes_server_apis AES Server APIs
* @{
*/
/***************************** Include Files *****************************************************/
#include "xaes.h"
#include "xasufw_status.h"
#include "xasufw_util.h"
#include "xasufw_config.h"
#include "xasu_def.h"
#include "xasu_aes_common.h"
#include "xfih.h"

/************************** Constant Definitions *************************************************/

/************************** Macros Definitions ***************************************************/
#define XAES_PUF_KEY			(0xAU) /**< PUF key */
#define XAES_EXPANDED_KEYS		(0xBU) /**< Expanded keys in AES engine */
#define XAES_MAX_KEY_SOURCES		XAES_EXPANDED_KEYS /**< Maximum key source value */
#define XAES_TIMEOUT_MAX		(0x1FFFFU) /**<  AES maximum timeout value in micro seconds */
#define XAES_INVALID_CFG		(0xFFFFFFFFU) /**<  AES invalid configuration */

typedef enum {
	XAES_INITIALIZED = 0x1, /**< AES in initialized state */
	XAES_STARTED, /**< AES in start state */
	XAES_UPDATE_IN_PROGRESS, /**< AES is in progress state during multiple data chunk updates */
	XAES_UPDATE_COMPLETED, /**< AES is in completed state after the final data chunk update */
} XAes_State;

/**************************** Type Definitions ***************************************************/
/**
 * @brief This structure defines look up table for AES key.
 */
typedef struct {
	u32 RegOffset; /**< Register offset for key source */
	u32 KeySrcSelVal; /**< Selection value for key source */
	u32 UsrWrAllowed; /**< User write allowed or not for key source */
	u32 KeyDecSrcAllowed; /**< Key decryption source allowed */
	u32 KeyDecSrcSelVal; /**< Selection value for key decryption source*/
	u32 KeyClearVal; /**< Key source clear value*/
	u32 KeyZeroedStatusMask; /**< Key zeroed status mask */
} XAes_KeyLookup;

/**
 * @brief This structure contains configuration information for AES core.
 * Each core should have an associated configuration structure.
 */
struct _XAes_Config {
	u16 DeviceId; /**< Unique ID of the device */
	u32 AesBaseAddress; /**< Base address of ASU_AES module */
	u32 KeyBaseAddress; /**< Base address of ASU_KEY module */
};

/************************** Variable Definitions *************************************************/
/** This array of structure defines look up table for AES key. */
static const XAes_KeyLookup AesKeyLookupTbl [XAES_MAX_KEY_SOURCES] = {
	/** EFUSE_KEY_RED_0 */
	{
		XAES_INVALID_CFG,
		XAES_KEY_SEL_EFUSE_KEY_RED_0_VALUE,
		XASU_FALSE,
		XASU_TRUE,
		XAES_KEY_TO_BE_DEC_SEL_EFUSE_KEY_0_VALUE,
		XAES_KEY_CLEAR_EFUSE_KEY_RED_0_MASK,
		XAES_KEY_ZEROED_STATUS_EFUSE_RED_KEY_0_MASK
	},

	/** EFUSE_KEY_RED_1 */
	{
		XAES_INVALID_CFG,
		XAES_KEY_SEL_EFUSE_KEY_RED_1_VALUE,
		XASU_FALSE,
		XASU_TRUE,
		XAES_KEY_TO_BE_DEC_SEL_EFUSE_KEY_1_VALUE,
		XAES_KEY_CLEAR_EFUSE_KEY_RED_1_MASK,
		XAES_KEY_ZEROED_STATUS_EFUSE_RED_KEY_1_MASK
	},

	/** USER_KEY_0 */
	{
		XAES_USER_KEY_0_0_OFFSET,
		XAES_KEY_SEL_USER_KEY_0_VALUE,
		XASU_TRUE,
		XASU_FALSE,
		XAES_INVALID_CFG,
		XAES_KEY_CLEAR_USER_KEY_0_MASK,
		XAES_KEY_ZEROED_STATUS_USER_KEY_0_MASK
	},

	/** USER_KEY_1 */
	{
		XAES_USER_KEY_1_0_OFFSET,
		XAES_KEY_SEL_USER_KEY_1_VALUE,
		XASU_TRUE,
		XASU_FALSE,
		XAES_INVALID_CFG,
		XAES_KEY_CLEAR_USER_KEY_1_MASK,
		XAES_KEY_ZEROED_STATUS_USER_KEY_1_MASK
	},

	/** USER_KEY_2 */
	{
		XAES_USER_KEY_2_0_OFFSET,
		XAES_KEY_SEL_USER_KEY_2_VALUE,
		XASU_TRUE,
		XASU_FALSE,
		XAES_INVALID_CFG,
		XAES_KEY_CLEAR_USER_KEY_2_MASK,
		XAES_KEY_ZEROED_STATUS_USER_KEY_2_MASK
	},

	/** USER_KEY_3 */
	{
		XAES_USER_KEY_3_0_OFFSET,
		XAES_KEY_SEL_USER_KEY_3_VALUE,
		XASU_TRUE,
		XASU_FALSE,
		XAES_INVALID_CFG,
		XAES_KEY_CLEAR_USER_KEY_3_MASK,
		XAES_KEY_ZEROED_STATUS_USER_KEY_3_MASK
	},

	/** USER_KEY_4 */
	{
		XAES_USER_KEY_4_0_OFFSET,
		XAES_KEY_SEL_USER_KEY_4_VALUE,
		XASU_TRUE,
		XASU_FALSE,
		XAES_INVALID_CFG,
		XAES_KEY_CLEAR_USER_KEY_4_MASK,
		XAES_KEY_ZEROED_STATUS_USER_KEY_4_MASK
	},

	/** USER_KEY_5 */
	{
		XAES_USER_KEY_5_0_OFFSET,
		XAES_KEY_SEL_USER_KEY_5_VALUE,
		XASU_TRUE,
		XASU_FALSE,
		XAES_INVALID_CFG,
		XAES_KEY_CLEAR_USER_KEY_5_MASK,
		XAES_KEY_ZEROED_STATUS_USER_KEY_5_MASK
	},

	/** USER_KEY_6 */
	{
		XAES_USER_KEY_6_0_OFFSET,
		XAES_KEY_SEL_USER_KEY_6_VALUE,
		XASU_TRUE,
		XASU_FALSE,
		XAES_INVALID_CFG,
		XAES_KEY_CLEAR_USER_KEY_6_MASK,
		XAES_KEY_ZEROED_STATUS_USER_KEY_6_MASK
	},

	/** USER_KEY_7 */
	{
		XAES_USER_KEY_7_0_OFFSET,
		XAES_KEY_SEL_USER_KEY_7_VALUE,
		XASU_TRUE,
		XASU_FALSE,
		XAES_INVALID_CFG,
		XAES_KEY_CLEAR_USER_KEY_7_MASK,
		XAES_KEY_ZEROED_STATUS_USER_KEY_7_MASK
	},

	/** PUF_KEY */
	{
		XAES_INVALID_CFG,
		XAES_KEY_SEL_PUF_KEY_VALUE,
		XASU_FALSE,
		XASU_FALSE,
		XAES_INVALID_CFG,
		XAES_KEY_CLEAR_PUF_KEY_MASK,
		XAES_KEY_ZEROED_STATUS_PUF_KEY_MASK
	},
};

/** AES configuration table for devices */
static XAes_Config AesConfigTable[XASU_XAES_NUM_INSTANCES] = {
	{
		XASU_XAES_0_DEVICE_ID,
		XASU_XAES_0_BASEADDR,
		XASU_XKEY_0_BASEADDR
	}
};

/**
 * @brief AES driver instance data structure. A pointer to an instance data
 * structure is passed around by functions to refer to a specific driver
 * instance.
 */
struct _XAes {
	u32 AesBaseAddress; /**< AES Base address */
	u32 KeyBaseAddress; /**< Key Vault Base address */
	u16 DeviceId;		/**< DeviceId is the unique ID of the device */
	u16 AesCmConfig;	/**< AES counter Measure configuration */
	XAsufw_Dma *AsuDmaPtr;	/**< PMCDMA Instance Pointer */
	XAes_State AesState;	/**< Aes internal State machine */
	u8 OperationType;	/**< AES operation type (Encryption/Decryption) */
	u8 EngineMode;		/**< Aes Engine mode*/
	u16 Reserved;		/**< Reserved */
};

static XAes XAes_Instance[XASU_XAES_NUM_INSTANCES]; /**< ASUFW AES HW instances */

/************************** Inline Function Definitions ******************************************/

/************************** Function Prototypes **************************************************/
static XAes_Config *XAes_LookupConfig(u16 DeviceId);
static s32 XAes_IsKeyZeroized(const XAes *InstancePtr, u32 KeySrc);
static void XAes_ConfigCounterMeasures(const XAes *InstancePtr);
static void XAes_ConfigAesOperation(const XAes *InstancePtr);
static void XAes_LoadKey(const XAes *InstancePtr, u32 KeySrc, u32 KeySize);
static s32 XAes_ProcessAndLoadIv(XAes *InstancePtr, u64 IvAddr, u32 IvLen);
static s32 XAes_GHashCal(XAes *InstancePtr, u64 IvAddr, u32 IvGen, u32 IvLen);
static s32 XAes_ReadTag(const XAes *InstancePtr, u32 TagOutAddr, u32 TagLen);
static s32 XAes_ReadNVerifyTag(const XAes *InstancePtr, u32 TagInAddr, u32 TagLen);
static s32 XAes_ProcessTag(const XAes *InstancePtr, u64 TagAddr, u32 TagLen);
static void XAes_ConfigAad(const XAes *InstancePtr);
static void XAes_ClearConfigAad(const XAes *InstancePtr);
static s32 XAes_CfgDmaWithAesAndXfer(const XAes *InstancePtr, u64 InDataAddr, u64 OutDataAddr,
	u32 Size, u8 IsLastChunk);
static s32 XAes_WaitForDone(const XAes *InstancePtr);
static void XAes_SetReset(XAes *InstancePtr);

/*************************************************************************************************/
/**
 * @brief	This function returns an AES instance pointer of the provided device ID.
 *
 * @param	DeviceId	The device ID of AES core.
 *
 * @return
 * 		- It returns pointer to the XAes_Instance corresponding to the Device ID.
 * 		- It returns NULL if the device ID is invalid.
 *
 *************************************************************************************************/
XAes *XAes_GetInstance(u16 DeviceId)
{
	XAes *XAes_InstancePtr = NULL;

	if (DeviceId >= XASU_XAES_NUM_INSTANCES) {
		goto END;
	}

	XAes_InstancePtr = &XAes_Instance[DeviceId];
	XAes_InstancePtr->DeviceId = DeviceId;

END:
	return XAes_InstancePtr;
}

/*************************************************************************************************/
/**
 * @brief	This function initializes the AES instance.
 *
 * @param	InstancePtr 	Pointer to the XAes instance.
 *
 * @return
 *		- XASUFW_SUCCESS, if initialization is successful.
 *		- XASUFW_AES_INVALID_PARAM, if InstancePtr or CfgPtr is NULL.
 *
 *************************************************************************************************/
s32 XAes_CfgInitialize(XAes *InstancePtr)
{
	CREATE_VOLATILE(Status, XASUFW_FAILURE);
	const XAes_Config *CfgPtr;

	/** Validate input parameters. */
	if (InstancePtr == NULL) {
		Status = XASUFW_AES_INVALID_PARAM;
		goto END;
	}

	CfgPtr = XAes_LookupConfig(InstancePtr->DeviceId);
	if (CfgPtr == NULL) {
		Status = XASUFW_AES_INVALID_PARAM;
		goto END;
	}

	/** Initialize AES instance. */
	InstancePtr->AesBaseAddress = CfgPtr->AesBaseAddress;
	InstancePtr->KeyBaseAddress = CfgPtr->KeyBaseAddress;
	InstancePtr->AesCmConfig = XASUFW_AES_CM_CONFIG;
	InstancePtr->AesState = XAES_INITIALIZED;

	Status = XASUFW_SUCCESS;

END:
	return Status;
}

/*************************************************************************************************/
/**
 * @brief	This function writes the provided user key into specified AES USER key registers.
 *
 * @param	InstancePtr	Pointer to the XAes instance.
 * @param	DmaPtr		Pointer to the AsuDma instance.
 * @param	KeyObjectAddr	Address of Aes key object structure, which contains
 *				- KeyAddress is address of user key.
 *				- KeySize is size of the user key.
 *			  	  - XASUFW_AES_KEY_SIZE_128 for 128 bit key size
 *				  - XASUFW_AES_KEY_SIZE_256 for 256 bit key size
 *				- KeySrc which indicates which hardware user key to be used to
 * 				  store the user key.
 *			  	  - XASU_AES_USER_KEY_0
 *				  - XASU_AES_USER_KEY_1
 *				  - XASU_AES_USER_KEY_2
 *				  - XASU_AES_USER_KEY_3
 *				  - XASU_AES_USER_KEY_4
 *				  - XASU_AES_USER_KEY_5
 *				  - XASU_AES_USER_KEY_6
 *				  - XASU_AES_USER_KEY_7
 *
 * @return
 *		- XASUFW_SUCCESS, if key write is successful.
 *		- XASUFW_AES_INVALID_PARAM, if InstancePtr or DmaPtr is NULL or ASU DMA is not
 * 			ready.
 *		- XASUFW_AES_INVALID_KEY_OBJECT_ADDRESS, if KeyObjectAddr is invalid.
 *		- XASUFW_AES_INVALID_KEY_SRC, if key source is invalid.
 *		- XASUFW_AES_INVALID_KEY_SIZE, if key size is invalid.
 *
 *************************************************************************************************/
/* TODO: In future, this API can be moved/modified after Key vault implementation. */
s32 XAes_WriteKey(XAes *InstancePtr, XAsufw_Dma *DmaPtr, u64 KeyObjectAddr)
{
	CREATE_VOLATILE(Status, XASUFW_FAILURE);
	s32 ClearStatus = XASUFW_FAILURE;
	XFih_Var XFihKeyClear;
	volatile u32 Index = 0U;
	u32 Key[XASU_AES_KEY_SIZE_256BIT_IN_WORDS];
	u32 Offset;
	u32 KeySizeInWords = 0U;
	XAsu_AesKeyObject KeyObject;

	/** Validate the input arguments. */
	if (InstancePtr == NULL) {
		Status = XASUFW_AES_INVALID_PARAM;
		goto END;
	}

	if ((DmaPtr == NULL) || (DmaPtr->AsuDma.IsReady != XIL_COMPONENT_IS_READY)) {
		Status = XASUFW_AES_INVALID_PARAM;
		goto END;
	}

	if (KeyObjectAddr == 0U) {
		Status = XASUFW_AES_INVALID_KEY_OBJECT_ADDRESS;
		goto END;
	}

	/** Initialize the AES instance with ASU DMA pointer. */
	InstancePtr->AsuDmaPtr = DmaPtr;

	/**
	 * Copy KeyObject strucure from 64-bit address space to local structure using
	 * ASU DMA.
	 */
	Status = XAsufw_DmaXfr(InstancePtr->AsuDmaPtr, KeyObjectAddr,
		(u64)(UINTPTR)&KeyObject, sizeof(XAsu_AesKeyObject), 0U);
	if (Status != XASUFW_SUCCESS) {
		goto END_CLR;
	}

	ASSIGN_VOLATILE(Status, XASUFW_FAILURE);
	if ((KeyObject.KeySrc < XASU_AES_USER_KEY_0) ||
			(KeyObject.KeySrc > XASU_AES_USER_KEY_7)) {
		Status = XASUFW_AES_INVALID_PARAM;
		goto END_CLR;
	}

	if (AesKeyLookupTbl[KeyObject.KeySrc].UsrWrAllowed != XASU_TRUE) {
		Status = XASUFW_AES_INVALID_KEY_SRC;
		goto END_CLR;
	}

	if ((KeyObject.KeySize != XASU_AES_KEY_SIZE_128_BITS) &&
			(KeyObject.KeySize != XASU_AES_KEY_SIZE_256_BITS)) {
		Status = XASUFW_AES_INVALID_KEY_SIZE;
		goto END_CLR;
	}

	Offset = AesKeyLookupTbl[KeyObject.KeySrc].RegOffset;
	if (Offset == XAES_INVALID_CFG) {
		Status = XASUFW_AES_INVALID_KEY_SRC;
		goto END_CLR;
	}

	if (KeyObject.KeySize == XASU_AES_KEY_SIZE_128_BITS) {
		KeySizeInWords = XASU_AES_KEY_SIZE_128BIT_IN_WORDS;
	} else {
		KeySizeInWords = XASU_AES_KEY_SIZE_256BIT_IN_WORDS;
	}

	/** Copy Key from 64-bit address space to local array using SSS DMA loopback. */
	Status = XAsufw_DmaXfr(InstancePtr->AsuDmaPtr,  (u64)KeyObject.KeyAddress,
			(u64)(UINTPTR)Key, (KeySizeInWords * XASUFW_WORD_LEN_IN_BYTES), 0U);
	if (Status != XASUFW_SUCCESS) {
		goto END_CLR;
	}

	ASSIGN_VOLATILE(Status, XASUFW_FAILURE);
	/**
	 * Traverse to Offset of last word of respective USER key register
	 * (i.e., XAES_USER_KEY_X_7), where X = 0 to 7 USER key.
	 * Writes the key to the respective key source registers, by changing the endianness.
	 */
	Offset = Offset + (KeySizeInWords * XASUFW_WORD_LEN_IN_BYTES) - XASUFW_WORD_LEN_IN_BYTES;

	for (Index = 0U; Index < KeySizeInWords; Index++) {
		XAsufw_WriteReg((InstancePtr->KeyBaseAddress + Offset), Xil_Htonl(Key[Index]));
		Offset = Offset - XASUFW_WORD_LEN_IN_BYTES;
	}
	if ((Index == KeySizeInWords) && (KeySizeInWords != 0U)) {
		Status = XASUFW_SUCCESS;
	}

END_CLR:
	/** Clear local key object structure. */
	XFIH_CALL(Xil_SecureZeroize, XFihKeyClear, ClearStatus, (u8 *)(UINTPTR)&KeyObject,
		sizeof(XAsu_AesKeyObject));

	Status = XAsufw_UpdateBufStatus(Status, ClearStatus);

	/** Clear local key array. */
	XFIH_CALL(Xil_SecureZeroize, XFihKeyClear, ClearStatus, (u8 *)Key,
		XASU_AES_KEY_SIZE_256BIT_IN_BYTES);

	Status = XAsufw_UpdateBufStatus(Status, ClearStatus);

END:
	return Status;
}

/*************************************************************************************************/
/**
 * @brief	This function initializes the AES engine for encrypt/decrypt operation and loads
 * 		the provided key and IV to the AES engine.
 *
 * @param	InstancePtr	Pointer to the XAes instance.
 * @param	DmaPtr		Pointer to the AsuDma instance.
 * @param	KeyObjectAddr	Address of Aes key object structure, which contains
 *				- KeyAddress is address of user key.
 *				- KeySize is size of the user key.
 *			  	  - XASUFW_AES_KEY_SIZE_128 for 128 bit key size
 *				  - XASUFW_AES_KEY_SIZE_256 for 256 bit key size
 *				- KeySrc which indicates which hardware user key to be used to
 * 				  store the user key.
 *			  	  - XASU_AES_USER_KEY_0
 *				  - XASU_AES_USER_KEY_1
 *				  - XASU_AES_USER_KEY_2
 *				  - XASU_AES_USER_KEY_3
 *				  - XASU_AES_USER_KEY_4
 *				  - XASU_AES_USER_KEY_5
 *				  - XASU_AES_USER_KEY_6
 *				  - XASU_AES_USER_KEY_7
 * @param	IvAddr		Address of the buffer holding IV.
 * @param	IvLen		Length of the IV in bytes.
 * @param	EngineMode	AES engine mode.
 * @param	OperationType	AES operation type.
 *
 * @return
 *		- XASUFW_SUCCESS, if initialization of AES engine is successful.
 *		- XASUFW_AES_INVALID_PARAM, if InstancePtr or DmaPtr is NULL or ASU DMA is not
 * 			ready.
 *		- XASUFW_AES_STATE_MISMATCH_ERROR, if AES state is invalid.
 *		- XASUFW_AES_INVALID_KEY_OBJECT_ADDRESS, if KeyObjectAddr is invalid.
 *		- XASUFW_AES_INVALID_KEY_SRC, if key source is invalid.
 *		- XASUFW_AES_INVALID_KEY_SIZE, if key size is invalid.
 *		- XASUFW_AES_INVALID_ENGINE_MODE, if AES engine mode is invalid.
 *		- XASUFW_AES_INVALID_OPERATION_TYPE, if AES operation type is invalid.
 *
 *************************************************************************************************/
s32 XAes_Init(XAes *InstancePtr, XAsufw_Dma *DmaPtr, u64 KeyObjectAddr, u64 IvAddr, u32 IvLen,
	      u8 EngineMode, u8 OperationType)
{
	CREATE_VOLATILE(Status, XASUFW_FAILURE);
	s32 ClearStatus = XASUFW_FAILURE;
	XFih_Var XFihKeyClear;
	XAsu_AesKeyObject KeyObject;

	/** Validate the input arguments.*/
	if (InstancePtr == NULL) {
		Status = XASUFW_AES_INVALID_PARAM;
		goto END;
	}

	if ((DmaPtr == NULL) || (DmaPtr->AsuDma.IsReady != XIL_COMPONENT_IS_READY)) {
		Status = XASUFW_AES_INVALID_PARAM;
		goto END;
	}

	if ((InstancePtr->AesState != XAES_INITIALIZED)) {
		Status = XASUFW_AES_STATE_MISMATCH_ERROR;
		goto END;
	}

	if (KeyObjectAddr == 0U) {
		Status = XASUFW_AES_INVALID_KEY_OBJECT_ADDRESS;
		goto END;
	}

	/** Initialize the AES instance with ASU DMA pointer. */
	InstancePtr->AsuDmaPtr = DmaPtr;

	/**
	 * Copy KeyObject strucure from 64-bit address space to local structure using
	 * ASU DMA.
	 */
	Status = XAsufw_DmaXfr(InstancePtr->AsuDmaPtr, KeyObjectAddr,
		(u64)(UINTPTR)&KeyObject, sizeof(XAsu_AesKeyObject), 0U);
	if (Status != XASUFW_SUCCESS) {
		goto END;
	}

	ASSIGN_VOLATILE(Status, XASUFW_FAILURE);
	if (KeyObject.KeySrc >= XAES_MAX_KEY_SOURCES) {
		Status = XASUFW_AES_INVALID_KEY_SRC;
		goto END;
	}

	if ((KeyObject.KeySize != XASU_AES_KEY_SIZE_128_BITS) &&
			(KeyObject.KeySize != XASU_AES_KEY_SIZE_256_BITS)) {
		Status = XASUFW_AES_INVALID_KEY_SIZE;
		goto END;
	}

	if ((EngineMode > XASU_AES_GCM_MODE) && (EngineMode != XASU_AES_CMAC_MODE) &&
			(EngineMode != XASU_AES_GHASH_MODE)) {
		Status = XASUFW_AES_INVALID_ENGINE_MODE;
		goto END;
	}

	if ((OperationType != XASU_AES_ENCRYPT_OPERATION) &&
			(OperationType != XASU_AES_DECRYPT_OPERATION)) {
		Status = XASUFW_AES_INVALID_OPERATION_TYPE;
		goto END;
	}

	/** Check whether key is zeroed or not. */
	Status = XAes_IsKeyZeroized(InstancePtr, KeyObject.KeySrc);
	if (Status != XASUFW_SUCCESS) {
		goto END;
	}

	/** Initialize the AES instance. */
	InstancePtr->EngineMode = EngineMode;
	InstancePtr->OperationType = OperationType;

	ASSIGN_VOLATILE(Status, XASUFW_FAILURE);
	/* Validate the IV with respect to the user provided engine mode */
	Status = XAsu_AesValidateIv(InstancePtr->EngineMode, IvAddr, IvLen);
	if (Status != XASUFW_SUCCESS) {
		Status = XASUFW_AES_INVALID_IV;
		goto END;
	}

	/* Release reset of AES engine */
	XAsufw_CryptoCoreReleaseReset(InstancePtr->AesBaseAddress, XAES_SOFT_RST_OFFSET);

	/** Configure AES DPA counter measures. */
	XAes_ConfigCounterMeasures(InstancePtr);

	/** Configure AES engine to encrypt/decrypt operation. */
	XAes_ConfigAesOperation(InstancePtr);

	/** Load key to AES engine. */
	XAes_LoadKey(InstancePtr, KeyObject.KeySrc, KeyObject.KeySize);

	/** Process and load IV to AES engine. */
	if ((InstancePtr->EngineMode != XASU_AES_ECB_MODE) &&
			(InstancePtr->EngineMode != XASU_AES_CMAC_MODE)) {
		ASSIGN_VOLATILE(Status, XASUFW_FAILURE);
		Status = XAes_ProcessAndLoadIv(InstancePtr, IvAddr, IvLen);
		if (Status != XASUFW_SUCCESS) {
			goto END;
		}
	}

	/** Update AES state machine to STARTED. */
	InstancePtr->AesState = XAES_STARTED;

END:
	/** Clear local key object buffer. */
	XFIH_CALL(Xil_SecureZeroize, XFihKeyClear, ClearStatus, (u8 *)(UINTPTR)&KeyObject,
		sizeof(XAsu_AesKeyObject));
	if ((InstancePtr != NULL) && (Status != XASUFW_SUCCESS)) {
		/** Set AES under reset upon failure. */
		XAes_SetReset(InstancePtr);
	}

	Status = XAsufw_UpdateBufStatus(Status, ClearStatus);

	return Status;
}

/*************************************************************************************************/
/**
 * @brief	This function updates AES engine with the provided input data and stores the
 * 		resultant data at specified output address.
 *
 * @param	InstancePtr	Pointer to the XAes instance.
 * @param	DmaPtr		Pointer to the AsuDma instance.
 * @param	InDataAddr	Address of the input data for encrypt/decrypt operation.
 * @param	OutDataAddr	Address of output buffer where the encrypted/decrypted data to be
 *				updated and if input data is AAD then, OutDataAddr will be zero.
 * @param	DataLength	Length of the data in bytes shall never be zero.
 * 				For AES-CBC and AES-ECB modes, length shall be aligned to 16 bytes.
 * @param	IsLastChunk	For the last update of data, this parameter should be set
 *		 	 	to TRUE otherwise, FALSE.
 *
 * @return
 *		- XASUFW_SUCCESS, if data updation is successful.
 *		- XASUFW_AES_INVALID_PARAM, if InstancePtr or DmaPtr is NULL or ASU DMA is
 * 			not ready.
 *		- XASUFW_AES_STATE_MISMATCH_ERROR, if Aes state is invalid.
 *		- XASUFW_AES_INVALID_INPUT_DATA, if input data address is invalid.
 *		- XASUFW_AES_INVALID_INPUT_DATA_LENGTH, if input data length is invalid.
 *		- XASUFW_AES_INVALID_ISLAST_CHUNK, if IsLastChunk is invalid.
 *		- XASUFW_AES_UNALIGNED_BLOCK_SIZE_INPUT_LENGTH, if input data length is not 16bytes
 * 			aligned for CBC and ECB mode.
 *
 *************************************************************************************************/
s32 XAes_Update(XAes *InstancePtr, XAsufw_Dma *DmaPtr, u64 InDataAddr, u64 OutDataAddr,
		u32 DataLength, u8 IsLastChunk)
{
	CREATE_VOLATILE(Status, XASUFW_FAILURE);

	/** Validate the input arguments. */
	if (InstancePtr == NULL) {
		Status = XASUFW_AES_INVALID_PARAM;
		goto END;
	}

	if ((DmaPtr == NULL) || (DmaPtr->AsuDma.IsReady != XIL_COMPONENT_IS_READY)) {
		Status = XASUFW_AES_INVALID_PARAM;
		goto END;
	}

	if ((InstancePtr->AesState != XAES_STARTED) &&
			(InstancePtr->AesState != XAES_UPDATE_IN_PROGRESS)) {
		Status = XASUFW_AES_STATE_MISMATCH_ERROR;
		goto END;
	}

	if (InDataAddr == 0U) {
		Status = XASUFW_AES_INVALID_INPUT_DATA;
		goto END;
	}

	/**
	 * The minimum length of plaintext/AAD data must be at least 8 bits, while the
	 * maximum length should be less than 0x1FFFFFFC bytes, which is the
	 * ASU DMA's maximum supported data transfer length.
	 */
	if ((DataLength == 0U) || (DataLength > XASU_ASU_DMA_MAX_TRANSFER_LENGTH)) {
		Status = XASUFW_AES_INVALID_INPUT_DATA_LENGTH;
		goto END;
	}

	if ((IsLastChunk != XASU_TRUE) && (IsLastChunk != XASU_FALSE)) {
		Status = XASUFW_AES_INVALID_ISLAST_CHUNK;
		goto END;
	}

	/**
	 * For the ECB, CBC, and CFB modes, the plaintext must be a sequence of one or more
	 * complete data blocks.
	 */
	if (((InstancePtr->EngineMode == XASU_AES_ECB_MODE) ||
			(InstancePtr->EngineMode == XASU_AES_CBC_MODE) ||
			(InstancePtr->EngineMode == XASU_AES_CFB_MODE)) &&
			((DataLength % XASU_AES_BLOCK_SIZE_IN_BYTES) != 0U)) {
		Status = XASUFW_AES_UNALIGNED_BLOCK_SIZE_INPUT_LENGTH;
		goto END;
	}

	/** Initialize the AES instance with ASU DMA pointer. */
	InstancePtr->AsuDmaPtr = DmaPtr;

	/**
	 * If the output data address is zero, the input data will be considered as AAD data.
	 * Configure AES AAD configurations before pushing AAD data to AES engine and clear AAD
	 * configuration post DMA transfer.
	 */
	if (OutDataAddr == 0U) {
		XAes_ConfigAad(InstancePtr);
	}

	/** Configure DMA with AES and transfer the data to AES engine. */
	Status = XAes_CfgDmaWithAesAndXfer(InstancePtr, InDataAddr, OutDataAddr, DataLength,
		IsLastChunk);
	if (Status != XASUFW_SUCCESS) {
		goto END;
	}

	if (OutDataAddr == 0U) {
		XAes_ClearConfigAad(InstancePtr);
	}

	/**
	 * Update the AES state machine to XAES_UPDATE_IN_PROGRESS or XAES_UPDATE_COMPLETED based
	 * on the IsLastChunk flag set by the user.
	 */
	if (IsLastChunk == TRUE) {
		InstancePtr->AesState = XAES_UPDATE_COMPLETED;
	} else {
		InstancePtr->AesState = XAES_UPDATE_IN_PROGRESS;
	}

END:
	if ((InstancePtr != NULL) && (Status != XASUFW_SUCCESS)) {
		/** Set AES under reset upon failure. */
		XAes_SetReset(InstancePtr);
	}

	return Status;
}

/*************************************************************************************************/
/**
 * @brief	This function waits for AES operation to complete. For MAC modes, it
 * 		generates/verifies the tag based on the AES operation type.
 *
 * @param	InstancePtr	Pointer to the XAes instance.
 * @param	DmaPtr		Pointer to the AsuDma instance.
 * @param	TagAddr		Address of the tag buffer.
 * @param	TagLen		Length of tag in bytes and it will be zero for all AES
 *				standard modes like, ECB, CBC, OFB, CFB, CTR
 *
 * @return
 *		- XASUFW_SUCCESS, if completion of final AES operation is successful.
 *		- XASUFW_AES_INVALID_PARAM, if InstancePtr or DmaPtr is NULL or ASU DMA is
 * 			not ready.
 *		- XASUFW_AES_STATE_MISMATCH_ERROR, if AES state is invalid.
 *
 *************************************************************************************************/
s32 XAes_Final(XAes *InstancePtr, XAsufw_Dma *DmaPtr, u64 TagAddr, u32 TagLen)
{
	CREATE_VOLATILE(Status, XASUFW_FAILURE);
	XFih_Var XFihTagStatus;

	/** Validate the input arguments. */
	if (InstancePtr == NULL) {
		Status = XASUFW_AES_INVALID_PARAM;
		goto END;
	}

	if ((DmaPtr == NULL) || (DmaPtr->AsuDma.IsReady != XIL_COMPONENT_IS_READY)) {
		Status = XASUFW_AES_INVALID_PARAM;
		goto END;
	}

	if (InstancePtr->AesState != XAES_UPDATE_COMPLETED) {
		Status = XASUFW_AES_STATE_MISMATCH_ERROR;
		goto END;
	}

	/** Validate the tag with respect to the user provided engine mode. */
	XFIH_CALL_GOTO(XAsu_AesValidateTag, XFihTagStatus, Status, END, InstancePtr->EngineMode,
		TagAddr, TagLen);

	/** Initialize the AES instance with ASU DMA pointer. */
	InstancePtr->AsuDmaPtr = DmaPtr;

	ASSIGN_VOLATILE(Status, XASUFW_FAILURE);
	/** Wait for AES operation to complete. */
	Status = XAes_WaitForDone(InstancePtr);
	if (Status != XASUFW_SUCCESS) {
		goto END;
	}

	/**
	 * For AES MAC modes,
	 * - For encryption, it generates and stores the tag at specified TagAddr.
	 * - For decryption, compares the generated tag with the provided tag.
	 */
	if ((InstancePtr->EngineMode == XASU_AES_CCM_MODE) ||
			(InstancePtr->EngineMode == XASU_AES_GCM_MODE) ||
			(InstancePtr->EngineMode == XASU_AES_CMAC_MODE)) {
		XFIH_CALL(XAes_ProcessTag, XFihTagStatus, Status, InstancePtr, TagAddr,
			TagLen);
	}

END:
	if (InstancePtr != NULL) {
		/** Set AES under reset on failure. */
		XAes_SetReset(InstancePtr);
	}

	return Status;
}

/*************************************************************************************************/
/**
 * @brief	This function returns a pointer reference of XAes_Config structure based on the
 * 		device ID.
 *
 * @param	DeviceId	The device ID of the AES core.
 *
 * @return
 * 		- CfgPtr, a reference to a config record in the configuration table
 * 			corresponding to <i>DeviceId</i>.
 * 		- NULL, if no valid device ID is found.
 *
 *************************************************************************************************/
static XAes_Config *XAes_LookupConfig(u16 DeviceId)
{
	XAes_Config *CfgPtr = NULL;
	u32 Index = 0U;

	/* Checks for all the instances */
	for (Index = 0U; Index < XASU_XAES_NUM_INSTANCES; Index++) {
		if (AesConfigTable[Index].DeviceId == DeviceId) {
			CfgPtr = &AesConfigTable[Index];
			break;
		}
	}

	return CfgPtr;
}

/*************************************************************************************************/
/**
 * @brief	This function checks whether given key source is in zeroized status or not.
 *
 * @param	InstancePtr	Pointer to the XAes instance.
 * @param	KeySrc		Key source.
 *
 * @return
 *		- XASUFW_SUCCESS, if key is not zeroized.
 *		- XASUFW_AES_KEY_ZEROIZED, if key is zeroized.
 *
 *************************************************************************************************/
static s32 XAes_IsKeyZeroized(const XAes *InstancePtr, u32 KeySrc)
{
	CREATE_VOLATILE(Status, XASUFW_AES_KEY_ZEROIZED);
	u32 KeyZeroedStatus;

	/** Read the key zeroized status register. */
	KeyZeroedStatus = XAsufw_ReadReg(InstancePtr->KeyBaseAddress + XAES_KEY_ZEROED_STATUS_OFFSET);

	/** Check the key zeroized status with its zeroized mask. */
	if ((KeyZeroedStatus & AesKeyLookupTbl[KeySrc].KeyZeroedStatusMask) == 0U) {
		Status = XASUFW_SUCCESS;
	}

	return Status;
}

/*************************************************************************************************/
/**
 * @brief	This function configures the AES engine for encrypt/decrypt operation based on the
 * 		OperationType selected by the client while sending the request.
 *
 * @param	InstancePtr	Pointer to the XAes instance.
 *
 * @note	On soft reset, the DPA CM is enabled by default. So, it needs to be configured at
 * 		every start of AES operation.
 *
 *************************************************************************************************/
static void XAes_ConfigCounterMeasures(const XAes *InstancePtr)
{
	if (InstancePtr->AesCmConfig == XASUFW_CONFIG_ENABLE) {
		XAsufw_WriteReg((InstancePtr->AesBaseAddress + XAES_CM_OFFSET),
			XAES_CM_ENABLE_MASK);
	}
	else {
		XAsufw_WriteReg((InstancePtr->AesBaseAddress + XAES_CM_OFFSET),
			XAES_CM_DISABLE_MASK);
	}
}

/*************************************************************************************************/
/**
 * @brief	This function configures the AES engine for encrypt/decrypt operation based on the
 * 		OperationType.
 *
 * @param	InstancePtr	Pointer to the XAes instance.
 *
 *************************************************************************************************/
static void XAes_ConfigAesOperation(const XAes *InstancePtr)
{
	if (InstancePtr->OperationType == XASU_AES_ENCRYPT_OPERATION) {
		XAsufw_WriteReg((InstancePtr->AesBaseAddress + XAES_MODE_CONFIG_OFFSET),
			(InstancePtr->EngineMode | (XAES_MODE_CONFIG_ENC_DEC_MASK)));
	} else {
		XAsufw_WriteReg((InstancePtr->AesBaseAddress + XAES_MODE_CONFIG_OFFSET),
			InstancePtr->EngineMode);
	}
}

/*************************************************************************************************/
/**
 * @brief	This function loads key from selected key source to AES engine.
 *
 * @param	InstancePtr	Pointer to the XAes instance.
 * @param	KeySrc		Key source.
 * @param	KeySize		Size of the key.
 *
 *************************************************************************************************/
static void XAes_LoadKey(const XAes *InstancePtr, u32 KeySrc, u32 KeySize)
{
	XAsufw_WriteReg((InstancePtr->KeyBaseAddress + XAES_KEY_SIZE_OFFSET), KeySize);

	XAsufw_WriteReg((InstancePtr->KeyBaseAddress + XAES_KEY_SEL_OFFSET),
		AesKeyLookupTbl[KeySrc].KeySrcSelVal);

	XAsufw_WriteReg((InstancePtr->AesBaseAddress + XAES_OPERATION_OFFSET), XAES_KEY_LOAD_VAL_MASK);
}

/*************************************************************************************************/
/**
 * @brief	This function processes the provided IV and writes it into the appropriate AES IV
 * 		registers based on length of the IV and loads IV to the AES engine.
 *
 * @param	InstancePtr	Pointer to the XAes instance.
 * @param	IvAddr		Address of an IV buffer.
 * @param	IvLen		Length of an IV in bytes.
 *
 * @return
 *		- XASUFW_SUCCESS, if processing and loading of IV to AES engine is successful.
 *		- XASUFW_FAILURE, if processing or loading of IV fails.
 *
 *************************************************************************************************/
static s32 XAes_ProcessAndLoadIv(XAes *InstancePtr, u64 IvAddr, u32 IvLen)
{
	CREATE_VOLATILE(Status, XASUFW_FAILURE);
	s32 ClearStatus = XASUFW_FAILURE;
	XFih_Var XFihIvClear;
	u32 Index = 0U;
	u32 Iv[XASU_AES_IV_SIZE_128BIT_IN_WORDS] = {0U};
	u32 IvLength = IvLen;

	/**
	 * For AES-GCM mode, if the IV length is not 96 bits, calculate GHASH and
	 * generate a new IV.
	 * In all other cases, copy IV from 64-bit address space to local array using ASU DMA.
	 */
	if ((InstancePtr->EngineMode == XASU_AES_GCM_MODE) &&
			(IvLength != XASU_AES_IV_SIZE_96BIT_IN_BYTES)) {
		Status = XAes_GHashCal(InstancePtr, IvAddr, (u32)Iv, IvLength);
		if (Status != XASUFW_SUCCESS) {
			goto END;
		}
		IvLength = XASU_AES_IV_SIZE_128BIT_IN_BYTES;
	} else {
		Status = XAsufw_DmaXfr(InstancePtr->AsuDmaPtr, IvAddr, (u64)(UINTPTR)Iv,
				       IvLength, 0U);
		if (Status != XASUFW_SUCCESS) {
			goto END;
		}
	}

	/** Write IV to the respective IV registers, by changing the endianness. */
	for (Index = 0U; Index < XASUFW_CONVERT_BYTES_TO_WORDS(IvLength); Index++) {
		XAsufw_WriteReg((InstancePtr->AesBaseAddress +
				 (XAES_IV_IN_3_OFFSET - (Index * XASUFW_WORD_LEN_IN_BYTES))),
				Xil_Htonl(Iv[Index]));
	}

	if ((InstancePtr->EngineMode == XASU_AES_GCM_MODE) &&
			(IvLength  == XASU_AES_IV_SIZE_96BIT_IN_BYTES)) {
		XAsufw_WriteReg((InstancePtr->AesBaseAddress + XAES_IV_IN_0_OFFSET), 0x01U);
	}

	/** Trigger IV Load. */
	XAsufw_WriteReg((InstancePtr->AesBaseAddress + XAES_OPERATION_OFFSET), XAES_IV_LOAD_VAL_MASK);

END:
	/** Zeroize local IV buffer. */
	XFIH_CALL(Xil_SecureZeroize, XFihIvClear, ClearStatus, (u8 *)Iv,
		XASU_AES_IV_SIZE_128BIT_IN_BYTES);

	Status = XAsufw_UpdateBufStatus(Status, ClearStatus);

	return Status;
}

/*************************************************************************************************/
/**
 * @brief	This function calculates the GHASH on provided data.
 *
 * @param	InstancePtr	Pointer to the XAes instance.
 * @param	IvAddr		Address of IV buffer.
 * @param	IvGen		Address of the buffer to store the generated MAC.
 * @param	IvLen		Length of the input IV buffer in bytes.
 *
 * @return
 *		- XASUFW_SUCCESS, if GHASH calculation on given Iv is successful.
 *		- XASUFW_FAILURE, if GHASH calculation on given Iv fails.
 *
 *************************************************************************************************/
static s32 XAes_GHashCal(XAes *InstancePtr, u64 IvAddr, u32 IvGen, u32 IvLen)
{
	CREATE_VOLATILE(Status, XASUFW_FAILURE);
	u32 ReadModeConfigReg;

	/* Store the mode configuration of previous mode i.e., GCM mode. */
	ReadModeConfigReg = XAsufw_ReadReg(InstancePtr->AesBaseAddress + XAES_MODE_CONFIG_OFFSET);

	InstancePtr->EngineMode = XASU_AES_GHASH_MODE;

	/** Configure engine mode to GHASH mode. */
	XAsufw_WriteReg((InstancePtr->AesBaseAddress + XAES_MODE_CONFIG_OFFSET),
			(XAES_MODE_CONFIG_AUTH_MASK | InstancePtr->EngineMode));

	/** Configure DMA with AES and transfer the data to AES engine. */
	Status = XAes_CfgDmaWithAesAndXfer(InstancePtr, IvAddr, 0U, IvLen, XASU_TRUE);
	if (Status != XASUFW_SUCCESS) {
		goto END;
	}

	/* Disable auth mask and restore mode configuration. */
	XAsufw_WriteReg((InstancePtr->AesBaseAddress + XAES_MODE_CONFIG_OFFSET),
			(ReadModeConfigReg & (~(XAES_MODE_CONFIG_AUTH_MASK))));

	ASSIGN_VOLATILE(Status, XASUFW_FAILURE);
	/** Get newly generated IV from the MAC registers. */
	Status = XAes_ReadTag(InstancePtr, IvGen, XASU_AES_MAX_TAG_LENGTH_IN_BYTES);

END:
	return Status;
}

/*************************************************************************************************/
/**
 * @brief	This function reads the tag from AES MAC registers during encryption operation.
 *
 * @param	InstancePtr	Pointer to the XAes instance.
 * @param	TagOutAddr	Address of the buffer to store tag.
 * @param	TagLen		Length of the tag in bytes.
 *
 * @return
 *		- XASUFW_SUCCESS, Upon successful read.
 *		- XASUFW_AES_TAG_GENERATE_FAILED, if tag read fails.
 *
 *************************************************************************************************/
static s32 XAes_ReadTag(const XAes *InstancePtr, u32 TagOutAddr, u32 TagLen)
{
	CREATE_VOLATILE(Status, XASUFW_AES_TAG_GENERATE_FAILED);
	volatile u32 Index = 0U;
	u32 *TagPtr = (u32 *)TagOutAddr;

	for (Index = 0U; Index < XASUFW_CONVERT_BYTES_TO_WORDS(TagLen); Index++) {
		TagPtr[Index] = Xil_EndianSwap32(XAsufw_ReadReg(InstancePtr->AesBaseAddress +
						 (XAES_MAC_OUT_3_MASK - (Index * XASUFW_WORD_LEN_IN_BYTES))));
		/*
		 * If AES DPA CM is enabled then, read MAC from both MAC_OUT and MAC_MASK_OUT registers,
		 * If disabled, no need to read the MAC_MASK_OUT register as it contains zeros.
		 */
#ifdef XASUFW_AES_CM_CONFIG
		TagPtr[Index] ^= Xil_EndianSwap32(XAsufw_ReadReg(InstancePtr->AesBaseAddress +
						  (XAES_MAC_MASK_OUT_3_MASK - (Index * XASUFW_WORD_LEN_IN_BYTES))));
#endif
	}
	if (Index == XASUFW_CONVERT_BYTES_TO_WORDS(TagLen)) {
		Status = XASUFW_SUCCESS;
	}

	return Status;
}

/*************************************************************************************************/
/**
 * @brief	This function reads from AES MAC registers and verifies it with the user provided
 * 		tag during decryption operation.
 *
 * @param	InstancePtr	Pointer to the XAes instance.
 * @param	TagInAddr	Address of the buffer which holds the expected MAC.
 * @param	TagLen		Length of the tag in bytes.
 *
 * @return
 *		- XASUFW_SUCCESS, if tag comparision is successful.
 *		- XASUFW_AES_TAG_COMPARE_FAILED, if tag comparision fails.
 *
 *************************************************************************************************/
static s32 XAes_ReadNVerifyTag(const XAes *InstancePtr, u32 TagInAddr, u32 TagLen)
{
	CREATE_VOLATILE(Status, XASUFW_AES_TAG_COMPARE_FAILED);
	volatile u32 Index = 0U;
	u32 ReadReg;
	const u32 *TagPtr = (const u32 *)TagInAddr;

	for (Index = 0U; Index < XASUFW_CONVERT_BYTES_TO_WORDS(TagLen); Index++) {
		ReadReg = Xil_EndianSwap32(XAsufw_ReadReg(InstancePtr->AesBaseAddress +
					   (XAES_MAC_OUT_3_MASK - (Index * XASUFW_WORD_LEN_IN_BYTES))));
		/*
		 * If AES DPA CM is enabled then, read MAC from both MAC_OUT and MAC_MASK_OUT registers,
		 * If disabled, no need to read the MAC_MASK_OUT register as it contains zeros.
		 */
#ifdef XASUFW_AES_CM_CONFIG
		ReadReg ^= Xil_EndianSwap32(XAsufw_ReadReg(InstancePtr->AesBaseAddress +
					    (XAES_MAC_MASK_OUT_3_MASK - (Index * XASUFW_WORD_LEN_IN_BYTES))));
#endif
		if (ReadReg != TagPtr[Index]) {
			goto END;
		}
	}
	if (Index == XASUFW_CONVERT_BYTES_TO_WORDS(TagLen)) {
		Status = XASUFW_SUCCESS;
	}

END:
	return Status;
}

/*************************************************************************************************/
/**
 * @brief	This function processes tag based on AES operation type, if the Operation type is
 * 		encryption then, it generates the tag and stores it into output TagAddr and for
 * 		decryption it generates and compares the tag with input TagAddr.
 *
 * @param	InstancePtr	Pointer to the XAes instance.
 * @param	TagAddr		Address of the tag buffer.
 * @param	TagLen		Length of the tag in bytes.
 *
 * @return
 *		- XASUFW_SUCCESS, if read/verification of the tag is successful based on AES
 *			operation type.
 *		- XASUFW_FAILURE, if read/verification of the tag fails.
 *
 *************************************************************************************************/
static s32 XAes_ProcessTag(const XAes *InstancePtr, u64 TagAddr, u32 TagLen)
{
	CREATE_VOLATILE(Status, XASUFW_FAILURE);
	u8 Tag[XASU_AES_MAX_TAG_LENGTH_IN_BYTES] = {0U};

	if (InstancePtr->OperationType == XASU_AES_ENCRYPT_OPERATION) {
		/* Generate tag, if AES operation type is encryption. */
		Status = XAes_ReadTag(InstancePtr, (u32)Tag, TagLen);
		if (Status != XASUFW_SUCCESS) {
			goto END;
		}

		ASSIGN_VOLATILE(Status, XASUFW_FAILURE);
		/*
		 * Copy generated tag from local array to 64-bit address space using
		 * SSS DMA loopback.
		 */
		Status = XAsufw_DmaXfr(InstancePtr->AsuDmaPtr, (u64)(UINTPTR)Tag, TagAddr,
				       TagLen, 0U);
	} else {
		/*
		 * Copy tag from 64-bit address space to local array using SSS DMA
		 * loopback.
		 */
		Status = XAsufw_DmaXfr(InstancePtr->AsuDmaPtr, TagAddr, (u64)(UINTPTR)Tag,
				       TagLen, 0U);
		if (Status != XASUFW_SUCCESS) {
			goto END;
		}

		ASSIGN_VOLATILE(Status, XASUFW_FAILURE);
		/* Generate and verify the tag, if AES operation type is decryption. */
		Status = XAes_ReadNVerifyTag(InstancePtr, (u32)Tag, TagLen);
	}

END:
	return Status;
}

/*************************************************************************************************/
/**
 * @brief	This function configures AAD.
 *
 * @param	InstancePtr	Pointer to the XAes instance.
 *
 *************************************************************************************************/
static void XAes_ConfigAad(const XAes *InstancePtr)
{
	if ((InstancePtr->EngineMode == XASU_AES_GCM_MODE) ||
	    (InstancePtr->EngineMode == XASU_AES_CCM_MODE)) {
		/**
		 * For modes like AES-GCM and AES-CCM, where payload is expected after AAD,
		 * configure only AUTH_MASK bit.
		 */
		XAsufw_RCMW((InstancePtr->AesBaseAddress + XAES_MODE_CONFIG_OFFSET),
			    XAES_MODE_CONFIG_AUTH_MASK, XAES_MODE_CONFIG_AUTH_MASK);
	} else {
		/**
		 * For MAC only modes like AES-GMAC and AES-CMAC, where no payload is expected to
		 * follow AAD data, configure both AUTH_MASK and AUTH_WITH_NO_PAYLOAD bit.
		 */
		XAsufw_RCMW((InstancePtr->AesBaseAddress + XAES_MODE_CONFIG_OFFSET),
			    (XAES_MODE_CONFIG_AUTH_MASK | XAES_MODE_CONFIG_AUTH_WITH_NO_PAYLOAD_MASK),
			    (XAES_MODE_CONFIG_AUTH_MASK | XAES_MODE_CONFIG_AUTH_WITH_NO_PAYLOAD_MASK));
	}
}

/*************************************************************************************************/
/**
 * @brief	This function clears configuration of AAD after pushing final AAD data to
 * 		AES engine.
 *
 * @param	InstancePtr	Pointer to the XAes instance.
 *
 *************************************************************************************************/
static void XAes_ClearConfigAad(const XAes *InstancePtr)
{
	/** Clear AUTH and AUTH_WITH_NO_PAYLOAD bits in AES mode configuration register. */
	XAsufw_RMW((InstancePtr->AesBaseAddress + XAES_MODE_CONFIG_OFFSET),
		   (XAES_MODE_CONFIG_AUTH_MASK | XAES_MODE_CONFIG_AUTH_WITH_NO_PAYLOAD_MASK), 0U);
}

/*************************************************************************************************/
/**
 * @brief	This function configures SSS and transfers data to AES engine using DMA.
 *
 * @param	InstancePtr	Pointer to the XAes instance.
 * @param	InDataAddr	Address of the input buffer.
 * @param	OutDataAddr	Address of output buffer to hold the resultant data.
 * @param	Size		Size of data to send/receive to/from AES engine in bytes.
 * @param	IsLastChunk	For the last update of data, this parameter should be set
 *		 	 	to TRUE otherwise, FALSE.
 *
 * @return
 *		- XASUFW_SUCCESS, upon successful transfer.
 *		- XASUFW_FAILURE, upon any failure.
 *
 *************************************************************************************************/
static s32 XAes_CfgDmaWithAesAndXfer(const XAes *InstancePtr, u64 InDataAddr, u64 OutDataAddr, u32 Size,
				     u8 IsLastChunk)
{
	CREATE_VOLATILE(Status, XASUFW_FAILURE);

	/** Configure SSS. */
	Status = XAsufw_SssAesWithDma(InstancePtr->AsuDmaPtr->SssDmaCfg);
	if (Status != XASUFW_SUCCESS) {
		goto END;
	}

	/* If OutDataAddr is non-zero address then, configure destination channel and transfer. */
	if (OutDataAddr != 0U) {
		XAsuDma_ByteAlignedTransfer(&InstancePtr->AsuDmaPtr->AsuDma, XCSUDMA_DST_CHANNEL,
					    OutDataAddr, Size, IsLastChunk);
	}

	XAsuDma_ByteAlignedTransfer(&InstancePtr->AsuDmaPtr->AsuDma, XCSUDMA_SRC_CHANNEL,
				    InDataAddr, Size, IsLastChunk);

	ASSIGN_VOLATILE(Status, XASUFW_FAILURE);
	/** Wait till the ASU source DMA done bit to set. */
	Status = XAsuDma_WaitForDoneTimeout(&InstancePtr->AsuDmaPtr->AsuDma,
					    XCSUDMA_SRC_CHANNEL);
	if (Status != XASUFW_SUCCESS) {
		goto END;
	}

	/** Acknowledge the transfer has completed from source. */
	XAsuDma_IntrClear(&InstancePtr->AsuDmaPtr->AsuDma, XCSUDMA_SRC_CHANNEL,
			  XCSUDMA_IXR_DONE_MASK);

	if (OutDataAddr != 0U) {
		ASSIGN_VOLATILE(Status, XASUFW_FAILURE);
		/** Wait till the ASU destination DMA done bit to set. */
		Status = XAsuDma_WaitForDoneTimeout(&InstancePtr->AsuDmaPtr->AsuDma,
						    XCSUDMA_DST_CHANNEL);
		if (Status != XASUFW_SUCCESS) {
			goto END;
		}

		/** Acknowledge the transfer has completed from destination. */
		XAsuDma_IntrClear(&InstancePtr->AsuDmaPtr->AsuDma, XCSUDMA_DST_CHANNEL,
				  XCSUDMA_IXR_DONE_MASK);
	}

END:
	return Status;
}

/*************************************************************************************************/
/**
 * @brief	This function waits for AES operation to complete.
 *
 * @param	InstancePtr	Pointer to the XAes instance.
 *
 * @return
 *		- XASUFW_SUCCESS, if AES operation is successful.
 *		- XASUFW_FAILURE, upon timeout.
 *
 *************************************************************************************************/
static s32 XAes_WaitForDone(const XAes *InstancePtr)
{
	CREATE_VOLATILE(Status, XASUFW_FAILURE);

	/* Check for AES operation is completed within Timeout(10sec) or not. */
	Status = (s32)Xil_WaitForEvent((InstancePtr->AesBaseAddress + XAES_INTERRUPT_STATUS_OFFSET),
				  XAES_INTERRUPT_STATUS_DONE_MASK, XAES_INTERRUPT_STATUS_DONE_MASK, XAES_TIMEOUT_MAX);

	return Status;
}

/*************************************************************************************************/
/**
 * @brief	This function places the AES hardware core into reset state and updates AES state
 * 		machine to initialized state.
 *
 * @param	InstancePtr	Pointer to the XAes instance.
 *
 *************************************************************************************************/
static void XAes_SetReset(XAes *InstancePtr)
{
	InstancePtr->AesState = XAES_INITIALIZED;
	XAsufw_CryptoCoreSetReset(InstancePtr->AesBaseAddress, XAES_SOFT_RST_OFFSET);
}

/*************************************************************************************************/
/**
 * @brief	This function updates data and key to the AES engine in split mode with DPA CM
 * 		enabled.
 *
 * @param	InstancePtr	Pointer to the XAes instance.
 * @param	DmaPtr		Pointer to the AsuDma instance.
 * @param 	KeyObjPtr	Pointer to the XAsu_AesKeyObject instance.
 * @param 	InputDataAddr	Input data address.
 * @param 	OutputDataAddr	Output address where the decrypted data to be stored.
 * @param	DataLength	Length of both input/output data in bytes.
 *
 * @return
 *		- XASUFW_SUCCESS, if decryption of data is successful.
 *		- XASUFW_AES_INVALID_PARAM, if InstancePtr or DmaPtr or KeyObjPtr is NULL or
 * 			ASU DMA is not ready or input or output address is invalid.
 *
 *************************************************************************************************/
s32 XAes_DpaCmDecryptData(XAes *InstancePtr, XAsufw_Dma *DmaPtr, XAsu_AesKeyObject *KeyObjPtr,
			  u32 InputDataAddr, u32 OutputDataAddr, u32 DataLength)
{
	CREATE_VOLATILE(Status, XASUFW_FAILURE);
	u32 Index = 0U;

	/** Validate the input arguments. */
	if (InstancePtr == NULL) {
		Status = XASUFW_AES_INVALID_PARAM;
		goto END;
	}

	if ((DmaPtr == NULL) || (DmaPtr->AsuDma.IsReady != XIL_COMPONENT_IS_READY)) {
		Status = XASUFW_AES_INVALID_PARAM;
		goto END;
	}

	if ((KeyObjPtr == NULL) || (InputDataAddr == 0U) || (OutputDataAddr == 0U)) {
		Status = XASUFW_AES_INVALID_PARAM;
		goto END;
	}

	if (DataLength == 0U) {
		Status = XASUFW_AES_INVALID_PARAM;
		goto END;
	}

	/** Initialize the AES instance with ASU DMA pointer. */
	InstancePtr->AsuDmaPtr = DmaPtr;

	/** Release soft reset of AES engine. */
	XAsufw_CryptoCoreReleaseReset(InstancePtr->AesBaseAddress, XAES_SOFT_RST_OFFSET);

	/** Configure AES engine for decrypt operation. */
	XAsufw_WriteReg((InstancePtr->AesBaseAddress + XAES_MODE_CONFIG_OFFSET),
			XASU_AES_GCM_MODE);

	/** Configure AES engine in split mode to update data and key to AES core. */
	XAsufw_WriteReg((InstancePtr->AesBaseAddress + XAES_SPLIT_CFG_OFFSET),
			(XAES_SPLIT_CFG_KEY_SPLIT_VALUE | XAES_SPLIT_CFG_DATA_SPLIT_VALUE));

	/** Configure AES DPA counter measures. */
	XAes_ConfigCounterMeasures(InstancePtr);

	/** Write key mask value. */
	for (Index = 0U; Index < XASU_AES_KEY_SIZE_256BIT_IN_WORDS; Index++) {
		XAsufw_WriteReg((InstancePtr->KeyBaseAddress + (XAES_KEY_MASK_0_OFFSET +
				 (u32)(Index * XASUFW_WORD_LEN_IN_BYTES))), 0x0U);
	}

	/** Write AES key. */
	Status = XAes_WriteKey(InstancePtr, DmaPtr, (u64)(UINTPTR)KeyObjPtr);
	if (Status != XASUFW_SUCCESS) {
		goto END;
	}

	/** Load key to AES engine. */
	XAes_LoadKey(InstancePtr, KeyObjPtr->KeySrc, KeyObjPtr->KeySize);

	ASSIGN_VOLATILE(Status, XASUFW_FAILURE);
	Status = XAes_CfgDmaWithAesAndXfer(InstancePtr, InputDataAddr, OutputDataAddr, DataLength, XASU_TRUE);

END:
	return Status;
}
/** @} */
