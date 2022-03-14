/******************************************************************************
* Copyright (c) 2018 - 2022 Xilinx, Inc.  All rights reserved.
* SPDX-License-Identifier: MIT
******************************************************************************/

/*****************************************************************************/
/**
*
* @file rpu.h
*
* This file contains RPU register definitions used by PSM Firmware
*
* <pre>
* MODIFICATION HISTORY:
*
* Ver	Who		Date		Changes
* ---- ---- -------- ------------------------------
* 1.00  rv   07/17/2018 Initial release
*
* </pre>
*
* @note
*
******************************************************************************/

#ifndef XPSMFW_RPU_H_
#define XPSMFW_RPU_H_

#ifdef __cplusplus
extern "C" {
#endif

/**
 * RPU Base Address
 */
#define RPU_BASEADDR      0XEB580000U

/**
 * Register: RPU_RPU_GLBL_CNTL
 */
#define RPU_RPU_GLBL_CNTL    ( ( RPU_BASEADDR ) + 0X00000000U )

#define RPU_RPU_GLBL_CNTL_GIC_AXPROT_SHIFT   10U
#define RPU_RPU_GLBL_CNTL_GIC_AXPROT_WIDTH   1U
#define RPU_RPU_GLBL_CNTL_GIC_AXPROT_MASK    0X00000400U

#define RPU_RPU_GLBL_CNTL_TCM_CLK_CNTL_SHIFT   8U
#define RPU_RPU_GLBL_CNTL_TCM_CLK_CNTL_WIDTH   1U
#define RPU_RPU_GLBL_CNTL_TCM_CLK_CNTL_MASK    0X00000100U

#define RPU_RPU_GLBL_CNTL_TCM_WAIT_SHIFT   7U
#define RPU_RPU_GLBL_CNTL_TCM_WAIT_WIDTH   1U
#define RPU_RPU_GLBL_CNTL_TCM_WAIT_MASK    0X00000080U

#define RPU_RPU_GLBL_CNTL_TCM_COMB_SHIFT   6U
#define RPU_RPU_GLBL_CNTL_TCM_COMB_WIDTH   1U
#define RPU_RPU_GLBL_CNTL_TCM_COMB_MASK    0X00000040U

#define RPU_RPU_GLBL_CNTL_TEINIT_SHIFT   5U
#define RPU_RPU_GLBL_CNTL_TEINIT_WIDTH   1U
#define RPU_RPU_GLBL_CNTL_TEINIT_MASK    0X00000020U

#define RPU_RPU_GLBL_CNTL_SLCLAMP_SHIFT   4U
#define RPU_RPU_GLBL_CNTL_SLCLAMP_WIDTH   1U
#define RPU_RPU_GLBL_CNTL_SLCLAMP_MASK    0X00000010U

#define RPU_RPU_GLBL_CNTL_SLSPLIT_SHIFT   3U
#define RPU_RPU_GLBL_CNTL_SLSPLIT_WIDTH   1U
#define RPU_RPU_GLBL_CNTL_SLSPLIT_MASK    0X00000008U

#define RPU_RPU_GLBL_CNTL_DBGNOCLKSTOP_SHIFT   2U
#define RPU_RPU_GLBL_CNTL_DBGNOCLKSTOP_WIDTH   1U
#define RPU_RPU_GLBL_CNTL_DBGNOCLKSTOP_MASK    0X00000004U

#define RPU_RPU_GLBL_CNTL_CFGIE_SHIFT   1U
#define RPU_RPU_GLBL_CNTL_CFGIE_WIDTH   1U
#define RPU_RPU_GLBL_CNTL_CFGIE_MASK    0X00000002U

#define RPU_RPU_GLBL_CNTL_CFGEE_SHIFT   0U
#define RPU_RPU_GLBL_CNTL_CFGEE_WIDTH   1U
#define RPU_RPU_GLBL_CNTL_CFGEE_MASK    0X00000001U

/**
 * Register: RPU_RPU_GLBL_STATUS
 */
#define RPU_RPU_GLBL_STATUS    ( ( RPU_BASEADDR ) + 0X00000004U )

#define RPU_RPU_GLBL_STATUS_DBGNOPWRDWN_SHIFT   0U
#define RPU_RPU_GLBL_STATUS_DBGNOPWRDWN_WIDTH   1U
#define RPU_RPU_GLBL_STATUS_DBGNOPWRDWN_MASK    0X00000001U

/**
 * Register: RPU_RPU_0_PWRDWN
 */
#define RPU_RPU_0_PWRDWN    ( ( RPU_BASEADDR ) + 0X00000108U )

#define RPU_RPU_0_PWRDWN_EN_SHIFT   0U
#define RPU_RPU_0_PWRDWN_EN_WIDTH   1U
#define RPU_RPU_0_PWRDWN_EN_MASK    0X00000001U

/**
 * Register: RPU_RPU_1_PWRDWN
 */
#define RPU_RPU_1_PWRDWN    ( ( RPU_BASEADDR ) + 0X00000208U )

#define RPU_RPU_1_PWRDWN_EN_SHIFT   0U
#define RPU_RPU_1_PWRDWN_EN_WIDTH   1U
#define RPU_RPU_1_PWRDWN_EN_MASK    0X00000001U

/**
 * Register: RPU_ERR_INJ
 */
#define RPU_RPU_ERR_INJ     ( ( RPU_BASEADDR ) + 0X00000020U )

#define RPU_RPU_ERR_INJ_DCCMINP_SHIFT   0U
#define RPU_RPU_ERR_INJ_DCCMINP_WIDTH   8U
#define RPU_RPU_ERR_INJ_DCCMINP_MASK    0X000000FFU

#define RPU_RPU_ERR_INJ_DCCMINP2_SHIFT   8U
#define RPU_RPU_ERR_INJ_DCCMINP2_WIDTH   8U
#define RPU_RPU_ERR_INJ_DCCMINP2_MASK    0X0000FF00U

/* RPU config register */
#define RPU_RPU_0_CFG		( ( RPU_BASEADDR ) + 0x00000100U )
#define RPU_RPU_1_CFG		( ( RPU_BASEADDR ) + 0x00000200U )

#define RPU_HIVEC_ADDR		(0xFFFC0000U)
#define RPU_VINITHI_MASK	(0x4U)

#define LPD_SLCR_BASEADDR    0xEB410000U

#define LPD_SLCR_RPU_PCIL_A0_IDS    ( (LPD_SLCR_BASEADDR) + 0x0001000CU )
#define LPD_SLCR_RPU_PCIL_A0_IDS_PACTIVE1_MASK  0x00000001U
#define LPD_SLCR_RPU_PCIL_A0_PS    ( (LPD_SLCR_BASEADDR) + 0x00010084U )
#define LPD_SLCR_RPU_PCIL_A0_PS_PSTATE_MASK  0x00000001U
#define LPD_SLCR_RPU_PCIL_A0_PR    ( (LPD_SLCR_BASEADDR) + 0x00010080U )
#define LPD_SLCR_RPU_PCIL_A0_PR_PREQ_MASK  0x00000001U
#define LPD_SLCR_RPU_PCIL_A0_PA    ( (LPD_SLCR_BASEADDR) + 0x00010088U )
#define LPD_SLCR_RPU_PCIL_A0_PA_PACCEPT_MASK  0x00000100U

#define LPD_SLCR_RPU_PCIL_A1_IDS    ( (LPD_SLCR_BASEADDR) + 0x0001010CU )
#define LPD_SLCR_RPU_PCIL_A1_IDS_PACTIVE1_MASK  0x00000001U
#define LPD_SLCR_RPU_PCIL_A1_PS    ( (LPD_SLCR_BASEADDR) + 0x00010184U )
#define LPD_SLCR_RPU_PCIL_A1_PS_PSTATE_MASK  0x00000001U
#define LPD_SLCR_RPU_PCIL_A1_PR    ( (LPD_SLCR_BASEADDR) + 0x00010180U )
#define LPD_SLCR_RPU_PCIL_A1_PR_PREQ_MASK  0x00000001U
#define LPD_SLCR_RPU_PCIL_A1_PA    ( (LPD_SLCR_BASEADDR) + 0x00010188U )
#define LPD_SLCR_RPU_PCIL_A1_PA_PACCEPT_MASK  0x00000100U

#define LPD_SLCR_RPU_PCIL_B0_IDS    ( (LPD_SLCR_BASEADDR) + 0x0001100CU )
#define LPD_SLCR_RPU_PCIL_B0_IDS_PACTIVE1_MASK  0x00000001U
#define LPD_SLCR_RPU_PCIL_B0_PS    ( (LPD_SLCR_BASEADDR) + 0x00011084U )
#define LPD_SLCR_RPU_PCIL_B0_PS_PSTATE_MASK  0x00000001U
#define LPD_SLCR_RPU_PCIL_B0_PR    ( (LPD_SLCR_BASEADDR) + 0x00011080U )
#define LPD_SLCR_RPU_PCIL_B0_PR_PREQ_MASK  0x00000001U
#define LPD_SLCR_RPU_PCIL_B0_PA    ( (LPD_SLCR_BASEADDR) + 0x00011088U )
#define LPD_SLCR_RPU_PCIL_B0_PA_PACCEPT_MASK  0x00000100U

#define LPD_SLCR_RPU_PCIL_B1_IDS    ( (LPD_SLCR_BASEADDR) + 0x0001110CU )
#define LPD_SLCR_RPU_PCIL_B1_IDS_PACTIVE1_MASK  0x00000001U
#define LPD_SLCR_RPU_PCIL_B1_PS    ( (LPD_SLCR_BASEADDR) + 0x00011184U )
#define LPD_SLCR_RPU_PCIL_B1_PS_PSTATE_MASK  0x00000001U
#define LPD_SLCR_RPU_PCIL_B1_PR    ( (LPD_SLCR_BASEADDR) + 0x00011180U )
#define LPD_SLCR_RPU_PCIL_B1_PR_PREQ_MASK  0x00000001U
#define LPD_SLCR_RPU_PCIL_B1_PA    ( (LPD_SLCR_BASEADDR) + 0x00011188U )
#define LPD_SLCR_RPU_PCIL_B1_PA_PACCEPT_MASK  0x00000100U

#define RPU_RPU0_CORE0_CFG0		( ( RPU_BASEADDR ) + 0x00000000U )
#define RPU_RPU0_CORE1_CFG0		( ( RPU_BASEADDR ) + 0x00000100U )
#define RPU_RPU1_CORE0_CFG0		( ( RPU_BASEADDR ) + 0x00010000U )
#define RPU_RPU1_CORE1_CFG0		( ( RPU_BASEADDR ) + 0x00010100U )

#define RPU_CORE_CPUHALT_MASK (0x00000001U)

#ifdef __cplusplus
}
#endif

#endif /* XPSMFW_RPU_H_ */