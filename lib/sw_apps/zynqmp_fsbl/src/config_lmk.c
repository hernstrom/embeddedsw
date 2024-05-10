#include "xiicps.h"
#include "xil_printf.h"
#include "xgpiops.h"
#include "xparameters.h"

#define IIC_DEVICE_ID XPAR_PSU_I2C_0_DEVICE_ID
#define IIC_SCLK_RATE 100000
#define MAX_SIZE 8

typedef u16 AddressType;

#define MAX_I2C_CONFIG_SIZE 58
const u8 i2c_config[MAX_I2C_CONFIG_SIZE][3] = {
		{0x76, 0x06, 0xFF}, // Set I/O Expander output
		{0x76, 0x07, 0xFC}, // Set I/O Expander output
		{0x57, 0x1D, 0x0C}, // 5a) Set oscctl1
		{0x57, 0x32, 0x5A}, // 5b) Set ipclksel
		{0x76, 0x03, 0x03}, //5c) Configure I/O Expander (not on the lmk03328)
		{0x57, 0x38, 0x1E}, // 5d1) PLL0
		{0x57, 0x39, 0x08}, //
		{0x57, 0x75, 0x00}, //
		{0x57, 0x42, 0x0C}, //
		{0x57, 0x78, 0x00}, //
		{0x57, 0x47, 0x1F}, // 5d2) PLL1
		{0x57, 0x48, 0x08}, //
		{0x57, 0x83, 0x00}, //
		{0x57, 0x51, 0x0C}, //
		{0x57, 0x86, 0x00}, //
		{0x57, 0x34, 0x00}, // 5e) PLL0 divider, lmk03328_reg_pll1_rdiv
		{0x57, 0x35, 0x00}, // lmk03328_reg_pll1_mdiv
		{0x57, 0x3A, 0x00}, // lmk03328_reg_pll1_ndiv_by1
		{0x57, 0x3B, 0x32}, // lmk03328_reg_pll1_ndiv_by0
		{0x57, 0x38, 0x06}, // lmk03328_reg_pll1_ctrl0
		{0x57, 0x3C, 0x00}, // lmk03328_reg_pll1_fracnum_by2
		{0x57, 0x3D, 0x00}, // lmk03328_reg_pll1_fracnum_by1
		{0x57, 0x3E, 0x00}, // lmk03328_reg_pll1_fracnum_by0
		{0x57, 0x3F, 0x00}, // lmk03328_reg_pll1_fracden_by2
		{0x57, 0x40, 0x00}, // lmk03328_reg_pll1_fracden_by1
		{0x57, 0x41, 0x01}, // lmk03328_reg_pll1_fracden_by0
		{0x57, 0x36, 0x00}, // 5e) PLL1 divider, lmk03328_reg_pll2_rdiv
		{0x57, 0x37, 0x00}, // lmk03328_reg_pll2_mdiv
		{0x57, 0x49, 0x00}, // lmk03328_reg_pll2_ndiv_by1
		{0x57, 0x4A, 0x32}, // lmk03328_reg_pll2_ndiv_by0
		{0x57, 0x47, 0x07}, // lmk03328_reg_pll2_ctrl0
		{0x57, 0x4B, 0x00}, // lmk03328_reg_pll2_fracnum_by2
		{0x57, 0x4C, 0x00}, // lmk03328_reg_pll2_fracnum_by1
		{0x57, 0x4D, 0x00}, // lmk03328_reg_pll2_fracnum_by0
		{0x57, 0x4E, 0x00}, // lmk03328_reg_pll2_fracden_by2
		{0x57, 0x4F, 0x00}, // lmk03328_reg_pll2_fracden_by1
		{0x57, 0x50, 0x01}, // lmk03328_reg_pll2_fracden_by0
		{0x57, 0x43, 0x04}, // 5f) write the loop filter, lmk03328_reg_pll1_lf_r2
		{0x57, 0x44, 0x00}, // lmk03328_reg_pll1_lf_c1
		{0x57, 0x45, 0x01}, // lmk03328_reg_pll1_lf_r3
		{0x57, 0x46, 0x01}, // lmk03328_reg_pll1_lf_c3
		{0x57, 0x52, 0x04}, // lmk03328_reg_pll2_lf_r2
		{0x57, 0x53, 0x00}, // lmk03328_reg_pll2_lf_c1
		{0x57, 0x54, 0x01}, // lmk03328_reg_pll2_lf_r3
		{0x57, 0x55, 0x01}, // lmk03328_reg_pll2_lf_c3
		{0x57, 0x76, 0x07}, // lmk03328_reg_pll1_ctrl3
		{0x57, 0x84, 0x07}, // lmk03328_reg_pll2_ctrl3
		{0x57, 0x26, 0x0F}, // 5g) configure PLL output, lmk03328_reg_outdiv_4
		{0x57, 0x25, 0x10}, // lmk03328_reg_outctl_4
		{0x57, 0x2A, 0x0F}, // lmk03328_reg_outdiv_6
		{0x57, 0x29, 0x10}, // lmk03328_reg_outctl_6
		{0x57, 0x2C, 0x0F}, // lmk03328_reg_outdiv_7
		{0x57, 0x2B, 0x10}, // lmk03328_reg_outctl_7
		{0x57, 0x1E, 0x0B}, // 5h) pwdn, lmk03328_reg_pwdn
		{0x57, 0x0C, 0x59}, // 5i) software reset by assert and deassert RESETN_SW bit
		{0x57, 0x0C, 0xD9}, // 5i) deassert RESETN_SW bit
		{0x57, 0x1B, 0x28}, // 5j) Set PLL status output, lmk03328_reg_stat0_int
		{0x57, 0x1C, 0x58}, // 5j) lmk03328_reg_stat1_int
};

static s32 IicPsConfig(u16 DeviceId, XIicPs* IicInstance)
{
	u32 Status;
	XIicPs_Config *ConfigPtr;	/* Pointer to configuration data */

	/*
	 * Initialize the IIC driver so that it is ready to use.
	 */
	ConfigPtr = XIicPs_LookupConfig(DeviceId);

	if (ConfigPtr == NULL) {
		return XST_FAILURE;
	}

	Status = XIicPs_CfgInitialize(IicInstance, ConfigPtr, ConfigPtr->BaseAddress);
	if (Status != XST_SUCCESS) {
		return XST_FAILURE;
	}

	/*
	 * Set the IIC serial clock rate.
	 */
	XIicPs_SetSClk(IicInstance, IIC_SCLK_RATE);
	return XST_SUCCESS;
}

static void Gpio_SetDirectionPinOutput(u32 Bank, u32 PinNumber)
{
	u32 DirModeReg = Xil_In32(XPAR_PSU_GPIO_0_BASEADDR +
				              Bank * XGPIOPS_REG_MASK_OFFSET +
				              XGPIOPS_DIRM_OFFSET);

 	DirModeReg |= (1 << PinNumber);

	Xil_Out32(XPAR_PSU_GPIO_0_BASEADDR +
			  Bank * XGPIOPS_REG_MASK_OFFSET +
			  XGPIOPS_DIRM_OFFSET, 
             DirModeReg);
}

static void Gpio_SetOutputEnablePin(u32 Bank, u32 PinNumber)
{
	u32 OpEnableReg = Xil_In32(XPAR_PSU_GPIO_0_BASEADDR +
							   Bank * XGPIOPS_REG_MASK_OFFSET +
							   XGPIOPS_OUTEN_OFFSET);
	OpEnableReg |= (1 << PinNumber);
	Xil_Out32(XPAR_PSU_GPIO_0_BASEADDR +
			  Bank * XGPIOPS_REG_MASK_OFFSET +
			  XGPIOPS_OUTEN_OFFSET, OpEnableReg);
}

static void Gpio_WritePin(u32 Bank, u32 PinNumber, u32 OutputValue)
{
	/*
	 * Get the 32 bit value to be written to the Mask/Data register where
	 * the upper 16 bits is the mask and lower 16 bits is the data.
	 */

	OutputValue &= 0x01U;
	u32 RegisterValue = ~(1U << (PinNumber + 16U)) & ((OutputValue << PinNumber) | 0xFFFF0000U);
	Xil_Out32(XPAR_PSU_GPIO_0_BASEADDR +
			  Bank * XGPIOPS_DATA_MASK_OFFSET +
			  XGPIOPS_DATA_LSW_OFFSET, RegisterValue);
}

u32 Configure_LMK(void)
{
	u32 Status;
	XIicPs IicInstance;
	u8 IicWriteBuffer[sizeof(AddressType)];

	// Initialize all GPIO lines
	const int NUM_BANKS = 6;
	for (int i = 0; i < NUM_BANKS; ++i) {
		Xil_Out32(XPAR_PSU_GPIO_0_BASEADDR + ((u32)(i) * XGPIOPS_REG_MASK_OFFSET) + XGPIOPS_INTDIS_OFFSET, 
				 0xFFFFFFFFU);
	}

	////////////////////////////////////////////
	// GPIO33 maps to GPIO Bank 1, Pin Number 7
	////////////////////////////////////////////

	// XGpioPs_SetDirectionPin(&Gpio, 33, 1);
	Gpio_SetDirectionPinOutput(1, 7);

	// XGpioPs_SetOutputEnablePin(&Gpio, 33, 1);
	Gpio_SetOutputEnablePin(1, 7);

	// XGpioPs_WritePin(&Gpio, 33, 0x0);
	Gpio_WritePin(1, 7, 0);

	///////////////////////////////////////////
	// GPIO82 maps to GPIO Bank 3, Pin Number 4
	///////////////////////////////////////////

	// XGpioPs_SetDirectionPin(&Gpio, 82, 1);
	Gpio_SetDirectionPinOutput(3, 4);

	// XGpioPs_SetOutputEnablePin(&Gpio, 82, 1);
	Gpio_SetOutputEnablePin(3, 4);

	// XGpioPs_WritePin(&Gpio, 82, 0x1);
	Gpio_WritePin(3, 4, 1);

	Status = IicPsConfig(IIC_DEVICE_ID, &IicInstance);
	if (Status != XST_SUCCESS) {
		return XST_FAILURE;
	}

	for (int i = 0; i < MAX_I2C_CONFIG_SIZE; i++ ) {
		/*
		 * Wait until bus is idle to start another transfer.
		 */
		while (XIicPs_BusIsBusy(&IicInstance));

		/*
		 * Send the Data.
		 */
		IicWriteBuffer[0] = (u8) i2c_config[i][1];
		IicWriteBuffer[1] = (u8) i2c_config[i][2];

		Status = XIicPs_MasterSendPolled(&IicInstance, IicWriteBuffer, 2, i2c_config[i][0]);
		if (Status != XST_SUCCESS) {
			return XST_FAILURE;
		}
	}

	/*
	 * Wait until bus is idle to before returning.
	 */
	while (XIicPs_BusIsBusy(&IicInstance));

	return XST_SUCCESS;
}