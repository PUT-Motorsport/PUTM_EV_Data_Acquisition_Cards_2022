/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __ISM330DHCXSensor_H__
#define __ISM330DHCXSensor_H__

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"
#include "ism330dhcx_reg.h"
#include "main.h"
extern SPI_HandleTypeDef hspi1;

/* TypeDef -------------------------------------------------------------------*/

//Status Type Def
typedef enum {
	ISM330DHCX_OK = 0, ISM330DHCX_ERROR = -1
} ISM330DHCXStatusTypeDef;

// PIN Sensors
typedef enum {
	ISM330DHCX_INT1_PIN, ISM330DHCX_INT2_PIN,
} ISM330DHCX_SensorIntPin_t;

//Event Status
typedef struct {
	unsigned int FreeFallStatus :1;
	unsigned int TapStatus :1;
	unsigned int DoubleTapStatus :1;
	unsigned int WakeUpStatus :1;
	unsigned int StepStatus :1;
	unsigned int TiltStatus :1;
	unsigned int D6DOrientationStatus :1;
	unsigned int SleepStatus :1;
} ISM330DHCX_Event_Status_t;

/* Define --------------------------------------------------------------------*/

#define ISM330DHCX_ACC_SENSITIVITY_FS_2G   0.061f
#define ISM330DHCX_ACC_SENSITIVITY_FS_4G   0.122f
#define ISM330DHCX_ACC_SENSITIVITY_FS_8G   0.244f
#define ISM330DHCX_ACC_SENSITIVITY_FS_16G  0.488f

#define ISM330DHCX_GYRO_SENSITIVITY_FS_125DPS    4.375f
#define ISM330DHCX_GYRO_SENSITIVITY_FS_250DPS    8.750f
#define ISM330DHCX_GYRO_SENSITIVITY_FS_500DPS   17.500f
#define ISM330DHCX_GYRO_SENSITIVITY_FS_1000DPS  35.000f
#define ISM330DHCX_GYRO_SENSITIVITY_FS_2000DPS  70.000f
#define ISM330DHCX_GYRO_SENSITIVITY_FS_4000DPS 140.000f

/**
 * Abstract class of an ISM330DHCX.
 */
class ISM330DHCXSensor {
public:
	/* ISM330DHCX Constructor -----------------------------------------------------*/

	/*default SA0=1,to set SA0=0  set the address to IIS2DLPC_I2C_ADD_L*/
	ISM330DHCXSensor();

	/* ISM330DHCX Function -------------------------------------------------------*/
	ISM330DHCXStatusTypeDef begin();
	ISM330DHCXStatusTypeDef end();
	ISM330DHCXStatusTypeDef ReadID(uint8_t *Id);
	ISM330DHCXStatusTypeDef ACC_Enable();
	ISM330DHCXStatusTypeDef ACC_Disable();
	ISM330DHCXStatusTypeDef ACC_GetSensitivity(float *Sensitivity);
	ISM330DHCXStatusTypeDef ACC_GetOutputDataRate(float *Odr);
	ISM330DHCXStatusTypeDef ACC_SetOutputDataRate(float Odr);
	ISM330DHCXStatusTypeDef ACC_GetFullScale(int32_t *FullScale);
	ISM330DHCXStatusTypeDef ACC_SetFullScale(int32_t FullScale);
	ISM330DHCXStatusTypeDef ACC_GetAxesRaw(int16_t *Value);
	ISM330DHCXStatusTypeDef ACC_GetAxes(int32_t *Acceleration);

	ISM330DHCXStatusTypeDef GYRO_Enable();
	ISM330DHCXStatusTypeDef GYRO_Disable();
	ISM330DHCXStatusTypeDef GYRO_GetSensitivity(float *Sensitivity);
	ISM330DHCXStatusTypeDef GYRO_GetOutputDataRate(float *Odr);
	ISM330DHCXStatusTypeDef GYRO_SetOutputDataRate(float Odr);
	ISM330DHCXStatusTypeDef GYRO_GetFullScale(int32_t *FullScale);
	ISM330DHCXStatusTypeDef GYRO_SetFullScale(int32_t FullScale);
	ISM330DHCXStatusTypeDef GYRO_GetAxesRaw(int16_t *Value);
	ISM330DHCXStatusTypeDef GYRO_GetAxes(int32_t *AngularRate);

	ISM330DHCXStatusTypeDef ReadReg(uint8_t reg, uint8_t *Data);
	ISM330DHCXStatusTypeDef WriteReg(uint8_t reg, uint8_t Data);
	ISM330DHCXStatusTypeDef ACC_GetEventStatus(
			ISM330DHCX_Event_Status_t *Status);

	ISM330DHCXStatusTypeDef ACC_Set_SelfTest(uint8_t Status);
	ISM330DHCXStatusTypeDef ACC_Get_SelfTest(uint8_t *Status);
	ISM330DHCXStatusTypeDef ACC_Get_DRDY_Status(uint8_t *Status);
	ISM330DHCXStatusTypeDef ACC_Enable_HP_Filter(
			ism330dhcx_hp_slope_xl_en_t CutOff);

	ISM330DHCXStatusTypeDef GYRO_Set_SelfTest(uint8_t Status);
	ISM330DHCXStatusTypeDef GYRO_Get_SelfTest(uint8_t *Status);
	ISM330DHCXStatusTypeDef GYRO_Get_DRDY_Status(uint8_t *Status);
	ISM330DHCXStatusTypeDef GYRO_Enable_HP_Filter(ism330dhcx_hpm_g_t CutOff);

	ISM330DHCXStatusTypeDef SetInterruptLatch(uint8_t Status);
	ISM330DHCXStatusTypeDef ISM330DHCX_Set_INT1_Drdy(uint8_t Status);
	ISM330DHCXStatusTypeDef ISM330DHCX_Set_INT2_Drdy(uint8_t Status);
	ISM330DHCXStatusTypeDef ISM330DHCX_Set_Drdy_Mode(uint8_t Status);

	ISM330DHCXStatusTypeDef ACC_EnableFreeFallDetection(
			ISM330DHCX_SensorIntPin_t IntPin);
	ISM330DHCXStatusTypeDef ACC_DisableFreeFallDetection();
	ISM330DHCXStatusTypeDef ACC_SetFreeFallThreshold(uint8_t Threshold);
	ISM330DHCXStatusTypeDef ACC_SetFreeFallDuration(uint8_t Duration);

	ISM330DHCXStatusTypeDef ACC_EnableWakeUpDetection(
			ISM330DHCX_SensorIntPin_t IntPin);
	ISM330DHCXStatusTypeDef ACC_DisableWakeUpDetection();
	ISM330DHCXStatusTypeDef ACC_SetWakeUpThreshold(uint8_t Threshold);
	ISM330DHCXStatusTypeDef ACC_SetWakeUpDuration(uint8_t Duration);

	ISM330DHCXStatusTypeDef ACC_EnableSingleTapDetection(
			ISM330DHCX_SensorIntPin_t IntPin);
	ISM330DHCXStatusTypeDef ACC_DisableSingleTapDetection();
	ISM330DHCXStatusTypeDef ACC_EnableDoubleTapDetection(
			ISM330DHCX_SensorIntPin_t IntPin);
	ISM330DHCXStatusTypeDef ACC_DisableDoubleTapDetection();
	ISM330DHCXStatusTypeDef ACC_SetTapThreshold(uint8_t Threshold);
	ISM330DHCXStatusTypeDef ACC_SetTapShockTime(uint8_t Time);
	ISM330DHCXStatusTypeDef ACC_SetTapQuietTime(uint8_t Time);
	ISM330DHCXStatusTypeDef ACC_SetTapDurationTime(uint8_t Time);

	ISM330DHCXStatusTypeDef ACC_Enable6DOrientation(
			ISM330DHCX_SensorIntPin_t IntPin);
	ISM330DHCXStatusTypeDef ACC_Disable6DOrientation();
	ISM330DHCXStatusTypeDef ACC_Set6DOrientationThreshold(uint8_t Threshold);
	ISM330DHCXStatusTypeDef ACC_Get6DOrientationXL(uint8_t *XLow);
	ISM330DHCXStatusTypeDef ACC_Get6DOrientationXH(uint8_t *XHigh);
	ISM330DHCXStatusTypeDef ACC_Get6DOrientationYL(uint8_t *YLow);
	ISM330DHCXStatusTypeDef ACC_Get6DOrientationYH(uint8_t *YHigh);
	ISM330DHCXStatusTypeDef ACC_Get6DOrientationZL(uint8_t *ZLow);
	ISM330DHCXStatusTypeDef ACC_Get6DOrientationZH(uint8_t *ZHigh);

	ISM330DHCXStatusTypeDef FIFO_Get_Num_Samples(uint16_t *NumSamples);
	ISM330DHCXStatusTypeDef FIFO_Get_Full_Status(uint8_t *Status);
	ISM330DHCXStatusTypeDef FIFO_ACC_Set_BDR(float Bdr);
	ISM330DHCXStatusTypeDef FIFO_GYRO_Set_BDR(float Bdr);
	ISM330DHCXStatusTypeDef FIFO_Set_INT1_FIFO_Full(uint8_t Status);
	ISM330DHCXStatusTypeDef FIFO_Set_INT2_FIFO_Full(uint8_t Status);
	ISM330DHCXStatusTypeDef FIFO_Set_Watermark_Level(uint16_t Watermark);
	ISM330DHCXStatusTypeDef FIFO_Set_Stop_On_Fth(uint8_t Status);
	ISM330DHCXStatusTypeDef FIFO_Set_Mode(uint8_t Mode);
	ISM330DHCXStatusTypeDef FIFO_Get_Tag(uint8_t *Tag);
	ISM330DHCXStatusTypeDef FIFO_Get_Data(uint8_t *Data);
	ISM330DHCXStatusTypeDef FIFO_ACC_Get_Axes(int32_t *Acceleration);
	ISM330DHCXStatusTypeDef FIFO_GYRO_Get_Axes(int32_t *AngularVelocity);

	ISM330DHCXStatusTypeDef ACC_Enable_DRDY_On_INT1();
	ISM330DHCXStatusTypeDef ACC_Disable_DRDY_On_INT1();

	ISM330DHCXStatusTypeDef GYRO_Enable_DRDY_On_INT2();
	ISM330DHCXStatusTypeDef GYRO_Disable_DRDY_On_INT2();

	ISM330DHCXStatusTypeDef DRDY_Set_Mode(uint8_t Mode);

	/**
	 * @brief Utility function to read data.
	 * @param  pBuffer: pointer to data to be read.
	 * @param  RegisterAddr: specifies internal address register to be read.
	 * @param  NumByteToRead: number of bytes to be read.
	 * @retval 0 if ok, an error code otherwise.
	 */
	uint8_t IO_Read(uint8_t *pBuffer, uint8_t RegisterAddr,
			uint16_t nBytesToRead) {
		if (&hspi1 != NULL) {
//
//			dev_spi->beginTransaction(
//					SPISettings(spi_speed, MSBFIRST, SPI_MODE3));
			HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);

//digitalWrite(cs_pin, LOW);
			uint8_t msg = RegisterAddr | 0x80;
			/* Write Reg Address */
			HAL_SPI_Transmit(&hspi1, &msg, 1, 100);

//			dev_spi->transfer(RegisterAddr | 0x80);
			/* Read the data */
//			for (uint16_t i = 0; i < NumByteToRead; i++) {
//				*(pBuffer + i) = dev_spi->transfer(0x00);
//			}
			HAL_SPI_Receive(&hspi1, pBuffer, nBytesToRead, 100);

			HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);

			return 0;
		}

		return 1;
	}

	/**
	 * @brief Utility function to write data.
	 * @param  pBuffer: pointer to data to be written.
	 * @param  RegisterAddr: specifies internal address register to be written.
	 * @param  NumByteToWrite: number of bytes to write.
	 * @retval 0 if ok, an error code otherwise.
	 */
	uint8_t IO_Write(uint8_t *pBuffer, uint8_t RegisterAddr,
			uint16_t nBytesToWrite) {
		if (&hspi1!=NULL) {
			HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);

			//digitalWrite(cs_pin, LOW);

			/* Write Reg Address */
			uint8_t data[2];
			data[0] = RegisterAddr;  // multibyte write
			data[1] = *pBuffer;

//			HAL_SPI_Transmit(&hspi1, &RegisterAddr, 1, 100);
			HAL_SPI_Transmit(&hspi1, data, 2, 100);

//			HAL_SPI_Receive(&hspi1, pBuffer, nBytesToWrite, 100);

			HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);

			return 0;
		}

		return 1;
	}


private:
	ISM330DHCXStatusTypeDef Init();
	/*Connection*/
//	TwoWire *dev_i2c;
//	SPIClass *dev_spi;
//	SPI_HandleTypeDef hspi;
	/*Configuration*/
	uint8_t address;
	int cs_pin;
	uint32_t spi_speed;

	uint8_t acc_is_enabled;
	uint8_t gyro_is_enabled;
	ism330dhcx_ctx_t reg_ctx;
};

#ifdef __cplusplus
extern "C" {
#endif
int32_t ISM330DHCX_io_write(void *handle, uint8_t WriteAddr, uint8_t *pBuffer,
		uint16_t nBytesToWrite);
int32_t ISM330DHCX_io_read(void *handle, uint8_t ReadAddr, uint8_t *pBuffer,
		uint16_t nBytesToRead);
#ifdef __cplusplus
}
#endif

#endif
