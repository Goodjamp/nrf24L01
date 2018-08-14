/********************************************************************************
  * @file    BME280_source.h
  * @author  Gerasimchuk A.
  * @version V1.0.0
  * @date    19-Octobre-2017
  * @brief
  */

#ifndef BME_280_HAL_H_
#define BME_280_HAL_H_

#include "stdint.h"

#include "BME280_source.h"

// new type for calculation
typedef  int32_t BME280_S32_t;
typedef  uint32_t BME280_U32_t;
typedef  int64_t BME280_S64_t;
typedef  uint64_t BME280_U64_t;
// temperature calibration data definition
#define dig_T1   handler->calibrationData.dig_T1_
#define dig_T2   handler->calibrationData.dig_T2_
#define dig_T3   handler->calibrationData.dig_T3_
// Pressure calibration data definition
#define dig_P1   handler->calibrationData.dig_P1_
#define dig_P2   handler->calibrationData.dig_P2_
#define dig_P3   handler->calibrationData.dig_P3_
#define dig_P4   handler->calibrationData.dig_P4_
#define dig_P5   handler->calibrationData.dig_P5_
#define dig_P6   handler->calibrationData.dig_P6_
#define dig_P7   handler->calibrationData.dig_P7_
#define dig_P8   handler->calibrationData.dig_P8_
#define dig_P9   handler->calibrationData.dig_P9_
// Humidity calibration data definition
#define dig_H1   handler->calibrationData.dig_H1_
#define dig_H2   handler->calibrationData.dig_H2_
#define dig_H3   handler->calibrationData.dig_H3_
#define dig_H4   handler->calibrationData.dig_H4_
#define dig_H5   handler->calibrationData.dig_H5_
#define dig_H6   handler->calibrationData.dig_H6_

#define TEMPERATURE_CALC    0.01F
#define PRESSURE_CALC      256.0F
#define HUMIDITY_CALC     1024.0F

//register ID definition
#define BME280_ID      0x60
// acquisition
//register RESET definition
#define BME280_RESET   0xB6

typedef enum{
	BME280_REG_HUM_LSB    = 0xFE,
	BME280_REG_HUM_MSB    = 0xFD,
	BME280_REG_TEMP_XLSB  = 0xFC,
	BME280_REG_TEMP_LSB   = 0xFB,
	BME280_REG_TEMP_MLSB  = 0xFA,
	BME280_REG_PRES_XLSB  = 0xF9,
	BME280_REG_PRES_LSB   = 0xF8,
	BME280_REG_PRES_MLSB  = 0xF7,
	BME280_REG_CONFIG     = 0xF5,
	BME280_REG_CTRL_MES   = 0xF4,
	BME280_REG_STATUS     = 0xF3,
	BME280_REG_CTRL_HUM   = 0xF2,
	BME280_REG_RESET      = 0xE0,
	BME280_REG_ID         = 0xD0,

	BME280_REG_CALIB25_31 = 0xE1,
	BME280_REG_CALIB24    = 0xA1,
	BME280_REG_CALIB00_23 = 0x88,
}BME280_REG;



//oversempling value
typedef enum{
         CTRL_MEAS_OSRS_SKIPED  = (uint8_t)0x0,
         CTRL_MEAS_OSRS_OVERx1  = (uint8_t)0x1,
}CTRL_MEAS_OSRS_OVER;
#define IS_CTRL_MEAS_OSRS_OVER(X)    (X == OVERSEMPLE_DISABLE)|| \
									 (X == OVERSEMPLE_2 )|| \
                                     (X == OVERSEMPLE_4 )|| \
                                     (X == OVERSEMPLE_8 )|| \
                                     (X == OVERSEMPLE_16)

#define  CTRL_MEAS_OSRS_FIELD_MASK                    0x7
//---shit of oversampling field for measurement value
#define  HUMIDITY_OVERSAMPLING_FIELD_SHIFT            0x0
#define  PRESSURE_OVERSAMPLING_FIELD_SHIFT            0x2
#define  TEMPERATURE_OVERSAMPLING_FIELD_SHIFT         0x5



//---fields of register STATUS definition---
/*measuring - Automatically set to ‘1’ whenever a conversion is running
and back to ‘0’ when the results have been transferred to the data registers*/
#define STATUS_MEASURING         (uint8_t)0x8
/*im_update - Automatically set to ‘1’ when the NVM data are being
copied to image registers and back to ‘0’ when the copying is done*/
#define STATUS_IM_UPDATE         (uint8_t)0x1


/*-----------------fields of register CTRL_MEAS definition-----------------*/
/*mode - Controls the sensor mode of the device*/

typedef enum{
    CTRL_MEAS_MODE_SLEEP  = (uint8_t)0x0,
    CTRL_MEAS_MODE_FORCED = (uint8_t)0x1,
    CTRL_MEAS_MODE_NORMAL = (uint8_t)0x3
}CTRL_MEAS_MODE_DEF;
#define  CTRL_MEAS_MODE_FIELD_MASK            0x3
#define  CTRL_MEAS_MODE_FIELD_SHIFT           0x0


/*-----------------fields of register CONFIG definition-----------------*/
/*t_sb - Controls inactive duration tstandby in normal mode*/
#define  CONFIG_T_SB_FIELD_MASK               0x7
#define  CONFIG_T_SB_FIELD_SHIFT              0x5


/*filter - Controls the time constant of the IIR filter*/
#define  FILTER_FIELD_MASK                    0x7
#define  FILTER_FIELD_SHIFT                   0x2

// Result measurement position definition in rez measurements array
#define SIZE_OF_REZ_BYTES                    0x8
#define PRESS_MSB                            0x0
#define PRESS_LSB                            0x1
#define PRESS_XLSB                           0x2
#define TEMP_MSB                             0x3
#define TEMP_LSB                             0x4
#define TEMP_XLSB                            0x5
#define HUM_MSB                              0x6
#define HUM_LSB                              0x7


//Calibration registers divided on the  two part
#define SIZE_OF_CALIBRATION_1  24
#define SIZE_OF_CALIBRATION_2  1
#define SIZE_OF_CALIBRATION_3  7
#pragma pack(push,1)
typedef struct{
	uint16_t dig_T1_;
	int16_t  dig_T2_;
	int16_t  dig_T3_;
	uint16_t dig_P1_;
	int16_t  dig_P2_;
	int16_t  dig_P3_;
	int16_t  dig_P4_;
	int16_t  dig_P5_;
	int16_t  dig_P6_;
	int16_t  dig_P7_;
	int16_t  dig_P8_;
	int16_t  dig_P9_;
	uint8_t  dig_H1_;
	int16_t  dig_H2_;
	uint8_t  dig_H3_;
	int16_t  dig_H4_;
	int16_t  dig_H5_;
	int8_t   dig_H6_;
}BME280_CALIB_COEF_DEF;
#pragma pack(pop)


#define IS_BME_AVAILABLE(X) ((X->sensorStatus == BME280_STATUS_OK)||\
						     (X->sensorStatus == BME280_STATUS_SENSOR_ERROR)|| \
						     (X->sensorStatus == BME280_STATUS_COMUNICATION_ERROR))
#endif
