/********************************************************************************
  * @file    BME280_user_interfase.h
  * @author  Gerasimchuk A.
  * @version V1.0.0
  * @date    27-Octobre-2017
  * @brief
  */



#ifndef BME280_USER_INTERFASE_H_
#define BME280_USER_INTERFASE_H_

#include "stdint.h"
#include "stdbool.h"

#include "BME280_source.h"

#define BME280_ADDRESS_LOW   (uint8_t)0x76
#define BME280_ADDRESS_HIGHT (uint8_t)0x77


// list of measurement value
typedef enum{
	MES_VALUE_TEMPERATURE  = 0,
	MES_VALUE_PRESSURE,
	MES_VALUE_HUMIDITY,
}MES_VALUE_DEF;

typedef enum{
	MES_STATE_ENABLE,
	MES_STATE_DISABLE,
}MES_STATE_DEF;

typedef enum{
	MES_MODE_CONTINUOUS  = 0,
	MES_MODE_SIMLE,
}MES_MODE_DEF;

typedef enum{
	OVERSEMPLE_DISABLE = 1,
	OVERSEMPLE_2,
	OVERSEMPLE_4,
	OVERSEMPLE_8,
	OVERSEMPLE_16
}OVERSEMPLE_DEF;

typedef enum{
	FILTER_DISABLE  = 0,
	FILTER_ORDER_2,
	FILTER_ORDER_4,
	FILTER_ORDER_8,
	FILTER_ORDER_16
}FILTER_DEF;

typedef enum{
	MEASUREMENT_DELAY_0_5ms = 0,
	MEASUREMENT_DELAY_10_0ms,
	MEASUREMENT_DELAY_20_0ms,
	MEASUREMENT_DELAY_65_5ms,
	MEASUREMENT_DELAY_125_0ms,
	MEASUREMENT_DELAY_250_0ms,
	MEASUREMENT_DELAY_500_0ms,
	MEASUREMENT_DELAY_1000_0ms,
}MEASUREMENT_DELAY_DEF;

typedef enum{
	BME280_STATUS_OK,                     // SENSOR READY FOR OPERATION
	BME280_STATUS_SENSOR_ERROR,           // SENSOR DATA ERROR
	BME280_STATUS_COMUNICATION_ERROR,     // INTARFACE CONUMICATION ERROR
	BME280_STATUS_BUSY                    // SENSOR IN MEASUREMEN PROCES
}BME280_STATUS;

typedef void(*bme280MesCallbackDef)(BME280_STATUS rezMesStatus,float rezMesTemperature,float rezMesPressure, float rezMesHumidity);

typedef struct{
	uint8_t               selfAddress;
	BME280_CALIB_COEF_DEF calibrationData;
	BME280_STATUS         sensorStatus;
	bme280MesCallbackDef  mesCallback;
}BME280Handler;

//BME280 configuration functions
void          BME280_setI2CAddress      (BME280Handler *handler, uint8_t address);
BME280_STATUS BME280_init               (BME280Handler *handler);
BME280_STATUS BME280_setValueMesState   (BME280Handler *handler, MES_VALUE_DEF mesValue, MES_STATE_DEF newMesState);
BME280_STATUS BME280_setOverSample      (BME280Handler *handler, MES_VALUE_DEF mesValue, OVERSEMPLE_DEF overSample);
BME280_STATUS BME280_setFilterParameters(BME280Handler *handler, FILTER_DEF filterPar);
BME280_STATUS BME280_setMesDelay        (BME280Handler *handler, MEASUREMENT_DELAY_DEF mesDelay);
BME280_STATUS BME280_setMesCallBack     (BME280Handler *handler, bme280MesCallbackDef mesCallbak);
BME280_STATUS BME280_reset              (BME280Handler *handler);
BME280_STATUS BME280_isOnLine        (BME280Handler *handler, bool *onLine);

// Measurement control functions
BME280_STATUS BME280_forcedMes          (BME280Handler *handler, float *rezMesTemperature, float *rezMesPressure, float *rezMesHumidity);
BME280_STATUS BME280_startContiniousMes (BME280Handler *handler, MES_STATE_DEF newMesState);

// Information functions
BME280_STATUS BME280_getStatus          (BME280Handler *handler);

//-------------------------user implementation  functions----------------------
typedef enum{
	TRANSACION_STATUS_OK,
	TRANSACION_STATUS_ERROR
}TRANSACION_STATUS;

TRANSACION_STATUS BMEReadData (uint8_t sensorAddress, uint8_t sensorReagister, uint8_t *data, uint8_t numData);
TRANSACION_STATUS BMEWriteData(uint8_t sensorAddress, uint8_t sensorReagister, uint8_t *data, uint8_t numData);
uint32_t  sensorGetTime(void);


#endif
