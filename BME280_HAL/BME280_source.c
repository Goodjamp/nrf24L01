/********************************************************************************
  * @file    BME280_source.c
  * @author  Gerasimchuk A.
  * @version V1.0.0
  * @date    19-Octobre-2017
  * @brief
  */
#include "stdint.h"
#include "stdbool.h"

#include "BME280_source.h"
#include "BME280_user_interface.h"


// static function prototype
static BME280_STATUS updateRegister(uint8_t deviceAddres, uint8_t regAddres, uint8_t data, uint8_t dataMask);
static BME280_S32_t BME280_compensate_T_int32(BME280Handler *handler, BME280_S32_t adc_T);
static BME280_U32_t BME280_compensate_P_int64(BME280Handler *handler, BME280_S32_t adc_P);
static BME280_U32_t bme280_compensate_H_int32(BME280Handler *handler, BME280_S32_t adc_H);
//static bme280MesCallbackDef mesComlitCallback;


static const uint8_t adresListOfOversem[] = {
		[MES_VALUE_TEMPERATURE] = (uint8_t)BME280_REG_CTRL_MES,
		[MES_VALUE_HUMIDITY]    = (uint8_t)BME280_REG_CTRL_HUM,
		[MES_VALUE_PRESSURE]    = (uint8_t)BME280_REG_CTRL_MES,
};

static const uint8_t shiftListOfOversemVal[] = {
		[MES_VALUE_TEMPERATURE] = (uint8_t)TEMPERATURE_OVERSAMPLING_FIELD_SHIFT,
		[MES_VALUE_HUMIDITY]    = (uint8_t)HUMIDITY_OVERSAMPLING_FIELD_SHIFT,
		[MES_VALUE_PRESSURE]    = (uint8_t)PRESSURE_OVERSAMPLING_FIELD_SHIFT,
};

// t_fine carries fine temperature as global value
BME280_S32_t t_fine;

static BME280_STATUS updateRegister(uint8_t deviceAddres, uint8_t regAddres, uint8_t data, uint8_t dataMask){
	uint8_t bufferDataTx;
	//uint8_t bufferDataRx;
	// read reg
	if(  BMEReadData( deviceAddres, regAddres, &bufferDataTx, 1) )
	{
		return BME280_STATUS_COMUNICATION_ERROR;
	}
	//update reg
	bufferDataTx &= ~(dataMask);
	bufferDataTx |= data;
	//write reg
	if(  BMEWriteData( deviceAddres, regAddres, &bufferDataTx, 1) )
	{
		return BME280_STATUS_COMUNICATION_ERROR;
	}
	/*
	// read reg for compare writed data and data from sensor
	if(  BMEReadData( deviceAddres, regAddres, &bufferDataRx, 1) )
	{
		return BME280_STATUS_COMUNICATION_ERROR;
	}

	if(bufferDataTx != bufferDataRx)
	{
		return BME280_STATUS_COMUNICATION_ERROR;
	}
*/
	return BME280_STATUS_OK;
}


void BME280_setI2CAddress(BME280Handler *handler, uint8_t address){
	handler->selfAddress = address;
}


BME280_STATUS BME280_init(BME280Handler *handler){
	uint16_t tempE4, tempE5, tempE6, tempE7;    // &((uint8_t)&data)[0]
	// Read first area of calibration data
	if(  BMEReadData( handler->selfAddress, BME280_REG_CALIB00_23, (uint8_t*)&(handler->calibrationData), SIZE_OF_CALIBRATION_1) )
	//if(  BMEReadData( address, BME280_REG_CALIB00_23, (uint8_t*)&(&handler->calibrationData)[0], SIZE_OF_CALIBRATION_1) )
	{
		return BME280_STATUS_COMUNICATION_ERROR;
	}
	// Read second area of calibration data
	if(  BMEReadData( handler->selfAddress, BME280_REG_CALIB24, &((uint8_t*)(&handler->calibrationData))[SIZE_OF_CALIBRATION_1], SIZE_OF_CALIBRATION_2) )
	{
		return BME280_STATUS_COMUNICATION_ERROR;
	}
	// Read  thread area of calibration data
	if(  BMEReadData( handler->selfAddress, BME280_REG_CALIB25_31, &((uint8_t*)(&handler->calibrationData))[SIZE_OF_CALIBRATION_1 + SIZE_OF_CALIBRATION_2], SIZE_OF_CALIBRATION_3) )
	{
		return BME280_STATUS_COMUNICATION_ERROR;
	}
	tempE4 =((uint8_t*)(&handler->calibrationData))[28];
	tempE5 =((uint8_t*)(&handler->calibrationData))[29];
	tempE6 =((uint8_t*)(&handler->calibrationData))[30];
	tempE7 =((uint8_t*)(&handler->calibrationData))[31];
	handler->calibrationData.dig_H4_ = (int16_t)((tempE4<<4) | (tempE5 & 0b1111));
	handler->calibrationData.dig_H5_ = (int16_t)((tempE6<<4) | (tempE5>>4) );
	handler->calibrationData.dig_H6_ = (int8_t)tempE7;
	// update some calib data 29
	return handler->sensorStatus = BME280_STATUS_OK;
}


BME280_STATUS BME280_setValueMesState   (BME280Handler *handler, MES_VALUE_DEF mesValue, MES_STATE_DEF newMesState){
	if(!IS_BME_AVAILABLE(handler))
	{
		return handler->sensorStatus;
	}
	//update reg
	uint8_t overSample    = (MES_STATE_ENABLE == newMesState) ? (CTRL_MEAS_OSRS_OVERx1):(CTRL_MEAS_OSRS_SKIPED);
	uint8_t newFieldValue =  overSample << shiftListOfOversemVal[mesValue] ;
    return handler->sensorStatus = updateRegister(handler->selfAddress,
    		                                      adresListOfOversem[mesValue],
    		                                      newFieldValue,
    		                                      (uint8_t)(CTRL_MEAS_OSRS_FIELD_MASK << shiftListOfOversemVal[mesValue]));
}

BME280_STATUS BME280_setOverSample(BME280Handler *handler, MES_VALUE_DEF mesValue, OVERSEMPLE_DEF overSample){
	if(!IS_BME_AVAILABLE(handler))
	{
		return handler->sensorStatus;
	}
	//update reg
	uint8_t newFieldValue = overSample << shiftListOfOversemVal[mesValue];
    return handler->sensorStatus = updateRegister(handler->selfAddress,
    		                                      adresListOfOversem[mesValue],
    		                                      newFieldValue,
    		                                      (uint8_t)(CTRL_MEAS_OSRS_FIELD_MASK << shiftListOfOversemVal[mesValue]));
}


BME280_STATUS BME280_setFilterParameters(BME280Handler *handler, FILTER_DEF filterPar){
	if(!IS_BME_AVAILABLE(handler))
	{
		return handler->sensorStatus;
	}
	//update reg
	uint8_t newFieldValue = filterPar << FILTER_FIELD_SHIFT;
    return handler->sensorStatus = updateRegister(handler->selfAddress,
    											  BME280_REG_CONFIG,
    		                                      newFieldValue,
    		                                      (uint8_t)(FILTER_FIELD_MASK << FILTER_FIELD_SHIFT));
}


BME280_STATUS BME280_setMesDelay(BME280Handler *handler, MEASUREMENT_DELAY_DEF mesDelay){
	if(!IS_BME_AVAILABLE(handler))
	{
		return handler->sensorStatus;
	}
	//update reg
	uint8_t newFieldValue = mesDelay << CONFIG_T_SB_FIELD_SHIFT;
    return handler->sensorStatus = updateRegister(handler->selfAddress,
    											  BME280_REG_CONFIG,
    		                                      newFieldValue,
    		                                      (uint8_t)(CONFIG_T_SB_FIELD_MASK << CONFIG_T_SB_FIELD_SHIFT));
}


BME280_STATUS BME280_isOnLine(BME280Handler *handler, bool *onLine){
	uint8_t registerValue;
	if(!IS_BME_AVAILABLE(handler))
	{
		return handler->sensorStatus;
	}
	if( TRANSACION_STATUS_ERROR == BMEReadData(handler->selfAddress, BME280_REG_ID, &registerValue, 1)  )
	{
		return handler->sensorStatus = BME280_STATUS_COMUNICATION_ERROR;;
	}
	*onLine = (BME280_ID == registerValue) ? true : false;
	return handler->sensorStatus = BME280_STATUS_OK;
}


BME280_STATUS BME280_reset(BME280Handler *handler){
	uint8_t registerValue = BME280_RESET;
	if(!IS_BME_AVAILABLE(handler))
	{
		return handler->sensorStatus;
	}
	if( TRANSACION_STATUS_ERROR == BMEWriteData(handler->selfAddress, BME280_REG_RESET, &registerValue, 1) )
	{
		return handler->sensorStatus = BME280_STATUS_COMUNICATION_ERROR;
	}
	return handler->sensorStatus = BME280_STATUS_OK;
}


BME280_STATUS BME280_forcedMes(BME280Handler *handler, float *rezMesTemperature, float *rezMesPressure, float *rezMesHumidity){
	uint8_t dataRx[SIZE_OF_REZ_BYTES];
	int32_t rezADC;
	int32_t  rezMesTemperatureInt;    // Returns temperature in DegC, resolution is 0.01 DegC
	uint32_t rezMesHumidityUInt;      // Returns humidity in %RH as unsigned 32 bit integer in Q22.10 format
	uint32_t rezMesPressureUInt;      // Returns pressure in Pa as unsigned 32 bit integer in Q24.8 format
	if(!IS_BME_AVAILABLE(handler))
	{
		return handler->sensorStatus;
	}
	handler->sensorStatus = BME280_STATUS_BUSY;

	// Start measurement
	if( (handler->sensorStatus = updateRegister(handler->selfAddress,
			BME280_REG_CTRL_MES,
			CTRL_MEAS_MODE_FORCED,
            (uint8_t)(CTRL_MEAS_MODE_FIELD_MASK << CTRL_MEAS_MODE_FIELD_SHIFT))) != BME280_STATUS_OK )
	{
		return handler->sensorStatus = BME280_STATUS_COMUNICATION_ERROR;
	}

	// wait for measurement complete
	while(1){
		if ( BMEReadData( handler->selfAddress, BME280_REG_STATUS, dataRx, 1) == TRANSACION_STATUS_ERROR )
		{
			return handler->sensorStatus = BME280_STATUS_COMUNICATION_ERROR;
		}

		// wait for measurement and copy complete
		if( !(dataRx[0] & (STATUS_MEASURING | STATUS_IM_UPDATE)))
		{
			break;
		}
	}

	// Read results of measurement
	if( BMEReadData( handler->selfAddress, BME280_REG_PRES_MLSB, dataRx, sizeof(dataRx)) == TRANSACION_STATUS_ERROR )
	{
		return handler->sensorStatus = BME280_STATUS_COMUNICATION_ERROR;
	}

    //Compensation Temperature
	rezADC =  dataRx[TEMP_MSB]<<12 | dataRx[TEMP_LSB]<<4 | dataRx[TEMP_XLSB];
	rezMesTemperatureInt = BME280_compensate_T_int32(handler, rezADC);
	(*rezMesTemperature) = rezMesTemperatureInt * TEMPERATURE_CALC;

	//Compensation Pressure
	rezADC = dataRx[PRESS_MSB]<<12 | dataRx[PRESS_LSB]<<4 | dataRx[PRESS_XLSB];
	rezMesPressureUInt = BME280_compensate_P_int64(handler, rezADC);//rezMesPressureInt
	(*rezMesPressure) = rezMesPressureUInt/PRESSURE_CALC;

	//Compensation Humidity
	rezADC =  dataRx[HUM_MSB]<<8 | dataRx[HUM_LSB];
	rezMesHumidityUInt = bme280_compensate_H_int32(handler, rezADC);
	(*rezMesHumidity) = rezMesHumidityUInt/HUMIDITY_CALC;

	return handler->sensorStatus = BME280_STATUS_OK;
}


// Returns temperature in DegC, resolution is 0.01 DegC. Output value of “5123” equals 51.23 DegC.
static BME280_S32_t BME280_compensate_T_int32(BME280Handler *handler, BME280_S32_t adc_T)
{
	BME280_S32_t var1, var2, T;
	var1 = ((((adc_T>>3) - ((BME280_S32_t)dig_T1<<1))) * ((BME280_S32_t)dig_T2)) >> 11;
	var2 = (((((adc_T>>4) - ((BME280_S32_t)dig_T1)) * ((adc_T>>4) - ((BME280_S32_t)dig_T1))) >> 12) *\
			((BME280_S32_t)dig_T3)) >> 14;
	t_fine = var1 + var2;
	T = (t_fine * 5 + 128) >> 8;
	return T;
}


// Returns pressure in Pa as unsigned 32 bit integer in Q24.8 format (24 integer bits and 8 fractional bits).
// Output value of “24674867” represents 24674867/256 = 96386.2 Pa = 963.862 hPa
static BME280_U32_t BME280_compensate_P_int64(BME280Handler *handler, BME280_S32_t adc_P) {
	BME280_S64_t var1, var2, p;
	var1 = ((BME280_S64_t) t_fine) - 128000;
	var2 = var1 * var1 * (BME280_S64_t) dig_P6;
	var2 = var2 + ((var1 * (BME280_S64_t) dig_P5) << 17);
	var2 = var2 + (((BME280_S64_t) dig_P4) << 35);
	var1 = ((var1 * var1 * (BME280_S64_t) dig_P3) >> 8)
			+ ((var1 * (BME280_S64_t) dig_P2) << 12);
	var1 = (((((BME280_S64_t) 1) << 47) + var1)) * ((BME280_S64_t) dig_P1)
			>> 33;
	if (var1 == 0) {
		return 0; // avoid exception caused by division by zero
	}
	p = 1048576 - adc_P;
	p = (((p << 31) - var2) * 3125) / var1;
	var1 = (((BME280_S64_t) dig_P9) * (p >> 13) * (p >> 13)) >> 25;
	var2 = (((BME280_S64_t) dig_P8) * p) >> 19;
	p = ((p + var1 + var2) >> 8) + (((BME280_S64_t) dig_P7) << 4);
	return (BME280_U32_t) p;
}

// Returns humidity in %RH as unsigned 32 bit integer in Q22.10 format (22 integer and 10 fractional bits).
// Output value of “47445” represents 47445/1024 = 46.333 %RH
BME280_U32_t bme280_compensate_H_int32(BME280Handler *handler, BME280_S32_t adc_H){
BME280_S32_t v_x1_u32r;
v_x1_u32r = (t_fine - ((BME280_S32_t)76800));
v_x1_u32r = (((((adc_H << 14) - (((BME280_S32_t)dig_H4) << 20) - (((BME280_S32_t)dig_H5) * v_x1_u32r)) +\
            ((BME280_S32_t)16384)) >> 15) * (((((((v_x1_u32r * ((BME280_S32_t)dig_H6)) >> 10) * (((v_x1_u32r *\
	        ((BME280_S32_t)dig_H3)) >> 11) + ((BME280_S32_t)32768))) >> 10) + ((BME280_S32_t)2097152)) *\
	        ((BME280_S32_t)dig_H2) + 8192) >> 14));
	v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) * ((BME280_S32_t)dig_H1)) >> 4));
	v_x1_u32r = (v_x1_u32r < 0 ? 0 : v_x1_u32r);
	v_x1_u32r = (v_x1_u32r > 419430400 ? 419430400 : v_x1_u32r);
	return (BME280_U32_t)(v_x1_u32r>>12);
}

/*
// Returns humidity in %RH as unsigned 32 bit integer in Q22.10 format (22 integer and 10 fractional bits).
// Output value of “47445” represents 47445/1024 = 46.333 %RH
static BME280_U32_t bme280_compensate_H_int32(BME280Handler *handler, BME280_S32_t adc_H) {
	    double humidity;
		double humidity_min = 0.0;
		double humidity_max = 100.0;
		double var1;
		double var2;
		double var3;
		double var4;
		double var5;
		double var6;

		var1 = ((double)t_fine) - 76800.0;
		var2 = (((double)dig_H4) * 64.0 + (((double)dig_H5) / 16384.0) * var1);
		var3 = adc_H - var2;
		var4 = ((double)dig_H2) / 65536.0;
		var5 = (1.0 + (((double)dig_H3) / 67108864.0) * var1);
		var6 = 1.0 + (((double)dig_H6) / 67108864.0) * var1 * var5;
		var6 = var3 * var4 * (var5 * var6);
		humidity = var6 * (1.0 - ((double)dig_H1) * var6 / 524288.0);

		if (humidity > humidity_max)
			humidity = humidity_max;
		else if (humidity < humidity_min)
			humidity = humidity_min;

		return (uint)humidity;
}
*/
