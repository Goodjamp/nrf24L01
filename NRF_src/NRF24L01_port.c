/*
 * NRF24L01_port.c
 *
 *  Created on: Febuary 25, 2015
 *      Author: Gerasimchuk
 *      Versin: 1
 */

#include "stdint.h"

#include "NRF24L01user.h"
#include "NRF24L01.h"


//---------NRF24L01_read_reg-----------------
// function NRF24L01_read_reg - read NRF24L01+ register
// input arg:
//          - inNRF        - pointer on selected NRF24L01+
//          - address_reg - register address, ref. NRF24L01_REG_ADDRESS "enum"
//          - num         -  amount of write data
//          - pdata_write - pointer to data
// output arg:
//        0 - operation complete successfully
//       >0 - ref. NRF_ERROR reason
inline NRF_ERROR NRF24L01_read_reg(nrfHeader inNRF ,NRF24L01_REG_ADDRESS address_reg, uint8_t num, uint8_t *pdata_read){
	if(!CHECK_NRF_ADDRESS(address_reg)){return NRF_ERROR_ADDRESS_REG;};
	if(nrf24l01_spi_RX(inNRF,(R_REGISTER|address_reg), pdata_read, num)==STATE_BUSY)
	{ // if SPI in processing - return
		return NRF_BUSY;
	};
	return NRF_OK;
}

//---------NRF24L01_write_reg-----------------
// function NRF24L01_write_reg - write NRF24L01+ register
// input arg:
//          - inNRF        - pointer on selected NRF24L01+
//          - address_reg - register address, ref. NRF24L01_REG_ADDRESS "enum"
//          - num         - amount of write data
//          - pdata_write - pointer to data
// output arg:
//        0 - operation complete successfully
//       >0 - ref. NRF_ERROR reason
inline  NRF_ERROR NRF24L01_write_reg(nrfHeader inNRF ,NRF24L01_REG_ADDRESS address_reg, uint8_t num, uint8_t *pdata_write){
	if(!CHECK_NRF_ADDRESS(address_reg)){return NRF_ERROR_ADDRESS_REG;};
	if(nrf24l01_spi_TX(inNRF,(uint8_t)(W_REGISTER|address_reg), pdata_write, num)==STATE_BUSY)
	{ // if SPI in processing - return
		return NRF_BUSY;
	};
	return NRF_OK;
}


//---------NRF24L01_write_fifo_tx-----------------
// function NRF24L01_write_fifo_tx - write NRF24L01+ fifo Tx data
// input arg:
//          - inNRF        - pointer on selected NRF24L01+
//          - num         -  amount of write data
//          - pdata_write - pointer to data
// output arg:
//        0 - operation complete successfully
//       >0 - ref. NRF_ERROR reason
inline  NRF_ERROR NRF24L01_write_fifo_tx(nrfHeader inNRF , uint8_t num, uint8_t *pdata_write){
	if(nrf24l01_spi_TX(inNRF,(uint8_t)(W_TX_PAYLOAD), pdata_write, num)==STATE_BUSY)
	{ // if SPI in processing - return
		return NRF_BUSY;
	};
	return NRF_OK;
}


//---------NRF24L01_read_fifo_rx-----------------
// function NRF24L01_read_fifo_rx - read NRF24L01+ fifo Rx data
// input arg:
//          - inNRF        - pointer on selected NRF24L01+
//          - num         -  amount of read data
//          - pdata_write - pointer to data
// output arg:
//        0 - operation complete successfully
//       >0 - ref. NRF_ERROR reason
inline  NRF_ERROR NRF24L01_read_fifo_rx(nrfHeader inNRF , uint8_t num,uint8_t *pdata_read){
	if(nrf24l01_spi_RX(inNRF,(uint8_t)(R_RX_PAYLOAD), pdata_read, num)==STATE_BUSY)
	{ // if SPI in processing - return
		return NRF_BUSY;
	};
	return NRF_OK;
}


//---------NRF24L01_FLUSH_TX-----------------
// function NRF24L01_FLUSH_TX - flush Tx FIFO
// input arg:
//          - inNRF        - pointer on selected NRF24L01+
// arg out:
//        0 - operation complete successfully
//       >0 - ref. NRF_ERROR reason
inline  NRF_ERROR NRF24L01_FLUSH_TX(nrfHeader inNRF){
	if(nrf24l01_spi_TX(inNRF,(uint8_t)(FLUSH_TX), (uint8_t*)0, 0)==STATE_BUSY)
	{ // if SPI in processing - return
		return NRF_BUSY;
	};
	return NRF_OK;
}

//---------NRF24L01_FLUSH_RX-----------------
// function NRF24L01_FLUSH_RX - flush Rx FIFO
// input arg:
//          - inNRF        - pointer on selected NRF24L01+
// arg out:
//        0 - operation complete successfully
//       >0 - ref. NRF_ERROR reason
inline  NRF_ERROR NRF24L01_FLUSH_RX(nrfHeader inNRF){
	if(nrf24l01_spi_TX(inNRF,(uint8_t)(FLUSH_RX), (uint8_t*)0, 0)==STATE_BUSY)
	{ // if SPI in processing - return
		return NRF_BUSY;
	};
	return NRF_OK;
}



