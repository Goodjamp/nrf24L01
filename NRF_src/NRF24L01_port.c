/*
 * NRF24L01_port.c
 *
 *  Created on: Febuary 25, 2015
 *      Author: Gerasimchuk
 *      Versin: 1
 */

#include "NRF24L01.h"
#include "spi_nrf24l01.h"

//---------NRF24L01_read_reg-----------------
// function NRF24L01_read_reg - read NRF24L01+ register
// input arg:
//          - pNRF        - pointer on selected NRF24L01+
//          - address_reg - register address, ref. NRF24L01_REG_ADDRESS "enum"
//          - num         -  amount of write data
//          - pdata_write - pointer to data
// output arg:
//        0 - operation complete successfully
//       >0 - ref. NRF_ERROR reason
NRF_ERROR NRF24L01_read_reg(S_nrf_config *const pNRF ,NRF24L01_REG_ADDRESS address_reg, u8 num, u8 *pdata_read){
	if(!CHECK_NRF_ADDRESS(address_reg)){return NRF_ERROR_ADDRESS_REG;};
	if(spi_nrf24l01_RX(pNRF,(R_REGISTER|address_reg), pdata_read, num)==BUSY_STATE){ // if SPI in processing - return
		return NRF_ERROR_BUSY;
	};
	return NRF_ERROR_OK;
}

//---------NRF24L01_write_reg-----------------
// function NRF24L01_write_reg - write NRF24L01+ register
// input arg:
//          - pNRF        - pointer on selected NRF24L01+
//          - address_reg - register address, ref. NRF24L01_REG_ADDRESS "enum"
//          - num         - amount of write data
//          - pdata_write - pointer to data
// output arg:
//        0 - operation complete successfully
//       >0 - ref. NRF_ERROR reason
NRF_ERROR NRF24L01_write_reg(S_nrf_config *const pNRF ,NRF24L01_REG_ADDRESS address_reg, u8 num,const u8 *pdata_write){
	if(!CHECK_NRF_ADDRESS(address_reg)){return NRF_ERROR_ADDRESS_REG;};
	if(spi_nrf24l01_TX(pNRF,(u8)(W_REGISTER|address_reg), pdata_write, num)==BUSY_STATE){ // if SPI in processing - return
		return NRF_ERROR_BUSY;
	};
	return NRF_ERROR_OK;
}


//---------NRF24L01_write_fifo_tx-----------------
// function NRF24L01_write_fifo_tx - write NRF24L01+ fifo Tx data
// input arg:
//          - pNRF        - pointer on selected NRF24L01+
//          - num         -  amount of write data
//          - pdata_write - pointer to data
// output arg:
//        0 - operation complete successfully
//       >0 - ref. NRF_ERROR reason
NRF_ERROR NRF24L01_write_fifo_tx(S_nrf_config *const pNRF , u8 num,const u8 *pdata_write){
	if(spi_nrf24l01_TX(pNRF,(u8)(W_TX_PAYLOAD), pdata_write, num)==BUSY_STATE){ // if SPI in processing - return
		return NRF_ERROR_BUSY;
	};
	return NRF_ERROR_OK;
}


//---------NRF24L01_read_fifo_rx-----------------
// function NRF24L01_read_fifo_rx - read NRF24L01+ fifo Rx data
// input arg:
//          - pNRF        - pointer on selected NRF24L01+
//          - num         -  amount of read data
//          - pdata_write - pointer to data
// output arg:
//        0 - operation complete successfully
//       >0 - ref. NRF_ERROR reason
NRF_ERROR NRF24L01_read_fifo_rx(S_nrf_config *const pNRF , u8 num,u8 *pdata_read){
	if(spi_nrf24l01_RX(pNRF,(u8)(R_RX_PAYLOAD), pdata_read, num)==BUSY_STATE){ // if SPI in processing - return
		return NRF_ERROR_BUSY;
	};
	return NRF_ERROR_OK;
}


//---------NRF24L01_FLUSH_TX-----------------
// function NRF24L01_FLUSH_TX - flush Tx FIFO
// input arg:
//          - pNRF        - pointer on selected NRF24L01+
// arg out:
//        0 - operation complete successfully
//       >0 - ref. NRF_ERROR reason
NRF_ERROR NRF24L01_FLUSH_TX(S_nrf_config *const pNRF){
	if(spi_nrf24l01_TX(pNRF,(u8)(FLUSH_TX), (u8*)0, 0)==BUSY_STATE){ // if SPI in processing - return
		return NRF_ERROR_BUSY;
	};
	return NRF_ERROR_OK;
}

//---------NRF24L01_FLUSH_RX-----------------
// function NRF24L01_FLUSH_RX - flush Rx FIFO
// input arg:
//          - pNRF        - pointer on selected NRF24L01+
// arg out:
//        0 - operation complete successfully
//       >0 - ref. NRF_ERROR reason
NRF_ERROR NRF24L01_FLUSH_RX(S_nrf_config *const pNRF){
	if(spi_nrf24l01_TX(pNRF,(u8)(FLUSH_RX), (u8*)0, 0)==BUSY_STATE){ // if SPI in processing - return
		return NRF_ERROR_BUSY;
	};
	return NRF_ERROR_OK;
}



