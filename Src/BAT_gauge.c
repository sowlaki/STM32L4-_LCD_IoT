/*
 * BAT_GAUGE.c
 *
 *  Created on: 11 dec. 2018
 *      Author: cesar
 */


#include "BAT_gauge.h"

// Pheripheral definition
I2C_HandleTypeDef hi2c1;


// Registers, hex address

const uint8_t Control[] = {0x00, 0x01};
const uint8_t Temperature[] = {0x02, 0x03};
const uint8_t Voltage[] = {0x04, 0x05};
const uint8_t Flags[] = {0x06, 0x07};
const uint8_t NominalAvailableCapacity[] = {0x08, 0x09};
const uint8_t FullAvailableCapacity[] = {0x0A, 0x0B};
const uint8_t RemainingCapacity[] = {0x0C, 0x0D};
const uint8_t FullChargeCapacity[] = {0x0E, 0x0F};
const uint8_t AverageCurrent[] = {0x10, 0x11};
const uint8_t StandbyCurrent[] = {0x12, 0x13};
const uint8_t MaxLoadCurrent[] = {0x14, 0x15};
const uint8_t AveragePower[] = {0x18, 0x19};
const uint8_t StateOfCharge[] = {0x1C, 0x1D};
const uint8_t InternalTemperature[] = {0x1E, 0x1F};
const uint8_t StateOfHealth[] = {0x20, 0x21};
const uint8_t RemainingCapacityUnfiltered[] = {0x28, 0x29};
const uint8_t RemainingCapacityFiltered[] = {0x2A, 0x2B};
const uint8_t FullChargeCapacityUnfiltered[] = {0x2C, 0x2D};
const uint8_t FullChargeCapacityFiltered[] = {0x2E, 0x2F};
const uint8_t StateOfChargeUnfiltered[] = {0x30, 0x31};

// Register extended
const uint8_t OpConfig[] = {0x3A, 0x3B};
const uint8_t DesignCapacity[] = {0x3C, 0x3D};
const uint8_t DataClass[] = {0x3E};
const uint8_t DataBlock[] = {0x3F};
const uint8_t BlockDataCheckSum[] = {0x60};
const uint8_t BlockDataControl[] = {0x61};



void	GAUGE_capacitySetup(void){



	const uint8_t databuffer[] = {

			0x00, 0x80,  /* 0: (0x8000) sealed to unsealed "key" */
			0x13, 0x00,	 /* 2: (0x0013) SET_CFGUPDATE  */
			0x00,  		 /* 4: (0x00) Enable blockdata memory controll */
			0x52, 		 /* 5: (0x52) Accese block 0x52in memory */
			0x00,		 /* 6: (0x00) Access offsett 0x00 in block 0x52 */

	};



	// UNSEAL by sending this twice
	Write_GAUGECMD(Control, databuffer, 2);
	Write_GAUGECMD(Control, databuffer, 2);

	Write_GAUGECMD(Control, &databuffer[2], 2);

	while(Read_GAUGECMD(Flags) != (1 << 4)); // Wait for flag to confirm CFGUPDATE

	Write_GAUGECMD(BlockDataControl, &databuffer[4], 1);
	Write_GAUGECMD(DataClass, &databuffer[5], 1 );

	uint8_t OLD_Csum = Read_GAUGECMD(BlockDataCheckSum);

	uint8_t OLD_DesCap_ = ReadDataBlock();
}


	void Write_GAUGECMD( uint8_t * addressBuffer,  uint8_t * dataBuffer, uint8_t nBytes){


		uint8_t int_buff[] = { addressBuffer[0], dataBuffer[0], addressBuffer[1], dataBuffer[1] };





		HAL_I2C_Master_Transmit(&hi2c1, GAUGE_ADDRESS,	int_buff, 2 , 100);

		if(nBytes == 2){

			HAL_I2C_Master_Transmit(&hi2c1, GAUGE_ADDRESS, &int_buff[2],  2, 100);
		}

}


	uint8_t Read_GAUGECMD(uint8_t * addressBuffer){

		uint8_t buffer[1];


		HAL_I2C_Master_Transmit(&hi2c1, GAUGE_ADDRESS<<1, addressBuffer, 2, 100);

		HAL_I2C_Master_Receive(&hi2c1, GAUGE_ADDRESS, buffer, 1, 100);

		return buffer[0];

	}

	void Read_DataBlock( int offset, int nBits, uint8_t * returnData){

	int i, k;
	k = (nBits % 8);

	uint8_t DataBuffer[k];
	uint8_t	BlockBuffer[] = {0x40};

		for (i = offset; i <= (offset + nBits); ++i ){
			int j = 0;
			BlockBuffer[j] = 0x40+i;

		DataBuffer[j] = Read_GAUGECMD(BlockBuffer);
			++j;

		};

}

