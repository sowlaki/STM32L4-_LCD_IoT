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




uint16_t Charge(void){

	uint8_t lsbAddress = StateOfCharge[0];
	uint8_t msbAddress = StateOfCharge[1];


	uint8_t lsbCharge = Read_GAUGECMD(&lsbAddress);
	HAL_Delay(1);
	uint8_t msbCharge = Read_GAUGECMD(&msbAddress);

	uint16_t rValue = ((lsbCharge) | (msbCharge << 8));

	return rValue;
}




void	GAUGE_CapacitySetup(uint16_t aBat_capacity){



	uint8_t databuffer[] = {

			0x00, 0x80,  /* 0: (0x8000) sealed to unsealed "key" */
			0x13, 0x00,	 /* 2: (0x0013) SET_CFGUPDATE  */
			0x00,  		 /* 4: (0x00) Enable blockdata memory controll */
			0x52, 		 /* 5: (0x52) Accese block 0x52in memory */
			0x00,		 /* 6: (0x00) Access offsett 0x00 in block 0x52 */
			0x42, 0x00,  /* 7: (0x0042) Exit CFGUPDATE */
			0x20, 0x00,  /* 9: (0x0020) Enter UNSEALED */
	};



	// UNSEAL by sending this twice
	Write_GAUGECMD(Control, databuffer, 2);
	Write_GAUGECMD(Control, databuffer, 2);

	Write_GAUGECMD(Control, &databuffer[2], 2);

	while(Read_GAUGECMD(Flags) != (1 << 4)); // Wait for flag to confirm CFGUPDATE

	Write_GAUGECMD(BlockDataControl, &databuffer[4], 1);
	Write_GAUGECMD(DataClass, &databuffer[5], 1 );

	uint8_t OLD_Csum = Read_GAUGECMD(BlockDataCheckSum);

	uint16_t OLD_DesCap = 0;
	Read_DataBlock(10, 16, OLD_DesCap);

	uint8_t OLD_DesCapLSB = (OLD_DesCap);
	uint8_t OLD_DesCapMSB = (OLD_DesCap << 8);

	// Write New CAPACITY
	uint8_t CapAddrBuffer[] = {0x4A, 0x4B};

	uint8_t CAPLSB = (aBat_capacity);
	uint8_t CAPMSB = (aBat_capacity << 8);
	uint8_t CapDataBuffer[] = {CAPLSB, CAPMSB};

	Write_GAUGECMD(CapAddrBuffer,CapDataBuffer,1);
	Write_GAUGECMD(&CapAddrBuffer[1], &CapDataBuffer[1], 1);


	// Set new values for GAUGE
	uint8_t temp = ((255 - OLD_Csum - OLD_DesCapLSB - OLD_DesCapMSB) % 256 );

	uint8_t NEW_Csum = ((temp + 0x04 + 0xB0) % (256) );

	uint8_t CsumBuff[] = {NEW_Csum};

	// Write new CSUM
	Write_GAUGECMD(BlockDataCheckSum, CsumBuff ,1);


	// Exit CFGUPDATE
	Write_GAUGECMD(Control, &databuffer[7], 2);

	// Read flag to check if CFGUPDATE has been exited.
	while((Read_GAUGECMD(Flags) & ( 1 << 4)) != 1){

	}

	// Return to unsealed mode
	Write_GAUGECMD(Control,	&databuffer[9], 2);

	// DONE !!!
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

	void Read_DataBlock( int offset, int nBits, uint16_t returnData){

	int k = (nBits % 8);


	uint8_t	Blockaddr1[] = {(0x40 + offset)};

	uint8_t byte1 = Read_GAUGECMD(Blockaddr1);

	if(k > 1){
	uint8_t Blockaddr2[] = {(0x40 + (offset + 1))};

	uint8_t byte2 = Read_GAUGECMD(Blockaddr2);

	returnData = (byte1 | (byte2 << 8));
	}
	else
	returnData = (byte1);

/*
		for (i = offset; i <= (offset + nBits); ++i ){
			int j = 0;
			BlockBuffer[j] = 0x40+i;

		DataBuffer[j] = Read_GAUGECMD(BlockBuffer);
			++j;

		};
       for( f = 0; i <= nBits; ++i){
    	   returnData = (DataBuffer[f] << (f+1));
       }
*/
	}

