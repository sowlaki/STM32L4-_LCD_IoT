/*
 * touch.c
 *
 *  Created on: 20 sep. 2018
 *      Author: Cesar Hanshoff (Knows only C and nothing else (currently))
 *      Language: C
 *
 */


/*
 * This are the drivers for operating the FocalTech FT5x06 capacitive touch screen (CTS).
 *
 * The driver(slave) uses I2C to communicate with a host MCU (master)
 *
 *
 * Read the CTS's program guide in order to understand the registers and commands that is sent through I2C.
 *
 *	http://www.newhavendisplay.com/specs/NHD-5.0-800480TF-ATXL-CTP.pdf
 *
 *
 *
 * 			*** WRITTEN IN C *** !!! *** WRITTEN IN C ***
 *
 *
 * 			*** NOTE *** I2C should already be initialized in main() application before functions from this file are called.
 *
 */




// INCLUDES
    // Include the header file
#include "touch.h"



// DEFINES (pointers and shit that is not in the header file)


// GLOBAL VARIABLES are called in touch.h
I2C_HandleTypeDef hi2c1;








// Data buffers


const uint8_t reg1[] = {0x04}; // X: LSB 8-bit
const uint8_t reg2[] = {0x03}; // X: MSB 4-bit
const uint8_t reg3[] = {0x05}; // Y: MSB 4-bit
const uint8_t reg4[] = {0x06}; // Y: LSB 8-bit


uint8_t X1[0], X2[0], Y1[0], Y2[0];

// Define Functions


int CTS_read_X(void){



     // Discard warning, without 'const' reg values will be changed when used in main.c


	  HAL_I2C_Master_Transmit(&hi2c1, CTS_ADDRESS, reg1,1, 100);

	  HAL_I2C_Master_Receive(&hi2c1, CTS_ADDRESS, X1, 1, 100); // Reading 8 LSB bits.

	  HAL_I2C_Master_Transmit(&hi2c1, CTS_ADDRESS, reg2,1, 100);

	  HAL_I2C_Master_Receive(&hi2c1, CTS_ADDRESS, X2, 1, 100); // Reading 4 MSB bits.


	  uint16_t X = ((X2[0] << 8) | X1[0] );

	  X &= ~(1UL << 12);		// Removing top 4 bits cause they are used for other things. (0x03)
	  X &= ~(1UL << 13);
	  X &= ~(1UL << 14);
	  X &= ~(1UL << 15);

	  int F = X;

	  return F;
}

int CTS_read_Y(void){


	  HAL_I2C_Master_Transmit(&hi2c1, 0x70, reg4,1, 100);

	  HAL_I2C_Master_Receive(&hi2c1, 0x70, Y1, 1, 100); // Reading 8 LSB bits.

	  HAL_I2C_Master_Transmit(&hi2c1, 0x70, reg3,1, 100);

	  HAL_I2C_Master_Receive(&hi2c1, 0x70, Y2, 1, 100); // Reading 4 MSB bits.


	  uint16_t Y = ((Y2[0] << 8) | Y1[0] );

	  Y &= ~(1UL << 12);	// Removing top 4 bits cause they are used for other things (0x05)
	  Y &= ~(1UL << 13);
	  Y &= ~(1UL << 14);
	  Y &= ~(1UL << 15);

	  int F = Y;

	  return F;

}

