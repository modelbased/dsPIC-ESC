#ifndef esc_h
#define esc_h


/*** ESC VERSION ***/
#define ESC_VERSION  2  // only v2 defined, default


/*** DSPIC33 HARDWARE UNITS ***/

#zero_ram // Zeroes global variables once on powerup. Does not zero local variables during execution!
#zero_local_ram // New compiler feature, should zero local ram.

/* IMPORTANT: Local variables (including pointers!) in interrupts
* are "not protected" so a higher priority interrupt can corrupt
* (at best) to address trap fault, at worst to weird bugs.
* SOLUTION: Don't use local vars in interrupts, if there are any
* make them "static".
*
* Higher priority interrupts can interrupt lower priority interrupts.
* Reliable operation requires sufficient stack space.
* INT_xxx Level = 1 (lowest) to 7 (highest) priority.
* Level = 7 is never disabled, probably best avoided.
* Level = 0 Disables the interrupt.
*/
#device  NESTED_INTERRUPTS=TRUE
#device  ICSP=2			// Programming pin port 

#fuses   NOWDT			// No Watch Dog Timer
#fuses   CKSFSM			// Clock Switching is enabled, fail Safe clock monitor is enabled
#fuses   NOJTAG			// JTAG disabled
#fuses   PUT2			// Power Up Timer

#build	(stack=2048)	//Large for interrupt nesting

// Tell compiler the MCU speed and sets up configuration for internal oscillator
#use	delay(internal=100MHZ)					//Divide by 2 for actual instruction rate (50Mhz)
#use	rs232(UART1, baud=1600000, stream=U1)	//uart user interface
#use	i2c(MASTER, I2C1, SLOW) 				//Used by EEPROM, if there is one


// Set aside program memory for calibration data
// table[256] + offset + parity{byte} per motor, plus a checksum(32bit)
// allow for word/byte storage inneficiency (see storage function)
// bytes needed = theta_lut_size * 4(bytes per word) * 2 motors
// Ensure always even addresses and sizes or addressing will break 
#define FLASH_START_ADDRESS		0x9000  // on-chip program memory
#define FLASH_SIZE_RESERVED		0x0C00	// two flash blocks (1536 bytes per block)
#define FLASH_END_ADDRESS		(FLASH_START_ADDRESS + FLASH_SIZE_RESERVED)
#org	FLASH_START_ADDRESS, FLASH_END_ADDRESS {}


// PWM Firmware settings
// Alt deadtime is the total value for both rising and falling pwm edges
// so divide by 2 to get dedtime for one edge. Estimate mosfets (IRFS7534)
// need 280ns to switch under datasheet conditions, so allowing 850ns
// or provides some headroom.
#define PWM_PERIOD			1420	// 20.8kHz per 'scope, can vary by chip
#define PWM_ALT_DEAD_TIME   50		// 850ns per 'scope


// SPI slave definitions
#define SPI_MODE_0  (SPI_L_TO_H | SPI_XMIT_L_TO_H) 
#define SPI_MODE_1  (SPI_L_TO_H) 
#define SPI_MODE_2  (SPI_H_TO_L) 
#define SPI_MODE_3  (SPI_H_TO_L | SPI_XMIT_L_TO_H)


/*** PCB WIRING NOTES ***

M0 (hip / left) motor PWM & ADC sensor:

Phase A PWM7   | ADC11 ADC PAIR 5
Phase B PWM6   | ADC10 ADC PAIR 5
Phase C PWM5   | ADC9  ADC PAIR 4

M1 (knee / right) motor PWM & ADC:

Phase A PWM1   | ADC8 ADC PAIR 4
Phase B PWM2   | ADC7 ADC PAIR 3
Phase C PWM3   | ADC6 ADC PAIR 3

ADC sensors:

M0 Motor  Temperature   AN12 ADC PAIR 6
M0 Mosfet Temperature   AN16 ADC PAIR 8

M1 Motor  Temperature   AN13 ADC PAIR 6
M1 Mosfet Temperature   AN17 ADC PAIR 8

Analog Input AN15 ADC PAIR 7

Motor hall position sensor ADCs:

Motor M0 Angle A AN02
Motor M0 Angle B AN01
Motoe M0 Angle C AN00

Motor M1 Angle A AN05
Motor M1 Angle B AN04
Motor M1 Angle C AN03

ADC PAIR    AN (Channel Number)
   0        0,1
   1        2,3
   2        4,5
   3        6,7
   4        8,9
   5        10,11
   6        12,13
   7        14,15
   8        16,17

***/

#endif
