#ifndef parameters_h
#define parameters_h


/*** GLOBAL DEFINITIONS ***/

// CPU and General Registers
#word CORCON	=			0x0044 // core control
#word IPC28		=			0x00DC // ADCP5 interrupt priority


// SPI Registers
#word SPI1STAT	=			0x0240			//SPI1 Stat register
#bit  SPI1ROV   =			SPI1STAT.6		//SPI1 RX Overflow flag
#word IFS0		=			0x0084			//Interrupt flag register 0
#bit  SPI1IF	=			IFS0.10			//SPI1 interrupt flag
#word SPI1BUF	=			0x0248			//SPI1 TX & RX register
#word SPI2STAT	=			0x0260			//SPI2 Status register
#word SPI2BUF	=			0x0268			//SPI2 TX & RX register
#bit  SPI2EN	=			SPI2STAT.15		//SPI2 Enable bit

// PWM registers
#word PWMCON1	=			0x0420 //PWM control register 1
#word PWMCON2	=			0x0440
#word PWMCON3	=			0x0460
#word PWMCON5	=			0x04A0
#word PWMCON6	=			0x04C0
#word PWMCON7	=			0x04E0
#word PDC1		=			0x0426 //motor1 phase C
#word PDC2		=			0x0446 //motor1 phase B
#word PDC3		=			0x0466 //motor1 phase A
#word PDC5		=			0x04A6 //motor0 phase C
#word PDC6		=			0x04C6 //motor0 phase B
#word PDC7		=			0x04E6 //motor0 phase A


//ADC read buffer registers
#word ADCON		=			0x0300 //adc control register
#word ADCBUF0	=			0x0340 //motor0 hall sensor C
#word ADCBUF1	=			0x0342 //motor0 hall sensor B
#word ADCBUF2	=			0x0344 //motor0 hall sensor A
#word ADCBUF3	=			0x0346 //motor1 hall sensor C
#word ADCBUF4	=			0x0348 //motor1 hall sensor B
#word ADCBUF5	=			0x034A //motor1 hall sensor A
#word ADCBUF6	=			0x034C //motor1 current sensor C
#word ADCBUF7	=			0x034E //motor1 current sensor B
#word ADCBUF8	=			0x0350 //motor1 current sensor A
#word ADCBUF9	=			0x0352 //motor0 current sensor C
#word ADCBUF10	=			0x0354 //motor0 current sensor B
#word ADCBUF11	=			0x0356 //motor0 current sensor A
#word ADCBUF12	=			0x0358 //motor0 temperature
#word ADCBUF13	=			0x035A //motor1 temperature
#word ADCBUF14	=			0x035C //Vbatt or unused
#word ADCBUF15	=			0x035E //spare adc port
#word ADCBUF16	=			0x0360 //motor0 mosfet temperature
#word ADCBUF17	=			0x0362 //motor1 mosfet temperature


// Sensors and scaling
#define ADC_10B_MIDPOINT 512		// Zero point of 10bit ADC reading. 511 results in (invalid) +32768 q15!!
#define ADC_FRC_MIDPOINT (ADC_10B_MIDPOINT * 64)	// ADC fractional mode returns unsigned Q16 result 


// Matrix coeffs for angle detremination, computed offline using
// high-level-control/scripts/motor_analysis/motor_angle_estimation.py
#define XINV_11 21845
#define XINV_12 (-10923)
#define XINV_13 (-10923)
#define XINV_21 0
#define XINV_22 (-18919)
#define XINV_23 18919


// MCU pin shorthands
#define BLUE_LED_ON			output_high(pin_d0)		//data and comms
#define BLUE_LED_OFF		output_low(pin_d0)
#define RED_LED_ON			output_high(pin_d11)	//error or fatal
#define RED_LED_OFF			output_low(pin_d11)
#define ORANGE_LED_ON		output_high(pin_d10)	//warning
#define ORANGE_LED_OFF		output_low(pin_d10)
#define PROBE_PIN_ON		output_high(pin_c14)	//ISCP CLK pin, easy to probe header
#define PROBE_PIN_OFF		output_low(pin_c14)
#define S0_ENCODER_CS_LOW	output_low(pin_f4)		//chip selects on SPI1 bus
#define S0_ENCODER_CS_HIGH	output_high(pin_f4)
#define S1_ENCODER_CS_LOW	output_low(pin_f5)
#define S1_ENCODER_CS_HIGH	output_high(pin_f5)
#define AUX1_CS_LOW			output_low(pin_d14)
#define AUX1_CS_HIGH		output_high(pin_d14)
#define AUX2_CS_LOW			output_low(pin_d15)
#define AUX2_CS_HIGH		output_high(pin_d15)

// Firmware constants
#define CALIBRATION_VOLTS			9000			// must be >25% of max (32767)
#define CALIBRATION_MAX_ATTEMPS		3
#define CALIBRATION_STEPS			30
#define THETA_LUT_SIZE				256				// ensure flash store reserved area set large enough
#define THETA_LUT_BITS				8				// bits to shift theta right to access LUT bin

#define BEMF_DAMP_K					(-2)			// -3 is the largest stable power (@ 10kHz, -6 at 20kHz)
#define HARD_CURRENT_LIMIT			15728			// Current not to exceed, 3000 centiAmps in Q15
#define Q15_SCALE_CURRENT			6250			// Max current is 6250 centiAmps

#define SPI_DMA_BUFFER_SIZE 		22  			//Size of SPI packet in Words
#define UART_DMA_BUFFER_SIZE		45



/** DMA BANK IN SPECIAL RAM AREAS  FIRST ***/
#bank_dma
uint8_t uart_transmit_buffer[UART_DMA_BUFFER_SIZE];
#bank_dma 
uint16_t spi_rx_a[SPI_DMA_BUFFER_SIZE];
#bank_dma 
uint16_t spi_rx_b[SPI_DMA_BUFFER_SIZE];
#bank_dma 
uint16_t spi_tx_a[SPI_DMA_BUFFER_SIZE];
#bank_dma 
uint16_t spi_tx_b[SPI_DMA_BUFFER_SIZE];


/*** GLOBAL ENUMERATIONS ***/
enum 
{
   INIT,            // pre-hardware initialisation, be safe
   RUN,             // default runmode, slaved to master controller
   SPI2_OFFLINE,	// default state when spi2 master is offline
   CALIBRATION,     // calibrate motors
   FATAL_ERROR		// fatal fault detected
} system_state;

enum 
{
   TERMINAL,	// send ansi/ascii serial terminal 
   BINARY       // send binary data for real-time plotting
} uart_mode;	// keeping at two allows use of ternary operator


/*** SYSTEM WIDE VARIABLES ***/

uint8_t			reset_cause;					// Why did MCU restart?
uint8_t			init_result;					// was initialisation successful
uint8_t			uart_input;						// from user
int16_t			spin_switch;					// for DEBUG, remove later
uint16_t		perf_counter;					// for DEBUG, remove later
q15_t			analog_ch15;					// spare ADC channel
q15_t			battery_volts;					// Not all ESCs have Vbatt sensing
uint16_t		uart_timer;						// for UI printing

// SPI2 Interface
struct   
{
	uint16_t bad_packets;						// SPI/DMA interrupt but packet was discarded
	uint16_t buffer_bank;						// Which ping-pong TX/RX buffers are we filling now?
	uint16_t chksum_rx;							// Checksum of RX packet
	uint16_t chksum_verify;						// Checksum calculated for comparison to received
	uint16_t chksum_tx;							// Checksum to send on TX packet
	uint16_t offline_counter;					// Time steps SPI2 interrupt is inactive
} spi;


// Motor state
typedef struct
{
	q15_t			qIa, qIb, qIc;				// motor a,b,c phase currents
	q15_t			qIaoff;						// ADC zero-bias offset of current sensors
	q15_t			qIboff;
	q15_t			qIcoff;
	q15_t			qIalpha, qIbeta;			// clark transformed alpha and beta currents
	q15_t			qId, qIq;					// park transformed quadrature and direct currents
	q15_t			qVd, qVq;					// quadrature and direct voltages we want to apply (from PI controller)
	q15_t			qValpha, qVbeta;			// inv park transformed voltages
	q15_t			qVr1, qVr2, qVr3;			// modified inv clark output voltages for SVM
	uint16_t		dPWM1, dPWM2, dPWM3;		// PWM duty level from SVM
	
	q15_t			angle_a;					// motor linear hall angle sensors
	q15_t			angle_b;
	q15_t			angle_c;	
	q15_t			angle_a_prev;				// For velocity estimation
	q15_t			angle_b_prev;
	q15_t			angle_c_prev;
	q16angle_t		theta;						// motor electrical phase angle
	q16angle_t		theta_prev;					// for omega estimation
	q16angle_t		theta_offset;				// motor elec phase angle offset from calibration
	q15_t			sin_theta, cos_theta;		// sin(theta) and cos(theta)
	q15_t			omega;						// electrical phase velocity

	q15_t			bemf;						// back emf
	q15_t			zeta;						// level of damping (0..1)
	q15_t			zeta_current;				// Current gnerated by active damping function
	int16_t			step_count;					// number of motor electrical steps since startup
	int16_t			phase_parity;				// motor phase wiring into esc

	int16_t			theta_lut[THETA_LUT_SIZE];	// linearisation calibration look up table for electrical angle
	q15_t			qIq_limit;					// User set current (quadrature) limit for FOC
} motor_state_t;

motor_state_t m0, m1; // motor 0 and motor 1

typedef struct
{
	int16_t mosfet_temperature;
	int16_t motor_temperature;
	uint16_t encoder;
} sensor_state_t;

sensor_state_t s0, s1; //motor0 and motor1 related sensors


// first struct element must be the 32bit Sum
// for optimised PI(D) to operate on
typedef struct
{
	int32_t	qdSum;		// 1.31 format
	q15_t	qKp;		// Proportional term
	q15_t	qKi;		// Integral term
	q15_t	qKc;		// Integral wind down term (usually symmetric)
	int16_t NKo;		// Kp * Err * 2^Nko otherwise Kp too small to matter
	q15_t	qOutMax;	// Max positive PID output
	q15_t	qOutMin;	// Min negative PID output
	q15_t	qInRef; 	// PID setpoint
	q15_t	qInMeas;	// PID term to match
	q15_t	qOut;		// PID output (i.e. Vd and Vq)
	q15_t	qFF;		// Static feed forward term
	q15_t	cmd_adj;	// PID setpoint after adjustments 
}	PID_t;

PID_t	m0D;	// Structure for PID of Flux component of current, or Id
PID_t	m0Q;	// Structure for PID of Torque component of current, or Iq
PID_t	m1D;
PID_t	m1Q;

#endif
