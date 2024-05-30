// High priority interrupt called at high frequency
// FAST uses shadow feature to save registers (w0 to w3)
// any other registers used must be saved by user
// Note: Shadow registers only 1 deep, so only one FAST interrupt allowed!
#warning Fast interrupt disabled until CCS fix
#INT_ADCP5	LEVEL=6 //FAST
void dspMeasureCurrents(void)
{

	// all interrupt local variables *must* be static because compiler,
	// until fix from CCS becomes available
	static q15_t adc_flt[12];

	// Read current ADCs, correct to Q15 and IIR filter each
	// ADC must be set to fractional (Q16) mode
	#asm
	push		w4			// manually push to stack in FAST interrupt
	push		CORCON		// because of AccB usage
	
	// AccB is used by dspAtan2 function which this interrupts
	push		0x0028		// save AccBL
	push		0x002A		// save AccBH
	push		0x002C		// save AccBU

	// Load pointers to source and destination arrays
	mov			#0x0340, w0		// adc base pointer at ADCBUF9 for motor0
	mov			adc_flt, w1		// flt base pointer
	mov			#32768,  w4		// ADC_FRC_MIDPOINT (Q16 -> Q1.15)

	// Start of loop
	do			#11, _end		// loop N+1 times

	// Load variables
	mov			[w0++], w2		// sample
	mov			[w1], w3		// flt
   	
	// Correct adc Q16 to Q15, calc delta
	sub			w2, w4,	w2		// sample = sample - midpoint (uint to int)
	sub			w2,w3,w2		// delta = sample - flt 

	// irr filter using accumulator for Q15 & saturation
	lac			w2, #3, B		// A = delta >> 3 i.e. filtering amount
	add			w3,B			// A = flt + A

	// store accumulator, last loop instruction
	_end: sac	B, [w1++]		// A = [flt]

	// restore AccB and w4
	pop			0x002C		
	pop			0x002A
	pop			0x0028
	pop			CORCON
	pop			w4
	#endasm


	// Luckily the C code below only requires push/pop instructions,
	// so registers above w4 do not have to be saved and C can be used

	// swap phases A and B depending on calibrated parity
	if (m0.phase_parity == 0)
	{
		m0.qIa = adc_flt[11] - m0.qIaOff; //2
		m0.qIb = adc_flt[10] - m0.qIaOff; //1
		m0.qIc = adc_flt[9]  - m0.qIaOff; //0
		
		// ONLY VIABLE WHEN COMPILER FIXED
		// #asm
		// mov			m0.qIaoff, w0
		// sub			adc_flt[11], w0 
		// mov			w0, m0.qIa
		// mov			m0.qIboff, w0
		// sub			adc_flt[10], w0 
		// mov			w0, m0.qIb
		// mov			m0.qIcoff, w0
		// sub			adc_flt[9], w0 
		// mov			w0, m0.qIc
		// #endasm
	} 
	else
	{
		m0.qIa = adc_flt[10] - m0.qIaOff; //1
		m0.qIb = adc_flt[11] - m0.qIbOff; //2
		m0.qIc = adc_flt[9]  - m0.qIcOff; //0
	}
   	
	m0.angle_a = adc_flt[2];
	m0.angle_b = adc_flt[1];
	m0.angle_c = adc_flt[0];

	if (m1.phase_parity == 0)
	{
		m1.qIa = adc_flt[8] - m1.qIaOff; //2
		m1.qIb = adc_flt[7] - m1.qIbOff; //1
		m1.qIc = adc_flt[6] - m1.qIcOff; //0
	} 
	else
	{
		m1.qIa = adc_flt[7] - m1.qIaOff; //1
		m1.qIb = adc_flt[8] - m1.qIbOff; //2
		m1.qIc = adc_flt[6] - m1.qIcOff; //0
	}
   	
	m1.angle_a = adc_flt[5];
	m1.angle_b = adc_flt[4];
	m1.angle_c = adc_flt[3];
}


#INT_DMA0 LEVEL = 4
void ProcessSPIPacket(void) 
{
	spi_process_incoming();
	spi_loadout();
	spi.buffer_bank = ~spi.buffer_bank;		// toggle to the other bank
	spi.offline_counter = 0;				// keep a running total instead of reset
	BLUE_LED_OFF;							// On when good packet received in incoming()
}


#INT_PWM1 LEVEL = 5
void dspFOC(void)
{
	static uint16_t omega_timer;

	// Save AccA so background tasks can use it
	// Note: AccB also used in dspFOC but not saved
	// as no other function uses it except #INT_ADCP5 
	#asm
	push		CORCON
	push		0x0022		// save AccAL
	push		0x0024		// save AccAH
	push		0x0026		// save AccAU
	#endasm

	// Very first thing for less jitter
	// Call every tenth pass for ~1ms timing
	if (omega_timer++ > 8)
	{
		dspOmegaEstimation();
		omega_timer = 0;
	}

	//for PID tuning ~3200 good with bench supply
	//m0Q.qInRef = 5000 * spin_switch; 

	//#warning LED updated added for testing
	//led_update();

	//Do FOC
	dspMeasureTheta(&m0);
	dspMeasureTheta(&m1);

	// Call frequently to ensure steps are not missed
	// Uses theta, call after getting latest theta
	disable_interrupts(INT_ADCP5);	// Can cause missed steps
	dspResolveStepCount(&m0);
	dspResolveStepCount(&m1);
	enable_interrupts(INT_ADCP5);

	dspClarkePark(&m0);
	dspClarkePark(&m1);
	dspCalcDamping(&m0);
	dspCalcDamping(&m1);

	m0D.qInMeas = m0.qId;
	m0Q.qInMeas = m0.qIq;
	m1D.qInMeas = m1.qId;
	m1Q.qInMeas = m1.qIq;

	dspAdjCurrents();

	dspPID(&m0D);
	dspPID(&m0Q);
	dspPID(&m1D);
	dspPID(&m1Q);
   	
	// Reset integral windup
	// Todo: reset when Vbatt == 0
	if (m0Q.qInRef == 0)
	{
		m0Q.qdSum = 0;
		m0D.qdSum = 0;
	}
	if (m1Q.qInRef == 0)
	{
		m1Q.qdSum = 0;
		m1D.qdSum = 0;
	}
   	
   	
	// PID -> Filter -> Voltages
	//dspFilterBiquadVd(&m0, &m0D);
	//dspFilterBiquadVq(&m0, &m0Q);
	//dspFilterBiquadVd(&m1, &m1D);
	//dspFilterBiquadVq(&m1, &m1Q);

	//dspIIRCascadeQ(&m0, &m0Q);
	//dspIIRCascadeD(&m0, &m0D);

	// PID -> Voltage (no filter)
	m0.qVd = m0D.qOut;
	m0.qVq = m0Q.qOut; 
	m1.qVd = m1D.qOut;
	m1.qVq = m1Q.qOut;

	dspInvParkClarke(&m0);
	dspInvParkClarke(&m1);
	dspSinModulation(&m0);
	dspSinModulation(&m1);
	dspApplyPWM();


	// Restore AccA
	#asm
	pop		0x0026		// save AccAU
	pop		0x0024		// save AccAH
	pop		0x0022		// save AccAL
	pop		CORCON
	#endasm
}


#INT_SPI1 LEVEL = 2
void spi1Xfer(void)
{
	static uint16_t selector;

	// Alternate reading s1 / s0
	if (selector)   //Data in from S1 side
	{
		//Assert chip selects for next round
		S1_ENCODER_CS_HIGH;
		S0_ENCODER_CS_LOW;
		
		//Clear 2 highest bits, leaving only data bits
		//14b to 16b shift enables wrap-around CORDIC behaviour
		s1.encoder = ((SPI1BUF & 0b0011111111111111) << 2);
		
		selector = ~selector;
	}
	else			//Data in from S0 side
	{
		S0_ENCODER_CS_HIGH;
		S1_ENCODER_CS_LOW;
		
		s0.encoder = ((SPI1BUF & 0b0011111111111111) << 2);
	
		selector = ~selector;
	}
		
	//Flag clearing must happen here after reading SPI1BUF contents
	SPI1IF  = 0;		// Clear interrupt flag
	SPI1ROV = 0;		// Clear RX overflow flag
	SPI1BUF = 0x3FFF;	// ANGLECOM register on AS5047 encoder
}


// UART 1 char to be fetched
#INT_RDA LEVEL = 1
void getuart(void) 
{
   uart_input = fgetc(U1);
}


// Interrupt called every 1ms, however
// expect jitter as it is low priority
// not for critical timings
#INT_TIMER4 LEVEL = 1
void updateCounters(void)
{
	if (spi.offline_counter < UINT16_MAX) spi.offline_counter++;
	uart_timer++;
	perf_counter = 0;
}


/*** Trap handlers ***/
// MCU resets if an error is trapped and no 
// handler has been defined below

#INT_ADDRERR
void addr_trap_handler(void) 
{
	// make safe
	setup_hspwm(HSPWM_DISABLED | HSPWM_CLOCK_DIV_BY_1, PWM_PERIOD);
	
	if (uart_mode == TERMINAL) fprintf(U1, "Address Error\n\r");
	
	BLUE_LED_OFF;
	ORANGE_LED_OFF;
	
	while(TRUE)
	{
		RED_LED_ON;
		delay_ms(100);
		RED_LED_OFF;
		delay_ms(100);
	}
}

#INT_MATHERR
void math_trap_handler(void) 
{
	// make safe
	setup_hspwm(HSPWM_DISABLED | HSPWM_CLOCK_DIV_BY_1, PWM_PERIOD);
	
	if (uart_mode == TERMINAL) fprintf(U1, "Math Error\n\r");
	
	BLUE_LED_OFF;
	ORANGE_LED_OFF;
	
	while(TRUE)
	{
		RED_LED_ON;
		delay_ms(100);
		RED_LED_OFF;
		delay_ms(100);
	}
}

#INT_DMAERR
void dma_trap_handler(void) 
{
	// make safe
	setup_hspwm(HSPWM_DISABLED | HSPWM_CLOCK_DIV_BY_1, PWM_PERIOD);
	
	if (uart_mode == TERMINAL) fprintf(U1, "DMA Error\n\r");
	
	BLUE_LED_OFF;
	ORANGE_LED_OFF;
	
	while(TRUE)
	{
		RED_LED_ON;
		delay_ms(100);
		RED_LED_OFF;
		delay_ms(100);
	}
}

#INT_STACKERR
void stack_trap_handler(void) 
{
	// make safe
	setup_hspwm(HSPWM_DISABLED | HSPWM_CLOCK_DIV_BY_1, PWM_PERIOD);
	
	if (uart_mode == TERMINAL) fprintf(U1, "Stack Error\n\r");
	
	BLUE_LED_OFF;
	ORANGE_LED_OFF;
	
	while(TRUE)
	{
		RED_LED_ON;
		delay_ms(100);
		RED_LED_OFF;
		delay_ms(100);
	}
}
