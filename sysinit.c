//Initialise Hardware and Firmware

// In center aligned mode
void init_pwm(void)
{
	// Datasheet says dont use deadtime, only alt_deadtime when center aligned
	setup_hspwm_unit(1, HSPWM_ENABLE | HSPWM_COMPLEMENTARY | HSPWM_CENTER_ALIGN_MODE | HSPWM_TRIGGER_INT_ENABLED, 0, PWM_ALT_DEAD_TIME);
	setup_hspwm_unit(2, HSPWM_ENABLE | HSPWM_COMPLEMENTARY | HSPWM_CENTER_ALIGN_MODE, 0, PWM_ALT_DEAD_TIME);
	setup_hspwm_unit(3, HSPWM_ENABLE | HSPWM_COMPLEMENTARY | HSPWM_CENTER_ALIGN_MODE, 0, PWM_ALT_DEAD_TIME);
	setup_hspwm_unit(5, HSPWM_ENABLE | HSPWM_COMPLEMENTARY | HSPWM_CENTER_ALIGN_MODE, 0, PWM_ALT_DEAD_TIME);
	setup_hspwm_unit(6, HSPWM_ENABLE | HSPWM_COMPLEMENTARY | HSPWM_CENTER_ALIGN_MODE, 0, PWM_ALT_DEAD_TIME);
	setup_hspwm_unit(7, HSPWM_ENABLE | HSPWM_COMPLEMENTARY | HSPWM_CENTER_ALIGN_MODE, 0, PWM_ALT_DEAD_TIME);
  	
	// Set Independent Time Base for each PWM Module
	bit_set(PWMCON1,9);
	bit_set(PWMCON2,9);
	bit_set(PWMCON3,9);
	bit_set(PWMCON5,9);
	bit_set(PWMCON6,9);
	bit_set(PWMCON7,9);
	
	// Period set here because of independent time base for center aligned pwm
	set_hspwm_phase(1, PWM_PERIOD);
	set_hspwm_phase(2, PWM_PERIOD);
	set_hspwm_phase(3, PWM_PERIOD);
	set_hspwm_phase(5, PWM_PERIOD);
	set_hspwm_phase(6, PWM_PERIOD);
	set_hspwm_phase(7, PWM_PERIOD);

	// Enable special event trigger at center of PWM perid, no interrupt
	//set_hspwm_event(HSPWM_SPECIAL_EVENT_INT_ENABLED, 0);
  	
	// PWM unit 1 fires the trigger event (this is not the special event)
	// Read in middle of PWM Period (1 center aligned period = 2 x PWM_PERIOD)
	// Divisor of 1 = fires every other period e.g. 20kHz PWM = 10kHz firing
	setup_hspwm_trigger(1, PWM_PERIOD, 1); //PWM unit, start delay, divisor

	// Enable the special event interrupt
	enable_interrupts(INT_PWM1);

	// Just to be sure, all PWM duty set to zero
	PDC1 = 0;
	PDC2 = 0;
	PDC3 = 0;
	PDC5 = 0;
	PDC6 = 0;
	PDC7 = 0;
  	

	// Note on HSPWM_UPDATED_IMMEDIATELY option
	// ERRATA ITEM NO 27 means PDC change must happen at right time
	// which seems to be the center of the pwm period
	// otherwise dead time does not get applied --> mosfet overheats/blows.
	// Must be tested thoroughly with 'scope before enabled.

	/**
	 * Enable PWM hardware
	 * PWM Freq = (ACLK * 8) / ( (PWM_PERIOD*2) * HSPWM_CLOCK_DIV_BY_X)
	 * 
	 * Where ACLK = 7.37M hz
	 * PWM_PERIOD * 2 because center aligned PWM doubles the period duration
	 * by counting up then counting down.
	 * 
	 * Note1 the above is true *given how the compiler is setting up the clocks* 
	 * by default.
	 * 
	 * Note2 PWM freq can vary by a few %1 to 2% because we're using the fast internal 
	 * oscillator the FRC and not an external crystal.
	 * 
	 * */
	setup_hspwm(HSPWM_ENABLED | HSPWM_CLOCK_DIV_BY_1, PWM_PERIOD);
  	
	// Note that setting PWM duty can and is done by writing directly
	// to the PWM duty registers (PDCx) in the code
}


void init_hardware(void)
{
	/*** SETUP CPU ***/

	//Set CORCON to enable dsp accumulator saturation.
	bit_set(CORCON,6);  // SATB: ACCB Saturation Enable bit
	bit_set(CORCON,7);  // SATA: ACCA Saturation Enable bit


	/*** SETUP TIMERS ***/
	
	// IMPORTANT: Timer frequencies as well as other interrupt events stongly
	// affect program operation. Review carefully before changing timers. 

	// TMR_INTERNAL = Half the oscillator frequency (which is 100Mhz, therefore 50Mhz)
	// Freq = (TMR_INTERNAL / (TMR_DIV_BY_x X PERIOD))
	// If period is not specified the default is 0xFFFF

	// Timer settings 
	setup_timer1(TMR_INTERNAL | TMR_DIV_BY_1, 500);		// 100khz for adc trigger (not the interrupt!)
	//setup_timer2(TMR_INTERNAL | TMR_DIV_BY_1);
	//setup_timer3(TMR_INTERNAL | TMR_DIV_BY_8);
	setup_timer4(TMR_INTERNAL | TMR_DIV_BY_8, 6250);	// for UI and background tasks
	//setup_timer5(TMR_INTERNAL | TMR_DIV_BY_8, 6250);
   	
	// Timer interrupts
	//enable_interrupts(INT_TIMER1);
	//enable_interrupts(INT_TIMER2);
	//enable_interrupts(INT_TIMER3);
	enable_interrupts(INT_TIMER4);
	//enable_interrupts(INT_TIMER5);
	 

	/*** SETUP ADC ***/

	setup_high_speed_adc(ADC_CLOCK_DIV_7 | ADC_OUTPUT_FRACTIONAL | ADC_DEDICATED_SAMPLE_CONSTANTLY);

	// Set triggers for ADC sampling
	// Current sensors
	setup_high_speed_adc_pair(3, TIMER1_PERIOD_MATCH);		// Sample sync'd to timer1
	setup_high_speed_adc_pair(4, TIMER1_PERIOD_MATCH);
	setup_high_speed_adc_pair(5, TIMER1_PERIOD_MATCH);
	
	// Temperature, aux and battery
	setup_high_speed_adc_pair(6, PWM_GEN1_PRIMARY_TRIGGER);	// Sample sync'd to PWM1 period
	setup_high_speed_adc_pair(7, PWM_GEN1_PRIMARY_TRIGGER);
	setup_high_speed_adc_pair(8, PWM_GEN1_PRIMARY_TRIGGER);
	  
	// Motor hall (angle) sensors
	setup_high_speed_adc_pair(0, TIMER1_PERIOD_MATCH);
	setup_high_speed_adc_pair(1, TIMER1_PERIOD_MATCH);
	setup_high_speed_adc_pair(2, TIMER1_PERIOD_MATCH);
	
	// Interrupt on ADC pair 5 sample ready
	// Should be some ns after timer1 triggers the ADC sampling 
	enable_interrupts(INT_ADCP5);

	
	/*** SETUP DMA ***/
	
	// Setup DMA channels 0 and 1 for SPI2 bus 
	setup_dma(0, DMA_IN_SPI2,  DMA_WORD);
	setup_dma(1, DMA_OUT_SPI2, DMA_WORD);
	  
	//Setup DMA channel 2 for UART1
	setup_dma(2, DMA_OUT_UART1, DMA_BYTE);
   	

	/*** SETUP SPI ***/

	// Clear SPI2 buffer
	SPI2BUF = 0;

	// Setup SPI2 bus (esc slave to master high level control)
	setup_spi2(SPI_SLAVE | SPI_MODE_0 | SPI_MODE_16B);

	// Start DMA operation for SPI2 bus 
	// DMA Device Type = 1 therefore (SPI_DMA_BUFFER_SIZE - 1)
	dma_start(0, DMA_PING_PONG | DMA_CONTINOUS, &spi_rx_a[0], &spi_rx_b[0], SPI_DMA_BUFFER_SIZE - 1);
	dma_start(1, DMA_PING_PONG | DMA_CONTINOUS, &spi_tx_a[0], &spi_tx_b[0], SPI_DMA_BUFFER_SIZE - 1);

	// Setup SPI bus 1 (sensors on spi bus)
	// Transferring 16bits at 16kbps results in 1kHz transfer freq,
	// set the baud rate to meet transfer freq desired.
	#use spi(MASTER, SPI1, MODE=1, XFER16, BAUD=64000, STREAM=SPI_ENC)

	// Enable SPI1 bus interrupt
	enable_interrupts(INT_SPI1);
	
	// Do *not* enable SPI2 bus interrupt, using DMA interrupt instead
	//enable_interrupts(INT_SPI2);

	// Enable DMA0 (SPI2 RX) interrupt
	enable_interrupts(INT_DMA0);
	
	// Kick off SPI1 bus transfers with a write to SPI1 TX buffer
	// 0x3FFF = ANGLECOM register of AS5047 encoder; writing to this address
	// gets the AS5047 to send back compensated angle reading in a 16bit word
	// where bits 15 and 14 are status and 14 to 0 is the angle
	SPI1BUF = 0x3FFF;

	/*** SETUP PWM ***/
	init_pwm();


	/*** SETUP MISC ***/
	
	// Enable UART interrupt for user interface
	enable_interrupts(INT_RDA);
}

void init_firmware(void) 
{
	// Note: with #zero_ram all non-initialised *global* parameters
	// are set to zero on startup. Initialise by exception. 

	// Default state machine states
	uart_mode = TERMINAL;

	//clear interface
	clear_terminal();
	term_xy(0,0);

	// PID parameters tuned empirically
	m0Q.qFF = 8191;
	m0Q.qKp = 16384;
	m0Q.qKi = 2048;			// for 20khz pwm interrupt
	m0Q.qKc = m0Q.qKi;		// (a)symmetrically reduce integral term
	m0Q.NKo = -4;			// Scales up terms that may be too small
	m0Q.qOutMax = 30000;		// Quick hack instead of normalising output voltages to < Vbatt
	m0Q.qOutMin = -30000;

	m0D.qFF = 8191;//m0Q.qFF;
	m0D.qKp = 16384; //m0Q.qKp;
	m0D.qKi = 2048;//m0Q.qKi;
	m0D.qKc = m0D.qKi;
	m0D.NKo = m0Q.NKo;		//(Kp * Err * 2^NKo)
	m0D.qOutMax = m0Q.qOutMax;
	m0D.qOutMin = m0Q.qOutMin;
	m0D.qInRef = 0;			// no field weakening by default
   	
	// Motor 1 same as motor 0
	m1Q.qFF = m0Q.qFF;
	m1Q.qKp = m0Q.qKp;
	m1Q.qKi = m0Q.qKi;
	m1Q.qKc = m0Q.qKc;
	m1Q.NKo = m0Q.NKo;
	m1Q.qOutMax = m0Q.qOutMax;
	m1Q.qOutMin = m0Q.qOutMin;

	m1D.qFF = m0D.qFF;
	m1D.qKp = m0D.qKp;
	m1D.qKi = m0D.qKi;
	m1D.qKc = m0D.qKc;
	m1D.NKo = m0D.NKo;
	m1D.qOutMax = m0D.qOutMax;
	m1D.qOutMin = m0D.qOutMin;

	// Load motor calibration data
	if (!LoadCalibration(&m0, 0))
		if (uart_mode == TERMINAL)
			fprintf(U1, "M0 calibration data load fail, calibration required \n\r");
	
	if (!LoadCalibration(&m1, 1))
		if (uart_mode == TERMINAL)
			fprintf(U1, "M1 calibration data load fail, calibration required \n\r");
}


uint16_t initialisation(void)
{
	BLUE_LED_ON;
	ORANGE_LED_ON;
	RED_LED_ON;

	// Initialise the hardware
	init_hardware();

	// Load default firmware values
	init_firmware();
	
	if (uart_mode == TERMINAL) delay_ms(1000);

	BLUE_LED_OFF;
	//ORANGE_LED_OFF; //orange is default "on" indicator
	RED_LED_OFF;

	// Return Non-zero if any error
	// No checks at the moment, always ok!
	// Intend to check sensors, at least for the motors.
	// e.g. noise level in signals should be within normal range
	return (0);
}
