// Tasks running in the background of the main loop
// and not called from an interrupt


void read_temperatures(void)
{	
	// #word ADCBUF12	=			0x0358 //motor0 temperature
	// #word ADCBUF13	=			0x035A //motor1 temperature
	// #word ADCBUF14	=			0x035C //Vbatt or unused
	// #word ADCBUF15	=			0x035E //spare adc port
	// #word ADCBUF16	=			0x0360 //motor0 mosfet temperature
	// #word ADCBUF17	=			0x0362 //motor1 mosfet temperature

	uint16_t buffer[4] = {0};

	buffer[0] = ADCBUF12;
	buffer[1] = ADCBUF13;
	buffer[2] = ADCBUF16;
	buffer[3] = ADCBUF17;

	// ADC to degC for TI LMT86 temperature sensors
	for (uint16_t i = 0; i < 4; i++)
	{
		buffer[i] >>= 6; // shift down to standard 10bit ADC (0..1023)
		buffer[i]  *= (-29);
		buffer[i]  += 19015;
		buffer[i]  /= 100;
	}

	// now in degC
	s0.motor_temperature  = (int16_t)filterIIR((q15_t)buffer[0], (q15_t)s0.motor_temperature,  4096); 
	s1.motor_temperature  = (int16_t)filterIIR((q15_t)buffer[1], (q15_t)s1.motor_temperature,  4096); 
	s0.mosfet_temperature = (int16_t)filterIIR((q15_t)buffer[2], (q15_t)s0.mosfet_temperature, 4096); 
	s1.mosfet_temperature = (int16_t)filterIIR((q15_t)buffer[3], (q15_t)s1.mosfet_temperature, 4096);

	 // for now leave in Q15
	analog_ch15   = (int16_t)filterIIR((q15_t)(ADCBUF15 - ADC_FRC_MIDPOINT), (q15_t)analog_ch15,  4096); 
	battery_volts = (int16_t)filterIIR((q15_t)(ADCBUF14 - ADC_FRC_MIDPOINT), (q15_t)battery_volts,4096); 
}


void systemStateEngine(void)
{
	// switch state according to current state
	switch(system_state)
	{
		case(RUN):
		{
			if (spi.offline_counter >= 20) //20ms -> 20 packet transfers
			{
				// Zero any commands
				m0Q.qInRef	= 0;
				m1Q.qInRef	= 0;
				m0D.qInRef	= 0;
				m1D.qInRef	= 0;
				m0.zeta		= 0;
				m1.zeta		= 0;

				// if loaded motors fail ~gracefully but only after enough time
				// for FOC control to zero everything correctly, otherwise damping
				// current continues to be added to commanded current and motors
				// never stop spinning!
				if (spi.offline_counter >= 30)
				{
					m0.zeta		= 3000;
					m1.zeta		= 3000;
				}

				// reset SPI checksums
				spi.chksum_rx = 0;
				spi.chksum_verify = 0;

				// Update system state 
				system_state = SPI2_OFFLINE;
			}
		} break;

		case(SPI2_OFFLINE):
		{
			if (spi.offline_counter < 20)
				system_state = RUN;
		} break;

		case(CALIBRATION):
		{
			// should not reach this point
		} break;

		case(FATAL_ERROR):
		{
			// TBC
		} break;

		default: {} break;
	}
}


void uartUIprint(void)
{
	switch (uart_mode)
	{
		case(TERMINAL):
		{
			if (uart_timer > 199)  
			{
				print_term_variables();
				uart_timer = 0;
			}
		} break;

		case(BINARY):
		{
			if (uart_timer > 1)
			{
				debug_plot_dspfoc();
				//debug_plot_calibration();
				//debug_plot_sensors();
				uart_timer = 0;
			}
		} break;

		default: {} break;
	}
}
