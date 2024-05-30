void debug_plot_dspfoc(void)
{
	int16_t x;
   	
	//Start of frame
	uart_transmit_buffer[0] = 0x55;
	uart_transmit_buffer[1] = 0x56;
  	
	x = m0Q.qInRef;
	uart_transmit_buffer[2] = make8(x,0);
	uart_transmit_buffer[3] = make8(x,1);

	x = ADCBUF11 - ADC_FRC_MIDPOINT;
	uart_transmit_buffer[4] = make8(x,0);
	uart_transmit_buffer[5] = make8(x,1);

	x = ADCBUF10 - ADC_FRC_MIDPOINT;
	uart_transmit_buffer[6] = make8(x,0);
	uart_transmit_buffer[7] = make8(x,1);

	x = ADCBUF9  - ADC_FRC_MIDPOINT;
	uart_transmit_buffer[8] = make8(x,0);
	uart_transmit_buffer[9] = make8(x,1);

	uart_transmit_buffer[10] = make8(m0.qIa,0);
	uart_transmit_buffer[11] = make8(m0.qIa,1);

	uart_transmit_buffer[12] = make8(m0.qIb,0);
	uart_transmit_buffer[13] = make8(m0.qIb,1);

	uart_transmit_buffer[14] = make8(m0.qIq,0);
	uart_transmit_buffer[15] = make8(m0.qIq,1);

	uart_transmit_buffer[16] = make8(m0.qId,0);
	uart_transmit_buffer[17] = make8(m0.qId,1);

	uart_transmit_buffer[18] = make8(m0.qVq,0);  //qVq from PI
	uart_transmit_buffer[19] = make8(m0.qVq,1);
  	
	uart_transmit_buffer[20] = make8(m0.qVd,0);  //qVd from PI
	uart_transmit_buffer[21] = make8(m0.qVd,1);

	uart_transmit_buffer[22] = make8(m0.qValpha,0);
	uart_transmit_buffer[23] = make8(m0.qValpha,1);
  	
	uart_transmit_buffer[24] = make8(m0.qVbeta,0);
	uart_transmit_buffer[25] = make8(m0.qVbeta,1);
  	
	uart_transmit_buffer[26] = make8(m0.qVr1,0);
	uart_transmit_buffer[27] = make8(m0.qVr1,1);
  	
	uart_transmit_buffer[28] = make8(m0.qVr2,0);
	uart_transmit_buffer[29] = make8(m0.qVr2,1);
  	
	uart_transmit_buffer[30] = make8(m0.qVr3,0);
	uart_transmit_buffer[31] = make8(m0.qVr3,1);
  	
	uart_transmit_buffer[32] = make8(m0.dPWM1,0);
	uart_transmit_buffer[33] = make8(m0.dPWM1,1);
  	
	uart_transmit_buffer[34] = make8(m0.dPWM2,0);
	uart_transmit_buffer[35] = make8(m0.dPWM2,1);
  	
	uart_transmit_buffer[36] = make8(m0.dPWM3,0);
	uart_transmit_buffer[37] = make8(m0.dPWM3,1);
  	
	uart_transmit_buffer[38] = make8(m0.theta,0);
	uart_transmit_buffer[39] = make8(m0.theta,1);

	uart_transmit_buffer[40] = make8(m0.omega,0);
	uart_transmit_buffer[41] = make8(m0.omega,1);
  	
	//Last parameter is (total bytes to send - 1)
	dma_start(2, DMA_ONE_SHOT | DMA_FORCE_NOW, uart_transmit_buffer, 41);
}


void debug_plot_calibration(void)
{
	int16_t x;

	//Start of frame
	uart_transmit_buffer[0] = 0x55;
	uart_transmit_buffer[1] = 0x56;

	uart_transmit_buffer[2] = make8(m0.theta,0);
	uart_transmit_buffer[3] = make8(m0.theta,1);

	uart_transmit_buffer[4] = make8(m0.omega,0);
	uart_transmit_buffer[5] = make8(m0.omega,1);

	uart_transmit_buffer[6] = 0;
	uart_transmit_buffer[7] = 0;
   	
	x = m0.theta_lut[m0.theta >> THETA_LUT_BITS];
	uart_transmit_buffer[8] = make8(x,0);
	uart_transmit_buffer[9] = make8(x,1);

	dma_start(2, DMA_ONE_SHOT | DMA_FORCE_NOW, uart_transmit_buffer, 9);
}


void debug_plot_sensors(void)
{
	int16_t x;

	//Start of frame
	uart_transmit_buffer[0] = 0x55;
	uart_transmit_buffer[1] = 0x56;

	x = perf_counter;
	uart_transmit_buffer[2] = make8(x,0);
	uart_transmit_buffer[3] = make8(x,1);

	uart_transmit_buffer[4] = make8(s0.motor_temperature,0);
	uart_transmit_buffer[5] = make8(s0.motor_temperature,1);

	uart_transmit_buffer[6] = make8(s1.motor_temperature,0);
	uart_transmit_buffer[7] = make8(s1.motor_temperature,1);

	uart_transmit_buffer[8] = make8(s0.mosfet_temperature,0);
	uart_transmit_buffer[9] = make8(s0.mosfet_temperature,1);

	uart_transmit_buffer[10] = make8(s1.mosfet_temperature,0);
	uart_transmit_buffer[11] = make8(s1.mosfet_temperature,1);

	x = battery_volts;
	uart_transmit_buffer[12] = make8(x,0);
	uart_transmit_buffer[13] = make8(x,1);

	x = analog_ch15;
	uart_transmit_buffer[14] = make8(x,0);
	uart_transmit_buffer[15] = make8(x,1);
   	
	dma_start(2, DMA_ONE_SHOT | DMA_FORCE_NOW, uart_transmit_buffer, 15);
}
