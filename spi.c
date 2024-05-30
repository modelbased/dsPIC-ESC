#inline
void reset_spi(void) //traditional solutions are usually best
{
	disable_interrupts(INT_DMA0);
		
	SPI2EN  = 0;   //disable SPI
	SPI2BUF = 0;   //clear buffer
	SPI2EN  = 1;   //enable SPI
		
	//Set up everything again or it wont work.
		
	//setup DMA on SPI2 SLAVE, Master comms
	setup_dma(0, DMA_IN_SPI2, DMA_WORD);
	setup_dma(1, DMA_OUT_SPI2, DMA_WORD);
		
	//Setup SPI slave must be enabled AFTER the DMA is setup
	setup_spi2(SPI_SLAVE | SPI_MODE_0 | SPI_MODE_16B);
		
	//enable DMA device operation
	// DMA Device Type = 1 therefore (SPI_DMA_BUFFER_SIZE - 1)
	dma_start(0, DMA_PING_PONG | DMA_CONTINOUS, &spi_rx_a[0], &spi_rx_b[0], SPI_DMA_BUFFER_SIZE - 1);
	dma_start(1, DMA_PING_PONG | DMA_CONTINOUS, &spi_tx_a[0], &spi_tx_b[0], SPI_DMA_BUFFER_SIZE - 1);
		
	enable_interrupts(INT_DMA0);
}

/**
 * Checksum for SPI packets based on the sum addition
 * algoritm used for IPv4 packet headers. This is  
 * faster than fletcher/adler/crc whilst providing
 * sufficient error robustness.
 * https://en.wikipedia.org/wiki/IPv4_header_checksum
 * */
#inline
void ChkSumRXSPI(void)
{
	static uint16_t len;
	static uint16_t *p;
	static uint16_t sum;
   	
	len = 19;			// lenght of data packet in 16bit words
	p = &spi_rx_a[0];	// pointer to data packet
	sum = 0;			// initialise the accumulator

	// Select which of the buffers is the correct one at the moment
	if (spi.buffer_bank == 0) 
		p = &spi_rx_a[0]; //buffer a
	else 
		p = &spi_rx_b[0]; //buffer b


	// This asm implementation uses the overflow carry bit
	// so that the accumulator can be 16bit instead of using
	// a slower 32bit accumulator that C would require	
	#asm
	clr		w0				// w0 will be the sum accumulator
	mov		p, w2			// w2 = pointer to data array
	mov		len, w1			// w1 = lenght of data array

	_whileloop:
	cp0		w1				// same as while(len--){}
	bra Z,	_end

	// code in loop
	dec		w1, w1			// decrement loop counter
	add		w0, [w2++], w0	// w0 = w0 + p[]
	addc	w0, #0, w0		// if add overflowed the carry bit is added
	// end of loop


	bra		_whileloop
	_end:
	
	com		w0, w0			// w0 = ~w0 
	mov		w0, sum			// move out to var
	
	#endasm
	
	spi.chksum_verify = sum;
}


#inline
void ChkSumTXSPI(void)
{
	static uint16_t len;
	static uint16_t *p;
	static uint32_t sum;
   	
	len = 19; // words 0 to 18
	p = &spi_tx_a[0];
	sum = 0;
   	
	if (spi.buffer_bank == 0) 
		p = &spi_tx_a[0]; //buffer a
	else 
		p = &spi_tx_b[0]; //buffer b
		
	#asm
	clr		w0
	mov		p, w2
	mov		len, w1

	_whileloop:
	cp0		w1
	bra Z,	_end

	// code in loop
	dec		w1, w1
	add		w0, [w2++], w0
	addc	w0, #0, w0
	// end of loop


	bra		_whileloop
	_end:
	
	com		w0, w0
	mov		w0, sum
	
	#endasm
	
	spi.chksum_tx = sum;
}


#inline
void spi_process_incoming(void)
{
	static uint16_t *buffer_to_process_ptr = &spi_rx_a[0]; //point to something by default
	static uint16_t i;
     
	//select correct buffer
	if (spi.buffer_bank == 0) 
		buffer_to_process_ptr = &spi_rx_a[0];	//buffer a
	else 
		buffer_to_process_ptr = &spi_rx_b[0];	//buffer b
   	
	if (buffer_to_process_ptr[0] == 0x000A)		// First word from master must always be 0x0A
	{ 
		ChkSumRXSPI(); //Calc own checksum to verify incoming packet
		spi.chksum_rx = buffer_to_process_ptr[21];
		
		if (spi.chksum_rx == spi.chksum_verify)
		{
			// Indicators
			BLUE_LED_ON;
			//spi.bad_packets = 0; // comment out to keep a running tally

			// Work round since compiler bug means we cannot pass variables to functions 
			// safely inside nested interrupts
			buffer_to_process_ptr[3] = (buffer_to_process_ptr[3] * 100);
			buffer_to_process_ptr[4] = (buffer_to_process_ptr[4] * 100);
			static q15_t num;
			static q15_t den;
			static q15_t r;
			for (i = 1; i < 5; i++)
			{
				num = buffer_to_process_ptr[i];
				den = Q15_SCALE_CURRENT;
				r	 = 0;
				#asm						// divSPI() fractional division
				mov		num, w2
				mov		den, w3
				repeat		#17
				divf		w2, w3
				bra OV,		_saturate		// overflow = saturation
				goto		_end			// no overflow = done
				_saturate:
				bra N,		_saturate_neg
				mov			#0x7FFF,w0		// saturate +32767
				goto		_end
				_saturate_neg:
				mov			#0x8000, w0		// saturate -32768
				_end:
				mov w0, r
				#endasm
				buffer_to_process_ptr[i] = r;
			}
			m0Q.qInRef = buffer_to_process_ptr[1];
			m1Q.qInRef = buffer_to_process_ptr[2];
			m0.qIq_limit = buffer_to_process_ptr[3]; 
			m1.qIq_limit = buffer_to_process_ptr[4];


			//Current command in centiAmps converted to Q15
			// m0Q.qInRef = divSPI(buffer_to_process_ptr[1], Q15_SCALE_CURRENT);
			// m1Q.qInRef = divSPI(buffer_to_process_ptr[2], Q15_SCALE_CURRENT);
			
			// Current limit can be set below HARD_CURRENT_LIMIT
			// Amps to cAmps to Q15
			// m0.qIq_limit = divSPI((buffer_to_process_ptr[3] * 100), Q15_SCALE_CURRENT); 
			// m1.qIq_limit = divSPI((buffer_to_process_ptr[4] * 100), Q15_SCALE_CURRENT);
			
			// Negative value creates positive feedback and is not allowed
			// Buffer array elements are uint_t so negative numbers are >32767
			if (buffer_to_process_ptr[5] < 32767) m0.zeta = buffer_to_process_ptr[5];
			else m0.zeta = 0;

			if (buffer_to_process_ptr[6] < 32767) m1.zeta = buffer_to_process_ptr[6];
			else m1.zeta = 0;
		}
		else
		{
			//spi.chksum_rx = 0;	// don't zero these, keep for display
			//spi.chksum_verify = 0;
			spi.bad_packets++;
			//reset_spi(); // if checksum fails reset likely not the solution
		}
	}
	else
	{
		spi.chksum_rx = 0;
		spi.chksum_verify = 0;
		spi.bad_packets++;
		reset_spi(); // forces a comms re-sync, can become misaligned
	}
}


#inline
void spi_loadout(void)
{
	static uint16_t *buffer_to_process_ptr = &spi_tx_a[0]; //point to something by default
		
	//select correct buffer
	if (spi.buffer_bank == 0) 
		buffer_to_process_ptr = &spi_tx_a[0];	//buffer a
	else
		buffer_to_process_ptr = &spi_tx_b[0];	//buffer b


	// Compiler nested interrupt variables bug workaround, cannot use
	// mulSPI() as vars passed to function become corrupted in nested interrupts 
	static q15_t n;
	static q15_t m;
	static q15_t r;
	n = m0.qIq;
	m = Q15_SCALE_CURRENT;
	r = 0;
	#asm
	mov		m, w0
	mov		n, w1
	mul.ss	w0, w1, w2	//w3:w2 = w1 * w0
	rlc		w2, w2	
	rlc		w3, w3		//w0 = (w3:w2) >> 15
	mov		w3, r
	#endasm
	buffer_to_process_ptr[0] = r;

	n = m1.qIq;
	m = Q15_SCALE_CURRENT;
	r = 0; 
	#asm
	mov		m, w0
	mov		n, w1
	mul.ss	w0, w1, w2	//w3:w2 = w1 * w0
	rlc		w2, w2	
	rlc		w3, w3		//w0 = (w3:w2) >> 15
	mov		w3, r
	#endasm
	buffer_to_process_ptr[1] = r;


	// buffer_to_process_ptr[0] = mulSPI(m0.qIq, Q15_SCALE_CURRENT);	//centiAmps
	// buffer_to_process_ptr[1] = mulSPI(m1.qIq, Q15_SCALE_CURRENT);	//centiAmps
		
	buffer_to_process_ptr[2] = s0.motor_temperature;				//degC
	buffer_to_process_ptr[3] = s1.motor_temperature;

	buffer_to_process_ptr[4] = s0.mosfet_temperature;				//degC
	buffer_to_process_ptr[5] = s1.mosfet_temperature;
		
	buffer_to_process_ptr[6] = s0.encoder;							// for joint absolute sensing
	buffer_to_process_ptr[7] = s1.encoder;

	buffer_to_process_ptr[8] = m0.omega;							// theta1 - theta0 per ~1ms
	buffer_to_process_ptr[9] = m1.omega;

	buffer_to_process_ptr[10] = m0.zeta_current;					// was m0 encoder velocity
	buffer_to_process_ptr[11] = m1.zeta_current;					// was m1 encoder velocity

	buffer_to_process_ptr[12] = m0.step_count;						// 20e-revs * 6steps per e-revs = 120 steps per motor revs
	buffer_to_process_ptr[13] = m1.step_count;

	buffer_to_process_ptr[14] = battery_volts;						// Unscaled Q15

	buffer_to_process_ptr[15] = m0.theta;							// CORDIC where 65535 = 2pi
	buffer_to_process_ptr[16] = m1.theta; 

	buffer_to_process_ptr[17] = 0;									// Unused
	buffer_to_process_ptr[18] = 0;									// Unused

	ChkSumTXSPI();
	buffer_to_process_ptr[19] = spi.chksum_tx;						// Checksum word
	buffer_to_process_ptr[20] = 0;									// Guard padding, must be zero
}
