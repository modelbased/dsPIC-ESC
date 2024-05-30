// UTILITY FUNCTIONS THAT ARE NOT INTERRUPT SAFE

// Function to add two signed integers with _intentional_ wrapping, i.e. 
// with exactly the same output as a + b, but without triggering the 
// math error trap for an overflow
int16_t AddInt16(int16_t a, int16_t b) {
   int32_t x;
   
   x = (int32_t)a + (int32_t)b;
   if (x > INT16_MAX) {
      return (int16_t)(x - UINT16_MAX);
   } else if (x < INT16_MIN) {
      return (int16_t)(x + UINT16_MAX);
   } else {
      return (int16_t)x;
   }
}


void readADCManually(void)
{
	// manual trigger for ADCs with GLOBAL_SOFTWARE_TRIGGER
	read_high_speed_adc();
}


// Alternative to q15_mul() using accumulator
q15_t dspMpy(q15_t n, q15_t m)
{
    static q15_t r;

	// Acc state not saved, ensure any interrupt
	// does save to avoid conflict
    #asm
    mov			n, W4
    mov			m, W5
    mpy			W4*W5, A
    sac.r		A, W0
    mov			w0, r
    #endasm

    return (r);
}


// Faster than q15_div(), does not use Accumualtors
q15_t dspDiv(q15_t num, q15_t den)
{
    static q15_t r;

    #asm
    mov 		num, w2
    mov 		den, w3

	repeat		#17
    divf		w2, w3
	bra OV,		_saturate	// overflow = saturation
	goto		_end		// no overflow = done

	_saturate:
	bra N,		_saturate_neg
	mov			#0x7FFF,w0	// saturate +32767
	goto		_end

	_saturate_neg:
	mov			#0x8000, w0	// saturate -32768

	_end:
	mov w0, r
 
	#endasm
    
    return (r);
}


// Input: sample[n], filtered[n-1], filter level
// Output: updated filtered[n] (var is updated)
// If no constant passed default 0.5 is used
q15_t filterIIR(q15_t sample, q15_t filtered, q15_t level = 16384)
{  	
	// In Q15:
	// flt_new = q15_add(flt_old, q15_mul(k, q15_add(sample, q15_neg(flt_old))));

	// Accumulator must be in saturation mode, which it should
	// flt_new = flt_old + (sample - flt_old) * k;

    #asm
	// save AccA & CORCON
	// push		CORCON
	// push		0x0022		// save AccAL
	// push		0x0024		// save AccAH
	// push		0x0026		// save AccAU

    // load variables
	mov			sample, w4		//w4 = sample
    mov			filtered, w5	//w5 = old
	mov			level, w6		//w6 = filter level
    
	// delta
	sub			w4,w5,w4	//w4 = delta
	
	// reduce and add
	mpy			w4*w6, A	//A = delta * constant
    add			w5,A		//A = A + old

	// store accumulator
	sac.r		A, w0		// store rounded result
    mov			w0, filtered

	// restore saved registers
	// pop			0x0026		
	// pop			0x0024		
	// pop			0x0022		
	// pop			CORCON

    #endasm
    
    return(filtered);
}


// Moving average filter
// Input: new sample
// Output: updated average
// Because values are stored internally
// use this function by creating a new copy
// for each filtered value. Uses 32b, so it is slow.
q15_t FIRStep(q15_t sample)
{
	#define FIR1SIZE	3
	static uint16_t pointer = 0; 
	static int32_t	buffer[FIR1SIZE] = {0};
	static int32_t	sum = 0;
	q15_t r = 0;

	sum -= buffer[pointer];
	buffer[pointer] = sample;
	sum += sample;

	r = (sum / FIR1SIZE);
	
	pointer++;
	if (pointer >= FIR1SIZE) pointer = 0;

	return (r);
}


//Adler32 checksum
//Input: int16 array & length to checksum over
//This algorithm is optimised and not suitable for arrays > 359 length
//For larger arrays the mod operator should be performed more frequently
//See wikipedia implementation of fletcher32 (similar to Adler32) for ref.
uint32_t checksum(uint16_t data, uint16_t len)
{
   uint32_t c0, c1;
   uint16_t i;
     
   c0=1;
   c1=0;
     
   for (i = 0; i < len; ++i) {
          c0 = c0 + *data++;
          c1 = c1 + c0;
   }
   c0 = c0 % 65521;
   c1 = c1 % 65521;
   return (c1 << 16 | c0);
}


void StoreCalibration(motor_state_t *m, uint16_t mID)
{
	// lut is in int16, whereas buffer is int8
	// note also using 4 bytes to store 1 word
	uint8_t buffer[(THETA_LUT_SIZE * 4)] = {0};
	uint32_t cx = 0;

	// allocate variables to store into write buffer
	// skipping every 4th byte because, this is a PIC
	for (uint16_t i = 0; i < THETA_LUT_SIZE; i++)
	{
		buffer[(i * 4) + 0] = make8(m->theta_lut[i],0); //byte 0 LSB
		buffer[(i * 4) + 1] = make8(m->theta_lut[i],1); //byte 1 MSB
		buffer[(i * 4) + 2] = 0; // ignore. inneficient, but simpler
		buffer[(i * 4) + 3] = 0; // cannot write to this byte, always returns zero
	}

	// pack in a few other variables into the third bytes of the buffer
	buffer[(0 * 4) + 2] = m->phase_parity;
	buffer[(1 * 4) + 2] = make8(m->theta_offset, 0); //LSB
	buffer[(2 * 4) + 2] = make8(m->theta_offset, 1); //MSB
	buffer[(3 * 4) + 2] = make8(m->qIaOff, 0);
	buffer[(4 * 4) + 2] = make8(m->qIaOff, 1);
	buffer[(5 * 4) + 2] = make8(m->qIbOff, 0);
	buffer[(6 * 4) + 2] = make8(m->qIbOff, 1);
	buffer[(7 * 4) + 2] = make8(m->qIcOff, 0);
	buffer[(8 * 4) + 2] = make8(m->qIcOff, 1);

	// calc then store checksum
	cx = checksum(buffer,(THETA_LUT_SIZE * 4));
	buffer[(9  * 4) + 2] = make8(cx,0);
	buffer[(10 * 4) + 2] = make8(cx,1);
	buffer[(11 * 4) + 2] = make8(cx,2);
	buffer[(12 * 4) + 2] = make8(cx,3);

	// erases FLASH_ERASE_SIZE block (1536 bytes) from start address
	// write_program_memory() should erase any subsequent blocks as needed
	// M0 stored in first flash block, m1 stored in second flash block
	erase_program_memory(FLASH_START_ADDRESS + (mID * (FLASH_SIZE_RESERVED/2)));
   	
	// write the buffer to program flash
	write_program_memory(FLASH_START_ADDRESS + (mID * (FLASH_SIZE_RESERVED/2)), buffer, (THETA_LUT_SIZE * 4));
}


uint8_t LoadCalibration(motor_state_t *m, uint16_t mID)
{
	uint8_t buffer[(THETA_LUT_SIZE * 4)] = {0};
	uint32_t cx_loaded	= 0;
	uint32_t cx_calc	= 0;
   	
	read_program_memory(FLASH_START_ADDRESS + (mID * (FLASH_SIZE_RESERVED/2)), buffer, (THETA_LUT_SIZE * 4));

	// extract checksum and clear it from buffer
	cx_loaded = make32(	buffer[(12 * 4) + 2],	   	
						buffer[(11 * 4) + 2],
						buffer[(10 * 4) + 2],
						buffer[(9  * 4) + 2]);

	buffer[(12 * 4) + 2] = 0;
	buffer[(11 * 4) + 2] = 0;
	buffer[(10 * 4) + 2] = 0;
	buffer[(9  * 4) + 2] = 0;
   	
	cx_calc = checksum(buffer,(THETA_LUT_SIZE * 4));

	if (cx_calc != cx_loaded) return(FALSE);	// stop loading if checksum fails

	// load lut
	for (uint16_t i = 0; i < THETA_LUT_SIZE; i++)
	{
		// use bytes 0 and 1, ignore 2 and 3 (see storage function)
		m->theta_lut[i] = make16(buffer[(i * 4) + 1], buffer[(i * 4) + 0]);
	}

	// load other variables
	m->phase_parity = buffer[(0 * 4) + 2];
	m->theta_offset = make16(buffer[(2 * 4) + 2], buffer[(1 * 4) + 2]);
	m->qIaoff		= make16(buffer[(4 * 4) + 2], buffer[(3 * 4) + 2]);
	m->qIbOff		= make16(buffer[(6 * 4) + 2], buffer[(5 * 4) + 2]);
	m->qIcOff		= make16(buffer[(8 * 4) + 2], buffer[(7 * 4) + 2]);
   	
	return(TRUE);
}


// Check it works!
void testDSPvsQ15Maths(void)
{

	int16_t i, j;
	q15_t a,b,x,y, errmul, errdiv;

	for(i = -10000; i < 32760; i+=1003)
	{
		for(j = -10000; j < 32760; j+=1001)
		{
			if (j != 0)
			{
				a = q15_mul(i,j);
				b = q15_div(i,j);
				x = dspmpy(i,j);
				y = dspdiv(i,j);
   	
				errmul = a - x;
				errdiv = b - y;
   	
				if (abs(errmul) > 1 || abs(errdiv) > 50)
				{
					fprintf(U1,"I=%d, J=%d, Q15MUL %d | DSPMUL %d   ||   Q15DIV %d | DSPDIV %d DIV ERROR = %d \n\r",i,j,a,x,b,y,errdiv);
					delay_ms(10);
				}
				else
				{
					if ((i % 1000 == 0) && (j % 5000 == 0)) 
					{
						fprintf(U1, "I=%d, J=%d, No errors \n\r",i,j);
					}
				}
			}
		}
	}
}



void TestOneComplementChecksum(void)
{
   	
	static uint16_t data[] = {0,1,2,3};
   	
	static uint16_t len;
	static uint16_t *p;
	static uint32_t sum;
   	
	len = 4;
	p = &data[0];
	sum = 0;
   	
	data[0] += 1000;
	data[1] += 1000;
	data[2] += 1000;
	data[3] += 1000;
		
	
	while(len--)  //len = Total num of bytes
	{
    	sum += (uint32_t)(*p++);
	}

	sum = (sum >> 16) + sum; //Add the carryout
	
	uint16_t result1 = ~((uint16_t)sum);
   	

	// ASM VERSION
	len = 4;
	p = &data;
	uint16_t sum2;
	sum2 = 0;

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
	mov		w0, sum2
	
	#endasm
   	
	uint16_t result2 = sum2;

	fprintf(U1, "C = [%u] | ASM = [%u] | Delta = [%d] \n\r",result1, result2, result1 - result2);
}
