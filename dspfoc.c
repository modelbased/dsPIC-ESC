// Same FOC algorith principles as C based foc.c
// using the dsPICs dsp engine and assembly where it helps


q16angle_t dspAtan2(q15_t q, i)
{
	static q16angle_t result;
	
	//constant definitions
	//NEG_PI_BY_2		0xC000
	//PI_BY_2			0x3FFF
	//PI				0x7FFF
	//NEG_PI			0x8000
	
	// in this table are the values of atan(n) scaled into the range 0 to PI
	// and then converted into fractional format
	static q15_t	cordic_data[] = 
	{
		0x2000,	// atan(1.0) / PI
		0x12E4,	// atan(0.5) / PI
		0x09FB,	// atan(0.25) / PI
		0x0511,	// atan(0.125) / PI
		0x028B,	// atan(0.0625) / PI
		0x0146,	// etc
		0x00A3,
		0x0051,
		0x0029,
		0x0014,
		0x000A,
		0x0005,
		0x0003,
		0x0001,
		0x0001,
		0x0000
	};

	// _atan2CORDIC: Perform an inverse tangent operation using
	// CORDIC iterative approximation. The parameters are assumed
	// to be dsPIC fractionals and the return value is a dsPIC fractional
	// with a value from -PI to PI scaled into the range -1.0 to 0.999
	// (to allow full scale use of the dsPIC fractional type)

	// On entry:
	// W0 = q (sin)
	// W1 = i (cos)
	// On Exit:
	// W0 = atan2(q, i)
	// Uses:
	// ACCA = rotated I value during calculation
	// ACCB = rotated Q value during calculation
	// 1 word on stack to store temporary I value
	// w4 = current phase offset
	// w5 = K (series divider 1.0, 0.5, 0.25 etc)
	// w6 = Q value (16 bit version of ACCB)
	// w7 = Accumulated phase
	// w8 = Pointer to CORDIC table
	// w9 = Pointer to storage word on stack

	// A conventional CORDIC routine can achieve its result by
	// just using shifts and adds. However this routine takes
	// advantage of the DSP mac and msc to achieve the same result
	// it also relies on the single cycle preload architecture to
	// get parameters for the next operation into place.
	// There are bounds checking for y = -1 and x = -1 other than
	// that the data is assumed to be correctly scaled and normalized
	// in the range -1.0 to 0.999 (0x8000 to 7FFF)
	// Apart from at the extrema the results agree with those returned
	// by the standard atan2 library routine to within 0.05 degrees

	// Total cycle count is 160 cycles for any rational value
	#asm

	mov			q, w0
	mov			i, w1

	lnk			#0x02				// reserve 2 bytes for storage
	push		w8
	push		w9
	//push		CORCON				// with interrup using AccB may be necessary?
	
	// check for values that cause errors 
	// due to asymptotic atan behaviour
	mov			#0x8000, w8
	cp			w0, w8				// Q = -1.0 ?
	bra NZ,		_checkI
	mov			#0xC000, w0
	bra			_exitCORDICRoutine

	_checkI:
	cp			w1, w8				// I = -1.0
	bra NZ,		_mainCORDICRoutine
	mov			#0x7FFF, w0
	bra			_exitCORDICRoutine

	_mainCORDICRoutine:
	// set w9 to point to the reserved 2 byte space
	// this can then be used to preload w6 in the dsp MACs
	mov			w14, w9		
	// ACCUM_PHASE (w7) is the total phase angle calculated
	clr			w7	

	// adjust q and i to be in quadrant I
	cp0			w1
	bra NN,		_setupIter
	mov			w1, [w9]			// w2 = temporary I
	cp0			w0
	bra LE,		_quadIII
	mov			w0, w1
	neg			[w9], w0
	mov			#0xC000, w7
	bra			_setupIter

	_quadIII:
	neg			w0, w1
	mov			[w9], w0
	mov			#0x3FFF, w7

	_setupIter:
	// set ACCA and ACCB to equal I and Q	
	lac			w0, #1, B
	lac			w1, #1, A
	
	mov			cordic_data, w8		// w8 points to CORDIC data table
	mov			#0x7FFF, w5			// w5 = K = 1.0

	do			#9, _endCORDICRoutine
	sac.r		A, [w9]				// put I onto local stack
	sac.r		B, w6				// w6 = Q
	cp0			w6					// if Q < 0 goto rotate positive
	bra N,		_rotate_pos

	_rotate_neg:
	mac			w5*w6, A			// I = I + Q * K, w6 = temp I
	mov			[w9], w6
	msc			w5*w6, B			// Q = Q - oldI * K
	mov			[w8++], w4
	subbr		w4, w7, w7
	bra			_endCORDICRoutine

	_rotate_pos:
	msc			w5*w6, A			// I = I - Q * K, w6 = temp I
	mov			[w9], w6
	mac			w5*w6, B			// Q = Q + oldI * K
	mov			[w8++], w4
	add		w4, w7, w7
	
	_endCORDICRoutine:
	lsr			w5, w5				// K = K / 2

	neg			w7, w0				// reverse the sign
   	
	_exitCORDICRoutine:	
	//pop		CORCON
	pop			w9
	pop			w8
	ulnk
	
	mov			w0, result
	#endasm
   	
	return (result);
}


// Calc Sin and Cos from Angle
void dspSinCos(motor_state_t *m)
{
	static q15_t	sin_table[] = 
	{ 
		0,1608,3212,4808,6393,7962,9512,11039,
		12540,14010,15446,16846,18205,19520,20787,22005,
		23170,24279,25330,26319,27245,28106,28898,29621,
		30273,30852,31357,31785,32138,32413,32610,32728,
		32767,32728,32610,32413,32138,31785,31357,30852,
		30273,29621,28898,28106,27245,26319,25330,24279,
		23170,22005,20787,19520,18205,16846,15446,14010,
		12540,11039,9512,7962,6393,4808,3212,1608,
		0,-1608,-3212,-4808,-6393,-7962,-9512,-11039,
		-12540,-14010,-15446,-16846,-18205,-19520,-20787,-22005,
		-23170,-24279,-25330,-26319,-27245,-28106,-28898,-29621,
		-30273,-30852,-31357,-31785,-32138,-32413,-32610,-32728,
		-32767,-32728,-32610,-32413,-32138,-31785,-31357,-30852,
		-30273,-29621,-28898,-28106,-27245,-26319,-25330,-24279,
		-23170,-22005,-20787,-19520,-18205,-16846,-15446,-14010,
		-12540,-11039,-9512,-7962,-6393,-4808,-3212,-1608
	};

	// input
	static q16angle_t theta; theta = m->theta;

	// outputs
	static q15_t sin_theta, cos_theta;

	#asm
	// Calculate Index and Remainder for fetching and interpolating Sin
	mov				#128,w0				//sin_table size is 128
	mov				theta,w1			// load qAngle & inc ptr to qCos
	mul.uu			w0,w1,w2			// high word in w3

	// Double Index since offsets are in bytes not words
	add				w3,w3,w3

	// Note at this point the w3 register has a value 0x00nn where nn
	// is the offset in bytes from the TabBase.  If below we always
	// use BYTE operations on the w3 register it will automatically
	// wrap properly for a TableSize of 128.
	mov				sin_table,w5		// Pointer into table base

	// Check for zero remainder
	cp0				w2
	bra NZ,			_jInterpolate

	// Zero remainder allows us to skip the interpolation and use the 
	// table value directly
	add			w3,w5,w4
	mov			[w4],w7
	mov			w7,sin_theta			// write qSin & inc pt to qCos

	// Add 0x40 to Sin index to get Cos index.  This may go off end of
	// table but if we use only BYTE operations the wrap is automatic.
	add.b		#0x40,w3
	add			w3,w5,w4
	mov			[w4],w7
	mov			w7,cos_theta			// write qCos
	goto		_end

	_jInterpolate:
	// Get Y1-Y0 = SinTable[Index+1] - SinTable[Index]  
	add			w3,w5,w4
	mov			[w4],w6					// Y0

	inc2.b		w3,w3					// (Index += 2)&0xFF
	add			w3,w5,w4

	subr		w6,[w4],w0			// Y1 - Y0

	// Calcuate Delta = (Remainder*(Y1-Y0)) >> 16
	mul.us		w2,w0,w0

	// w1 contains upper word of (Remainder*(Y1-Y0)) 
	// *pSin = Y0 + Delta
	add			w1,w6,w7 
	mov			w7,sin_theta			// write qSin & inc pt to qCos

	// ================= COS =========================

	// Add 0x40 to Sin index to get Cos index.  This may go off end of
	// table but if we use only BYTE operations the wrap is automatic.
	// Actualy only add 0x3E since Index increment by two above
	add.b		#0x3E,w3
	add			w3,w5,w4

	// Get Y1-Y0 = SinTable[Index+1] - SinTable[Index]  
	add			w3,w5,w4
	mov			[w4],w6					// Y0

	inc2.b		w3,w3					// (Index += 2)&0xFF
	add			w3,w5,w4

	subr		w6,[w4],w0			// Y1 - Y0

	// Calcuate Delta = (Remainder*(Y1-Y0)) >> 16
	mul.us		w2,w0,w0

	// w1 contains upper word of (Remainder*(Y1-Y0)) 
	// *pCos = Y0 + Delta
	add			w1,w6,w7 
	mov			w7,cos_theta		// write qCos
   	
	_end:
	#endasm

	m->sin_theta = sin_theta;
	m->cos_theta = cos_theta;
}


void dspMeasureTheta(motor_state_t *m)
{
	static q15_t cos_est;
	static q15_t sin_est;

	// These angles do not refer to motor electrical phases
	// These are three linear hall sensors reading the 
	// rotor magnets, in the same location a six-step
	// latching hall sensor would be
	static q15_t angle_a; angle_a = m->angle_a;
	static q15_t angle_b; angle_b = m->angle_b;
	static q15_t angle_c; angle_c = m->angle_c;

	// Convert from Q16 ADC to -1 .. 0 .. +1 Q15
	//angle_a = (ADCBUF2 - ADC_FRC_MIDPOINT);
	//angle_b = (ADCBUF1 - ADC_FRC_MIDPOINT);
	//angle_c = (ADCBUF0 - ADC_FRC_MIDPOINT);
	
	// Clark transform (amplitude invariant)
	// cos_est = (2/3)A - (1/3)B - (1/3)C
	// sin_est = (sqrt3/3)B - (sqrt3/3)C

	#asm
	mov			#21845, w4	// (2/3)
	mov		#10923, w5	// (1/3)

	mov		angle_a, w7
	mpy		w4*w7, A	// A = (2/3)A
	mov			angle_b, w7
	msc			w5*w7, A	// A = A - (1/3)B
	mov			angle_c, w7
	msc			w5*w7, A	// A = A - (1/3)C
	sac			A, w6
	mov			w6, sin_est	// A -> w0 -> cos_est
	
	mov		#18918, w4	// (sqrt3/3)

	mov			angle_b, w7
	mpy			w4*w7, A	// A = (sqrt(3)/3)B
	mov		angle_c, w7
	msc			w4*w7, A	// A = A - (sqrt(3)/3)C
	sac			A, w7
	mov			w7, cos_est	// A -> w0 -> sin_est
    #endasm
    
	// subtract 90deg so cos_est/sin_est from clarke transform
	// yield same result as previous phase angle based on 
	// pseudo inverso matrix function
   	
	static q16angle_t  theta_raw; theta_raw = dspAtan2(sin_est, cos_est) - 16384 + m->theta_offset;
    m->theta =  theta_raw - m->theta_lut[(theta_raw >> THETA_LUT_BITS)]; 
    
    dspSinCos(m);
}



void dspClarkePark(motor_state_t *m)
{
	// inputs
	static q15_t qIa; qIa = m->qIa;
	static q15_t qIb; qIb = m->qIb;
	static q15_t qIc; qIc = m->qIc;
	static q15_t sin_theta; sin_theta = m->sin_theta;
	static q15_t cos_theta; cos_theta = m->cos_theta;

	// outputs
	static q15_t qIalpha, qIbeta;
	static q15_t qIq, qId;

	#asm

	// Clark transform (amplitude invariant)
	// Ialpha = (2/3)Ia - (1/3)Ib - (1/3)Ic
	// Ibeta  = (sqrt3/3)Ib - (sqrt3/3)Ic

	mov			#21845, w4	// (2/3)
	mov		#10923, w5	// (1/3)

	mov		qIa, w7
	mpy		w4*w7, A	// A = (2/3)Ia
	mov			qIb, w7
	msc			w5*w7, A	// A = A - (1/3)Ib
	mov			qIc, w7
	msc			w5*w7, A	// A = A - (1/3)Ic
	sac			A, w6
	mov			w6, qIalpha	// A -> w0 -> Ialpha
	
	mov		#18918, w4	// (sqrt3/3)

	mov			qIb, w7
	mpy			w4*w7, A	// A = (sqrt(3)/3)Ib
	mov		qIc, w7
	msc			w4*w7, A	// A = A - (sqrt(3)/3)Ic
	sac			A, w7
	mov			w7, qIbeta

	// Parke transform
	// Id =  Ialpha*cos(Angle) + Ibeta*sin(Angle)
	// Iq = -Ialpha*sin(Angle) + Ibeta*cos(Angle)

	mov		sin_theta,w4
	mov		cos_theta,w5

	//Id =	Ialpha*cos(Angle) + Ibeta*sin(Angle)
	mpy		w4*w7, A		// A = Ibeta * Sin
	mac		w5*w6, A		// A = A + Ialpha * Cos
	sac		A, w3
	mov		w3, qId

	//Iq = -Ialpha*sin(Angle) + Ibeta*cos(Angle)
	mpy		w5*w7,A			// A = Ibeta * Cos
	msc		w4*w6,A			// A = A - Ialpha * qSin
	sac		A,w3
	mov		w3,qIq	

	#endasm

	m->qIalpha	= qIalpha;
	m->qIbeta	= qIbeta;
	m->qIq		= qIq;
	m->qId		= qId;
}


void dspCalcDamping(motor_state_t *m)
{
	// inputs
	// damping = zeta_b * bemf * 2^k
	static q15_t k;		k		= BEMF_DAMP_K;	   	
	static q15_t zeta;	zeta	= m->zeta;
	static q15_t qVq;	qVq		= m->qVq;
	static q15_t bemf;	bemf	= m->bemf;
	static q15_t qIq;	qIq		= m->qIq;
   	
	// output
	static q15_t zeta_current;

	#asm

	// Calc and filter bemf
	// bemf estimate
	mov			qVq, w4
	lac			w4, A			// A = qVq
	mov			qIq, w7
	neg			w7,w5			// keep qIq unchanged for later
	add			w5, A			// A = A + (-qIq)
	sac.r		A, w4			// w4 = bemf estimate

	// load variables
    mov			bemf, w5		//w5 = filtered value
	mov			#1024, w6		//w6 = filter constant
    
	// delta
	sub			w4,w5,w4		//w4 = delta
	
	// reduce and add
	mpy			w4*w6, A		//A = delta * constant
    add			w5,A			//A = A + old

	// store accumulator
	sac.r		A, w4			// store rounded result
	mov			w4, bemf

	// Bemf damping
	mov			zeta,w5
	mpy			w4*w5, A
	mov			k, w2
	sftac		A,w2			// AccA = zeta * bemf * 2^k

	// Store damping current
	sac.r		A, w0
	mov			w0, zeta_current			//Current that causes damping
	#endasm

	m->zeta_current = zeta_current;
	m->bemf = bemf;
}


void dspAdjCurrents()
{
   	
	static q15_t current_cmd;
	static q15_t zeta_current; 
   	
	// add damping to current commanded
	current_cmd  = m0Q.qInRef;
	zeta_current = m0.zeta_current;
	#asm
	mov		current_cmd, w0
	mov		zeta_current, w1
	lac		w0, A
	neg		w1, w1
	add		w1, A
	sac.r	A, w0
	mov		w0, current_cmd
	#endasm
	m0Q.cmd_adj = current_cmd;	//above adjustment for damping
	m0D.cmd_adj = m0D.qInRef;	//no adjustment
   	
	// Apply hard current limit
	if (m0Q.cmd_adj >  HARD_CURRENT_LIMIT) m0Q.cmd_adj =  HARD_CURRENT_LIMIT;
	if (m0Q.cmd_adj < -HARD_CURRENT_LIMIT) m0Q.cmd_adj = -HARD_CURRENT_LIMIT;
   	
	current_cmd  = m1Q.qInRef;
	zeta_current = m1.zeta_current;
	#asm
	mov		current_cmd, w0
	mov		zeta_current, w1
	lac		w0, A
	neg		w1, w1
	add		w1, A
	sac.r	A, w0
	mov		w0, current_cmd
	#endasm
	m1Q.cmd_adj = current_cmd;	//above adjustment for damping
	m1D.cmd_adj = m0D.qInRef;	//no adjustment
   	
	if (m1Q.cmd_adj >  HARD_CURRENT_LIMIT) m1Q.cmd_adj =  HARD_CURRENT_LIMIT;
	if (m1Q.cmd_adj < -HARD_CURRENT_LIMIT) m1Q.cmd_adj = -HARD_CURRENT_LIMIT;
}


void dspPID(PID_t *p)
{

	// Annoying but necessary until syntax for addressing
	// structs in asm with this compiler is figured out
	static q15_t	qFF; qFF			= p->qFF;
	static q15_t	qKp; qKp			= p->qKp;
	static q15_t	qKi; qKi			= p->qKi;
	static q15_t	qKc; qKc			= p->qKc;
	static q15_t	qOutMax; qOutMax	= p->qOutMax;
	static q15_t	qOutMin; qOutMin	= p->qOutMin;
	static q15_t	qInRef; qInRef		= p->qInRef;
	static q15_t	qInMeas; qInMeas	= p->qInMeas;
	static int16_t	NKo; NKo			= p->NKo;
	static q15_t	cmd_adj; cmd_adj	= p->cmd_adj;
   	
	// output
	static q15_t	qOut;

	#asm
	// Pointer of struct to w0
	mov			p, w0

	// Err  = InRef - InMeas
	mov			cmd_adj,w7
	mov			qInMeas,w5
	sub			w7,w5,w4

	// U  = Sum + Kp * Err * 2^NKo
	lac			[++w0],B				// AccB = Sum          
	mov			[--w0],w5
	mov			w5, 0x0028				// AccBLow

	// Feed foward term
	mov			qFF, w5		   	
	mpy			w5*w7, A

	mov			qKp,w5
	mac			w4*w5, A				// Switch to mpy if FF term is removed
	mov			NKo, w2
	sftac		A,w2					// AccA = Kp*Err*2^NKo    
		
	add			A						// Sum = Sum + Kp*Err*2^NKo
	sac			A,w6					// store U before tests

	// if( U > Outmax )
	//     Out = Outmax
	// else if( U < Outmin )
	//     Out = Outmin
	// else        
	//     Out = U 

	mov			qOutMax,w1
	cp			w6,w1
	bra GT,		_jPI5					// U > Outmax; w1 = Outmax

	mov			qOutMin,w1
	cp			w6,w1
	bra LE,		_jPI5					// U < Outmin; w1 = Outmin

	mov			w6,w1					// w1 = U
	_jPI5:
	mov			w1,qOut

	// Ki * Err
	mov			qKi,w5
	mpy			w4*w5,A

	// Exc = U - Out
	sub			w6,w1,w6

	// Ki * Err - Kc * Exc 
	mov			qKc,w5
	msc			w5*w6,A

	// Sum = Sum + Ki * Err - Kc * Exc 
	add			A

	sac			A,[++w0]				// store Sum 
	mov			0x0022,w5				// AccALow
	mov			w5,[--w0]

	#endasm
	p->qOut = qOut;
}



void dspFilterBiquadVd(motor_state_t *m, PID_t *mD)
{
	// Biquad Direct Form I
	// Source: http://www.earlevel.com/main/2013/10/13/biquad-calculator-v2/
	// y[n] = a0*x[n] + a1*x[n-1] + a2*x[n-2] – b1*y[n-1] – b2*y[n-2]
	// These parameters have the excellent property of all being less than ±1
	
	// Low Pass, Sample rate = 20kHz; Fc = 861; Q = 0.1 )
	q15_t a0 = 255; //0.007782502182560118
	q15_t a1 = 510; //0.015565004365120236
	q15_t a2 = 255; //0.007782502182560118
	q15_t b1 = -27034; //-0.8250256907387904
	q15_t b2 = -4713; //-0.1438443005309692

	// High Shelf, Sample rate = 10khz; Fc = 3731; Gain = -30db
	// q15_t a0 = 5833; //0.17801276023953236
	// q15_t a1 = 5412; //0.1651632367245255
	// q15_t a2 = 1914; //0.058402445546922664
	// q15_t b1 = -30336; //-0.9258033287822514
	// q15_t b2 = 10727; //0.3273817712932319

	// Notch, Sample = 10khz, notch = 4150hz
	// q15_t a0 = 9243; //0.28207097012569005
	// q15_t a1 = 15911; //0.48558067716991055
	// q15_t a2 = 9243; //0.28207097012569005
	// q15_t b1 = 15911; //0.48558067716991055
	// q15_t b2 = -14282; //-0.43585805974861985

	#warning biquad filters have static local vars to be removed

	static q15_t xn;
	static q15_t xn1;
	static q15_t xn2;
	static q15_t yn;
	static q15_t yn1;
	static q15_t yn2;

	xn = mD->qOut;	//qVd

	#asm

	mov		a0, w4
	mov		xn, w5
	mpy		w4*w5, A

	mov		a1, w4
	mov		xn1, w5
	mac		w4*w5, A

	mov		a2, w4
	mov		xn2, w5
	mac		w4*w5, A

	mov		b1, w4
	mov		yn1, w5
	msc		w4*w5, A

	mov		b2, w4
	mov		yn2, w5
	msc		w4*w5, A

	sac.r	A, w0
	mov		w0, yn		//filter output

	#endasm

	yn2 = yn1;
	yn1 = yn;
	xn2 = xn1;
	xn1 = xn;
   	
	m->qVd = yn;
}



void dspFilterBiquadVq(motor_state_t *m, PID_t *mQ)
{
	// Biquad Direct Form I
	// Source: http://www.earlevel.com/main/2013/10/13/biquad-calculator-v2/
	// y[n] = a0*x[n] + a1*x[n-1] + a2*x[n-2] - b1*y[n-1] - b2*y[n-2]
	// Coefficients (Sample Rate = 20kHz; Fc = 861; Q = 0.1 )
	// These parameters have the excellent property of all being less than ±1
	q15_t a0 = 255; //0.007782502182560118
	q15_t a1 = 510; //0.015565004365120236
	q15_t a2 = 255; //0.007782502182560118
	q15_t b1 = -27034; //-0.8250256907387904
	q15_t b2 = -4713; //-0.1438443005309692
   	
	// High Shelf, Sample rate = 10khz; Fc = 3731; Gain = -30db
	// q15_t a0 = 5833; //0.17801276023953236
	// q15_t a1 = 5412; //0.1651632367245255
	// q15_t a2 = 1914; //0.058402445546922664
	// q15_t b1 = -30336; //-0.9258033287822514
	// q15_t b2 = 10727; //0.3273817712932319

	// Notch, Sample = 10khz, notch = 4150hz
	//q15_t a0 = 9243; //0.28207097012569005
	//q15_t a1 = 15911; //0.48558067716991055
	//q15_t a2 = 9243; //0.28207097012569005
	//q15_t b1 = 15911; //0.48558067716991055
	//q15_t b2 = -14282; //-0.43585805974861985

	static q15_t xn;
	static q15_t xn1;
	static q15_t xn2;
	static q15_t yn;
	static q15_t yn1;
	static q15_t yn2;

	// newest sample for filtering
	xn = mQ->qOut;	// qVq

	#asm

	mov		a0, w4
	mov		xn, w5
	mpy		w4*w5, A

	mov		a1, w4
	mov		xn1, w5
	mac		w4*w5, A

	mov		a2, w4
	mov		xn2, w5
	mac		w4*w5, A

	mov		b1, w4
	mov		yn1, w5
	msc		w4*w5, A

	mov		b2, w4
	mov		yn2, w5
	msc		w4*w5, A

	sac.r	A, w0
	mov		w0, yn		//filter output

	#endasm

	yn2 = yn1;
	yn1 = yn;
	xn2 = xn1;
	xn1 = xn;
   	
	m->qVq = yn;
}


void dspIIRCascadeQ(motor_state_t *m, PID_t *mQ)
{
	 static q15_t sample; sample		= mQ->qOut;
	 static q15_t filtered; filtered	= m->qVq;

	#asm
	mov			sample, w4		//w4 = sample
    mov			filtered, w5	//w5 = old
	mov			#2048, w6		//w6 = filter level
    
	// delta
	sub			w4,w5,w4	//w4 = delta
	
	// reduce and add
	mpy			w4*w6, A	//A = delta * constant
    add			w5,A		//A = A + old

	// store accumulator
	sac.r		A, w0		// store rounded result
    mov			w0, filtered
	#endasm

	m->qVq = filtered;

}


void dspIIRCascadeD(motor_state_t *m, PID_t *mD)
{
	static  q15_t sample;  sample		= mD->qOut;
	static q15_t filtered; filtered		= m->qVd;

	#asm
	mov			sample, w4		//w4 = sample
    mov			filtered, w5	//w5 = old
	mov			#2048, w6		//w6 = filter level
    
	// delta
	sub			w4,w5,w4	//w4 = delta
   	
	// reduce and add
	mpy			w4*w6, A	//A = delta * constant
    add			w5,A		//A = A + old

	// store accumulator
	sac.r		A, w0		// store rounded result
    mov			w0, filtered
	#endasm

	m->qVd = filtered;

}


void dspInvParkClarke(motor_state_t *m)
{  
	// inputs
	static q15_t qVq; qVq = m->qVq;
	static q15_t qVd; qVd = m->qVd;
	static q15_t sin_theta; sin_theta = m->sin_theta;
	static q15_t cos_theta; cos_theta = m->cos_theta;

	// outputs
	static q15_t qValpha, qVbeta;
	static q15_t qVr1, qVr2, qVr3;

	#asm

	// Inverse Park
	mov		qVd,w6
	mov		qVq,w7

	mov		sin_theta,w4
	mov		cos_theta,w5

	//Valpha =  Vd*cos(Angle) - Vq*sin(Angle)
	mpy		w5*w6,A			// A = Vd * Cos
	msc		w4*w7,A			// A = A - Vq * Sin
	sac		A,w0
	mov		w0, qValpha

	//Vbeta  =  Vd*sin(Angle) + Vq*cos(Angle)
	mpy		w4*w6,A			// A = Vd * Sin
	mac		w5*w7,A			//A = A + Vq * Cos
	sac		A,w1
	mov		w1,qVbeta

	// Inverse Clarke
	// Vr1 = valpha
	mov		w0, qVr1

	// Vr2 = -(1/2)valpha + (sqrt3 / 2)vbeta
	neg		w0,w0			// Valpha = -Valpha
	lac		w0, #1, A		// A = -Valpha/2
	mov		#28377, w4		// (sqrt3 / 2)
	mov		w1, w5			// Vbeta in dsp friendly register
	mac		w4*w5, A		// A = A + (sqrt3 /2) * Vbeta
	sac		A, w2
	mov		w2, qVr2
	
	// Vr3 = -(1/2)valpha - (sqrt3 / 2)vbeta
	lac		w0, #1, A		// A = -Valpha/2
	msc		w4*w5, A		// A = A - (sqrt3 / 2) * Vbeta
	sac		A, w2
	mov		w2, qVr3

	#endasm
   	
	m->qValpha = qValpha;
	m->qVbeta  = qVbeta;
   	
	m->qVr1 = qVr1;
	m->qVr2 = qVr2;
	m->qVr3 = qVr3;
}



void dspSinModulation(motor_state_t *m)
{
	// local variables
	static q15_t qVoffset, qVmax, qVmin;

	// inputs
	static q15_t qVr1; qVr1 = m->qVr1;
	static q15_t qVr2; qVr2 = m->qVr2;
	static q15_t qVr3; qVr3 = m->qVr3;

	// outputs
	static uint16_t dPWM1, dPWM2, dPWM3;

	// Determine largest and smallest phase voltages
	if (qVr1 > qVr2 && qVr1 > qVr3) qVmax = qVr1;
		else if (qVr2 > qVr3) qVmax = qVr2;
			else qVmax = qVr3;
	
	if (qVr1 < qVr2 && qVr1 < qVr3) qVmin = qVr1;
		else if (qVr2 < qVr3) qVmin = qVr2;
			else qVmin = qVr3;

	// Calc Voffset for 3rd harmonic injection
	// qVoffset = q15_mul(16384, q15_add(qVmin, qVmax));
	#asm
	mov		qVmin, w0
	lac		w0, #1, A		// A = Vmin/2
	mov		qVmax, w0
	add		w0, #1, A		// A = A + Vmax/2
	sac		A, w0			
	mov		w0, qVoffset	// For debug, ok to remove this variable

	// Offset each phase voltage
	// qVr1 = qVr1 - qVoffset;
	mov		qVr1, w4
	sub		w4, w0, w4		// w4 = offseted Vr1

	// qVr2 = qVr2 - qVoffset;
	mov		qVr2, w5
	sub		w5, w0, w5		// w5 = offseted Vr2
	
	// qVr3 = qVr3 - qVoffset;
	mov		qVr3, w6
	sub		w6, w0, w6		// w6 = offsetted Vr3
	
	// Scale to PWM duty range
	// dPWM1 = q15_mul(qVr1, (PWM_PERIOD_MID)) + PWM_PERIOD_MID;
	mov		#710, w7		// w7 = PWM_PERIOD_MID
	mpy		w4*w7,A			// A = Vr1 * PWM_PERIOD_MID
	add		w7, A			// A = A + PWM_PERIOD_MID
	sac.r	A, w4			// store rounded result			

	// dPWM2 = q15_mul(qVr2, (PWM_PERIOD_MID)) + PWM_PERIOD_MID;
	mpy		w5*w7,A			// A = Vr1 * PWM_PERIOD_MID
	add		w7, A			// A = A + PWM_PERIOD_MID
	sac.r	A, w5			// store rounded result		

	// dPWM3 = q15_mul(qVr3, (PWM_PERIOD_MID)) + PWM_PERIOD_MID;
	mpy		w6*w7,A			// A = Vr1 * PWM_PERIOD_MID
	add		w7, A			// A = A + PWM_PERIOD_MID
	sac.r	A, w6			// store rounded result			

	// Ensure registers will be loaded within range
	//if (dPWM1 > PWM_PERIOD) dPWM1 = PWM_PERIOD;
	mov		#1370, w0		// w0 = PWM_PERIOD
	cp		w0, w4			// compare: w0 - w4
	bra C,	_skip1
	mov		#1370, w4		// clip at PWM_PERIOD
	_skip1:
	mov	w4, dPWM1
     
	//if (dPWM2 > PWM_PERIOD) dPWM2 = PWM_PERIOD;
	cp		w0, w5			// compare: w0 - w5
	bra C,	_skip2
	mov		#1370, w5		// clip at PWM_PERIOD
	_skip2:
	mov	w5, dPWM2	 

	//if (dPWM3 > PWM_PERIOD) dPWM3 = PWM_PERIOD;
	cp		w0, w6			// compare: w0 - w6
	bra C,	_skip3
	mov		#1370, w6		// clip at PWM_PERIOD
	_skip3:
	mov	w6, dPWM3
   	
	#endasm

	m->dPWM1 = dPWM1;
	m->dPWM2 = dPWM2;
	m->dPWM3 = dPWM3;

}


void dspApplyPWM()
{
	// swap phases A and B depending on calibrated parity
	if (m0.phase_parity == 0)
	{
		PDC7 = m0.dPWM1;
		PDC6 = m0.dPWM2;
		PDC5 = m0.dPWM3;
	} 
	else
	{
		PDC7 = m0.dPWM2;
		PDC6 = m0.dPWM1;
		PDC5 = m0.dPWM3;
	}

	if (m1.phase_parity == 0)
	{
		PDC1 = m1.dPWM1;
		PDC2 = m1.dPWM2;
		PDC3 = m1.dPWM3;
	} 
	else
	{
		PDC1 = m1.dPWM2;
		PDC2 = m1.dPWM1;
		PDC3 = m1.dPWM3;
	}	
}

#inline
void dspOmegaEstimation(void)
{
	static q16angle_t m0o_now, m1o_now;
	static q15_t m0o_flt, m1o_flt;

	// calc omega now
	m0o_now = m0.theta - m0.theta_prev;
	m1o_now = m1.theta - m1.theta_prev;
   	
	// store previous omega
	m0.theta_prev = m0.theta;
	m1.theta_prev = m1.theta;


	// apply filter
	// Saving AccA not necessary if when called from 
	// dspFOC interrupt function
	#asm
	//push		0x0022			// save AccAL
	//push		0x0024			// save AccAH
	//push		0x0026			// save AccAU
	//push		CORCON			// save Acc state
	
	mov			#8191, w6		//w6 = filter level
	
	mov			m0o_now, w4		//w4 = sample
    mov			m0o_flt, w5		//w5 = old
	sub			w4,w5,w4		//w4 = delta
	mpy			w4*w6, A		//A = delta * constant
    add			w5,A			//A = A + old
	sac.r		A, w0			// store rounded result
    mov			w0, m0o_flt

	mov			m1o_now, w4		//w4 = sample
    mov			m1o_flt, w5		//w5 = old
	sub			w4,w5,w4		//w4 = delta
	mpy			w4*w6, A		//A = delta * constant
    add			w5,A			//A = A + old
	sac.r		A, w0			// store rounded result
    mov			w0, m1o_flt
	
	//pop			CORCON
	//pop			0x0026
	//pop			0x0024		
	//pop			0x0022		

    #endasm

	// output from filter
	m0.omega = m0o_flt;
	m1.omega = m1o_flt;
}


// #inline
// void dspResolveStepCount(void)
// {
//	/*** alt step counter using theta **/
//	static q16angle_t m0t_prev;
//	static q16angle_t m1t_prev;

//	// >> 13 means 8 steps per e-rev, 160 per m-rev
//	m0.step_count += (m0.theta >> 13) - (m0t_prev >> 13);
//	m1.step_count += (m1.theta >> 13) - (m1t_prev >> 13);

//	// store for next call
//	m0t_prev = m0.theta;
//	m1t_prev = m1.theta;
// }


void dspResolveStepCount(motor_state_t *m)
{
	// Check for sign changes
	if ((m->angle_a > 0) > (m->angle_a_prev > 0)) 
	{
		// Positive sign change for sensor a
		if ((m->angle_b > 0) && (m->angle_c < 0)) { (m->step_count)++; }
		else if ((m->angle_b < 0) && (m->angle_c > 0)) { (m->step_count)--; }
	} 
	else if ((m->angle_a > 0) < (m->angle_a_prev > 0)) 
		{
			if ((m->angle_b > 0) && (m->angle_c < 0)) { (m->step_count)--; }
			else if ((m->angle_b < 0) && (m->angle_c > 0)) { (m->step_count)++; }
		} 
		else if ((m->angle_b > 0) > (m->angle_b_prev > 0)) 
			{
				// Positive sign change for sensor b
				if ((m->angle_c > 0) && (m->angle_a < 0)) { (m->step_count)++; }
				else if ((m->angle_c < 0) && (m->angle_a > 0)) { (m->step_count)--; }
			} 
			else if ((m->angle_b > 0) < (m->angle_b_prev > 0)) 
				{
					if ((m->angle_c > 0) && (m->angle_a < 0)) { (m->step_count)--; }
					else if ((m->angle_c < 0) && (m->angle_a > 0)) { (m->step_count)++; }
				} 
				else if ((m->angle_c > 0) > (m->angle_c_prev > 0)) 
					{
						// Positive sign change for sensor c
						if ((m->angle_a > 0) && (m->angle_b < 0)) { (m->step_count)++; }
						else if ((m->angle_a < 0) && (m->angle_b > 0)) { (m->step_count)--; }
					} 
					else if ((m->angle_c > 0) < (m->angle_c_prev > 0)) 
					{
						if ((m->angle_a > 0) && (m->angle_b < 0)) { (m->step_count)--; }
						else if ((m->angle_a < 0) && (m->angle_b > 0)) { (m->step_count)++; }
					}

	m->angle_a_prev = m->angle_a; 
	m->angle_b_prev = m->angle_b; 
	m->angle_c_prev = m->angle_c;
}

