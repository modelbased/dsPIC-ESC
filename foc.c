// Slower but clearer (no asm) FOC implementation.
// Ideally kept in sync with dspFOC functions, since these
// functions are used for the calibration procedures. The 
// faster (asm) FOC functions must only be called by the 
// relevant interrupt or the compiler inserts code to
// disabled interrupts when they are called.

void MeasureADCs()
{
	// swap phases A and B depending on calibrated parity
	// correct ADC fractional offset to +32767..-32768
	if (m0.phase_parity == 0)
	{
		m0.qIa = ADCBUF11 - ADC_FRC_MIDPOINT;
		m0.qIb = ADCBUF10 - ADC_FRC_MIDPOINT;
		m0.qIc = ADCBUF9  - ADC_FRC_MIDPOINT;
	} 
	else
	{
		m0.qIa = ADCBUF10 - ADC_FRC_MIDPOINT;
		m0.qIb = ADCBUF11 - ADC_FRC_MIDPOINT;
		m0.qIc = ADCBUF9  - ADC_FRC_MIDPOINT;
	}

	if (m1.phase_parity == 0)
	{
		m1.qIa = ADCBUF8 - ADC_FRC_MIDPOINT;
		m1.qIb = ADCBUF7 - ADC_FRC_MIDPOINT;
		m1.qIc = ADCBUF6 - ADC_FRC_MIDPOINT;
	} 
	else
	{
		m1.qIa = ADCBUF7 - ADC_FRC_MIDPOINT;
		m1.qIb = ADCBUF8 - ADC_FRC_MIDPOINT;
		m1.qIc = ADCBUF6 - ADC_FRC_MIDPOINT;
	}
   	
	m0.angle_a = (ADCBUF2 - ADC_FRC_MIDPOINT);
	m0.angle_b = (ADCBUF1 - ADC_FRC_MIDPOINT);
	m0.angle_c = (ADCBUF0 - ADC_FRC_MIDPOINT);

	m1.angle_a = (ADCBUF5 - ADC_FRC_MIDPOINT);
	m1.angle_b = (ADCBUF4 - ADC_FRC_MIDPOINT);
	m1.angle_c = (ADCBUF3 - ADC_FRC_MIDPOINT);
}


void MeasureTheta(motor_state_t *m)
{
	q15_t cos_est = 0;
	q15_t sin_est = 0;

	// Apply pseudoinverse of matrix which maps
	// (cos theta, sin theta) to (cos theta, cos theta + 2pi/3, cos theta + 4pi/3)
	cos_est = q15_add(q15_mul(XINV_11, m->angle_a), q15_mul(XINV_12, m->angle_b));
	cos_est = q15_add(cos_est, q15_mul(XINV_13, m->angle_c));
	sin_est = q15_add(q15_mul(XINV_21, m->angle_a), q15_mul(XINV_22, m->angle_b));
	sin_est = q15_add(sin_est, q15_mul(XINV_23, m->angle_c));

	// Calculate electrical phase angle, use extra temp variable to avoid math
	// error interrupt being triggered. Note that sin and cos are in the wrong
	// order, this is likely due to the matrix equation above having slightly
	// the wrong formulation, but the angles work as expected so this is left
	// as is for now.
	m->theta = q15_atan2(sin_est, cos_est) + m->theta_offset;

	// Calculate sin and cos of the angle for Park transforms
	m->sin_theta = q15_sin(m->theta);
	m->cos_theta = q15_cos(m->theta);
}


void Clarke(motor_state_t *m)
{
	q15_t x, y, z;

	// Clark transform (amplitude invariant)
	// Ialpha = (2/3)Ia - (1/3)Ib - (1/3)Ic
	// Ibeta  = (sqrt3/3)Ib - (sqrt3/3)Ic
	x = q15_mul(m->qIa, 21845);
	y = q15_mul(m->qIb, 10923);
	z = q15_mul(m->qIc, 10923);
	m->qIalpha = q15_add(x, q15_add(q15_neg(y), q15_neg(z)));

	x = q15_mul(m->qIb, 18918);
	y = q15_mul(m->qIc, 18918);
	m->qIbeta = q15_add(x, q15_neg(y));
}

void Park(motor_state_t *m)
{
	q15_t x, y;

	// Park transform
	// Id =  Ialpha*cos(Angle) + Ibeta*sin(Angle)
	// Iq = -Ialpha*sin(Angle) + Ibeta*cos(Angle)
	x = q15_mul(m->qIalpha, m->cos_theta);
	y = q15_mul(m->qIbeta, m->sin_theta);
	m->qId = q15_add(x, y);

	x = q15_mul(m->qIalpha, m->sin_theta);
	y = q15_mul(m->qIbeta, m->cos_theta);
	m->qIq = q15_add(q15_neg(x), y);
}


void PID(PID_t *p)
{
	q15_t qInRef	= p->qInRef;
	q15_t qInMeas	= p->qInMeas;
	q15_t qKp		= p->qKp;
	q15_t x,y;
	
	x = q15_neg(qInMeas);
	x = q15_add(qInRef, x);
	y = q15_mul(qKp, x);

	p->qOut = y;
}


void InvPark(motor_state_t *m)
{
	q15_t x, y;
	// Inverse Park
	// Valpha =  Vd*cos(Angle) - Vq*sin(Angle)
	// Vbeta  =  Vd*sin(Angle) + Vq*cos(Angle)
	x = q15_mul(m->qVd, m->cos_theta);
	y = q15_mul(m->qVq, m->sin_theta);
	m->qValpha = q15_add(x, q15_neg(y));

	x = q15_mul(m->qVd, m->sin_theta);
	y = q15_mul(m->qVq, m->cos_theta);
	m->qVbeta = q15_add(x, y);
}

void InvClarke(motor_state_t *m)
{
	q15_t x, y;
  	
	// Inverse Clark
	// va = valpha
	// vb = -(1/2)valpha + (sqrt3 / 2)vbeta
	// vc = -(1/2)valpha - (sqrt3 / 2)vbeta
	m->qVr1 = m->qValpha;

	x = q15_mul(m->qValpha, 16384);
	y = q15_mul(m->qVbeta, 28378);
	m->qVr2 = q15_add(q15_neg(x), y);

	m->qVr3 = q15_add(q15_neg(x), q15_neg(y));
}


void SinModulation(motor_state_t *m)
{
	q15_t qVoffset, qVmax, qVmin;

	if (m->qVr1 > m->qVr2 && m->qVr1 > m->qVr3) qVmax = m->qVr1;
		else if (m->qVr2 > m->qVr3) qVmax = m->qVr2;
			else qVmax = m->qVr3;
	
	if (m->qVr1 < m->qVr2 && m->qVr1 < m->qVr3) qVmin = m->qVr1;
		else if (m->qVr2 < m->qVr3) qVmin = m->qVr2;
			else qVmin = m->qVr3;

	qVoffset = q15_mul(16384, q15_add(qVmin, qVmax));

	m->qVr1 = m->qVr1 - qVoffset;
	m->qVr2 = m->qVr2 - qVoffset;
	m->qVr3 = m->qVr3 - qVoffset;
	
	
	// Scale to PWM duty range
   m->dPWM1 = q15_mul(m->qVr1, (PWM_PERIOD / 2)) + (PWM_PERIOD / 2);
   m->dPWM2 = q15_mul(m->qVr2, (PWM_PERIOD / 2)) + (PWM_PERIOD / 2);
   m->dPWM3 = q15_mul(m->qVr3, (PWM_PERIOD / 2)) + (PWM_PERIOD / 2);
     
   // Ensure registers will be loaded within range
   if (m->dPWM1 > PWM_PERIOD) m->dPWM1 = PWM_PERIOD;
     
   if (m->dPWM2 > PWM_PERIOD) m->dPWM2 = PWM_PERIOD;
     
   if (m->dPWM3 > PWM_PERIOD) m->dPWM3 = PWM_PERIOD;
}


void ApplyPWM()
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
