// Check and update motor (electrical) step count
// Needs to be called frequently enough to not miss a step
void ResolveStepCountCalibration(motor_state_t *m) 
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


// Just sensor reading with no adjustments
q16angle_t MeasureThetaRaw(motor_state_t *m)
{
	q15_t x, y, z;
	
	q15_t cos_est = 0;
	q15_t sin_est = 0;

	// Clark transform (amplitude invariant)
	// Ialpha = (2/3)Ia - (1/3)Ib - (1/3)Ic
	// Ibeta  = (sqrt3/3)Ib - (sqrt3/3)Ic
	x = q15_mul(m->angle_a, 21845);
	y = q15_mul(m->angle_b, 10923);
	z = q15_mul(m->angle_c, 10923);
	sin_est = q15_add(x, q15_add(q15_neg(y), q15_neg(z)));

	x = q15_mul(m->angle_b, 18918);
	y = q15_mul(m->angle_c, 18918);
	cos_est = q15_add(x, q15_neg(y));
    
    // -16384 due to using clarke transform which has slightly different
    // output to pseudo inverse matrix method
    q16angle_t theta_raw = q15_atan2(sin_est, cos_est) - 16384;
    
    return (theta_raw);
}


void CalibrateTheta(motor_state_t *m)
{
	int16_t i;
   	
	m->qVq			= 0;
	m->theta		= 0;
	m->sin_theta	= 0;
	m->cos_theta	= 32767; // +ve 1

	// Increment pwm gradually for smooth movement
	// Apply voltage in the direct voltage frame to lock rotor
	for (i = 0; i < CALIBRATION_VOLTS; i++)
	{
		m->qVd = i;		// because compiler needs it this way
		InvPark(m);
		InvClarke(m);
		SinModulation(m);
		ApplyPWM();
		delay_us(20);	// ramp up gradually
	}
  	
	// Delay some time to ensure the angle settles
	delay_ms(500);
   	
	q16angle_t theta_raw = MeasureThetaRaw(m);
	m->theta_offset = q15_neg(theta_raw);
   	
	// Motor off
	m->qVd = 0;
	InvPark(m);
	InvClarke(m);
	SinModulation(m);
	ApplyPWM();
   	
	if (uart_mode == TERMINAL) fprintf(U1, "Theta Offset [%u] \n\r",m->theta_offset);
}


uint16_t CheckMotorSpin(motor_state_t *m)
{ 
	uint16_t loop_counter = 0;
	uint16_t spin_success_flag = 0;
	
	// Apply a quadrature voltage to spin motor open loop
	m->qVd = 0;				  	
	m->qVq = CALIBRATION_VOLTS;

	m->step_count = 0; //reset

	while((m->step_count < CALIBRATION_STEPS) && (loop_counter < 6000))
	{
		MeasureTheta(m);
		ResolveStepCountCalibration(m);
		InvPark(m);
		InvClarke(m);
		SinModulation(m);
		ApplyPWM();
		loop_counter++;
		delay_us(10);
	}
   	
	// Can move less than calibration steps if motor is loaded
	if (m->step_count > (CALIBRATION_STEPS / 3)) spin_success_flag = 1;
   	
	if (uart_mode == TERMINAL) 
	{
		fprintf(U1, "Steps [%d] (minimum 10) \n\r",m->step_count);
		fprintf(U1, "Phase parity [%u] \n\r",m->phase_parity);
	}
	delay_ms(500);
	
	// return to original position
	if (spin_success_flag)
	{
		loop_counter = 0;
		m->qVq = -CALIBRATION_VOLTS;
		while((m->step_count > 0) && (loop_counter < 6000))
		{
			MeasureTheta(m);
			ResolveStepCountCalibration(m);
			InvPark(m);
			InvClarke(m);
			SinModulation(m);
			ApplyPWM();
			loop_counter++;
			delay_us(10);
		}
	}
   	
	// Motor off
	m->qVq = 0;
	InvPark(m);
	InvClarke(m);
	SinModulation(m);
	ApplyPWM();

	if (spin_success_flag) return(TRUE); else return(FALSE);
}


void LineariseTheta(motor_state_t *m)
{
	int16_t		idx = 0;
	int32_t		idx32 = 0; 

	int32_t		Q16_360DEG = 65536;	// one electrical rev = 1 pole pair  	
	int16_t		e_revs = 10;		// electrical revs to spin. Note motors have 20 pole pairs.
	int16_t		theta_steps = 64;	// theta increments 2^x!. sync manually with delay_steps
	int16_t		delay_steps = 200;	// delay in us per step. yes, found empirically for this motor.

	int16_t		sweeps		= 4;	// sweep fwd and bwd this many times
	int16_t		bin_size	= ((Q16_360DEG / theta_steps) / THETA_LUT_SIZE) * e_revs; // samples per lut entry
   	
	int32_t		errors[THETA_LUT_SIZE] = {0};	// always zero function/local variables!
   	
	if (uart_mode == TERMINAL) fprintf(U1, "Linearisation... \n\r");

	// We know the zero offset correctly from prior theta calibration
	// the zero offset is not applied to the LUT to maintain ability
	// to switch LUT off

	// Spin motor through known angles and store delta with sensor reading
	// starting from theta = zero
	
	// Quadrature voltage to use
	m->qVd		= CALIBRATION_VOLTS;	// rotor zero aligns to d-axis	
	m->qVq		= 0;
	
	// move motor fwd so it starts at/near (true) zero
	for (idx32 = 0; idx32 < (Q16_360DEG * 2); idx32+= theta_steps)
	{
		m->theta = idx32 % Q16_360DEG;
		m->sin_theta = q15_sin(m->theta);
		m->cos_theta = q15_cos(m->theta);
		InvPark(m);
		InvClarke(m);
		SinModulation(m);
		ApplyPWM();
		delay_us(delay_steps);
	}
	
	for (idx = 0; idx < sweeps; idx++) // multiple sweeps
	{
		// sweep forward
		for(idx32 = 0; idx32 < (Q16_360DEG * e_revs); idx32+=theta_steps)
		{
			// set theta
			m->theta = idx32 % Q16_360DEG;

			// move motor
			m->sin_theta = q15_sin(m->theta);
			m->cos_theta = q15_cos(m->theta);
			InvPark(m);
			InvClarke(m);
			SinModulation(m);
			ApplyPWM();

			// calc error
			int16_t error = (MeasureThetaRaw(m) + m->theta_offset) - m->theta;

			// add to errors buffer for averaging (FIR, IRR filter not helpful)
			errors[m->theta >> THETA_LUT_BITS] += error;

			// don't move motor too fast
			delay_us(delay_steps);
		}
		
		// sweep backwards
		for(idx32 = ((Q16_360DEG * e_revs) - 1); idx32 > 0; idx32-=theta_steps)
		{
			// set theta
			m->theta = idx32 % Q16_360DEG;

			// move motor
			m->sin_theta = q15_sin(m->theta);
			m->cos_theta = q15_cos(m->theta);
			InvPark(m);
			InvClarke(m);
			SinModulation(m);
			ApplyPWM();

			// calc error
			int16_t error = (MeasureThetaRaw(m) + m->theta_offset) - m->theta;

			// add to errors buffer for averaging
			errors[m->theta >> THETA_LUT_BITS] += error;

			// don't move motor too fast
			delay_us(delay_steps);
		}
	}

	// average fwd and bwk passes and store in lut
	for(idx = 0; idx < THETA_LUT_SIZE; idx++)
	{
		m->theta_lut[idx] = errors[idx] / (((int32_t)bin_size) * 2 * (int32_t)sweeps); //* 2 for fwd and bwd passes
	}

	// subtract DC offset from look up table corrections
	// DC offset is the average of the table values
	// and can be from e.g. friction
	int32_t dc_offset = 0;
	for (idx = 0; idx < THETA_LUT_SIZE; idx++) dc_offset += m->theta_lut[idx];								// integrate
	if (uart_mode == TERMINAL) fprintf(U1, "Sum of lookup table [%Ld] \n\r", dc_offset);

	dc_offset = dc_offset / THETA_LUT_SIZE;																	// average
	if (uart_mode == TERMINAL) fprintf(U1, "DC offset [%Ld] \n\r", dc_offset);																	// average
	
	for (idx = 0; idx < THETA_LUT_SIZE; idx++) m->theta_lut[idx] = m->theta_lut[idx] - (int16_t)dc_offset;	// subtract
   	
	// Motor off
	m->qVd = 0;
	InvPark(m);
	InvClarke(m);
	SinModulation(m);
	ApplyPWM();
}


// Initilise ADC offsets by taking avg of many readings
void calibrate_adc_offset(motor_state_t *m)
{
	#define SAMPLES 16384
	
	int32_t a = 0;
	int32_t b = 0;
	int32_t c = 0;

	// motor off
	m->qVd = 0;
	m->qVq = 0;
	InvPark(m);
	InvClarke(m);
	SinModulation(m);
	ApplyPWM();

	// read and average ADCs
	for (uint16_t i = 0; i < SAMPLES; i++)
	{
		MeasureADCs();
		a += m->qIa;
		b += m->qIb;
		c += m->qIc;
	}

	a /= SAMPLES;
	b /= SAMPLES;
	c /= SAMPLES;

	m->qIaoff = (q15_t)a;
	m->qIboff = (q15_t)b;
	m->qIcoff = (q15_t)c;

	if (uart_mode == TERMINAL)
	{
		fprintf(U1, "Phase A current sensor offset [%d] (in Q15) \n\r", m->qIaOff);
		fprintf(U1, "Phase B current sensor offset [%d] (in Q15) \n\r", m->qIbOff);
		fprintf(U1, "Phase C current sensor offset [%d] (in Q15) \n\r", m->qIcOff);
	} 
}

void CalibrateMotor()
{
	int16_t calibration_attempts = 0;
	uint16_t spin_result = 0;

	BLUE_LED_ON;
	ORANGE_LED_OFF;
	RED_LED_ON;

	if (uart_mode == TERMINAL)
	{
		clear_terminal();
		term_xy(0,0);
	} 

	// set system for calibration mode
	disable_interrupts(INT_PWM1);		// disable FOC
	disable_interrupts(INT_RDA);		// disable UI
	enable_interrupts(INTR_GLOBAL);		// enable other interrupt tasks

	// stop both motors, must directly address struct
	m0.qVd     = 0;
	m0.qVq     = 0;
	m0.qValpha = 0;
	m0.qVbeta  = 0;
	m0.qVr1    = 0;
	m0.qVr2    = 0;
	m0.qVr3    = 0;
	m0.dPWM1   = 0;
	m0.dPWM2   = 0;
	m0.dPWM3   = 0;
	m1.qVd     = 0;
	m1.qVq     = 0;
	m1.qValpha = 0;
	m1.qVbeta  = 0;
	m1.qVr1    = 0;
	m1.qVr2    = 0;
	m1.qVr3    = 0;
	m1.dPWM1   = 0;
	m1.dPWM2   = 0;
	m1.dPWM3   = 0;
	m0Q.qInRef = 0;
	m0D.qInRef = 0;
	m1Q.qInRef = 0;
	m1D.qInRef = 0;
	ApplyPWM();

	// Start with M0
	if (uart_mode == TERMINAL) fprintf(U1, "CALIBRATING M0... \n\r");

	// ADC sensors can have a bias, find any
	calibrate_adc_offset(&m0);

	calibration_attempts = 0;
	while (calibration_attempts < CALIBRATION_MAX_ATTEMPS)
	{
		// calibrate phase angle
		CalibrateTheta(&m0);
	   	
		delay_ms(500); //stop motor jumping
	   	
		// if motor spins then break, else change parity and start again
		spin_result = CheckMotorSpin(&m0);
		if (spin_result) break;
		else m0.phase_parity = !m0.phase_parity;
		calibration_attempts++;
	}

	if (spin_result)
	{
		//#warning Linearisation disabled M0
		LineariseTheta(&m0);
		StoreCalibration(&m0, 0);
		if (uart_mode == TERMINAL) fprintf(U1, "M0 calibration SUCCESS \n\r");
	}
	else
		if (uart_mode == TERMINAL) fprintf(U1, "M0 calibration FAIL, no spin detected \n\r");
   	

	// Now do M1
	if (uart_mode == TERMINAL) fprintf(U1, "\n\rCALIBRATING M1... \n\r");

	// ADC sensors can have a bias, find any
	calibrate_adc_offset(&m1);

	calibration_attempts = 0;
	spin_result = 0;
	while (calibration_attempts < CALIBRATION_MAX_ATTEMPS)
	{
		// calibrate phase angle
		CalibrateTheta(&m1);
		delay_ms(500); //stop motor jumping

		// if motor spins then break, else change parity and start again
		spin_result = CheckMotorSpin(&m1);
		if (spin_result) break;
		else m1.phase_parity = !m1.phase_parity;
		calibration_attempts++;
	}

	if (spin_result)
	{	//#warning	Linearisation disabled M1
		LineariseTheta(&m1);
		StoreCalibration(&m1, 1);
		if (uart_mode == TERMINAL) fprintf(U1, "M1 calibration SUCCESS \n\r");
	}
	else
		if (uart_mode == TERMINAL) fprintf(U1, "M1 calibration FAIL, no spin detected \n\r");


	// done, resume operations
	delay_ms(3000);
	system_state = SPI2_OFFLINE; //assume master offline when done
	enable_interrupts(INT_RDA);
	enable_interrupts(INT_PWM1);
   	
	BLUE_LED_OFF;
	ORANGE_LED_ON;
	RED_LED_OFF;
}
