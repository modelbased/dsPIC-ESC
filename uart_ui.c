//UI over RS232 UART interface

#define ESC 27

//Foreground Colours
//http://ascii-table.com/ansi-escape-sequences.php
#define F_BLACK 30
#define F_RED 31
#define F_GREEN 32
#define F_YELLOW 33
#define F_BLUE 34
#define F_MAGENTA 35
#define F_CYAN 36
#define F_WHITE 37

void clear_terminal()
{
   fputc(ESC,U1);
   fprintf(U1,"[2J");//clears screen
}

//cool, like a proper UI!
void term_xy(byte x, byte y)
{
   fputc(ESC,U1);
   fprintf(U1,"[%u;%uf",y+1,x+1);
}

// very colour, much terminal
void term_colour(byte colour)
{
   fputc(ESC,U1);
   fprintf(U1,"[%um",colour);
}


void print_term_static(void)
{

	term_colour(F_WHITE);
	term_xy(27,0); fprintf(U1,"Motor0");
	term_xy(42,0); fprintf(U1,"Motor1");

	term_xy(0,0);  fprintf(U1,"Reset: ");
	term_xy(0,2);  fprintf(U1,"Volts");
	term_xy(0,4);  fprintf(U1,"Current CMD (cA)");
	term_xy(0,6);  fprintf(U1,"Current (cA)");
	term_xy(0,8);  fprintf(U1,"Current Limit (cA)");
	term_xy(0,10); fprintf(U1,"Temp Mosfet (•C)");
	term_xy(0,12); fprintf(U1,"Temp Motor (•C)");
	term_xy(0,14); fprintf(U1,"Encoder");
	term_xy(0,16); fprintf(U1,"Battery Voltage");
	term_xy(0,18); fprintf(U1,"Damping Zeta(Q15)");
	term_xy(0,20); fprintf(U1,"Phase Angle Theta");
	term_xy(0,22); fprintf(U1,"Phase Vel Omega");
	term_xy(0,24); fprintf(U1,"Step Count");

	term_xy(0,26); fprintf(U1,"ChkSum Obs & Vrfy");
	term_xy(0,28); fprintf(U1,"SPI Bad Packets");
	term_xy(0,30); fprintf(U1,"System State");

	term_xy(0,32); fprintf(U1,"[z -> toggle terminal/binary]");
	term_xy(0,33); fprintf(U1,"[x -> start motor calibrations]");
	term_xy(0,34); fprintf(U1,"[. -> reset] [c -> redraw terminal]");
	term_xy(0,36); fprintf(U1,"[q-a & w-s sets current, e-d/r-f sets zeta]");

	term_xy(0,37); fprintf(U1,"Compiled: ");fprintf(U1,__date__);fprintf(U1,"T");fprintf(U1,__time__);
	term_xy(0,38); fprintf(U1,"Compiled for Hardware: ");fprintf(U1,"ESC v%u",ESC_VERSION);

	term_colour(F_CYAN);
	term_xy(7,0);
	switch(reset_cause)
	{
		case(0):  fprintf(U1,"POWER UP     "); break;
		case(1):  fprintf(U1,"BROWNOUT     "); break;
		case(4):  fprintf(U1,"WATCHDOG     "); break;
		case(6):  fprintf(U1,"SOFTWARE     "); break;
		case(7):  fprintf(U1,"MCLR         "); break;
		case(14): fprintf(U1,"ILLEGAL OP   "); break;
		case(15): fprintf(U1,"TRAP CONFLICT"); break;
		default:  fprintf(U1,"?? %d      ??",reset_cause); break;
	}
}

void print_term_variables(void)
{
	term_colour(F_GREEN); //motor0
	term_xy(25,2);  fprintf(U1,"%6d",m0.qVq);
	term_xy(25,4);  fprintf(U1,"%6d",dspMpy(m0Q.qInRef, Q15_SCALE_CURRENT));
	term_xy(25,6);  fprintf(U1,"%6d",dspMpy(m0.qIq, Q15_SCALE_CURRENT));
	term_xy(25,8);  fprintf(U1,"%6d",dspMpy(m0.qIq_limit, Q15_SCALE_CURRENT));
	term_xy(25,10); fprintf(U1,"%6d",s0.mosfet_temperature);
	term_xy(25,12); fprintf(U1,"%6d",s0.motor_temperature);
	term_xy(25,14); fprintf(U1,"%6d",s0.encoder);
	term_xy(25,16); fprintf(U1,"%6d",battery_volts);
	term_xy(25,18); fprintf(U1,"%6d",m0.zeta);
	term_xy(25,20); fprintf(U1,"%6d",m0.theta);
	term_xy(25,22); fprintf(U1,"%6d",m0.omega);
	term_xy(25,24); fprintf(U1,"%6d",m0.step_count);
		
	term_colour(F_YELLOW); //motor1
	term_xy(40,2);  fprintf(U1,"%6d",m1.qVq);
	term_xy(40,4);  fprintf(U1,"%6d",dspMpy(m1Q.qInRef, Q15_SCALE_CURRENT));
	term_xy(40,6);  fprintf(U1,"%6d",dspMpy(m1.qIq, Q15_SCALE_CURRENT));
	term_xy(40,8);  fprintf(U1,"%6d",dspMpy(m1.qIq_limit, Q15_SCALE_CURRENT));
	term_xy(40,10); fprintf(U1,"%6d",s1.mosfet_temperature);
	term_xy(40,12); fprintf(U1,"%6d",s1.motor_temperature);
	term_xy(40,14); fprintf(U1,"%6d",s1.encoder);
	term_xy(40,16); fprintf(U1,"%6d",battery_volts);
	term_xy(40,18); fprintf(U1,"%6d",m1.zeta);
	term_xy(40,20); fprintf(U1,"%6d",m1.theta);
	term_xy(40,22); fprintf(U1,"%6d",m1.omega);
	term_xy(40,24); fprintf(U1,"%6d",m1.step_count);

	term_colour(F_WHITE);
	term_xy(25,26); fprintf(U1,"%4X",spi.chksum_rx);
	term_xy(40,26); fprintf(U1,"%4X",spi.chksum_verify);
	term_xy(25,28); fprintf(U1,"%6u",spi.bad_packets);
   	
	term_xy(25,30);
	switch(system_state)
	{   	
		case(INIT): 		fprintf(U1,"INIT       "); break;
		case(RUN):			fprintf(U1,"RUN        "); break;
		case(SPI2_OFFLINE):	fprintf(U1,"SPI2_OFFLN "); break;
		case(CALIBRATION):	fprintf(U1,"CALIBRATION"); break;
		case(FATAL_ERROR):	fprintf(U1,"FATAL_ERROR"); break;
		default: fprintf(U1,"???"); break;
	}
}


void process_uart(void)
{
	//Basic user inputs, always available
	switch(uart_input)
	{
		case('.'): reset_cpu(); break;
		// spin switch is for PID tuning only
		//case('='): if (spin_switch < 2) spin_switch++; else spin_switch = 0; break;
		//case('='): if (spin_switch < 1) spin_switch++; else spin_switch = -1;  break;
		case('x'): system_state = CALIBRATION; break;
		case('z'): uart_mode = (uart_mode == TERMINAL) ? BINARY : TERMINAL; break;
		case('c'):{if (uart_mode == TERMINAL) {clear_terminal(); print_term_static();}}break;
		default: {} break;
	}
   	
   	
	// Terminal interface commands
	// generally only applicable when SPI comms offline
	if (system_state == SPI2_OFFLINE)
	switch(uart_input)
	{
		case('q'): if (m0Q.qInRef < ( 2000)) m0Q.qInRef += 100; break;
		case('a'): if (m0Q.qInRef > (-2000)) m0Q.qInRef -= 100; break;
		case('w'): if (m1Q.qInRef < ( 2000)) m1Q.qInRef += 100; break;
		case('s'): if (m1Q.qInRef > (-2000)) m1Q.qInRef -= 100; break;
	   	
		case('e'): if (m0.zeta < 30000) m0.zeta += 500; break;
		case('d'): if (m0.zeta >= 500)  m0.zeta -= 500; break;
	   	
		case('r'): if (m1.zeta < 30000 ) m1.zeta += 500; break;
		case('f'): if (m1.zeta >= 500)   m1.zeta -= 500; break;
	   	
		case(' '): 
		{
			m0Q.qInRef	= 0; 
			m1Q.qInRef	= 0;
			m0.zeta 	= 0;
			m1.zeta 	= 0;
		} break;
		default: break;
	}
  	
	// must clear input char when done
	uart_input = "";
}
