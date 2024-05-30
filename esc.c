#include <33FJ64GS608.h>
#include <stdint.h>
#include "esc.h"
#include "libmathq15.c"
#include "parameters.h"


// CCS Compiler prefers it this way
// for single compilation unit
// order matters...
#include "utils.c"
#include "uart_plot.c"
#include "uart_ui.c"
#include "sysinit.c"
#include "dspfoc.c"
#include "foc.c"
#include "calibration.c"
#include "spi.c"
#include "tasks.c"
#include "int_tasks.c"


void main(void) 
{

	reset_cause = restart_cause();		// Always find reset cause before anything else
	disable_interrupts(INTR_GLOBAL);	// Running running until after sysinit

	system_state = INIT;
	init_result = initialisation();		// system could go to fault state on bad init (TODO)

	// init result always ok, so onto default state
	system_state = SPI2_OFFLINE;		// assume no SPI2 until it comes online
	spi.offline_counter = 99;			// so it doesn't go change straight to RUN
	
	// Only print this once for the sake of efficiency
	if (uart_mode == TERMINAL) print_term_static();
   	
	enable_interrupts(INTR_GLOBAL);

	while(TRUE)
	{
		switch (system_state)
		{
			case(RUN): 
			{
				/**
				 * In this mode the key tasks are interrupt driven,
				 * so there is not much to do here that isn't already being
				 * done on an interrupt of as a background task
				 * in this while(true) loop
				 * */
			
			} break;

			case(SPI2_OFFLINE):
			{
				// stop the motors and zero any commanded inputs


			} break;
			
			case(CALIBRATION):
			{
				CalibrateMotor();
			   	
				// Overwrites terminal, so necessary
				if(uart_mode == TERMINAL) 
				{
					clear_terminal();
					print_term_static();
				}
			} break;

			case(FATAL_ERROR): 
			{
				// Todo: A background task to observe essential sensors
				// (current and angle) and raise error if sensors are 
				// offline.

				// disable motors
				setup_hspwm(HSPWM_DISABLED, PWM_PERIOD);

			   	
				// indicate with LEDs
				while(system_state == FATAL_ERROR)
				{
					// asses if ok to resume RUN mode
					//systemStateEngine();

					// indicate
					RED_LED_ON;
					ORANGE_LED_ON;
					BLUE_LED_ON;
					delay_ms(500);
					RED_LED_OFF;
					ORANGE_LED_OFF;
					BLUE_LED_OFF;
					delay_ms(500);
				}
			} break;
			default: {} break;
		}
	   	
		/*** Background tasks always running***/
	   	
		// Process any UI input
		if (uart_input) process_uart();
		perf_counter++; //track mainloop rate
	   	
		// Read temperatures
		read_temperatures();

		// Check for and switch states as needed
		systemStateEngine();

		// UI Printing over UART
		uartUIprint();
	}
} 	
