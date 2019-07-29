/*****************************************************************************
******************************************************************************
*                                                                            *
*     NNNNNN         N                A             TTTTTTT TTTTTTTTTT       *
*     N NNNNN        N               AAA                  T TTTTT            *
*     NN NNNNN       N              AAAAA                 T TTTTT            *
*     N N NNNNN      N             A AAAAA                T TTTTT            *
*     N  N NNNNN     N            A A AAAAA               T TTTTT            *
*     N   N NNNNN    N           A   A AAAAA              T TTTTT            *
*     N    N NNNNN   N          A     A AAAAA             T TTTTT            *
*     N     N NNNNN  N         AAAAAAAAA AAAAA            T TTTTT            *
*     N      N NNNNN N        A         A AAAAA           T TTTTT            *
*     N       N NNNNNN       A           A AAAAA          T TTTTT            *
*     N        N NNNNN  OO  A             A AAAAA  OO     T TTTTT     OO     *
*                       OO                         OO                 OO     *
*                                                                            *
*     Gesellschaft fuer Netzwerk- und Automatisierungstechnologie m.b.H      *
*         Konrad-Zuse-Platz 9, D-53227 Bonn, Tel.:+49 228/965 864-0          *
*                             www.nateurope.com                              *
*                                                                            *
******************************************************************************
******************************************************************************
*
* Module      : main.c
*
* Description : EMCH firmware for "Native Mini"
*
* Author      : Tom Welbers
*
******************************************************************************
******************************************************************************
*
*                    Copyright (c) by N.A.T. GmbH
*
*       All rights reserved. Copying, compilation, modification,
*       distribution or any other use whatsoever of this material
*       is strictly prohibited except in accordance with a Software
*       License Agreement with N.A.T. GmbH.
*
******************************************************************************/
/******************************************************************************/
/*	INCLUDES										*/
/*****************************************************************************/
#include <asf.h>
#include <string.h>
#include "FreeRTOSuser.h"
#include "console.h"
#include "ethernet.h"
#include "task_defs.h"
#include "task.h"
#include "cu.h"
#include "cm.h"
#include "pm.h"
#include "webserver.h"
#include "twi_i2c.h"
#include "cfg.h"
#include "emch_rel.h"
#include "mcmc.h"
#include "ping.h"
#include "telnet.h"
#include "printd.h"
#include "shell.h"
#include "udpecho.h"
#include "tcpecho.h"
#include "mux.h"
#include "lcd.h"
#include "conf_platform.h"
#include "button.h"
/*****************************************************************************/
/*	PRIVATE MACROS										     */
/*****************************************************************************/
#ifdef RELEASE	
	#define HALT_ON_ASSERTION	1
#else			
	#define HALT_ON_ASSERTION	0
#endif

/** Watchdog period 3000ms */
#define WDT_PERIOD                        3000

/** Watchdog restart 2000ms */
#define WDT_RESTART_PERIOD                2000


#define print_welcome_msg() { get_emch_rel(NULL); printd("\n"); } while(0)

/*****************************************************************************/
/*	EXTERNAL REFERENCES					     */
/*****************************************************************************/
extern void io_init(void);

/*****************************************************************************/
/*	PRIVATE FUNCTION PROTOTYPES					     */
/*****************************************************************************/
static void prvSetupHardware(void);
static void adc_init(void);
static void watchdog_init(void);
static void task_watchdog(void *pvParameter);

/* FreeRTOS hook (or callback) functions.	*/
void vApplicationMallocFailedHook(void);
void vApplicationIdleHook(void);
void vApplicationStackOverflowHook(xTaskHandle *pxTask, signed char *pcTaskName);
void vApplicationTickHook(void);

/*****************************************************************************/
/*	PUBLIC GLOBALS					     */
/*****************************************************************************/
uint32_t g_uptime_ms = 0; /* Set by tick handler for uptime measurement */

/*---------------------------------------------------------------------------*
Function:		main
Description:	mch main program
Notes:
Parameters:		none
Return:			none
*-----------------------------------------------------------------------------*/
int main (void)
{
	/** Prepare the hardware to run */
	prvSetupHardware();
	
	/* Watchdog init */
	//watchdog_init();
		
	/* Load config */
	cfg_init();
	/* Create network init components */
	ethernet_init();
	
	/* Create twi init task components */
	twi_init();
	/* Create SHELL task, it creates the UART task and PRINT task too */
	shell_init();

	/* Initialize TELNET task */
	if (cfg_sw_read()->telnet_enable)
		telnet_init();
	
	/* Create HTTP task. */
	if (cfg_sw_read()->http_enable)
		http_init();

	/* Create CM task, it initializes IMPI msg service */
	cm_init();
	
	/*Start LCD display task only if we code for the DISCOVERY version of the product */
	if (cfg_get_board()->type & BOARD_NAMC_DISCOVERY_MASK) {
		/* initilize the LCD display*/
		lcd_init();
		/* spawns the button task which detect when a button is pressed */
		spawn_button_task();
		/* checks if we in the extender mode meaning that we don't need to bootup or start tasks. */
		if (ioport_get_pin_level(PIN_MODE_CHECK) == cfg_get_board()->pin_mode_check_is_extender) {
			ioport_set_pin_level(MUX_CONTROL, MUX_CONTROL_INACTIVE_LEVEL);
			printd("Extender Mode on.\n");
			printd("Display will display wrong information.\n");
			while(1){
				if (ioport_get_pin_level(PIN_MODE_CHECK) == cfg_get_board()->pin_mode_check_is_nonextender)
				{
					NVIC_SystemReset();
				}
			}
		}
	}
	
	/* Start the scheduler. */	
	vTaskStartScheduler();
	/* If all is well, the scheduler will now be running, and the following line
	will never be reached.  If the following line does execute, then there was
	insufficient FreeRTOS heap memory available for the idle and/or timer tasks
	to be created.  See the memory management section on the FreeRTOS web site
	for more details. */

	/* Will only get here if there was insufficient memory to create the idle task. */
	return 0;
}



/*---------------------------------------------------------------------------*
Function: prvSetupHardware
Description:
Notes:
Parameters:		none
Return:			none
*-----------------------------------------------------------------------------*/
static void prvSetupHardware(void)
{
	/* ASF function to setup clocking. CPU Speed is 120 MHz */
	sysclk_init();

	/* Disable watchdog */
	wdt_disable(WDT);

	/* Ensure all priority bits are assigned as preemption priority bits. */
	NVIC_SetPriorityGrouping(0);

	/* Initialize FreeRTOS Trace */
#if (configUSE_FREERTOS_TRACE == 1)
	vTraceInitTraceData();
	//uiTraceStart();
#endif

	/* Initialize and set all GPIO to correct state */
	io_init();

	/* Configure UART0 console for serial debug output */
	configure_console();

	/* Clear command window */
	printd("\n========================================\n");

	print_welcome_msg();

	/* Initialize ADC module */
	adc_init();
}

static void adc_init(void)
{
	struct afec_config afec_cfg;

	afec_get_config_defaults(&afec_cfg);
	
	afec_cfg.settling_time = AFEC_SETTLING_TIME_3;
	afec_cfg.tracktim = 0xf; // Results in 40 uS hold time

	afec_enable(AFEC0);
	afec_init(AFEC0, &afec_cfg);
	//afec_set_trigger(AFEC0, AFEC_TRIG_SW);

	afec_enable(AFEC1);
	afec_init(AFEC1, &afec_cfg);
	//afec_set_trigger(AFEC1, AFEC_TRIG_SW);
}


/*---------------------------------------------------------------------------*
Function:		get_run_time_counter_value()
Description:	Gets the acutal conter value of timer configured by FreeRTOSconfig.h
Notes:			Configured by configure_timer_for_run_time_stats()
Parameters:		none
Return:			Actual conter value
*-----------------------------------------------------------------------------*/
uint32_t get_run_time_counter_value(void)
{
	return TC0->TC_CHANNEL[1].TC_CV;
}

/*---------------------------------------------------------------------------*
Function:		configure_timer_for_run_time_stats
Description:	Configures timer 0 channel 1 for FreeRTOS task stats. It is
				needed for measuring the acutal CPU load of each task.
Notes:			Called by vTaskGetRunTimeStats
Parameters:
Return:			none
*-----------------------------------------------------------------------------*/
void configure_timer_for_run_time_stats(void)
{
	pmc_enable_periph_clk(ID_TC1);
	tc_init(TC0, 1,			// Init timer counter 0 channel 1.
	TC_CMR_TCCLKS_TIMER_CLOCK5		// Use slow clock to avoid overflow.
	);

	tc_write_rc(TC0, 1, 0xffffffff);	// Load the highest possible value into TC.

	tc_start(TC0, 1);					// Start Timer counter 0 channel 1.
}

/*-----------------------------------------------------------*/

void vApplicationMallocFailedHook(void)
{
	static volatile uint32_t ulCount = 0;
	/* vApplicationMallocFailedHook() will only be called if
	configUSE_MALLOC_FAILED_HOOK is set to 1 in FreeRTOSConfig.h.  It is a hook
	function that will get called if a call to pvPortMalloc() fails.
	pvPortMalloc() is called internally by the kernel whenever a task, queue,
	timer or semaphore is created.  It is also called by various parts of the
	demo application.  If heap_1.c or heap_2.c are used, then the size of the
	heap available to pvPortMalloc() is defined by configTOTAL_HEAP_SIZE in
	FreeRTOSConfig.h, and the xPortGetFreeHeapSize() API function can be used
	to query the size of free heap space that remains (although it does not
	provide information on how the remaining heap might be fragmented). 

	Just count the number of malloc fails as some failures may occur simply
	because the network load is very high, resulting in the consumption of a
	lot of network buffers. */
	ulCount++;	
	
	printd("\r\nvApplicationMallocFailedHook(%ld)\n", ulCount);
}

/*-----------------------------------------------------------*/

void vApplicationIdleHook(void)
{
	/* vApplicationIdleHook() will only be called if configUSE_IDLE_HOOK is set
	to 1 in FreeRTOSConfig.h.  It will be called on each iteration of the idle
	task.  It is essential that code added to this hook function never attempts
	to block in any way (for example, call xQueueReceive() with a block time
	specified, or call vTaskDelay()).  If the application makes use of the
	vTaskDelete() API function (as this demo application does) then it is also
	important that vApplicationIdleHook() is permitted to return to its calling
	function, because it is the responsibility of the idle task to clean up
	memory allocated by the kernel to any task that has since been deleted. */
}

/*-----------------------------------------------------------*/

void vApplicationStackOverflowHook(xTaskHandle *pxTask, signed char *pcTaskName)
{

	printex("\n ###### Stack overflow by Task %s ######\n", (char*)pcTaskName);
	//cu_set_fanduty(100);
	/* Run time stack overflow checking is performed if
	configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2.  This hook
	function is called if a stack overflow is detected. */
	vAssertCalled( __LINE__, __FILE__ );
}

/*-----------------------------------------------------------*/

void vApplicationTickHook(void)
{
	//vTracePrintF(xTraceOpenLabel("Timer"), "Timer handler!");
	g_uptime_ms++; // Used to determine the time the kernel is running already.

	/* This function will be called by each tick interrupt if
	configUSE_TICK_HOOK is set to 1 in FreeRTOSConfig.h.  User code can be
	added here, but the tick hook is called from an interrupt context, so
	code must not attempt to block, and only the interrupt safe FreeRTOS API
	functions can be used (those that end in FromISR()). */
}

/*-----------------------------------------------------------*/
void vAssertCalled( uint32_t ulLine, const char *pcFile )
{
/* The following two variables are just to ensure the parameters are not
optimised away and therefore unavailable when viewed in the debugger. */
	volatile uint32_t ulLineNumber = ulLine, ulSetNonZeroInDebuggerToReturn = 0;
	volatile const char * const pcFileName = pcFile;
	extern pwm_channel_t pwm_fan_ctl;
	
	ioport_set_pin_level(PIN_LED_ERR, LED_ACTIVE_LEVEL);
	ioport_set_pin_level(PIN_LED_OK, LED_INACTIVE_LEVEL);
	pwm_channel_update_duty(PWM, &pwm_fan_ctl, 100);
	
	printex("\nAssert triggered. File %s, Line %d\n", pcFile, ulLine);
	
	taskENTER_CRITICAL();
	while( ulSetNonZeroInDebuggerToReturn == 0 )
	{
		#if (HALT_ON_ASSERTION == 0)
			NVIC_SystemReset();
		#endif
		/* If you want to set out of this function in the debugger to see the
		assert() location then set ulSetNonZeroInDebuggerToReturn to a non-zero
		value. */
	}
	taskEXIT_CRITICAL();

	( void ) pcFileName;
	( void ) ulLineNumber;
}
