
/*
 * button.c
 *
 * Created: 17.01.2019 11:20:30
 *  Author: welbers
 */ 

#include <string.h>
#include <asf.h>
#include "conf_board.h"
#include "FreeRTOS.h"
#include "FreeRTOSuser.h"
#include "FreeRTOSConfig.h"
#include "cfg.h"
#include "ioport.h"
#include "lcd.h"
#include "task.h"
#include "button.h"
#include "task_defs.h"
#include "sys.h"

#define BOUNCE_DELAY 100
PRIVATE void button_handler(uint32_t pio_id, uint32_t pin);
void task_button(void);
struct button_device {
	sys_sem_t sem;
	uint8_t button_task_running;
};
static struct button_device button_dev;
/**
 * \brief spawns the button task
 *
 * \param   none
 * \return  none
 *
 */

void spawn_button_task(void)
{
	TSPAWN("button", tskIDLE_PRIORITY, configMINIMAL_STACK_SIZE*3, task_button, NULL, NULL);
}

/**
 * \brief initialize button
 *
 * \param   none
 * \return  none
 *
 */
void button_init(void)
{
	/* Set LCD button pins as input , they have hardware pull up and enable debounce */
	ioport_set_pin_dir(PIN_LCD_BUTTON1 ,IOPORT_DIR_INPUT);
	ioport_set_pin_dir(PIN_LCD_BUTTON2 ,IOPORT_DIR_INPUT);
	ioport_set_pin_dir(PIN_LCD_BUTTON3 ,IOPORT_DIR_INPUT);
	/* enable clock */
	pmc_enable_periph_clk(ID_PIOD);

	/* add handler and interrupt to the button pins */
	pio_handler_set(PIN_LCD_BUTTON1_PIO, PIN_LCD_BUTTON1_ID, PIN_LCD_BUTTON1_MASK, PIO_IT_FALL_EDGE, button_handler);
	pio_handler_set(PIN_LCD_BUTTON2_PIO, PIN_LCD_BUTTON2_ID, PIN_LCD_BUTTON2_MASK, PIO_IT_FALL_EDGE, button_handler);
	pio_handler_set(PIN_LCD_BUTTON3_PIO, PIN_LCD_BUTTON3_ID, PIN_LCD_BUTTON3_MASK, PIO_IT_FALL_EDGE, button_handler);
	
	/* enable interrupt */
	NVIC_SetPriority(PIOD_IRQn, /*configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY*/1);
	NVIC_EnableIRQ(PIOD_IRQn);
	pio_enable_interrupt(LCD_BUTTON_PIO, LCD_BUTTON_MASKS);
}

/**
 * \brief interrupt handler
 *
 * \param   pio_od  the pio_id where a interrupt is triggered
 * \param   pin     the pin which triggered the interrupt
 * \return  none
 *
 */
PRIVATE void button_handler(uint32_t pio_id, uint32_t pin){
	/* Disable interrupts for all buttons */
	portBASE_TYPE xBUTTONTaskWoken = pdFALSE;
	if (button_dev.button_task_running == 1 && button_dev.sem != NULL){
		/* disable interrupt for a moment until we are done processing information */
		pio_disable_interrupt(LCD_BUTTON_PIO, LCD_BUTTON_MASKS);
		xSemaphoreGiveFromISR(button_dev.sem, &xBUTTONTaskWoken);
		portEND_SWITCHING_ISR(pdFALSE);
		/* enable interrupt again since we are done processing information */
		pio_enable_interrupt(LCD_BUTTON_PIO, LCD_BUTTON_MASKS);
	}
	return;
}

/**
 * \brief Task which checks if a button is pressed and if so changes information.
 *
 * \param   none
 * \return  none
 *
 */
void task_button(void)
{
	uint8_t unmanaged_counter = 0;
	while (init_status.task_lcd_init != true){
		TSLEEP(2);
	}
	sys_sem_new(&button_dev.sem, 0);
	button_init();
	button_dev.button_task_running = 1;
	while (1){
		if (button_dev.sem != NULL) {
			/* start collecting information */
			sys_arch_sem_wait(&button_dev.sem, 0);
			/* Detect which button is pressed and proceed in the given if then*/
			if (ioport_get_pin_level(PIN_LCD_BUTTON1) == IOPORT_PIN_LEVEL_LOW && ioport_get_pin_level(PIN_LCD_BUTTON2) != IOPORT_PIN_LEVEL_LOW && ioport_get_pin_level(PIN_LCD_BUTTON3) != IOPORT_PIN_LEVEL_LOW){
				/* changes the sensor number in array place 1 */ 
				next_sensor(1);
				/* refresh LCD display line 2 */
				lcd_refresh(2);
				/* Bouncing protection */
				TSLEEP(BOUNCE_DELAY);
			}
			/* line 3 (MCH sensors) refresh */
			if (ioport_get_pin_level(PIN_LCD_BUTTON2) == IOPORT_PIN_LEVEL_LOW && ioport_get_pin_level(PIN_LCD_BUTTON1) != IOPORT_PIN_LEVEL_LOW && ioport_get_pin_level(PIN_LCD_BUTTON3) != IOPORT_PIN_LEVEL_LOW){
				/* changes the sensor number in array place 2 */ 
				next_sensor(2);
				/* refresh LCD display line 3 */
				lcd_refresh(3);
				/* Bouncing protection */
				TSLEEP(BOUNCE_DELAY);
			}
			/* line 4 (AMC sensors) refresh */
			if (ioport_get_pin_level(PIN_LCD_BUTTON3) == IOPORT_PIN_LEVEL_LOW && ioport_get_pin_level(PIN_LCD_BUTTON2) != IOPORT_PIN_LEVEL_LOW && ioport_get_pin_level(PIN_LCD_BUTTON1) != IOPORT_PIN_LEVEL_LOW){
				/* changes the sensor number in array place 3 */ 
				next_sensor(3);
				/* refresh LCD display line 4 */
				lcd_refresh(4);
				/* Bouncing protection */
				TSLEEP(BOUNCE_DELAY);
			}
			/* if all buttons are pressed at the same time for a given amount of time reset the software and start it in UNMANAGMENT MODE/MANAGMENT MODE */
			/* The unmanagment mode does not have any communication the PCB which is getting tested. */
			if ((ioport_get_pin_level(PIN_LCD_BUTTON1) == IOPORT_PIN_LEVEL_LOW) && (ioport_get_pin_level(PIN_LCD_BUTTON2) == IOPORT_PIN_LEVEL_LOW) && (ioport_get_pin_level(PIN_LCD_BUTTON3) == IOPORT_PIN_LEVEL_LOW)){
				while (((ioport_get_pin_level(PIN_LCD_BUTTON1) == IOPORT_PIN_LEVEL_LOW) && \
				(ioport_get_pin_level(PIN_LCD_BUTTON2) == IOPORT_PIN_LEVEL_LOW) && \
				(ioport_get_pin_level(PIN_LCD_BUTTON3) == IOPORT_PIN_LEVEL_LOW)) == 1){
					if (unmanaged_counter == 30){
						TSLEEP(50);
						CFG_SW		*cfg_sw = cfg_get_sw();				/* This config structure is used by the firmware only */
						if (cfg_sw->emch_flags & CFG_AMC1_UNMANAGED){
							cfg_sw->emch_flags &=~ CFG_AMC1_UNMANAGED;
							printd("You are using AMC1(5) in managment mode now\n");
						} else {
							cfg_sw->emch_flags |= CFG_AMC1_UNMANAGED;
							printd("You are using AMC1(5) in unmanagment mode now\n");
						}
				
						cfg_sw_write();
						cfg_base_write();
						unmanaged_counter = 0;
						NVIC_SystemReset();
					}
					unmanaged_counter++;
					TSLEEP(100);
				}
			} else {
				unmanaged_counter = 0;
			}
			TSLEEP(BOUNCE_DELAY);
		}
	}
}
