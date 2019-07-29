/******************************************************************************
*******************************************************************************
*                                                                             *
*     NNNNNN         N                A             TTTTTTT TTTTTTTTTT        *
*     N NNNNN        N               AAA                  T TTTTT             *
*     NN NNNNN       N              AAAAA                 T TTTTT             *
*     N N NNNNN      N             A AAAAA                T TTTTT             *
*     N  N NNNNN     N            A A AAAAA               T TTTTT             *
*     N   N NNNNN    N           A   A AAAAA              T TTTTT             *
*     N    N NNNNN   N          A     A AAAAA             T TTTTT             *
*     N     N NNNNN  N         AAAAAAAAA AAAAA            T TTTTT             *
*     N      N NNNNN N        A         A AAAAA           T TTTTT             *
*     N       N NNNNNN       A           A AAAAA          T TTTTT             *
*     N        N NNNNN  OO  A             A AAAAA  OO     T TTTTT     OO      *
*                       OO                         OO                 OO      *
*                                                                             *
*     Gesellschaft fuer Netzwerk- und Automatisierungstechnologie m.b.H       *
*        Konrad-Zuse-Platz 9, D-53227 Bonn, Tel.:+49 228/965 864-0            *
*                            www.nateurope.com                                *
*                                                                             *
*******************************************************************************
*******************************************************************************
*
* Module      : lcd.c
*
* Description :
*
* Author      : Tom Welbers
*
******************************************************************************
******************************************************************************
*
*       Copyright (c) by N.A.T. GmbH
*
* All rights reserved. Copying, compilation, modification,
* distribution or any other use whatsoever of this material
* is strictly prohibited except in accordance with a Software
* License Agreement with N.A.T. GmbH.
*
******************************************************************************
******************************************************************************/


/*****************************************************************************/
/*  NOTES		                                                             */
/*****************************************************************************/
/*
	the code with it not commended in  generate_sensor_value and lcd_NumberWithExpo is form helmut.
	Since he does not comment out how his magic numbers and code works i was unable to figure it out in the time i worked with it ( ~ 1day)
	Because of this i left it without commands.
*/
/*****************************************************************************/
/*  INCLUDES                                                                 */
/*****************************************************************************/
#include <string.h>
#include <asf.h>
#include "lcd.h"
#include "ethernet.h"
#include "FreeRTOS.h"
#include "FreeRTOSuser.h"
#include "FreeRTOS_clib.h"
#include "task_defs.h"
#include "twi.h"
#include "cm.h"
#include "ipmi_fruinfo.h"
#include "ip_addr.h"
#include "ipmi_lib.h"
#include "conf_board.h"
#include "conf_platform.h"
#include "88e6320.h"
#include "button.h"
#include "buildinfo.h"
#include "emch_rel.h"
#include "ipmi.h"
#include "twi_i2c.h"


/*****************************************************************************/
/*  PRIVATE MACROS                                                           */
/*****************************************************************************/


/*****************************************************************************/
/*  EXTERNAL REFERENCES                                                      */
/*****************************************************************************/

/*****************************************************************************/
/*  PRIVATE FUNCTION PROTOTYPES                                              */
/*****************************************************************************/

void task_lcd_init(void *pvParameters);
uint8_t check_threshold ( char state_buf, uint8_t sensorReading, uint8_t sensor_id);
uint8_t lcd_sendcommand (unsigned char command, uint8_t cd);
char *get_amc_name(void);
uint8_t lcd_senddata (unsigned char command, uint8_t cd);
void lcd_sendinit ();
void lcd_send ();
void generate_lcd_send();
void lcd_setpos(char x, char y);
void lcd_refresh_line_two();
void lcd_refresh_line_sensors(int i);
void lcd_refresh_line_one();
void check_full_sensor(uint8_t line_select,char *sensorPtr,uint8_t fru_id);
void generate_sensor_name(char *sensorPtr, uint8_t sensor_id);
void generate_sensor_value(uint8_t sensor_id,uint8_t line_select, char *sensorPtr,char sensorReading);
void lcd_NumberWithExpo(int value, unsigned char ipmiExpo, unsigned char lin,uint8_t sensor_id);
void generate_units(unsigned char *sensorPtr ,uint8_t sensor_id);
uint8_t lcd_sendcommand (unsigned char command, uint8_t cd);
void lcd_setrom (unsigned char rom);


/*****************************************************************************/
/*  PRIVATE GLOBALS                                                          */
/*****************************************************************************/

unsigned char lineset[4][20] = {{}};
twi_package_t packet_write;
IPMI_DB	*fru;
PRIVATE unsigned button_flag = 0;
PRIVATE unsigned refresh_flag = 0;
PRIVATE unsigned name_flag = 0;
PRIVATE unsigned flag = 0;
PRIVATE unsigned auto_change_flag = 0;
PRIVATE unsigned char buffer_SVN[80] ={0x00};

/*****************************************************************************/
/*  PUBLIC FUNCTION DEFINITIONS                                              */
/*****************************************************************************/

#define ip2str(ip_ulong)			ipaddr_ntoa((ip_addr_t *)(&ip_ulong))

struct LCD_def temp = {
	.sensor_number = {0},
	.lcd_sensor_name = {{0}},
	.lcd_sensor_value = {{0}},
	.lcd_sensor_unit = {{0}},
	.offset = { 0, 0 },
	.value_threshold = { 0 , 0 },
	.fru_id_sensors ={},
	.buff_lcd_sensor_name ={{}},
	.buf_number = {0},
	.lcd_refresh = {1,1},
};

struct LCD_def *tempvalue = &temp;

struct LCD_def *lcd_def_get(void) {
	return (tempvalue);
}



/**
 * \brief starts the LCD task
 *
 * \param   none
 * \return  none
 *
 */
void lcd_init(void)
{
	TSPAWN("lcd", tskIDLE_PRIORITY, configMINIMAL_STACK_SIZE*2, task_lcd_init, NULL, NULL);
}

/**
 * \brief The LCD task
 *
 * \param   none
 * \return  none
 *
 */
void task_lcd_init(void *pvParameters)
{
	while (get_ipmiDB(0)->state < FRU_STATE_M4) TSLEEP(10);
	/* Wait for all ipmi service tasks have been started */
	while (init_status.task_tx != true || init_status.task_rx != true || init_status.task_rxQ != true || init_status.task_ager != true)
	TSLEEP(2);
	/* give FRU number of eMCH and AMC to the structure so the system knows where to get the sensor infos from */
	tempvalue->fru_id_sensors[MCH_SENSORS] = 3;
	tempvalue->fru_id_sensors[AMC_SENSORS] = 5;
	/* initialize the LCD display the code is given in the LCD display example code from the LCD controller company */
	lcd_sendinit();
	init_status.task_lcd_init = true;
	while (1){
		/* waits for lcd_refresh to be done */
		/* checks if we are in loop back if so changed MUX to send Ethernet signal to cables not to front ports.*/
		if (ioport_get_pin_level(PIN_LOOPBACK_CTRL) == IOPORT_PIN_LEVEL_LOW){
			ioport_set_pin_level(MUX_CONTROL, MUX_CONTROL_INACTIVE_LEVEL);
		} else {
			ioport_set_pin_level(MUX_CONTROL, MUX_CONTROL_ACTIVE_LEVEL);
		}
		/* lcd_send() refreshes all lines.*/
		lcd_send();
		TSLEEP(500);
	}
	/* Kill ourselves */
	TKILL(NULL);
}

/**
 * \brief sending and refreshing the LCD data
 *
 * \param   none
 * \return  none
 *
 */
void lcd_send ()
{

	/* Generates the arrays for sending */
	generate_lcd_send();	
	/* send each element of the 4 arrays */
		for (uint8_t i=0; i < 4; i++){
			/* button_flag pauses the task when a button is pressed. This is because sometimes when both would happen 
			at ones it glitches and prints wrong stuff on the LCD display*/
			while (button_flag == 1){
				TSLEEP(1);
			}
			refresh_flag = 1;
			lcd_setpos(1,i);
			lcd_senddata(i,0);
			refresh_flag = 0;
		}
}

/**
 * \brief generating the data for the LCD display
 *
 * \param   none
 * \return  none
 *
 */
void generate_lcd_send()
{
	CFG_SW	*cfg_sw = cfg_get_sw();
	/* check if unmanaged mode if then refresh only some lines and display unmanaged mode */
	if (cfg_sw->emch_flags & CFG_AMC1_UNMANAGED){
		strncpy(lineset[LCD_DISPLAY_LINE_ONE],"Unmanaged mode!",15); /* 15 =  length of "Unmanaged mode!"*/
		lcd_refresh_line_two();
		strncpy(lineset[LCD_DISPLAY_LINE_FOUR],"Unmanaged mode!",15); /* 15 =  length of "Unmanaged mode!"*/
		lcd_refresh_line_sensors(1);
		name_flag = 0;
		return;
	} else if (ioport_get_pin_level(PIN_MODE_CHECK) == cfg_get_board()->pin_mode_check_is_extender) {
		/* mode_cheak is high for Standalone mode if not standalone show that extender mode is active*/
		strncpy(lineset[LCD_DISPLAY_LINE_ONE],"                    ",20); /*20 = length of "...."*/
		strncpy(lineset[LCD_DISPLAY_LINE_ONE],"Extender Mode!",14); /* 14 =  length of "Extender Mode!"*/
		strncpy(lineset[LCD_DISPLAY_LINE_TWO],"                    ",20); /*20 = length of "...."*/
		strncpy(lineset[LCD_DISPLAY_LINE_THREE],"                    ",20); /*20 = length of "...."*/
		strncpy(lineset[LCD_DISPLAY_LINE_FOUR],"                    ",20); /*20 = length of "...."*/
		NVIC_SystemReset();
		return;
	} else if (!((ioport_get_pin_level(PIN_EN_AMC1) == EN_ACTIVE_LEVEL) &&  (ioport_get_pin_level(PIN_PS1_AMC1) == PS_ACTIVE_LEVEL))){
		/* Check if module is present and enabled if not then show no AMC is found*/
		strncpy(lineset[LCD_DISPLAY_LINE_ONE],"                    ",20); /*20 = length of "...."*/
		strncpy(lineset[LCD_DISPLAY_LINE_ONE],"NO AMC FOUND",12); /*12 = length of "NO AMC FOUND"*/
		lcd_refresh_line_two();
		lcd_refresh_line_sensors(1);
		lcd_refresh_line_sensors(0);
		name_flag = 0;
		return;
	}
	/* refresh lcd display lines one to four */
	lcd_refresh_line_one();
	lcd_refresh_line_two();
	lcd_refresh_line_sensors(1);
	lcd_refresh_line_sensors(0);
}

/**
 * \brief Generates/refresh data for LCD display line 1
 *
 * \param   none
 * \return  none
 *
 */
void lcd_refresh_line_one()
{
	int memcpysize = 0;
	if( name_flag == 0){
		unsigned char *buf_ptr = get_amc_name();
		strncpy(lineset[LCD_DISPLAY_LINE_ONE],"                    ",20); /*20 = length of "...."*/
		/* if communication failed show comm error*/
		if(tempvalue->amc_name_lenght > (int)sizeof(lineset[LCD_DISPLAY_LINE_ONE])){
			memcpysize = MAX_SYMBOL_LENGHT;
			} else {
			memcpysize = tempvalue->amc_name_lenght;
		}
		if (fru->state == FRU_STATE_M7)
		{
			memcpy(lineset[LCD_DISPLAY_LINE_ONE], "comm error", MAX_SYMBOL_LENGHT);
			memcpysize = 10;
		}
		else{
			memcpy(lineset[LCD_DISPLAY_LINE_ONE], buf_ptr, MAX_SYMBOL_LENGHT);
		}
		/* Adding spaces until the m state of the display is located */
		for (int i = memcpysize;i<AMC_MAX_NAME_LENGTH;i++){
			lineset[LCD_DISPLAY_LINE_ONE][i] = ' ';
		}		
		/* Adding the Symbol M for the frustate*/
		lineset[LCD_DISPLAY_LINE_ONE][M_STATE_SYMBOL_LOCATION] = SYMBOL_M;
		/* shifting frustate to delete the 4 LB*/
	}
	name_flag = 1;
	int frustate =  (fru->state>>4);
	if (frustate > 0 && frustate <= 7) {
		/* Convert number to ASCII equivalent and adding it to lineset*/
		lineset[LCD_DISPLAY_LINE_ONE][M_STATE_LOCATION] = '1' + frustate - 1;
		} else {
		lineset[LCD_DISPLAY_LINE_ONE][M_STATE_LOCATION] = 'X';
	}
	
}

/**
 * \brief Generates/refresh data for LCD display line 2 and changes what line 2 displays
 *
 * \param   none
 * \return  none
 *
 */
void lcd_refresh_line_two(){
	char AMC_PORT[] = "//";
	char FRONT_PORT[] = "//";
	struct netif *p_netif;
	char *valueip;
	memset(lineset[LCD_DISPLAY_LINE_TWO], 0x00,sizeof(lineset[0]));
	/* This is the automatic changing of the information on line. every 10 interval */
	if (tempvalue->buf_number[1] <= 9 && auto_change_flag == 0) {
		tempvalue->sensor_number[1] = 0;
	} else if (tempvalue->buf_number[1] >= 10 && tempvalue->buf_number[1] <=19 && auto_change_flag == 0) {
		tempvalue->sensor_number[1] = 1;
	} else if(tempvalue->buf_number[1] >= 20 && auto_change_flag == 0) {
		tempvalue->sensor_number[1] = 2;
	}
	/* switch case to check which data is suppose to be displayed */
	switch (tempvalue->sensor_number[1]) {
		/* displays the ip adress of the eMCH*/
	case 0:
		p_netif = ethernet_get_netif();
		valueip = ip2str(p_netif->ip_addr.addr);
		sprintf(lineset[LCD_DISPLAY_LINE_TWO],"MCH: %15s",valueip);
		tempvalue->buf_number[1]++;
		break;
		/* shows the link states of the ethernet connection */
	case 1:
		if(switch_88e6320_get_link_status(1) > 0) {
			AMC_PORT[0] = LINK_UP;
		} else {
			AMC_PORT[0] = LINK_DOWN;
		}
		if (switch_88e6320_get_link_status(0) > 0) {
			AMC_PORT[1] = LINK_UP;
		} else {
			AMC_PORT[1] = LINK_DOWN;
		}
		if (switch_88e6320_get_link_status(3) > 0) {
			FRONT_PORT[1] = LINK_UP;
		} else {
			FRONT_PORT[1] = LINK_DOWN;
		}
		if (switch_88e6320_get_link_status(4) > 0) {
			FRONT_PORT[0] = LINK_UP;
		} else {
			FRONT_PORT[0] = LINK_DOWN;
		}
		sprintf(lineset[LCD_DISPLAY_LINE_TWO],"LINK: AMC %s RJ45 %s",AMC_PORT,FRONT_PORT);
		tempvalue->buf_number[1]++;
		break;
		/* shows the firmware version and SVN */
	case 2:
		memset(lineset[LCD_DISPLAY_LINE_FOUR],' ',sizeof(lineset[0])+1);
		if (flag == 0){
			sprintf(buffer_SVN," EMCH Firmware V%d.%d.%d (r%d)",EMCH_VERSION_MAJOR,EMCH_VERSION_MINOR,			EMCH_VERSION_PATCH,EMCH_VERSION_REVISION);
			flag = 1;
		}		/* checks how long the name is and saves it into k */
		uint8_t k = strlen(buffer_SVN);		/* scrolls the data if k is larger than 20*/		if (k >= 20) {
			char buf_name = buffer_SVN[0];
			for (uint8_t i=0; i  < k-1; i++){
				buffer_SVN[i] = buffer_SVN[i+1];
			}
			buffer_SVN[k-1]=buf_name;
		}
		sprintf(lineset[LCD_DISPLAY_LINE_TWO], "%s",buffer_SVN);
		tempvalue->buf_number[1]++;
		break;
	default:
		break;
	}
	if (tempvalue->buf_number[1] >= 31) {
		tempvalue->buf_number[1] = 0;
		flag = 0;
	}
}

/**
 * \brief lcd_refresh_line_sensors
 *
 * \param  none
 * \return none
 *
 */
void lcd_refresh_line_sensors(int i){
	uint8_t line_select = 0;
	uint8_t sensor_id = 0;
	uint8_t treshhold_shift;
	/* this switch case selects first LCD display line three to display AMC sensor and in 2 for loop LCD line four and MCH sensors */
	switch(i) {
		case 0:
		line_select = LCD_DISPLAY_LINE_FOUR;
		sensor_id = AMC_SENSORS;
		break;
		case 1:
		line_select = LCD_DISPLAY_LINE_THREE;
		sensor_id = MCH_SENSORS;
		break;
		default:
		break;
	}
	fru = get_ipmiDB(tempvalue->fru_id_sensors[i]);
	/* Protection while to wait until AMC fru data is copied.*/
	if ((fru->state <= FRU_STATE_M1 || fru->state == FRU_STATE_M7) /*&& sensor_id == AMC_SENSORS */) {
		strncpy(lineset[LCD_DISPLAY_LINE_FOUR],"                    ",20); /*20 = length of "...."*/
		strncpy(lineset[LCD_DISPLAY_LINE_FOUR],"NO AMC FOUND",12); /*12 = length of "NO AMC FOUND"*/
		return;
	}
	/* gets data for the sensor*/
	unsigned char *sensorPtr = (unsigned char*)&(fru->sensor[tempvalue->sensor_number[line_select]]);
	/* checks if real sensor if not change to next sensor and check again */
	check_full_sensor(line_select,sensorPtr,tempvalue->fru_id_sensors[i]);
	/* fru get ipmiDB again since fru can change during check_full_sensor in case of a new sensor ( old one was fake) */
	fru = get_ipmiDB(tempvalue->fru_id_sensors[i]);
	/* get fru data for specific sensor*/
	sensorPtr = (unsigned char*)&(fru->sensor[tempvalue->sensor_number[line_select]]);
	char sensorReading = sensorPtr[13];
	generate_sensor_name(sensorPtr, sensor_id);
	generate_sensor_value(sensor_id,line_select,sensorPtr,sensorReading);
	/* EMPTY all lines. */
	memset(lineset[line_select], 0x00,sizeof(lineset[0]));
	/* Checks if Threshold is == 1 */
	treshhold_shift = check_threshold(fru->result[3],sensorReading,sensor_id);
	lineset[line_select][0] = tempvalue->value_threshold[sensor_id];
	strncpy(lineset[line_select]+treshhold_shift, tempvalue->lcd_sensor_name[sensor_id], strnlen(tempvalue->lcd_sensor_name[sensor_id],NAME_LENGTH));
	uint8_t k = strlen(tempvalue->lcd_sensor_name[sensor_id]);
	/* This if/else adds the : after the temperature sensor name */
	if (k >=12) {
		k = NAME_LENGTH + treshhold_shift;
		for (; k<20;k++) {
			lineset[line_select][k] = ' ';
		}
		lineset[line_select][NAME_LENGTH+treshhold_shift] = ':';
	} else {
		for (; k<20;k++) {
			lineset[line_select][k] = ' ';
		}
		lineset[line_select][strlen(tempvalue->lcd_sensor_name[sensor_id])+treshhold_shift] = ':';
	}
	
	strncpy(lineset[line_select] + (MAX_SYMBOL_LENGHT-strnlen(tempvalue->lcd_sensor_value[sensor_id],SENSOR_VALUE_LENGTH)-strnlen(tempvalue->lcd_sensor_unit[sensor_id],UNIT_LENGTH)), tempvalue->lcd_sensor_value[sensor_id], SENSOR_VALUE_LENGTH);
	strncpy(lineset[line_select] + (MAX_SYMBOL_LENGHT-strnlen(tempvalue->lcd_sensor_unit[sensor_id],UNIT_LENGTH)),tempvalue->lcd_sensor_unit[sensor_id],UNIT_LENGTH);
}

/**
 * \brief returns pointer to AMC name
 *
 * \param  none
 * \return pointer to AMC name
 *
 */
char *get_amc_name(void){
	/* This is from ipmi_fruinfo.c just modified for the needs*/
	fru = get_ipmiDB(5);
	/* from IpmiFru_PrintRecords function */
	unsigned char *buf = fru->fruData;
	int rec_offs = buf[FRU_BOARD_INFO] *8;
	/* from PrintBoardInfo function*/
	unsigned char *buf_ptr;
	buf_ptr = &buf[rec_offs];
	int len = buf_ptr[1] *8;
	uint8_t lenInfo;
	buf_ptr = &buf_ptr[6];
	lenInfo=*buf_ptr++;
	/*from printFruInfoData function*/
	len = (lenInfo & 0x3f);
	buf_ptr =&buf_ptr[len];
	/* from PrintBoardInfo function*/
	lenInfo=*buf_ptr++;
	/*from printFruInfoData function*/
	len = (lenInfo & 0x3f);
	tempvalue->amc_name_lenght = len;
	return buf_ptr;
}

/**
 * \brief Task which send the wanted information to the LCD-Display using TWI
 *
 * \param command   the data which needs to be send ( example 0x80)
 * \param cd        if its an instructions ( during initialize phase) or if its a package of data
 * \return          none
 *
 */
uint8_t lcd_senddata (unsigned char command, uint8_t cd)
{
	uint8_t result = FAIL;
	uint8_t timeout = 0;
	memset((void*)&packet_write, 0x00, sizeof(packet_write));
	uint8_t command_tx[21] = {};
	for (int  i = 1; i<22 ;i++) {
		command_tx[i] = lineset[command][i-1];
	}
	
	if (cd == 1) {
		/* add 0x80 for command instructions */
		command_tx[0] = COMMAND_COMMAND;
	} else if ( cd == 0) {
		/* add 0x40 for to define that the next package is data */
		command_tx[0] = COMMAND_DATA;
	} else {
		printd("Error");
		return(-1);
	}
	packet_write.chip = (0x3C);						// TWI slave bus address
	packet_write.buffer = (uint8_t *) command_tx;	// transfer data destination buffer
	packet_write.length = 21;						// transfer data size (bytes)
	result = FAIL;
	while (result != TWI_SUCCESS){
		result = twi1_master_write(&packet_write);
		delay_ms(5);
		/* After a write cycle, wait 5 ms for the cycle to complete */
		if (result == TWI_SUCCESS) {
			//	printd(".");
			timeout = 0;
		} else {
			printd("#");
			if (timeout++ > 5) {
				break;
			}
		}
	}
	if (result != SUCCEED) {
		printd("I2C failed, code %d.\n", result);
		return FAIL;
	}
	return 0;
}

/**
 * \brief Calculates sensor values
 *
 * \param sensor_id      AMC or eMCH sensor is used
 * \param line_select    on which line the data needs to be saved
 * \param sensor_id      the sensor data taken from FRU
 * \param sensorReading  If a sensor is present or not
 * \return               none
 *
 */
void generate_sensor_value(uint8_t sensor_id,uint8_t line_select, char *sensorPtr,char sensorReading){
	unsigned char value;
	int M, B, Bexpo, y=0;
	volatile unsigned long z;
	int type = sensorPtr[3];
	switch(type) {
		/* this is the normal case */
		case 0x01:
			ipmiSendReqMsg(fru, IPMI_NETFN_SENSEVT_REQ, IPMI_CMD_GET_SENSOR_READING, sensorPtr[7]);
			value = fru->result[1];
			switch(sensorPtr[20] >> 6) {
			case 0:	z = (unsigned long)value;
				y = (int)z;
				break;
			case 1:	y = (char)value;
				break;
			case 2:	y = (char)value;
				break;
			case 3:	z = (unsigned long)value;
				y = (int)z;
				break;
			}
			/* calculates sensor value*/
			M = (short)((sensorPtr[24] | (sensorPtr[25] & 0x40) << 2));
			if(sensorPtr[25] & 0x80)
			M = M | 0xfffffe00;
			B = (short)(sensorPtr[26] | (sensorPtr[27] & 0x40) << 2);
			if(sensorPtr[27] & 0x80)
			B = B | 0xfffffe00;
			Bexpo = sensorPtr[29] & 0x7;
			if ((sensorPtr[29] & 0x8) == 0){	/* pos expo */
				while(Bexpo){
					B = B *10;
					Bexpo--;
				}
				} else {
				Bexpo = 8 - Bexpo;
				while (Bexpo){
					B = B / 10;
					Bexpo--;
				}
			}
			y = y * M + B;
			if (sensorReading == 1) {
				/* call a function for expo values */
				lcd_NumberWithExpo(y, sensorPtr[29] >> 4, sensorPtr[23] & 0x7f,sensor_id);
				} else if(sensorReading <= 0x0c) {			/* discrete sensors */
				memcpy(tempvalue->lcd_sensor_value[sensor_id], &value, sizeof(tempvalue->lcd_sensor_value[0]));
				} else {
				memcpy(tempvalue->lcd_sensor_value[sensor_id], &value, sizeof(tempvalue->lcd_sensor_value[0]));
			}
		break;
		case 0x03:
			memset(lineset[line_select], 0x00, MAX_SYMBOL_LENGHT);
			strncpy(tempvalue->lcd_sensor_name[sensor_id], "no sensor!", NAME_LENGTH);
		break;
		
		default:
			memset(lineset[line_select], 0x00, MAX_SYMBOL_LENGHT);
			strncpy(tempvalue->lcd_sensor_name[sensor_id], "no sensor!", NAME_LENGTH);
		break;
	}
}

/**
 * \brief calculates sensor value
 *
 * \param value      do not know code not from me
 * \param ipmiExpo   do not know code not from me
 * \param lin        do not know code not from me
 * \param sensor_id  AMC or eMCH sensor is used
 * \return             none
 *
 */
void lcd_NumberWithExpo(int value, unsigned char ipmiExpo, unsigned char lin,uint8_t sensor_id){
	unsigned char expo = ipmiExpo & 0x7;
	unsigned char buf[16] = {};
	unsigned char nachKomma[16] = {};
	static char returnbuf[8] = {};
	int temp;
	int i,len = 0;
	if((ipmiExpo & 0x8) == 0) {
		/* pos Exponent */
		sprintf(returnbuf,"%d",value);
		len = strlen(&buf[0]);
		len+=expo;
		while(expo > 0){
			printd("0");
			expo--;
		}
	} else {
		/* Linearization is 1/x */
		if(lin == 7) {
			if(value != 0) {
				expo = 8 - expo;
				temp = 1;
				for(i = 0; i < expo; i++)
				temp *= 10;
				sprintf(returnbuf, "%d", temp / value);
			}
			else {
				sprintf(returnbuf, "0");
			}
			len = strlen(&buf[0]);
		}

		else {
			/* neg Exponent */
			sprintf(buf,"%u",value);
			i = strlen(&buf[0]);
			expo = 8-expo;
			nachKomma[expo] = 0x00;
			while(expo) {
				if(i > 0) {
					nachKomma[expo-1] = buf[i-1];
					buf[i-1] = 0x00;
					} else {
					nachKomma[expo-1] = '0';
				}
				i--;
				expo--;
			}
			if(buf[0] == 0x00) {
				sprintf(buf,"%c",'0');
			}
			sprintf(returnbuf,/* sizeof(returnbuf),*/"%s.%s",buf,nachKomma);
			len = strnlen(buf, sizeof(buf)) + strnlen(nachKomma, sizeof(nachKomma)) + 1;
		}
	}
	/* checks if the unit is ampere, if so switch place . with the number in front
	this is becuase in emch the current is *10 but it cannot be changed earlier duo to precision reduction then */
	if(tempvalue->lcd_sensor_unit[sensor_id][0] == 'A' && sensor_id == MCH_SENSORS){
		for (int i = 4; i>=0 ; i--) {
			returnbuf[i] = returnbuf[i-1];
		}
		returnbuf[0] = ' ';
	}
	memcpy(tempvalue->lcd_sensor_value[sensor_id], returnbuf, sizeof(tempvalue->lcd_sensor_value[0]));
}


/**
 * \brief generates the sensor name and add it into the structure
 *
 * \param sensorPtr    the sensor data taken from FRU
 * \param sensor_id    AMC or eMCH sensor is used
 * \return             none
 *
 */
void generate_sensor_name(char *sensorPtr, uint8_t sensor_id){
	uint8_t idStringLen = 0;
	idStringLen = sensorPtr[47] & 0x3f;
	/* checks if a button was recently pressed if so collect a new name */
	if (tempvalue->lcd_refresh[sensor_id] == 1){
		generate_units(sensorPtr,sensor_id);
		memset(tempvalue->lcd_sensor_name[sensor_id],0,sizeof(tempvalue->lcd_sensor_name[sensor_id]));
		memset(tempvalue->buff_lcd_sensor_name[sensor_id],0,sizeof(tempvalue->buff_lcd_sensor_name[sensor_id]));
		/* saves the name in the structure */
		for(uint8_t i=0; i < idStringLen && i < MAX_SYMBOL_LENGHT; i++){
			tempvalue->buff_lcd_sensor_name[sensor_id][i] = sensorPtr[48+i];
		}
		/* adds a space after the name of the sensor*/
		uint8_t kt = strlen(tempvalue->buff_lcd_sensor_name[sensor_id]);
		tempvalue->buff_lcd_sensor_name[sensor_id][kt] = ' ';
		tempvalue->lcd_refresh[sensor_id] = 0;
	}
	/* Puts the name into the structure for names */
	for (uint8_t i=0; i < idStringLen && i < NAME_LENGTH; i++){
		tempvalue->lcd_sensor_name[sensor_id][i] = tempvalue->buff_lcd_sensor_name[sensor_id][i];
	}
	/* checks how long the name is and saves it into k */
	uint8_t k = strlen(tempvalue->buff_lcd_sensor_name[sensor_id]);
	/* checks if k is larger then name space. If so make the name scroll */
	if (k >= NAME_LENGTH) {
		char buf_sensor_name_temp = tempvalue->buff_lcd_sensor_name[sensor_id][0];
		for (uint8_t i=0; i  < k-1; i++) {
			tempvalue->buff_lcd_sensor_name[sensor_id][i] = tempvalue->buff_lcd_sensor_name[sensor_id][i+1];
		}
		tempvalue->buff_lcd_sensor_name[sensor_id][k-1]=buf_sensor_name_temp;
	}
	
}

/**
 * \brief Checks if the sensor is a full sensor ( if its a hardware sensor)
 *
 * \param line_select  on which line the data needs to be saved
 * \param sensorPtr    the sensor data taken from FRU
 * \param fru_id       number of which sensor the data is from
 * \return               none
 *
 * \ note   this actually checks if its a hardware sensor and if not it will skip it
 * \ note   this function is needed because there are fake sensor with version id or other information present
 *
 */
void check_full_sensor(uint8_t line_select,char *sensorPtr,uint8_t fru_id){
	uint8_t type = sensorPtr[3];
	while (type != 0x01 || tempvalue->sensor_number[line_select]  > fru->sensorNum-1){
		/* increases the sensor number of specific line */
		tempvalue->sensor_number[line_select] = tempvalue->sensor_number[line_select] + 1;
		/* checks if fru_id is MCH_SENSORS */
		if (fru_id == tempvalue->fru_id_sensors[MCH_SENSORS]){
			/* checks if buf_number is 3 or bigger if so reduce it to 0*/
			if (tempvalue->buf_number[0] >= 3){
				tempvalue->buf_number[0] = 0;
			}
			/* selects the fru_id for specific MCH components (MCMC,CU,PM)*/
			switch (tempvalue->buf_number[0]){
			case 0:
				tempvalue->fru_id_sensors[MCH_SENSORS] = MCMC_FRU_ID;
				fru_id= MCMC_FRU_ID;
				break;
			case 1:
				tempvalue->fru_id_sensors[MCH_SENSORS] = CU_FRU_ID;
				fru_id = CU_FRU_ID;
				break;
			case 2:
				tempvalue->fru_id_sensors[MCH_SENSORS] = PM_FRU_ID;
				fru_id = PM_FRU_ID; //PM_FRU_ID
				break;
			default:
				break;
			}
			tempvalue->buf_number[0] ++;
		}
		/* checks if the number of sensors is bigger then available sensors. if then reset sensor_number to 0*/
		if (tempvalue->sensor_number[line_select]  > fru->sensorNum-1){
			tempvalue->sensor_number[line_select] = 0;
		}
		/* gets new fru */
		fru = get_ipmiDB(fru_id);
		
		/* gets new sensor data */
		sensorPtr = (unsigned char*)&(fru->sensor[tempvalue->sensor_number[line_select]]);
		/* gets new sensor type*/
		type = sensorPtr[3];
	}
	return;
}

/**
 * \brief Checks if the sensor value is near or above the Threshold value
 *
 * \param state_buf      This is where the actually unit is saved
 * \param sensorReading  If a sensor is present or not
 * \param sensor_id      AMC or eMCH sensor is used
 * \return               none
 *
 */
uint8_t check_threshold ( char state_buf, uint8_t sensorReading, uint8_t sensor_id)
{
	int threshold_shift = 0;
	state_buf &= ~0xC0;
	if (sensorReading == 1){
		if ((state_buf & 0x20) == 0x20){
			threshold_shift = 1;
			tempvalue->value_threshold[sensor_id] = THRESHOLD_HIGH;
		} else if((state_buf & 0x10) == 0x10){
			threshold_shift = 1;
			tempvalue->value_threshold[sensor_id] = THRESHOLD_HIGH;
		} else if((state_buf & 0x08) == 0x08){
			threshold_shift = 1;
			tempvalue->value_threshold[sensor_id] = THRESHOLD_HIGH;
		} else if((state_buf & 0x04) == 0x04){
			threshold_shift = 1;
			tempvalue->value_threshold[sensor_id] = THRESHOLD_LOW;
		} else if((state_buf & 0x02) == 0x02){
			threshold_shift = 1;
			tempvalue->value_threshold[sensor_id] = THRESHOLD_LOW;
		} else if((state_buf & 0x01) == 0x01){
			threshold_shift = 1;
			tempvalue->value_threshold[sensor_id] = THRESHOLD_LOW;
		} else{
			threshold_shift = 0;
		}
	}
	return(threshold_shift);
}

/**
 * \brief Generates the units for the LCD display
 *
 * \param sensor_id  AMC or eMCH sensor is used 
 * \param sensorPtr  the sensor data taken from FRU
 * \return           none
 *
 */

void generate_units(unsigned char *sensorPtr ,uint8_t sensor_id) 
{
	memset(tempvalue->lcd_sensor_unit[sensor_id],0,UNIT_LENGTH);
	unsigned char units = sensorPtr[21];
	switch (units){
		case 1:	tempvalue->lcd_sensor_unit[sensor_id][0] ='C';
		break;
		case 2:	tempvalue->lcd_sensor_unit[sensor_id][0] ='V';
		break;
		case 3:	tempvalue->lcd_sensor_unit[sensor_id][0] ='K';
		break;
		case 4:	tempvalue->lcd_sensor_unit[sensor_id][0] ='V';
		break;
		case 5:	tempvalue->lcd_sensor_unit[sensor_id][0] ='A';
		break;
		case 6:	tempvalue->lcd_sensor_unit[sensor_id][0] ='W';
		break;
		case 18:tempvalue->lcd_sensor_unit[sensor_id][0] ='R';tempvalue->lcd_sensor_unit[sensor_id][1] ='P';tempvalue->lcd_sensor_unit[sensor_id][2] ='M';
		break;
		case 19:tempvalue->lcd_sensor_unit[sensor_id][0] ='H';tempvalue->lcd_sensor_unit[sensor_id][1] ='Z';
		break;
		default:
		tempvalue->lcd_sensor_unit[sensor_id][0] ='X';
		break;
	}
}

/**
 * \brief sets the LCD display position
 *
 * \param x   the x positon on the display
 * \param y   the y positon on the display
 * \return    none
 *
 */
void lcd_setpos(char x, char y){
	uint8_t posline = 0;
	/* x defines row position and y defines column position of LCD display*/
	switch(y){
		case 0:
		posline = POSITION_LINE_ONE;
		break;
		
		case 1:
		posline = POSITION_LINE_TWO;
		break;
		
		case 2:
		posline = POSITION_LINE_THREE;
		break;
		
		case 3:
		posline = POSITION_LINE_FOUR;
		break;
		
		default:
		posline = POSITION_LINE_ONE;
		break;
	}
	lcd_sendcommand(LCD_HOME_L1+posline + (x-1),1);
}

/**
 * \brief increments the sensor number and if its reaches end resets to 0
 *
 * \param  sensor_id  AMC or eMCH sensor is used
 * \return none
 *
 */
void next_sensor(uint8_t sensor_id){
	
	switch(sensor_id){
	case 1:
		auto_change_flag = 1;
		tempvalue->sensor_number[1] = tempvalue->sensor_number[1] + 1;
		if(tempvalue->sensor_number[1] >= 3) {
			tempvalue->sensor_number[1] = 0;
		}
		break;
	case 2:
		tempvalue->sensor_number[2] = tempvalue->sensor_number[2] + 1;
		tempvalue->lcd_refresh[MCH_SENSORS] = 1;
		break;
	case 3:
		tempvalue->sensor_number[3] = tempvalue->sensor_number[3] + 1;
		tempvalue->lcd_refresh[AMC_SENSORS] = 1;
		break;
	default:
		break;
	}
}

/**
 * \brief initialize of LCD display
 *
 * \param  none
 * \return none
 *
 */
void lcd_sendinit ()
{
	lcd_sendcommand(0x3A,1); //8-Bit data length extension Bit RE=1; REV=0
	printd(".");
	lcd_sendcommand(0x09,1); //4 line display
	printd(".");
	lcd_sendcommand(0x06,1); //Bottom view
	printd(".");
	lcd_sendcommand(0x1E,1); //Bias setting BS1=1
	printd(".");
	lcd_sendcommand(0x39,1); //8-Bit data length extension Bit RE=0; IS=1
	printd(".");
	lcd_sendcommand(0x1B,1); //BS0=1 -> Bias=1/6
	printd(".");
	lcd_sendcommand(0x6E,1); //Divider on and set value
	printd(".");
	lcd_sendcommand(0x57,1); //Booster on and set contrast (BB1=C5, DB0=C4)
	printd(".");
	lcd_sendcommand(0x72,1); //Set contrast (DB3-DB0=C3-C0)
	printd(".");
	lcd_sendcommand(0x38,1); //8-Bit data length extension Bit RE=0; IS=0
	printd(".");
	lcd_sendcommand(0x0C,1); //Display on, cursor on, blink on
	printd(".");
	/* this clears the display of any information and resets its position*/
	lcd_sendcommand(0x01,1);
	lcd_setpos(1,1);
	/* this sets the ROM for the display since different HEX values show different symbols on the LCD display */
	lcd_setrom(ROMB);
	/* this sets the NAT logo on the lines and shows for 1000ms*/
	strncpy(lineset[LCD_DISPLAY_LINE_ONE],  "   _  _   _ _____       ",MAX_SYMBOL_LENGHT);
	strncpy(lineset[LCD_DISPLAY_LINE_TWO],  "  | \\| | /_\\_   _|       ",MAX_SYMBOL_LENGHT);
	strncpy(lineset[LCD_DISPLAY_LINE_THREE],"  | .` |/ _ \\| |       ",MAX_SYMBOL_LENGHT);
	strncpy(lineset[LCD_DISPLAY_LINE_FOUR], "  |_|\\_/_/ \\_\\_|        ",MAX_SYMBOL_LENGHT);
	TSLEEP(20);
	for (uint8_t z = 0 ; z<4;z++){
		for (uint8_t i=0; i < sizeof(lineset[0]); i++){
			lcd_setpos(i+1,z);
			lcd_sendcommand(lineset[z][i],0);
		}
	}
	TSLEEP(1000);
// 	memset(lineset[LCD_DISPLAY_LINE_ONE], ' ', MAX_SYMBOL_LENGHT);
// 	memset(lineset[LCD_DISPLAY_LINE_TWO], ' ', MAX_SYMBOL_LENGHT);
// 	memset(lineset[LCD_DISPLAY_LINE_THREE], ' ', MAX_SYMBOL_LENGHT);
// 	memset(lineset[LCD_DISPLAY_LINE_FOUR], ' ', MAX_SYMBOL_LENGHT);
}

/**
 * \brief Changes the Rom code (ROMA=0x00, ROMB=0x04, ROMC=0x0C)
 *
 * \param   rom  which rom is used
 * \return  none
 *
 */
void lcd_setrom (unsigned char rom)
{
	lcd_sendcommand(0x2A,1);
	lcd_sendcommand(0x72,1);
	lcd_sendcommand(rom,0);
	lcd_sendcommand(0x28,1);
}

/**
 * \brief   refresh selected LCD line
 * \param   line_id  which LCD line needs to be refreshed.
 * \return  none
 *
 */
void lcd_refresh(uint8_t line_id){
	button_flag = 1;
	CFG_SW	*cfg_sw = cfg_get_sw();
	switch (line_id){
	case 1:
		return;
		break;
		
	case 2:
		lcd_refresh_line_two();
		break;
		
	case 3:
		lcd_refresh_line_sensors(MCH_SENSORS);
		break;
		
	case 4:
		if (cfg_sw->emch_flags & CFG_AMC1_UNMANAGED){
			break;
		}
		lcd_refresh_line_sensors(AMC_SENSORS);
		break;
		
	default:
		break;

	}
	while (refresh_flag == 1){
		TSLEEP(1);
	}
	lcd_setpos(1,line_id-1);
	lcd_senddata(line_id-1,0);
	
	button_flag = 0;
}



uint8_t lcd_sendcommand (unsigned char command, uint8_t cd)
{
	uint8_t result = FAIL;
	uint8_t timeout = 0;
	memset((void*)&packet_write, 0x00, sizeof(packet_write));
	uint8_t command_tx[] = {0x00 , command};

	if( cd == 1){
		/* add 0x80 for command instructions */
		command_tx[0] = COMMAND_COMMAND;
	}
	else if( cd == 0){
		/* add 0x40 for to define that the next package is data */
		command_tx[0] = COMMAND_DATA;
	}
	else {
		printd("Error");
		return(-1);
	}
	packet_write.chip = (0x3C);						// TWI slave bus address
	packet_write.buffer = (uint8_t *) command_tx;	// transfer data destination buffer
	packet_write.length = 2;						// transfer data size (bytes)
	result = FAIL;
	while (result != TWI_SUCCESS){
		result = twi1_master_write(&packet_write);
		delay_ms(5);
		/* After a write cycle, wait 5 ms for the cycle to complete */
		if (result == TWI_SUCCESS){
			//	printd(".");
			timeout = 0;
		}
		else{
			printd("#");
			if(timeout++ > 5)
			{
				break;
			}
		}
	}
	if (result != SUCCEED){
		printd("I2C failed, code %d.\n", result);
		return FAIL;
	}
	return 0;
}