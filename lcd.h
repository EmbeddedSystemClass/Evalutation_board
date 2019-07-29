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
* Module      :
*
* Description :
*
* Author      :
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



#ifndef NAMC_DISCOVERY_LCD
#define NAMC_DISCOVERY_LCD

/* initialization of LCD display*/
 void lcd_init(void);
 
 /* this changes to the next sensor to be displayed */
 void next_sensor(uint8_t sensor_id);

 void lcd_refresh(uint8_t line_id);


#define AMC_SENSORS 0
#define MCH_SENSORS 1
#define LCD_DISPLAY_LINE_ONE 0
#define LCD_DISPLAY_LINE_TWO 1
#define LCD_DISPLAY_LINE_THREE 2
#define LCD_DISPLAY_LINE_FOUR 3


/* define commands for LCd display */
#define ROMA        0x00
#define ROMB        0x04
#define ROMC        0x0C
#define POSITION_LINE_ONE 0
#define POSITION_LINE_TWO 0x20
#define POSITION_LINE_THREE 0x40
#define POSITION_LINE_FOUR 0x60
#define COMMAND_COMMAND 0x80
#define COMMAND_DATA 0x40
#define LCD_HOME_L1   0x80
#define LINE1         0x00

/* Define for lengths of data */
#define NAME_LENGTH 12
#define UNIT_LENGTH 3
#define SENSOR_VALUE_LENGTH 4
#define AMC_MAX_NAME_LENGTH 18
#define M_STATE_SYMBOL_LOCATION 18
#define M_STATE_LOCATION 19

/* FRU id defines */
#define MCMC_FRU_ID 3
#define CU_FRU_ID 40
#define PM_FRU_ID 50

/* defines for symbols */
#define THRESHOLD_HIGH 0x22
#define THRESHOLD_LOW  0x23
#define MAX_SYMBOL_LENGHT 20
#define SYMBOL_M 0x4D
#define LINK_UP 0x12
#define LINK_DOWN 0x13

struct LCD_def {
	char lcd_sensor_name[2][12];
	char buff_lcd_sensor_name[2][21];
	char lcd_sensor_value[2][6];
	char lcd_sensor_unit[2][3];
	int sensor_number[4];
	int fru_id_sensors[2];
	size_t offset[2];
	char value_threshold[2];
	int buf_number[2]; // [0] is used in check_threshold and [1] in lcd_refresh_line_two
	int lcd_refresh[2];
	uint8_t amc_name_lenght;
};

#endif /* NAMC_DISCOVERY_LCD */