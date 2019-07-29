/*
 * FreeRTOS+CLI V1.0.1 (C) 2012 Real Time Engineers ltd.
 *
 * FreeRTOS+CLI is an add-on component to FreeRTOS.  It is not, in itself, part
 * of the FreeRTOS kernel.  FreeRTOS+CLI is licensed separately from FreeRTOS,
 * and uses a different license to FreeRTOS.  FreeRTOS+CLI uses a dual license
 * model, information on which is provided below:
 *
 * - Open source licensing -
 * FreeRTOS+CLI is a free download and may be used, modified and distributed
 * without charge provided the user adheres to version two of the GNU General
 * Public license (GPL) and does not remove the copyright notice or this text.
 * The GPL V2 text is available on the gnu.org web site, and on the following
 * URL: http://www.FreeRTOS.org/gpl-2.0.txt
 *
 * - Commercial licensing -
 * Businesses and individuals who wish to incorporate FreeRTOS+CLI into
 * proprietary software for redistribution in any form must first obtain a
 * (very) low cost commercial license - and in-so-doing support the maintenance,
 * support and further development of the FreeRTOS+CLI product.  Commercial
 * licenses can be obtained from http://shop.freertos.org and do not require any
 * source files to be changed.
 *
 * FreeRTOS+CLI is distributed in the hope that it will be useful.  You cannot
 * use FreeRTOS+CLI unless you agree that you use the software 'as is'.
 * FreeRTOS+CLI is provided WITHOUT ANY WARRANTY; without even the implied
 * warranties of NON-INFRINGEMENT, MERCHANTABILITY or FITNESS FOR A PARTICULAR
 * PURPOSE. Real Time Engineers Ltd. disclaims all conditions and terms, be they
 * implied, expressed, or statutory.
 *
 * 1 tab == 4 spaces!
 *
 * http://www.FreeRTOS.org
 * http://www.FreeRTOS.org/FreeRTOS-Plus
 *
 */
/*****************************************************************************/
/*	INCLUDES							     */
/*****************************************************************************/
#include "FreeRTOS.h"
#include "FreeRTOS_CLI.h"
#include "task.h"
#include "88e6131.h"
#include "88e6320.h"
#include "cfg.h"
#include "emch_rel.h"
#include "task_defs.h"
#include "ping.h"
#include "FreeRTOSuser.h"
#include "FreeRTOS_clib.h"
#include "shell.h"
#include "printd.h"
#include "ethernet.h"
#include "conf_board.h"

#ifdef EMCH_FIRMWARE
#include "twi_i2c.h"
#include "ipmi_fruinfo.h"
#include "ipmi_sensor.h"
#include "ipmi.h"
#include "diag.h"
#include "cu.h"
#include "pm.h"
#include "ping.h"
#include "sdr_rep.h"
#include "sel.h"
#include "session.h"
#include "rmcp.h"
#include "cmu.h"
#include "csif.h"
#include "ipmi_msg.h"
#include "lshm.h"
#endif

/* LwIP includes */
#include "lwip/stats.h"
#include "ip_addr.h"
#include "netifapi.h"

/* Standard includes. */
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

/*
 * string operations
 */
#define ISCOLON(c) \
	(((char)(c) == ':') ? 1 : 0)

#define ISSPACE(c) \
	(((char)(c) == ' ') ? 1 : 0)

#define SKIP_WS(s)  { while((*s == ' ') || (*s == '\t')) s++; }

#define SKIP_NWS(s) { while((*s != ' ') && (*s != '\t')) s++; }

/*****************************************************************************/
/*	DEFINES										     */
/*****************************************************************************/
#define set_bootsign()		gpbr_write(GPBR0, 1)
#define clear_bootsign()	gpbr_write(GPBR0, 0)
#define get_bootsign()		gpbr_read(GPBR0)

#define ip2str(ip_ulong)			ipaddr_ntoa((ip_addr_t *)(&ip_ulong))
#define str2ip(ip_string, ip_ulong)	ipaddr_aton(ip_string, (ip_addr_t *)(&ip_ulong))

/*****************************************************************************/
/*	PRIVATE CLI FUNCTION PROTOTYPES										     */
/*****************************************************************************/
static int32_t str2l(int8_t *string, uint8_t slen);

/* Common commands function prototypes */	
static portBASE_TYPE br_cmd(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString);
static portBASE_TYPE ip_cmd(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString);
static portBASE_TYPE ni_cmd(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString);
static portBASE_TYPE reboot_cmd(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString);
static portBASE_TYPE password_cmd(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString);
static portBASE_TYPE ping_cmd(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString);
static portBASE_TYPE ti_cmd(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString);
static portBASE_TYPE version_cmd(int8_t *pcWriteBuffer,size_t xWriteBufferLen, const int8_t *pcCommandString);
static portBASE_TYPE dummy_cmd(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString);

#ifdef EMCH_FIRMWARE
/* Application only commands function prototypes */	
static portBASE_TYPE bl_cmd(int8_t *pcWriteBuffer,size_t xWriteBufferLen, const int8_t *pcCommandString);
static portBASE_TYPE clrf_cmd(int8_t *pcWriteBuffer,size_t xWriteBufferLen, const int8_t *pcCommandString);
static portBASE_TYPE cfg_cmd(int8_t *pcWriteBuffer,size_t xWriteBufferLen, const int8_t *pcCommandString);
static portBASE_TYPE dbg_cmd(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString);
static portBASE_TYPE demo_mode_cmd(int8_t *pcWriteBuffer,size_t xWriteBufferLen, const int8_t *pcCommandString);
static portBASE_TYPE diag_cmd(int8_t *pcWriteBuffer,size_t xWriteBufferLen, const int8_t *pcCommandString);
static portBASE_TYPE fan_ctl_cmd(int8_t *pcWriteBuffer,size_t xWriteBufferLen, const int8_t *pcCommandString);
static portBASE_TYPE fru_start_cmd(int8_t *pcWriteBuffer,size_t xWriteBufferLen, const int8_t *pcCommandString);
static portBASE_TYPE shutdown_cmd(int8_t *pcWriteBuffer,size_t xWriteBufferLen, const int8_t *pcCommandString);

/* Info commands */
static portBASE_TYPE sdrrep_info_cmd(int8_t *pcWriteBuffer,size_t xWriteBufferLen, const int8_t *pcCommandString);
static portBASE_TYPE imsg_info_cmd(int8_t *pcWriteBuffer,size_t xWriteBufferLen, const int8_t *pcCommandString);
static portBASE_TYPE idb_info_cmd(int8_t *pcWriteBuffer,size_t xWriteBufferLen, const int8_t *pcCommandString);
static portBASE_TYPE lshm_info_cmd(int8_t *pcWriteBuffer,size_t xWriteBufferLen, const int8_t *pcCommandString);
static portBASE_TYPE sel_info_cmd(int8_t *pcWriteBuffer,size_t xWriteBufferLen, const int8_t *pcCommandString);
static portBASE_TYPE rmcp_info_cmd(int8_t *pcWriteBuffer,size_t xWriteBufferLen, const int8_t *pcCommandString);
static portBASE_TYPE session_info_cmd(int8_t *pcWriteBuffer,size_t xWriteBufferLen, const int8_t *pcCommandString);

/* Print and show commands */
static portBASE_TYPE show_cu_cmd(int8_t *pcWriteBuffer,size_t xWriteBufferLen, const int8_t *pcCommandString);
static portBASE_TYPE show_fru_cmd(int8_t *pcWriteBuffer,size_t xWriteBufferLen, const int8_t *pcCommandString);
static portBASE_TYPE show_fruinfo_cmd(int8_t *pcWriteBuffer,size_t xWriteBufferLen, const int8_t *pcCommandString);
static portBASE_TYPE show_sensorinfo_cmd(int8_t *pcWriteBuffer,size_t xWriteBufferLen, const int8_t *pcCommandString);
static portBASE_TYPE show_pm_cmd(int8_t *pcWriteBuffer,size_t xWriteBufferLen, const int8_t *pcCommandString);
static portBASE_TYPE show_pwrconf_cmd(int8_t *pcWriteBuffer,size_t xWriteBufferLen, const int8_t *pcCommandString);
static portBASE_TYPE show_ekey_cmd(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString);
static portBASE_TYPE show_link_state_cmd(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString);

/* Detailed debug commands */
static portBASE_TYPE imsg_dbg_cmd(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString);
static portBASE_TYPE lshm_dbg_cmd(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString);
static portBASE_TYPE rmcp_dbg_cmd(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString);
static portBASE_TYPE sdrrep_dbg_cmd(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString);
static portBASE_TYPE sel_dbg_cmd(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString);
static portBASE_TYPE session_dbg_cmd(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString);
static portBASE_TYPE csif_dbg_cmd(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString);
static portBASE_TYPE cmu_dbg_cmd(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString);
static portBASE_TYPE ipmi_dbg_cmd(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString);

#endif	

/*****************************************************************************/
/*	CLI COMMAND DEFINITIONS 									     */
/*****************************************************************************/
/* Common commands definitions */	

/* Command seperators, just for formatting  output generated by ? command */
static const CLI_Command_Definition_t app_command_seperator =
{
	(const int8_t *const) "#13374#", /* The command string to type. */
	(const int8_t *const) "=== Application Commands: ===\n",
	dummy_cmd, /* The function to run. */
	0 /* No parameters are expected. */
};

static const CLI_Command_Definition_t cfg_command_seperator =
{
	(const int8_t *const) "#13374#", /* The command string to type. */
	(const int8_t *const) "=== Configuration Commands: ===\n",
	NULL, /* The function to run. */
	0 /* No parameters are expected. */
};

static const CLI_Command_Definition_t print_command_seperator =
{
	(const int8_t *const) "#13374#", /* The command string to type. */
	(const int8_t *const) "=== Print and Show Commands: ===\n",
	NULL, /* The function to run. */
	0 /* No parameters are expected. */
};

static const CLI_Command_Definition_t network_command_seperator =
{
	(const int8_t *const) "#13374#", /* The command string to type. */
	(const int8_t *const) "=== Network Support Commands: ===\n",
	NULL, /* The function to run. */
	0 /* No parameters are expected. */
};

static const CLI_Command_Definition_t diag_command_seperator =
{
	(const int8_t *const) "#13374#", /* The command string to type. */
	(const int8_t *const) "=== Diag Support Commands: ===\n",
	NULL, /* The function to run. */
	0 /* No parameters are expected. */
};

static const CLI_Command_Definition_t debug_command_seperator =
{
	(const int8_t *const) "#13374#", /* The command string to type. */
	(const int8_t *const) "=== Debug Support Commands: ===\n",
	NULL, /* The function to run. */
	0 /* No parameters are expected. */
};

static const CLI_Command_Definition_t br_cmd_definition =
{
	(const int8_t *const) "br", /* The command string to type. */
	(const int8_t *const) "  br                        - set device serial baudrate\n",
	br_cmd, /* The function to run. */
	1 /* No parameters are expected. */
};

static const CLI_Command_Definition_t ip_cmd_definition =
{
	(const int8_t *const) "ip",
	(const int8_t *const) "  ip                        - set interface config parameters\n",
	ip_cmd, /* The function to run. */
	0 /* Three parameters are expected, which can take any value. */
};

static const CLI_Command_Definition_t ni_cmd_definition =
{
	(const int8_t *const) "ni", /* The command string to type. */
	(const int8_t *const) "  ni                        - print network configuration\n",
	ni_cmd, /* The function to run. */
	0 /* No parameters are expected. */
};

static const CLI_Command_Definition_t reboot_cmd_definition =
{
	(const int8_t *const) "reboot",
	(const int8_t *const) "  reboot                    - perform a system reboot\n",
	reboot_cmd, /* The function to run. */
	0 /* No parameters are expected. */
};

static const CLI_Command_Definition_t password_cmd_definition =
{
	(const int8_t *const) "password", /* The command string to type. */
	(const int8_t *const) "  password                  - change the device password\n",
	password_cmd, /* The function to run. */
	0 /* One parameters is expected. */
};

static const CLI_Command_Definition_t ping_cmd_definition =
{
	(const int8_t *const) "ping", /* The command string to type. */
	(const int8_t *const) "  ping <host>               - send ICMP echo requests to host\n",
	ping_cmd, /* The function to run. */
	-1 /* Variable number of parameters is expected. */
};

static const CLI_Command_Definition_t ti_cmd_definition =
{
	(const int8_t *const) "ti", /* The command string to type. */
	(const int8_t *const) "  ti                        - show task manager and memory usage\n",
	ti_cmd, /* The function to run. */
	0 /* No parameters are expected. */
};

static const CLI_Command_Definition_t version_cmd_definition =
{
	(const int8_t *const) "version", /* The command string to type. */
	(const int8_t *const) "  version                   - print firmware version information\n",
	version_cmd, /* The function to run. */
	0 /* No parameters are expected. */
};

/* Application only commands definitions */	
#ifdef EMCH_FIRMWARE
static const CLI_Command_Definition_t bl_cmd_definition =
{
	(const int8_t *const) "bl", /* The command string to type. */
	(const int8_t *const) "  bl                        - boot into bootloader\n",
	bl_cmd, /* The function to run. */
	0 /* No parameters are expected. */
};

static const CLI_Command_Definition_t clrf_cmd_definition =
{
	(const int8_t *const) "clrf", /* The command string to type. */
	(const int8_t *const) "  clrf                      - erases BP-FRU contents\n",
	clrf_cmd, /* The function to run. */
	0 /* No parameters are expected. */
};

static const CLI_Command_Definition_t cfg_cmd_definition =
{
	(const int8_t *const) "cfg", /* The command string to type. */
	(const int8_t *const) "  cfg                       - set / show config\n",
	cfg_cmd, /* The function to run. */
	0 /* No parameters are expected. */
};

static const CLI_Command_Definition_t dbg_cmd_definition =
{
	(const int8_t *const) "dbg", /* The command string to type. */
	(const int8_t *const) "  dbg                       - see <dbg ?>\n",
	dbg_cmd, /* The function to run. */
	-1 /* No parameters are expected. */
};

static const CLI_Command_Definition_t demo_mode_cmd_definition =
{
	(const int8_t *const) "demo", /* The command string to type. */
	(const int8_t *const) "  demo                      - simulate MTCA state machine\n",
	demo_mode_cmd, /* The function to run. */
	0 /* No parameters are expected. */
};

static const CLI_Command_Definition_t diag_cmd_definition =
{
	(const int8_t *const) "diag", /* The command string to type. */
	(const int8_t *const) "  diag                      - perform a hardware unit test\n",
	diag_cmd, /* The function to run. */
	0 /* No parameters are expected. */
};

static const CLI_Command_Definition_t fan_ctl_cmd_definition =
{
	(const int8_t *const) "fan_ctl", /* The command string to type. */
	(const int8_t *const) "  fan_ctl                   - control the FAN speed manually\n",
	fan_ctl_cmd, /* The function to run. */
	0 /* No parameters are expected. */
};

static const CLI_Command_Definition_t fru_start_cmd_definition =
{
	(const int8_t *const) "fru_start", /* The command string to type. */
	(const int8_t *const) "  fru_start <fruId>         - set FRU device to M4 state manually\n",
	fru_start_cmd, /* The function to run. */
	1 /* No parameters are expected. */
};

static const CLI_Command_Definition_t shutdown_cmd_definition =
{
	(const int8_t *const) "shutdown", /* The command string to type. */
	(const int8_t *const) "  shutdown <fruId>          - set FRU device to M1 state manually\n",
	shutdown_cmd, /* The function to run. */
	1 /* No parameters are expected. */
};

static const CLI_Command_Definition_t sel_info_cmd_definition =
{
	(const int8_t *const) "sel_info", /* The command string to type. */
	(const int8_t *const) "  sel_info                  - print SEL information\n",
	sel_info_cmd, /* The function to run. */
	0 /* No parameters are expected. */
};

static const CLI_Command_Definition_t rmcp_info_cmd_definition =
{
	(const int8_t *const) "rmcp_info", /* The command string to type. */
	(const int8_t *const) "  rmcp_info                 - print RMCP interface information\n",
	rmcp_info_cmd, /* The function to run. */
	0 /* No parameters are expected. */
};

static const CLI_Command_Definition_t session_info_cmd_definition =
{
	(const int8_t *const) "session_info", /* The command string to type. */
	(const int8_t *const) "  session_info              - print RMCP interface information\n",
	session_info_cmd, /* The function to run. */
	0 /* No parameters are expected. */
};

static const CLI_Command_Definition_t sdrrep_info_cmd_definition =
{
	(const int8_t *const) "sdrrep_info", /* The command string to type. */
	(const int8_t *const) "  sdrrep_info               - print SDR repository information\n",
	sdrrep_info_cmd, /* The function to run. */
	0 /* No parameters are expected. */
};

static const CLI_Command_Definition_t imsg_info_cmd_definition =
{
	(const int8_t *const) "imsg_info", /* The command string to type. */
	(const int8_t *const) "  imsg_info                 - print IPMI MSG information\n",
	imsg_info_cmd, /* The function to run. */
	0 /* No parameters are expected. */
};

static const CLI_Command_Definition_t idb_info_cmd_definition =
{
	(const int8_t *const) "idb_info", /* The command string to type. */
	(const int8_t *const) "  idb_info                  - print IPMI data base information\n",
	idb_info_cmd, /* The function to run. */
	0 /* No parameters are expected. */
};

static const CLI_Command_Definition_t lshm_info_cmd_definition =
{
	(const int8_t *const) "lshm_info", /* The command string to type. */
	(const int8_t *const) "  lshm_info                 - print local ShM information\n",
	lshm_info_cmd, /* The function to run. */
	0 /* No parameters are expected. */
};

static const CLI_Command_Definition_t show_ekey_cmd_definition =
{
	(const int8_t *const) "show_ekey", /* The command string to type. */
	(const int8_t *const) "  show_ekey                 -  show all activated connections\n",
	show_ekey_cmd, /* The function to run. */
	0 /* No parameters are expected. */
};

static const CLI_Command_Definition_t show_cu_cmd_definition =
{
	(const int8_t *const) "show_cu", /* The command string to type. */
	(const int8_t *const) "  show_cu                   - show Cooling Unit Status\n",
	show_cu_cmd, /* The function to run. */
	0 /* No parameters are expected. */
};

static const CLI_Command_Definition_t show_fru_cmd_definition =
{
	(const int8_t *const) "show_fru", /* The command string to type. */
	(const int8_t *const) "  show_fru                  - show all FRUs\n",
	show_fru_cmd, /* The function to run. */
	0 /* No parameters are expected. */
};

static const CLI_Command_Definition_t show_fruinfo_cmd_definition =
{
	(const int8_t *const) "show_fruinfo", /* The command string to type. */
	(const int8_t *const) "  show_fruinfo <fruid>      - show FRU contents\n",
	show_fruinfo_cmd, /* The function to run. */
	1 /* 1 parameters are expected. */
};

static const CLI_Command_Definition_t show_sensorinfo_cmd_definition =
{
	(const int8_t *const) "show_sensorinfo", /* The command string to type. */
	(const int8_t *const) "  show_sensorinfo <fru_id>  - show sensors for FRU\n",
	show_sensorinfo_cmd, /* The function to run. */
	1 /* 1 parameters are expected. */
};

static const CLI_Command_Definition_t show_link_state_cmd_definition =
{
	(const int8_t *const) "show_link_state", /* The command string to type. */
	(const int8_t *const) "  show_link_state           - show ethernet link state\n",
	show_link_state_cmd, /* The function to run. */
	0 /* 1 parameters are expected. */
};
static const CLI_Command_Definition_t show_pm_cmd_definition =
{
	(const int8_t *const) "show_pm", /* The command string to type. */
	(const int8_t *const) "  show_pm                   - show Power Module Status\n",
	show_pm_cmd, /* The function to run. */
	0 /* No parameters are expected. */
};

static const CLI_Command_Definition_t show_pwrconf_cmd_definition =
{
	(const int8_t *const) "show_pwrconf", /* The command string to type. */
	(const int8_t *const) "  show_pwrconf              - show Backplane Power Configuration\n",
	show_pwrconf_cmd, /* The function to run. */
	0 /* No parameters are expected. */
};

/* Debug command definitions */
static const CLI_Command_Definition_t cmu_dbg_cmd_definition =
{
	(const int8_t *const) "cmu_dbg", /* The command string to type. */
	(const int8_t *const) "  cmu_dbg                   - configure CM upper part debug\n",
	cmu_dbg_cmd, /* The function to run. */
	0 /* No parameters are expected. */
};

static const CLI_Command_Definition_t csif_dbg_cmd_definition =
{
	(const int8_t *const) "csif_dbg", /* The command string to type. */
	(const int8_t *const) "  csif_dbg                  - configure CM/ShM interface debug\n",
	csif_dbg_cmd, /* The function to run. */
	0 /* No parameters are expected. */
};

static const CLI_Command_Definition_t lshm_dbg_cmd_definition =
{
	(const int8_t *const) "lshm_dbg", /* The command string to type. */
	(const int8_t *const) "  lshm_dbg                  - configure local ShM debug\n",
	lshm_dbg_cmd, /* The function to run. */
	0 /* No parameters are expected. */
};

static const CLI_Command_Definition_t sel_dbg_cmd_definition =
{
	(const int8_t *const) "sel_dbg", /* The command string to type. */
	(const int8_t *const) "  sel_dbg                   - configure SEL debug\n",
	sel_dbg_cmd, /* The function to run. */
	0 /* No parameters are expected. */
};

static const CLI_Command_Definition_t sdrrep_dbg_cmd_definition =
{
	(const int8_t *const) "sdrrep_dbg", /* The command string to type. */
	(const int8_t *const) "  sdrrep_dbg                - configure SDR repository debug\n",
	sdrrep_dbg_cmd, /* The function to run. */
	0 /* No parameters are expected. */
};

static const CLI_Command_Definition_t session_dbg_cmd_definition =
{
	(const int8_t *const) "session_dbg", /* The command string to type. */
	(const int8_t *const) "  session_dbg               - configure session management debug\n",
	session_dbg_cmd, /* The function to run. */
	0 /* No parameters are expected. */
};

static const CLI_Command_Definition_t rmcp_dbg_cmd_definition =
{
	(const int8_t *const) "rmcp_dbg", /* The command string to type. */
	(const int8_t *const) "  rmcp_dbg                  - configure RMCP interface debug\n",
	rmcp_dbg_cmd, /* The function to run. */
	0 /* No parameters are expected. */
};

static const CLI_Command_Definition_t imsg_dbg_cmd_definition =
{
	(const int8_t *const) "imsg_dbg", /* The command string to type. */
	(const int8_t *const) "  imsg_dbg                  - configure IPMI MSG debug (high level/SM)\n",
	imsg_dbg_cmd, /* The function to run. */
	0 /* No parameters are expected. */
};

static const CLI_Command_Definition_t ipmi_dbg_cmd_definition =
{
	(const int8_t *const) "ipmi_dbg", /* The command string to type. */
	(const int8_t *const) "  ipmi_dbg                  - configure IPMI MSG debug (low level/driver)\n",
	ipmi_dbg_cmd, /* The function to run. */
	0 /* No parameters are expected. */
};

#endif


void vRegisterCLICommands(void)
{
	
	#ifdef EMCH_FIRMWARE
	/* application support commands */
	FreeRTOS_CLIRegisterCommand(&app_command_seperator);
	FreeRTOS_CLIRegisterCommand(&reboot_cmd_definition);
	FreeRTOS_CLIRegisterCommand(&bl_cmd_definition);
	FreeRTOS_CLIRegisterCommand(&fru_start_cmd_definition);
	FreeRTOS_CLIRegisterCommand(&shutdown_cmd_definition);
	
	/* debug support commands */
	FreeRTOS_CLIRegisterCommand(&debug_command_seperator);
	FreeRTOS_CLIRegisterCommand(&dbg_cmd_definition);
	FreeRTOS_CLIRegisterCommand(&imsg_dbg_cmd_definition);
	FreeRTOS_CLIRegisterCommand(&ipmi_dbg_cmd_definition);
	FreeRTOS_CLIRegisterCommand(&rmcp_dbg_cmd_definition);
	FreeRTOS_CLIRegisterCommand(&cmu_dbg_cmd_definition);
	FreeRTOS_CLIRegisterCommand(&lshm_dbg_cmd_definition);
	FreeRTOS_CLIRegisterCommand(&csif_dbg_cmd_definition);
	FreeRTOS_CLIRegisterCommand(&sdrrep_dbg_cmd_definition);
	FreeRTOS_CLIRegisterCommand(&sel_dbg_cmd_definition);
	FreeRTOS_CLIRegisterCommand(&session_dbg_cmd_definition);
	
	/* network support commands */
	FreeRTOS_CLIRegisterCommand(&network_command_seperator);
	FreeRTOS_CLIRegisterCommand(&ip_cmd_definition);
	FreeRTOS_CLIRegisterCommand(&ping_cmd_definition);
	FreeRTOS_CLIRegisterCommand(&ni_cmd_definition);
	
	/* print support commands */
	FreeRTOS_CLIRegisterCommand(&print_command_seperator);
	FreeRTOS_CLIRegisterCommand(&version_cmd_definition);
	FreeRTOS_CLIRegisterCommand(&sdrrep_info_cmd_definition);
	FreeRTOS_CLIRegisterCommand(&sel_info_cmd_definition);
	FreeRTOS_CLIRegisterCommand(&session_info_cmd_definition);
	FreeRTOS_CLIRegisterCommand(&rmcp_info_cmd_definition);
	FreeRTOS_CLIRegisterCommand(&lshm_info_cmd_definition);
	FreeRTOS_CLIRegisterCommand(&imsg_info_cmd_definition);
	FreeRTOS_CLIRegisterCommand(&idb_info_cmd_definition);
	FreeRTOS_CLIRegisterCommand(&show_cu_cmd_definition);
	FreeRTOS_CLIRegisterCommand(&show_fru_cmd_definition);
	FreeRTOS_CLIRegisterCommand(&show_fruinfo_cmd_definition);
	FreeRTOS_CLIRegisterCommand(&show_sensorinfo_cmd_definition);
	FreeRTOS_CLIRegisterCommand(&show_pm_cmd_definition);
	FreeRTOS_CLIRegisterCommand(&show_pwrconf_cmd_definition);
	FreeRTOS_CLIRegisterCommand(&show_ekey_cmd_definition);
	FreeRTOS_CLIRegisterCommand(&show_link_state_cmd_definition);

	/* cfg support commands */
	FreeRTOS_CLIRegisterCommand(&cfg_command_seperator);
	FreeRTOS_CLIRegisterCommand(&cfg_cmd_definition);
	FreeRTOS_CLIRegisterCommand(&password_cmd_definition);
	
	/* diag support commands */
	FreeRTOS_CLIRegisterCommand(&diag_command_seperator);
	FreeRTOS_CLIRegisterCommand(&ti_cmd_definition);
	FreeRTOS_CLIRegisterCommand(&diag_cmd_definition);
	FreeRTOS_CLIRegisterCommand(&fan_ctl_cmd_definition);
	FreeRTOS_CLIRegisterCommand(&clrf_cmd_definition);
	FreeRTOS_CLIRegisterCommand(&demo_mode_cmd_definition);
	
	/* Bootloader only commands */
	#else
	/* application support commands */
	FreeRTOS_CLIRegisterCommand(&app_command_seperator);
	FreeRTOS_CLIRegisterCommand(&reboot_cmd_definition);
	
	/* network support commands */
	FreeRTOS_CLIRegisterCommand(&network_command_seperator);
	FreeRTOS_CLIRegisterCommand(&ip_cmd_definition);
	FreeRTOS_CLIRegisterCommand(&ping_cmd_definition);

	/* print support commands */
	FreeRTOS_CLIRegisterCommand(&print_command_seperator);
	FreeRTOS_CLIRegisterCommand(&ni_cmd_definition);
	FreeRTOS_CLIRegisterCommand(&version_cmd_definition);
	
	/* cfg support commands */
	FreeRTOS_CLIRegisterCommand(&cfg_command_seperator);
	FreeRTOS_CLIRegisterCommand(&password_cmd_definition);
	
	/* diag support commands */
	FreeRTOS_CLIRegisterCommand(&diag_command_seperator);
	FreeRTOS_CLIRegisterCommand(&ti_cmd_definition);
	#endif
}

/*****************************************************************************/
/*	CLI FUNCTION IMPLEMENTATIONS										     */
/*****************************************************************************/
/* Common commands function implementations */	

static portBASE_TYPE dummy_cmd(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString)
{
	return pdFALSE;
}

static portBASE_TYPE br_cmd(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString)
{
	int8_t *pcParam1;
	portBASE_TYPE xParam1Length;
	
	/* Get all expected parameters */
	pcParam1 = (int8_t*)FreeRTOS_CLIGetParameter(pcCommandString, 1, &xParam1Length);

	/* Terminate patameter strings */
	pcParam1[xParam1Length] = '\0';
	
	/* There is no more data to return after this single string, so return pdFALSE. */
	return pdFALSE;
}

static portBASE_TYPE ip_cmd(int8_t *pcWriteBuffer,size_t xWriteBufferLen,const int8_t *pcCommandString)
{
	/* Global lwip structure holding the network configuration */
	struct netif *p_netif = ethernet_get_netif();
	
	CFG_BASE *cfg_base_p = cfg_base_read();
	char *in_str;
	long ip, nm, gw, bc;
	
	if (cfg_base_p->dhcp_enable == 1)
		printd("WARN - DHCP client enabled\n");
	
	printd("IP Configuration Setup:\n");
	printd("------------------------\n");
	
	in_str = user_str("IP Address: ", ip2str(p_netif->ip_addr.addr));
	str2ip(in_str, ip);

	in_str = user_str("Netmask:    ", ip2str(p_netif->netmask.addr));
	str2ip(in_str, nm);
	
	in_str = user_str("Gateway:    ", ip2str(p_netif->gw.addr));
	str2ip(in_str, gw);
	
	/* Prepare broad cast address to xxx.xxx.xxx.255 */
	bc = ip | 0xff000000;
	
	in_str = user_str("Broadcast:  ", ip2str(bc));
	str2ip(in_str, bc);
	
	/* Apply config to base config structure */
	cfg_base_p->netif_ip = ip;
	cfg_base_p->netif_nm = nm;
	cfg_base_p->netif_gw = gw;
	cfg_base_p->netif_bc = bc;
		
	/* Apply settings to LWIP */
	netifapi_netif_set_addr(p_netif,(ip_addr_t *)(&ip), (ip_addr_t *)(&nm), (ip_addr_t *)(&gw));
	
	/* Save configuration to flash */
	int mode = user_par("Save config to FLASH? (y/n)", ('y' | CHAR_IDENT));
	
	if (mode == (CHAR_IDENT | 'y'))
	{
		cfg_base_write();
		printd("Configuration saved.\n");
	}
	
	else
	{
		printd("Configuration not saved.\n");
	}
	
	return pdFALSE;
}

static portBASE_TYPE ni_cmd(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString)
{
	CFG_HW *cfg_hw_p = cfg_hw_read();
	CFG_BASE *cfg_base_p = cfg_base_read();
	struct netif *p_netif = ethernet_get_netif();
		
	printd("\n**** Current Network configuration: ****\n");
	printd("IEEE address:\t%02X:%02X:%02X:%02X:%02X:%02X\n",	
		cfg_hw_p->ieee_addr[0], cfg_hw_p->ieee_addr[1],
		cfg_hw_p->ieee_addr[2], cfg_hw_p->ieee_addr[3],
		cfg_hw_p->ieee_addr[4], cfg_hw_p->ieee_addr[5]);
	printd("IP address: %s\n", ip2str(p_netif->ip_addr.addr));
	printd("Netmask:    %s\n", ip2str(p_netif->netmask.addr));
	printd("Gateway:    %s\n", ip2str(p_netif->gw.addr));
	
	printd("Web password:\t\%s\n", cfg_base_p->password);
	
#ifndef RELEASE
	stats_display();
#endif
	return pdFALSE;
}

static portBASE_TYPE version_cmd(int8_t *pcWriteBuffer,size_t xWriteBufferLen, const int8_t *pcCommandString)
{
	get_emch_rel(NULL);
	return pdFALSE;
}

static portBASE_TYPE reboot_cmd(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString)
{
#ifdef EMCH_FIRMWARE
	pm_ShutdownFruPower();
#endif
	NVIC_SystemReset();
	return pdFALSE;
}

static portBASE_TYPE password_cmd(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString)
{
	CFG_BASE *cfg_base_p = cfg_base_read();
	char *password = cfg_base_p->password;
	char *in_str;
	char *in_str_confirm;

	printd("Password configuration\n");
	printd("----------------------\n");
	
	in_str = user_str("Current password ", NULL);

	if (!strncmp(in_str,password, strlen(password)))
	{
		in_str = user_str("New password     ", NULL);
		in_str_confirm = user_str("Confirm          ", NULL);
	
		if (!strncmp(in_str,in_str_confirm, strlen(in_str)))
		{
			strcpy(password, in_str);
			cfg_sw_write();
			printd("Done.\n");
		}
		
		else
		{	
			printd("Passwords mismatch.\n");
		}
	}	
	
	else
	{
		printd("Old password mismatch.\n");
	}
	
	return pdFALSE;
}

static portBASE_TYPE ping_cmd(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString)
{
	static int8_t paramNum = 0;
	static int8_t *pcPingCmd = NULL;
	int8_t *pcParam = NULL;
	
	portBASE_TYPE xParamLength = NULL;

	if (paramNum++ == 0)
	{
		/* When calling the interpreter the fist time, the ping_cmd_str must be cleared */
		pcPingCmd = (int8_t*)ping_get_cmd_str();
		memset(pcPingCmd, 0x00, 128);
	}
	
	/* Get all expected parameters */
	pcParam = (int8_t*)FreeRTOS_CLIGetParameter(pcCommandString, paramNum, &xParamLength);

	/* build the ping command string containing all parameters to run ping() */
	if (pcParam != NULL)
	{
		/* Add whitespace character */
		pcParam[xParamLength] = ' ';
		strncat((char*)pcPingCmd, (char*)pcParam, xParamLength+1);
		return pdTRUE;
	}
	
	/* If no more parameter is followed, run ping() */
	else
	{
		/* Terminate pcPingCmd */
		pcPingCmd[strlen((char*)pcPingCmd)-1] = 0x00;
		ping((char*)pcPingCmd);
		paramNum = 0;
	}

	/* There is no more data to return after this single string, so return pdFALSE. */
	return pdFALSE;
}

static portBASE_TYPE ti_cmd(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString)
{
	signed char *buf;
	unsigned int i = 0;
	
	/* Allocate some space (64 bytes per line) for the task print string */
	buf = freertos_mem_alloc(64*uxTaskGetNumberOfTasks());
	
	if (!buf)
		return pdFALSE;
	
	printd("Task          State  Priority  Stack  #\n************************************************\n");
	vTaskList(buf);
		
	/* As the buf is to large to print in one step, print it char by char instead
	to avoid overflow */
	for (i = 0; i<strlen(buf); i++)
		printd("%c", buf[i]);
		
	printd("\nTask          Abs. Time        % Time\n************************************************\n");
	vTaskGetRunTimeStats(buf);
	
	/* As the buf is to large to print in one step, print it char by char instead
	to avoid overflow */
	for (i = 0; i<strlen(buf); i++)
		printd("%c", buf[i]);

	/* The buf is not used anymore */
	freertos_mem_free(buf);
		
	/* Print how much heap memory is currently available */
	printd("\n\nAvailable HEAP memory: %d bytes\n",xPortGetFreeHeapSize());
	
	/* Print system uptime */
	printd("Kernel uptime: %s\n", get_timestamp_str());
	
	return pdFALSE;
	
	
}

/* Application commands function implementations */
#ifdef EMCH_FIRMWARE
static portBASE_TYPE bl_cmd(int8_t *pcWriteBuffer,size_t xWriteBufferLen,const int8_t *pcCommandString)
{
	set_bootsign();
	pm_ShutdownFruPower();
	NVIC_SystemReset();
	
	/* There is no more data to return after this single string, so return pdFALSE. */
	return pdFALSE;
}

static portBASE_TYPE clrf_cmd(int8_t *pcWriteBuffer,size_t xWriteBufferLen, const int8_t *pcCommandString)
{
	twi1_erase_fru(0, 1024*16);
	return pdFALSE;
}

static portBASE_TYPE cfg_cmd(int8_t *pcWriteBuffer,size_t xWriteBufferLen, const int8_t *pcCommandString)
{
	cfg_modify_menu();
	return pdFALSE;
}

static portBASE_TYPE dbg_cmd(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString)
{
	return pdFALSE;
}

static portBASE_TYPE fru_start_cmd(int8_t *pcWriteBuffer,size_t xWriteBufferLen, const int8_t *pcCommandString)
{
	int8_t *pcParam1;
	portBASE_TYPE xParam1Length;
	int32_t num = -1;
	IPMI_DB *fru;
		
	/* Get all expected parameters */
	pcParam1 = (int8_t*)FreeRTOS_CLIGetParameter(pcCommandString, 1, &xParam1Length);
		
	/* Terminate patameter strings */
	pcParam1[xParam1Length] = '\0';
		
	/* Convert string to long integer */
	num = str2l(&pcParam1[0], xParam1Length);

	if (num < 5 || num > 6)
	{
		printd("Invalid FRU id.\n");
		return pdFALSE;
	}
	
	fru = get_ipmiDB(num);
	if (fru == NULL)
	{
		printd("Invalid FRU id.\n");
		return pdFALSE;
	}
	
	if (fru->state < FRU_STATE_M1)
	{
		printd("FRU device not installed.\n");
		return pdFALSE;
	}
	
	printd("Starting FRU(%d)\n", fru->fruId);
	
	fru->MHandle &= ~MHANDLE_OPENED_STATE;
	fru->MHandle |= MHANDLE_CLOSED_STATE;

	return pdFALSE;
}
static portBASE_TYPE shutdown_cmd(int8_t *pcWriteBuffer,size_t xWriteBufferLen, const int8_t *pcCommandString)
{
	int8_t *pcParam1;
	portBASE_TYPE xParam1Length;
	int32_t num = -1;
	IPMI_DB *fru;
	
	/* Get all expected parameters */
	pcParam1 = (int8_t*)FreeRTOS_CLIGetParameter(pcCommandString, 1, &xParam1Length);
	
	/* Terminate patameter strings */
	pcParam1[xParam1Length] = '\0';
	
	if (!(strncmp(pcParam1, "system", 6)))
	{
		printd("- Not supported yet.\n");
		//pm_ShutdownSystemSoft();
		return pdFALSE;
	}
	
	if (!(strncmp(pcParam1, "system_hard", 11)))
	{
		printd("- Not supported yet.\n");
		//pm_ShutdownSystemSoft();
		return pdFALSE;
	}
	
	/* Convert string to long integer */
	num = str2l(&pcParam1[0], xParam1Length);

	if (num < 5 || num > 6)
	{
		printd("Invalid FRU id.\n");
		return pdFALSE;
	}
	
	fru = get_ipmiDB(num);
	if (fru == NULL)
	{
		printd("Invalid FRU id.\n");
		return pdFALSE;
	}
	
	if (fru->state < FRU_STATE_M1)
	{
		printd("FRU device not installed.\n");
		return pdFALSE;
	}
	
	printd("Shutdown FRU(%d)\n", fru->fruId);
	fru->MHandle  &= ~MHANDLE_CLOSED_STATE;
	fru->MHandle |= MHANDLE_OPENED_STATE;
	
	return pdFALSE;
}

static portBASE_TYPE demo_mode_cmd(int8_t *pcWriteBuffer,size_t xWriteBufferLen, const int8_t *pcCommandString)
{
	extern void diag_mmc_demo_task1(void *pvParameter);
	extern void diag_mmc_demo_task2(void *pvParameter);
	
	if (!task_handle.task_demo1)
	{
		TSPAWN("DEMO1", tskIDLE_PRIORITY, configMINIMAL_STACK_SIZE*2, diag_mmc_demo_task1, NULL, NULL);
		printd("Spawned demo task for AMC1\n");
	}
	
	else
	{
		TKILL(task_handle.task_demo1);
		printd("Stopped demo task for AMC1\n");
	}
	
	if (!task_handle.task_demo1)
	{
		TSPAWN("DEMO2", tskIDLE_PRIORITY, configMINIMAL_STACK_SIZE*2, diag_mmc_demo_task2, NULL, NULL);
		printd("Spawned demo task for AMC2\n");
	}
	
	else
	{
		TKILL(task_handle.task_demo2);
		printd("Stopped demo task for AMC2\n");
	}
	
	return pdFALSE;
}

static portBASE_TYPE diag_cmd(int8_t *pcWriteBuffer,size_t xWriteBufferLen, const int8_t *pcCommandString)
{
	/* Create the test task */
	
	if (task_handle.task_diag != NULL)
	{
		printd("Diag task already runnning.\n");
	}
	
	else
	{
		TSPAWN("DIAG", tskIDLE_PRIORITY, configMINIMAL_STACK_SIZE*2, task_diag, &task_handle.task_diag, NULL);
	}
	
	return pdFALSE;
}

static portBASE_TYPE fan_ctl_cmd(int8_t *pcWriteBuffer,size_t xWriteBufferLen, const int8_t *pcCommandString)
{
	extern int diag_fan_ctl(int mode);
	
	static int mode = 4;

	diag_fan_ctl(0);
	mode = user_par("Enter mode", mode);
	diag_fan_ctl(mode);
	
	return pdFALSE;
}

static portBASE_TYPE sdrrep_info_cmd(int8_t *pcWriteBuffer,size_t xWriteBufferLen, const int8_t *pcCommandString)
{
	static int dev = SDR_REP_DEV_CM;
	static int mode = SDR_REP_INFO_MODE_DEC_ALL;
	static u_short rec_id = 0x0001;
	void *par = NULL;
		
	printd("Enter SDR repository device (%d=CM, %d=ShM)",SDR_REP_DEV_CM, SDR_REP_DEV_SHM);
	dev = user_par(NULL, dev);
	SdrRep_Info(dev, SDR_REP_INFO_MODE_HELP, NULL);

	mode = user_par("Enter info mode", mode);
	if((mode == SDR_REP_INFO_MODE_DEC_SINGLE) || (mode == SDR_REP_INFO_MODE_RAW_SINGLE))
	{
		rec_id = user_par("Enter record id", rec_id);
		par = (void *)(u_long)rec_id;
	}
	
	SdrRep_Info(dev, mode, par);
	
	return pdFALSE;
}

static portBASE_TYPE sel_info_cmd(int8_t *pcWriteBuffer,size_t xWriteBufferLen, const int8_t *pcCommandString)
{
	static int dev = SEL_DEV_CM;
	static int mode = SEL_INFO_MODE_DEC_ALL;
		
	printd("Enter SEL device (%d=CM, %d=ShM)",SEL_DEV_CM, SEL_DEV_SHM);
	dev = user_par(NULL, dev);

	Sel_Info(dev, SEL_INFO_MODE_HELP);
	mode = user_par("Enter info mode", mode);
	Sel_Info(dev, mode);
	
	return pdFALSE;
}

static portBASE_TYPE lshm_info_cmd(int8_t *pcWriteBuffer,size_t xWriteBufferLen, const int8_t *pcCommandString)
{
	static int mode = LSHM_INFO_MODE_CU, dev = 0;
	if (dev >= LSHM_DEV_MAX)
		dev = 0;
	Lshm_Info(dev, LSHM_INFO_MODE_HELP);
	mode = user_par("Enter info mode", mode);
	if(mode > LSHM_INFO_MODE_HELP)
		dev = user_par("Enter device", dev);
	Lshm_Info(dev, mode);
	
	return pdFALSE;
}

static portBASE_TYPE idb_info_cmd(int8_t *pcWriteBuffer,size_t xWriteBufferLen, const int8_t *pcCommandString)
{
	static int mode = 1;
	idb_info(0);
	mode = user_par("Enter info mode", mode);
	idb_info(mode);
	return(0);
	
	return pdFALSE;
}

static portBASE_TYPE imsg_info_cmd(int8_t *pcWriteBuffer,size_t xWriteBufferLen, const int8_t *pcCommandString)
{
	static int mode = 1;
	IpmiMsg_Info(0);
	mode = user_par("Enter info mode", mode);
	IpmiMsg_Info(mode);
	return(0);
	
	return pdFALSE;
}

static portBASE_TYPE show_cu_cmd(int8_t *pcWriteBuffer,size_t xWriteBufferLen, const int8_t *pcCommandString)
{
	extern void cu_print_info(void);		/* cu.h */
	
	cu_print_info();
	return pdFALSE;
}

static portBASE_TYPE show_fru_cmd(int8_t *pcWriteBuffer,size_t xWriteBufferLen,const int8_t *pcCommandString)
{
	IpmiFru_PrintAllFru();

	return pdFALSE;
}	

static portBASE_TYPE show_ekey_cmd(int8_t *pcWriteBuffer,size_t xWriteBufferLen,const int8_t *pcCommandString)
{
	extern void ekey_PrintAllLinks(void);		/* ekey.h */
	ekey_PrintAllLinks();
	return pdFALSE;
}


static portBASE_TYPE show_fruinfo_cmd(int8_t *pcWriteBuffer,size_t xWriteBufferLen,const int8_t *pcCommandString)
{
	int8_t *pcParam1;
	portBASE_TYPE xParam1Length;
	int32_t num = -1;
	
	/* Get all expected parameters */
	pcParam1 = (int8_t*)FreeRTOS_CLIGetParameter(pcCommandString, 1, &xParam1Length);
	
	/* Terminate patameter strings */
	pcParam1[xParam1Length] = '\0';
	
	/* Convert string to long integer */
	num = str2l(&pcParam1[0], xParam1Length);

	if (num == -1)
	{
		sprintf((char*)pcWriteBuffer, "Invalid FRU id.\n");
		return pdFALSE;
	}
	
	IpmiFru_PrintRecords((int)num);	

	/* There is no more data to return after this single string, so return pdFALSE. */
	return pdFALSE;
}

static portBASE_TYPE show_sensorinfo_cmd(int8_t *pcWriteBuffer,size_t xWriteBufferLen, const int8_t *pcCommandString)
{
	int8_t *pcParam1;
	portBASE_TYPE xParam1Length;
	int32_t num = -1;
	
	/* Get all expected parameters */
	pcParam1 = (int8_t*)FreeRTOS_CLIGetParameter(pcCommandString, 1, &xParam1Length);
	
	/* Terminate patameter strings */
	pcParam1[xParam1Length] = '\0';
	
	/* Convert string to long integer */
	num = str2l(&pcParam1[0], xParam1Length);
	
	if (num == -1)
	{
		sprintf((char*)pcWriteBuffer, "Invalid FRU id.\n");
		return pdFALSE;
	}

	IpmiSensor_PrintSensor((int)num);
	
	return pdFALSE;
}

static portBASE_TYPE show_pm_cmd(int8_t *pcWriteBuffer,size_t xWriteBufferLen, const int8_t *pcCommandString)
{
	pm_printPM();
	/* There is no more data to return after this single string, so return pdFALSE. */
	return pdFALSE;
}

static portBASE_TYPE show_pwrconf_cmd(int8_t *pcWriteBuffer,size_t xWriteBufferLen, const int8_t *pcCommandString)
{
	pm_printPowerConfig();
	/* There is no more data to return after this single string, so return pdFALSE. */
	return pdFALSE;
}


static portBASE_TYPE show_link_state_cmd(int8_t *pcWriteBuffer,size_t xWriteBufferLen, const int8_t *pcCommandString)
{
	if (cfg_get_board()->type == BOARD_NATIVE_MINI) {
		if (switch_get_link_status(4)) {
			printd("--- AMC1 Port 0: 1000Base-X\n");
		}
		
		if (switch_get_link_status(6)) {
			printd("--- AMC1 Port 1: 1000Base-X\n");
		}

		if (switch_get_link_status(5)) {
			printd("--- AMC2 Port 0: 1000Base-X\n");
		}
		
		if (switch_get_link_status(7)) {
			printd("--- AMC2 Port 1: 1000Base-X\n");
		}
		
		if (switch_get_link_status(0)) {
			printd("--- RJ45 Uplink: Link\n");
		}
	}
	
	if (cfg_get_board()->type == BOARD_NATIVE_R1_MINI) {
		if (switch_get_link_status(4)) {
			printd("--- AMC1 Port 0: 1000Base-X\n");
		}
		
		if (switch_get_link_status(6)) {
			printd("--- AMC2 Port 0: 1000Base-X\n");
		}

		if (switch_get_link_status(5)) {
			printd("--- AMC3 Port 0: 1000Base-X\n");
		}
		
		if (switch_get_link_status(7)) {
			printd("--- AMC4 Port 0: 1000Base-X\n");
		}
		
		if (switch_get_link_status(0)) {
			printd("--- RJ45 Uplink: Link\n");
		}
	}
	
	if (cfg_get_board()->type == BOARD_NAMC_DISCOVERY_V11 || cfg_get_board()->type == BOARD_NAMC_DISCOVERY_V12) {
		if(switch_88e6320_get_link_status(0)) {
			printd("--- AMC1 Port 1: 1000Base-X\n");
		}
		
		if(switch_88e6320_get_link_status(1)) {
			printd("--- AMC1 Port 0: 1000Base-X\n");
		}
		
		if(switch_88e6320_get_link_status(3)) {
			printd("--- RJ45 Uplink (Port 1): Link\n");
		}

		if(switch_88e6320_get_link_status(4)) {
			printd("--- RJ45 Uplink (Port 0): Link\n");
		}
	}

	return pdFALSE;
}

static portBASE_TYPE rmcp_info_cmd(int8_t *pcWriteBuffer,size_t xWriteBufferLen, const int8_t *pcCommandString)
{
	static int mode = 1;
	Rmcp_Info(0);
	mode = user_par("Enter info mode", mode);
	Rmcp_Info(mode);
	return pdFALSE;
}

static portBASE_TYPE session_info_cmd(int8_t *pcWriteBuffer,size_t xWriteBufferLen, const int8_t *pcCommandString)
{
	static int mode = 2;
	Session_Info(0);
	mode = user_par("Enter info mode", mode);
	Session_Info(mode);
	return pdFALSE;
}


/* Detailed debug commands */
static portBASE_TYPE imsg_dbg_cmd(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString)
{
	u_long dbg = IpmiMsg_Dbg(IPMI_MSG_DBG_GET);
	IpmiMsg_Dbg(IPMI_MSG_DBG_SHOW);
	dbg = user_par("Enter debug state", dbg);
	IpmiMsg_Dbg(dbg);
	
	return pdFALSE;
}

static portBASE_TYPE ipmi_dbg_cmd(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString)
{
	u_long dbg = Ipmi_Dbg(IPMI_MSG_DBG_GET);
	Ipmi_Dbg(IPMI_MSG_DBG_SHOW);
	dbg = user_par("Enter debug state", dbg);
	Ipmi_Dbg(dbg);
	
	return pdFALSE;
}

static portBASE_TYPE lshm_dbg_cmd(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString)
{
	u_long dbg = Lshm_Dbg(LSHM_DBG_GET);
	Lshm_Dbg(LSHM_DBG_SHOW);
	dbg = user_par("Enter debug state", dbg);
	Lshm_Dbg(dbg);
	return pdFALSE;
}
static portBASE_TYPE rmcp_dbg_cmd(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString)
{
	u_long dbg = Rmcp_Dbg(RMCP_DBG_GET);
	Rmcp_Dbg(RMCP_DBG_SHOW);
	dbg = user_par("Enter debug state", dbg);
	Rmcp_Dbg(dbg);
	return pdFALSE;
}
static portBASE_TYPE sdrrep_dbg_cmd(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString)
{
	u_long dbg = SdrRep_Dbg(SDR_REP_DBG_GET);
	SdrRep_Dbg(SDR_REP_DBG_SHOW);
	dbg = user_par("Enter debug state", dbg);
	SdrRep_Dbg(dbg);
	return pdFALSE;
}
static portBASE_TYPE sel_dbg_cmd(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString)
{
	u_long dbg = Sel_Dbg(SEL_DBG_GET);
	Sel_Dbg(SEL_DBG_SHOW);
	dbg = user_par("Enter debug state", dbg);
	Sel_Dbg(dbg);
	return pdFALSE;
}
static portBASE_TYPE session_dbg_cmd(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString)
{
	u_long dbg = Session_Dbg(SESS_DBG_GET);
	Session_Dbg(SESS_DBG_SHOW);
	dbg = user_par("Enter debug state", dbg);
	Session_Dbg(dbg);
	return pdFALSE;
}
static portBASE_TYPE csif_dbg_cmd(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString)
{
	u_long dbg = Csif_Dbg(CSIF_DBG_GET);
	Csif_Dbg(CSIF_DBG_SHOW);
	dbg = user_par("Enter debug state", dbg);
	Csif_Dbg(dbg);
	return pdFALSE;
}
static portBASE_TYPE cmu_dbg_cmd(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString)
{
	u_long dbg = Cmu_Dbg(CMU_DBG_GET);
	Cmu_Dbg(CMU_DBG_SHOW);
	dbg = user_par("Enter debug state", dbg);
	Cmu_Dbg(dbg);
	return pdFALSE;
}

#endif
/*-----------------------------------------------------------*/


static int32_t str2l(int8_t *string, uint8_t slen)
{
	int32_t num = -1;
	
	/* Check if string is not too long */
	if (slen > 8)
	return -1;
	
	
	/* Check if string only contains numbers */
	for (int i=0; i<slen; i++)
	{
		num = string[i]-'0';
		if (num < 0 || num > 9)
		return -1;
	}

	num = 0;
	
	/* Calculate number */
	for (int i=0; i<slen; i++)
	{
		num = 10*num + string[i]-'0';
	}
	
	return num;
}