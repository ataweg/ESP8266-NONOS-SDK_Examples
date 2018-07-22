/*
 * ESPRESSIF MIT License
 *
 * Copyright (c) 2016 <ESPRESSIF SYSTEMS (SHANGHAI) PTE LTD>
 *
 * Permission is hereby granted for use on ESPRESSIF SYSTEMS ESP8266 only, in which case,
 * it is free of charge, to any person obtaining a copy of this software and associated
 * documentation files (the "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the Software is furnished
 * to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all copies or
 * substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 */

#include "osapi.h"
#include "user_interface.h"

#include "driver/key.h"

#define WPS_KEY_NUM        1

// --------------------------------------------------------------------------
// pin definitions
// --------------------------------------------------------------------------

// Hardware
#if defined( ESP201 )

   #define BUTTON_IO_NUM     4
   #define BUTTON_IO_MUX     PERIPHS_IO_MUX_GPIO4_U
   #define BUTTON_IO_FUNC    FUNC_GPIO4

#elif defined( NODEMCU )
   #define BUTTON_IO_NUM     0
   #define BUTTON_IO_MUX     PERIPHS_IO_MUX_GPIO0_U
   #define BUTTON_IO_FUNC    FUNC_GPIO0

#elif defined( NODEMCU_MODUL )
   #define BUTTON_IO_NUM     0
   #define BUTTON_IO_MUX     PERIPHS_IO_MUX_GPIO0_U
   #define BUTTON_IO_FUNC    FUNC_GPIO0

#else
   #define BUTTON_IO_MUX     PERIPHS_IO_MUX_MTCK_U
   #define BUTTON_IO_NUM     13
   #define BUTTON_IO_FUNC    FUNC_GPIO13
#endif

LOCAL struct keys_param keys;
LOCAL struct single_key_param *single_key;

LOCAL void ICACHE_FLASH_ATTR
user_wps_status_cb(int status)
{
	switch (status) {
		case WPS_CB_ST_SUCCESS:
			wifi_wps_disable();
			wifi_station_connect();
			break;
		case WPS_CB_ST_FAILED:
		case WPS_CB_ST_TIMEOUT:
			wifi_wps_start();
			break;
	}
}

LOCAL void ICACHE_FLASH_ATTR
user_wps_key_short_press(void)
{
#if 0
	wifi_wps_disable();
	wifi_wps_enable(WPS_TYPE_PBC);
	wifi_set_wps_cb(user_wps_status_cb);
	wifi_wps_start();
#else
   os_printf( "user_wps_key_short_press\r\n" );
#endif
}

LOCAL void ICACHE_FLASH_ATTR
user_wps_key_long_press(void)
{
   os_printf( "user_wps_key_long_press\r\n" );
}

/******************************************************************************
 * FunctionName : user_rf_cal_sector_set
 * Description  : SDK just reversed 4 sectors, used for rf init data and paramters.
 *                We add this function to force users to set rf cal sector, since
 *                we don't know which sector is free in user's application.
 *                sector map for last several sectors : ABCCC
 *                A : rf cal
 *                B : rf init data
 *                C : sdk parameters
 * Parameters   : none
 * Returns      : rf cal sector
*******************************************************************************/
uint32 ICACHE_FLASH_ATTR
user_rf_cal_sector_set(void)
{
    enum flash_size_map size_map = system_get_flash_size_map();
    uint32 rf_cal_sec = 0;

    switch (size_map) {
        case FLASH_SIZE_4M_MAP_256_256:
            rf_cal_sec = 128 - 5;
            break;

        case FLASH_SIZE_8M_MAP_512_512:
            rf_cal_sec = 256 - 5;
            break;

        case FLASH_SIZE_16M_MAP_512_512:
        case FLASH_SIZE_16M_MAP_1024_1024:
            rf_cal_sec = 512 - 5;
            break;

        case FLASH_SIZE_32M_MAP_512_512:
        case FLASH_SIZE_32M_MAP_1024_1024:
            rf_cal_sec = 1024 - 5;
            break;

        case FLASH_SIZE_64M_MAP_1024_1024:
            rf_cal_sec = 2048 - 5;
            break;
        case FLASH_SIZE_128M_MAP_1024_1024:
            rf_cal_sec = 4096 - 5;
            break;
        default:
            rf_cal_sec = 0;
            break;
    }

    return rf_cal_sec;
}

void ICACHE_FLASH_ATTR
user_rf_pre_init(void)
{
}

// --------------------------------------------------------------------------
//
// --------------------------------------------------------------------------

#include <driver/uart.h>
#include <time.h>             // struct tm
#include <sntp.h>             // sntp_get_real_time()

// --------------------------------------------------------------------------
//

extern char __BUILD_UTIME;
extern char __BUILD_NUMBER;

const uint32_t build_number = ( uint32_t ) &__BUILD_NUMBER;
const time_t   build_utime  = ( time_t ) &__BUILD_UTIME;
time_t build_time;

#define TIMEZONE_CORRECTION         3600           // in seconds

// --------------------------------------------------------------------------
//
// --------------------------------------------------------------------------

typedef struct rst_info *rst_info_t;
rst_info_t sys_rst_info;

// --------------------------------------------------------------------------
// functions
// --------------------------------------------------------------------------

// see https://electronicfreakblog.wordpress.com/2014/03/06/die-zeit-im-sommer-und-im-winter/

bool ICACHE_FLASH_ATTR isSummer( time_t timestamp )
{
   // create tm struct
   struct tm *dt = gmtime( &timestamp );

   uint8_t mon   = dt->tm_mon + 1;
   uint8_t mday  = dt->tm_mday;
   uint8_t wday  = dt->tm_wday;
   uint8_t hour  = dt->tm_hour;

   // no summer time from November to Februar
   if( mon > 10 || mon < 3 )
      return false;

   // summer time from April to September
   if( mon > 3 && mon < 10 )
      return true;

   // summer time starts on the last Sunday of March
   if( mon == 3 && ( mday - wday ) >= 25 && hour >= 2 )
      return true;

   // and ends on the last Sunday of October
   if( mon == 10 && ( mday - wday ) < 25 && hour < 3 )
      return true;

   return false;
}

// --------------------------------------------------------------------------
//
// --------------------------------------------------------------------------

char* ICACHE_FLASH_ATTR _sntp_get_real_time( time_t t )
{
   char *time_str = sntp_get_real_time( t );
   int len = strlen( time_str );

   if( time_str[ len - 1 ] == '\n' )
      time_str[ len  - 1 ] = 0;

   return time_str;
}

// --------------------------------------------------------------------------
//
// --------------------------------------------------------------------------

void ICACHE_FLASH_ATTR systemPrintInfo()
{
   os_printf( "\r\n\r\n" );
   os_printf( "-------------------------------------------\r\n" );
   os_printf( "BOOTUP...\r\n" );
   os_printf( "\r\n" );
   os_printf( "ESP8266 platform starting...\r\n" );
   os_printf( "==== System info: ====\r\n" );
   os_printf( "SDK version:    %s  rom %d\r\n", system_get_sdk_version(), system_upgrade_userbin_check() );
   os_printf( "Project:        wps ( ESP8266 )\r\n" );
   os_printf( "Build time:     %s\r\n", _sntp_get_real_time( build_time ) );
   os_printf( "Build number:   %u\r\n", build_number );
   os_printf( "Time:           %d ms\r\n", system_get_time()/1000 );
   os_printf( "Chip id:        0x%x\r\n", system_get_chip_id() );
   os_printf( "CPU freq:       %d MHz\r\n", system_get_cpu_freq() );
   os_printf( "Flash size map: %d\r\n", system_get_flash_size_map() );
   os_printf( "Free heap size: %d\r\n", system_get_free_heap_size() );
   os_printf( "Memory info:\r\n" );
   system_print_meminfo();
   os_printf( " ==== End System info ====\r\n" );
   os_printf( " -------------------------------------------\r\n\r\n" );
}

// --------------------------------------------------------------------------
// print reset cause
// --------------------------------------------------------------------------

const char * ICACHE_FLASH_ATTR reason2Str( int reason )
{
   const char * rst_msg;

   switch( reason )
   {
      case REASON_DEFAULT_RST:        rst_msg = "DEFAULT_RST"     ; break;   // 0: normal startup by power on
      case REASON_WDT_RST:            rst_msg = "WDT_RST"         ; break;   // 1: hardware watch dog reset  exception reset, GPIO status won’t change
      case REASON_EXCEPTION_RST:      rst_msg = "EXCEPTION_RST"   ; break;   // 2: software watch dog reset, GPIO status won’t change
      case REASON_SOFT_WDT_RST:       rst_msg = "SOFT_WDT_RST"    ; break;   // 3: software restart,system_restart, GPIO status won’t change
      case REASON_SOFT_RESTART:       rst_msg = "SOFT_RESTART"    ; break;   // 4:
      case REASON_DEEP_SLEEP_AWAKE:   rst_msg = "DEEP_SLEEP_AWAKE"; break;   // 5: wake up from deep-sleep
      case REASON_EXT_SYS_RST:        rst_msg = "EXT_SYS_RST"     ; break;   // 6: external system reset
      default:                        rst_msg = "unknown";
   }

   return rst_msg;
}

void ICACHE_FLASH_ATTR systemPrintResetInfo( rst_info_t sys_rst_info )
{
   os_printf( "reset reason: %x: %s\r\n", sys_rst_info->reason, reason2Str( sys_rst_info->reason ) );

   if( sys_rst_info->reason == REASON_WDT_RST ||
         sys_rst_info->reason == REASON_EXCEPTION_RST ||
         sys_rst_info->reason == REASON_SOFT_WDT_RST )
   {
      if( sys_rst_info->reason == REASON_EXCEPTION_RST )
      {
         os_printf( "Fatal exception ( %d ):\r\n", sys_rst_info->exccause );
      }

      os_printf( "epc1=0x%08x, epc2=0x%08x, epc3=0x%08x, excvaddr=0x%08x, depc=0x%08x\r\n",
            sys_rst_info->epc1, sys_rst_info->epc2, sys_rst_info->epc3,
            sys_rst_info->excvaddr, sys_rst_info->depc );
   }
}

// --------------------------------------------------------------------------
//
// --------------------------------------------------------------------------

void ICACHE_FLASH_ATTR
user_init(void)
{
   // ----------------------------------
   // get and save reset reasons
   // ----------------------------------

   sys_rst_info = system_get_rst_info();

   system_update_cpu_freq( SYS_CPU_160MHZ );

   // ----------------------------------
   // Setup serial port
   // ----------------------------------

   // Turn log printing on or off.
   system_set_os_print( 1 );

   uart_init( BIT_RATE_115200, BIT_RATE_115200 );  // configures UART0 and UART1, use GPIO2,
   os_delay_us( 1000 );

   // ----------------------------------
   // print system info before wifi starts
   // ----------------------------------

   build_time = build_utime;
   // timezone correction, we get UTC time, but we are at UTC+1
   build_time += TIMEZONE_CORRECTION * 1;
   // summertime correction
   if( isSummer( build_time ) )
       build_time += TIMEZONE_CORRECTION;

   systemPrintInfo();
   systemPrintResetInfo( sys_rst_info );

   // ----------------------------------
   //
   // ----------------------------------

	single_key = key_init_single(BUTTON_IO_NUM, BUTTON_IO_MUX, BUTTON_IO_FUNC,
		                                    user_wps_key_long_press, user_wps_key_short_press);

	keys.key_num = WPS_KEY_NUM;
	keys.single_key = &single_key;

	key_init(&keys);

	wifi_set_opmode(STATION_MODE);
}
