/*
 * common.h
 *
 *  Created on: Apr 29, 2024
 *      Author: wkhaled
 */

#ifndef COMMON_H_
#define COMMON_H_

#define TINY_GSM_MODEM_SIM7600
#define SerialAT Serial1


#define UART_BAUD 115200

#define DUMP_AT_COMMANDS

#define SerialMon Serial

#include <TinyGsmClient.h>
#include <TinyGSM.h>
#define MODEM_TX 17
#define MODEM_RX 16
#define MODEM_PWRKEY 27

#ifdef DUMP_AT_COMMANDS
#include <StreamDebugger.h>
#endif 

 extern StreamDebugger debugger ;
 extern TinyGsm modem ;




 #endif /* COMMON_H_ */
