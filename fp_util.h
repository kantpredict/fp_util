#ifndef FP_UTIL_H
#define FP_UTIL_H

#include <Arduino.h>
#include <mcp_can.h> // Include mcp_can.h here as it's needed for MCP_CAN type

#define FP_VERSION "v1.0"
#define FP_SERIAL_SPEED 115200
#define FP_DEF_MON_INTRVL 5 /* s */
#define FP_MIN_MON_INTRVL 1 /* s */
#define FP_CMD_LEN_MAX 8
#define FP_MIN_VOLTS 4350    /* cV */
#define FP_MAX_VOLTS 5760    /* cV */
#define FP_DEF_VOLTS 5020    /*cV default voltage - implement later*/
#define FP_MIN_AMPS 10
#define FP_MAX_AMPS 200      /* dA */ //HARD CODED TO 10 AMPS (100 DECIAMPS)
#define FP_LOGIN_INTRVL 1000 /* ms */
#define FP_RX_INTRVL 2000    /* ms */
#define FP_SERIAL_LEN 6
#define FP_CVSET_LEN 8
#define FP_DVSET_LEN 5
#define FP_ARG_OFFSET 2
#define FP_P "\n>> "

#define FP_CAN_SPEED CAN_125KBPS
#define FP_CAN_PIN_CS 10
#define FP_CAN_PIN_INT 3

/* Is 00 below a broadcast ? */
#define FP_CAN_ID_LOGIN 0x05004800

/*
 * Not sure why the 'FF' below. Last byte is "walk in", which seems to only
 * happen during start up, from 44v to the default voltage.
 */
// Based on protocol doc, this should be 0x05xx4004 for setting voltage and current limits
// We will use this ID for both 'v' and 'a' commands.
#define FP_CAN_ID_CVSET 0x05FF4004 // Using 0xFF as placeholder for device ID initially, will be replaced


/*
 * It seems the '01' below is the device ID, all hard-coded here. This would need
 * to be changed if supporting multiple units.
 */
#define FP_CAN_ID_STAT 0x05014004 // Note: This ID is also used for status messages according to the doc
#define FP_CAN_ID_DVSET 0x05019C00
#define FP_CAN_DEV_ID 0x01
#define FP_CAN_DEV_ADDR (FP_CAN_DEV_ID << 2) /* Single device on bus with address ID '1' */
#define FP_CAN_MSG_LEN 8

struct fp_device_status {
  float in_voltage;
  float out_voltage;
  float current;
  uint8_t in_temp;
  uint8_t out_temp;
  bool ramping;
  bool warning;
  bool alarm;
};

struct fp_state {
  bool serial_received;
  uint8_t serial[FP_SERIAL_LEN];
  unsigned long last_login;
  bool req_menu;
  bool req_status;
  bool req_mon;
  unsigned long mon_interval;
  unsigned long mon_last;
  unsigned long rx_last;
  unsigned long wait_start;
  uint32_t wait_id;
  struct fp_device_status stat;
  int last_requested_voltage; // Added last_requested_voltage here (int is sufficient for cV)
};

#endif /* FP_UTIL_H */
