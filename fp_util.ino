/*
 * fp_util
 *
 * Utility to manage an Eltek Flatpack 2 power supply via an MCP2515 CAN controller chip.
 * This should work with all shields, as long as the CS & INT pins are correct.
 *
 * Note the use of F("") strings. This is to save SRAM by storing strings in flash (PROGMEM),
 * which I needed when extending this code to include an OLED display.
 * YMMV.
 *
 * Derived from https://github.com/the6p4c/Flatpack2/blob/master/Arduino/fp2_control/
 *
 * Information sources:
 * https://github.com/neggles/flatpack2s2/blob/main/docs/Protocol.md
 * https://github.com/the6p4c/Flatpack2/blob/master/Protocol.md
 * https://openinverter.org/forum/viewtopic.org?t=1351
 * https://github.com/neggles/flatpack2s2
 * https://github.com/tomvanklinken/Flatpack2
 *
 * This is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * A copy of the GNU Lesser General Public License, <http://www.gnu.org/licenses/>.
 *
 * Copyright (c) 2023 James Morris <jmorris@namei.org>
 * */

#include <mcp_can.h>
#include <SPI.h>
#include <LibPrintf.h>
#include "fp_util.h" // Ensure you have fp_util.h in the same directory as this file

// Forward declarations for functions used before their definitions
void fp_mon_exit(void);
void fp_cmd_monitor(String arg);
void fp_print_serial(void);
void fp_process_status(uint32_t rxid, uint8_t len, uint8_t rxbuf[]);
void fp_print_status(void);
const char *fp_bool_str(bool val);
uint8_t fp_sendmsg(uint32_t can_id, uint8_t buf[], uint8_t len); // Forward declaration for fp_sendmsg


const char *alarms0Strings[] = { "OVS_LOCK_OUT", "MOD_FAIL_PRIMARY", "MOD_FAIL_SECONDARY",
                                 "HIGH_MAINS", "LOW_MAINS", "HIGH_TEMP", "LOW_TEMP", "CURRENT_LIMIT" };
const char *alarms1Strings[] = { "INTERNAL_VOLTAGE", "MODULE_FAIL", "MOD_FAIL_SECONDARY",
                                 "FAN1_SPEED_LOW", "FAN2_SPEED_LOW", "SUB_MOD1_FAIL", "FAN3_SPEED_LOW", "INNER_VOLT" };
MCP_CAN FP_CAN(FP_CAN_PIN_CS);

struct fp_state fp = {
  serial_received: false,
  serial: { 0 },
  last_login: 0,
  req_menu: false,
  req_status: false,
  req_mon: false,
  mon_interval: FP_DEF_MON_INTRVL * 1000L,
  mon_last: 0,
  rx_last: 0,
  wait_start: 0,
  wait_id: 0,
  stat: { 0 }, // Initialize the nested struct
  last_requested_voltage: FP_DEF_VOLTS // Initialize with default voltage in cV (used for 'a' command)
};
void fp_err(const __FlashStringHelper *msg) {
  Serial.print(F(FP_P));
  Serial.print(F("ERROR: "));
  Serial.println(msg);
}

void fp_prompt(const __FlashStringHelper *msg) {
  Serial.print(F(FP_P));
  Serial.print(msg);
}

bool fp_logged_in(void) {
  return fp.serial_received;
}

/*
 * Fills buffer for setting current voltage and current limit using FP_CAN_ID_CVSET (0x05xx4004).
 * Based on protocol: bytes 0 and 1 are Max Current (dA), bytes 4 and 5 are Desired Voltage (cV).
 * Bytes 2 and 3 are Measured Voltage (cV), bytes 6 and 7 are OVP Voltage (cV).
 */
void fp_fill_cv_cl_set(uint8_t buf[], int amps_val_dA, int volts_val_cV) {
  // Bytes 0 and 1: Max Current (deciAmps) - Little-endian
  buf[0] = (amps_val_dA & 0xFF);
  buf[1] = ((amps_val_dA >> 8) & 0xFF);

  // Bytes 2 and 3: Measured Voltage (centiVolts) - Using desired voltage for now as per protocol note - Little-endian
  buf[2] = (volts_val_cV & 0xFF);
  buf[3] = ((volts_val_cV >> 8) & 0xFF);

  // Bytes 4 and 5: Desired Voltage (centiVolts) - Little-endian
  buf[4] = (volts_val_cV & 0xFF);
  buf[5] = ((volts_val_cV >> 8) & 0xFF);

  // Bytes 6 and 7: OVP Voltage (centiVolts) - Set to max as per original code - Little-endian
  int ovp_volts_cV = FP_MAX_VOLTS; // Use the defined constant directly
  buf[6] = (ovp_volts_cV & 0xFF);
  buf[7] = ((ovp_volts_cV >> 8) & 0xFF);
}


/*
 * Sets voltage only using FP_CAN_ID_CVSET (0x05xx4004).
 * Keeps current limit at FP_MAX_AMPS (deciAmps).
 */
void fp_cmd_cur_volts(String arg) {
  uint8_t rv, txbuf[FP_CVSET_LEN];
  float val = arg.toFloat();
  int cv = (int)(val * 100.0); // Convert volts to centivolts

  /* Invalid or missing arg */
  if (cv < FP_MIN_VOLTS || cv > FP_MAX_VOLTS) {
    fp_err(F("invalid voltage argument."));
    return;
  }

  fp_prompt(F("Setting the voltage to "));
  printf("%.2fv... ", val);

  // Store the last requested voltage in centiVolts
  fp.last_requested_voltage = cv; // Access directly from fp state

  // Use the fill function to set voltage and max current limit
  fp_fill_cv_cl_set(txbuf, FP_MAX_AMPS, cv);

  // Use FP_CAN_ID_CVSET (0x05xx4004) which controls both voltage and current limit
  // Replace the placeholder 0xFF with the actual device ID
  uint32_t can_id = (FP_CAN_ID_CVSET & 0xFFFFFF00) | FP_CAN_DEV_ADDR;
  rv = fp_sendmsg(can_id, txbuf, FP_CVSET_LEN);
  if (rv) {
    fp_prompt(F("Done. Check [s]tatus to verify.\n"));
  } else {
    fp_err(F("sendMsgBuf failed in fp_cmd_set_cur_volts"));
  }
}

void fp_fill_dvset(uint8_t buf[], int val) {
  buf[0] = 0x29;
  buf[1] = 0x15;
  buf[2] = 0x00;
  // Default voltage in centiVolts (val) - Little-endian

  Serial.print(F("  fp_fill_dvset: val = "));
  Serial.print(val);
  Serial.print(F(" (0x"));
  Serial.print(val, HEX);
  Serial.println(F(")"));
  Serial.print(F("  fp_fill_dvset: Low byte (val & 0xFF) = "));
  Serial.print((val & 0xFF), HEX);
  Serial.println();
  Serial.print(F("  fp_fill_dvset: High byte ((val >> 8) & 0xFF) = "));
  Serial.print(((val >> 8) & 0xFF), HEX);
  Serial.println();


  buf[3] = (val & 0xFF);
  buf[4] = ((val >> 8) & 0xFF);
}

void fp_res_set_def_volts(void) {
  fp_stop_wait();
  printf(FP_P "Done. This change will take effect after the next power cycle.\n\n");
}

void fp_cmd_def_volts(String arg) {
  uint8_t txbuf[FP_DVSET_LEN];
  float val = arg.toFloat();
  int cv = (int)(val * 100.0); // Convert volts to centivolts
/* Invalid or missing arg */
  if (cv < FP_MIN_VOLTS || cv > FP_MAX_VOLTS) {
    fp_err(F("invalid voltage argument."));
    return;
  }

  fp_prompt(F("Setting the default voltage to "));
  printf("%.2fv...", val);
  fp_fill_dvset(txbuf, cv);
  // Use the correct CAN ID for setting default voltage
  uint32_t can_id = FP_CAN_ID_DVSET; // This ID already includes the device ID
  if (FP_CAN.sendMsgBuf(can_id, 1, FP_DVSET_LEN, txbuf) != CAN_OK) {
    fp_err(F("sendMsgBuf failed in fp_cmd_set_def_volts"));
    return;
  }

  fp_start_wait(FP_CAN_ID_DVSET);
}

// Define FP_CSET_LEN based on protocol (8 bytes for Set voltage and current limits)
#define FP_CSET_LEN 8

// Define the CAN ID for setting voltage and current limits (0x05xx4004)
// This is the same as FP_CAN_ID_CVSET
#define FP_CAN_ID_CSET FP_CAN_ID_CVSET

// Function to prepare the buffer for setting current limit and voltage
// When setting current limit, use the last requested voltage as the desired voltage setpoint.
void fp_make_cset(uint8_t buf[], uint16_t amps_tenths) {
  // Use the last requested voltage as the desired voltage setpoint
  int desired_volts_cV = fp.last_requested_voltage; // Access directly from fp state

  // Fill the buffer with the new current limit and the last requested voltage as the desired voltage
  fp_fill_cv_cl_set(buf, amps_tenths, desired_volts_cV);
}

// Helper function to send a CAN message and return success/failure
uint8_t fp_sendmsg(uint32_t can_id, uint8_t buf[], uint8_t len) {
  uint8_t rv = FP_CAN.sendMsgBuf(can_id, 1, len, buf);
  // Return true (1) for success (CAN_OK), false (0) otherwise
  return (rv == CAN_OK);
}


// User-provided function to handle the command for setting current limits
void fp_cmd_cur_amps(String arg) {
  uint8_t rv, txbuf[FP_CSET_LEN];
  float val = arg.toFloat();  // Convert input string to float

  // Validate the input using FP_MIN_AMPS and FP_MAX_AMPS which are in deciAmps
  if (val < (float)FP_MIN_AMPS / 10.0 || val > (float)FP_MAX_AMPS / 10.0) {
    Serial.print(F("Invalid current. Range is "));
    Serial.print((float)FP_MIN_AMPS / 10.0, 1); // Display with 1 decimal place
    Serial.print(F("–"));
    Serial.print((float)FP_MAX_AMPS / 10.0, 1); // Display with 1 decimal place
    Serial.println(F(" A"));
    return;
  }

  uint16_t tenths = (uint16_t)(val * 10);  // Convert to tenths of an amp

  fp_make_cset(txbuf, tenths);
  Serial.print(F("Setting current limit to "));
  Serial.print(val, 1);
  Serial.print(F(" A (raw: "));
  Serial.print(tenths);
  Serial.println(F(")"));

  // Use the correct CAN ID for setting voltage and current limits
  // Replace the placeholder 0xFF with the actual device ID
  uint32_t can_id = (FP_CAN_ID_CSET & 0xFFFFFF00) | FP_CAN_DEV_ADDR;
  rv = fp_sendmsg(can_id, txbuf, FP_CSET_LEN);
  if (rv) // fp_sendmsg returns 1 for success
    Serial.println(F("✓ Current limit set"));
  else
    Serial.println(F("✗ Failed to set current limit"));
}


// Definition of fp_bool_str - Moved here to be before fp_print_status
const char *fp_bool_str(bool val) {
  return val ?
 "True" : "False";
}

/*
 * Status monitor functions
 */

/* Note: we don't flush input here, so we can process a command if one was typed.
 */
void fp_mon_exit(void) {
  fp.req_mon = false;
  fp.mon_interval = FP_DEF_MON_INTRVL * 1000L;
  fp_prompt(F("Exited monitor mode.\n"));
}

void fp_cmd_monitor(String arg) {

  if (arg.length()) {
    unsigned int val = arg.toInt();
    if (val < FP_MIN_MON_INTRVL) {
      fp_err(F("invalid argument"));
      return;
    }
    fp.mon_interval = val * 1000L;
  }

  fp_prompt(F("Entering monitor mode, press enter to stop...\n"));
  Serial.println(F("\n[Vo Io Vi Ti To Ramp Warn Alarm]"));
  fp.req_mon = true;
}

void fp_monitor(void) {
  unsigned long now = millis();
  if (now - fp.mon_last > fp.mon_interval) {
    fp.mon_last = now;
    printf("%.2f ", fp.stat.out_voltage);
    printf("%.2f ", fp.stat.current);
    printf("%.2f ", fp.stat.in_voltage);
    printf("%d ", fp.stat.in_temp); // Keep temperature as int as per original struct
    printf("%d ", fp.stat.out_temp); // Keep temperature as int as per original struct
    printf("%d %d %d\n", fp.stat.ramping, fp.stat.warning, fp.stat.alarm);
  }
}

void fp_print_status(void) {

  Serial.print(F(FP_P));
  Serial.println(F("Status:\n"));
  Serial.print(F("  Serial #:            "));
  fp_print_serial();
  Serial.print(F("\n  Output voltage:      "));
  printf("%.2f V DC\n", fp.stat.out_voltage);
  Serial.print(F("  Output current:      "));
  printf("%.2f A DC\n", fp.stat.current);
  Serial.print(F("  Input voltage:       "));
  printf("%.2f V AC\n", fp.stat.in_voltage);
  Serial.print(F("  Intake temperature:  "));
  printf("%d C\n", fp.stat.in_temp);
  Serial.print(F("  Output temperature:  "));
  printf("%d C\n", fp.stat.out_temp);
  Serial.print(F("  Voltage ramping:     "));
  printf("%s\n", fp_bool_str(fp.stat.ramping));
/* TODO: just print the warning or alarm values */
  Serial.print(F("  Warning:             "));
  printf("%s\n", fp_bool_str(fp.stat.warning));
  Serial.print(F("  Alarm:               "));
  printf("%s\n", fp_bool_str(fp.stat.alarm));

  Serial.println(F(""));

/* TODO: is there an ack message for this ?
 */
  if (fp.stat.warning || fp.stat.alarm) {
    uint8_t rv, txbuf[3] = { 0x08, uint8_t(fp.stat.warning ? 0x04 : 0x08), 0x00 };
    rv = FP_CAN.sendMsgBuf(0x0501BFFC, 1, 3, txbuf);
    if (rv != CAN_OK && rv != CAN_SENDMSGTIMEOUT)
      fp_err(F("sendMsgBuf failed in fp_print_status"));
  }
}

// Definition of fp_process_status - Moved here to be before fp_read_CAN
void fp_process_status(uint32_t rxid, uint8_t len, uint8_t rxbuf[]) {
  // The original fp_read_status function was essentially doing the initial read.
  // We can keep the status parsing logic here.
  fp.stat.in_temp = rxbuf[0];
  fp.stat.out_temp = rxbuf[7];
  fp.stat.current = 0.1 * (rxbuf[1] | (rxbuf[2] << 8));
  fp.stat.out_voltage = 0.01 * (rxbuf[3] | (rxbuf[4] << 8));
  fp.stat.in_voltage = rxbuf[5] | (rxbuf[6] << 8);

  /* TODO: use masks & functions for these ?
 */
  fp.stat.ramping = (rxid == 0x05014010) ? true : false;
  fp.stat.warning = (rxid == 0x05014008) ? true : false;
  fp.stat.alarm = (rxid == 0x0501400C) ? true : false;


  /* Monitoring in effect */
  if (fp.req_mon) {
    fp_monitor();
    return;
  }

  /* User requested status via 's' */
  if (fp.req_status) {
    fp.req_status = false;
    fp_print_status();
  }
}


void fp_banner(void) {
  printf("\n>>>> Starting fp_util %s <<<<\n", FP_VERSION);
}

void setup() {
  Serial.begin(FP_SERIAL_SPEED);
  while (!Serial) {
    ; /* wait for serial port to connect. Needed for native USB.
 */
  }

  fp_banner();

  pinMode(FP_CAN_PIN_CS, OUTPUT);
  pinMode(FP_CAN_PIN_INT, INPUT);

  if (FP_CAN.begin(MCP_ANY, FP_CAN_SPEED, MCP_8MHZ) != CAN_OK) {
    fp_err(F("failed to initialize MCP2515"));
    while (1)
      ;
  }
  FP_CAN.setMode(MCP_NORMAL);
  fp_prompt(F("Looking for a Flatpack2 device... "));

  // Set the startup requested voltage using the 'set current voltage' command
  uint8_t txbuf_vset[FP_CVSET_LEN];
  int startup_volts_cv = FP_DEF_VOLTS; // Use the defined default voltage value

  Serial.print(F("Attempting to set startup requested voltage to "));
  printf("%.2fv...\n", (float)startup_volts_cv / 100.0);

  // Use fp_fill_cv_cl_set which prepares the buffer for the 0x05xx4004 command
  // Set current limit to max (FP_MAX_AMPS) and desired voltage to startup_volts_cv
  fp_fill_cv_cl_set(txbuf_vset, FP_MAX_AMPS, startup_volts_cv);

  // Use FP_CAN_ID_CVSET (0x05xx4004) which controls both voltage and current limit
  // Replace the placeholder 0xFF with the actual device ID
  uint32_t can_id_vset = (FP_CAN_ID_CVSET & 0xFFFFFF00) | FP_CAN_DEV_ADDR;

  Serial.print(F("Sending CAN message to ID 0x"));
  Serial.print(can_id_vset, HEX);
  Serial.print(F(" with data: "));
  for(int i = 0; i < FP_CVSET_LEN; i++) {
    Serial.print(txbuf_vset[i], HEX);
    Serial.print(" ");
  }
  Serial.println();


  uint8_t rv = FP_CAN.sendMsgBuf(can_id_vset, 1, FP_CVSET_LEN, txbuf_vset);
  if (rv != CAN_OK) {
    fp_err(F("sendMsgBuf failed when setting startup voltage in setup"));
    printf("Error code: %d\n", rv);
  } else {
    Serial.println(F("Startup voltage CAN message sent successfully."));
  }

  // Note: We are not waiting for a specific response for this command in setup
}

void fp_do_login() {
  uint8_t rv, txbuf[FP_CAN_MSG_LEN] = { 0 };
  for (int i = 0; i < FP_SERIAL_LEN; i++) {
    txbuf[i] = fp.serial[i];
  }

  // Use the LOGIN CAN ID with the device address
  uint32_t can_id = FP_CAN_ID_LOGIN | FP_CAN_DEV_ADDR;
  rv = FP_CAN.sendMsgBuf(can_id, 1, FP_CAN_MSG_LEN, txbuf);
  if (rv != CAN_OK && rv != CAN_SENDMSGTIMEOUT) {
    fp_err(F("sendMsgBuf failed in fp_do_login"));
    printf("%d\n", rv);
  }
}

void fp_print_serial(void) {
  if (fp.serial_received) {
    for (int i = 0; i < FP_SERIAL_LEN; i++)
      printf("%.2X", fp.serial[i]);
  } else
    Serial.print(F("[none]"));
}

void fp_process_login_req(uint32_t rxid, uint8_t rxbuf[]) {

  for (int i = 0; i < FP_SERIAL_LEN; i++)
    fp.serial[i] = rxbuf[i + 1];
  fp.serial_received = true;
  fp.req_menu = true;

  Serial.print(F("found serial #: "));
  fp_print_serial();
  Serial.print(F("\n"));
}


void processWarningOrAlarmMessage(uint32_t rxid, uint8_t len, uint8_t rxbuf[]) {
  bool isWarning = rxbuf[1] == 0x04;
  if (isWarning) {
    Serial.print(F("  Warnings:           "));
  } else {
    Serial.print(F("  Alarms:             "));
  }

  uint8_t alarms0 = rxbuf[3];
  uint8_t alarms1 = rxbuf[4];
  for (int i = 0; i < 8; i++) {
    if (alarms0 & (1 << i)) {
      Serial.print(F(" "));
      Serial.print(alarms0Strings[i]);
    }

    if (alarms1 & (1 << i)) {
      Serial.print(F(" "));
      Serial.print(alarms1Strings[i]);
    }
  }

  Serial.println(F("\n"));
}

bool fp_rx_CAN(void) {
  return digitalRead(FP_CAN_PIN_INT) ? false : true;
}

void fp_read_CAN(void) {
  uint32_t rxid;
  uint8_t len = 0;
  uint8_t rxbuf[FP_CAN_MSG_LEN];

  if (!fp_rx_CAN())
    return;
  if (FP_CAN.readMsgBuf(&rxid, &len, rxbuf) != CAN_OK) {
    fp_err(F("readMsgBuf failed in fp_read_CAN"));
    return;
  }

  fp.rx_last = millis();
/* Limit ID to lowest 29 bits */
  rxid &= CAN_EXTENDED_ID;
  if (!fp.serial_received && (rxid & 0xFFFF0000) == 0x05000000) {
    if (len < FP_SERIAL_LEN) {
      fp_err(F("Introduction message length mismatch!\n"));
      return;
    }
    fp_process_login_req(rxid, rxbuf);
    return;
  }

  if (fp_wait()) {
    if (fp_process_wait(rxid))
      return;
  }

  /* Still waiting? Check for timeout. */
  if (fp_wait())
    fp_wait_timeout();
/*
 * Process status messages (0x05xx4004)
 * Note: The same ID is used for sending voltage/current set commands.
 * We differentiate based on whether we are receiving or sending.
 */
  if ((rxid & 0xFFFFFF00) == 0x05014000) // Check for status message pattern
    fp_process_status(rxid, len, rxbuf);
  else if (rxid == 0x0501BFFC) // Alert message
    processWarningOrAlarmMessage(rxid, len, rxbuf);

  return;
}

/* Log in every second to stay in contact with device.
 */
void fp_login(void) {

  if (fp.serial_received) {
    unsigned long now = millis();
/* Reset login state if we have not heard from the device (probably powered off) */
    if (now - fp.rx_last >= FP_RX_INTRVL) {
      fp.serial_received = false;
      fp_prompt(F("Lost connection! Looking for device again..."));
      return;
    }

    /* Otherwise, log in again if it's time to do so */
    if (now - fp.last_login >= FP_LOGIN_INTRVL) {
      fp_do_login();
/* TODO: check for failed login */
      fp.last_login = millis();
    }
  }
}

bool fp_wait(void) {
  return !!fp.wait_id;
}

void fp_start_wait(uint32_t wait_id) {
  fp.wait_id = wait_id;
  fp.wait_start = millis();
}

/*
 * It seems the device will ignore bad input rather than produce an error,
 * so we check for a timeout while waiting then fail the operation.
 */
bool fp_wait_timeout(void) {
  unsigned long now = millis();

  if ((millis() - fp.wait_start) > FP_RX_INTRVL) {
    fp_stop_wait();
    fp_err(F("Timed out waiting for response: task failed.\n"));
    return false;
  }

  return true;
}

void fp_stop_wait(void) {
  fp.wait_id = 0;
}

/*
 * We are waiting for an Ack message with fp.wait_id, see if this is it.
 */
bool fp_process_wait(uint32_t rxid) {
  bool rv = false;

  /* Not the message we are waiting for */
  if (rxid != fp.wait_id) {
    return rv;
  }

  switch (rxid) {
    case FP_CAN_ID_DVSET:
      fp_res_set_def_volts();
      rv = true;
      break;
    // Add cases for other commands that expect a specific response if needed
    default:
      fp_err(F("fp_process_wait: unexpected rxid"));
      printf(" 0x%08lx\n", rxid);
  }
  return rv;
}

/*
 * Menu command processing.
 */

void fp_cmd_status(void) {
  fp.req_status = true;
}

static void fp_display_menu(void) {
  fp_prompt(F("Commands:\n\n"));
  Serial.println(F("  h             -  Help, prints this."));
  Serial.println(F("  s             -  Display full device status."));
  Serial.print(F("  m [seconds]   -  Continous monitoring, press enter to stop. Default is "));
  Serial.println(FP_DEF_MON_INTRVL);
  Serial.println(F("                   second(s) between updates."));
  Serial.print(F("  v volts       -  Set the output voltage. Range: "));
  printf("%.2f to %.2f volts.\n", (float)FP_MIN_VOLTS / 100.0, (float)FP_MAX_VOLTS / 100.0);

  // Updated menu option for setting current limit
  Serial.print(F("  a amps        -  Set the output current limit. Range: "));
  printf("%.1f to %.1f amps.\n", (float)FP_MIN_AMPS / 10.0, (float)FP_MAX_AMPS / 10.0);

  Serial.print(F("  d volts       -  Set the default startup voltage. Default is "));
  printf("%.2fv.\n", (float)FP_DEF_VOLTS / 100.0);

  Serial.println(F("\n"));
}

void fp_menu(void) {
  if (!fp.req_menu)
    return;
  fp.req_menu = false;
  fp_display_menu();
}

void fp_get_cmd(void) {
  String cmd;
  float arg;
  char c;
  int v;

  if (!Serial.available())
    return;

  if (!fp_logged_in())
    return;
/* Exit monitor mode on key press */
  if (fp.req_mon) {
    fp_mon_exit();
    return;
  }

  cmd = Serial.readString();
  if (cmd.length() > FP_CMD_LEN_MAX) {
    fp_err(F("invalid command length"));
    fp.req_menu = true;
    return;
  }

  cmd.trim();
  c = tolower(cmd.charAt(0));

  switch (c) {
    case 's':
      fp_cmd_status();
      break;
    case 'm':
      fp_cmd_monitor(cmd.substring(FP_ARG_OFFSET));
      break;
    case 'v':
      fp_cmd_cur_volts(cmd.substring(FP_ARG_OFFSET));
      break;
    case 'a': // Handle setting current limit using the provided function
      fp_cmd_cur_amps(cmd.substring(FP_ARG_OFFSET));
      break;
    case 'd':
      fp_cmd_def_volts(cmd.substring(FP_ARG_OFFSET));
      break;
    default:
/* Error or help: display menu, and error message if needed.
 */
      if (c != 'h' && c != '\0')
        fp_err(F("invalid command"));
      fp.req_menu = true;
      break;
  }
}

void loop() {
  fp_read_CAN();
  fp_login();
  fp_menu();
  fp_get_cmd();
}
