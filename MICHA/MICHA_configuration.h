// MICHA PROJECT
// Register and I/O configuration
// For Arduino MKR Zero

#define VERSION 2 // version of the current PCB


// constants
#define ID  1                 // default modbus ID
uint16_t speed_step = 400;    // default speed Hz decrease/increase
boolean debug_flag = false;   // debug mode enable flag (with debug_flag false, less Serial.print, better timing...)
// ms Delay after startup before Setup
#define SETUP_DELAY 1000
// Modbus serial speed
#define BAUD_RATE 19200


// configuration of the relative register adresses
// coils
#define THERMIS_POW_REG                   0x01   // register which stores the thermistor power state
#define PUMP_DIR_REG                      0x10   // register which stores the pump direction
#define PUMP_POW_REG                      0x11   // register which stores the pump power state
#define TANK1_REG                         0x20   // register which stores the tank 1 state
#define TANK2_REG                         0x21   // register which stores the tank 2 state
#define SOL_HOT_REG                       0x30   // register which stores the hot water solenoid state
#define SOL_COLD_REG                      0x31   // register which stores the cold water solenoid state
#define VALVE1_POW_REG                    0x32   // register which stores the valve 1 power state
#define VALVE1_DIR_REG                    0x33   // register which stores the valve 1 direction
#define VALVE2_POW_REG                    0x34   // register which stores the valve 2 power state
#define VALVE2_DIR_REG                    0x35   // register which stores the valve 2 direction
#define BOOT_FLAG_REG                     0x40   // register which stores the boot state
#define DEBUG_FLAG_REG                    0x41   // register which stores the debug state
// input registers
#define GEN_STATE_REG                     0x00   // register which stores the general state of the system
#define THERMI1_REG                       0x01   // register which stores the thermistor 1 value (0 - 4095)
#define THERMI2_REG                       0x02   // register which stores the thermistor 2 value (0 - 4095)
#define THERMI3_REG                       0x03   // register which stores the thermistor 3 value (0 - 4095)
#define THERMI4_REG                       0x04   // register which stores the thermistor 4 value (0 - 4095)
#define PUMP_ERR_REG                      0x10   // register which stores the error code returned by the pump regulator
#define PUMP_SERVO_PERIODMAX_REG          0x11   // register which stores the max period of the servo signal returned by the pump
#define PUMP_SERVO_PERIODMIN_REG          0x12   // register which stores the min period of the servo signal returned by the pump
#define PUMP_SERVO_PERIODAVG_REG          0x13   // register which stores the period average of the servo signal returned by the pump (on some time)
#define PUMP_SERVO_PERIODSTDDEV_REG       0x14   // register which stores the period standard deviation of the servo signal returned by the pump  (on some time)
#define ERROR_CODE_REG                    0x20   // register which stores the general error codes
// holding registers
#define ID_REG                            0x00   // register which stores the modbus ID
#define PUMP_SPEED_REG                    0x10   // register which stores the pump speed
#define PUMP_SPEED_INC_REG                0x11   // register which stores the increasing/decreasing value of the pump frequency
#define PUMP_SPIN_RATE_REG                0x12   // register which stores the pump spining rate approved
#define PUMP_SERVO_PULSES_REG             0x13   // register which stores the pulse count of the servo signal returned by the pump (on some time)


// pin assignment
#if VERSION == 2
  #define THERMIS_POW_PIN       A0         // thermistor power pin
  #define PUMP_ERR_PIN          A1         // pump error signal pin
  #define PUMP_SERVO_PIN        17         // pump servo signal pin (A2)
  #define THERMI1_PIN           A3         // thermistor 1 pin
  #define THERMI2_PIN           A4         // thermistor 2 pin
  #define THERMI3_PIN           A5         // thermistor 3 pin
  #define THERMI4_PIN           A6         // thermistor 4 pin
  #define PUMP_POW_PIN          0          // pump power pin
#elif VERSION == 1
  #define THERMI1_PIN           A0         // thermistor 1 pin
  #define THERMI2_PIN           A1         // thermistor 2 pin
  #define THERMI3_PIN           A2         // thermistor 3 pin
  #define THERMI4_PIN           A3         // thermistor 4 pin
  #define THERMIS_POW_PIN       21         // thermistor power pin
  #define PUMP_ERR_PIN          19         // pump error signal pin
  #define PUMP_POW_PIN          20         // pump power pin   DO NOT USE FOR NOW
  #define PUMP_SERVO_PIN        0          // pump servo signal pin
#endif
// common pins
#define PUMP_DIR_PIN          1          // pump direction pin
#define PUMP_SPEED_PIN        2          // pump speed pin
#define TANK1_PIN             3          // tank 1 pin
#define TANK2_PIN             4          // tank 2 pin
#define SOL_HOT_PIN           5          // hot water solenoid pin
#define VALVE1_POW_PIN        6          // valve 1 power pin
#define VALVE1_DIR_PIN        7          // valve 1 direction pin
#define SOL_COLD_PIN          8          // cold water solenoid pin
#define VALVE2_POW_PIN        9          // valve 2 power pin
#define VALVE2_DIR_PIN        10         // valve 2 direction pin
