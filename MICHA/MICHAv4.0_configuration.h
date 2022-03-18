// MICHA PROJECT
// Register and I/O configuration
// For Arduino MKR Zero


// settings
#define VERSION 4             // version of the current PCB
#define ID  1                 // default modbus ID
#define SETUP_DELAY 1000      // ms Delay after startup before Setup
#define BAUD_RATE 19200       // Modbus serial speed
boolean debug_flag = false;   // debug mode enable flag (with debug_flag false, less Serial.print, better timing...)
boolean level1_flag = false;  // allow to enable/disable the level 1 sensor management, i.e. the input level monitoring (0 = disable)
boolean level2_flag = false;  // allow to enable/disable the level 2 sensor management, i.e. the output level monitoring (0 = disable)
boolean press_flag = false;   // allow to enable/disable the pressure sensor management (0 = disable)
uint16_t speed_step = 400;    // default speed Hz decrease/increase




// configuration of the relative register adresses
// coils / output coils
#define THERMIS_POW_REG                   0x00   // register which stores the thermistor power state
#define LEVEL1_FLAG_REG                   0x01   // register which stores the flag which enables/disables the level 1 sensor management
#define LEVEL2_FLAG_REG                   0x02   // register which stores the flag which enables/disables the level 2 sensor management
#define PRESS_FLAG_REG                    0x03   // register which stores the flag which enables/disables the pressure sensor management
#define PUMP_DIR_REG                      0x10   // register which stores the pump direction
#define PUMP_POW_REG                      0x11   // register which stores the pump power state
#define TANK1_REG                         0x20   // register which stores the tank 1 state
#define SOL_HOT_REG                       0x30   // register which stores the hot water solenoid state
#define BOOT_FLAG_REG                     0x40   // register which stores the boot state
#define DEBUG_FLAG_REG                    0x41   // register which stores the debug state
// input bits / discrete inputs
#define LEVEL_SENSOR1_REG                 0x01   // register which stores the state of the input level sensor (1 for water)
#define LEVEL_SENSOR2_REG                 0x02   // register which stores the state of the output level sensor (1 for water)
#define EMERGENCY_STOP_REG                0x10   // register which stores the state of  the emergency stop button (0 for active emergency stop)
// input registers
#define GEN_STATE_REG                     0x00   // register which stores the general state of the system
#define THERMI1_REG                       0x01   // register which stores the thermistor 1 value (0 (high temp) - 4095 (low temp))
#define THERMI2_REG                       0x02   // register which stores the thermistor 2 value (0 (high temp) - 4095 (low temp))
#define THERMI3_REG                       0x03   // register which stores the thermistor 3 value (0 (high temp) - 4095 (low temp))
#define THERMI4_REG                       0x04   // register which stores the thermistor 4 value (0 (high temp) - 4095 (low temp))
#define PRESS_SENSOR_REG                  0x10   // register which stores the pressure sensor value (0 (low pressure) - 4095 (high pressure))
#define ERROR_CODE_REG                    0x20   // register which stores the general error codes
// holding registers
#define ID_REG                            0x00   // register which stores the modbus ID
#define PUMP_SPEED_REG                    0x10   // register which stores the pump speed
#define PUMP_SPEED_INC_REG                0x11   // register which stores the increasing/decreasing value of the pump frequency


// pin assignment
#if VERSION == 4
  #define PRESS_SENSOR_PIN      A0         // pressure sensor input pin (0 (low pressure) - 4095 (high pressure))
  #define THERMI1_PIN           A3         // thermistor 1 input pin (0 (high temp) - 4095 (low temp))
  #define THERMI2_PIN           A4         // thermistor 2 input pin (0 (high temp) - 4095 (low temp))
  #define THERMI3_PIN           A5         // thermistor 3 input pin (0 (high temp) - 4095 (low temp))
  #define THERMI4_PIN           A6         // thermistor 4 input pin (0 (high temp) - 4095 (low temp))
  #define PUMP_POW_PIN          0          // pump power pin
  #define PUMP_DIR_PIN          1          // pump direction pin
  #define PUMP_SPEED_PIN        2          // pump speed pin
  #define TANK1_PIN             3          // tank 1 pin
  #define THERMIS_POW_PIN       4          // thermistor power pin
  #define SOL_HOT_PIN           5          // hot water solenoid pin
  #define LEVEL_SENSOR1_PIN     6          // level sensor 1 input pin
  #define LEVEL_SENSOR2_PIN     7          // level sensor 2 input pin
  #define EMERGENCY_STOP_PIN    10         // emergency stop input pin (0 for active emergency stop)
#endif
