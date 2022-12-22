// MICHA project
// Slave modbus code (modbus server)
// Board type: Arduino MKR Zero
// Components of the pastorizator managed:
//    - pump
//    - 1 heating tank (3000W)
//    - 1 solenoid
//    - 4 thermistors
//    - 2 level sensors
//    - 1 pressure sensor
//
// WARNING: please verify the output states match with your hardware states (is it the same logic?)
//
// Version notes:
//  - v4.0.0:
//          - Raspberry motherboard is replaced by Odroid C4 (deletion, addition and new assignement of some pins)
//          - deletion of the pins and the registers related to: pump error, pump servo, tank2, valve 1, valve 2, cold solenoid
//          - deletion of all the code line related to the previous pins and registers
//          - following pins arLEVEL_SENSOR1_REG added: PRESS_SENSOR_PIN, LEVEL_SENSOR1_PIN, LEVEL_SENSOR2_PIN, EMERGENCY_STOP_PIN
//          - following variables are added:
//              - level1_flag: default value (1 or 0) if level sensor 1 is unplugged
//              - level2_flag: default value (1 or 0) if level sensor 2 is unplugged
//              - press_flag: to enable/disable the use of the pressure sensor
//          - following registers are added: PRESS_SENSOR_REG, LEVEL_SENSOR1_REG, LEVEL_SENSOR2_REG, EMERGENCY_STOP_REG,
//            LEVEL1_FLAG_REG, LEVEL2_FLAG_REG, PRESS_FLAG_REG
//          - following pin assignement is updated: THERMIS_POW_PIN (4 instead of A0)
//          - following regsister is updated: THERMIS_POW_REG (0x00 instead of 0x01)
//          - deletion of V1 and V2 PCB
//          - new functions: 
//            - manage_levels(): polls the level sensor and stops the pump if the input tank is empty (level 1) or the output tank is full (level 2)
//            - get_pressure(): poll the pressure sensor
//            - stop_pump(): stops the pump (this function is called at several place)
//  - v2.0.0:
//          - dissociation of the pin assignement of the V1 and V2 PCB
//  - v1.1.5:
//          - speed_step as a variable (instead of define)
//          - modification of speed_step by PUMP_SPEED_INC_REG implemented
//          - debug_flag moves in MICHA_configuration.h
//  - v1.1.4:
//          - pump servo signal decoding implemented
//          - debug management implemented (DEBUG_FLAG_REG)
//  - v1.1.3:
//          - registrer structure updated: the pump servo registers have been added/modified
//  - v1.1.2:
//          - reading and decoding the pump error signal implemented (writing in register not yet implemented)
//          - loop section reorganized
//  - v1.1.1:
//          - reading the servo signal updated (tested)
//  - v1.1.0:
//          - reset detection implemented (BOD12, BOD33 and WDT) :
//              - add BOOT_FLAG_REG
//              - GEN_STATE_REG and ERROR_CODE_REG managed to store unwanted reboots
//  - v1.0.1:
//          - THERMIS_POW_REG now controls the thermistor power pin
//  - v1.0.0:
//          - PUMP_SPEED_INC_REG added to store the increasing/decreasing frequency value of the pump
//          - first version tested in the pastorizator
//  - v0.3.5:
//          - progressive speeding up of the pump speed implemented:
//            - thermistor reading delays deleted
//            - hardware clock used
//  - v0.3.4:
//          - SAMD21turboPWM library usedto manage the pump speed
//          - PUMP_SPEED_PIN and PUMP_POW_PIN updated (PUMP_SPEED_PIN = 2 (PWM) and PUMP_POW_PIN = 0)
//  - v0.3.3:
//          - registers updated
//          - valve pin names updated
//          - Using of a constant for the default modbus ID
//          - Output updated only when a register is modified
//          - function to read the pump error signal added (not tested)
//          - function to read the pump servo signal added (not tested)
//          - correction moyenne valeurs thermistance 4
//  - v0.3.2:
//          - registers updated:
//            - VALVE1_DIR_REG
//            - VALVE2_DIR_REG
//            - ERROR_CODE_REG
//            - POMPE_TAUX_PATINAGE_REG
//          - pin assignment updated: SOL_COLD_PIN and VANNE_EVAC1_PIN switched
//          - register comments added
//  - v0.3.1:
//          - thermistor average implemented
//  - v0.3:
//          - function prototypes deleted (because unnecessary)
//          - tanks, solenoids and valves management functions implemented
//  - v0.2.2:
//          - Single control valves replaced by double control valves
//          - solenoid names updated
//          - valve names updated
//          - register addresses and pin number moved to MICHA_confirutation.h
//          - register names used to assign register default values
//          - default value for the valve 2 register added
//  - v0.2.1: 
//          - valve 2 added
//  - v0.2:
//          - id in memory flash implemented
//  - v0.1:
//          - register structure updated with all other registers (pump, valves...)
//  - v0.0:
//          - modbus communication implemented
//          - thermistor registrer structure

#include <ArduinoRS485.h>           // ArduinoModbus.h depends on the ArduinoRS485
#include <ArduinoModbus.h>
#include <FlashStorage.h>
#include <SAMD21turboPWM.h>
#include "MICHA41_configuration.h"    // register configuration and pin assignment file

// To store the ID in the flash memory
struct StructID
{
  boolean init;                   // set to 1 when the structure is initialized for the first time
  int8_t id;                      // modbus ID
  int16_t write_number = 0;       // number of rewriting of the ID since the last board flashing
};

FlashStorage(ID_FLASH, StructID);


// Pump motor management
TurboPWM pwm;
unsigned long long int steps = 0; // to store the number of pump steps
boolean speed_flag = false;       // to indicate if a speed change is occured
boolean stopped = true;           // flag to be able to force the complete stop of the pump

// miscellaneous variables
uint32_t time_ref1 = 0;           // thermistor reading reference time
uint32_t time_ref2 = 0;           // reference time for other operations

StructID id;                      // stores the ID for the flash memory


void setup()
{
  Serial.begin(9600);
  delay(SETUP_DELAY);
  
  // Storing the modbus ID of the flash memory
  id = ID_FLASH.read();

  if (id.init == false)   // if the ID was not yet initialized
  {
    id.id = ID; // using of the default ID (1)
    id.write_number = 0;  // initialized of the write number since the last flashing
    id.init = true;

    ID_FLASH.write(id);   // writing in flash memory

    delay(1000);  // Leave a small delay to start the Arduino Terminal window
    Serial.print("First start. The modbus ID is ");
  } else  // if the ID was yet initialized
  {
    Serial.print("The modbus ID is ");
  }
  Serial.println(id.id);
  Serial.print("\n");
  
  // Main configuration
  analogReadResolution(12);       // setting in 12 bit mode (10 bit mode by default)
  analogReference(AR_EXTERNAL);   // using of an external voltage as AREF reference
  
  // Input pin configuration
  pinMode(THERMI1_PIN,INPUT);
  pinMode(THERMI2_PIN,INPUT);
  pinMode(THERMI3_PIN,INPUT);
  pinMode(THERMI4_PIN,INPUT);
  pinMode(PRESS_SENSOR_PIN, INPUT);
  pinMode(LEVEL_SENSOR1_PIN, INPUT);
  pinMode(LEVEL_SENSOR2_PIN, INPUT);
  pinMode(EMERGENCY_STOP_REG, INPUT);
  
  // Output pin configuration
  pinMode(PUMP_SPEED_PIN,OUTPUT);
  pinMode(PUMP_DIR_PIN,OUTPUT);
  pinMode(PUMP_POW_PIN,OUTPUT);
  pinMode(TANK1_PIN,OUTPUT);
  pinMode(SOL_HOT_PIN,OUTPUT);
  pinMode(THERMIS_POW_PIN,OUTPUT);

  // Default assignment of outputs
  digitalWrite(PUMP_SPEED_PIN,LOW);           // default pump speed pin on 0
  digitalWrite(PUMP_DIR_PIN,LOW);             // pump direction pin on 0
  digitalWrite(PUMP_POW_PIN,LOW);             // pump power OFF
  digitalWrite(TANK1_PIN,LOW);                // tank 1 OFF
  digitalWrite(SOL_HOT_PIN,LOW);              // hot water solenoid on 0
  digitalWrite(THERMIS_POW_PIN,HIGH);         // thermistor power OFF


  // Configuration and launching of the modbus server
  if (!ModbusRTUServer.begin(id.id, BAUD_RATE))      // parameters: ID=1, baudrate=9600, config=SERIAL_8N1 (8 bits, non parity, 1 bit stop)
  {
    Serial.println("Starting of the Modbus RTU server failed.");
    while (1);
  }


  // Modbus register configuration
  ModbusRTUServer.configureCoils(0x00, 80);
  ModbusRTUServer.configureDiscreteInputs(0x00, 32);
  ModbusRTUServer.configureInputRegisters(0x00, 48);
  ModbusRTUServer.configureHoldingRegisters(0x00, 32);

  // Default assignment of registers
  ModbusRTUServer.coilWrite(THERMIS_POW_REG,0);                         // thermistors - power: OFF
  ModbusRTUServer.coilWrite(LEVEL1_FLAG_REG,level1_flag);               // level 1 sensor management: sets by level1_flag
  ModbusRTUServer.coilWrite(LEVEL2_FLAG_REG,level2_flag);               // level 2 sensor management: sets by level2_flag
  ModbusRTUServer.coilWrite(PRESS_FLAG_REG,press_flag);                 // pressure sensor management: sets by press_flag
  ModbusRTUServer.coilWrite(PUMP_DIR_REG,0);                            // pump - direction: 0
  ModbusRTUServer.coilWrite(PUMP_POW_REG,0);                            // pump - power: OFF
  ModbusRTUServer.coilWrite(TANK1_REG,0);                               // tank 1: OFF
  ModbusRTUServer.coilWrite(SOL_HOT_REG,0);                             // solenoid hot water: 0
  ModbusRTUServer.coilWrite(BOOT_FLAG_REG,1);                           // starting flag: ON
  ModbusRTUServer.coilWrite(DEBUG_FLAG_REG,debug_flag);                 // debug flag: sets by debug_flag  
  ModbusRTUServer.inputRegisterWrite(GEN_STATE_REG,0);                  // general state: 0 (no problem)
  ModbusRTUServer.inputRegisterWrite(ERROR_CODE_REG,0);                 // error code: 0
  ModbusRTUServer.holdingRegisterWrite(ID_REG,id.id);                   // modbus ID
  ModbusRTUServer.holdingRegisterWrite(PUMP_SPEED_REG,0);               // pump - speed: 0
  ModbusRTUServer.holdingRegisterWrite(PUMP_SPEED_INC_REG,speed_step);  // pump - speed increasing/decreasing: 2000 Hz

  // Configuration to manage the pump speed
  pwm.setClockDivider(1,false);
  pwm.enable(1,true);

  // Updating of the starting cause variable
  uint8_t rcause = REG_PM_RCAUSE;

  if(rcause!=0x01 && rcause!=0x10 && rcause!=0x40) // if unwanted starting
  {
    ModbusRTUServer.inputRegisterWrite(GEN_STATE_REG,ModbusRTUServer.inputRegisterRead(GEN_STATE_REG)|1); // unwanted starting flag enable

    // Updating of the error register (if BOD12, BOD33 or WDT error)
    switch(rcause)
    {
      case 0x02: // BOD12 reset
        ModbusRTUServer.inputRegisterWrite(ERROR_CODE_REG,ModbusRTUServer.inputRegisterRead(ERROR_CODE_REG)|0x0001);
        break;
      case 0x04: // BOD33 reset
        ModbusRTUServer.inputRegisterWrite(ERROR_CODE_REG,ModbusRTUServer.inputRegisterRead(ERROR_CODE_REG)|0x0002);
        break;
      case 0x20: // WDT reset
        ModbusRTUServer.inputRegisterWrite(ERROR_CODE_REG,ModbusRTUServer.inputRegisterRead(ERROR_CODE_REG)|0x0004);
        break;
      default: // others
        ModbusRTUServer.inputRegisterWrite(ERROR_CODE_REG,ModbusRTUServer.inputRegisterRead(ERROR_CODE_REG)|0x8000);
        break;
    }
  }
  Serial.println("Setup done.");
}

void loop() {
  // Declaration and assignment of time variables
  uint32_t tps = millis();
  uint32_t interval1 = tps - time_ref1;               // for 1 s interval
  uint32_t interval2 = tps - time_ref2;               // for 35 ms interval 
  uint32_t requests = 0;   // number of requests received
    
  if(interval1>1000) // 1 s passed
  {
    // To display the modbus ID when debugging (if it has just been modified, this is display but a reboot is necessary to use this new ID)
    if (debug_flag)
    {
      Serial.print("\n");
      Serial.print("Slave ID = ");
      Serial.println(id.id);

      int freq = pwm.frequency(1);
      Serial.print(", Frq=");
      Serial.print(freq);
      
      Serial.print(", Nb.Req=");
      Serial.print(requests);
      
      Serial.println();
    }
    get_thermis();
    get_pressure();
    
    time_ref1 = tps;

    if (requests == 0) {
      // TODO: Implement a "security mode" to protect the machine if the master is not running for x? seconds --> no heating, no pumping
    }
    requests = 0;
  }

  if (interval2 > 35) // 35 ms passed
  {
    debug_flag = ModbusRTUServer.coilRead(DEBUG_FLAG_REG);
    speed_step = ModbusRTUServer.holdingRegisterRead(PUMP_SPEED_INC_REG);

    // An updated library is needed to receive an indication of requests treated...
    /*requests += */ModbusRTUServer.poll(); // scans if a command is coming from the master

    manage_emergency_stop();
    manage_id();
    manage_pump();
    manage_tanks();
    manage_valves();
    manage_levels();
    manage_pressure();

    time_ref2 = tps;
  }
  delay(1);
}

// Reads the emergency stop level
void manage_emergency_stop()
{
  uint8_t emergency_level = digitalRead(EMERGENCY_STOP_PIN);

  ModbusRTUServer.discreteInputWrite(EMERGENCY_STOP_REG,emergency_level);
}

// Reads and stores the level sensor values into the register
void manage_levels()
{
    level1_flag = ModbusRTUServer.coilRead(LEVEL1_FLAG_REG);
    pinMode(LEVEL_SENSOR1_PIN, level1_flag==1 ? INPUT_PULLUP : INPUT_PULLDOWN);
    uint8_t level_sensor1 = digitalRead(LEVEL_SENSOR1_PIN);
    ModbusRTUServer.discreteInputWrite(LEVEL_SENSOR1_REG,level_sensor1);

    // To debug
    if (debug_flag)
    {
      Serial.print("Level sensor 1 = ");
      Serial.print(level_sensor1);
      Serial.println(";\n");
    }

    level2_flag = ModbusRTUServer.coilRead(LEVEL2_FLAG_REG);
    pinMode(LEVEL_SENSOR2_PIN, level2_flag==1 ? INPUT_PULLUP : INPUT_PULLDOWN);
    uint8_t level_sensor2 = digitalRead(LEVEL_SENSOR2_PIN);
    
    ModbusRTUServer.discreteInputWrite(LEVEL_SENSOR2_REG,level_sensor2);
    // To debug
    if (debug_flag)
    {
      Serial.print("Level sensor 2 = ");
      Serial.print(level_sensor2);
      Serial.println(";\n");
    }
}

// Updates the solenoid outputs when a register is modified
void manage_valves()
{
  int8_t sol_hot = ModbusRTUServer.coilRead(SOL_HOT_REG);
  
  if(digitalRead(SOL_HOT_PIN)!=sol_hot)
  {
    if (debug_flag) {
      Serial.println("Solenoid 1 state modified");
    }
    digitalWrite(SOL_HOT_PIN,sol_hot);
  }
}

// Updates the tank outputs when a register is modified
void manage_tanks()
{
  int8_t tank1 = ModbusRTUServer.coilRead(TANK1_REG);
  
  if(digitalRead(TANK1_PIN)!=tank1)
  {
    if (debug_flag) {
      Serial.println("Tank 1 state modified");
    }
    digitalWrite(TANK1_PIN,tank1);
  }
}

uint32_t pressure = 0;             // to store the raw pressure values in the goal to compute an average
uint16_t pressure_min = 4096;      // to store the raw pressure values in the goal to compute an average
uint16_t pressure_max = 0;             // to store the raw pressure values in the goal to compute an average
uint16_t nb_pressures = 0;

// Updates the pressure with current value
void manage_pressure()
{      
    press_flag = ModbusRTUServer.coilRead(PRESS_FLAG_REG);
    if (press_flag)
    {
      uint16_t measure = analogRead(PRESS_SENSOR_PIN);
      pressure = pressure + measure;
      if (measure > 0 && measure < pressure_min) {
        pressure_min = measure;
      }
      if (measure > pressure_max) {
        pressure_max = measure;
      }
      nb_pressures = nb_pressures + 1 ;
    }
}

// Reads and stores the thermistor values into the registers (the storage value is an average of 3 successive values)
void get_thermis()
{
  int16_t thermis[4] = {0,0,0,0};             // to store the thermistor raw values in the goal to compute an average

  ModbusRTUServer.coilWrite(THERMIS_POW_REG,1);  // thermistor power ON
  digitalWrite(THERMIS_POW_PIN,!ModbusRTUServer.coilRead(THERMIS_POW_REG));
  
  for(int8_t i=0;i<3;i++) // reading 3 values
  {
    delay(10);
    
    thermis[0] = thermis[0] + analogRead(THERMI1_PIN);
    thermis[1] = thermis[1] + analogRead(THERMI2_PIN);
    thermis[2] = thermis[2] + analogRead(THERMI3_PIN);
    thermis[3] = thermis[3] + analogRead(THERMI4_PIN);
  }

  ModbusRTUServer.coilWrite(THERMIS_POW_REG,0);  // thermistor power OFF
  digitalWrite(THERMIS_POW_PIN,!ModbusRTUServer.coilRead(THERMIS_POW_REG));

  for(int8_t i=0;i<4;i++) // average of the 3 values
  {
    thermis[i] = thermis[i]/3;
  }

  // Sttoring the averages in the registers
  ModbusRTUServer.inputRegisterWrite(THERMI1_REG,thermis[0]);
  ModbusRTUServer.inputRegisterWrite(THERMI2_REG,thermis[1]);
  ModbusRTUServer.inputRegisterWrite(THERMI3_REG,thermis[2]);
  ModbusRTUServer.inputRegisterWrite(THERMI4_REG,thermis[3]);

  // To debug
  if (debug_flag) {
    for (int8_t i=0;i<4;i++)
    {
      Serial.print("Thermi");
      Serial.print(i+1);
      Serial.print(" = ");
      Serial.print(thermis[i]);
      Serial.print("; ");
    }
    Serial.println("\n");
  }
}

void get_pressure()
{
  press_flag = ModbusRTUServer.coilRead(PRESS_FLAG_REG);
  
  if(press_flag)  //if the monitoring of the pressure sensor is enable
  {
    if (debug_flag)
    {
      Serial.print("Pression (total) = ");
      Serial.print(pressure_min);
      Serial.print(" =< ");
      Serial.print(pressure);
      Serial.print(" / ");
      Serial.print(nb_pressures);
      Serial.print(" <= ");
      Serial.print(pressure_max);
      Serial.println(";\n");
    }
    if (nb_pressures > 0)
    {
        // To debug
        pressure = pressure / nb_pressures ;  // average of cumulated values
        // Storing the average in the register
        ModbusRTUServer.inputRegisterWrite(PRESS_SENSOR_REG,pressure);
        ModbusRTUServer.inputRegisterWrite(PRESS_MIN_SENSOR_REG,pressure_min);
        ModbusRTUServer.inputRegisterWrite(PRESS_MAX_SENSOR_REG,pressure_max);
        pressure = 0;
        pressure_min = 4096;
        pressure_max = 0;
        nb_pressures = 0;
    }
  }
}

// Updates the pump outputs when a register is modified
void manage_pump()
{
  int pump_speed = ModbusRTUServer.holdingRegisterRead(PUMP_SPEED_REG);
  int frequency_inc = ModbusRTUServer.holdingRegisterRead(PUMP_SPEED_INC_REG);
  int8_t pump_dir = ModbusRTUServer.coilRead(PUMP_DIR_REG);
  int8_t pump_pow = ModbusRTUServer.coilRead(PUMP_POW_REG);
  
  if(digitalRead(PUMP_POW_PIN)!=pump_pow)
  {
    if (debug_flag) {
      Serial.print("Pump power=");
      Serial.print(pump_pow);
      Serial.println(" (modified)");
    }
    pwm.timer(1,1,0xFFFFFF,false);   // Stops until resetted...
    pwm.analogWrite(PUMP_SPEED_PIN,0);
    digitalWrite(PUMP_POW_PIN,pump_pow);
    if (pump_pow == HIGH) // Normally CLOSE: if HIGH, current is stopped: do not continue!
    {
      return;
    }
  }

 int frequency_curr = 0;
 if (!stopped)
 {
  frequency_curr = pwm.frequency(1); // current frequency (compute by the pwm instance) 
 }

 if (digitalRead(PUMP_DIR_PIN)!=pump_dir)
  {
    if (debug_flag) {
      Serial.print("Pump direction=");
      Serial.print(pump_dir);
      Serial.println(" (modified)");
    }
    pump_speed = 0; // Stops before changing direction
    speed_flag = true;
  }

 if (frequency_curr!=pump_speed )
  {
    if (debug_flag) {
      Serial.print("Pump speed=");
      Serial.print(pump_speed);
      Serial.println(" (modified)");
    }
    speed_flag = true;
  }

 if(speed_flag)
  { 
    if(frequency_curr < pump_speed)  // if the speed must increase
    {
      frequency_curr += frequency_inc;

      if(frequency_curr>=pump_speed)
      {
        steps = calculSteps(pump_speed);      
        speed_flag = false;
      } else {
        steps = calculSteps(frequency_curr);      
      }
      pwm.timer(1,1,steps,false);
      pwm.analogWrite(PUMP_SPEED_PIN,500); //duty-cycle = 50%
      stopped = false;
    }
    else 
    {
      if(frequency_curr > pump_speed) { // if the speed must decrease
        frequency_curr -= frequency_inc;
      }
      if(frequency_curr <= 1000) // force to 0 Hz
      {
        speed_flag = false;
        stopped = true;
      } else
      {
        if(frequency_curr <= pump_speed) // the frequency can not go under the setpoint frequency
        {
          steps = calculSteps(pump_speed);
          speed_flag = false;
        } else {
          steps = calculSteps(frequency_curr);      
        }
        pwm.timer(1,1,steps,false);
        pwm.analogWrite(PUMP_SPEED_PIN,500); //duty-cycle = 50%
        stopped = false;
      }
    }
    if (stopped) {
      stop_pump();
    }
  }
}

void stop_pump()
{
  pwm.timer(1,1,0xFFFFFF,false);
  pwm.analogWrite(PUMP_SPEED_PIN,0);
  digitalWrite(PUMP_DIR_PIN,ModbusRTUServer.coilRead(PUMP_DIR_REG));
}

// Updates the modbus ID in the flash memory only when modified in the register
void manage_id()
{
  if(id.id != ModbusRTUServer.holdingRegisterRead(ID_REG))
  {
    id.id = ModbusRTUServer.holdingRegisterRead(ID_REG);
    id.write_number++;
    ID_FLASH.write(id);
    
    Serial.print("\n");
    Serial.println("Writing modbus ID in the flash memory");
    Serial.print("New ID = ");
    Serial.print(id.id);
    Serial.print(" ; Write number = ");
    Serial.println(id.write_number);
    Serial.print("\n");
  }
}

// Returns the number of steps  according to the frequency
unsigned long long int calculSteps(int f)
{
  if(f<=0)
    return 0;
    
  return VARIANT_MCK/(2*f); //VARIANT_MCK = 48MHz
}
