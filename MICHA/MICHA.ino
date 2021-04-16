// MICHA project
// Slave modbus code
// Board type: Arduino MKR Zero
// Components of the pastorizator managed:
//    - pump
//    - 2 heating taaks
//    - 2 valves
//    - 2 solenoids
//    - 4 thermistors
//
// WARNING: please verify the output states match with your hardware states (is it the same logic?)
//
// Version notes:
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
//          - pinassignment updated: SOL_COLD_PIN and VANNE_EVAC1_PIN switched
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
#include "MICHA_configuration.h"    // register configuration and pin assignment file

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
unsigned long long int steps = 0;
boolean speed_flag = 0;

// variables diverses
uint32_t time_ref1 = 0;           // thermistor reading reference time
uint32_t time_ref2 = 0;           // reference time for other operations
uint32_t time_ref_ledOn = 0;      // reference time for pump error signal reading
uint32_t time_ref4 = 0;           // reference time for full pump error signal
StructID id;                      // stores the ID for the flash memory
uint32_t flip, flop = 0;
boolean currServo = false;
uint8_t pump_err_count = 0;       // to store the pulse number count (pump error signal)
boolean pump_err_flag = false;    // indicates if a pump erro is occuring
boolean pump_err_prevState = 0;   // previous state of the pump error signal


void setup()
{
  Serial.begin(9600);
  
  
  // Storing the modbus ID of the flash memory
  id = ID_FLASH.read();

  if (id.init == false)   // if the ID was not yet initialized
  {
    id.id = ID; // using of the default ID (1)
    id.write_number = 0;  // initialized of the write number since the last flashing
    id.init = true;

    ID_FLASH.write(id);   // writing in flash memory

    Serial.print("First start. The modbus ID is ");
    Serial.println(id.id);
    Serial.print("\n");
  }else
  {
    Serial.print("The modbus ID is ");
    Serial.println(id.id);
    Serial.print("\n");
  }
  
  // Main configuration
  analogReadResolution(12);       // setting in 12 bit mode (10 bit mode by default)
  analogReference(AR_EXTERNAL);   // using of an external voltage as AREF reference
  
  // Input pin configuration
  pinMode(THERMI1_PIN,INPUT);
  pinMode(THERMI2_PIN,INPUT);
  pinMode(THERMI3_PIN,INPUT);
  pinMode(THERMI4_PIN,INPUT);
  pinMode(PUMP_ERR_PIN,INPUT);
  pinMode(PUMP_SERVO_PIN,INPUT_PULLUP);
  
  // Output pin configuration
  pinMode(PUMP_SPEED_PIN,OUTPUT);
  pinMode(PUMP_DIR_PIN,OUTPUT);
  pinMode(PUMP_POW_PIN,OUTPUT);
  pinMode(TANK1_PIN,OUTPUT);
  pinMode(TANK2_PIN,OUTPUT);
  pinMode(SOL_HOT_PIN,OUTPUT);
  pinMode(SOL_COLD_PIN,OUTPUT);
  pinMode(VALVE1_POW_PIN,OUTPUT);
  pinMode(VALVE1_DIR_PIN,OUTPUT);
  pinMode(VALVE2_POW_PIN,OUTPUT);
  pinMode(VALVE2_DIR_PIN,OUTPUT);
  pinMode(THERMIS_POW_PIN,OUTPUT);

  // Default assignment of outputs
  digitalWrite(PUMP_SPEED_PIN,LOW);           // default pump speed pin on 0
  digitalWrite(PUMP_DIR_PIN,LOW);             // pump direction pin on 0
  digitalWrite(PUMP_POW_PIN,LOW);             // pump power OFF
  digitalWrite(TANK1_PIN,LOW);                // tank 1 OFF
  digitalWrite(TANK2_PIN,LOW);                // tank 2 OFF
  digitalWrite(SOL_HOT_PIN,LOW);              // hot water solenoid on 0
  digitalWrite(SOL_COLD_PIN,LOW);             // cold water solenoid on 0
  digitalWrite(VALVE1_POW_PIN,LOW);           // valve 1 power on 0
  digitalWrite(VALVE1_DIR_PIN,LOW);           // valev 1 direction on 0
  digitalWrite(VALVE2_POW_PIN,LOW);           // valve 2 power on 0
  digitalWrite(VALVE2_DIR_PIN,LOW);           // valve 2 direction on 0
  digitalWrite(THERMIS_POW_PIN,HIGH);         // thermistor power OFF


  // Configuration and launching of the modbus server
  if (!ModbusRTUServer.begin(id.id, 9600))      // parameters: ID=1, baudrate=9600, config=SERIAL_8N1 (8 bits, non parity, 1 bit stop)
  {
    Serial.println("Starting of the Modbus RTU server failed.");
    while (1);
  }


  // Modbus register configuration
  ModbusRTUServer.configureCoils(0x00, 64);
  ModbusRTUServer.configureInputRegisters(0x00, 48);
  ModbusRTUServer.configureHoldingRegisters(0x00, 32);

  // Default assignment of registers
  ModbusRTUServer.coilWrite(BOOT_FLAG_REG,1);                     // starting flag: ON
  ModbusRTUServer.coilWrite(THERMIS_POW_REG,0);                   // thermistors - power: OFF
  ModbusRTUServer.coilWrite(PUMP_DIR_REG,0);                      // pump - direction: 0
  ModbusRTUServer.coilWrite(PUMP_POW_REG,0);                      // pump - power: OFF
  ModbusRTUServer.coilWrite(TANK1_REG,0);                         // tank 1: OFF
  ModbusRTUServer.coilWrite(TANK2_REG,0);                         // tank 2: OFF
  ModbusRTUServer.coilWrite(SOL_HOT_REG,0);                       // solenoid hot water: 0
  ModbusRTUServer.coilWrite(SOL_COLD_REG,0);                      // solenoid cold water: 0
  ModbusRTUServer.coilWrite(VALVE1_POW_REG,0);                    // valve 1 - power: OFF
  ModbusRTUServer.coilWrite(VALVE1_DIR_REG,0);                    // valve 1 - direction: 0
  ModbusRTUServer.coilWrite(VALVE2_POW_REG,0);                    // valve 2 - power: OFF
  ModbusRTUServer.coilWrite(VALVE2_DIR_REG,0);                    // valve 2 - direction: 0
  ModbusRTUServer.inputRegisterWrite(GEN_STATE_REG,0);            // general state: 0 (no problem)
  ModbusRTUServer.inputRegisterWrite(ERROR_CODE_REG,0);           // error code: 0
  ModbusRTUServer.holdingRegisterWrite(ID_REG,id.id);             // modbus ID
  ModbusRTUServer.holdingRegisterWrite(PUMP_SPEED_REG,0);         // pump - speed: 0
  ModbusRTUServer.holdingRegisterWrite(PUMP_SPEED_INC_REG,2000);  // pump - speed increasing/decreasing: 2000 Hz

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
      default: // autre
        ModbusRTUServer.inputRegisterWrite(ERROR_CODE_REG,ModbusRTUServer.inputRegisterRead(ERROR_CODE_REG)|0x8000);
        break;
    }
  }
}

void loop() {
  // Declaration and assignment of time variables
  uint32_t tps = millis();
  uint32_t interval1 = tps - time_ref1;               // for 1 s interval
  uint32_t interval2 = tps - time_ref2;               // for 35 ms interval 
  uint32_t interval4 = tps - time_ref4;               // for 1.8 s interval
  uint32_t pump_error_ledOn = tps - time_ref_ledOn;   // to compute the led light time of the pump error code

  // Other variables
  boolean highServo = HIGH == digitalRead(PUMP_SERVO_PIN);  // reading the pump servo signal (interrupts would be way better!)
  
  if (currServo) {
    if (!highServo) {
      flop++;
      currServo = false;
    }
  } else {
    if (highServo) {
      flip++;
      currServo = true;
    }    
  }
    
  if(interval1>1000) // 1 s passed
  {
    // To display the modbus ID when debugging (if it has just been modified, this is display but a reboot is necessary to use this new ID)
    Serial.print("\n");
    Serial.print("Slave ID = ");
    Serial.println(id.id);
    
    get_thermis();
    get_pumpSignals(flip,flop);
    flip = flop = 0;
    time_ref1 = tps;
  }

  if (interval2 > 35) // 35 ms passed
  {
    ModbusRTUServer.poll(); // scans if a command is coming from the master

    poll_pumpError(tps,pump_error_ledOn,interval4);  // polls the pump error pin
    manage_id();
    manage_pump();
    manage_tanks();
    manage_valves();

    time_ref2 = tps;
  }
}

// Polls the pump error pin
void poll_pumpError(uint32_t tps, uint32_t pump_error_ledOn, uint32_t interval4)
{
  boolean pump_err_state = digitalRead(PUMP_ERR_PIN);  // reading the pump error signal
  static uint32_t time_pump_err_ledOn = 0;  // stores the light length of time of the pump error led
  static boolean timeFlag_ledOn = true; // indicates when stop the light time measure of pump error led

  // Reading the pump error signal
  if(pump_err_state!=pump_err_prevState)  // when no error the pump error signal is LOW, otherwise HIGH
  {
    pump_err_prevState = pump_err_state;

    if(pump_err_state)
    { 
      if(pump_err_flag==false)  // if beginning of the pump error code flow, initialization of the timers and the counter
      {
        interval4 = 0;
        time_ref4 = tps;
        pump_err_count = 1;
        timeFlag_ledOn = true;
      }else
      {
        pump_err_count++;
      }

      time_ref_ledOn = tps;
      pump_err_flag = true;
    }else
    {
      if(timeFlag_ledOn)
      {
        time_pump_err_ledOn = pump_error_ledOn;
        timeFlag_ledOn = false;
      }
    }
  }

  if(pump_err_flag && (interval4 > 1700))  // a pump error is occuring and 1.8 s passed (period of a full pump error signal)
  {
    switch(pump_err_count)
    {
      case 2:
        if(time_pump_err_ledOn < 105)
        {
          Serial.println("Out of tolerance (2 fast pulses)");
        }else
        {
          Serial.println("AD Midpoint sampling abnormal (2 slow pulses)");
        }
        break;
      case 3:
        Serial.println("Motor wire or encoder wire is not connected (3 slow pulses)");
        break;
      case 4:
        if(time_pump_err_ledOn < 105)
        {
          Serial.println("The motor is overloaded for a long time (4 fast pulses)");
        }else
        {
          Serial.println("Undervoltage - voltage<20V (4 slow pulses)");
        }
        break;
      case 5:
        Serial.println("Overvoltage - voltage>90V (5 slow pulses)");
        break;
      default:
        Serial.println("Pump error other");
        break;
    }

    pump_err_count = 0;
    pump_err_flag = false;
    time_ref_ledOn = tps;
    time_ref4 = tps;
  }
}

// Updates the valve and solenoid outputs when a register is modified
void manage_valves()
{
  int8_t sol_hot = ModbusRTUServer.coilRead(SOL_HOT_REG);
  int8_t sol_cold = ModbusRTUServer.coilRead(SOL_COLD_REG);
  int8_t valve1_pow = ModbusRTUServer.coilRead(VALVE1_POW_REG);
  int8_t valve1_dir = ModbusRTUServer.coilRead(VALVE1_DIR_REG);
  int8_t valve2_pow = ModbusRTUServer.coilRead(VALVE2_POW_REG);
  int8_t valve2_dir = ModbusRTUServer.coilRead(VALVE2_DIR_REG);
  
  if(digitalRead(SOL_HOT_PIN)!=sol_hot)
  {
    Serial.println("Solenoid 1 state modified");
    digitalWrite(SOL_HOT_PIN,sol_hot);
  }

  if(digitalRead(SOL_COLD_PIN)!=sol_cold)
  {
    Serial.println("Solenoid 2 state modified");
    digitalWrite(SOL_COLD_PIN,sol_cold);
  }

  if(digitalRead(VALVE1_POW_PIN)!=valve1_pow)
  {
    Serial.println("Valve 1 power state modified");
    digitalWrite(VALVE1_POW_PIN,valve1_pow);
  }

  if(digitalRead(VALVE1_DIR_PIN)!=valve1_dir)
  {
    Serial.println("Valve 1 direction state modified");
    digitalWrite(VALVE1_DIR_PIN,valve1_dir);
  }

  if(digitalRead(VALVE2_POW_PIN)!=valve2_pow)
  {
    Serial.println("Valve 2 power state modified");
    digitalWrite(VALVE2_POW_PIN,valve2_pow);
  }

  if(digitalRead(VALVE2_DIR_PIN)!=valve2_dir)
  {
    Serial.println("Valve 2 direction state modified");
    digitalWrite(VALVE2_DIR_PIN,valve2_dir);
  }
}

// Updates the tank outputs when a register is modified
void manage_tanks()
{
  int8_t tank1 = ModbusRTUServer.coilRead(TANK1_REG);
  int8_t tank2 = ModbusRTUServer.coilRead(TANK2_REG);
  
  if(digitalRead(TANK1_PIN)!=tank1)
  {
    Serial.println("Tank 1 state modified");
    digitalWrite(TANK1_PIN,tank1);
  }

  if(digitalRead(TANK2_PIN)!=tank2)
  {
    Serial.println("Tank 2 state modified");
    digitalWrite(TANK2_PIN,tank2);
  }
}

// Reads and stores the thermistor values into the registers (the storage value is an average of 3 successive values)
void get_thermis()
{
  int16_t thermis[4] = {0,0,0,0};             // to store the thermistor values in the goal to compute an average

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

// Updates the pump outputs when a register is modified
void manage_pump()
{
  uint16_t pump_speed = ModbusRTUServer.holdingRegisterRead(PUMP_SPEED_REG);
  static uint16_t pump_speed_prev = 0;
  static int frequency_curr = 0; // current frequency (compute by the pwm instance)
  int16_t frequency_inc = ModbusRTUServer.holdingRegisterRead(PUMP_SPEED_INC_REG);
  int8_t pump_dir = ModbusRTUServer.coilRead(PUMP_DIR_REG);
  int8_t pump_pow = ModbusRTUServer.coilRead(PUMP_POW_REG);
  
  if(pump_speed_prev!=pump_speed)
  {
    Serial.println("Pump speed modified");

    steps = calculSteps(pump_speed);
    speed_flag = 1;
    
    pump_speed_prev = pump_speed;
  }

  if(speed_flag)
  { 
    frequency_curr = pwm.frequency(1);
    
    if(frequency_curr < pump_speed)  // if the speed must increase
    {
      frequency_curr += frequency_inc;
      steps = calculSteps(frequency_curr);

      if(frequency_curr>=pump_speed)
      {
        steps = calculSteps(pump_speed);
        pwm.timer(1,1,steps,false);
        pwm.analogWrite(PUMP_SPEED_PIN,500); //duty-cycle = 50%
      
        speed_flag = 0;
      }else
      {
        pwm.timer(1,1,steps,false);
        pwm.analogWrite(PUMP_SPEED_PIN,500); //duty-cycle = 50%
      }
    }
    else if(frequency_curr > pump_speed) // if the speed must decrease
    {
      frequency_curr -= frequency_inc;
      steps = calculSteps(frequency_curr);

      if(frequency_curr <= 100) // force to 0 Hz
      {
        pwm.timer(1,1,0xFFFFFF,false);
        pwm.analogWrite(PUMP_SPEED_PIN,0);
        speed_flag = 0;
      }else
      {
        if(frequency_curr <= pump_speed) // the freqyuency can not go under the setpoint frequency
        {
          steps = calculSteps(pump_speed);
          pwm.timer(1,1,steps,false);
          pwm.analogWrite(PUMP_SPEED_PIN,500); //duty-cycle = 50%

          speed_flag = 0;
        }else
        {
          pwm.timer(1,1,steps,false);
          pwm.analogWrite(PUMP_SPEED_PIN,500); //duty-cycle = 50%
        }
      }
    }
  }

  if(digitalRead(PUMP_DIR_PIN)!=pump_dir)
  {
    Serial.println("Pump direction modified");
    digitalWrite(PUMP_DIR_PIN,pump_dir);
  }

  if(digitalRead(PUMP_POW_PIN)!=pump_pow)
  {
    Serial.println("Pump power modified");
    digitalWrite(PUMP_POW_PIN,pump_pow);
  }
}

// Updates the servo and error registers 
void get_pumpSignals(uint32_t flip, uint32_t flop)
{
  ModbusRTUServer.inputRegisterWrite(PUMP_SERVO_REG, flip > flop ? flip : flop);
  
  uint32_t pompe_e_us = pulseIn(PUMP_ERR_PIN,HIGH,16); // retourne la durée de l'impulsion HAUTE PWM en µs (attend l'impulsion pendant 16 µs)  
  ModbusRTUServer.inputRegisterWrite(PUMP_ERR_REG,pompe_e_us);
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
