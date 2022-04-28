#!/usr/bin/python3
# -*- coding: utf-8 -*-

# This code allows to communicate with the MICHA board in a pastorizator configuration. It can be use as:
#     - a library to communicate with the MICHA board using another main code file
#     - a standalone code to test the I/O of the MICHA board

import traceback
from serial import Serial, PARITY_NONE
from umodbus.client.serial import rtu
import Odroid.GPIO as GPIO
import time

SLAVE_ID = 1

VOLTAGE_REF = 2.497 # value of the excitement voltage reference
THERMI_WIRE = 0.6 # value of the twin wire resistor

# Odroid configuration
GPIO.setmode(GPIO.BOARD)
# pin used
START_BTN_PIN               = 15    # pin on which the start button is connected
PAUSE_BTN_PIN               = 13    # pin on which the pause button is connected
STOP_BTN_PIN                = 33    # pin on which the start button is connected
START_LED_PIN               = 16    # pin on which the start led is connected
PAUSE_LED_PIN               = 18    # pin on which the pause led is connected
STATE_LED_PIN               = 36     # pin on which the state led is connected
BUZZER_PIN                  = 35    # pin on which the buzzer is connected
# pin configuration
GPIO.setup(START_BTN_PIN, GPIO.IN)
GPIO.setup(PAUSE_BTN_PIN, GPIO.IN)
GPIO.setup(STOP_BTN_PIN, GPIO.IN)
GPIO.pinMode(STOP_BTN_PIN, GPIO.PUD_UP)
GPIO.setup(START_LED_PIN, GPIO.OUT)
GPIO.setup(PAUSE_LED_PIN, GPIO.OUT)
GPIO.setup(STATE_LED_PIN, GPIO.OUT)
GPIO.setup(BUZZER_PIN, GPIO.OUT)

# Registre configuration
# coils
THERMIS_POW_REG             = 0x00  # register which stores the thermistor power state
LEVEL1_FLAG_REG             = 0x01  # register which stores the flag which enables/disables the level 1 sensor management
LEVEL2_FLAG_REG             = 0x02  # register which stores the flag which enables/disables the level 2 sensor management
PRESS_FLAG_REG              = 0x03  # register which stores the flag which enables/disables the pressure sensor management
PUMP_DIR_REG                = 0x10  # register which stores the pump direction
PUMP_POW_REG                = 0x11  # register which stores the pump power state
TANK1_REG                   = 0x20  # register which stores the tank 1 state
SOL_HOT_REG                 = 0x30  # register which stores the hot water solenoid state
BOOT_FLAG_REG               = 0x40  # register which stores the boot state
DEBUG_FLAG_REG              = 0x41  # register which stores the state of the debug mode
# discrete registers
LEVEL_SENSOR1_REG           = 0x01  # register which stores the state of the input level sensor (1 for water)
LEVEL_SENSOR2_REG           = 0x02  # register which stores the state of the output level sensor (1 for water)
EMERGENCY_STOP_REG          = 0x10  # register which stores the state of  the emergency stop button (0 for active emergency stop)
# input registers
GEN_STATE_REG               = 0x00  # register which stores the general state of the system
THERMI1_REG                 = 0x01  # register which stores the thermistor 1 value (0 - 4095)
THERMI2_REG                 = 0x02  # register which stores the thermistor 2 value (0 - 4095)
THERMI3_REG                 = 0x03  # register which stores the thermistor 3 value (0 - 4095)
THERMI4_REG                 = 0x04  # register which stores the thermistor 4 value (0 - 4095)
PRESS_SENSOR_REG            = 0x10  # register which stores the pressure sensor value (0 (low pressure) - 4095 (high pressure))
ERROR_CODE_REG              = 0x20  # register which stores the general error codes
# holding registers
ID_REG                      = 0x00  # register which stores the modbus ID
PUMP_SPEED_REG              = 0x10  # register which stores the pump speed
PUMP_SPEED_INC_REG          = 0x11  # register which stores the increasing/decreasing value of the pump frequency


# Class to manage the MICHA board
class Micha:
    def __init__(self,device='/dev/ttyS1'):	# serial0 for RPi, ttyS1 for Odroid
        self.device = device
        self.id = 0
        self.boot_flag = 1
        self.thermi = 0
        self.thermis_pow = 0
        self.pump_speed = 0
        self.pump_speed_inc = 0
        self.pump_dir = 0
        self.pump_power = 0
        self.tank1 = 0
        self.sol_hot = 0
        self.general_state = 0
        self.error_code = 0
        self.debug_flag = 0
        self.level = 0
        self.level1_flag = 0
        self.level2_flag = 0
        self.press_flag = 0
        self.busy = False
        self.port = None
    
    # Configuration and starting of the modbus communication
    def get_serial_port(self):
        while self.busy:
            time.sleep(0.1)
        self.busy = True
        while not self.port: # In case of error, we reset the access to the ModBus
            try:
                """Return a serial.Serial instance which is ready to use with a RS485 adaptor."""
                self.port = Serial(port=self.device, baudrate=19200, parity=PARITY_NONE, stopbits=1, bytesize=8, timeout=1)
            except:
                traceback.print_exc()
                self.port = None
                time.sleep(1) # Do not retry too fast...
        return self.port

    def close_serial_port(self):
        if self.port:
            try:
                self.port.close()
            except:
                pass
            self.port = None
        self.busy = False

    def release_serial_port(self):
        self.busy = False
        
    def read_pin(self,reg): # read a single coil register at reg address 
        try:
            serial_port = self.get_serial_port()
            message = rtu.read_coils(SLAVE_ID, reg, 1)
            response = rtu.send_message(message, serial_port)
            response = response[0]
            self.release_serial_port()
            return response
        except:
            traceback.print_exc()
            self.close_serial_port()
        return None

    def write_pin(self,reg,val): # write val in a single coil register at reg address
        try:
            serial_port = self.get_serial_port()
            
            message = rtu.write_single_coil(SLAVE_ID, reg, val)
            response = rtu.send_message(message, serial_port)
            self.release_serial_port()
            return response
        except:
            traceback.print_exc()
            self.close_serial_port()
        return None

    def read_discrete(self,reg): # read a discrete input register at reg address
        try:
            serial_port = self.get_serial_port()
            message = rtu.read_discrete_inputs(SLAVE_ID, reg, 1)
            response = rtu.send_message(message, serial_port)
            self.release_serial_port()
            return response[0]
        except:
            traceback.print_exc()
            self.close_serial_port()
        return None

    def read_input(self,reg): # read a single input register at reg address
        try:
            serial_port = self.get_serial_port()
            message = rtu.read_input_registers(SLAVE_ID, reg, 1)
            response = rtu.send_message(message, serial_port)
            self.release_serial_port()
            return response[0]
        except:
            traceback.print_exc()
            self.close_serial_port()
        return None

    def read_holding(self,reg): # read a single holding register at reg address
        try:
            serial_port = self.get_serial_port()
            message = rtu.read_holding_registers(SLAVE_ID, reg, 1)
            response = rtu.send_message(message, serial_port)
            self.release_serial_port()
            return response[0]
        except:
            traceback.print_exc()
            self.close_serial_port()
        return None

    def write_holding(self,reg, val): # write val in the holding register at reg address
        try:
            serial_port = self.get_serial_port()            
            message = rtu.write_single_register(SLAVE_ID, reg, val)
            response = rtu.send_message(message, serial_port)
            self.release_serial_port()
            return response
        except:
            traceback.print_exc()
            self.close_serial_port()
        return None
    
    def get_id(self): # to get the modbus ID
        self.id = self.read_holding(ID_REG)
        return self.id

    def get_boot_flag(self): # to get the boot state
        self.boot_flag = self.read_pin(BOOT_FLAG_REG)
        return self.boot_flag
    
    def set_boot_flag(self,flag=0): # to set the boot state
        if self.boot_flag != flag:
            self.boot_flag = flag
            response = self.write_pin(BOOT_FLAG_REG, flag)
            return response
        return 0
    
    def get_thermis_pow(self): # to get the power state of the thermistors (stored in the register), returns the thermistors power state
        self.thermis_pow = self.read_pin(THERMIS_POW_REG)
        return self.thermis_pow
    
    def set_thermis_pow(self,power=0): # to set the power of the thermistors
        if self.thermis_pow != power:
            self.thermis_pow = power
            response = self.write_pin(THERMIS_POW_REG, power)
            return response
        return 0
    
    def get_thermi(self, th=0): # to get the thermistor value, returns the thermistor value
        self.thermi = th
        
        try:
            serial_port = self.get_serial_port()
            
            if self.thermi==0: # get the value of all the thermistors
                message = rtu.read_input_registers(SLAVE_ID, THERMI1_REG, 4)
            elif self.thermi==1: # get the thermistor 1 value
                message = rtu.read_input_registers(SLAVE_ID, THERMI1_REG, 1)
            elif self.thermi==2: # get the thermistor 2 value
                message = rtu.read_input_registers(SLAVE_ID, THERMI2_REG, 1)
            elif self.thermi==3: # get the thermistor 3 value
                message = rtu.read_input_registers(SLAVE_ID, THERMI3_REG, 1)
            elif self.thermi==4: # get the thermistor 4 value
                message = rtu.read_input_registers(SLAVE_ID, THERMI4_REG, 1)
            else:
                print("ERROR: no thermistor was found at this value")
            
            response = rtu.send_message(message, serial_port)
            
            self.close_serial_port()
        except:
            traceback.print_exc()
    
        return response
    
    def set_pump_power(self,power=0): # to set the power of the pump
        if self.pump_power != power:
            self.pump_power = power
            response = self.write_pin(PUMP_POW_REG, power)
            return response
        return 0

    def set_pump_speed(self,speed=0): # to set the speed of the pump
        if self.pump_speed!=speed:
            self.pump_speed = speed
            return self.write_holding(PUMP_SPEED_REG, speed)
        return 0
    
    def set_pump_speed_inc(self,inc=0): # to set the speed incrementation of the pump
        if self.pump_speed_inc!=inc:
            self.pump_speed_inc = inc
            return self.write_holding(PUMP_SPEED_INC_REG, inc)
        return 0
    
    def set_pump_dir(self,dir=0): # to set the direction of the pump
        if self.pump_dir!=dir:
            self.pump_dir = dir
            response = self.write_pin(PUMP_DIR_REG, dir)
            return response
        return 0
    
    def get_pump_power(self): # to get the power state of the pump (stored in the register), returns the pump power state
        self.pump_power = self.read_pin(PUMP_POW_REG)
        return self.pump_power
    
    def get_pump_dir(self): # to get the direction state of the pump (stored in the register), returns the pump direction value
        self.pump_dir = self.read_pin(PUMP_DIR_REG)
        return self.pump_dir
    
    def get_pump_speed(self): # to get the speed of the pump (stored in the register), returns the pump speed
        self.pump_speed = self.read_holding(PUMP_SPEED_REG)
        return self.pump_speed
    
    def get_pump_speed_inc(self): # to get the speed incrementation of the pump (stored in the register), returns the pump speed incrementation
        self.pump_speed_inc = self.read_holding(PUMP_SPEED_INC_REG)
        return self.pump_speed_inc
    
    def set_tank1(self,state=0): # to set the state of the tank 1
        if self.tank1 != state:
            self.tank1 = state
            response = self.write_pin(TANK1_REG, state)
            return response
        return 0
    
    def get_tank1(self): # to get the state of the tank 1 (stored in the register)
        self.tank1 = self.read_pin(TANK1_REG)
        return self.tank1
    
    def set_sol_hot(self,state=0): # to set the state of the hot water solenoid
        if self.sol_hot != state:
            self.sol_hot = state
            response = self.write_pin(SOL_HOT_REG, state)
            return response
        return 0
    
    def get_sol_hot(self): # to get the state of the hot water solenoid (stored in the register)
        self.sol_hot = self.read_pin(SOL_HOT_REG)
        return self.sol_hot

    def get_level1_sensor(self): # to get the level 1 sensor value, returns the level 1 sensor value
        return self.read_discrete(LEVEL_SENSOR1_REG)

    def set_level1_flag(self,state=0): # to set the level 1 sensor flag value
        if self.level1_flag != state:
            self.level1_flag = state
            response = self.write_pin(LEVEL1_FLAG_REG, state)
            return response
        return 0

    def get_level1_flag(self): # to get the level 1 sensor flag value, returns the level 1 sensor flag value
        return self.read_pin(LEVEL1_FLAG_REG)

    def get_level2_sensor(self): # to get the level 2 sensor value, returns the level 2 sensor value
        return self.read_discrete(LEVEL_SENSOR2_REG)

    def set_level2_flag(self,state=0): # to set the level 2 sensor flag value
        if self.level2_flag != state:
            self.level2_flag = state
            response = self.write_pin(LEVEL2_FLAG_REG, state)
            return response
        return 0

    def get_level2_flag(self): # to get the level 2 sensor flag value, returns the level 2 sensor flag value
        return self.read_pin(LEVEL2_FLAG_REG)

    def get_press_sensor(self): # to get the pressure sensor value, returns the pressure sensor value
        return self.read_input(PRESS_SENSOR_REG)

    def set_press_flag(self, state=0):  # to set the pressure sensor flag value
        if self.press_flag != state:
            self.press_flag = state
            response = self.write_pin(PRESS_FLAG_REG, state)
            return response
        return 0

    def get_press_flag(self):  # to get the pressure sensor flag value, returns the pressure sensor flag value
        return self.read_pin(PRESS_FLAG_REG)

    def get_emergency_stop(self): # to get the emergency stop value, returns the emergency stop value
        return self.read_discrete(EMERGENCY_STOP_REG)

    def get_general_state(self): # to get the general state of the system (stored in the register)
        self.general_state = self.read_input(GEN_STATE_REG)
        return self.general_state
    
    def get_error_code(self): # to get the general error code
        self.error_code = self.read_input(ERROR_CODE_REG)
        return self.error_code

    def get_debug_flag(self):  # to get the debug state
        self.debug_flag = self.read_pin(DEBUG_FLAG_REG)
        return self.debug_flag

    def set_debug_flag(self, flag=0):  # to set the debug state
        if self.debug_flag != flag:
            self.debug_flag = flag
            response = self.write_pin(DEBUG_FLAG_REG, flag)
            return response
        return 0
        
    def get_startBtn(self): # to get the state of the start button
        return GPIO.digitalRead(START_BTN_PIN)
    
    def get_pauseBtn(self): # to get the state of the pause button
        return GPIO.digitalRead(PAUSE_BTN_PIN)
    
    def get_stopBtn(self): # to get the state of the start button
        return GPIO.digitalRead(STOP_BTN_PIN)
        
    def get_startLed(self):
        return GPIO.digitalRead(START_LED_PIN)
        
    def set_startLed(self, state=0):
        GPIO.output(START_LED_PIN, state)
        return 0
    
    def get_pauseLed(self):
        return GPIO.digitalRead(PAUSE_LED_PIN)
        
    def set_pauseLed(self, state=0):
        GPIO.output(PAUSE_LED_PIN, state)
        return 0
        
    def get_stateLed(self):
        return GPIO.digitalRead(STATE_LED_PIN)
        
    def set_stateLed(self, state=0):
        GPIO.output(STATE_LED_PIN, state)
        return 0
    def get_buzzer(self):
        return GPIO.digitalRead(BUZZER_PIN)
        
    def set_buzzer(self, state=0):
        GPIO.output(BUZZER_PIN, state)
        return 0
    
# test section
if __name__ == "__main__":
    
    def boot_monitoring():
        if pasto.get_boot_flag():
            print("\n\nTHE DEVICE HAS REBOOTED!")
            pasto.set_boot_flag()
        
        return 0
    
    def menu_choice(mini,maxi):
        choice = '-1'
        
        while (int(choice)<mini or int(choice)>maxi):
            choice = input("Choose an option: ")
            
            boot_monitoring()
                
            if (int(choice)<mini or int(choice)>maxi):
                print("ERROR: incorrect choice. Your choice must be [{}:{}]".format(mini,maxi))
        
        return choice
    
    # Allows to manage the thermistors
    def thermis(choice):
        """Display the value of the thermistors."""
                    
        if choice=='tps': # gets the current thermistors power pin state
            print("\nCurrent thermistor power pin state = {}\n".format(pasto.get_thermis_pow()))
        elif choice=='tp0': # sets the thermistors power pin state to 0
            pasto.set_thermis_pow(0)
            print('\nThermistor power pin state sets to 0 (OFF)\n')
        elif choice=='tp1': # sets the thermistors power pin state to 1
            pasto.set_thermis_pow(1)
            print('\nThermistor power pin state sets to 1 (ON)\n')
        elif choice=='ts':
            thermi1 = pasto.get_thermi(1)[0]
            thermi2 = pasto.get_thermi(2)[0]
            thermi3 = pasto.get_thermi(3)[0]
            thermi4 = pasto.get_thermi(4)[0]
            
            thermi1_mV = (VOLTAGE_REF*thermi1/4096)*1000
            thermi2_mV = (VOLTAGE_REF*thermi2/4096)*1000
            thermi3_mV = (VOLTAGE_REF*thermi3/4096)*1000
            thermi4_mV = (VOLTAGE_REF*thermi4/4096)*1000
            
            print('\n')
            i = 1
            for value in pasto.get_thermi():
                print("Thermistor {} = {} ({:4.3f} mV)".format(i,value,(VOLTAGE_REF*value/4096)*1000))
                i+=1
            print('\n')
        elif choice=='ts1':
            thermi1 = pasto.get_thermi(1)[0]
            thermi1_mV = (VOLTAGE_REF*thermi1/4096)*1000
            print("\nThermistor 1 = {} ({:4.3f} mV)\n".format(thermi1,thermi1_mV))
        elif choice=='ts2':
            thermi2 = pasto.get_thermi(2)[0]
            thermi2_mV = (VOLTAGE_REF*thermi2/4096)*1000
            print("\nThermistor 2 = {} ({:4.3f} mV)\n".format(thermi2,thermi2_mV))
        elif choice=='ts3':
            thermi3 = pasto.get_thermi(3)[0]
            thermi3_mV = (VOLTAGE_REF*thermi3/4096)*1000
            print("\nThermistor 3 = {} ({:4.3f} mV)\n".format(thermi3,thermi3_mV))
        elif choice=='ts4':
            thermi4 = pasto.get_thermi(4)[0]
            thermi4_mV = (VOLTAGE_REF*thermi4/4096)*1000
            print("\nThermistor 4 = {} ({:4.3f} mV)\n".format(thermi4,thermi4_mV))                
                        
        return 0

    # Allows to manage the pump
    def pump(choice):
        """Get or set a value related to the pump."""
        
        if choice=='ps': # gets all the current pump pin state
            print("\nCurrent pump power pin state = {}".format(pasto.get_pump_power()))
            print("Current pump direction pin state = {}".format(pasto.get_pump_dir()))
            print("Current pump speed = {}".format(pasto.get_pump_speed()))
            print("Current pump speed incrementation = {}\n".format(pasto.get_pump_speed_inc()))
        elif choice=='pps': # gets the current pump power pin state
            print("\nCurrent pump power pin state = {}\n".format(pasto.get_pump_power()))
        elif choice=='pp0': # sets the pump power pin state to 0
            pasto.set_pump_power(0)
            print('\nPump power pin state sets to 0 (OFF)\n')
        elif choice=='pp1': # sets the pump power pin state to 1
            pasto.set_pump_power(1)
            print('\nPump power pin state sets to 1 (ON)\n')
        elif choice=='pds': # gets the current pump direction pin state
            print("\nCurrent pump direction pin state = {}\n".format(pasto.get_pump_dir()))
        elif choice=='pd0': # sets the pump direction pin state to 0
            pasto.set_pump_dir(0)
            print('\nPump direction pin state sets to 0\n')
        elif choice=='pd1': # sets the pump direction pin state to 1
            pasto.set_pump_dir(1)
            print('\nPump direction pin state sets to 1\n')
        elif choice=='pss': # gets the current pump speed value
            print("\nCurrent pump speed = {}\n".format(pasto.get_pump_speed()))
        elif choice=='psis': # gets the current pump speed incrementation value
            print("\nCurrent pump speed incrementation = {}\n".format(pasto.get_pump_speed_inc()))
        
        return 0

    # Allows to manage the tank
    def tank(choice):
        """Get or set a value related to the pump."""
            
        if choice=='cps': # gets the current cistern pin state
            print("\nCurrent heating cistern power pin state = {}\n".format(pasto.get_tank1()))
        elif choice=='cp0': # sets the cistern pin state to 0
            pasto.set_tank1(0)
            print('\nHeating cistern power pin state sets to 0 (OFF)\n')
        elif choice=='cp1': # sets the cistern pin state to 1
            pasto.set_tank1(1)
            print('\nHeating cistern power pin state sets to 1 (ON)\n')
        
        return 0

    # Allows to manage the water solenoid valve
    def sol(choice):
        """Get or set a value related to the water solenoid valve."""
        
        if choice=='ss': # gets the current water solenoid pin state
            print("\nCurrent water solenoid pin state = {}\n".format(pasto.get_sol_hot()))
        elif choice=='s0': # sets the water solenoid pin state to 0
            pasto.set_sol_hot(0)
            print('\nWater solenoid pin state sets to 0 (CLOSED)\n')
        elif choice=='s1': # sets the water solenoid pin state to 1
            pasto.set_sol_hot(1)
            print('\nWater solenoid pin state sets to 1 (OPENED)\n')
                
        return 0
    
    def levelSensors(choice):
        """Get or set a value related to the level sensors."""
        
        if choice=='ls': # gets all the current level sensor values
            if pasto.get_level1_flag():
                print("\nCurrent level sensor 1 value = {}".format(pasto.get_level1_sensor()))
            else:
                print("\nCurrent level sensor 1 value = désactivé")
            if pasto.get_level2_flag():
                print("Current level sensor 2 value = {}".format(pasto.get_level2_sensor()))
            else:
                print("Current level sensor 2 value = désactivé")
        elif choice=='lfs': # gets all the current level sensor flag states
            print("\nLevel sensor 1 flag state = {}".format(pasto.get_level1_flag()))
            print("Level sensor 2 flag state = {}\n".format(pasto.get_level2_flag()))
        elif choice=='l1s': # gets the current level sensor 1 value
            print("\nCurrent level sensor 1 value = {}\n".format(pasto.get_level1_sensor()))
        elif choice=='lf1s': # gets the current level sensor 1 flag state
            print("\nLevel sensor 1 flag state = {}\n".format(pasto.get_level1_flag()))
        elif choice=='lf10': # sets the level sensor 1 flag state to 0
            pasto.set_level1_flag(0)
            print('\nLevel sensor 1 flag state sets to 0 (OFF)\n')
        elif choice=='lf11': # sets the level sensor 1 flag state to 1
            pasto.set_level1_flag(1)
            print('\nLevel sensor 1 flag state sets to 1 (ON)\n')
        elif choice=='l2s': # gets the current level sensor 2 value
            print("\nCurrent level sensor 2 value = {}\n".format(pasto.get_level2_sensor()))
        elif choice=='lf2s': # gets the level sensor 2 flag state
            print("\nLevel sensor 2 flag state = {}\n".format(pasto.get_level2_flag()))
        elif choice=='lf20': # gsts the level sensor 2 flag state to 0
            pasto.set_level2_flag(0)
            print('\nLevel sensor 2 flag state sets to 0 (OFF)\n')
        elif choice=='lf21': # sets the level sensor 2 flag state to 1
            pasto.set_level2_flag(1)
            print('\nLevel sensor 2 flag state sets to 1 (ON)\n')
        
        return 0
    
    def emergency_stop():
        """Get the emergency stop pin state"""
        
        print("\nCurrent emergency stop pin state = {}".format(pasto.get_emergency_stop()))
        
        return 0
        
    def pressSensor(choice):
        """Get or set a value related to the pressure sensor."""
        
        Vcc = 5 # power voltage applied to the pressure sensor
        pressure = pasto.get_press_sensor() # raw value of the pressure (0-4095)
        pressure_V = pressure/1638 # pressure in V
        pressure_psi = 125*(pressure_V/Vcc)-12.5 # pressure in Psi
        pressure_bar = pressure_psi/14.504 # pressure in bar
        
        
        if choice=='prs': # gets the current pressure sensor value
            if pasto.get_press_flag():
                print("\nCurrent pressure sensor value = {} ({:4.3f} mV, {:4.3f} bars)\n".format(pressure, pressure_V, pressure_bar))
            else:
                print("\nCurrent pressure sensor value = désactivé\n")
        elif choice=='prfs': # gets the current pressure sensor flag state
            print("\nLevel sensor 1 flag state = {}\n".format(pasto.get_press_flag()))
        elif choice=='prf0': # sets the pressure sensor flag state to 0
            pasto.set_press_flag(0)
            print('\nPressure sensor flag state sets to 0 (OFF)\n')
        elif choice=='prf1': # sets the pressure sensor flag state to 1
            pasto.set_press_flag(1)
            print('\nPressure sensor flag state sets to 1 (ON)\n')
        
        return 0

    def generalState(choice):
        """Get the error code and the general state of the system"""
        
        if choice=='id':
            print("\nModbus ID = {}\n".format(pasto.get_id()))
        elif choice=='bss':
            print("\nBoot state = {}\n".format(pasto.get_boot_flag()))
        elif choice=='dms':
            print("\nDebug mode = {}\n".format(pasto.get_debug_flag()))
        elif choice=='dm0':
            pasto.set_debug_flag(0)
            print('\nDebug mode sets to 0 (OFF)\n')
        elif choice=='dm1':
            pasto.set_debug_flag(1)
            print('\nDebug mode sets to 1 (ON)\n')
        elif choice=='gss':
            print("\nGeneral state = {}\n".format(pasto.get_general_state()))
        elif choice=='ecs':
            print("\nError code = {}\n".format(pasto.get_error_code()))

        return 0

    def registers():
        """Display the value of all the registers/states."""
        
        # State
        print("\nModbus ID \t\t= {}".format(pasto.get_id()))
        print("Boot state flag \t= {}".format(pasto.get_boot_flag()))
        print("Debug mode flag \t= {}".format(pasto.get_debug_flag()))
        print("General state \t\t= {}".format(pasto.get_general_state()))
        print("Error code \t\t= {}".format(pasto.get_error_code()))
        print("Emergency stop \t\t= {}".format(pasto.get_emergency_stop()))
        # Thermistors
        print("Thermistor power \t= {}".format(pasto.get_thermis_pow()))
        i = 1
        for value in pasto.get_thermi():
            print("Thermistor {} \t\t= {}".format(i,value))
            i+=1
        # Pump
        print("Pump power \t\t= {}".format(pasto.get_pump_power()))
        print("Pump direction \t\t= {}".format(pasto.get_pump_dir()))
        print("Pump speed \t\t= {}".format(pasto.get_pump_speed()))
        print("Pump speed increment \t= {}".format(pasto.get_pump_speed_inc()))
        # Tank
        print("Tank 1 power \t\t= {}".format(pasto.get_tank1()))
        # Water solenoid
        print("Water solenoid \t\t= {}".format(pasto.get_sol_hot()))
        # Level sensors
        print("Level sensor 1 flag \t= {}".format(pasto.get_level1_flag()))
        print("Level sensor 1 \t\t= {}".format(pasto.get_level1_sensor()))
        print("Level sensor 2 flag \t= {}".format(pasto.get_level2_flag()))
        print("Level sensor 2 \t\t= {}".format(pasto.get_level2_sensor()))
        # Pressure sensor
        print("Pressure sensor flag \t= {}".format(pasto.get_press_flag()))
        print("Pressure sensor \t= {}".format(pasto.get_press_sensor()))
        print("Start button pin state \t= {}".format(pasto.get_startBtn()))
        print("Pause button pin state \t= {}".format(pasto.get_pauseBtn()))
        print("Stop button pin state \t= {}".format(pasto.get_stopBtn()))
        print("Start led pin state \t= {}".format(pasto.get_startLed()))
        print("Pause led pin state \t= {}".format(pasto.get_pauseLed()))
        print("State led pin state \t= {}\n".format(pasto.get_stateLed()))
        
        
    def electricalPanel(choice):
        """Get or set a value related to the control panel (leds end buttons)."""
        
        if choice=='eps': # gets all the state of the control panel
            print("\nStart button pin state = {}".format(pasto.get_startBtn()))
            print("Pause button pin state = {}".format(pasto.get_pauseBtn()))
            print("Stop button pin state = {}".format(pasto.get_stopBtn()))
            print("Start led pin state = {}".format(pasto.get_startLed()))
            print("Pause led pin state = {}".format(pasto.get_pauseLed()))
            print("State led pin state = {}\n".format(pasto.get_stateLed()))
        elif choice=='epgbs': # gets the green button state (start button)
            print("\nStart button pin state = {}\n".format(pasto.get_startBtn()))
        elif choice=='epgls': # gets the green led state (start led)
            print("\nStart led pin state = {}\n".format(pasto.get_startLed()))
        elif choice=='epgl1': # sets the green led state to 1 (start led)
            pasto.set_startLed(1)
            print("\nStart led pin state sets to 1\n")
        elif choice=='epgl0': # sets the green led state to 0 (start led)
            pasto.set_startLed(0)
            print("\nStart led pin state sets to 0\n")
        elif choice=='epobs': # gets the orange button state (pause button)
            print("\nPause button pin state = {}\n".format(pasto.get_pauseBtn()))
        elif choice=='epols': # gets the orange led state (pause led)
            print("\nPause led pin state = {}\n".format(pasto.get_pauseLed()))
        elif choice=='epol1': # sets the orange led state to 1 (pause led)
            pasto.set_pauseLed(1)
            print("\nPause led pin state sets to 1\n")
        elif choice=='epol0': # sets the orange led state to 0 (pause led)
            pasto.set_pauseLed(0)
            print("\nPause led pin state sets to 0\n")
        elif choice=='eprbs': # gets the red button state (stop button)
            print("\nStop button pin state = {}\n".format(pasto.get_stopBtn()))
        elif choice=='eprls': # gets the red led state (state led)
            print("\nState led pin state = {}\n".format(pasto.get_stateLed()))
        elif choice=='eprl1': # sets the red led state to 1 (state led)
            pasto.set_stateLed(1)
            print("\nState led pin state sets to 1\n")
        elif choice=='eprl0': # sets the red led state to 0 (state led)
            pasto.set_stateLed(0)
        
        return 0
        
    def buzzer(choice):
        """Get the set a value related to the buzzer"""
        
        if choice=='bs': # gets the state of the buzzer pin
            print("\nBuzzer pin state = {}\n".format(pasto.get_buzzer()))
        elif choice=='b0':
            pasto.set_buzzer(0)
            print("\nBuzzer pin state sets to 0\n")
        elif choice=='b1':
            pasto.set_buzzer(1)
            print("\nBuzzer pin state sets to 1\n")
        return 0

    ################ main program ################

    pasto = Micha()
    choice = -1

    while choice!='exit':
        
        print("################ MENU ################")
        print(" GENERAL\n",
              "-------\n",
              "all \t- Show all register/state values\n",
              "bs \t- Show the buzzer pin state",
              "b0 \t- Set the buzzer pin state to 0",
              "b1 \t- Set the buzzer pin state to 1",
              "bss \t- Show boot state\n",
              "dms \t- Show debug mode state\n",
              "dm0 \t- Set debug mode to 0 (OFF)\n",
              "dm1 \t- Set debug mode to 1 (ON)\n",
              "ess \t- Get the emergency stop pin state\n",
              "ecs \t- Show error code\n",
              "id \t- Show the modbus ID\n",
              "exit \t- Exit\n",
              "\n ELECTRICAL PANEL",
              "\n ---------------\n",
              "eps \t- Show all the electrical panel states\n",
              "epgbs \t- Show the start button pin state (green button)\n",
              "epgls \t- Show the start led pin state (green button)\n",
              "epgl0 \t- Set the start led pin state to 0 (green button)\n",
              "epgl1 \t- Set the start led pin state to 1 (green button)\n",
              "epobs \t- Show the pause button pin state (orange button)\n",
              "epols \t- Show the pause led pin state (orange button)\n",
              "epol0 \t- Set the pause led pin state to 0 (orange button)\n",
              "epol1 \t- Set the pause led pin state to 1 (orange button)\n",
              "eprbs \t- Show the stop button pin state (red button)\n",
              "eprls \t- Show the state led pin state (red button)\n",
              "eprl0 \t- Set the state led pin state to 0 (red button)\n",
              "eprl1 \t- Set the state led pin state to 1 (red button)\n",
              "\n HEATING CISTERN",
              "\n ---------------\n",
              "cps \t- Show the current heating cistern power pin state\n",
              "cp0 \t- Set the heating cistern power pin state to 0 (OFF)\n",
              "cp1 \t- Set the heating cistern power pin state to 1 (ON)\n",
              "\n LEVEL SENSORS",
              "\n -------------\n",
              "ls \t- Show all the level sensor values\n",
              "lfs \t- Show all the current level sensor flag states\n",
              "l1s \t- Show the current level sensor 1 value\n",
              "lf1s \t- Show the current level sensor 1 flag state\n",
              "lf10 \t- Set the level sensor 1 flag state to 0\n",
              "lf11 \t- Set the level sensor 1 flag state to 1\n",
              "l2s \t- Show the current level sensor 2 value\n",
              "lf2s \t- Show the current level sensor 2 flag state\n",
              "lf20 \t- Set the level sensor 2 flag state to 0\n",
              "lf21 \t- Set the level sensor 2 flag state to 1\n",
              "\n PRESSURE SENSOR",
              "\n ---------------\n",
              "prs \t- Show the current pressure sensor value\n",
              "prfs \t- Show the current pressure sensor flag state\n",
              "prf0 \t- Set the pressure sensor flag state to 0\n",
              "prf1 \t- Set the pressure sensor flag state to 1\n",
              "\n THERMISTORS",
              "\n -------------\n",
              "tps \t- Show the current thermistor power pin state\n",
              "tp0 \t- Set the thermistor power pin state to 0\n",
              "tp1 \t- Set the thermistor power pin state to 1\n",
              "ts \t- Show all thermistor values\n",
              "ts1 \t- Show thermistor 1 value\n",
              "ts2 \t- Show thermistor 2 value\n",
              "ts3 \t- Show thermistor 3 value\n",
              "ts4 \t- Show thermistor 4 value\n",
              "\n PUMP",
              "\n ----\n",
              "ps \t- Show all the pump registers\n",
              "pds \t- Show the current pump direction pin state\n",
              "pd0 \t- Set the pump direction pin state to 0\n",
              "pd1 \t- Set the pump direction pin state to 1\n",
              "pps \t- Show the current pump power\n",
              "pp0 \t- Set the pump power pin state to 0 (OFF)\n",
              "pp1 \t- Set the pump power pin state to 1 (ON)\n",
              "pss \t- Show the current pump speed\n",
              "psX \t- Set the pump speed to X (0 <= X <= 65000)\n",
              "psis \t- Show the current pump speed incrementation\n",
              "psiX \t- Set the pump speed incrementation to X (0 <= X <= 65000)\n",
              "\n WATER SOLENOID",
              "\n --------------\n",
              "ss \t- Show the current water solenoid pin state\n",
              "s0 \t- Set the water solenoid pin state to 0\n",
              "s1 \t- Set the water solenoid pin state to 1\n")
        
        choice = input('Entrez votre commande : ')
        
        
        if choice=='all':
            registers()
            input()
        elif choice=='bss':
            generalState('bss')
            input()
        elif choice=='cps':
            tank('cps')
            input()
        elif choice=='cp0':
            tank('cp0')
            input()
        elif choice=='cp1':
            tank('cp1')
            input()
        elif choice=='dms':
            generalState('dms')
            input()
        elif choice=='dm0':
            generalState('dm0')
            input()
        elif choice=='dm1':
            generalState('dm1')
            input()
        elif choice=='ess':
            try:
                while True:
                    emergency_stop()
                    time.sleep(1)
            except:
                pass
        elif choice=='ecs':
            generalState('ecs')
        elif choice=='id':
            generalState('id')
            input()
        elif choice=='ls':
            try:
                while True:
                    levelSensors('ls')
                    time.sleep(1)
            except:
                pass
        elif choice=='lfs':
            levelSensors('lfs')
            input()
        elif choice=='l1s':
            try:
                while True:
                    levelSensors('l1s')
                    time.sleep(1)
            except:
                pass
        elif choice=='lf1s':
            levelSensors('lf1s')
            input()
        elif choice=='lf10':
            levelSensors('lf10')
            input()
        elif choice=='lf11':
            levelSensors('lf11')
            input()
        elif choice=='l2s':
            try:
                while True:
                    levelSensors('l2s')
                    time.sleep(1)
            except:
                pass
        elif choice=='lf2s':
            levelSensors('lf2s')
            input()
        elif choice=='lf20':
            levelSensors('lf20')
            input()
        elif choice=='lf21':
            levelSensors('lf21')
            input()
        elif choice=='ps':
            pump('ps')
            input()
        elif choice=='pps':
            pump('pps')
            input()
        elif choice=='pp0':
            pump('pp0')
            input()
        elif choice=='pp1':
            pump('pp1')
            input()
        elif choice=='pds':
            pump('pds')
            input()
        elif choice=='pd0':
            pump('pd0')
            input()
        elif choice=='pd1':
            pump('pd1')
            input()
        elif choice.find('ps')==0: # if the string 'ps' is found, there is possible un number after (to change the pump speed or the pump speed incrementtation)
            if choice.find('psi')==0: # if the string 'psi' is found, it's about the pump speed increment
                if choice=='psis': # if it's 'psis', show the pump speed incrementation value
                    pump('psis')
                    input()
                else: # else, there is perhaps a number to update the pump speed incrementation...
                    try:
                        value=int(choice[3:]) # try to extract the number to update the speed incrementation
                        
                        if value>=0 and value<=65000:
                            pasto.set_pump_speed_inc(value)
                            print('\nPump speed incrementation sets to {}\n'.format(value))
                            input()
                        else:
                            print('\nWrong speed incrementation value (0 <= increment <= 65000)\n')
                            input()
                    except: # if it's not a valid int, continue to 'Wrong command' below
                        print('\nWrong command!\n')
                        input()
            else: # else, it's about the pump speed
                if choice=='pss': # if it's 'pss', show the pump speed value
                    pump('pss')
                    input()
                else: # else, there is perhaps a number to update the pump speed...
                    try:
                        value=int(choice[2:]) # try to extract the number to update the speed
                        
                        if value>=0 and value <=65000:
                            pasto.set_pump_speed(value)
                            print('\nPump speed sets to {}\n'.format(value))
                            input()
                        else:
                            print('\nWrong speed value (0 <= speed <= 65000)\n')
                            input()
                    except: # if it's not a valid int, continue to 'Wrong command' print below
                        print('\nWrong command!\n')
                        input()
        elif choice=='prs':
            try:
                while True:
                    pressSensor('prs')
                    time.sleep(1)
            except:
                pass
        elif choice=='prfs':
            pressSensor('prfs')
            input()
        elif choice=='prf0':
            pressSensor('prf0')
            input()
        elif choice=='prf1':
            pressSensor('prf1')
            input()
        elif choice=='ss':
            sol('ss')
            input()
        elif choice=='s0':
            sol('s0')
            input()
        elif choice=='s1':
            sol('s1')
            input()
        elif choice=='tps':
            thermis('tps')
            input()
        elif choice=='tp0':
            thermis('tp0')
            input()
        elif choice=='tp1':
            thermis('tp1')
            input()
        elif choice=='ts':
            try:
                while True:
                    thermis('ts')
                    time.sleep(1)
            except:
                pass
        elif choice=='ts1':
            try:
                while True:
                    thermis('ts1')
                    time.sleep(1)
            except:
                pass
        elif choice=='ts2':
            try:
                while True:
                    thermis('ts2')
                    time.sleep(1)
            except:
                pass
        elif choice=='ts3':
            try:
                while True:
                    thermis('ts3')
                    time.sleep(1)
            except:
                pass
        elif choice=='ts4':
            try:
                while True:
                    thermis('ts4')
                    time.sleep(1)
            except:
                pass
        elif choice=='eps':
            try:
                while True:
                    electricalPanel('eps')
                    time.sleep(1)
            except:
                pass
        elif choice=='epgbs':
            try:
                while True:
                    electricalPanel('epgbs')
                    time.sleep(1)
            except:
                pass
        elif choice=='epgls':
            electricalPanel('epgls')
            input()
        elif choice=='epgl1':
            electricalPanel('epgl1')
            input()
        elif choice=='epgl0':
            electricalPanel('epgl0')
            input()
        elif choice=='epobs':
            try:
                while True:
                    electricalPanel('epobs')
                    time.sleep(1)
            except:
                pass
        elif choice=='epols':
            electricalPanel('epols')
            input()
        elif choice=='epol1':
            electricalPanel('epol1')
            input()
        elif choice=='epol0':
            electricalPanel('epol0')
            input()
        elif choice=='eprbs':
            try:
                while True:
                    electricalPanel('eprbs')
                    time.sleep(1)
            except:
                pass
        elif choice=='eprls':
            electricalPanel('eprls')
            input()
        elif choice=='eprl1':
           electricalPanel('eprl1')
           input()
        elif choice=='eprl0':
            electricalPanel('eprl0')
            input()
        elif choice=='bs':
            buzzer('bs')
            input()
        elif choice=='b0':
            buzzer('b0')
            input()
        elif choice=='b1':
            buzzer('b1')
            input()
        elif choice!='exit' and choice!='':
            print('\nWrong command!\n')
            input()
        
        
#         choice = menu_choice(0,10)  
#         print("\n")
#         
#         if choice!='0':
#             choice = menuPrincipal[choice]()
#         
#             choice = '-1' 
# 
#     hardConf.close()
    print("\nBye!\n")


