#!/usr/bin/python3
# -*- coding: utf-8 -*-

# This code allows to communicate with the MICHA board in a pastorizator configuration. It can be use as:
#     - a library to communicate with the MICHA board using another main code file
#     - a standalone code to test the I/O of the MICHA board

import traceback
from serial import Serial, PARITY_NONE
from umodbus.client.serial import rtu
import time

SLAVE_ID = 1

VOLTAGE_REF = 2.497 # value of the excitement voltage reference
THERMI_WIRE = 0.6 # value of the twin wire resistor


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
    def __init__(self,device='/dev/serial0'):
        self.device = device
        self.boot_flag = 1
        self.thermi = 0
        self.pump_speed = 0
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

    def get_boot_flag(self): # to get the boot state
        self.boot_flag = self.read_pin(BOOT_FLAG_REG)
        return self.boot_flag
    
    def set_boot_flag(self,flag=0): # to set the boot state
        if self.boot_flag != flag:
            self.boot_flag = flag
            response = self.write_pin(BOOT_FLAG_REG, flag)
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
                    
        if choice==0:
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
            
        elif choice==1:
            thermi1 = pasto.get_thermi(1)[0]
            thermi1_mV = (VOLTAGE_REF*thermi1/4096)*1000
            print("\nThermistor 1 = {} ({:4.3f} mV\n)".format(thermi1,thermi1_mV))
        elif choice==2:
            thermi2 = pasto.get_thermi(2)[0]
            thermi2_mV = (VOLTAGE_REF*thermi2/4096)*1000
            print("\nThermistor 2 = {} ({:4.3f} mV\n)".format(thermi2,thermi2_mV))
        elif choice==3:
            thermi3 = pasto.get_thermi(3)[0]
            thermi3_mV = (VOLTAGE_REF*thermi3/4096)*1000
            print("\nThermistor 3 = {} ({:4.3f} mV\n)".format(thermi3,thermi3_mV))
        elif choice==4:
            thermi4 = pasto.get_thermi(4)[0]
            thermi4_mV = (VOLTAGE_REF*thermi4/4096)*1000
            print("\nThermistor 4 = {} ({:4.3f} mV\n)".format(thermi4,thermi4_mV))                
                        
        return 0

    # Allows to manage the pump
    def pump(choice):
        """Get or set a value related to the pump."""
        
        if choice=='pps': # gets the current pump power pin state
            print("\nCurrent pump power pin state = {}\n".format(pasto.get_pump_power()))
        elif choice=='pp0': # sets the pump power pin state to 0
            pasto.set_pump_power(0)
            print('\nPump power pin state sets to 0 (OFF??)\n')
        elif choice=='pp1': # sets the pump power pin state to 1
            pasto.set_pump_power(1)
            print('\nPump power pin state sets to 1 (ON??)\n')
        elif choice=='pss': # gets the current pump speed value
            print("\nCurrent pump speed = {}\n".format(pasto.get_pump_speed()))
        elif choice=='psX': # sets the pump speed to X (0<=X<=65000)
            print('This function isn\'t yet implemented')
        elif choice=='pds': # gets the current pump direction pin state
            print("\nCurrent pump direction pin state = {}\n".format(pasto.get_pump_dir()))
        elif choice=='pd0': # sets the pump direction pin state to 0
            pasto.set_pump_dir(0)
            print('\nPump direction pin state sets to 0\n')
        elif choice=='pd1': # sets the pump direction pin state to 1
            pasto.set_pump_dir(1)
            print('\nPump direction pin state sets to 1\n')

#                             updatedSpeed = input("Enter a new speed: ")
#                             pasto.set_pump_speed(int(updatedSpeed))
        
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

    # Allows to manage the solenoid valve
    def sol(choice):
        """Get or set a value related to the solenoid valve."""
        
        if choice=='ss': # gets the current solenoid pin state
            print("\nCurrent solenoid pin state = {}\n".format(pasto.get_sol_hot()))
        elif choice=='s0': # sets the solenoid pin state to 0
            pasto.set_sol_hot(0)
            print('\nSolenoid pin state sets to 0 (CLOSED)\n')
        elif choice=='s1': # sets the solenoid pin state to 1
            pasto.set_sol_hot(1)
            print('\nSolenoid pin state sets to 1 (OPENED)\n')
                
        return 0
    
    def levelSensors(choice):
        """Get or set a value related to the level sensors."""
        
        if choice=='l1s': # gets the current level sensor 1 value
            print("\nCurrent level sensor 1 value = {}\n".format(pasto.get_level1_sensor()))
        elif choice=='lf1s': # gets the current level sensor 1 flag state
            print("\nLevel sensor 1 flag state = {}\n".format(pasto.get_level1_flag()))
        elif choice=='lf10': # sets the level sensor 1 flag state to 0
            pasto.set_level1_flag(0)
            print('\nLevel sensor 1 flag state sets to 0 (OFF)\n')
        elif choice=='lf11': # sets the level sensor 1 flag state to 1
            pasto.set_level1_flag(1)
            print('\nLevel sensor 1 flag state sets to 1 (ON)\n')
        if choice=='l2s': # gets the current level sensor 2 value
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

    def generalState(choice):
        """Get the error code and the general state of the system"""
        
        if choice=='bss':
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
        """Display the value of all the registers."""
        
        print("########## Registers ##########")
        # State
        print("Boot state flag \t= {}".format(pasto.get_boot_flag()))
        print("Debug mode flag \t= {}".format(pasto.get_debug_flag()))
        print("General state \t= {}".format(pasto.get_general_state()))
        print("Error code \t= {}".format(pasto.get_error_code()))
        # Thermistors
        i = 1
        for value in pasto.get_thermi():
            print("Thermistor {} \t\t= {}".format(i,value))
            i+=1
        # Pompe
        print("Pum power \t\t= {}".format(pasto.get_pump_power()))
        print("Pump speed \t\t= {}".format(pasto.get_pump_speed()))
        print("Pump direction \t\t= {}".format(pasto.get_pump_dir()))
        print("Tank 1 state \t\t= {}".format(pasto.get_tank1()))
        print("Hot water solenoid \t= {}".format(pasto.get_sol_hot()))

    ################ main program ################

    pasto = Micha()
    choice = -1

    while choice!='exit':
        
        print("################ MENU ################")
        print(" all - Show all registers value\n",
              "bss - Show boot state\n",
              "cps - Show the current heating cistern power pin state\n",
              "cp0 - Set the heating cistern power pin state to 0\n",
              "cp1 - Set the heating cistern power pin state to 1\n",
              "dms - Show debug mode state\n",
              "dm0 - Set debug mode to 0 (OFF)\n",
              "dm1 - Set debug mode to 1 (ON)\n",
              "ecs - Show error code\n",
              "l1s - Show the current level sensor 1 value\n",
              "lf1s - Show the current level sensor 1 flag state\n",
              "lf10 - Set the level sensor 1 flag state to 0\n",
              "lf11 - Set the level sensor 1 flag state to 1\n",
              "l2s - Show the current level sensor 2 value\n",
              "lf2s - Show the current level sensor 2 flag state\n",
              "lf20 - Set the level sensor 2 flag state to 0\n",
              "lf21 - Set the level sensor 2 flag state to 1\n",
              "pds - Show the current pump direction pin state\n",
              "pd0 - Set the pump direction pin state to 0\n",
              "pd1 - Set the pump direction pin state to 1\n",
              "pps - Show the current pump power\n",
              "pp0 - Set the pump power pin state to 0\n",
              "pp1 - Set the pump power pin state to 1\n",
              "pss - Show the current pump speed\n",
              "psX - Set the pump speed to X (0 <= X <= 65000)\n",
              "ss - Show the current solenoid pin state\n",
              "s0 - Set the solenoid pin state to 0\n",
              "s1 - Set the solenoid pin state to 1\n",
              "ts - Show all thermistor values\n",
              "ts1 - Show thermistor 1 value\n",
              "ts2 - Show thermistor 2 value\n",
              "ts3 - Show thermistor 3 value\n",
              "ts4 - Show thermistor 4 value\n",
              "exit - Exit\n")
        
        choice = input('Enrez votre commande : ')
        
        if choice=='all':
            registers()
        elif choice=='bss':
            generalState('bss')
        elif choice=='cps':
            tank('cps')
        elif choice=='cp0':
            tank('cp0')
        elif choice=='cp1':
            tank('cp1')
        elif choice=='dms':
            generalState('dms')
        elif choice=='dm0':
            generalState('dm0')
        elif choice=='dm1':
            generalState('dm1')
        elif choice=='ecs':
            generalState('ecs')
        elif choice=='l1s':
            levelSensors('l1s')
        elif choice=='lf1s':
            levelSensors('lf1s')
        elif choice=='lf10':
            levelSensors('lf10')
        elif choice=='lf11':
            levelSensors('lf11')
        elif choice=='l2s':
            levelSensors('l2s')
        elif choice=='lf2s':
            levelSensors('lf2s')
        elif choice=='lf20':
            levelSensors('lf20')
        elif choice=='lf21':
            levelSensors('lf21')
        elif choice=='ss':
            sol('ss')
        elif choice=='s0':
            sol('s0')
        elif choice=='s1':
            sol('s1')
        elif choice=='ts':
            thermis(0)
        elif choice=='ts1':
            thermis(1)
        elif choice=='ts2':
            thermis(2)
        elif choice=='ts3':
            thermis(3)
        elif choice=='ts4':
            thermis(4)
        elif choice=='pps':
            pump('pps')
        elif choice=='pp0':
            pump('pp0')
        elif choice=='pp1':
            pump('pp1')
        elif choice=='pds':
            pump('pds')
        elif choice=='pd0':
            pump('pd0')
        elif choice=='pd1':
            pump('pd1')
        elif choice=='pss':
            pump('pss')
        elif choice=='psXXX':
            pump('psXXX')            
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


