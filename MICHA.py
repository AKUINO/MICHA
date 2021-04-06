#!/usr/bin/python3
# -*- coding: utf-8 -*-

# This code allows to communicate with the MICHA board in a pastorizator configuration. It can be use as:
#     - a library to communicate with the MICHA board using another main code file
#     - a standalone code to test the I/O of the MICHA board

import traceback
from serial import Serial, PARITY_NONE
from umodbus.client.serial import rtu


# Registre configuration
# coils
#THERMIS_POW_REG = 0x01 # register which stores the thermistor power state
PUMP_DIR_REG = 0x10 # register which stores the pump direction
PUMP_POW_REG = 0x11 # register which stores the pump power state
TANK1_REG = 0x20 # register which stores the tank 1 state
TANK2_REG =0x21   # register which stores the tank 2 state
SOL_HOT_REG = 0x30   # register which stores the hot water solenoid state
SOL_COLD_REG = 0x31   # register which stores the cold water solenoid state
VALVE1_POW_REG = 0x32   # register which stores the valve 1 power state
VALVE2_POW_REG = 0x34   # register which stores the valve 2 power state
VALVE1_DIR_REG = 0x33   # register which stores the valve 1 direction
VALVE2_DIR_REG = 0x35   # register which stores the valve 2 direction
# input registers
GEN_STATE_REG = 0x00 # register which stores the general state of the system
THERMI1_REG = 0x01 # register which stores the thermistor 1 value (0 - 4095)
THERMI2_REG = 0x02 # register which stores the thermistor 2 value (0 - 4095)
THERMI3_REG = 0x03 # register which stores the thermistor 3 value (0 - 4095)
THERMI4_REG = 0x04 # register which stores the thermistor 4 value (0 - 4095)
PUMP_ERR_REG = 0x10   # register which stores the error code returned by the pump regulator
PUMP_SERVO_REG = 0x11   # register which stores the speed returned by the pump servo
ERROR_CODE_REG = 0x20   # register which stores the general error codes
# holding registers
ID_REG = 0x00   # register which stores the modbus ID
PUMP_SPEED_REG = 0x10   # register which stores the pump speed
#PUMP_SPIN_RATE_REG = 0x11   # register which stores the pump spining rate approved


# Class to manage the MICHA board
class Micha:
    def __init__(self):
        self.thermi = 0
        self.pump_speed = 0
        self.pump_dir = 0
        self.pump_power = 0
        self.tank1 = 0
        self.tank2 = 0
        self.sol_hot = 0
        self.sol_cold = 0
        self.valve1_power = 0
        self.valve1_dir = 0
        self.valve2_power = 0
        self.valve2_dir = 0
        self.general_state = 0
        self.error_code = 0
        
    def get_thermi(self, th=0):
        self.thermi = th
        
        try:
            serial_port = get_serial_port()
            
            if self.thermi==0: # get the value of all the thermistors
                message = rtu.read_input_registers(1, THERMI1_REG, 4)
            elif self.thermi==1: # get the thermistor 1 value
                message = rtu.read_input_registers(1, THERMI1_REG, 1)
            elif self.thermi==2: # get the thermistor 2 value
                message = rtu.read_input_registers(1, THERMI2_REG, 1)
            elif self.thermi==3: # get the thermistor 3 value
                message = rtu.read_input_registers(1, THERMI3_REG, 1)
            elif self.thermi==4: # get the thermistor 4 value
                message = rtu.read_input_registers(1, THERMI4_REG, 1)
            else:
                print("ERROR: no thermistor was found at this value")
            
            response = rtu.send_message(message, serial_port)
            
            serial_port.close()
        except:
            traceback.print_exc()
    
        return response
    
    def set_pump_power(self,power=0): # to set the power of the pump
        if self.pump_power != power:
            self.pump_power = power
            
            try:
                serial_port = get_serial_port()
                
                message = rtu.write_single_coil(1, PUMP_POW_REG, power)
                response = rtu.send_message(message, serial_port)
                
                serial_port.close()
            except:
                traceback.print_exc()
            return response
        return 0

    def set_pump_speed(self,speed=0): # to set the speed of the pump
        if self.pump_speed!=speed:
            self.pump_speed = speed
            
            try:
                serial_port = get_serial_port()
                
                message = rtu.write_single_register(1, PUMP_SPEED_REG, speed)
                response = rtu.send_message(message, serial_port)
                
                serial_port.close()
            except:
                traceback.print_exc()
            return response
        return 0
    
    def set_pump_dir(self,dir=0): # to set the direction of the pump
        if self.pump_dir!=dir:
            self.pump_dir = dir
            
            try:
                serial_port = get_serial_port()
                
                message = rtu.write_single_coil(1, PUMP_DIR_REG, dir)
                response = rtu.send_message(message, serial_port)
                
                serial_port.close()
            except:
                traceback.print_exc()
            
            return response
        return 0
    
    def get_pump_power(self): # to get the power state of the pump (stored in the register)
        try:
            serial_port = get_serial_port()
            
            message = rtu.read_coils(1, PUMP_POW_REG, 1)
            response = rtu.send_message(message, serial_port)
            self.pump_power = response[0]
            
            serial_port.close()
        except:
            traceback.print_exc()
            
        return self.pump_power
    
    def get_pump_dir(self): # to get the direction state of the pump (stored in the register)
        try:
            serial_port = get_serial_port()
            
            message = rtu.read_coils(1, PUMP_DIR_REG, 1)
            response = rtu.send_message(message, serial_port)
            self.pump_dir = response[0]
            
            serial_port.close()
        except:
            traceback.print_exc()
            
        return self.pump_dir
    
    def get_pump_speed(self): # to get the speed of the pump (stored in the register)
        try:
            serial_port = get_serial_port()
            
            message = rtu.read_holding_registers(1, PUMP_SPEED_REG, 1)
            response = rtu.send_message(message, serial_port)
            self.pump_speed = response[0]
            
            serial_port.close()
        except:
            traceback.print_exc()
            
        return self.pump_speed
    
    def get_pump_error(self): # to get the error code returned by the pump regulator
        try:
            serial_port = get_serial_port()
            
            message = rtu.read_input_registers(1, PUMP_ERR_REG, 1)
            response = rtu.send_message(message, serial_port)
            
            serial_port.close()
        except:
            traceback.print_exc()
        
        return response[0]
            
    def get_pump_servo(self): # to get the pump speed returned by the servo of the pump
        try:
            serial_port = get_serial_port()
            
            message = rtu.read_input_registers(1, PUMP_SERVO_REG, 1)
            response = rtu.send_message(message, serial_port)
            
            serial_port.close()
        except:
            traceback.print_exc()
        
        return response[0]
    
    def set_tank1(self,state=0): # to set the state of the tank 1
        if self.tank1 != state:
            self.tank1 = state
            
            try:
                serial_port = get_serial_port()
                
                message = rtu.write_single_coil(1, TANK1_REG, state)
                response = rtu.send_message(message, serial_port)
                
                serial_port.close()
            except:
                traceback.print_exc()
            return response
        return 0

    def set_tank2(self,state=0): # to set the state of the tank 2
        if self.tank2 != state:
            self.tank2 = state
            
            try:
                serial_port = get_serial_port()
                
                message = rtu.write_single_coil(1, TANK2_REG, state)
                response = rtu.send_message(message, serial_port)
                
                serial_port.close()
            except:
                traceback.print_exc()
            return response
        return 0
    
    def get_tank1(self): # to get the state of the tank 1 (stored in the register)
        try:
            serial_port = get_serial_port()
            
            message = rtu.read_coils(1, TANK1_REG, 1)
            response = rtu.send_message(message, serial_port)
            self.tank1 = response[0]
            
            serial_port.close()
        except:
            traceback.print_exc()
            
        return self.tank1
    
    def get_tank2(self): # to get the state of the tank 2 (stored in the register)
        try:
            serial_port = get_serial_port()
            
            message = rtu.read_coils(1, TANK2_REG, 1)
            response = rtu.send_message(message, serial_port)
            self.tank2 = response[0]
            
            serial_port.close()
        except:
            traceback.print_exc()
            
        return self.tank2
    
    def set_sol_hot(self,state=0): # to set the state of the hot water solenoid
        if self.sol_hot != state:
            self.sol_hot = state
            
            try:
                serial_port = get_serial_port()
                
                message = rtu.write_single_coil(1, SOL_HOT_REG, state)
                response = rtu.send_message(message, serial_port)
                
                serial_port.close()
            except:
                traceback.print_exc()
            return response
        return 0
    
    def set_sol_cold(self,state=0): # to set the state of the cold water solenoid
        if self.sol_cold != state:
            self.sol_cold = state
            
            try:
                serial_port = get_serial_port()
                
                message = rtu.write_single_coil(1, SOL_COLD_REG, state)
                response = rtu.send_message(message, serial_port)
                
                serial_port.close()
            except:
                traceback.print_exc()
            return response
        return 0
    
    def get_sol_hot(self): # to get the state of the hot water solenoid (stored in the register)
        try:
            serial_port = get_serial_port()
            
            message = rtu.read_coils(1, SOL_HOT_REG, 1)
            response = rtu.send_message(message, serial_port)
            self.sol_hot = response[0]
            
            serial_port.close()
        except:
            traceback.print_exc()
            
        return self.sol_hot
    
    def get_sol_cold(self): # to get the state of the cold water solenoid (stored in the register)
        try:
            serial_port = get_serial_port()
            
            message = rtu.read_coils(1, SOL_COLD_REG, 1)
            response = rtu.send_message(message, serial_port)
            self.sol_cold = response[0]
            
            serial_port.close()
        except:
            traceback.print_exc()
            
        return self.sol_cold
    
    def set_valve1_power(self,power=0): # to set the power state of the valve 1
        if self.valve1_power != power:
            self.valve1_power = power
            
            try:
                serial_port = get_serial_port()
                
                message = rtu.write_single_coil(1, VALVE1_POW_REG, power)
                response = rtu.send_message(message, serial_port)
                
                serial_port.close()
            except:
                traceback.print_exc()
            return response
        return 0
    
    def set_valve2_power(self,power=0): # to set the power state of the valve 2
        if self.valve2_power != power:
            self.valve2_power = power
            
            try:
                serial_port = get_serial_port()
                
                message = rtu.write_single_coil(1, VALVE2_POW_REG, power)
                response = rtu.send_message(message, serial_port)
                
                serial_port.close()
            except:
                traceback.print_exc()
            return response
        return 0
    
    def get_valve1_power(self): # to get the power state of the valve 1 (stored in the register)
        try:
            serial_port = get_serial_port()
            
            message = rtu.read_coils(1, VALVE1_POW_REG, 1)
            response = rtu.send_message(message, serial_port)
            self.valve1_power = response[0]
            
            serial_port.close()
        except:
            traceback.print_exc()
            
        return self.valve1_power
    
    def get_valve2_power(self): # to get the power state of the valve 2 (stored in the register)
        try:
            serial_port = get_serial_port()
            
            message = rtu.read_coils(1, VALVE2_POW_REG, 1)
            response = rtu.send_message(message, serial_port)
            self.valve2_power= response[0]
            
            serial_port.close()
        except:
            traceback.print_exc()
            
        return self.valve2_power
    
    def set_valve1_dir(self,dir=0): # to set the direction of the valve 1
        if self.valve1_dir != dir:
            self.valve1_dir = dir
            
            try:
                serial_port = get_serial_port()
                
                message = rtu.write_single_coil(1, VALVE1_DIR_REG, dir)
                response = rtu.send_message(message, serial_port)
                
                serial_port.close()
            except:
                traceback.print_exc()
            return response
        return 0
    
    def set_valve2_dir(self,dir=0): # to set the direction of the valve 2
        if self.valve2_dir!=dir:
            self.valve2_dir = dir
            
            try:
                serial_port = get_serial_port()
                
                message = rtu.write_single_coil(1, VALVE2_DIR_REG, dir)
                response = rtu.send_message(message, serial_port)
                
                serial_port.close()
            except:
                traceback.print_exc()
            return response
        return 0
    
    def get_valve1_dir(self): # to get the direction of the valve 1 (stored in the register)
        try:
            serial_port = get_serial_port()
            
            message = rtu.read_coils(1, VALVE1_DIR_REG, 1)
            response = rtu.send_message(message, serial_port)
            self.valve1_dir = response[0]
            
            serial_port.close()
        except:
            traceback.print_exc()
            
        return self.valve1_dir
    
    def get_valve2_dir(self): # to get the direction of the valve 2 (stored in the register)
        try:
            serial_port = get_serial_port()
            
            message = rtu.read_coils(1, VALVE2_DIR_REG, 1)
            response = rtu.send_message(message, serial_port)
            self.valve2_dir= response[0]
            
            serial_port.close()
        except:
            traceback.print_exc()
            
        return self.valve2_dir
    
    def get_general_state(self): # to get the general state of the system (stored in the register)
        try:
            serial_port = get_serial_port()
            
            message = rtu.read_input_registers(1, GEN_STATE_REG, 4)
            response = rtu.send_message(message, serial_port)
            self.general_state= response[0]
            
            serial_port.close()
        except:
            traceback.print_exc()
            
        return self.general_state
    
    def get_error_code(self): # to get the general error code
        try:
            serial_port = get_serial_port()
            
            message = rtu.read_input_registers(1, ERROR_CODE_REG, 4)
            response = rtu.send_message(message, serial_port)
            self.error_code= response[0]
            
            serial_port.close()
        except:
            traceback.print_exc()
            
        return self.error_code
    
# test section
if __name__ == "__main__":
    # Configuration and starting of the modbus communication
    def get_serial_port():
        """Return a serial.Serial instance which is ready to use with a RS485 adaptor."""
        port = Serial(port='/dev/serial0', baudrate=9600, parity=PARITY_NONE, stopbits=1, bytesize=8, timeout=1)
        
        return port
    
    def menu_choice(mini,maxi):
        choice = '-1'
        
        while (int(choice)<mini or int(choice)>maxi):
            choice = input("Choose an option: ")
                
            if (int(choice)<mini or int(choice)>maxi):
                print("ERROR: incorrect choice. Your choice must be [{}:{}]".format(mini,maxi))
        
        return choice

    # A sub-menu to manage the thermistors
    def subMenu_thermis():
        """Display a sub-menu to manage the thermistors."""
        choice = '-1'
        
        while choice!='0':
            print("########## THERMISTORS SUB-MENU ##########")
            print(" 1 - Get the thermistor 1 value\n",
                  "2 - Get the thermistor 2 value\n",
                  "3 - Get the thermistor 3 value\n",
                  "4 - Get the thermistor 4 value\n",
                  "5 - Get teh value of all the thermistors\n",
                  "0 - Back\n")
            
            choice = menu_choice(0,5)
            print("\n")
            
            # If the choice is valid
            if choice!='0':
                if choice=='1':
                    print("Thermistor 1 = {}".format(pasto.get_thermi(1)[0]))
                elif choice=='2':
                    print("Thermistor 2 = {}".format(pasto.get_thermi(2)[0]))
                elif choice=='3':
                    print("Thermistor 3 = {}".format(pasto.get_thermi(3)[0]))
                elif choice=='4':
                    print("Thermistor 4 = {}".format(pasto.get_thermi(4)[0]))
                elif choice=='5':
                    i = 1
                    for value in pasto.get_thermi():
                        print("Thermistor {} = {}".format(i,value))
                        i+=1
                
                input()
                
                choice = '-1'
        
        return 0

    # A sub-menu to manage the pump
    def subMenu_pump():
        """Display a sub-menu to manage the pump."""
        choice = '-1'
        
        while choice!='0':
            print("########## PUMP SUB-MENU ##########")
            print(" 1 - Power\n",
                  "2 - Speed\n",
                  "3 - Direction\n",
                  "4 - Get the speed return by the servo\n",
                  "5 - Get the error code return by the regulator\n",
                  "0 - Back\n")
            
            choice = menu_choice(0,5)
            print("\n")
            
            # If the choice is valid
            if choice!='0':
                if choice=='1':
                    print("### Power ###")
                    
                    if pasto.get_pump_power()==0:
                        print(" 0 - (OFF)\n",
                              "1 - ON\n")
                    elif pasto.get_pump_power()==1:
                        print(" 0 - OFF\n",
                              "1 - (ON)\n")
                    
                    choice = menu_choice(0,1) 
                    print("\n")
                    
                    if choice=='0':
                        pasto.set_pump_power(0)
                        print("Power OFF")
                    elif choice=='1':
                        pasto.set_pump_power(1)
                        print("Power ON")
                    input()
                elif choice=='2':
                    while choice!='0':
                        print("### Speed ###")
                        print("\nCurrent speed = {}\n".format(pasto.get_pump_speed()))
                        print(" 1 - Modify\n",
                              "0 - Back\n")
                        
                        choice = menu_choice(0,1) 
                        print("\n")
                        
                        if choice!='0':
                            updatedSpeed = input("Enter a new speed: ")
                            pasto.set_pump_speed(int(updatedSpeed))
                            choice = '-1'
                elif choice=='3':
                    print("### Direction ###")
                    
                    if pasto.get_pump_dir()==0:
                        print(" 0 - (Suction mode)\n",
                              "1 - Backflow mode\n")
                    elif pasto.get_pump_dir()==1:
                        print(" 0 - Suction mode\n",
                              "1 - (Backflow mode)\n")
                    
                    choice = menu_choice(0,1) 
                    print("\n")
                    
                    if choice=='0':
                        pasto.set_pump_dir(0)
                        print("Suction mode ON")
                    elif choice=='1':
                        pasto.set_pump_dir(1)
                        print("Backflow mode ON")
                    input()
                elif choice=='4':
                    print("Speed returned by the servo = {}".format(pasto.get_pump_servo()))
                    input()
                elif choice=='5':
                    print("Error returned by the regulator = {}".format(pasto.get_pump_error()))
                    input()
                
                choice= '-1'
        
        return 0

    # Sub-menu to manage the tanks
    def subMenu_tanks():
        """Display a sub-menu to manage the tanks."""
        choice = '-1'
        
        while choice!='0':
            print("########## TANKS SUB-MENU ##########")
            print(" 1 - Tank 1 state\n",
                  "2 - Tank 2 state\n",
                  "0 - Back\n")
            
            choice = menu_choice(0,2)
            print("\n")
            
            # If the choice is valid
            if choice!='0':
                if choice=='1':
                    print("### Tank 1 ###")
                    
                    if pasto.get_tank1()==0:
                        print(" 0 - (OFF)\n",
                              "1 - ON\n")
                    elif pasto.get_tank1()==1:
                        print(" 0 - OFF\n",
                              "1 - (ON)\n")
                    
                    choice = menu_choice(0,1) 
                    print("\n")
                    
                    if choice=='0':
                        pasto.set_tank1(0)
                        print("Tank 1 OFF")
                    elif choice=='1':
                        pasto.set_tank1(1)
                        print("Tank 1 ON")
                    input()
                elif choice=='2':
                    print("### Tank 2 ###")
                    
                    if pasto.get_tank2()==0:
                        print(" 0 - (OFF)\n",
                              "1 - ON\n")
                    elif pasto.get_tank2()==1:
                        print(" 0 - OFF\n",
                              "1 - (ON)\n")
                    
                    choice = menu_choice(0,1) 
                    print("\n")
                    
                    if choice=='0':
                        pasto.set_tank2(0)
                        print("Tank 2 OFF")
                    elif choice=='1':
                        pasto.set_tank2(1)
                        print("Tank 2 ON")
                    input()
            
                choice = '-1'
        
        return 0

    # Sub-menu to manage the valves and solenoids
    def subMenu_valvesSol():
        """Display en sub-menu to manage the valves and solenoids."""
        choice = '-1'
        
        while choice!='0':
            print("########## VALVES AND SOLENOIDS SUB-MENU ##########")
            print(" 1 - Hot water solenoid\n",
                  "2 - Cold water solenoid\n",
                  "3 - Manage the valve 1\n",
                  "4 - Manage the valve 2\n",
                  "0 - Back\n")
            
            choice = menu_choice(0,4)
            print("\n")
            
            # If the choice is valid
            if choice!='0':
                if choice=='1':
                    print("### Hot water solenoid ###")
                    
                    if pasto.get_sol_hot()==0:
                        print(" 0 - (Close)\n",
                              "1 - Open\n")
                    elif pasto.get_sol_hot()==1:
                        print(" 0 - Close\n",
                              "1 - (Open)\n")
                    
                    choice = menu_choice(0,1) 
                    print("\n")
                    
                    if choice=='0':
                        pasto.set_sol_hot(0)
                        print("Hot water solenoid is closing")
                    elif choice=='1':
                        pasto.set_sol_hot(1)
                        print("Hot water solenoid is opening")
                    input()
                elif choice=='2':
                    print("### Cold water solenoid ###")
                    
                    if pasto.get_sol_cold()==0:
                        print(" 0 - (Close)\n",
                              "1 - Open\n")
                    elif pasto.get_sol_cold()==1:
                        print(" 0 - Close\n",
                              "1 - (Open)\n")
                    
                    choice = menu_choice(0,1) 
                    print("\n")
                    
                    if choice=='0':
                        pasto.set_sol_cold(0)
                        print("Cold water solenoid is closing")
                    elif choice=='1':
                        pasto.set_sol_cold(1)
                        print("Cold water solenoid is opening")
                    input()
                elif choice=='3':
                    while choice!='0':
                        print("### Valve 1 ###")
                        print(" 1 - Power\n",
                                  "2 - Direction\n",
                                  "0 - Back")
                        
                        choice = menu_choice(0,2) 
                        print("\n")
                        
                        if choice!='0':
                            if choice=='1':
                                print("### Valve 1 power ###")

                                if pasto.get_valve1_power()==0:
                                    print(" 0 - (OFF)\n",
                                          "1 - ON")
                                elif pasto.get_valve1_power()==1:
                                    print(" 0 - OFF\n",
                                          "1 - (ON)\n")
                                    
                                choice = menu_choice(0,1) 
                                print("\n")
                                
                                if choice=='0':
                                    pasto.set_valve1_power(0)
                                    print("Valve 1 power OFF")
                                elif choice=='1':
                                    pasto.set_valve1_power(1)
                                    print("Valve 1 power ON")
                                input()
                            
                            elif choice=='2':
                                print("### Valve 1 direction ###")

                                if pasto.get_valve1_dir()==0:
                                    print(" 0 - (Direction 1)\n",
                                          "1 - Direction 2")
                                elif pasto.get_valve1_dir()==1:
                                    print(" 0 - Direction 1\n",
                                          "1 - (Direction 2)\n")
                                    
                                choice = menu_choice(0,1) 
                                print("\n")
                                
                                if choice=='0':
                                    pasto.set_valve1_dir(0)
                                    print("Valve 1 set in direction 1")
                                elif choice=='1':
                                    pasto.set_valve1_dir(1)
                                    print("Valve 1 set in direction 2")
                                input()
                            choice = '-1'
                elif choice=='4':
                    while choice!='0':
                        print("### Valve 2 ###")
                        print(" 1 - Power\n",
                                  "2 - Direction\n",
                                  "0 - Back")
                        
                        choice = menu_choice(0,2) 
                        print("\n")
                        
                        if choice!='0':
                            if choice=='1':
                                print("### Valve 2 power ###")

                                if pasto.get_valve2_power()==0:
                                    print(" 0 - (OFF)\n",
                                          "1 - ON")
                                elif pasto.get_valve2_power()==1:
                                    print(" 0 - OFF\n",
                                          "1 - (ON)\n")
                                    
                                choice = menu_choice(0,1) 
                                print("\n")
                                
                                if choice=='0':
                                    pasto.set_valve2_power(0)
                                    print("Valve 2 power OFF")
                                elif choice=='1':
                                    pasto.set_valve2_power(1)
                                    print("Valve 2 power ON")
                                input()
                            
                            elif choice=='2':
                                print("### Valve 2 direction ###")

                                if pasto.get_valve2_dir()==0:
                                    print(" 0 - (Direction 1)\n",
                                          "1 - Direction 2")
                                elif pasto.get_valve2_dir()==1:
                                    print(" 0 - Direction 1\n",
                                          "1 - (Direction 2)\n")
                                    
                                choice = menu_choice(0,1) 
                                print("\n")
                                
                                if choice=='0':
                                    pasto.set_valve2_dir(0)
                                    print("Valve 2 set in direction 1")
                                elif choice=='1':
                                    pasto.set_valve2_dir(1)
                                    print("Valve 2 set in direction 2")
                                input()
                            choice = '-1'
            
                choice = '-1'
                
        return 0

    def subMenu_generalState():
        """Display the error code and the general state of the system"""
        
        print("########## General state ##########")
        print("General state = {}".format(pasto.get_general_state()))
        print("Error code = {}\n".format(pasto.get_error_code()))
        
        input()

    def subMenu_registers():
        """Display the value of all the registers."""
        print("########## Registers ##########")
        # Thermistors
        i = 1
        for value in pasto.get_thermi():
            print("Thermistor {} \t\t= {}".format(i,value))
            i+=1
        # Pompe
        print("Pum power \t\t= {}".format(pasto.get_pump_power()))
        print("Pump speed \t\t= {}".format(pasto.get_pump_speed()))
        print("Pump direction \t\t= {}".format(pasto.get_pump_dir()))
        print("Pump servo \t\t= {}".format(pasto.get_pump_servo()))
        print("Pump error \t\t= {}".format(pasto.get_pump_error()))
        print("Tank 1 state \t\t= {}".format(pasto.get_tank1()))
        print("Tank 2 state \t\t= {}".format(pasto.get_tank2()))
        print("Hot water solenoid \t= {}".format(pasto.get_sol_hot()))
        print("Cold water solenoid \t= {}".format(pasto.get_sol_cold()))
        print("Valve 1 power \t\t= {}".format(pasto.get_valve1_power()))
        print("Valve 1 direction \t= {}".format(pasto.get_valve1_dir()))
        print("Valve 2 power \t\t= {}".format(pasto.get_valve2_power()))
        print("Valve 2 direction \t= {}".format(pasto.get_valve2_dir()))
        
        input()

    ################ main program ################

    choice = '-1'
    menuPrincipal = {'1':subMenu_thermis, '2':subMenu_pump, '3':subMenu_tanks, '4':subMenu_valvesSol, '5':subMenu_generalState, '6':subMenu_registers}
    pasto = Micha()

    while choice!='0':
        
        print("################ MENU ################")
        print(" 1 - Thermistors\n",
              "2 - Pump\n",
              "3 - Tanks\n",
              "4 - Valves/solenoids\n",
              "5 - General state\n",
              "6 - All registers\n",
              "0 - Exit\n")
        
        choice = menu_choice(0,6)  
        print("\n")
        
        if choice!='0':
            choice = menuPrincipal[choice]()
        
            choice = '-1' 

    print("\nBye!\n")

