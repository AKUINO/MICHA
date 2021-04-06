#!/usr/bin/python3
# -*- coding: utf-8 -*-

import traceback
from serial import Serial, PARITY_NONE
from umodbus.client.serial import rtu


# configuration des registres
# coils
THERMIS_ALIM_REG = 0x01 # excitation des thermistances
POMPE_D_REG = 0x10 # direction de la pompe
POMPE_A_REG = 0x11 # alimentation de la pompe
CUVE1_REG = 0x20 # gestion de la chauffe cuve 1
CUVE2_REG =0x21   # gestion de la chauffe cuve 2
SOL_CHAUD_REG = 0x30   # solénoïde eau chaude
SOL_FROID_REG = 0x31   # solénoïde eau froide
VANNE_EVAC1_ALIM_REG = 0x32   # état de la vanne dévacuation 1
VANNE_EVAC2_ALIM_REG = 0x34   # état de la vanne dévacuation 2
VANNE_EVAC1_DIR_REG = 0x33   # direction de la vanne d'évacuation 1
VANNE_EVAC2_DIR_REG = 0x35   # direction de la vanne d'évacuation 2
# input registers
ETAT_GEN_REG = 0x00 # état général du système
THERMI1_REG = 0x01 # valeur de la thermistance 1
THERMI2_REG = 0x02 # valeur de la thermistance 2
THERMI3_REG = 0x03 # valeur de la thermistance 3
THERMI4_REG = 0x04 # valeur de la thermistance 4
POMPE_E_REG = 0x10   # erreur renvoyée par la pompe
POMPE_S_REG = 0x11   # signal servo de la pompe
ERREURS_REG = 0x20   # erreurs dans le système
# holding registers
ID_REG = 0x00   # ID Modbus de l'Arduino
POMPE_V_REG = 0x10   # vitesse de la pompe
POMPE_TAUX_PATINAGE_REG = 0x11   # taux d'erreur patinage


# Permet de gérer les thermistances
class Micha:
    def __init__(self):
        self.thermi = "all"
        self.pompe_vitesse = 0
        self.pompe_direction = 0
        self.pompe_alimentation = 0
        self.chauffe_cuve1 = 0
        self.chauffe_cuve2 = 0
        self.sol_eauChaude = 0
        self.sol_eauFroide = 0
        self.vanne1_alim = 0
        self.vanne1_dir = 0
        self.vanne2_alim = 0
        self.vanne2_dir = 0
        self.etat_general = 0
        self.code_erreurs = 0
        
    def lecture_thermi(self, th="all"):
        self.thermi = th
        
        try:
            serial_port = get_serial_port()
            
            if self.thermi=="all": #lecture de l'ensemble des thermistances
                message = rtu.read_input_registers(1, THERMI1_REG, 4)
            elif self.thermi=="thermi1": #lecture de la thermistance 1
                message = rtu.read_input_registers(1, THERMI1_REG, 1)
            elif self.thermi=="thermi2": #lecture de la thermistance 2
                message = rtu.read_input_registers(1, THERMI2_REG, 1)
            elif self.thermi=="thermi3": #lecture de la thermistance 3
                message = rtu.read_input_registers(1, THERMI3_REG, 1)
            elif self.thermi=="thermi4": #lecture de la thermistance 4
                message = rtu.read_input_registers(1, THERMI4_REG, 1)
            else:
                print("ERREUR: aucune thermistance ne correspond à cette valeur")
            
            response = rtu.send_message(message, serial_port)
            
            serial_port.close()
        except:
            traceback.print_exc()
    
        return response
    
    def modif_pompe_alimentation(self,alim=0): #permet de modifier l'alimentation
        if self.pompe_alimentation!=alim:
            self.pompe_alimentation = alim
            
            try:
                serial_port = get_serial_port()
                
                message = rtu.write_single_coil(1, POMPE_A_REG, alim)
                response = rtu.send_message(message, serial_port)
                
                serial_port.close()
            except:
                traceback.print_exc()
            return response
        return 0

    def modif_pompe_vitesse(self,vit=0): # permet de mofifier la vitesse
        if self.pompe_vitesse!=vit:
            self.pompe_vitesse = vit
            
            try:
                serial_port = get_serial_port()
                
                message = rtu.write_single_register(1, POMPE_V_REG, vit)
                response = rtu.send_message(message, serial_port)
                
                serial_port.close()
            except:
                traceback.print_exc()
            return response
        return 0
    
    def modif_pompe_direction(self,dir=0): # permet de modifier la direction
        if self.pompe_direction!=dir:
            self.pompe_direction = dir
            
            try:
                serial_port = get_serial_port()
                
                message = rtu.write_single_coil(1, POMPE_D_REG, dir)
                response = rtu.send_message(message, serial_port)
                
                serial_port.close()
            except:
                traceback.print_exc()
            
            return response
        return 0
    
    def lecture_pompe_alimentation(self):
        try:
            serial_port = get_serial_port()
            
            message = rtu.read_coils(1, POMPE_A_REG, 1)
            response = rtu.send_message(message, serial_port)
            self.pompe_alimentation = response[0]
            
            serial_port.close()
        except:
            traceback.print_exc()
            
        return self.pompe_alimentation
    
    def lecture_pompe_direction(self):
        try:
            serial_port = get_serial_port()
            
            message = rtu.read_coils(1, POMPE_D_REG, 1)
            response = rtu.send_message(message, serial_port)
            self.pompe_direction = response[0]
            
            serial_port.close()
        except:
            traceback.print_exc()
            
        return self.pompe_direction
    
    def lecture_pompe_vitesse(self):
        try:
            serial_port = get_serial_port()
            
            message = rtu.read_holding_registers(1, POMPE_V_REG, 1)
            response = rtu.send_message(message, serial_port)
            self.pompe_vitesse = response[0]
            
            serial_port.close()
        except:
            traceback.print_exc()
            
        return self.pompe_vitesse
    
    def lecture_pompe_signalErreur(self): # permet de récupérer le signal d'erreur envoyé par la pompe
        try:
            serial_port = get_serial_port()
            
            message = rtu.read_input_registers(1, POMPE_E_REG, 1)
            response = rtu.send_message(message, serial_port)
            
            serial_port.close()
        except:
            traceback.print_exc()
        
        return response[0]
            
    def lecture_pompe_signalServo(self): # permet de récupérer le signal servo envoyé par la pompe
        try:
            serial_port = get_serial_port()
            
            message = rtu.read_input_registers(1, POMPE_S_REG, 1)
            response = rtu.send_message(message, serial_port)
            
            serial_port.close()
        except:
            traceback.print_exc()
        
        return response[0]
    
    def modif_cuve1_chauffe(self,chauffe=0): #permet de gérer la chauffe de la cuve 1
        if self.chauffe_cuve1!=chauffe:
            self.chauffe_cuve1 = chauffe
            
            try:
                serial_port = get_serial_port()
                
                message = rtu.write_single_coil(1, CUVE1_REG, chauffe)
                response = rtu.send_message(message, serial_port)
                
                serial_port.close()
            except:
                traceback.print_exc()
            return response
        return 0

    def modif_cuve2_chauffe(self,chauffe=0): #permet de gérer la chauffe de la cuve 2
        if self.chauffe_cuve2!=chauffe:
            self.chauffe_cuve2 = chauffe
            
            try:
                serial_port = get_serial_port()
                
                message = rtu.write_single_coil(1, CUVE2_REG, chauffe)
                response = rtu.send_message(message, serial_port)
                
                serial_port.close()
            except:
                traceback.print_exc()
            return response
        return 0
    
    def lecture_cuve1_chauffe(self):
        try:
            serial_port = get_serial_port()
            
            message = rtu.read_coils(1, CUVE1_REG, 1)
            response = rtu.send_message(message, serial_port)
            self.chauffe_cuve1 = response[0]
            
            serial_port.close()
        except:
            traceback.print_exc()
            
        return self.chauffe_cuve1
    
    def lecture_cuve2_chauffe(self):
        try:
            serial_port = get_serial_port()
            
            message = rtu.read_coils(1, CUVE2_REG, 1)
            response = rtu.send_message(message, serial_port)
            self.chauffe_cuve2 = response[0]
            
            serial_port.close()
        except:
            traceback.print_exc()
            
        return self.chauffe_cuve2
    
    def modif_sol_eauChaude(self,ouverture=0): #permet de gérer le solénoïde d'eau chaude
        if self.sol_eauChaude!=ouverture:
            self.sol_eauChaude = ouverture
            
            try:
                serial_port = get_serial_port()
                
                message = rtu.write_single_coil(1, SOL_CHAUD_REG, ouverture)
                response = rtu.send_message(message, serial_port)
                
                serial_port.close()
            except:
                traceback.print_exc()
            return response
        return 0
    
    def modif_sol_eauFroide(self,ouverture=0): #permet de gérer le solénoïde d'eau froide
        if self.sol_eauFroide!=ouverture:
            self.sol_eauFroide = ouverture
            
            try:
                serial_port = get_serial_port()
                
                message = rtu.write_single_coil(1, SOL_FROID_REG, ouverture)
                response = rtu.send_message(message, serial_port)
                
                serial_port.close()
            except:
                traceback.print_exc()
            return response
        return 0
    
    def lecture_sol_eauChaude(self):
        try:
            serial_port = get_serial_port()
            
            message = rtu.read_coils(1, SOL_CHAUD_REG, 1)
            response = rtu.send_message(message, serial_port)
            self.sol_eauChaude = response[0]
            
            serial_port.close()
        except:
            traceback.print_exc()
            
        return self.sol_eauChaude
    
    def lecture_sol_eauFroide(self):
        try:
            serial_port = get_serial_port()
            
            message = rtu.read_coils(1, SOL_FROID_REG, 1)
            response = rtu.send_message(message, serial_port)
            self.sol_eauFroide = response[0]
            
            serial_port.close()
        except:
            traceback.print_exc()
            
        return self.sol_eauFroide
    
    def modif_vanne1_alimentation(self,ouverture=0): #permet de gérer la vanne d'évacuation 1
        if self.vanne1_alim!=ouverture:
            self.vanne1_alim = ouverture
            
            try:
                serial_port = get_serial_port()
                
                message = rtu.write_single_coil(1, VANNE_EVAC1_ALIM_REG, ouverture)
                response = rtu.send_message(message, serial_port)
                
                serial_port.close()
            except:
                traceback.print_exc()
            return response
        return 0
    
    def modif_vanne2_alimentation(self,ouverture=0): #permet de gérer la vanne d'évacuation 2
        if self.vanne2_alim!=ouverture:
            self.vanne2_alim = ouverture
            
            try:
                serial_port = get_serial_port()
                
                message = rtu.write_single_coil(1, VANNE_EVAC2_ALIM_REG, ouverture)
                response = rtu.send_message(message, serial_port)
                
                serial_port.close()
            except:
                traceback.print_exc()
            return response
        return 0
    
    def lecture_vanne1_alimentation(self):
        try:
            serial_port = get_serial_port()
            
            message = rtu.read_coils(1, VANNE_EVAC1_ALIM_REG, 1)
            response = rtu.send_message(message, serial_port)
            self.vanne1_alim = response[0]
            
            serial_port.close()
        except:
            traceback.print_exc()
            
        return self.vanne1_alim
    
    def lecture_vanne2_alimentation(self):
        try:
            serial_port = get_serial_port()
            
            message = rtu.read_coils(1, VANNE_EVAC2_ALIM_REG, 1)
            response = rtu.send_message(message, serial_port)
            self.vanne2_alim= response[0]
            
            serial_port.close()
        except:
            traceback.print_exc()
            
        return self.vanne2_alim
    
    def modif_vanne1_direction(self,dir=0): #permet de gérer la vanne d'évacuation 1
        if self.vanne1_dir!=dir:
            self.vanne1_dir = dir
            
            try:
                serial_port = get_serial_port()
                
                message = rtu.write_single_coil(1, VANNE_EVAC1_DIR_REG, dir)
                response = rtu.send_message(message, serial_port)
                
                serial_port.close()
            except:
                traceback.print_exc()
            return response
        return 0
    
    def modif_vanne2_direction(self,dir=0): #permet de gérer la vanne d'évacuation 2
        if self.vanne2_dir!=dir:
            self.vanne2_dir = dir
            
            try:
                serial_port = get_serial_port()
                
                message = rtu.write_single_coil(1, VANNE_EVAC2_DIR_REG, dir)
                response = rtu.send_message(message, serial_port)
                
                serial_port.close()
            except:
                traceback.print_exc()
            return response
        return 0
    
    def lecture_vanne1_direction(self):
        try:
            serial_port = get_serial_port()
            
            message = rtu.read_coils(1, VANNE_EVAC1_DIR_REG, 1)
            response = rtu.send_message(message, serial_port)
            self.vanne1_dir = response[0]
            
            serial_port.close()
        except:
            traceback.print_exc()
            
        return self.vanne1_dir
    
    def lecture_vanne2_direction(self):
        try:
            serial_port = get_serial_port()
            
            message = rtu.read_coils(1, VANNE_EVAC2_DIR_REG, 1)
            response = rtu.send_message(message, serial_port)
            self.vanne2_dir= response[0]
            
            serial_port.close()
        except:
            traceback.print_exc()
            
        return self.vanne2_dir
    
    def lecture_etatGeneral(self):
        try:
            serial_port = get_serial_port()
            
            message = rtu.read_input_registers(1, ETAT_GEN_REG, 4)
            response = rtu.send_message(message, serial_port)
            self.etat_general= response[0]
            
            serial_port.close()
        except:
            traceback.print_exc()
            
        return self.etat_general
    
    def lecture_codeErreurs(self):
        try:
            serial_port = get_serial_port()
            
            message = rtu.read_input_registers(1, ERREURS_REG, 4)
            response = rtu.send_message(message, serial_port)
            self.code_erreurs= response[0]
            
            serial_port.close()
        except:
            traceback.print_exc()
            
        return self.code_erreurs
    
# test section
if __name__ == "__main__":
    # Configuration et ouverture de la communication Modbus
    def get_serial_port():
        """Retourne une instance serial.Serial, prêt à être utilisé avec le RS485."""
        port = Serial(port='/dev/serial0', baudrate=9600, parity=PARITY_NONE, stopbits=1, bytesize=8, timeout=1)
        
        return port
    
    def choixMenu(mini,maxi):
        choixM = '-1'
        
        while (int(choixM)<mini or int(choixM)>maxi):
            choixM = input("Veuillez choisir une option: ")
                
            if (int(choixM)<mini or int(choixM)>maxi):
                print("ERREUR: choix incorrect")
        
        return choixM

    # Sous-menu pour la gestion des thermistances
    def ssMenuThermis():
        """Affiche un sous-menu pour gérer les thermistances."""
        choix = '-1'
        
        while choix!='0':
            print("########## SOUS-MENU THERMISTANCES ##########")
            print(" 1 - Lecture de la thermistance 1\n",
                  "2 - Lecture de la thermistance 2\n",
                  "3 - Lecture de la thermistance 3\n",
                  "4 - Lecture de la thermistance 4\n",
                  "5 - Lecture de l'ensemble des thermistances\n",
                  "0 - Retour\n")
            
            choix = choixMenu(0,5)
            print("\n")
            
            # Un choix valide a été fait dans le sous-menu
            if choix!='0':
                if choix=='1':
                    print("Thermistance 1 = {}".format(pasto.lecture_thermi("thermi1")[0]))
                elif choix=='2':
                    print("Thermistance 2 = {}".format(pasto.lecture_thermi("thermi2")[0]))
                elif choix=='3':
                    print("Thermistance 3 = {}".format(pasto.lecture_thermi("thermi3")[0]))
                elif choix=='4':
                    print("Thermistance 4 = {}".format(pasto.lecture_thermi("thermi4")[0]))
                elif choix=='5':
                    i = 1
                    for valeur in pasto.lecture_thermi():
                        print("Thermistance {} = {}".format(i,valeur))
                        i+=1
                
                input()
                
                choix = '-1'
        
        return 0

    # Sous-menu pour la gestion de la pompe
    def ssMenuPompe():
        """Affiche un sous-menu pour gérer la pompe."""
        choix = '-1'
        
        while choix!='0':
            print("########## SOUS-MENU POMPE ##########")
            print(" 1 - Alimentation\n",
                  "2 - Vitesse\n",
                  "3 - Sens de rotation\n",
                  "4 - Lecture du signal servo\n",
                  "5 - Lecture du signal d'erreur\n",
                  "0 - Retour\n")
            
            choix = choixMenu(0,5)
            print("\n")
            
            # Un choix valide a été fait dans le sous-menu
            if choix!='0':
                if choix=='1':
                    print("### Alimentation ###")
                    
                    if pasto.lecture_pompe_alimentation()==0:
                        print(" 0 - (OFF)\n",
                              "1 - ON\n")
                    elif pasto.lecture_pompe_alimentation()==1:
                        print(" 0 - OFF\n",
                              "1 - (ON)\n")
                    
                    choix = choixMenu(0,1) 
                    print("\n")
                    
                    if choix=='0':
                        pasto.modif_pompe_alimentation(0)
                        print("Alimentation OFF")
                    elif choix=='1':
                        pasto.modif_pompe_alimentation(1)
                        print("Alimentation ON")
                    input()
                elif choix=='2':
                    while choix!='0':
                        print("### Vitesse ###")
                        print("\nVitesse actuelle = {}\n".format(pasto.lecture_pompe_vitesse()))
                        print(" 1 - Modifier\n",
                              "0 - Retour\n")
                        
                        choix = choixMenu(0,1) 
                        print("\n")
                        
                        if choix!='0':
                            maj_vitesse = input("Entrez une nouvelle vitesse: ")
                            pasto.modif_pompe_vitesse(int(maj_vitesse))
                            choix = '-1'
                elif choix=='3':
                    print("### Sens de rotation ###")
                    
                    if pasto.lecture_pompe_direction()==0:
                        print(" 0 - (Aspiration)\n",
                              "1 - Refoulement\n")
                    elif pasto.lecture_pompe_direction()==1:
                        print(" 0 - Aspiration\n",
                              "1 - (Refoulement)\n")
                    
                    choix = choixMenu(0,1) 
                    print("\n")
                    
                    if choix=='0':
                        pasto.modif_pompe_direction(0)
                        print("Mode aspiration actif")
                    elif choix=='1':
                        pasto.modif_pompe_direction(1)
                        print("Mode refoulement actif")
                    input()
                elif choix=='4':
                    print("Signal servo = {}".format(pasto.lecture_pompe_signalServo()))
                    input()
                elif choix=='5':
                    print("Signal erreur = {}".format(pasto.lecture_pompe_signalErreur()))
                    input()
                
                choix= '-1'
        
        return 0

    # Sous-menu pour la gestion des cuves
    def ssMenuCuves():
        """Affiche un sous-menu pour gérer les cuves."""
        choix = '-1'
        
        while choix!='0':
            print("########## SOUS-MENU CUVES ##########")
            print(" 1 - Chauffe cuve 1\n",
                  "2 - Chauffe cuve 2\n",
                  "0 - Retour\n")
            
            choix = choixMenu(0,2)
            print("\n")
            
            # Un choix valide a été fait dans le sous-menu
            if choix!='0':
                if choix=='1':
                    print("### Chauffe cuve 1 ###")
                    
                    if pasto.lecture_cuve1_chauffe()==0:
                        print(" 0 - (OFF)\n",
                              "1 - ON\n")
                    elif pasto.lecture_cuve1_chauffe()==1:
                        print(" 0 - OFF\n",
                              "1 - (ON)\n")
                    
                    choix = choixMenu(0,1) 
                    print("\n")
                    
                    if choix=='0':
                        pasto.modif_cuve1_chauffe(0)
                        print("Chauffe cuve 1 OFF")
                    elif choix=='1':
                        pasto.modif_cuve1_chauffe(1)
                        print("Chauffe cuve 1 ON")
                    input()
                elif choix=='2':
                    print("### Chauffe cuve 2 ###")
                    
                    if pasto.lecture_cuve2_chauffe()==0:
                        print(" 0 - (OFF)\n",
                              "1 - ON\n")
                    elif pasto.lecture_cuve2_chauffe()==1:
                        print(" 0 - OFF\n",
                              "1 - (ON)\n")
                    
                    choix = choixMenu(0,1) 
                    print("\n")
                    
                    if choix=='0':
                        pasto.modif_cuve2_chauffe(0)
                        print("Chauffe cuve 2 OFF")
                    elif choix=='1':
                        pasto.modif_cuve2_chauffe(1)
                        print("Chauffe cuve 2 ON")
                    input()
            
                choix = '-1'
        
        return 0

    # Sous-menu pour la gestion des vannes/solénoïdes
    def ssMenuVannesSol():
        """Affiche un sous-menu pour gérer les vannes et solénoïdes."""
        choix = '-1'
        
        while choix!='0':
            print("########## SOUS-MENU VANNES/SOLENOÏDES ##########")
            print(" 1 - Gestion solénoïde haut chaude\n",
                  "2 - Gestion solénoïde haut froide\n",
                  "3 - Gestion vanne d'évactuation 1\n",
                  "4 - Gestion vanne d'évactuation 2\n",
                  "0 - Retour\n")
            
            choix = choixMenu(0,4)
            print("\n")
            
            # Un choix valide a été fait dans le sous-menu
            if choix!='0':
                if choix=='1':
                    print("### Solénoïde eau chaude ###")
                    
                    if pasto.lecture_sol_eauChaude()==0:
                        print(" 0 - (Fermé)\n",
                              "1 - Ouvert\n")
                    elif pasto.lecture_sol_eauChaude()==1:
                        print(" 0 - Fermé\n",
                              "1 - (Ouvert)\n")
                    
                    choix = choixMenu(0,1) 
                    print("\n")
                    
                    if choix=='0':
                        pasto.modif_sol_eauChaude(0)
                        print("Fermeture de l'arrivée d'eau chaude")
                    elif choix=='1':
                        pasto.modif_sol_eauChaude(1)
                        print("Ouverture de l'arrivée d'eau chaude")
                    input()
                elif choix=='2':
                    print("### Solénoïde eau froide ###")
                    
                    if pasto.lecture_sol_eauFroide()==0:
                        print(" 0 - (Fermé)\n",
                              "1 - Ouvert\n")
                    elif pasto.lecture_sol_eauFroide()==1:
                        print(" 0 - Fermé\n",
                              "1 - (Ouvert)\n")
                    
                    choix = choixMenu(0,1) 
                    print("\n")
                    
                    if choix=='0':
                        pasto.modif_sol_eauFroide(0)
                        print("Fermeture de l'arrivée d'eau froide")
                    elif choix=='1':
                        pasto.modif_sol_eauFroide(1)
                        print("Ouverture de l'arrivée d'eau froide")
                    input()
                elif choix=='3':
                    while choix!='0':
                        print("### Vanne d'évacuation 1 ###")
                        print(" 1 - Alimentation\n",
                                  "2 - Direction\n",
                                  "0 - Retour")
                        
                        choix = choixMenu(0,2) 
                        print("\n")
                        
                        if choix!='0':
                            if choix=='1':
                                print("### Alimentation vanne 1 ###")

                                if pasto.mlecture_vanne1_alimentation()==0:
                                    print(" 0 - (OFF)\n",
                                          "1 - ON")
                                elif pasto.mlecture_vanne1_alimentation()==1:
                                    print(" 0 - OFF\n",
                                          "1 - (ON)\n")
                                    
                                choix = choixMenu(0,1) 
                                print("\n")
                                
                                if choix=='0':
                                    pasto.momodif_vanne1_alimentation(0)
                                    print("Alimentation vanne 1 OFF")
                                elif choix=='1':
                                    pasto.momodif_vanne1_alimentation(1)
                                    print("Alimentation vanne 1 ON")
                                input()
                            
                            elif choix=='2':
                                print("### Direction vanne 1 ###")

                                if pasto.lecture_vanne1_direction()==0:
                                    print(" 0 - (Direction 1)\n",
                                          "1 - Direction 2")
                                elif pasto.lecture_vanne1_direction()==1:
                                    print(" 0 - Direction 1\n",
                                          "1 - (Direction 2)\n")
                                    
                                choix = choixMenu(0,1) 
                                print("\n")
                                
                                if choix=='0':
                                    pasto.modif_vanne1_direction(0)
                                    print("Vanne 1 en direction 1")
                                elif choix=='1':
                                    pasto.modif_vanne1_direction(1)
                                    print("Vanne 1 en direction 2")
                                input()
                            choix = '-1'
                elif choix=='4':
                    while choix!='0':
                        print("### Vanne d'évacuation 2 ###")
                        print(" 1 - Alimentation\n",
                                  "2 - Direction\n",
                                  "0 - Retour")
                        
                        choix = choixMenu(0,2) 
                        print("\n")
                        
                        if choix!='0':
                            if choix=='1':
                                print("### Alimentation vanne 2 ###")

                                if pasto.mlecture_vanne2_alimentation()==0:
                                    print(" 0 - (OFF)\n",
                                          "1 - ON")
                                elif pasto.mlecture_vanne2_alimentation()==1:
                                    print(" 0 - OFF\n",
                                          "1 - (ON)\n")
                                    
                                choix = choixMenu(0,1) 
                                print("\n")
                                
                                if choix=='0':
                                    pasto.momodif_vanne2_alimentation(0)
                                    print("Alimentation vanne 2 OFF")
                                elif choix=='1':
                                    pasto.momodif_vanne2_alimentation(1)
                                    print("Alimentation vanne 2 ON")
                                input()
                            
                            elif choix=='2':
                                print("### Direction vanne 2 ###")

                                if pasto.lecture_vanne2_direction()==0:
                                    print(" 0 - (Direction 1)\n",
                                          "1 - Direction 2")
                                elif pasto.lecture_vanne2_direction()==1:
                                    print(" 0 - Direction 1\n",
                                          "1 - (Direction 2)\n")
                                    
                                choix = choixMenu(0,1) 
                                print("\n")
                                
                                if choix=='0':
                                    pasto.modif_vanne2_direction(0)
                                    print("Vanne 2 en direction 1")
                                elif choix=='1':
                                    pasto.modif_vanne2_direction(1)
                                    print("Vanne 2 en direction 2")
                                input()
                            choix = '-1'
            
                choix = '-1'
                
        return 0

    def ssMenuEtatGeneral():
        """Affiche les informations sur l'état général."""
        
        print("########## Etat général ##########")
        print("Etat général = {}".format(pasto.lecture_etatGeneral()))
        print("Code erreurs = {}\n".format(pasto.lecture_codeErreurs()))
        
        input()

    def ssMenuRegistres():
        """Affiche les informations sur sur tous les registres."""
        print("########## Registres ##########")
        # Thermistances
        i = 1
        for valeur in pasto.lecture_thermi():
            print("Thermistance {} \t\t= {}".format(i,valeur))
            i+=1
        # Pompe
        print("Alimentation pompe \t= {}".format(pasto.lecture_pompe_alimentation()))
        print("Vitesse pompe \t\t= {}".format(pasto.lecture_pompe_vitesse()))
        print("Direction pompe \t= {}".format(pasto.lecture_pompe_direction()))
        print("Signal servo pompe \t= {}".format(pasto.lecture_pompe_signalServo()))
        print("Signal erreur pompe \t= {}".format(pasto.lecture_pompe_signalErreur()))
        print("Chauffe cuve 1 \t\t= {}".format(pasto.lecture_cuve1_chauffe()))
        print("Chauffe cuve 2 \t\t= {}".format(pasto.lecture_cuve2_chauffe()))
        print("Solénoïde eau chaude \t= {}".format(pasto.lecture_sol_eauChaude()))
        print("Solénoïde eau froide \t= {}".format(pasto.lecture_sol_eauFroide()))
        print("Alimentation vanne 1 \t= {}".format(pasto.lecture_vanne1_alimentation()))
        print("Direction vanne 1 \t= {}".format(pasto.lecture_vanne1_direction()))
        print("Alimentation vanne 2 \t= {}".format(pasto.lecture_vanne2_alimentation()))
        print("Direction vanne 2 \t= {}".format(pasto.lecture_vanne2_direction()))
        
        input()

    ################ programme principal ################

    choix = '-1'
    menuPrincipal = {'1':ssMenuThermis, '2':ssMenuPompe, '3':ssMenuCuves, '4':ssMenuVannesSol, '5':ssMenuEtatGeneral, '6':ssMenuRegistres}
    pasto = Micha()

    while choix!='0':
        
        print("################ MENU ################")
        print(" 1 - Thermistances\n",
              "2 - Pompe\n",
              "3 - Cuves\n",
              "4 - Vannes/solénoïdes\n",
              "5 - Etat général\n",
              "6 - Affichage de tous les registres\n",
              "0 - Quitter\n")
        
        choix = choixMenu(0,6)  
        print("\n")
        
        if choix!='0':
            choix = menuPrincipal[choix]()
        
            choix = '-1' 

    print("\nFermeture du programme...\n")

