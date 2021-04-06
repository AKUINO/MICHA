// PROJET MICHA
// Programme esclave modbus
// Type de carte: Arduino MKR Zero
// Gestion des éléments du pasteurisateur:
//    - pompe
//    - cuves de chauffe
//    - vannes
//    - solénoides
//    - capteurs de température (thermistances)
//
// ATTENTION: les sorties correspondent directement aux valeurs des registres liés: penser à vérifier la logique
//
// Notes de version:
//  - v1.0.1:
//          - Le registre THERMIS_ALIM_REG contrôle l'I/O gérant l'alimentation des thermistances
//  - v1.0.0:
//          - ajout d'un registre contenant la valeur d'incrémentation de la fréquence de la pompe (POMPE_V_INC_REG)
//  - v0.3.5:
//          - implémentation de l'augmentation progressive de vitesse de la pompe:
//            - suppression des délais utilisés pour la lecture des thermistances
//            - utilisation d'un clock hardware
//  - v0.3.4:
//          - utilisation de SAMD21turboPWM pour gérer la vitesse de la pompe
//          - modification des pins de la pompe: POMPE_V_PIN prend la pin 2 (PWM) et POMPE_A_PIN prend la pin 0
//  - v0.3.3:
//          - mise à jour et restructuration des registres (modification position et nom des registres des vannes)
//          - mise à jour des noms des pins gérant les vannes
//          - passage par une constante pour l'ID par défaut
//          - mise à jour des sorties uniquement si modification de la valeur des registres
//          - lecture du signal d'erreur PWM de la pompe
//          - lecture du signal servo de la pompe
//          - correction moyenne valeurs thermistance 4
//  - v0.3.2:
//          - mise à jour des registres: ajout de:
//            - VANNE_EVAC1_DIR_REG
//            - VANNE_EVAC2_DIR_REG
//            - ERREURS_REG
//            - POMPE_TAUX_PATINAGE_REG
//          - mise à jour de l'assignation des pins i/o: switche entre SOL_FROID_PIN et VANNE_EVAC1_PIN
//          - ajout de commentaires pour les registres
//  - v0.3.1:
//          - la valeur des thermistances dans le registre est maintenant la moyenne de 3 valeurs consécutives (et plus une seule)
//  - v0.3:
//          - prototype des fonctions enlevées car pas nécessaires
//          - ajout des fonctions de gestion des cuves et de gestion des vannes et solénoïdes
//  - v0.2.2:
//          - remplacement des vannes d'évacuation simple commande par des doubles commandes --> nécessitent 2 sorties 
//            chacune (ouverture/fermeture) mais les registres ne changent pas (1 registre/vanne)
//          - modification des noms associés aux vannes d'eau chaude et d'eau froide: remplacement de valve par solenoide
//          - modification des noms associés aux vannes d'évacuation: remplacement de valve par vanne
//          - déportation dans un fichier header de la configuration des adresses des registres et de l'assignation des pins
//          - utilisation des noms associés aux registres pour assigner les valeurs par défaut aux registres
//          - le registre associé à la vanne d'évacuation 2 n'avait pas de valeur par défaut --> corrigé
//  - v0.2.1: 
//          - ajout de la vanne d'évacuation 2
//  - v0.2:
//          - id stockée de manière permanente dnas la mémoire flash
//  - v0.1:
//          - nouvelle structure des registres modbus: tous les registres actuels sont implémentés
//  - v0.0:
//          - communication modbus implémentée
//          - structure des registres modbus basique: seule le registre d'une thermistance

#include <ArduinoRS485.h>           // ArduinoModbus.h dépend de ArduinoRS485
#include <ArduinoModbus.h>
#include <FlashStorage.h>
#include <SAMD21turboPWM.h>
#include "MICHA_configuration.h"    // configuration des registres et assignations des pins

// pour stockage en mémoire flash de l'ID
typedef struct StructID
{
  boolean init;                   // mise à 1 quand la structure a été initialiser une première fois
  int8_t id;                      // id de la carte sur le réseau modbus
  int16_t nbr_ecriture = 0;       // nombre de réécriture de l'id depuis le dernier upload du programme sur la carte
};

FlashStorage(ID_FLASH, StructID);


// gestion du moteur de la pompe
TurboPWM pwm;
unsigned long long int steps = 0;
int8_t vitesse_flag = 0;

// variables diverses
uint32_t tps_ref = 0;             // temps de référence (pour la fréquence de mesure des thermistances)
int8_t id_origine;                // id de la carte initiale
StructID id;                      // stocke id pour mémoire flash


void setup()
{
  Serial.begin(9600); // pour débug

  
  // stockage de l'ID de la carte dans la mémoire flash
  id = ID_FLASH.read();

  if (id.init == false)   // si l'ID n'a encore jamais été initialisé
  {
    id.id = ID; // utilisation de l'ID par défaut (1)
    id.nbr_ecriture = 0;  // initialisation du nombre d'écriture depuis le flashage
    id.init = true;

    ID_FLASH.write(id);   // écriture dans la mémoire flash

    Serial.print("Premier demarrage. L'ID de la carte est ");
    Serial.println(id.id);
    Serial.print("\n");
  }else
  {
    Serial.print("L'ID de la carte est ");
    Serial.println(id.id);
    Serial.print("\n");
  }
  
  // configuration générale
  analogReadResolution(12);       // passage en mode 12 bits (10 bits par défaut)
  analogReference(AR_EXTERNAL);   // utilisation d'une tension externe comme référence sur AREF
  
  // configuration entrées
  pinMode(THERMI1_PIN,INPUT);
  pinMode(THERMI2_PIN,INPUT);
  pinMode(THERMI3_PIN,INPUT);
  pinMode(THERMI4_PIN,INPUT);
  pinMode(POMPE_E_PIN,INPUT);
  pinMode(POMPE_S_PIN,INPUT);
  
  // configuration sorties
  pinMode(POMPE_V_PIN,OUTPUT);
  pinMode(POMPE_D_PIN,OUTPUT);
  pinMode(POMPE_A_PIN,OUTPUT);
  pinMode(CUVE1_PIN,OUTPUT);
  pinMode(CUVE2_PIN,OUTPUT);
  pinMode(SOL_CHAUD_PIN,OUTPUT);
  pinMode(SOL_FROID_PIN,OUTPUT);
  pinMode(VANNE_EVAC1_ALIM_PIN,OUTPUT);
  pinMode(VANNE_EVAC1_DIR_PIN,OUTPUT);
  pinMode(VANNE_EVAC2_ALIM_PIN,OUTPUT);
  pinMode(VANNE_EVAC2_DIR_PIN,OUTPUT);
  pinMode(THERMIS_ALIM_PIN,OUTPUT);

  // assignation valeur défaut sorties
  digitalWrite(POMPE_V_PIN,LOW);            // -----TEMPORAIRE-----
  digitalWrite(POMPE_D_PIN,LOW);            // mode pompage
  digitalWrite(POMPE_A_PIN,LOW);            // pompe non-alimentée
  digitalWrite(CUVE1_PIN,LOW);              // chauffe cuve 1 OFF
  digitalWrite(CUVE2_PIN,LOW);              // chauffe cuve 2 OFF
  digitalWrite(SOL_CHAUD_PIN,LOW);          // solénoide eau chaude mode fermé
  digitalWrite(SOL_FROID_PIN,LOW);          // solénoide eau froide mode fermé
  digitalWrite(VANNE_EVAC1_ALIM_PIN,LOW);      // vanne évacuation 1 fermée
  digitalWrite(VANNE_EVAC1_DIR_PIN,HIGH);     // vanne évacuation 1 fermée
  digitalWrite(VANNE_EVAC2_ALIM_PIN,LOW);      // vanne évacuation 2 fermée
  digitalWrite(VANNE_EVAC2_DIR_PIN,HIGH);     // vanne évacuation 2 fermée
  digitalWrite(THERMIS_ALIM_PIN,HIGH);      // alimentation thermistances OFF


  // lancement du serveur ModbusRTU
  if (!ModbusRTUServer.begin(id.id, 9600))      // paramètres: ID=1, baudrate=9600, config=SERIAL_8N1 (8 bits, sans parité, 1 bit stop)
  {
    Serial.println("Echec du lancement du serveur Modbus RTU");
    while (1);
  }


  // configuration des registres modbus
  ModbusRTUServer.configureCoils(0x00, 64);
  ModbusRTUServer.configureInputRegisters(0x00, 48);
  ModbusRTUServer.configureHoldingRegisters(0x00, 32);

  // assignation de la valeur par défaut des registres
  ModbusRTUServer.coilWrite(THERMIS_ALIM_REG,0);              // thermistances - alimentation: OFF
  ModbusRTUServer.coilWrite(POMPE_D_REG,0);                   // pompe - direction: aspiration
  ModbusRTUServer.coilWrite(POMPE_A_REG,0);                   // pompe - alimentation: OFF
  ModbusRTUServer.coilWrite(CUVE1_REG,0);                     // cuve 1 - chauffe: OFF
  ModbusRTUServer.coilWrite(CUVE2_REG,0);                     // cuve 2 - chauffe: OFF
  ModbusRTUServer.coilWrite(SOL_CHAUD_REG,0);                 // solénoide eau chaude: fermé
  ModbusRTUServer.coilWrite(SOL_FROID_REG,0);                 // solénoide eau froide: fermé
  ModbusRTUServer.coilWrite(VANNE_EVAC1_ALIM_REG,0);          // vanne évacuation 1 - alimentation: OFF
  ModbusRTUServer.coilWrite(VANNE_EVAC1_DIR_REG,0);           // vanne évacuation 1 - direction
  ModbusRTUServer.coilWrite(VANNE_EVAC2_ALIM_REG,0);          // vanne évacuation 2 - alimentation: OFF
  ModbusRTUServer.coilWrite(VANNE_EVAC2_DIR_REG,0);           // vanne évacuation 2 - direction
  ModbusRTUServer.inputRegisterWrite(ETAT_GEN_REG,0);         // état général: 0 (aucun problème)
  ModbusRTUServer.inputRegisterWrite(ERREURS_REG,0);          // code erreurs: 0
  ModbusRTUServer.holdingRegisterWrite(ID_REG,id.id);         // ID du device
  ModbusRTUServer.holdingRegisterWrite(POMPE_V_REG,0);        // pompe - vitesse
  ModbusRTUServer.holdingRegisterWrite(POMPE_V_INC_REG,2000); // pompe - vitesse

  // configuration pour gestion de la vitesse de la pompe
  pwm.setClockDivider(1,false);
  pwm.enable(1,true);
}

void loop() {
  //variable pour la gestion de la fréquence de mesure des thermistances
  uint32_t tps = millis();
  uint32_t intervalle = tps - tps_ref;
  
  //scrute commande du maitre
  ModbusRTUServer.poll();
    
  // lecture des entrées (thermistances et signaux de la pompe) chaque seconde et mise à jour des registres correspondant
  if(intervalle>1000)  //si 1 sec écoulée
  {
    // débug: indique l'ID actuel du device (si modifié lors de la session actuelle, indique l'ID modifié)
    Serial.print("\n");
    Serial.print("Slave ID = ");
    Serial.println(id.id);
    
    lecture_thermi();
    lecture_signauxPompe();
    tps_ref = tps;
  }

  gestion_id();
  gestion_pompe();
  gestion_cuves();
  gestion_vannes();

  delay(50);
}

// met à jour les sorties dédiées aux vannes et aux solénoides en fonction des valeurs des registres (uniquement si modifiées)
void gestion_vannes()
{
  int8_t sol_chaud = ModbusRTUServer.coilRead(SOL_CHAUD_REG);
  int8_t sol_froid = ModbusRTUServer.coilRead(SOL_FROID_REG);
  int8_t vanne_evac1_alim = ModbusRTUServer.coilRead(VANNE_EVAC1_ALIM_REG);
  int8_t vanne_evac1_dir = ModbusRTUServer.coilRead(VANNE_EVAC1_DIR_REG);
  int8_t vanne_evac2_alim = ModbusRTUServer.coilRead(VANNE_EVAC2_ALIM_REG);
  int8_t vanne_evac2_dir = ModbusRTUServer.coilRead(VANNE_EVAC2_DIR_REG);
  
  if(digitalRead(SOL_CHAUD_PIN)!=sol_chaud)
  {
    Serial.println("Etat solénoïde 1 modifié");
    digitalWrite(SOL_CHAUD_PIN,sol_chaud);
  }

  if(digitalRead(SOL_FROID_PIN)!=sol_froid)
  {
    Serial.println("Etat solénoïde 2 modifié");
    digitalWrite(SOL_FROID_PIN,sol_froid);
  }

  if(digitalRead(VANNE_EVAC1_ALIM_PIN)!=vanne_evac1_alim)
  {
    Serial.println("Alimentation vanne 1 modifiée");
    digitalWrite(VANNE_EVAC1_ALIM_PIN,vanne_evac1_alim);
  }

  if(digitalRead(VANNE_EVAC1_DIR_PIN)!=vanne_evac1_dir)
  {
    Serial.println("Direction vanne 1 modifiée");
    digitalWrite(VANNE_EVAC1_DIR_PIN,vanne_evac1_dir);
  }

  if(digitalRead(VANNE_EVAC2_ALIM_PIN)!=vanne_evac2_alim)
  {
    Serial.println("Alimentation vanne 2 modifiée");
    digitalWrite(VANNE_EVAC2_ALIM_PIN,vanne_evac2_alim);
  }

  if(digitalRead(VANNE_EVAC2_DIR_PIN)!=vanne_evac2_dir)
  {
    Serial.println("Direction vanne 2 modifiée");
    digitalWrite(VANNE_EVAC2_DIR_PIN,vanne_evac2_dir);
  }
}

// met à jour les sorties dédiées aux cuves en fonction des valeurs des registres (uniquement si modifiées)
void gestion_cuves()
{
  int8_t cuve1 = ModbusRTUServer.coilRead(CUVE1_REG);
  int8_t cuve2 = ModbusRTUServer.coilRead(CUVE2_REG);
  
  if(digitalRead(CUVE1_PIN)!=cuve1)
  {
    Serial.println("Etat chauffe cuve 1 modifié");
    digitalWrite(CUVE1_PIN,cuve1);  // cuve 1
  }

  if(digitalRead(CUVE2_PIN)!=cuve2)
  {
    Serial.println("Etat chauffe cuve 2 modifié");
    digitalWrite(CUVE2_PIN,cuve2);  // cuve 2
  }
}

// lit et stocke les valeurs des thermistances dans les registres (moyenne sur 3 valeurs successives)
void lecture_thermi()
{
  int16_t thermis[4] = {0,0,0,0};             // stocke les valeurs des thermistances pour moyenne

  ModbusRTUServer.coilWrite(THERMIS_ALIM_REG,1);  // alimentation des thermistances ON
  digitalWrite(THERMIS_ALIM_PIN,!ModbusRTUServer.coilRead(THERMIS_ALIM_REG));
  
  for(int8_t i=0;i<3;i++) // prise des 3 échantillons
  {
    delay(10);
    
    thermis[0] = thermis[0] + analogRead(THERMI1_PIN);
    thermis[1] = thermis[1] + analogRead(THERMI2_PIN);
    thermis[2] = thermis[2] + analogRead(THERMI3_PIN);
    thermis[3] = thermis[3] + analogRead(THERMI4_PIN);
  }

  ModbusRTUServer.coilWrite(THERMIS_ALIM_REG,0);  // alimentation des thermistances OFF
  digitalWrite(THERMIS_ALIM_PIN,!ModbusRTUServer.coilRead(THERMIS_ALIM_REG));

  for(int8_t i=0;i<4;i++) // moyenne
  {
    thermis[i] = thermis[i]/3;
  }

  // stockage dans les registres
  ModbusRTUServer.inputRegisterWrite(THERMI1_REG,thermis[0]);
  ModbusRTUServer.inputRegisterWrite(THERMI2_REG,thermis[1]);
  ModbusRTUServer.inputRegisterWrite(THERMI3_REG,thermis[2]);
  ModbusRTUServer.inputRegisterWrite(THERMI4_REG,thermis[3]);

  // pour debug
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

// met à jour les sorties dédiées à la pompe en fonction des valeurs des registres (uniquement si modifiées)
void gestion_pompe()
{
  uint16_t pompe_v = ModbusRTUServer.holdingRegisterRead(POMPE_V_REG);
  static uint16_t pompe_v_prec = 0;
  static int frequence_actuelle = 0;
  int16_t frequence_inc = ModbusRTUServer.holdingRegisterRead(POMPE_V_INC_REG);;
  int8_t pompe_d = ModbusRTUServer.coilRead(POMPE_D_REG);
  int8_t pomp_a = ModbusRTUServer.coilRead(POMPE_A_REG);
  
  if(pompe_v_prec!=pompe_v)
  {
    Serial.println("Vitesse pompe modifiée");

    steps = calculSteps(pompe_v);
    vitesse_flag = 1;
    
    pompe_v_prec = pompe_v;
  }

  if(vitesse_flag)
  { 
    frequence_actuelle = pwm.frequency(1);
    
    if(frequence_actuelle < pompe_v)  //si la vitesse doit augmenter
    {
      frequence_actuelle += frequence_inc;
      steps = calculSteps(frequence_actuelle);

      if(frequence_actuelle>=pompe_v)
      {
        steps = calculSteps(pompe_v);
        pwm.timer(1,1,steps,false);
        pwm.analogWrite(POMPE_V_PIN,500); //duty-cycle = 50%
      
        vitesse_flag = 0;
      }else
      {
        pwm.timer(1,1,steps,false);
        pwm.analogWrite(POMPE_V_PIN,500); //duty-cycle = 50%
      }
    }
    else if(frequence_actuelle > pompe_v) // si la vitesse doit diminuer
    {
      frequence_actuelle -= frequence_inc;
      steps = calculSteps(frequence_actuelle);

      if(frequence_actuelle <= 100) // on force à 0 Hz
      {
        pwm.timer(1,1,0xFFFFFF,false);
        pwm.analogWrite(POMPE_V_PIN,0);
        vitesse_flag = 0;
      }else
      {
        if(frequence_actuelle <= pompe_v) // la fréquence ne doit pas passer sous la fréquence consigne
        {
          steps = calculSteps(pompe_v);
          pwm.timer(1,1,steps,false);
          pwm.analogWrite(POMPE_V_PIN,500); //duty-cycle = 50%

          vitesse_flag = 0;
        }else
        {
          pwm.timer(1,1,steps,false);
          pwm.analogWrite(POMPE_V_PIN,500); //duty-cycle = 50%
        }
      }
    }
  }

  if(digitalRead(POMPE_D_PIN)!=pompe_d)
  {
    Serial.println("Direction pompe modifiée");
    digitalWrite(POMPE_D_PIN,pompe_d);  // direction
  }

  if(digitalRead(POMPE_A_PIN)!=pomp_a)
  {
    Serial.println("Alimentation pompe modifiée");
    digitalWrite(POMPE_A_PIN,pomp_a);  // alimentation
  }
}



void lecture_signauxPompe()
{
  uint32_t pompe_e_us = pulseIn(POMPE_E_PIN,HIGH,16); // retourne la durée de l'impulsion HAUTE PWM en µs (attend l'impulsion pendant 16 µs)
  
  ModbusRTUServer.inputRegisterWrite(POMPE_S_REG,digitalRead(POMPE_S_PIN));
  ModbusRTUServer.inputRegisterWrite(POMPE_E_REG,pompe_e_us);
}

// met à jour l'id du device dans la mémoire flash uniquement lorsqu'il y a une modification de sa valeur dans le registre
void gestion_id()
{
  if(id.id != ModbusRTUServer.holdingRegisterRead(ID_REG))
  {
    id.id = ModbusRTUServer.holdingRegisterRead(ID_REG);
    id.nbr_ecriture++;
    ID_FLASH.write(id);
    
    Serial.print("\n");
    Serial.println("Ecriture ID dans flash");
    Serial.print("Nouvel ID = ");
    Serial.print(id.id);
    Serial.print(" ; Nombre d'ecriture = ");
    Serial.println(id.nbr_ecriture);
    Serial.print("\n");
  }
}

// Retourne la valeur des steps en fonction de la fréquence voulue
unsigned long long int calculSteps(int f)
{
  if(f<=0)
    return 0;
    
  return VARIANT_MCK/(2*f); //VARIANT_MCK = 48MHz
}
