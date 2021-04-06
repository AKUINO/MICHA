// PROJET MICHA
// Configuration des registres et des entrées sortie
// Pour carte Arduino MKR Zero


// constantes
#define ID                      1     // ID Modbus par défaut de l'Arduino

// configuration adresse relative registres
// coils
#define THERMIS_ALIM_REG        0x01   // excitation des thermistances
#define POMPE_D_REG             0x10   // direction de la pompe
#define POMPE_A_REG             0x11   // alimentation de la pompe
#define CUVE1_REG               0x20   // gestion de la chauffe cuve 1
#define CUVE2_REG               0x21   // gestion de la chauffe cuve 2
#define SOL_CHAUD_REG           0x30   // solénoïde eau chaude
#define SOL_FROID_REG           0x31   // solénoïde eau froide
#define VANNE_EVAC1_ALIM_REG    0x32   // alimentation de la vanne d'évacuation 1
#define VANNE_EVAC1_DIR_REG     0x33   // direction de la vanne d'évacuation 1
#define VANNE_EVAC2_ALIM_REG    0x34   // alimentation de la vanne d'évacuation 2
#define VANNE_EVAC2_DIR_REG     0x35   // direction de la vanne d'évacuation 2
// input registers
#define ETAT_GEN_REG            0x00   // état général du système
#define THERMI1_REG             0x01   // valeur de la thermistance 1
#define THERMI2_REG             0x02   // valeur de la thermistance 2
#define THERMI3_REG             0x03   // valeur de la thermistance 3
#define THERMI4_REG             0x04   // valeur de la thermistance 4
#define POMPE_E_REG             0x10   // erreur renvoyée par la pompe
#define POMPE_S_REG             0x11   // signal servo de la pompe
#define ERREURS_REG             0x20   // erreurs dans le système
// holding registers
#define ID_REG                  0x00   // ID Modbus de l'Arduino
#define POMPE_V_REG             0x10   // vitesse de la pompe
#define POMPE_V_INC_REG         0x11   // valeur d'incrémentation/décrémentation de la fréquence
#define POMPE_TAUX_PATINAGE_REG 0x12   // taux d'erreur patinage




// assignation des pins au hardware
#define THERMI1_PIN           A0         // pin thermistance 1
#define THERMI2_PIN           A1         // pin thermistance 2
#define THERMI3_PIN           A2         // pin thermistance 3
#define THERMI4_PIN           A3         // pin thermistance 4
#define THERMIS_ALIM_PIN      21         // pin commande alimentation thermistances
#define POMPE_E_PIN           19         // pin retour erreur pompe
#define POMPE_S_PIN           20         // pin retour servo pompe
#define POMPE_A_PIN           0          // pin commande vitesse pompe (PWM)
#define POMPE_D_PIN           1          // pin commande direction pompe
#define POMPE_V_PIN           2          // pin commande alimentation pompe
#define CUVE1_PIN             3          // pin commande chauffe cuve 1
#define CUVE2_PIN             4          // pin commande chauffe cuve 2
#define SOL_CHAUD_PIN         5          // pin commande solénoide eau chaude
#define SOL_FROID_PIN         8          // pin commande solénoide eau froide
#define VANNE_EVAC1_ALIM_PIN  6          // pin commande alimentation vanne d'évacuation 1
#define VANNE_EVAC1_DIR_PIN   7          // pin commande direction vanne d'évacuation 1
#define VANNE_EVAC2_ALIM_PIN  9          // pin commande alimentation vanne d'évacuation 2
#define VANNE_EVAC2_DIR_PIN   10         // pin commande direction vanne d'évacuation 2
