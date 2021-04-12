// PROJET MICHA
// Configuration des registres et des entrées sortie
// Pour carte Arduino MKR Zero


// constantes
#define ID                      1     // ID Modbus par défaut de l'Arduino

// configuration adresse relative registres
// coils
#define BOOT_FLAG_REG           0x00   // register which stores the boot state
#define THERMIS_POW_REG         0x01   // register which stores the thermistor power state
#define PUMP_DIR_REG            0x10   // register which stores the pump direction
#define PUMP_POW_REG            0x11   // register which stores the pump power state
#define TANK1_REG               0x20   // register which stores the tank 1 state
#define TANK2_REG               0x21   // register which stores the tank 2 state
#define SOL_HOT_REG             0x30   // register which stores the hot water solenoid state
#define SOL_COLD_REG            0x31   // register which stores the cold water solenoid state
#define VALVE1_POW_REG          0x32   // register which stores the valve 1 power state
#define VALVE1_DIR_REG          0x33   // register which stores the valve 1 direction
#define VALVE2_POW_REG          0x34   // register which stores the valve 2 power state
#define VALVE2_DIR_REG          0x35   // register which stores the valve 2 direction
// input registers
#define GEN_STATE_REG           0x00   // register which stores the general state of the system
#define THERMI1_REG             0x01   // register which stores the thermistor 1 value (0 - 4095)
#define THERMI2_REG             0x02   // register which stores the thermistor 2 value (0 - 4095)
#define THERMI3_REG             0x03   // register which stores the thermistor 3 value (0 - 4095)
#define THERMI4_REG             0x04   // register which stores the thermistor 4 value (0 - 4095)
#define PUMP_ERR_REG            0x10   // register which stores the error code returned by the pump regulator
#define PUMP_SERVO_REG          0x11   // register which stores the speed returned by the pump servo
#define ERROR_CODE_REG          0x20   // register which stores the general error codes
// holding registers
#define ID_REG                  0x00   // register which stores the modbus ID
#define PUMP_SPEED_REG          0x10   // register which stores the pump speed
#define PUMP_SPEED_INC_REG      0x11   // register which stores the increasing/decreasing value of the pump frequency
#define PUMP_SPIN_RATE_REG      0x12   // register which stores the pump spining rate approved


// assignation des pins au hardware
#define THERMI1_PIN           A0         // pin thermistance 1
#define THERMI2_PIN           A1         // pin thermistance 2
#define THERMI3_PIN           A2         // pin thermistance 3
#define THERMI4_PIN           A3         // pin thermistance 4
#define THERMIS_POW_PIN       21         // pin commande alimentation thermistances
#define PUMP_ERR_PIN          19         // pin retour erreur pompe
#define PUMP_SPEED_PIN        20         // pin retour servo pompe
#define PUMP_POW_PIN          0          // pin commande alimentation pompe (PWM)
#define PUMP_DIR_PIN          1          // pin commande direction pompe
#define PUMP_SPEED_PIN        2          // pin commande vitesse pompe
#define TANK1_PIN             3          // pin commande chauffe cuve 1
#define TANK2_PIN             4          // pin commande chauffe cuve 2
#define SOL_HOT_PIN           5          // pin commande solénoide eau chaude
#define SOL_COLD_PIN          8          // pin commande solénoide eau froide
#define VALVE1_POW_PIN        6          // pin commande alimentation vanne d'évacuation 1
#define VALVE1_DIR_PIN        7          // pin commande direction vanne d'évacuation 1
#define VALVE2_POW_PIN        9          // pin commande alimentation vanne d'évacuation 2
#define VALVE2_DIR_PIN        10         // pin commande direction vanne d'évacuation 2
