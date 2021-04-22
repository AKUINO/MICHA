#define interruptPin A1

volatile uint64_t pump_servoInterval = 0;  // to store the period of the pump servo signal
uint32_t time_ref1 = 0;           // thermistor reading reference time
volatile uint64_t time_ref5 = 0;  // reference time for pump servo signal
volatile boolean pump_working_flag = false;    // indicates if the pump is working
volatile uint16_t pump_servo_pulseCounter = 0;  // to store the pulse count of the pump servo signal
volatile uint16_t pump_servo_periodMax = 0; // to store the period max of the pump servo signal
volatile uint16_t pump_servo_periodMin = 65000; // to store the period min of the pump servo signal
volatile uint16_t pump_servo_periodAvg = 0; // to store the period average of the pump servo signal
volatile uint16_t pump_servo_periodStdDev = 0; // to store the period standard deviation of the pump servo signal


void setup() {
  delay(1000);
  Serial.begin(9600);
  pinMode(interruptPin,INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(interruptPin),int_ISR,FALLING);
}

void loop() {
  uint32_t tps = millis();
  uint32_t interval1 = tps - time_ref1;

  if(interval1>1000) // 1 s passed
  {
    time_ref1 = tps;
    //delayMicroseconds(1000000);
    Serial.print(pump_servo_pulseCounter);
    Serial.print("\t");
    Serial.print(pump_servoInterval);
    Serial.print("\t");
    Serial.print(pump_servo_periodMin);
    Serial.print("\t");
    Serial.print(pump_servo_periodMax);
    Serial.print("\t");
    Serial.println(pump_servo_periodAvg);
    pump_servo_pulseCounter = false;
  }
}

void int_ISR()
{
  uint64_t now = superMicros();  // updates the current time variable

  if(pump_working_flag==false)  // if the pump has just started, initialization of the timer and the counter
  {
    time_ref5 = now;
    pump_servo_pulseCounter = 1;
    pump_servo_periodMin = 65000;
    pump_servo_periodMax = 0;
    pump_servo_periodAvg = 0;
    pump_working_flag = true;
  }else
  {
    pump_servoInterval = now - time_ref5;
    time_ref5 = now;
    pump_servo_pulseCounter++;

    if(pump_servoInterval > pump_servo_periodMax)
    {
      pump_servo_periodMax = pump_servoInterval;
    }else if(pump_servoInterval < pump_servo_periodMin)
    {
      pump_servo_periodMin = pump_servoInterval;
    }

    pump_servo_periodAvg += pump_servoInterval/pump_servo_pulseCounter;
  }
}

uint16_t average()
{
  return 0;
}

uint16_t standardDeviation()
{
  return 0;
}

// Returns the time in microseconds since the Arduino starting (on 64 bits)
uint64_t superMicros()
{
  static uint32_t nbRollover = 0;
  static uint32_t previousMicros = 0;
  uint32_t currentMicros = micros();
  
  if (currentMicros < previousMicros) {
     nbRollover++;
  }
  previousMicros = currentMicros;

  uint64_t finalMicros = nbRollover;
  finalMicros <<= 32;
  finalMicros +=  currentMicros;
  return finalMicros;
}
