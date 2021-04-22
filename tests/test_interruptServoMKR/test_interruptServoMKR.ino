#define interruptPin A1

volatile uint32_t pump_servoInterval = 0;  // to store the period of the pump servo signal
uint32_t time_ref1 = 0;           // thermistor reading reference time
volatile uint32_t time_ref5 = 0;  // reference time for pump servo signal
volatile boolean pump_working_flag = false;    // indicates if the pump is working
volatile uint16_t pump_servo_pulseCounter = 0;  // to store the pulse count of the pump servo signal
volatile uint16_t pump_servo_periodMax = 0; // to store the period max of the pump servo signal
volatile uint16_t pump_servo_periodMin = 65000; // to store the period min of the pump servo signal
volatile uint16_t pump_servo_periodStdDev = 0; // to store the period standard deviation of the pump servo signal
volatile uint32_t pump_servo_periodTotal = 0;  // to store the total of period of the pump servo signal


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
    Serial.println(pump_servo_periodTotal/pump_servo_pulseCounter);
    pump_working_flag = false;
  }
}

void int_ISR()
{
  uint32_t now = micros();  // updates the current time variable

  if( ! pump_working_flag )  // if the pump has just started, initialization of the timer and the counter
  {
    time_ref5 = now;
    pump_servo_pulseCounter = 0;
    pump_servo_periodMin = 65000;
    pump_servo_periodMax = 0;
    pump_servo_periodTotal = 0;
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

    pump_servo_periodTotal += pump_servoInterval;
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
