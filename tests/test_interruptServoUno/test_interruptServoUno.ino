// code pour tester interrupt servo à différentes fréquences
// pour Arduino Uno

#define pin A5
#define pin4 A4

void setup() {
  pinMode(pin,OUTPUT);
  pinMode(pin4,OUTPUT);

}

void loop() {
  digitalWrite(pin,HIGH);
  //digitalWrite(pin4,HIGH);
  delayMicroseconds(10000);
  digitalWrite(pin,LOW);
  //digitalWrite(pin4,LOW);
  delayMicroseconds(10000);

}
