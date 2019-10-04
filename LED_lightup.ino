void setup() {
  // put your setup code here, to run once:
  pinMode(13, OUTPUT);
  Serial.begin(9600);

}

void loop() {
  // put your main code here, to run repeatedly:
  if (Serial.available()){
    char val = Serial.read();
    if (val == 'D') {
      digitalWrite(13, HIGH);
    }
    if (val == 'O') {
      digitalWrite(13, LOW);
    }
  }

}
