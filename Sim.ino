typedef struct {
  int id;
  int x;
  int y;
  int z;
  int temp;
  int o2;
  int bpm;
} myPacket;

myPacket packet = { 1, 50, 40, 30, 204, 76, 55};
int pack[7] = { 0, 50, 0, 3, 204, 76, 55 };
myPacket *a_packet = &packet;



void setup() {
  // put your setup code here, to run once:
  pinMode(13, OUTPUT);
  Serial.begin(115200);

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
    if (val == 'R') {
      for (int i = 0; i < 7; i++) {
        Serial.print(pack[i]);
        Serial.print('\t');
      }
      Serial.print('\n');
      pack[1] += 100;
      pack[2] += 50;
    }
  }

}
