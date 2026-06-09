// HC-SR04 초음파 거리 센서 → 시리얼 출력 (현장 배선)
// TRIG → pin 6, ECHO → pin 7
// 출력: Duration:... / DIstance:88mm  (9600 baud)
// ultrasonic_node.py 가 DIstance/Distance 줄을 파싱함

int trigPin = 6;
int echoPin = 7;

void setup() {
  Serial.begin(9600);
  pinMode(echoPin, INPUT);
  pinMode(trigPin, OUTPUT);
  digitalWrite(trigPin, LOW);
}

void loop() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH, 30000UL);
  long distance = (duration == 0) ? -1 : ((340L * duration) / 1000) / 2;

  Serial.print("Duration:");
  Serial.print(duration);
  Serial.print("\nDIstance:");
  Serial.print(distance);
  Serial.println("mm");

  delay(500);
}
