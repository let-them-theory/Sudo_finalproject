// HC-SR04 → 시리얼 한 줄 출력
// TRIG pin 6, ECHO pin 7, 9600 baud
// 출력 예: Distance:123.4mm

const int trigPin = 6;
const int echoPin = 7;

void setup() {
  Serial.begin(9600);
  pinMode(echoPin, INPUT);
  pinMode(trigPin, OUTPUT);
}

void loop() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  unsigned long duration = pulseIn(echoPin, HIGH, 30000UL);
  float distanceMm = -1.0f;
  if (duration > 0) {
    distanceMm = (340.0f * (float)duration) / 2000.0f;
  }

  Serial.print("Distance:");
  Serial.print(distanceMm, 1);
  Serial.println("mm");

  delay(200);
}
