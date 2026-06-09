// HC-SR04 ultrasonic distance sensor → Serial output
// Wiring: TRIG → pin 9, ECHO → pin 10
// Output format: "DIST:23.4\n" at ~10 Hz

#define TRIG_PIN 9   // ← 핀 바꾸려면 여기 수정
#define ECHO_PIN 10  // ← 핀 바꾸려면 여기 수정

#define MEASURE_INTERVAL_MS 100  // 10Hz
#define MAX_DISTANCE_CM 400.0f

void setup() {
    Serial.begin(115200);
    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);
    digitalWrite(TRIG_PIN, LOW);
}

void loop() {
    static unsigned long last_ms = 0;
    unsigned long now = millis();
    if (now - last_ms < MEASURE_INTERVAL_MS) return;
    last_ms = now;

    // 10μs 트리거 펄스
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);

    // ECHO 펄스 폭 측정 (타임아웃 30ms = ~510cm)
    long duration = pulseIn(ECHO_PIN, HIGH, 30000UL);

    float dist_cm = (duration == 0) ? -1.0f : duration * 0.01715f;

    if (dist_cm > MAX_DISTANCE_CM) dist_cm = -1.0f;

    Serial.print("DIST:");
    Serial.println(dist_cm, 1);
}
