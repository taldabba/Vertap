int powerPin = 11;
int delayTime = 750;

void setup() {
    pinMode(powerPin, OUTPUT);
}
void loop() {
    digitalWrite(powerPin, HIGH);
    delay(delayTime);
    digitalWrite(powerPin, LOW);
    delay(delayTime);
} 