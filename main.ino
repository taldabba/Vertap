int ledWire1 = 12;
int ledWire2 = 11;
int ledWire3 = 10;
int ledWire4 = 9;
int ledWire5 = 8;
int ledWire6 = 7;
int ledArray[6] = {ledWire1, ledWire2, ledWire3, ledWire4, ledWire5, ledWire6};
int delayTime = 50;
int led = 0;

void setup() {
    for (int i=0; i<5; i++) {
        pinMode(ledArray[i], OUTPUT);
    }
}
void loop() {
    for (int i=0; i<5; i++) {
        digitalWrite(ledArray[i], HIGH);
        delay(delayTime);
        digitalWrite(ledArray[i], LOW);
        led=i;
    }
    led++;
    digitalWrite(ledArray[led], HIGH);
    delay(delayTime);
    digitalWrite(ledArray[led], LOW);
    led--;
    while (led>0) {
        digitalWrite(ledArray[led], HIGH);
        delay(delayTime);
        digitalWrite(ledArray[led], LOW);
        led--;
    }
} 